import importlib.util
import math
import sys
import threading
import time
import types
from pathlib import Path


class FakeMessage(dict):
    pass


class FakeRos:
    def __init__(self, host="127.0.0.1", port=9090):
        self.host = host
        self.port = port
        self.is_connected = True
        self.callbacks = {}
        self.current_status_list = []
        self.goal_counter = 0
        self.emit_acceptance = True
        self.emit_terminal = True
        self.terminal_status = 3
        self.terminal_text = "Goal reached."
        self.emit_motion = True
        self.motion_linear_x = 0.1
        self.motion_angular_z = 0.2
        self.amcl_displacement = 0.08
        self.odom_displacement = 0.08
        self.published = []
        self.advertise_counts = {}

    def run(self):
        self.is_connected = True

    def terminate(self):
        self.is_connected = False

    def emit(self, name, message):
        callback = self.callbacks.get(name)
        if callback:
            callback(message)


class FakeTopic:
    def __init__(
        self,
        ros,
        name,
        message_type,
        compression=None,
        latch=False,
        throttle_rate=0,
        queue_size=100,
        queue_length=0,
        reconnect_on_close=True,
    ):
        self.ros = ros
        self.name = name
        self.message_type = message_type
        self.is_advertised = False
        self.is_subscribed = False

    def advertise(self):
        self.is_advertised = True
        self.ros.advertise_counts[self.name] = (
            self.ros.advertise_counts.get(self.name, 0) + 1
        )

    def unadvertise(self):
        self.is_advertised = False

    def subscribe(self, callback):
        self.is_subscribed = True
        self.ros.callbacks[self.name] = callback
        if self.name == "/move_base/status":
            callback({"status_list": list(self.ros.current_status_list)})
        elif self.name == "/amcl_pose":
            callback(amcl_message(0.0, 0.0))
        elif self.name == "/odom":
            callback(odom_message(0.0, 0.0))

    def unsubscribe(self):
        self.is_subscribed = False
        self.ros.callbacks.pop(self.name, None)

    def publish(self, message):
        self.ros.published.append((self.name, dict(message)))
        if self.name == "/move_base/cancel":
            goal_id = message.get("id")
            final = status_entry(goal_id, 2, "cancelled")
            self.ros.current_status_list = [final]
            self.ros.emit(
                "/move_base/status",
                {"status_list": list(self.ros.current_status_list)},
            )
            return
        if self.name != "/move_base_simple/goal":
            return

        self.ros.goal_counter += 1
        goal_id = "fake-goal-{}".format(self.ros.goal_counter)
        if not self.ros.emit_acceptance:
            return

        active = status_entry(goal_id, 1, "accepted")
        self.ros.current_status_list = [active]
        self.ros.emit(
            "/move_base/status",
            {"status_list": list(self.ros.current_status_list)},
        )
        if self.ros.emit_motion:
            self.ros.emit(
                "/cmd_vel",
                {
                    "linear": {
                        "x": self.ros.motion_linear_x,
                        "y": 0.0,
                        "z": 0.0,
                    },
                    "angular": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": self.ros.motion_angular_z,
                    },
                },
            )
            self.ros.emit(
                "/amcl_pose",
                amcl_message(self.ros.amcl_displacement, 0.0),
            )
            self.ros.emit(
                "/odom",
                odom_message(self.ros.odom_displacement, 0.0),
            )
        if self.ros.emit_terminal:
            final = status_entry(
                goal_id,
                self.ros.terminal_status,
                self.ros.terminal_text,
            )
            self.ros.current_status_list = [final]
            self.ros.emit(
                "/move_base/status",
                {"status_list": list(self.ros.current_status_list)},
            )


def status_entry(goal_id, status, text):
    return {
        "goal_id": {"id": goal_id},
        "status": status,
        "text": text,
    }


def amcl_message(x, y):
    return {
        "pose": {
            "pose": {
                "position": {"x": x, "y": y, "z": 0.0},
            }
        }
    }


def odom_message(x, y):
    return {
        "pose": {
            "pose": {
                "position": {"x": x, "y": y, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            }
        }
    }


fake_roslibpy = types.ModuleType("roslibpy")
fake_roslibpy.Ros = FakeRos
fake_roslibpy.Topic = FakeTopic
fake_roslibpy.Message = FakeMessage
sys.modules["roslibpy"] = fake_roslibpy

module_path = Path(__file__).resolve().with_name("x3_bridge.py")
spec = importlib.util.spec_from_file_location("x3_bridge_under_test", module_path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def send(bridge, **kwargs):
    return bridge.send_navigation_goal_pose(
        "map",
        0.10,
        0.0,
        0.0,
        acceptance_timeout=kwargs.get("acceptance_timeout", 0.02),
        result_timeout=kwargs.get("result_timeout", 0.02),
        monitor_ready_timeout=0.02,
        max_odom_path_m=kwargs.get("max_odom_path_m", 0.10),
        max_amcl_displacement_m=kwargs.get(
            "max_amcl_displacement_m", 0.10
        ),
        cancel_timeout=0.02,
    )


def test_success_and_persistent_publisher():
    bridge = module.X3Bridge()
    first = send(bridge)
    second = send(bridge)
    assert first["move_base_acceptance_known"] is True
    assert first["terminal_status_known"] is True
    assert first["move_base_status"] == 3
    assert first["arrived"] is True
    assert first["publisher_created"] is True
    assert second["publisher_reused"] is True
    assert first["nonzero_cmd_vel_messages"] == 1
    assert math.isclose(first["amcl_displacement_m"], 0.08)
    assert bridge.ros.advertise_counts["/move_base_simple/goal"] == 1
    assert [name for name, _ in bridge.ros.published] == [
        "/move_base_simple/goal",
        "/move_base_simple/goal",
    ]


def test_terminal_failure():
    bridge = module.X3Bridge()
    bridge.ros.terminal_status = 4
    bridge.ros.terminal_text = "no valid control"
    result = send(bridge)
    assert result["move_base_acceptance_known"] is True
    assert result["terminal_status_known"] is True
    assert result["move_base_status_name"] == "ABORTED"
    assert result["arrived"] is False
    assert result["move_base_status_text"] == "no valid control"


def test_speed_watchdog_cancels_correlated_goal():
    bridge = module.X3Bridge()
    bridge.ros.emit_terminal = False
    bridge.ros.motion_linear_x = 0.25
    result = send(bridge)
    assert result["safety_watchdog_triggered"] is True
    assert result["safety_watchdog_reason"] == "linear_speed_limit_exceeded"
    assert result["safety_cancel_terminal_observed"] is True
    assert result["move_base_status"] == 2
    assert result["navigation_outcome"] == "safety_watchdog_cancelled"
    assert [name for name, _ in bridge.ros.published].count(
        "/move_base_simple/goal"
    ) == 1
    assert [name for name, _ in bridge.ros.published].count(
        "/move_base/cancel"
    ) == 1


def test_distance_watchdog_cancels_correlated_goal():
    bridge = module.X3Bridge()
    bridge.ros.emit_terminal = False
    bridge.ros.motion_linear_x = 0.08
    bridge.ros.motion_angular_z = 0.0
    bridge.ros.odom_displacement = 0.11
    result = send(bridge)
    assert result["safety_watchdog_triggered"] is True
    assert result["safety_watchdog_reason"] == "odom_path_limit_exceeded"
    assert result["safety_cancel_terminal_observed"] is True
    assert result["navigation_outcome"] == "safety_watchdog_cancelled"


def test_acceptance_and_result_timeouts():
    bridge = module.X3Bridge()
    bridge.ros.emit_acceptance = False
    acceptance = send(bridge)
    assert acceptance["publish_call_returned"] is True
    assert acceptance["move_base_acceptance_known"] is False
    assert acceptance["navigation_outcome"] == "acceptance_timeout"

    bridge = module.X3Bridge()
    bridge.ros.emit_terminal = False
    terminal = send(bridge)
    assert terminal["move_base_acceptance_known"] is True
    assert terminal["terminal_status_known"] is True
    assert terminal["move_base_status"] == 2
    assert terminal["navigation_outcome"] == "result_timeout_cancelled"


def test_concurrent_send_is_single_publish():
    bridge = module.X3Bridge()
    bridge.ros.emit_terminal = False
    results = []

    def first_send():
        results.append(send(bridge, result_timeout=0.10))

    thread = threading.Thread(target=first_send)
    thread.start()
    time.sleep(0.02)
    rejected = send(bridge)
    thread.join()
    assert rejected["navigation_outcome"] == "goal_in_progress"
    assert rejected["publish_attempted"] is False
    assert [name for name, _ in bridge.ros.published].count(
        "/move_base_simple/goal"
    ) == 1
    assert results[0]["navigation_outcome"] == "result_timeout_cancelled"


def test_distance_limits_cannot_be_relaxed():
    for keyword in (
        "max_odom_path_m",
        "max_amcl_displacement_m",
    ):
        bridge = module.X3Bridge()
        try:
            send(bridge, **{keyword: 0.100001})
        except ValueError as exc:
            assert "watchdog limit is out of range" in str(exc)
        else:
            raise AssertionError(f"{keyword} accepted a value above 0.10 m")
        assert not bridge.ros.published


def run():
    test_success_and_persistent_publisher()
    test_terminal_failure()
    test_speed_watchdog_cancels_correlated_goal()
    test_distance_watchdog_cancels_correlated_goal()
    test_acceptance_and_result_timeouts()
    test_concurrent_send_is_single_publish()
    test_distance_limits_cannot_be_relaxed()
    print("x3_bridge_navigation_unit_test: PASS")


if __name__ == "__main__":
    run()
