import importlib.util
import json
import math
import os
import sys
import threading
import time
import types
from pathlib import Path


class Message:
    def __init__(self, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)


class String(Message):
    def __init__(self, data=""):
        super().__init__(data=data)


class Primitive:
    def __init__(self, **kwargs):
        self.kwargs = kwargs

    def on_init(self, func):
        return func

    def mcp(self, _name):
        return lambda func: func


def install_stubs():
    api = types.ModuleType("robonix_api")
    api.Primitive = Primitive
    api.Ok = lambda: True
    sys.modules["robonix_api"] = api

    std = types.ModuleType("std_msgs_mcp")
    std.String = String
    sys.modules["std_msgs_mcp"] = std

    mcp = types.ModuleType("rosmaster_x3_bridge_mcp")
    names = [
        "GetX3Status_Request", "GetX3Status_Response",
        "CheckNavReady_Request", "CheckNavReady_Response",
        "PreviewSafeWaypoint_Request", "PreviewSafeWaypoint_Response",
        "PrepareSafeNavigation_Request", "PrepareSafeNavigation_Response",
        "PreviewMoveOptions_Request", "PreviewMoveOptions_Response",
        "PrepareSelectedMove_Request", "PrepareSelectedMove_Response",
        "SendNavGoal_Request", "SendNavGoal_Response",
        "CancelNavGoal_Request", "CancelNavGoal_Response",
        "GoToWaypoint_Request", "GoToWaypoint_Response",
    ]
    for name in names:
        setattr(mcp, name, type(name, (Message,), {}))
    sys.modules["rosmaster_x3_bridge_mcp"] = mcp

    adapter = types.ModuleType("x3_bridge")
    adapter.X3Bridge = type("X3Bridge", (), {})
    sys.modules["x3_bridge"] = adapter
    return mcp


if os.environ.get("STAGE1_USE_REAL_MODULE") == "1":
    import rosmaster_x3_bridge_mcp as MCP
    import rosmaster_x3_bridge.main as provider
else:
    MCP = install_stubs()
    MODULE_PATH = Path(__file__).parent / "main.py"
    SPEC = importlib.util.spec_from_file_location(
        "stage1_provider",
        MODULE_PATH,
    )
    provider = importlib.util.module_from_spec(SPEC)
    SPEC.loader.exec_module(provider)


class RosState:
    is_connected = True


class FakeBridge:
    def __init__(self):
        self.ros = RosState()
        self.x = 0.0
        self.y = 0.0
        self.yaw_deg = 0.0
        self.quaternion_scale = 1.0
        self.amcl_fail = False
        self.costmap_fail = False
        self.plan_fail = False
        self.plan_failures_remaining = 0
        self.plan_delay_sec = 0.0
        self.costmap_value = 0
        self.make_plan_call_count = 0
        self.plan_timeouts = []
        self.navigation_goal_sent = 0
        self.cmd_vel_published = 0

    def get_amcl_pose(self):
        if self.amcl_fail:
            raise RuntimeError("fake AMCL unavailable")
        yaw = math.radians(self.yaw_deg)
        scale = self.quaternion_scale
        return {
            "frame_id": "map",
            "x": self.x,
            "y": self.y,
            "orientation_z": math.sin(yaw / 2.0),
            "orientation_w": math.cos(yaw / 2.0),
            "raw_pose": {
                "position": {"x": self.x, "y": self.y, "z": 0.0},
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": math.sin(yaw / 2.0) * scale,
                    "w": math.cos(yaw / 2.0) * scale,
                },
            },
        }

    def get_global_costmap_grid(self, timeout=8.0):
        del timeout
        if self.costmap_fail:
            raise RuntimeError("fake costmap unavailable")
        width = 120
        height = 120
        return {
            "ok": True,
            "topic": "/move_base/global_costmap/costmap",
            "frame_id": "map",
            "resolution": 0.05,
            "width": width,
            "height": height,
            "origin_x": -3.0,
            "origin_y": -3.0,
            "origin_orientation": {"w": 1.0},
            "data": [self.costmap_value] * (width * height),
        }

    def make_plan_to_pose(
        self,
        goal_x,
        goal_y,
        goal_yaw_deg,
        tolerance=0.05,
        timeout=8.0,
    ):
        del goal_yaw_deg
        self.make_plan_call_count += 1
        self.plan_timeouts.append(timeout)
        if self.plan_delay_sec > 0.0:
            time.sleep(self.plan_delay_sec)
        should_fail = self.plan_fail or self.plan_failures_remaining > 0
        if self.plan_failures_remaining > 0:
            self.plan_failures_remaining -= 1
        if should_fail:
            return {
                "service_name": "/move_base/make_plan",
                "call_ok": False,
                "path_available": False,
                "plan_pose_count": 0,
            }
        return {
            "service_name": "/move_base/make_plan",
            "call_ok": True,
            "path_available": True,
            "plan_pose_count": max(2, int(math.hypot(goal_x, goal_y) / 0.05)),
            "plan_last_x": goal_x,
            "plan_last_y": goal_y,
            "tolerance_m": tolerance,
        }


FAKE = FakeBridge()
provider.ensure_bridge = lambda: FAKE
provider._navigation_ready_snapshot = lambda: {"ok": True, "ready": True}


def data(response):
    return json.loads(response.result_json.data)


def reset():
    FAKE.x = 0.0
    FAKE.y = 0.0
    FAKE.yaw_deg = 0.0
    FAKE.quaternion_scale = 1.0
    FAKE.amcl_fail = False
    FAKE.costmap_fail = False
    FAKE.plan_fail = False
    FAKE.plan_failures_remaining = 0
    FAKE.plan_delay_sec = 0.0
    FAKE.costmap_value = 0
    FAKE.make_plan_call_count = 0
    FAKE.plan_timeouts = []
    with provider._MOVE_OPTION_SESSIONS_LOCK:
        provider._MOVE_OPTION_SESSIONS.clear()
    with provider._PREPARED_NAV_LOCK:
        provider._PREPARED_NAV_TOKENS.clear()


def preview(max_options=5):
    return data(provider.preview_move_options(
        MCP.PreviewMoveOptions_Request(max_options=max_options)
    ))


def select(session_id, option_id):
    return data(provider.prepare_selected_move(
        MCP.PrepareSelectedMove_Request(
            session_id=session_id,
            option_id=option_id,
        )
    ))


def prepare_request(**overrides):
    values = {
        "x": 0.10,
        "y": 0.0,
        "yaw_deg": 0.0,
        "window": 0.05,
        "goal_cost_limit": 80,
        "plan_tolerance": 0.05,
        "strict_high_cost": True,
    }
    values.update(overrides)
    return MCP.PrepareSafeNavigation_Request(**values)


def assert_safety(result):
    assert result["navigation_goal_sent"] is False
    assert result["direct_cmd_vel_published"] is False
    assert result["waypoint_file_modified"] is False


def run():
    reset()

    assert preview(0)["reason"] == "invalid_max_options"
    assert preview(6)["reason"] == "invalid_max_options"

    FAKE.amcl_fail = True
    result = preview(5)
    assert result["reason"] == "amcl_unavailable"
    assert_safety(result)
    FAKE.amcl_fail = False

    FAKE.costmap_fail = True
    result = preview(5)
    assert result["reason"] == "costmap_unavailable"
    assert_safety(result)
    FAKE.costmap_fail = False

    FAKE.plan_fail = True
    result = preview(5)
    assert result["reason"] == "no_safe_options"
    assert result["option_count"] == 0
    assert result["candidate_stats"]["plan_checked"] <= result[
        "candidate_stats"
    ]["plan_check_limit"]
    assert FAKE.make_plan_call_count == result["candidate_stats"][
        "plan_checked"
    ]
    assert result["candidate_stats"]["plan_check_limit_reached"] is True
    assert_safety(result)
    FAKE.plan_fail = False
    FAKE.make_plan_call_count = 0
    FAKE.plan_timeouts = []

    FAKE.costmap_value = 100
    result = preview(5)
    assert result["reason"] == "no_safe_options"
    assert result["option_count"] == 0
    assert FAKE.make_plan_call_count == 0
    assert_safety(result)
    FAKE.costmap_value = 0

    original_ready = provider._navigation_ready_snapshot
    provider._navigation_ready_snapshot = lambda: {"ok": False, "ready": False}
    assert preview(5)["reason"] == "nav_not_ready"
    provider._navigation_ready_snapshot = original_ready

    os.environ["X3_MOVE_OPTIONS_MAX_COUNT"] = "invalid"
    os.environ["X3_MOVE_OPTIONS_SESSION_TTL_SEC"] = "NaN"
    config = provider._move_options_config()
    assert config["max_options"] == 5
    assert config["session_ttl_sec"] == 60.0
    os.environ.pop("X3_MOVE_OPTIONS_MAX_COUNT")
    os.environ.pop("X3_MOVE_OPTIONS_SESSION_TTL_SEC")

    default_config = provider._move_options_config()
    assert default_config["min_radius_m"] == 0.08
    assert default_config["max_radius_m"] == 0.08

    bounded_environment = {
        "X3_MOVE_OPTIONS_MIN_RADIUS_M": "0.01",
        "X3_MOVE_OPTIONS_MAX_RADIUS_M": "0.50",
        "X3_MOVE_OPTIONS_RADIUS_STEP_M": "0.01",
        "X3_MOVE_OPTIONS_ANGLE_STEP_DEG": "1",
        "X3_MOVE_OPTIONS_MAX_PLAN_CHECKS": "999",
        "X3_MOVE_OPTIONS_PLAN_TIMEOUT_SEC": "99",
        "X3_MOVE_OPTIONS_PREVIEW_TIME_BUDGET_SEC": "99",
    }
    os.environ.update(bounded_environment)
    config = provider._move_options_config()
    assert config["min_radius_m"] == 0.05
    assert config["max_radius_m"] == 0.08
    assert config["radius_step_m"] == 0.01
    assert config["angle_step_deg"] == 15
    assert config["max_plan_checks"] == 24
    assert config["plan_timeout_sec"] == 5.0
    assert config["preview_time_budget_sec"] == 30.0
    for name in bounded_environment:
        os.environ.pop(name)

    FAKE.quaternion_scale = 0.0
    result = preview(5)
    assert result["reason"] == "amcl_unavailable"
    assert FAKE.make_plan_call_count == 0
    FAKE.quaternion_scale = 2.0
    FAKE.yaw_deg = 90.0
    normalized_pose = provider._amcl_pose_snapshot(FAKE.get_amcl_pose())
    assert abs(normalized_pose["quaternion_norm_input"] - 2.0) < 1e-9
    assert abs(normalized_pose["yaw_deg"] - 90.0) < 1e-9
    normalized_orientation = normalized_pose["orientation"]
    normalized_norm = math.sqrt(sum(
        float(normalized_orientation[name]) ** 2
        for name in ("x", "y", "z", "w")
    ))
    assert abs(normalized_norm - 1.0) < 1e-9
    FAKE.quaternion_scale = 1.0
    FAKE.yaw_deg = 0.0

    os.environ["X3_MOVE_OPTIONS_MAX_PLAN_CHECKS"] = "5"
    FAKE.plan_fail = True
    FAKE.make_plan_call_count = 0
    result = preview(5)
    assert result["candidate_stats"]["plan_checked"] == 5
    assert result["candidate_stats"]["plan_check_limit"] == 5
    assert result["candidate_stats"]["plan_check_limit_reached"] is True
    assert result["option_count"] == 0
    assert FAKE.make_plan_call_count == 5
    FAKE.plan_fail = False
    os.environ.pop("X3_MOVE_OPTIONS_MAX_PLAN_CHECKS")

    FAKE.plan_failures_remaining = 3
    FAKE.make_plan_call_count = 0
    result = preview(2)
    assert result["ok"] is True
    assert result["option_count"] == 2
    assert result["candidate_stats"]["plan_checked"] > 3
    assert result["candidate_stats"]["plan_passed"] >= 2
    assert FAKE.make_plan_call_count == result["candidate_stats"][
        "plan_checked"
    ]

    FAKE.make_plan_call_count = 0
    FAKE.plan_timeouts = []
    result = preview(1)
    assert result["option_count"] == 1
    assert result["candidate_stats"]["plan_checked"] == 1
    assert FAKE.make_plan_call_count == 1
    assert max(FAKE.plan_timeouts) <= 3.0

    original_config = provider._move_options_config

    def tight_budget_config():
        value = original_config()
        value["preview_time_budget_sec"] = 0.01
        value["max_plan_checks"] = 24
        return value

    provider._move_options_config = tight_budget_config
    FAKE.plan_fail = True
    FAKE.plan_delay_sec = 0.02
    FAKE.make_plan_call_count = 0
    FAKE.plan_timeouts = []
    budget_result = preview(5)
    assert budget_result["preview_time_budget_reached"] is True
    assert budget_result["preview_elapsed_sec"] >= 0.01
    assert FAKE.make_plan_call_count <= 1
    assert all(timeout <= 0.01 for timeout in FAKE.plan_timeouts)
    assert budget_result["option_count"] == 0
    provider._move_options_config = original_config
    FAKE.plan_fail = False
    FAKE.plan_delay_sec = 0.0

    FAKE.make_plan_call_count = 0
    FAKE.plan_timeouts = []
    result = preview(5)
    assert result["ok"] is True
    assert result["reason"] == "session_created"
    assert 1 <= result["option_count"] <= 5
    assert result["candidate_stats"]["geometry_candidate_total"] <= 144
    assert result["candidate_stats"]["costmap_passed"] > 0
    assert result["candidate_stats"]["plan_checked"] <= result[
        "candidate_stats"
    ]["plan_check_limit"]
    assert FAKE.make_plan_call_count == result["candidate_stats"][
        "plan_checked"
    ]
    assert result["search"]["min_radius_m"] >= 0.05
    assert result["search"]["max_radius_m"] <= 0.08
    assert result["search"]["radius_step_m"] >= 0.01
    assert result["search"]["angle_step_deg"] >= 15
    assert result["search"]["plan_timeout_sec"] <= 3.0
    assert [item["option_id"] for item in result["options"]] == list(
        range(1, result["option_count"] + 1)
    )
    for option in result["options"]:
        assert option["unknown_count"] == 0
        assert option["lethal_count"] == 0
        assert option["high_cost_count"] == 0
        assert option["make_plan_ok"] is True
        assert option["direction_description_reliable"] is False
    for index, option in enumerate(result["options"]):
        for other in result["options"][index + 1:]:
            assert provider._circular_angle_distance_deg(
                option["relative_angle_deg"],
                other["relative_angle_deg"],
            ) >= 30.0
    assert_safety(result)

    session_id = result["session_id"]
    selected = select(session_id, 1)
    assert selected["ok"] is True
    assert selected["reason"] == "token_issued"
    assert selected["session_consumed"] is True
    assert selected["token_issued"] is True
    assert selected["confirmation_text"].startswith("CONFIRM_MOVE_")
    token_record = provider._PREPARED_NAV_TOKENS[selected["token"]]
    assert token_record["source"] == "prepare_selected_move"
    assert token_record["execution_authorized"] is True
    assert token_record["move_option_session_id"] == session_id
    assert token_record["selected_option_id"] == 1
    assert token_record["prepared_pose"]["frame_id"] == "map"
    assert token_record["source_session_id"] == session_id
    assert token_record["source_option_id"] == 1
    assert token_record["confirmation_text"] == selected["confirmation_text"]
    assert_safety(selected)
    assert select(session_id, 1)["reason"] == "session_already_used"

    assert select("missing", 1)["reason"] == "session_not_found"

    result = preview(2)
    session_id = result["session_id"]
    assert select(session_id, 255)["reason"] == "invalid_option_id"
    assert select(session_id, 1)["ok"] is True

    result = preview(1)
    session_id = result["session_id"]
    with provider._MOVE_OPTION_SESSIONS_LOCK:
        provider._MOVE_OPTION_SESSIONS[session_id]["expires_monotonic"] = (
            time.monotonic() - 1.0
        )
    assert select(session_id, 1)["reason"] == "session_expired"

    result = preview(1)
    session_id = result["session_id"]
    FAKE.amcl_fail = True
    assert select(session_id, 1)["reason"] == "amcl_check_failed"
    FAKE.amcl_fail = False

    result = preview(1)
    session_id = result["session_id"]
    FAKE.x = 0.20
    moved = select(session_id, 1)
    assert moved["reason"] == "robot_pose_changed"
    assert_safety(moved)
    FAKE.x = 0.0

    result = preview(1)
    session_id = result["session_id"]
    FAKE.costmap_value = 100
    invalidated = select(session_id, 1)
    assert invalidated["reason"] == "costmap_check_failed"
    assert select(session_id, 1)["reason"] == "selected_target_invalidated"
    FAKE.costmap_value = 0

    result = preview(1)
    session_id = result["session_id"]
    FAKE.plan_fail = True
    assert select(session_id, 1)["reason"] == "make_plan_failed"
    FAKE.plan_fail = False

    result = preview(1)
    session_id = result["session_id"]
    entered = threading.Event()
    release = threading.Event()
    original_prepare = provider.prepare_safe_navigation

    def slow_prepare(request):
        entered.set()
        assert release.wait(5.0)
        return original_prepare(request)

    provider.prepare_safe_navigation = slow_prepare
    first_result = {}

    def first_select():
        first_result.update(select(session_id, 1))

    thread = threading.Thread(target=first_select)
    thread.start()
    assert entered.wait(5.0)
    second_result = select(session_id, 1)
    assert second_result["reason"] == "session_selection_in_progress"
    release.set()
    thread.join(5.0)
    assert not thread.is_alive()
    assert first_result["reason"] == "token_issued"
    provider.prepare_safe_navigation = original_prepare
    matching_tokens = [
        record
        for record in provider._PREPARED_NAV_TOKENS.values()
        if record.get("source_session_id") == session_id
    ]
    assert len(matching_tokens) == 1

    result = preview(1)
    old_session_id = result["session_id"]
    with provider._MOVE_OPTION_SESSIONS_LOCK:
        provider._MOVE_OPTION_SESSIONS.clear()
    assert select(old_session_id, 1)["reason"] == "session_not_found"

    with provider._MOVE_OPTION_SESSIONS_LOCK:
        provider._MOVE_OPTION_SESSIONS.clear()
        now = time.monotonic()
        for index in range(provider._MOVE_OPTION_SESSION_LIMIT):
            provider._MOVE_OPTION_SESSIONS[str(index)] = {
                "created_monotonic": float(index),
                "expires_monotonic": now + 1000.0,
            }
        provider._purge_move_option_sessions_locked(now)
        assert len(provider._MOVE_OPTION_SESSIONS) == (
            provider._MOVE_OPTION_SESSION_LIMIT - 1
        )

    reset()
    strict_false = data(provider.prepare_safe_navigation(
        prepare_request(strict_high_cost=False)
    ))
    assert strict_false["token_issued"] is False

    too_far = data(provider.prepare_safe_navigation(
        prepare_request(x=0.80)
    ))
    assert too_far["token_issued"] is False

    for unsafe_cost in (-1, 80, 100):
        FAKE.costmap_value = unsafe_cost
        unsafe = data(provider.prepare_safe_navigation(prepare_request()))
        assert unsafe["token_issued"] is False
    FAKE.costmap_value = 0

    FAKE.plan_fail = True
    no_plan = data(provider.prepare_safe_navigation(prepare_request()))
    assert no_plan["token_issued"] is False
    FAKE.plan_fail = False

    positive_prepare = data(provider.prepare_safe_navigation(prepare_request()))
    assert positive_prepare["token_issued"] is True
    assert positive_prepare["navigation_goal_sent"] is False
    assert positive_prepare["cmd_vel_published"] is False
    assert positive_prepare["waypoint_file_modified"] is False

    preview_request = MCP.PreviewSafeWaypoint_Request(
        min_radius=0.20,
        max_radius=0.20,
        step_radius=0.05,
        angles=8,
        window=0.05,
        goal_cost_limit=80,
        plan_tolerance=0.05,
        strict_high_cost=True,
    )
    legacy_preview = data(provider.preview_safe_waypoint(preview_request))
    assert legacy_preview["ok"] is True
    assert legacy_preview["navigation_goal_sent"] is False
    assert legacy_preview["cmd_vel_published"] is False
    assert legacy_preview["waypoint_file_modified"] is False

    good_quality = FAKE.get_amcl_pose()
    good_quality.update({
        "freshness_verified": True,
        "stamp_sec": time.time(),
        "message_age_sec": 0.1,
        "position_variance_max": 0.01,
        "yaw_variance": 0.02,
    })
    quality_pose = provider._amcl_pose_snapshot(
        good_quality,
        require_quality=True,
    )
    assert quality_pose["quality"]["freshness_verified"] is True

    stale_quality = dict(good_quality)
    stale_quality["message_age_sec"] = 5.0
    try:
        provider._amcl_pose_snapshot(stale_quality, require_quality=True)
        raise AssertionError("stale AMCL pose was accepted")
    except RuntimeError as exc:
        assert "stale" in str(exc)

    uncertain_quality = dict(good_quality)
    uncertain_quality["position_variance_max"] = 0.20
    try:
        provider._amcl_pose_snapshot(uncertain_quality, require_quality=True)
        raise AssertionError("uncertain AMCL pose was accepted")
    except RuntimeError as exc:
        assert "covariance" in str(exc)

    assert FAKE.navigation_goal_sent == 0
    assert FAKE.cmd_vel_published == 0
    print("stage1_unit_test: PASS")


if __name__ == "__main__":
    run()
