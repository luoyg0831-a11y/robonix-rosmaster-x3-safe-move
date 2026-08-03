#!/usr/bin/env python3

import logging
import os
import sys
from pathlib import Path


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

import move_base_cli as cli  # noqa: E402


class FakeApi:
    def __init__(self, execute_ok=True):
        self.preview_calls = []
        self.prepare_calls = []
        self.execute_calls = []
        self.cancel_calls = 0
        self.hard_estop_calls = 0
        self.execute_ok = execute_ok

    def preview(self, max_options):
        self.preview_calls.append(max_options)
        return {
            "ok": True,
            "session_id": "session-1",
            "options": [
                {
                    "option_id": 1,
                    "direction_label": "前方",
                    "x": 0.09,
                    "y": 0.0,
                    "radius_m": 0.09,
                    "window_max_cost": 0,
                    "path_pose_count": 6,
                },
                {
                    "option_id": 2,
                    "direction_label": "左侧",
                    "x": 0.0,
                    "y": 0.09,
                    "radius_m": 0.09,
                    "window_max_cost": 0,
                    "path_pose_count": 7,
                },
            ],
        }

    def prepare(self, session_id, option_id):
        self.prepare_calls.append((session_id, option_id))
        return {
            "ok": True,
            "token": "token-1",
            "confirmation_text": "internal-confirmation",
            "target": {
                "x": 0.0,
                "y": 0.09,
                "distance_from_current_m": 0.09,
            },
            "costmap": {
                "distance_m": 0.09,
                "stats": {"goal_cost": 0, "max_cost": 0},
            },
            "plan": {"plan_pose_count": 7},
        }

    def execute(self, token, confirmation_text):
        self.execute_calls.append((token, confirmation_text))
        return {
            "ok": self.execute_ok,
            "reason": (
                "navigation_succeeded"
                if self.execute_ok
                else "navigation_aborted"
            ),
            "arrived": self.execute_ok,
            "move_base_status_name": (
                "SUCCEEDED" if self.execute_ok else "ABORTED"
            ),
            "move_base_status_text": "done",
            "cmd_vel_observation": {"nonzero_messages": 4},
            "amcl_observation": {
                "displacement_from_validation_m": 0.08
            },
            "odom_observation": {
                "displacement_m": 0.075,
                "path_length_m": 0.08,
            },
        }

    def cancel(self):
        self.cancel_calls += 1
        return {"ok": True}

    def hard_estop(self):
        self.hard_estop_calls += 1
        return True


def reader(values):
    items = iter(values)
    return lambda _prompt: next(items)


def run_success_test():
    api = FakeApi()
    output = []
    old_gate = os.environ.get("X3_ALLOW_PREPARED_NAV_EXECUTION")
    os.environ.pop("X3_ALLOW_PREPARED_NAV_EXECUTION", None)
    try:
        code = cli.run_workflow(
            api,
            read_input=reader([
                "9",
                "\x1b[200~２\x1b[201~",
            ]),
            write=output.append,
        )
        assert code == 0
        assert api.preview_calls == [5]
        assert api.prepare_calls == [("session-1", 2)]
        assert api.execute_calls == [("token-1", "internal-confirmation")]
        assert "X3_ALLOW_PREPARED_NAV_EXECUTION" not in os.environ
        assert any("距离=0.09m" in line for line in output)
        assert any("SUCCEEDED" in line for line in output)
    finally:
        if old_gate is None:
            os.environ.pop("X3_ALLOW_PREPARED_NAV_EXECUTION", None)
        else:
            os.environ["X3_ALLOW_PREPARED_NAV_EXECUTION"] = old_gate


def run_cancel_tests():
    api = FakeApi()
    assert cli.run_workflow(
        api,
        read_input=reader(["q"]),
        write=lambda _line: None,
    ) == 0
    assert not api.prepare_calls
    assert not api.execute_calls

    api = FakeApi()
    assert cli.run_workflow(
        api,
        read_input=reader(["9", "q"]),
        write=lambda _line: None,
    ) == 0
    assert not api.prepare_calls
    assert not api.execute_calls


def run_failure_test():
    api = FakeApi(execute_ok=False)
    code = cli.run_workflow(
        api,
        read_input=reader(["1"]),
        write=lambda _line: None,
    )
    assert code == 5
    assert len(api.execute_calls) == 1


def run_dry_run_test():
    api = FakeApi()
    code = cli.run_workflow(
        api,
        dry_run=True,
        read_input=reader(["1"]),
        write=lambda _line: None,
    )
    assert code == 0
    assert api.prepare_calls == [("session-1", 1)]
    assert not api.execute_calls


def run_watchdog_estop_test():
    class WatchdogApi(FakeApi):
        def execute(self, token, confirmation_text):
            self.execute_calls.append((token, confirmation_text))
            return {
                "ok": False,
                "reason": "navigation_safety_watchdog",
                "arrived": False,
                "move_base_status_name": "ACTIVE",
                "safety_watchdog_triggered": True,
                "safety_watchdog_reason": "odom_path_limit_exceeded",
                "safety_cancel_terminal_observed": False,
                "cmd_vel_observation": {"nonzero_messages": 2},
                "amcl_observation": {
                    "displacement_from_validation_m": 0.11,
                },
                "odom_observation": {
                    "displacement_m": 0.10,
                    "path_length_m": 0.11,
                },
            }

    api = WatchdogApi()
    output = []
    code = cli.run_workflow(
        api,
        read_input=reader(["1"]),
        write=output.append,
    )
    assert code == 5
    assert api.cancel_calls == 1
    assert api.hard_estop_calls == 1
    assert any("硬急停=已完成" in line for line in output)


def run_dependency_logging_test():
    loggers = [logging.getLogger(name) for name in cli._DEPENDENCY_LOGGERS]
    previous = [logger.level for logger in loggers]
    try:
        for logger in loggers:
            logger.setLevel(logging.DEBUG)
        cli.configure_dependency_logging()
        assert all(logger.level == logging.WARNING for logger in loggers)
    finally:
        for logger, level in zip(loggers, previous):
            logger.setLevel(level)


if __name__ == "__main__":
    run_success_test()
    run_cancel_tests()
    run_failure_test()
    run_dry_run_test()
    run_watchdog_estop_test()
    run_dependency_logging_test()
    print("move_base_cli_unit_test: PASS")
