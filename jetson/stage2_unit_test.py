import json
import math
import os
import secrets
import threading
import time
import types

import stage1_unit_test as stage1


provider = stage1.provider
MCP = stage1.MCP
FAKE = stage1.FAKE
ORIGINAL_EXECUTION_BUDGET_FUNCTION = (
    provider._prepared_navigation_execution_budget_sec
)

for name in (
    "ExecutePreparedNavigation_Request",
    "ExecutePreparedNavigation_Response",
):
    if not hasattr(MCP, name):
        setattr(MCP, name, type(name, (stage1.Message,), {}))


def controlled_make_plan(
    self,
    goal_x,
    goal_y,
    goal_yaw_deg,
    tolerance=0.05,
    timeout=8.0,
):
    result = stage1.FakeBridge.make_plan_to_pose(
        self,
        goal_x,
        goal_y,
        goal_yaw_deg,
        tolerance=tolerance,
        timeout=timeout,
    )
    if result.get("call_ok") is not True:
        return result
    if self.plan_empty:
        result.update({
            "path_available": False,
            "plan_pose_count": 0,
            "plan_last_x": None,
            "plan_last_y": None,
        })
    elif self.plan_terminal_offset_m:
        result["plan_last_x"] = goal_x + self.plan_terminal_offset_m
    return result


def controlled_get_global_costmap_grid(self, timeout=8.0):
    self.costmap_timeouts.append(timeout)
    if self.costmap_delay_sec:
        time.sleep(self.costmap_delay_sec)
    return stage1.FakeBridge.get_global_costmap_grid(
        self,
        timeout=timeout,
    )


def fake_send_navigation_goal_pose(
    self,
    frame_id,
    x,
    y,
    yaw_deg,
    acceptance_timeout=5.0,
    result_timeout=90.0,
    **_watchdog,
):
    self.last_acceptance_timeout = acceptance_timeout
    self.last_result_timeout = result_timeout
    with self.publish_lock:
        self.goal_publish_call_count += 1
        self.published_targets.append({
            "frame_id": frame_id,
            "x": x,
            "y": y,
            "yaw_deg": yaw_deg,
        })
    if self.publish_delay_sec:
        time.sleep(self.publish_delay_sec)
    if self.publish_raise_unexpected:
        raise RuntimeError("unexpected fake adapter failure")
    if self.publish_fail:
        return {
            "publish_attempted": True,
            "publish_call_returned": False,
            "publish_outcome_unknown": True,
            "publish_error_type": "RuntimeError",
            "cleanup_attempted": True,
            "cleanup_completed": True,
            "cleanup_error_type": None,
            "move_base_acceptance_known": False,
            "terminal_status_known": False,
            "navigation_outcome": "publish_outcome_unknown",
        }
    with self.publish_lock:
        self.goal_publish_success_count += 1
    status_names = {
        1: "ACTIVE",
        2: "PREEMPTED",
        3: "SUCCEEDED",
        4: "ABORTED",
        5: "REJECTED",
        8: "RECALLED",
        9: "LOST",
    }
    result = {
        "publish_attempted": True,
        "publish_call_returned": True,
        "publish_outcome_unknown": False,
        "publish_error_type": None,
        "cleanup_attempted": True,
        "cleanup_completed": not self.cleanup_fail,
        "cleanup_error_type": (
            "RuntimeError" if self.cleanup_fail else None
        ),
        "move_base_acceptance_known": self.acceptance_known,
        "move_base_goal_id": (
            "fake-move-base-goal-1" if self.acceptance_known else None
        ),
        "move_base_status": (
            self.navigation_status if self.acceptance_known else None
        ),
        "move_base_status_name": (
            status_names.get(self.navigation_status)
            if self.acceptance_known else None
        ),
        "move_base_status_text": self.navigation_status_text,
        "terminal_status_known": self.terminal_status_known,
        "arrival_known": self.terminal_status_known,
        "arrived": (
            self.terminal_status_known and self.navigation_status == 3
        ),
        "navigation_outcome": self.navigation_outcome,
        "cmd_vel_messages": self.cmd_vel_messages,
        "nonzero_cmd_vel_messages": self.nonzero_cmd_vel_messages,
        "max_linear_mps": 0.12 if self.nonzero_cmd_vel_messages else 0.0,
        "max_angular_rps": 0.2 if self.nonzero_cmd_vel_messages else 0.0,
        "amcl_messages": self.amcl_messages,
        "amcl_first_pose": {"x": 0.0, "y": 0.0},
        "amcl_last_pose": {
            "x": self.amcl_displacement_m,
            "y": 0.0,
        },
        "amcl_displacement_m": self.amcl_displacement_m,
        "odom_monitor_ready": True,
        "odom_messages": 3,
        "odom_first_pose": {"x": 0.0, "y": 0.0},
        "odom_last_pose": {"x": 0.08, "y": 0.0},
        "odom_displacement_m": 0.08,
        "odom_path_length_m": 0.09,
        "safety_watchdog_triggered": False,
    }
    return result


FAKE.make_plan_to_pose = types.MethodType(controlled_make_plan, FAKE)
FAKE.get_global_costmap_grid = types.MethodType(
    controlled_get_global_costmap_grid,
    FAKE,
)
FAKE.send_navigation_goal_pose = types.MethodType(
    fake_send_navigation_goal_pose,
    FAKE,
)


def reset_all():
    stage1.reset()
    FAKE.plan_empty = False
    FAKE.plan_terminal_offset_m = 0.0
    FAKE.costmap_delay_sec = 0.0
    FAKE.costmap_timeouts = []
    FAKE.publish_delay_sec = 0.0
    FAKE.publish_fail = False
    FAKE.publish_raise_unexpected = False
    FAKE.cleanup_fail = False
    FAKE.acceptance_known = True
    FAKE.terminal_status_known = True
    FAKE.navigation_status = 3
    FAKE.navigation_status_text = "Goal reached."
    FAKE.navigation_outcome = "succeeded"
    FAKE.cmd_vel_messages = 5
    FAKE.nonzero_cmd_vel_messages = 3
    FAKE.amcl_messages = 3
    FAKE.amcl_displacement_m = 0.08
    FAKE.last_acceptance_timeout = None
    FAKE.last_result_timeout = None
    FAKE.goal_publish_call_count = 0
    FAKE.goal_publish_success_count = 0
    FAKE.published_targets = []
    FAKE.publish_lock = threading.Lock()
    provider._navigation_ready_snapshot = lambda: {
        "ok": True,
        "ready": True,
    }
    provider._prepared_navigation_execution_budget_sec = (
        ORIGINAL_EXECUTION_BUDGET_FUNCTION
    )
    os.environ.pop("X3_ALLOW_PREPARED_NAV_EXECUTION", None)
    os.environ.pop("X3_ALLOW_NAV_GOAL", None)


def execute(token, confirmation, allow_execute=True):
    return stage1.data(provider.execute_prepared_navigation(
        MCP.ExecutePreparedNavigation_Request(
            token=token,
            allow_execute=allow_execute,
            confirmation_text=confirmation,
        )
    ))


def add_authorized_token(
    *,
    target=None,
    prepared_pose=None,
    expires_offset_sec=120.0,
    used=False,
    executing=False,
    execution_authorized=True,
    source="prepare_selected_move",
):
    token = secrets.token_urlsafe(18)
    confirmation = "CONFIRM_MOVE_TEST_" + secrets.token_hex(3).upper()
    now_epoch = time.time()
    now_monotonic = time.monotonic()
    if prepared_pose is None:
        prepared_pose = provider._amcl_pose_snapshot(FAKE.get_amcl_pose())
    if target is None:
        target = {
            "frame_id": "map",
            "x": 0.10,
            "y": 0.0,
            "yaw_deg": 0.0,
        }
    record = {
        "token": token,
        "created_epoch": now_epoch,
        "created_monotonic": now_monotonic,
        "expires_epoch": now_epoch + expires_offset_sec,
        "expires_monotonic": now_monotonic + expires_offset_sec,
        "source": source,
        "execution_authorized": execution_authorized,
        "executing": executing,
        "used": used,
        "execution_claimed_epoch": None,
        "execution_claimed_monotonic": None,
        "execution_claim_id": None,
        "execution_deadline_monotonic": None,
        "used_epoch": now_epoch if used else None,
        "used_monotonic": now_monotonic if used else None,
        "used_reason": "test_used" if used else None,
        "goal_publish_attempted": False,
        "goal_publish_completed": False,
        "goal_publish_call_returned": False,
        "goal_publish_outcome_unknown": False,
        "goal_topic_cleanup_attempted": False,
        "goal_topic_cleanup_completed": None,
        "goal_topic_cleanup_error_type": None,
        "move_option_session_id": "test_session",
        "selected_option_id": 1,
        "confirmation_text": confirmation,
        "prepared_pose": prepared_pose,
        "target": target,
        "parameters": {
            "window": 0.05,
            "goal_cost_limit": 80,
            "plan_tolerance": 0.05,
            "strict_high_cost": True,
            "max_prepare_distance_m": 0.10,
            "plan_timeout_sec": 2.0,
            "token_ttl_sec": 300.0,
        },
        "validation_digest": "test_validation_digest",
    }
    with provider._PREPARED_NAV_LOCK:
        provider._PREPARED_NAV_TOKENS[token] = record
    return token, confirmation


def assert_no_motion():
    assert FAKE.goal_publish_call_count == 0
    assert FAKE.goal_publish_success_count == 0
    assert FAKE.cmd_vel_published == 0


def assert_unconsumed(token):
    with provider._PREPARED_NAV_LOCK:
        record = provider._PREPARED_NAV_TOKENS[token]
        assert record["used"] is False
        assert record["executing"] is False


def assert_consuming_failure(expected_reason, configure=None, **token_kwargs):
    reset_all()
    token, confirmation = add_authorized_token(**token_kwargs)
    if configure is not None:
        configure()
    os.environ["X3_ALLOW_PREPARED_NAV_EXECUTION"] = "1"
    result = execute(token, confirmation)
    assert result["reason"] == expected_reason, result
    assert result["token_claimed"] is True
    assert result["token_consumed"] is True
    assert result["navigation_goal_sent"] is False
    assert result["direct_cmd_vel_published"] is False
    assert_no_motion()
    print("third_validation_failure=" + json.dumps({
        "reason": result["reason"],
        "token_consumed": result["token_consumed"],
        "navigation_goal_sent": result["navigation_goal_sent"],
        "goal_publish_count": FAKE.goal_publish_call_count,
        "cmd_vel_publish_count": FAKE.cmd_vel_published,
    }, sort_keys=True))
    with provider._PREPARED_NAV_LOCK:
        record = provider._PREPARED_NAV_TOKENS[token]
        assert record["used"] is True
        assert record["executing"] is False
        assert record["used_reason"] == expected_reason
        assert record["goal_publish_attempted"] is False
        assert record["goal_publish_completed"] is False
    retry = execute(token, confirmation)
    assert retry["reason"] == "token_already_used"
    assert_no_motion()


def run_rejection_tests():
    reset_all()
    assert execute("", "") ["reason"] == "invalid_request"
    assert execute("unknown", "anything")["reason"] == "token_not_found"
    assert_no_motion()

    token, confirmation = add_authorized_token(expires_offset_sec=-1.0)
    expired = execute(token, confirmation)
    assert expired["reason"] == "token_expired"
    assert expired["token_consumed"] is False
    assert_no_motion()

    reset_all()
    token, confirmation = add_authorized_token()
    os.environ["X3_ALLOW_PREPARED_NAV_EXECUTION"] = "1"
    denied = execute(token, confirmation, allow_execute=False)
    assert denied["reason"] == "execution_not_allowed"
    assert_unconsumed(token)
    assert_no_motion()

    reset_all()
    token, confirmation = add_authorized_token()
    os.environ["X3_ALLOW_NAV_GOAL"] = "1"
    closed = execute(token, confirmation)
    assert closed["reason"] == "execution_gate_closed"
    assert closed["execution_gate_open"] is False
    assert_unconsumed(token)
    assert_no_motion()
    os.environ["X3_ALLOW_PREPARED_NAV_EXECUTION"] = "true"
    assert execute(token, confirmation)["reason"] == "execution_gate_closed"
    assert_unconsumed(token)

    reset_all()
    token, confirmation = add_authorized_token()
    os.environ["X3_ALLOW_PREPARED_NAV_EXECUTION"] = "1"
    mismatch = execute(token, confirmation + "_WRONG")
    assert mismatch["reason"] == "confirmation_mismatch"
    assert_unconsumed(token)
    assert_no_motion()

    reset_all()
    direct = stage1.data(provider.prepare_safe_navigation(
        stage1.prepare_request()
    ))
    assert direct.get("ok") is True, direct
    direct_token = direct["confirmation"]["token"]
    os.environ["X3_ALLOW_PREPARED_NAV_EXECUTION"] = "1"
    unauthorized = execute(direct_token, "anything")
    assert unauthorized["reason"] == "token_not_execution_authorized"
    assert_unconsumed(direct_token)
    assert_no_motion()

    reset_all()
    token, confirmation = add_authorized_token(used=True)
    os.environ["X3_ALLOW_PREPARED_NAV_EXECUTION"] = "1"
    assert execute(token, confirmation)["reason"] == "token_already_used"
    assert_no_motion()

    reset_all()
    token, confirmation = add_authorized_token(executing=True)
    os.environ["X3_ALLOW_PREPARED_NAV_EXECUTION"] = "1"
    assert execute(token, confirmation)["reason"] == "execution_in_progress"
    assert_no_motion()


def run_third_validation_failures():
    assert_consuming_failure(
        "nav_not_ready",
        lambda: setattr(
            provider,
            "_navigation_ready_snapshot",
            lambda: {"ok": False, "ready": False},
        ),
    )
    assert_consuming_failure(
        "amcl_check_failed",
        lambda: setattr(FAKE, "amcl_fail", True),
    )
    assert_consuming_failure(
        "amcl_check_failed",
        lambda: setattr(FAKE, "quaternion_scale", 0.0),
    )
    assert_consuming_failure(
        "robot_pose_changed",
        lambda: setattr(FAKE, "x", 0.11),
    )
    assert_consuming_failure(
        "target_invalid",
        target={
            "frame_id": "map",
            "x": math.nan,
            "y": 0.0,
            "yaw_deg": 0.0,
        },
    )
    assert_consuming_failure(
        "target_invalid",
        target={
            "frame_id": "map",
            "x": 0.60,
            "y": 0.0,
            "yaw_deg": 0.0,
        },
    )
    for value in (-1, 100, 80):
        assert_consuming_failure(
            "costmap_check_failed",
            lambda value=value: setattr(FAKE, "costmap_value", value),
        )
    assert_consuming_failure(
        "make_plan_failed",
        lambda: setattr(FAKE, "plan_fail", True),
    )
    assert_consuming_failure(
        "make_plan_failed",
        lambda: setattr(FAKE, "plan_empty", True),
    )
    assert_consuming_failure(
        "make_plan_failed",
        lambda: setattr(FAKE, "plan_terminal_offset_m", 0.20),
    )


def run_positive_and_replay_test():
    reset_all()
    preview = stage1.preview(1)
    selected = stage1.select(preview["session_id"], 1)
    token = selected["token"]
    confirmation = selected["confirmation_text"]
    with provider._PREPARED_NAV_LOCK:
        record = provider._PREPARED_NAV_TOKENS[token]
        assert record["source"] == "prepare_selected_move"
        assert record["execution_authorized"] is True
        assert record["move_option_session_id"] == preview["session_id"]
        assert record["selected_option_id"] == 1
        assert record["prepared_pose"]["frame_id"] == "map"
        frozen_target = dict(record["target"])

    os.environ["X3_ALLOW_PREPARED_NAV_EXECUTION"] = "1"
    result = execute(token, confirmation)
    assert result["ok"] is True
    assert result["reason"] == "navigation_succeeded"
    assert result["third_validation_passed"] is True
    assert result["goal_publish_attempted"] is True
    assert result["goal_publish_completed"] is True
    assert result["navigation_goal_sent"] is True
    assert result["navigation_goal_sent_known"] is True
    assert result["goal_publish_call_returned"] is True
    assert result["goal_publish_outcome_unknown"] is False
    assert result["move_base_acceptance_known"] is True
    assert result["move_base_goal_id"] == "fake-move-base-goal-1"
    assert result["move_base_status"] == 3
    assert result["move_base_status_name"] == "SUCCEEDED"
    assert result["terminal_status_known"] is True
    assert result["arrived"] is True
    assert result["motion_evidence_confirmed"] is True
    assert result["cmd_vel_observation"]["nonzero_messages"] == 3
    assert result["amcl_observation"][
        "displacement_from_validation_m"
    ] == 0.08
    assert result["goal_topic_cleanup_completed"] is True
    assert result["execution_started"] is True
    assert result["execution_elapsed_sec"] >= 0.0
    assert result["execution_time_budget_sec"] == 20.0
    assert result["execution_time_budget_reached"] is False
    assert result["token_consumed"] is True
    assert FAKE.goal_publish_call_count == 1
    assert FAKE.goal_publish_success_count == 1
    assert FAKE.published_targets == [frozen_target]
    assert FAKE.cmd_vel_published == 0
    with provider._PREPARED_NAV_LOCK:
        record = provider._PREPARED_NAV_TOKENS[token]
        assert record["used"] is True
        assert record["executing"] is False
        assert record["execution_claimed_epoch"] is not None
        assert record["execution_claimed_monotonic"] is not None
        assert record["used_epoch"] is not None
        assert record["used_monotonic"] is not None
        assert record["used_reason"] == "navigation_succeeded"
        assert record["goal_publish_attempted"] is True
        assert record["goal_publish_completed"] is True
        assert record["goal_publish_call_returned"] is True
        assert record["goal_publish_outcome_unknown"] is False

    replay = execute(token, confirmation)
    assert replay["reason"] == "token_already_used"
    assert FAKE.goal_publish_call_count == 1
    assert FAKE.cmd_vel_published == 0
    print("provider_publish_normal=" + json.dumps({
        "reason": result["reason"],
        "navigation_goal_sent": result["navigation_goal_sent"],
        "navigation_goal_sent_known": result["navigation_goal_sent_known"],
        "goal_publish_completed": result["goal_publish_completed"],
        "goal_publish_outcome_unknown": (
            result["goal_publish_outcome_unknown"]
        ),
        "goal_topic_cleanup_completed": (
            result["goal_topic_cleanup_completed"]
        ),
        "goal_publish_count": FAKE.goal_publish_call_count,
        "cmd_vel_publish_count": FAKE.cmd_vel_published,
    }, sort_keys=True))


def run_concurrency_test():
    reset_all()
    token, confirmation = add_authorized_token()
    FAKE.publish_delay_sec = 0.05
    os.environ["X3_ALLOW_PREPARED_NAV_EXECUTION"] = "1"
    barrier = threading.Barrier(3)
    results = []
    result_lock = threading.Lock()

    def worker():
        barrier.wait()
        result = execute(token, confirmation)
        with result_lock:
            results.append(result)

    threads = [threading.Thread(target=worker) for _ in range(2)]
    for thread in threads:
        thread.start()
    barrier.wait()
    for thread in threads:
        thread.join()

    reasons = {result["reason"] for result in results}
    assert "navigation_succeeded" in reasons
    assert reasons <= {
        "navigation_succeeded",
        "execution_in_progress",
        "token_already_used",
    }
    assert FAKE.goal_publish_call_count == 1
    assert FAKE.goal_publish_success_count == 1
    assert FAKE.cmd_vel_published == 0
    print("concurrency_goal_publish_count: 1; cmd_vel_publish_count: 0")


def run_cleanup_failure_test():
    reset_all()
    token, confirmation = add_authorized_token()
    FAKE.cleanup_fail = True
    os.environ["X3_ALLOW_PREPARED_NAV_EXECUTION"] = "1"
    result = execute(token, confirmation)
    assert result["ok"] is True
    assert result["reason"] == "navigation_succeeded"
    assert result["navigation_goal_sent"] is True
    assert result["navigation_goal_sent_known"] is True
    assert result["goal_publish_completed"] is True
    assert result["goal_publish_call_returned"] is True
    assert result["goal_publish_outcome_unknown"] is False
    assert result["goal_topic_cleanup_attempted"] is True
    assert result["goal_topic_cleanup_completed"] is False
    assert result["goal_topic_cleanup_error_type"] == "RuntimeError"
    assert result["token_consumed"] is True
    assert FAKE.goal_publish_call_count == 1
    assert FAKE.goal_publish_success_count == 1
    assert FAKE.cmd_vel_published == 0
    replay = execute(token, confirmation)
    assert replay["reason"] == "token_already_used"
    assert FAKE.goal_publish_call_count == 1
    print("provider_cleanup_failure=" + json.dumps({
        "reason": result["reason"],
        "navigation_goal_sent": result["navigation_goal_sent"],
        "navigation_goal_sent_known": (
            result["navigation_goal_sent_known"]
        ),
        "goal_publish_completed": result["goal_publish_completed"],
        "goal_topic_cleanup_completed": (
            result["goal_topic_cleanup_completed"]
        ),
        "goal_topic_cleanup_error_type": (
            result["goal_topic_cleanup_error_type"]
        ),
        "goal_publish_count": FAKE.goal_publish_call_count,
        "cmd_vel_publish_count": FAKE.cmd_vel_published,
    }, sort_keys=True))


def run_navigation_outcome_tests():
    terminal_cases = (
        (2, "PREEMPTED", "navigation_preempted"),
        (4, "ABORTED", "navigation_aborted"),
        (5, "REJECTED", "navigation_rejected"),
        (8, "RECALLED", "navigation_recalled"),
        (9, "LOST", "navigation_lost"),
    )
    for status, name, expected_reason in terminal_cases:
        reset_all()
        token, confirmation = add_authorized_token()
        FAKE.navigation_status = status
        FAKE.navigation_status_text = "fake " + name.lower()
        FAKE.navigation_outcome = "failed"
        os.environ["X3_ALLOW_PREPARED_NAV_EXECUTION"] = "1"
        result = execute(token, confirmation)
        assert result["ok"] is False
        assert result["reason"] == expected_reason, result
        assert result["move_base_acceptance_known"] is True
        assert result["terminal_status_known"] is True
        assert result["move_base_status"] == status
        assert result["move_base_status_name"] == name
        assert result["navigation_goal_sent"] is True
        assert result["token_consumed"] is True
        assert result["direct_cmd_vel_published"] is False
        assert FAKE.goal_publish_call_count == 1

    reset_all()
    token, confirmation = add_authorized_token()
    FAKE.acceptance_known = False
    FAKE.terminal_status_known = False
    FAKE.navigation_status = None
    FAKE.navigation_outcome = "acceptance_timeout"
    os.environ["X3_ALLOW_PREPARED_NAV_EXECUTION"] = "1"
    acceptance_timeout = execute(token, confirmation)
    assert acceptance_timeout["reason"] == "goal_acceptance_timeout"
    assert acceptance_timeout["navigation_goal_sent"] is None
    assert acceptance_timeout["navigation_goal_sent_known"] is False
    assert acceptance_timeout["token_consumed"] is True
    assert FAKE.goal_publish_call_count == 1

    reset_all()
    token, confirmation = add_authorized_token()
    FAKE.terminal_status_known = False
    FAKE.navigation_status = 1
    FAKE.navigation_status_text = "active"
    FAKE.navigation_outcome = "result_timeout"
    os.environ["X3_ALLOW_PREPARED_NAV_EXECUTION"] = "1"
    result_timeout = execute(token, confirmation)
    assert result_timeout["reason"] == "navigation_result_timeout"
    assert result_timeout["move_base_acceptance_known"] is True
    assert result_timeout["navigation_goal_sent"] is True
    assert result_timeout["terminal_status_known"] is False
    assert result_timeout["token_consumed"] is True
    assert FAKE.goal_publish_call_count == 1

    reset_all()
    token, confirmation = add_authorized_token()
    FAKE.cmd_vel_messages = 2
    FAKE.nonzero_cmd_vel_messages = 0
    FAKE.amcl_displacement_m = 0.0
    os.environ["X3_ALLOW_PREPARED_NAV_EXECUTION"] = "1"
    no_motion = execute(token, confirmation)
    assert no_motion["ok"] is False
    assert (
        no_motion["reason"]
        == "navigation_succeeded_without_motion_evidence"
    )
    assert no_motion["move_base_status"] == 3
    assert no_motion["arrived"] is True
    assert no_motion["motion_evidence_confirmed"] is False
    assert FAKE.goal_publish_call_count == 1
    assert FAKE.cmd_vel_published == 0
    print("navigation_outcome_matrix: PASS")


def run_publish_failure_test():
    reset_all()
    token, confirmation = add_authorized_token()
    FAKE.publish_fail = True
    os.environ["X3_ALLOW_PREPARED_NAV_EXECUTION"] = "1"
    result = execute(token, confirmation)
    assert result["reason"] == "goal_publish_outcome_unknown"
    assert result["third_validation_passed"] is True
    assert result["goal_publish_attempted"] is True
    assert result["goal_publish_completed"] is False
    assert result["goal_publish_call_returned"] is False
    assert result["goal_publish_outcome_unknown"] is True
    assert result["navigation_goal_sent"] is None
    assert result["navigation_goal_sent_known"] is False
    assert "may already have been delivered" in result["message"]
    assert "Do not retry this token" in result["message"]
    assert result["token_consumed"] is True
    assert FAKE.goal_publish_call_count == 1
    assert FAKE.goal_publish_success_count == 0
    assert FAKE.cmd_vel_published == 0
    with provider._PREPARED_NAV_LOCK:
        record = provider._PREPARED_NAV_TOKENS[token]
        assert record["used"] is True
        assert record["executing"] is False
        assert record["used_reason"] == "goal_publish_outcome_unknown"
        assert record["goal_publish_attempted"] is True
        assert record["goal_publish_completed"] is False
        assert record["goal_publish_call_returned"] is False
        assert record["goal_publish_outcome_unknown"] is True
    replay = execute(token, confirmation)
    assert replay["reason"] == "token_already_used"
    assert FAKE.goal_publish_call_count == 1
    print(
        "provider_publish_exception="
        + json.dumps({
            "reason": result["reason"],
            "goal_publish_attempted": result["goal_publish_attempted"],
            "goal_publish_completed": result["goal_publish_completed"],
            "goal_publish_outcome_unknown": (
                result["goal_publish_outcome_unknown"]
            ),
            "navigation_goal_sent": result["navigation_goal_sent"],
            "navigation_goal_sent_known": (
                result["navigation_goal_sent_known"]
            ),
            "goal_publish_count": FAKE.goal_publish_call_count,
            "cmd_vel_publish_count": FAKE.cmd_vel_published,
        }, sort_keys=True)
    )


def run_execution_freshness_tests():
    reset_all()
    os.environ["X3_PREPARED_NAV_EXECUTION_BUDGET_SEC"] = "0.1"
    assert ORIGINAL_EXECUTION_BUDGET_FUNCTION() == 5.0
    os.environ["X3_PREPARED_NAV_EXECUTION_BUDGET_SEC"] = "999"
    assert ORIGINAL_EXECUTION_BUDGET_FUNCTION() == 30.0
    os.environ.pop("X3_PREPARED_NAV_EXECUTION_BUDGET_SEC")
    assert ORIGINAL_EXECUTION_BUDGET_FUNCTION() == 20.0

    reset_all()
    token, confirmation = add_authorized_token(expires_offset_sec=0.03)

    def slow_readiness():
        time.sleep(0.05)
        return {"ok": True, "ready": True}

    provider._navigation_ready_snapshot = slow_readiness
    os.environ["X3_ALLOW_PREPARED_NAV_EXECUTION"] = "1"
    expired = execute(token, confirmation)
    assert expired["reason"] == "token_expired_before_publish"
    assert expired["token_consumed"] is True
    assert expired["navigation_goal_sent"] is False
    assert_no_motion()
    expired_retry = execute(token, confirmation)
    assert expired_retry["reason"] in {
        "token_expired",
        "token_already_used",
    }
    assert_no_motion()
    print("freshness_token_expired=" + json.dumps({
        "reason": expired["reason"],
        "execution_elapsed_sec": expired["execution_elapsed_sec"],
        "goal_publish_count": FAKE.goal_publish_call_count,
        "token_consumed": expired["token_consumed"],
    }, sort_keys=True))

    reset_all()
    token, confirmation = add_authorized_token()
    provider._prepared_navigation_execution_budget_sec = lambda: 0.01
    provider._navigation_ready_snapshot = slow_readiness
    os.environ["X3_ALLOW_PREPARED_NAV_EXECUTION"] = "1"
    timed_out = execute(token, confirmation)
    assert timed_out["reason"] == "execution_validation_timeout"
    assert timed_out["execution_time_budget_reached"] is True
    assert timed_out["token_consumed"] is True
    assert_no_motion()
    assert execute(token, confirmation)["reason"] == "token_already_used"
    print("freshness_budget_timeout=" + json.dumps({
        "reason": timed_out["reason"],
        "execution_elapsed_sec": timed_out["execution_elapsed_sec"],
        "execution_time_budget_sec": timed_out[
            "execution_time_budget_sec"
        ],
        "execution_time_budget_reached": timed_out[
            "execution_time_budget_reached"
        ],
        "goal_publish_count": FAKE.goal_publish_call_count,
    }, sort_keys=True))

    reset_all()
    token, confirmation = add_authorized_token()

    def close_gate_after_claim():
        os.environ.pop("X3_ALLOW_PREPARED_NAV_EXECUTION", None)
        return {"ok": True, "ready": True}

    provider._navigation_ready_snapshot = close_gate_after_claim
    os.environ["X3_ALLOW_PREPARED_NAV_EXECUTION"] = "1"
    gate_closed = execute(token, confirmation)
    assert (
        gate_closed["reason"]
        == "execution_gate_closed_before_publish"
    )
    assert gate_closed["token_consumed"] is True
    assert gate_closed["execution_gate_open"] is False
    assert_no_motion()
    print("freshness_gate_closed=" + json.dumps({
        "reason": gate_closed["reason"],
        "execution_gate_open": gate_closed["execution_gate_open"],
        "goal_publish_count": FAKE.goal_publish_call_count,
        "token_consumed": gate_closed["token_consumed"],
    }, sort_keys=True))

    reset_all()
    token, confirmation = add_authorized_token()

    def replace_claim_id():
        with provider._PREPARED_NAV_LOCK:
            provider._PREPARED_NAV_TOKENS[token][
                "execution_claim_id"
            ] = "replaced-claim-id"
        return {"ok": True, "ready": True}

    provider._navigation_ready_snapshot = replace_claim_id
    os.environ["X3_ALLOW_PREPARED_NAV_EXECUTION"] = "1"
    claim_lost = execute(token, confirmation)
    assert claim_lost["reason"] == "execution_claim_lost"
    assert claim_lost["token_consumed"] is True
    assert_no_motion()
    assert execute(token, confirmation)["reason"] == "token_already_used"
    print("freshness_claim_lost=" + json.dumps({
        "reason": claim_lost["reason"],
        "goal_publish_count": FAKE.goal_publish_call_count,
        "token_consumed": claim_lost["token_consumed"],
    }, sort_keys=True))

    reset_all()
    token, confirmation = add_authorized_token()
    provider._prepared_navigation_execution_budget_sec = lambda: 0.5
    os.environ["X3_ALLOW_PREPARED_NAV_EXECUTION"] = "1"
    bounded = execute(token, confirmation)
    assert bounded["reason"] == "navigation_succeeded"
    assert FAKE.costmap_timeouts
    assert FAKE.plan_timeouts
    assert 0.0 < max(FAKE.costmap_timeouts) <= 0.5
    assert 0.0 < max(FAKE.plan_timeouts) <= 0.5
    assert max(FAKE.plan_timeouts) <= 3.0
    print("freshness_bounded_timeouts=" + json.dumps({
        "reason": bounded["reason"],
        "execution_time_budget_sec": bounded[
            "execution_time_budget_sec"
        ],
        "max_costmap_timeout_sec": max(FAKE.costmap_timeouts),
        "max_make_plan_timeout_sec": max(FAKE.plan_timeouts),
        "goal_publish_count": FAKE.goal_publish_call_count,
    }, sort_keys=True))


def run():
    run_rejection_tests()
    run_third_validation_failures()
    run_positive_and_replay_test()
    run_concurrency_test()
    run_cleanup_failure_test()
    run_navigation_outcome_tests()
    run_publish_failure_test()
    run_execution_freshness_tests()
    reset_all()
    print("stage2_unit_test: PASS")


if __name__ == "__main__":
    run()
