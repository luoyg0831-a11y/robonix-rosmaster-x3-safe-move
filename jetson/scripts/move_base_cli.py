#!/usr/bin/env python3
"""Deterministic, guarded short-distance move workflow for ROSMASTER X3."""

from __future__ import annotations

import argparse
import json
import logging
import math
import os
import re
import subprocess
import sys
import unicodedata
from contextlib import contextmanager
from typing import Callable, Optional


CONFIRMATION_PHRASE = "确认执行"
_ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
_DEPENDENCY_LOGGERS = ("roslibpy", "twisted", "autobahn")


def configure_dependency_logging() -> None:
    """Keep routine transport lifecycle messages out of the user-facing CLI."""
    for logger_name in _DEPENDENCY_LOGGERS:
        logging.getLogger(logger_name).setLevel(logging.WARNING)


def normalize_terminal_input(value: str) -> str:
    """Normalize full-width input and remove terminal paste control codes."""
    normalized = unicodedata.normalize("NFKC", str(value))
    normalized = _ANSI_ESCAPE_RE.sub("", normalized)
    return "".join(
        char
        for char in normalized
        if not unicodedata.category(char).startswith("C")
    ).strip()


def _result_json(response) -> dict:
    return json.loads(response.result_json.data)


class ProviderApi:
    """Call the provider functions in one process so sessions stay private."""

    def __init__(self):
        from rosmaster_x3_bridge import main as provider_main
        import rosmaster_x3_bridge_mcp as mcp

        self._provider = provider_main
        self._mcp = mcp

    def preview(self, max_options: int) -> dict:
        return _result_json(self._provider.preview_move_options(
            self._mcp.PreviewMoveOptions_Request(max_options=max_options)
        ))

    def prepare(self, session_id: str, option_id: int) -> dict:
        return _result_json(self._provider.prepare_selected_move(
            self._mcp.PrepareSelectedMove_Request(
                session_id=session_id,
                option_id=option_id,
            )
        ))

    def execute(self, token: str, confirmation_text: str) -> dict:
        return _result_json(self._provider.execute_prepared_navigation(
            self._mcp.ExecutePreparedNavigation_Request(
                token=token,
                allow_execute=True,
                confirmation_text=confirmation_text,
            )
        ))

    def cancel(self) -> Optional[dict]:
        try:
            return _result_json(self._provider.cancel_nav_goal(
                self._mcp.CancelNavGoal_Request()
            ))
        except Exception:
            return None

    @staticmethod
    def hard_estop() -> bool:
        try:
            completed = subprocess.run(
                [
                    "bash",
                    "/home/jetson/x3_rosbridge_adapter/scripts/estop_hard.sh",
                ],
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=25,
                check=False,
            )
            return completed.returncode == 0
        except Exception:
            return False

    def close(self) -> None:
        bridge = getattr(self._provider, "bridge", None)
        if bridge is not None:
            try:
                bridge.close()
            finally:
                self._provider.bridge = None


@contextmanager
def prepared_execution_gate():
    """Open only the dedicated prepared-navigation gate in this process."""
    names = (
        "X3_ALLOW_PREPARED_NAV_EXECUTION",
        "X3_PREPARED_NAV_EXECUTION_BUDGET_SEC",
        "X3_PREPARED_NAV_RESULT_TIMEOUT_SEC",
        "X3_ALLOW_NAV_GOAL",
        "X3_MAX_NAV_GOAL_DISTANCE_M",
    )
    previous = {name: os.environ.get(name) for name in names}
    os.environ["X3_ALLOW_PREPARED_NAV_EXECUTION"] = "1"
    os.environ["X3_PREPARED_NAV_EXECUTION_BUDGET_SEC"] = "20"
    os.environ["X3_PREPARED_NAV_RESULT_TIMEOUT_SEC"] = "15"
    os.environ.pop("X3_ALLOW_NAV_GOAL", None)
    os.environ.pop("X3_MAX_NAV_GOAL_DISTANCE_M", None)
    try:
        yield
    finally:
        for name, value in previous.items():
            if value is None:
                os.environ.pop(name, None)
            else:
                os.environ[name] = value


def _finite_float(value, default: float = 0.0) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def _print_options(preview: dict, write: Callable[[str], None]) -> None:
    write("安全导航点：")
    for option in preview.get("options", []):
        write(
            "  {id}. {direction}  坐标=({x:.3f}, {y:.3f})  "
            "距离={distance:.2f}m  代价={cost}  路径点={poses}".format(
                id=option.get("option_id"),
                direction=option.get("direction_label", "方向待确认"),
                x=_finite_float(option.get("x")),
                y=_finite_float(option.get("y")),
                distance=_finite_float(option.get("radius_m")),
                cost=option.get("window_max_cost", "?"),
                poses=option.get("path_pose_count", "?"),
            )
        )


def _print_prepared(prepared: dict, write: Callable[[str], None]) -> None:
    target = prepared.get("target") or {}
    current_pose = prepared.get("current_pose") or {}
    costmap = prepared.get("costmap") or {}
    stats = costmap.get("stats") or {}
    plan = prepared.get("plan") or {}
    distance = target.get("distance_from_current_m")
    if distance is None and current_pose:
        distance = math.hypot(
            _finite_float(target.get("x"))
            - _finite_float(current_pose.get("x")),
            _finite_float(target.get("y"))
            - _finite_float(current_pose.get("y")),
        )
    write("已完成第二次安全校验：")
    write(
        "  目标坐标=({x:.3f}, {y:.3f})  距离={distance:.2f}m".format(
            x=_finite_float(target.get("x")),
            y=_finite_float(target.get("y")),
            distance=_finite_float(distance),
        )
    )
    write(
        "  目标代价={goal_cost}  窗口最大代价={max_cost}  路径点={poses}".format(
            goal_cost=stats.get("goal_cost", "?"),
            max_cost=stats.get("max_cost", "?"),
            poses=plan.get("plan_pose_count", "?"),
        )
    )


def _print_result(result: dict, write: Callable[[str], None]) -> None:
    status_name = result.get("move_base_status_name")
    status_text = result.get("move_base_status_text")
    cmd = result.get("cmd_vel_observation") or {}
    amcl = result.get("amcl_observation") or {}
    odom = result.get("odom_observation") or {}
    displacement = amcl.get("displacement_from_validation_m")
    if displacement is None:
        displacement = amcl.get("observer_displacement_m")

    if result.get("ok") is True and result.get("arrived") is True:
        write("导航完成：move_base 已报告 SUCCEEDED。")
    else:
        write("导航未成功：{}。".format(result.get("reason", "unknown")))
    write(
        "  状态={}  非零速度消息={}  AMCL位移={:.3f}m".format(
            status_name or "待确认",
            cmd.get("nonzero_messages", 0),
            _finite_float(displacement),
        )
    )
    if status_text:
        write("  move_base: {}".format(status_text))
    write(
        "  里程计位移={:.3f}m  路程={:.3f}m".format(
            _finite_float(odom.get("displacement_m")),
            _finite_float(odom.get("path_length_m")),
        )
    )
    if result.get("safety_watchdog_triggered"):
        write("  安全看门狗：{}".format(
            result.get("safety_watchdog_reason", "unknown")
        ))


def run_workflow(
    api,
    max_options: int = 5,
    dry_run: bool = False,
    read_input: Callable[[str], str] = input,
    write: Callable[[str], None] = print,
) -> int:
    """Run one preview-select-execute workflow with an internal token gate."""
    write("正在扫描安全导航点，最多返回 {} 个...".format(max_options))
    preview = api.preview(max_options)
    if preview.get("ok") is not True:
        write("扫描失败：{}。".format(preview.get("reason", "unknown")))
        return 2

    options = preview.get("options") or []
    if not options:
        write("当前没有通过全部安全校验的导航点，小车不会移动。")
        return 3
    _print_options(preview, write)

    valid_ids = {int(option["option_id"]) for option in options}
    while True:
        selection = normalize_terminal_input(
            read_input("请输入导航点编号并执行，或输入 q 取消：")
        )
        if selection.lower() == "q":
            write("已取消，小车不会移动。")
            return 0
        if selection.isascii() and selection.isdigit():
            option_id = int(selection)
            if option_id in valid_ids:
                break
        write("编号无效，请输入列表中的编号。")

    prepared = api.prepare(preview["session_id"], option_id)
    if prepared.get("ok") is not True:
        write("目标准备失败：{}。小车不会移动。".format(
            prepared.get("reason", "unknown")
        ))
        return 4
    _print_prepared(prepared, write)

    if dry_run:
        write("无运动检查完成，未发送导航目标。")
        return 0

    write("编号已确认，正在进行第三次安全校验并等待导航结果...")
    execution_started = False
    try:
        with prepared_execution_gate():
            execution_started = True
            result = api.execute(
                prepared["token"],
                prepared["confirmation_text"],
            )
    except KeyboardInterrupt:
        if execution_started:
            api.cancel()
        write("执行被中断，已请求取消当前导航目标。")
        return 130

    _print_result(result, write)
    if result.get("reason") in {
        "goal_acceptance_timeout",
        "navigation_result_timeout",
        "goal_publish_outcome_unknown",
        "navigation_safety_watchdog",
    }:
        api.cancel()
        write("结果存在不确定性，已请求取消目标；禁止重试本次 token。")
        if result.get("safety_cancel_terminal_observed") is not True:
            hard_estop_ok = api.hard_estop()
            write("硬急停={}。".format(
                "已完成" if hard_estop_ok else "结果待确认"
            ))
    return 0 if result.get("ok") is True else 5


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="ROSMASTER X3 安全短距离导航"
    )
    parser.add_argument(
        "--max-options",
        type=int,
        default=5,
        choices=range(1, 6),
        metavar="1..5",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="完成候选和选择校验，但不允许执行移动",
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    configure_dependency_logging()
    api = ProviderApi()
    try:
        return run_workflow(
            api,
            max_options=args.max_options,
            dry_run=args.dry_run,
        )
    except (EOFError, KeyboardInterrupt):
        print("已取消，小车不会移动。")
        return 130
    finally:
        api.close()


if __name__ == "__main__":
    sys.exit(main())
