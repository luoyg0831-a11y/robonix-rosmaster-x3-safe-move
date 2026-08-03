#!/usr/bin/env python3
"""ROSMASTER X3 — Robonix primitive provider.

First version: read-only status query through ROS1 rosbridge + roslibpy.

Safety policy:
- This provider does NOT publish /cmd_vel.
- Prepared navigation execution is one-time, independently gated, and
  revalidated immediately before publishing a PoseStamped goal.
- All other preview and preparation capabilities remain motion-free.
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
import os
import secrets
import threading
import time
import sys
import traceback
from pathlib import Path

from robonix_api import Primitive, Ok

# Make existing adapter importable.
ADAPTER_SCRIPTS = Path("/home/jetson/x3_rosbridge_adapter/scripts")
if str(ADAPTER_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(ADAPTER_SCRIPTS))

from x3_bridge import X3Bridge  # noqa: E402

import rosmaster_x3_bridge_mcp  # noqa: E402
import std_msgs_mcp  # noqa: E402


provider = Primitive(
    id="rosmaster_x3_bridge",
    namespace="robonix/primitive/rosmaster_x3_bridge",
)

bridge: X3Bridge | None = None
bridge_cfg: dict = {
    "rosbridge_host": "127.0.0.1",
    "rosbridge_port": 9090,
}


# Short-lived preparation tokens are intentionally process-local.
# Restarting the provider invalidates every outstanding token.
_PREPARED_NAV_TOKENS: dict[str, dict] = {}
_PREPARED_NAV_LOCK = threading.Lock()
_PREPARED_NAV_TOKEN_LIMIT = 32

# Short-lived move-option sessions are also intentionally process-local.
# Lock order, when both locks are needed, is always:
# _MOVE_OPTION_SESSIONS_LOCK -> _PREPARED_NAV_LOCK.
_MOVE_OPTION_SESSIONS: dict[str, dict] = {}
_MOVE_OPTION_SESSIONS_LOCK = threading.Lock()
_MOVE_OPTION_SESSION_LIMIT = 16

_AMCL_MAX_MESSAGE_AGE_SEC = 2.0
_AMCL_MAX_POSITION_VARIANCE_M2 = 0.01
_AMCL_MAX_YAW_VARIANCE_RAD2 = 0.10
# Preserve 0.05 m of motion evidence with the 0.03 m goal tolerance,
# while leaving convergence margin inside the 0.10 m path watchdog.
_MOVE_OPTION_OPERATIONAL_MAX_RADIUS_M = 0.08


def ensure_bridge() -> X3Bridge:
    """Create ROS1 rosbridge connection on first tool call."""
    global bridge

    if bridge is not None and bridge.ros.is_connected:
        return bridge

    host = bridge_cfg.get("rosbridge_host", "127.0.0.1")
    port = int(bridge_cfg.get("rosbridge_port", 9090))

    bridge = X3Bridge(host=host, port=port)
    bridge.connect()
    return bridge


@provider.on_init
def init(cfg: dict):
    """Store config. Actual rosbridge connection is created lazily."""
    global bridge_cfg
    bridge_cfg.update(cfg or {})
    return Ok()


@provider.mcp("robonix/primitive/rosmaster_x3_bridge/get_status")
def get_status(
    msg: rosmaster_x3_bridge_mcp.GetX3Status_Request,
) -> rosmaster_x3_bridge_mcp.GetX3Status_Response:
    """Read ROSMASTER X3 status through ROS1 rosbridge.

    Returns JSON containing:
    - rosbridge connected state
    - odom
    - scan summary
    - AMCL pose
    - move_base status

    This is read-only and must not move the robot.
    """
    _ = msg

    try:
        b = ensure_bridge()
        payload = {
            "ok": True,
            "rosbridge_connected": bool(b.ros.is_connected),
            "odom": b.get_odom(),
            "scan": b.get_scan_summary(),
            "amcl": b.get_amcl_pose(),
            "move_base_status": b.get_move_base_status(),
        }
    except Exception as exc:
        payload = {
            "ok": False,
            "error": str(exc),
            "traceback": traceback.format_exc(),
        }

    return rosmaster_x3_bridge_mcp.GetX3Status_Response(
        status_json=std_msgs_mcp.String(
            data=json.dumps(payload, ensure_ascii=False)
        )
    )


@provider.mcp("robonix/primitive/rosmaster_x3_bridge/check_nav_ready")
def check_nav_ready(
    msg: rosmaster_x3_bridge_mcp.CheckNavReady_Request,
) -> rosmaster_x3_bridge_mcp.CheckNavReady_Response:
    """Read-only check of ROS navigation prerequisites.

    Checks:
    - rosbridge connection
    - /odom
    - /scan
    - /amcl_pose
    - /move_base/status
    - /cmd_vel idle state
    - /move_base/make_plan path test
    - TF map -> odom -> base_footprint
    - global and local costmap validity

    Safety:
    - Does not publish /cmd_vel.
    - Does not send navigation goals.
    - Does not call X3Bridge.stop() or X3Bridge.close().
    """
    _ = msg

    checks = {}
    payload = {
        "ok": False,
        "ready": False,
        "checks": checks,
        "safety": {
            "read_only": True,
            "raw_cmd_vel": "not used",
            "navigation_goal": "not sent",
        },
        "limitations": [
            (
                "costmap structure is checked, but current robot-footprint "
                "clearance is not yet evaluated"
            ),
        ],
    }

    try:
        b = ensure_bridge()
        connected = bool(b.ros.is_connected)
        checks["rosbridge"] = {
            "ok": connected,
            "connected": connected,
        }
    except Exception as exc:
        checks["rosbridge"] = {
            "ok": False,
            "error": str(exc),
        }
        payload["message"] = "navigation is not ready because rosbridge connection failed"

        return rosmaster_x3_bridge_mcp.CheckNavReady_Response(
            result_json=std_msgs_mcp.String(
                data=json.dumps(payload, ensure_ascii=False)
            )
        )

    def run_check(name, func, validator=None):
        try:
            data = func()
            valid = True if validator is None else bool(validator(data))
            checks[name] = {
                "ok": valid,
                "data": data,
            }
            if not valid:
                checks[name]["error"] = "data received but validation failed"
        except Exception as exc:
            checks[name] = {
                "ok": False,
                "error": str(exc),
            }

    run_check(
        "odom",
        b.get_odom,
        lambda data: "x" in data and "y" in data,
    )

    run_check(
        "scan",
        b.get_scan_summary,
        lambda data: int(data.get("valid_ranges", 0)) > 0,
    )

    run_check(
        "amcl",
        b.get_amcl_pose,
        lambda data: (
            data.get("frame_id") == "map"
            and "x" in data
            and "y" in data
        ),
    )

    # An empty or terminal status is allowed. A pending/active transition is
    # not ready because a new simple goal would preempt existing navigation.
    run_check(
        "move_base_status",
        b.get_move_base_status,
        lambda data: (
            isinstance(data, dict)
            and data.get("active") is not True
        ),
    )

    # Read-only observation of /cmd_vel.
    # No message during the observation window means that no active
    # velocity command was detected.
    run_check(
        "cmd_vel_idle",
        b.get_cmd_vel_state,
        lambda data: data.get("idle") is True,
    )

    # Read-only global and local costmap validation.
    # Checks metadata and OccupancyGrid data dimensions.
    run_check(
        "costmap_status",
        b.get_costmap_status,
        lambda data: (
            data.get("ok") is True
            and data.get(
                "global_costmap",
                {},
            ).get("ok") is True
            and data.get(
                "local_costmap",
                {},
            ).get("ok") is True
        ),
    )

    # Read-only TF chain check.
    # Verifies that map can reach base_footprint through the TF tree.
    run_check(
        "tf_chain_status",
        b.get_tf_chain_status,
        lambda data: (
            data.get("chain_ok") is True
            and data.get("chain") == [
                "map",
                "odom",
                "base_footprint",
            ]
        ),
    )

    # Read-only make_plan test using a nearby 5 cm target.
    # It requests a path but does not submit a navigation goal.
    run_check(
        "make_plan_status",
        b.get_make_plan_status,
        lambda data: (
            data.get("service_present") is True
            and data.get("type_match") is True
            and data.get("call_ok") is True
            and data.get("path_available") is True
            and int(data.get("plan_pose_count", 0)) > 0
        ),
    )

    ready = all(item.get("ok", False) for item in checks.values())

    payload["ok"] = ready
    payload["ready"] = ready

    if ready:
        payload["message"] = "ROS navigation prerequisites are ready"
    else:
        payload["message"] = "one or more ROS navigation prerequisites failed"

    return rosmaster_x3_bridge_mcp.CheckNavReady_Response(
        result_json=std_msgs_mcp.String(
            data=json.dumps(payload, ensure_ascii=False)
        )
    )


@provider.mcp(
    "robonix/primitive/rosmaster_x3_bridge/preview_safe_waypoint"
)
def preview_safe_waypoint(
    msg: rosmaster_x3_bridge_mcp.PreviewSafeWaypoint_Request,
) -> rosmaster_x3_bridge_mcp.PreviewSafeWaypoint_Response:
    """Preview nearby safe waypoint candidates without moving the robot.

    The tool reads AMCL and the global costmap, evaluates nearby candidate
    cells, and calls /move_base/make_plan for candidates that pass costmap
    checks. It never sends a navigation goal and never publishes /cmd_vel.
    """
    payload = {
        "ok": False,
        "preview_only": True,
        "safe_candidate_found": False,
        "navigation_goal_sent": False,
        "cmd_vel_published": False,
        "waypoint_file_modified": False,
        "safety": {
            "read_only": True,
            "global_costmap_read_only": True,
            "make_plan_only": True,
            "raw_cmd_vel": "not used",
            "navigation_goal": "not sent",
            "waypoint_file": "not modified",
        },
    }

    try:
        min_radius = float(msg.min_radius)
        max_radius = float(msg.max_radius)
        step_radius = float(msg.step_radius)
        angles = int(msg.angles)
        window = float(msg.window)
        goal_cost_limit = int(msg.goal_cost_limit)
        plan_tolerance = float(msg.plan_tolerance)
        strict_high_cost = bool(msg.strict_high_cost)

        numeric_values = (
            min_radius,
            max_radius,
            step_radius,
            window,
            plan_tolerance,
        )

        if not all(math.isfinite(value) for value in numeric_values):
            raise ValueError("all floating-point parameters must be finite")

        if min_radius <= 0.0:
            raise ValueError("min_radius must be greater than zero")

        if max_radius < min_radius:
            raise ValueError(
                "max_radius must be greater than or equal to min_radius"
            )

        if max_radius > 1.0:
            raise ValueError(
                "max_radius must not exceed 1.0 metre for nearby preview"
            )

        if step_radius <= 0.0:
            raise ValueError("step_radius must be greater than zero")

        if angles < 4 or angles > 72:
            raise ValueError("angles must be between 4 and 72")

        if window <= 0.0 or window > 0.5:
            raise ValueError(
                "window must be greater than zero and no more than 0.5 metre"
            )

        if goal_cost_limit < 1 or goal_cost_limit > 101:
            raise ValueError(
                "goal_cost_limit must be between 1 and 101"
            )

        if plan_tolerance < 0.0 or plan_tolerance > 1.0:
            raise ValueError(
                "plan_tolerance must be between 0.0 and 1.0 metre"
            )

        radii = []
        radius = min_radius

        while radius <= max_radius + 1e-9:
            radii.append(round(radius, 6))
            radius += step_radius

            if len(radii) > 100:
                raise ValueError("too many radius samples")

        candidate_total = len(radii) * angles

        if candidate_total > 720:
            raise ValueError(
                "candidate count exceeds the safety limit of 720"
            )

        parameters = {
            "min_radius": min_radius,
            "max_radius": max_radius,
            "step_radius": step_radius,
            "angles": angles,
            "window": window,
            "goal_cost_limit": goal_cost_limit,
            "plan_tolerance": plan_tolerance,
            "strict_high_cost": strict_high_cost,
            "radii": radii,
            "candidate_total": candidate_total,
        }
        payload["parameters"] = parameters

        b = ensure_bridge()

        amcl = b.get_amcl_pose()
        raw_pose = amcl["raw_pose"]

        current_x = float(raw_pose["position"]["x"])
        current_y = float(raw_pose["position"]["y"])

        orientation = raw_pose.get("orientation", {})
        qx = float(orientation.get("x", 0.0) or 0.0)
        qy = float(orientation.get("y", 0.0) or 0.0)
        qz = float(orientation.get("z", 0.0) or 0.0)
        qw = float(orientation.get("w", 1.0) or 1.0)

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)
        current_yaw_deg = math.degrees(current_yaw)

        payload["current_pose"] = {
            "frame_id": amcl.get("frame_id", "map"),
            "x": current_x,
            "y": current_y,
            "yaw_deg": current_yaw_deg,
            "orientation": {
                "x": qx,
                "y": qy,
                "z": qz,
                "w": qw,
            },
        }

        grid = b.get_global_costmap_grid(timeout=8.0)

        if grid.get("ok") is not True:
            raise RuntimeError("global costmap validation failed")

        if grid.get("frame_id") != "map":
            raise RuntimeError("global costmap frame is not map")

        width = int(grid.get("width", 0))
        height = int(grid.get("height", 0))
        resolution = float(grid.get("resolution", 0.0))
        origin_x = float(grid.get("origin_x", 0.0))
        origin_y = float(grid.get("origin_y", 0.0))
        data = grid.get("data", [])

        if width <= 0 or height <= 0 or resolution <= 0.0:
            raise RuntimeError("global costmap metadata is invalid")

        if len(data) != width * height:
            raise RuntimeError("global costmap data length mismatch")

        payload["costmap"] = {
            "ok": True,
            "topic": grid.get("topic"),
            "frame_id": grid.get("frame_id"),
            "resolution": resolution,
            "width": width,
            "height": height,
            "width_m": width * resolution,
            "height_m": height * resolution,
            "origin_x": origin_x,
            "origin_y": origin_y,
            "origin_orientation": grid.get(
                "origin_orientation",
                {},
            ),
            "expected_cells": width * height,
            "data_length": len(data),
            "size_match": len(data) == width * height,
        }

        def world_to_map(x, y):
            mx = int((x - origin_x) / resolution)
            my = int((y - origin_y) / resolution)

            if (
                mx < 0
                or my < 0
                or mx >= width
                or my >= height
            ):
                return None

            return mx, my

        def cell_value(mx, my):
            return int(data[my * width + mx])

        def window_stats(x, y):
            position = world_to_map(x, y)

            if position is None:
                return {
                    "inside": False,
                    "goal_cost": None,
                    "unknown": 999999,
                    "lethal_ge100": 999999,
                    "high_ge80": 999999,
                    "max_cost": 999999,
                }

            mx, my = position
            radius_cells = max(
                1,
                int(math.ceil(window / resolution)),
            )

            unknown = 0
            lethal = 0
            high = 0
            max_cost = -999

            for yy in range(
                my - radius_cells,
                my + radius_cells + 1,
            ):
                for xx in range(
                    mx - radius_cells,
                    mx + radius_cells + 1,
                ):
                    if (
                        xx < 0
                        or yy < 0
                        or xx >= width
                        or yy >= height
                    ):
                        unknown += 1
                        continue

                    value = cell_value(xx, yy)

                    if value < 0:
                        unknown += 1

                    if value >= 100:
                        lethal += 1

                    if value >= 80:
                        high += 1

                    if value > max_cost:
                        max_cost = value

            return {
                "inside": True,
                "goal_cost": cell_value(mx, my),
                "unknown": unknown,
                "lethal_ge100": lethal,
                "high_ge80": high,
                "max_cost": max_cost,
                "map_x": mx,
                "map_y": my,
                "window_radius_cells": radius_cells,
            }

        failures = {
            "outside_costmap": 0,
            "goal_cost_high": 0,
            "unknown_window": 0,
            "lethal_window": 0,
            "make_plan_empty": 0,
            "make_plan_error": 0,
        }

        passed = []
        make_plan_calls = 0
        make_plan_call_ok = 0
        make_plan_paths_available = 0
        make_plan_pose_count_total = 0

        for radius in radii:
            for index in range(angles):
                relative_angle = (
                    -math.pi
                    + (
                        2.0
                        * math.pi
                        * float(index)
                        / float(angles)
                    )
                )
                direction = current_yaw + relative_angle

                goal_x = (
                    current_x
                    + radius * math.cos(direction)
                )
                goal_y = (
                    current_y
                    + radius * math.sin(direction)
                )
                goal_yaw_deg = current_yaw_deg

                stats = window_stats(goal_x, goal_y)

                if not stats["inside"]:
                    failures["outside_costmap"] += 1
                    continue

                if (
                    stats["goal_cost"] is None
                    or stats["goal_cost"] >= goal_cost_limit
                ):
                    failures["goal_cost_high"] += 1
                    continue

                if stats["unknown"] > 0:
                    failures["unknown_window"] += 1
                    continue

                if stats["lethal_ge100"] > 0:
                    failures["lethal_window"] += 1
                    continue

                if (
                    strict_high_cost
                    and stats["high_ge80"] > 0
                ):
                    failures["goal_cost_high"] += 1
                    continue

                make_plan_calls += 1

                try:
                    plan = b.make_plan_to_pose(
                        goal_x=goal_x,
                        goal_y=goal_y,
                        goal_yaw_deg=goal_yaw_deg,
                        tolerance=plan_tolerance,
                        timeout=8.0,
                    )
                except Exception:
                    failures["make_plan_error"] += 1
                    continue

                if plan.get("call_ok") is not True:
                    failures["make_plan_error"] += 1
                    continue

                make_plan_call_ok += 1
                plan_pose_count = int(
                    plan.get("plan_pose_count", 0)
                )
                make_plan_pose_count_total += plan_pose_count

                if (
                    plan.get("path_available") is not True
                    or plan_pose_count <= 0
                ):
                    failures["make_plan_empty"] += 1
                    continue

                make_plan_paths_available += 1

                front_penalty = abs(relative_angle)
                radius_penalty = abs(radius - 0.25)

                score = (
                    stats["goal_cost"] * 1.0
                    + stats["max_cost"] * 0.2
                    + front_penalty * 5.0
                    + radius_penalty * 20.0
                    - min(plan_pose_count, 50) * 0.05
                )

                passed.append({
                    "x": goal_x,
                    "y": goal_y,
                    "yaw_deg": goal_yaw_deg,
                    "radius": radius,
                    "relative_angle_deg": math.degrees(
                        relative_angle
                    ),
                    "goal_cost": stats["goal_cost"],
                    "max_cost": stats["max_cost"],
                    "unknown": stats["unknown"],
                    "lethal_ge100": stats["lethal_ge100"],
                    "high_ge80": stats["high_ge80"],
                    "map_x": stats["map_x"],
                    "map_y": stats["map_y"],
                    "window_radius_cells": stats[
                        "window_radius_cells"
                    ],
                    "plan_pose_count": plan_pose_count,
                    "plan_last_x": plan.get("plan_last_x"),
                    "plan_last_y": plan.get("plan_last_y"),
                    "score": score,
                })

        passed.sort(key=lambda candidate: candidate["score"])

        payload["failures"] = failures
        payload["candidate_summary"] = {
            "candidate_total": candidate_total,
            "candidate_passed": len(passed),
            "candidate_failed": candidate_total - len(passed),
            "make_plan_calls": make_plan_calls,
            "make_plan_call_ok": make_plan_call_ok,
            "make_plan_paths_available": (
                make_plan_paths_available
            ),
            "make_plan_pose_count_total": (
                make_plan_pose_count_total
            ),
        }
        payload["candidates"] = passed

        if passed:
            payload["ok"] = True
            payload["safe_candidate_found"] = True
            payload["best_candidate"] = passed[0]
            payload["message"] = (
                "safe waypoint candidates found; preview only"
            )
        else:
            payload["best_candidate"] = None
            payload["message"] = (
                "no safe waypoint candidate found; "
                "do not execute navigation"
            )

    except Exception as exc:
        payload["ok"] = False
        payload["safe_candidate_found"] = False
        payload["error"] = str(exc)
        payload["traceback"] = traceback.format_exc()
        payload["message"] = (
            "safe waypoint preview failed; "
            "do not execute navigation"
        )

    return (
        rosmaster_x3_bridge_mcp.PreviewSafeWaypoint_Response(
            result_json=std_msgs_mcp.String(
                data=json.dumps(
                    payload,
                    ensure_ascii=False,
                )
            )
        )
    )


def _bounded_env_float(
    name: str,
    default: float,
    minimum: float,
    maximum: float,
) -> float:
    """Read and clamp a finite floating-point environment value."""
    raw_value = os.environ.get(name)

    if raw_value is None:
        return default

    try:
        value = float(raw_value)
    except (TypeError, ValueError):
        return default

    if not math.isfinite(value):
        return default

    return min(max(value, minimum), maximum)


def _prepare_safe_navigation_response(
    payload: dict,
) -> rosmaster_x3_bridge_mcp.PrepareSafeNavigation_Response:
    """Create the generated MCP response object."""
    return (
        rosmaster_x3_bridge_mcp.PrepareSafeNavigation_Response(
            result_json=std_msgs_mcp.String(
                data=json.dumps(
                    payload,
                    ensure_ascii=False,
                )
            )
        )
    )


def _purge_prepared_navigation_tokens_locked(
    now_monotonic: float,
) -> None:
    """Remove expired tokens while the preparation lock is held."""
    expired_tokens = [
        token
        for token, record in _PREPARED_NAV_TOKENS.items()
        if float(record.get("expires_monotonic", 0.0))
        <= now_monotonic
        and record.get("executing") is not True
    ]

    for token in expired_tokens:
        _PREPARED_NAV_TOKENS.pop(token, None)

    while len(_PREPARED_NAV_TOKENS) >= _PREPARED_NAV_TOKEN_LIMIT:
        evictable_tokens = [
            token
            for token, record in _PREPARED_NAV_TOKENS.items()
            if record.get("executing") is not True
        ]
        if not evictable_tokens:
            break
        oldest_token = min(
            evictable_tokens,
            key=lambda item: float(
                _PREPARED_NAV_TOKENS[item].get(
                    "created_monotonic",
                    0.0,
                )
            ),
        )
        _PREPARED_NAV_TOKENS.pop(oldest_token, None)


@provider.mcp(
    "robonix/primitive/rosmaster_x3_bridge/prepare_safe_navigation"
)
def prepare_safe_navigation(
    msg: rosmaster_x3_bridge_mcp.PrepareSafeNavigation_Request,
) -> rosmaster_x3_bridge_mcp.PrepareSafeNavigation_Response:
    """Revalidate a target and issue a short-lived preparation token.

    Safety:
    - This capability is read-only with respect to robot motion.
    - It never publishes /cmd_vel.
    - It never publishes /move_base_simple/goal.
    - It does not write the waypoint JSON file.
    - strict_high_cost must be true before a token can be issued.
    """
    payload = {
        "ok": False,
        "prepared": False,
        "validation_passed": False,
        "preview_only": True,
        "token_issued": False,
        "navigation_goal_sent": False,
        "cmd_vel_published": False,
        "waypoint_file_modified": False,
        "safety": {
            "read_only": True,
            "global_costmap_read_only": True,
            "make_plan_only": True,
            "strict_high_cost_required": True,
            "raw_cmd_vel": "not used",
            "navigation_goal": "not sent",
            "waypoint_file": "not modified",
            "token_storage": "process memory only",
        },
    }

    try:
        target_x = float(msg.x)
        target_y = float(msg.y)
        requested_yaw_deg = float(msg.yaw_deg)
        window = float(msg.window)
        goal_cost_limit = int(msg.goal_cost_limit)
        plan_tolerance = float(msg.plan_tolerance)
        strict_high_cost = bool(msg.strict_high_cost)

        floating_values = (
            target_x,
            target_y,
            requested_yaw_deg,
            window,
            plan_tolerance,
        )

        if not all(
            math.isfinite(value)
            for value in floating_values
        ):
            raise ValueError(
                "all floating-point parameters must be finite"
            )

        if window < 0.05 or window > 0.5:
            raise ValueError(
                "window must be between 0.05 and 0.5 metre"
            )

        if goal_cost_limit < 1 or goal_cost_limit > 80:
            raise ValueError(
                "goal_cost_limit must be between 1 and 80"
            )

        if plan_tolerance < 0.0 or plan_tolerance > 0.10:
            raise ValueError(
                "plan_tolerance must be between 0.0 and 0.10 metre"
            )

        normalized_yaw_deg = (
            (requested_yaw_deg + 180.0) % 360.0
        ) - 180.0

        max_prepare_distance_m = _bounded_env_float(
            "X3_MAX_PREPARE_DISTANCE_M",
            default=0.10,
            minimum=0.05,
            maximum=0.10,
        )

        token_ttl_sec = _bounded_env_float(
            "X3_PREPARE_TOKEN_TTL_SEC",
            default=300.0,
            minimum=60.0,
            maximum=600.0,
        )

        request_parameters = {
            "x": target_x,
            "y": target_y,
            "yaw_deg": normalized_yaw_deg,
            "window": window,
            "goal_cost_limit": goal_cost_limit,
            "plan_tolerance": plan_tolerance,
            "strict_high_cost": strict_high_cost,
            "max_prepare_distance_m": (
                max_prepare_distance_m
            ),
            "plan_timeout_sec": 8.0,
            "token_ttl_sec": token_ttl_sec,
        }

        payload["parameters"] = request_parameters

        if not strict_high_cost:
            payload["message"] = (
                "preparation blocked: "
                "strict_high_cost must be true"
            )
            payload["gates"] = {
                "strict_high_cost_required": False,
            }
            return _prepare_safe_navigation_response(
                payload
            )

        b = ensure_bridge()

        current_pose = _fresh_amcl_pose_snapshot(b)
        current_x = current_pose["x"]
        current_y = current_pose["y"]
        current_yaw_deg = current_pose["yaw_deg"]

        distance_from_current_m = math.hypot(
            target_x - current_x,
            target_y - current_y,
        )

        payload["current_pose"] = copy.deepcopy(current_pose)

        payload["target"] = {
            "frame_id": "map",
            "x": target_x,
            "y": target_y,
            "yaw_deg": normalized_yaw_deg,
            "distance_from_current_m": (
                distance_from_current_m
            ),
        }

        gates = {
            "strict_high_cost_required": True,
            "distance_within_limit": (
                distance_from_current_m
                <= max_prepare_distance_m + 1e-9
            ),
        }

        payload["gates"] = gates

        if not gates["distance_within_limit"]:
            payload["message"] = (
                "preparation blocked: target is farther "
                "than max_prepare_distance_m"
            )
            return _prepare_safe_navigation_response(
                payload
            )

        grid = b.get_global_costmap_grid(
            timeout=8.0
        )

        if grid.get("ok") is not True:
            raise RuntimeError(
                "global costmap validation failed"
            )

        if grid.get("frame_id") != "map":
            raise RuntimeError(
                "global costmap frame is not map"
            )

        width = int(grid.get("width", 0))
        height = int(grid.get("height", 0))
        resolution = float(
            grid.get("resolution", 0.0)
        )
        origin_x = float(
            grid.get("origin_x", 0.0)
        )
        origin_y = float(
            grid.get("origin_y", 0.0)
        )
        data = grid.get("data", [])

        if (
            width <= 0
            or height <= 0
            or resolution <= 0.0
        ):
            raise RuntimeError(
                "global costmap metadata is invalid"
            )

        if len(data) != width * height:
            raise RuntimeError(
                "global costmap data length mismatch"
            )

        map_x = int(
            math.floor(
                (target_x - origin_x)
                / resolution
            )
        )
        map_y = int(
            math.floor(
                (target_y - origin_y)
                / resolution
            )
        )

        inside_costmap = (
            0 <= map_x < width
            and 0 <= map_y < height
        )

        stats = {
            "inside": inside_costmap,
            "goal_cost": None,
            "unknown": 0,
            "lethal_ge100": 0,
            "high_ge80": 0,
            "max_cost": None,
            "map_x": map_x,
            "map_y": map_y,
            "window_radius_cells": None,
        }

        if inside_costmap:
            radius_cells = max(
                1,
                int(
                    math.ceil(
                        window / resolution
                    )
                ),
            )

            unknown = 0
            lethal = 0
            high = 0
            max_cost = -999

            for yy in range(
                map_y - radius_cells,
                map_y + radius_cells + 1,
            ):
                for xx in range(
                    map_x - radius_cells,
                    map_x + radius_cells + 1,
                ):
                    if (
                        xx < 0
                        or yy < 0
                        or xx >= width
                        or yy >= height
                    ):
                        unknown += 1
                        continue

                    value = int(
                        data[yy * width + xx]
                    )

                    if value < 0:
                        unknown += 1

                    if value >= 100:
                        lethal += 1

                    if value >= 80:
                        high += 1

                    if value > max_cost:
                        max_cost = value

            stats.update({
                "goal_cost": int(
                    data[map_y * width + map_x]
                ),
                "unknown": unknown,
                "lethal_ge100": lethal,
                "high_ge80": high,
                "max_cost": max_cost,
                "window_radius_cells": (
                    radius_cells
                ),
            })

        payload["costmap"] = {
            "topic": grid.get("topic"),
            "frame_id": grid.get("frame_id"),
            "resolution": resolution,
            "width": width,
            "height": height,
            "origin_x": origin_x,
            "origin_y": origin_y,
            "stats": stats,
        }

        goal_cost = stats.get("goal_cost")

        gates.update({
            "inside_costmap": inside_costmap,
            "goal_cost_below_limit": (
                goal_cost is not None
                and goal_cost
                < goal_cost_limit
            ),
            "unknown_window_clear": (
                stats["unknown"] == 0
            ),
            "lethal_window_clear": (
                stats["lethal_ge100"] == 0
            ),
            "high_cost_window_clear": (
                stats["high_ge80"] == 0
            ),
        })

        costmap_gate_names = (
            "inside_costmap",
            "goal_cost_below_limit",
            "unknown_window_clear",
            "lethal_window_clear",
            "high_cost_window_clear",
        )

        if not all(
            gates[name]
            for name in costmap_gate_names
        ):
            payload["message"] = (
                "preparation blocked: "
                "target failed strict costmap validation"
            )
            return _prepare_safe_navigation_response(
                payload
            )

        plan = b.make_plan_to_pose(
            goal_x=target_x,
            goal_y=target_y,
            goal_yaw_deg=normalized_yaw_deg,
            tolerance=plan_tolerance,
            timeout=8.0,
        )

        plan_pose_count = int(
            plan.get("plan_pose_count", 0)
        )

        plan_last_x = plan.get("plan_last_x")
        plan_last_y = plan.get("plan_last_y")

        terminal_error_m = None

        if (
            plan_last_x is not None
            and plan_last_y is not None
        ):
            terminal_error_m = math.hypot(
                float(plan_last_x) - target_x,
                float(plan_last_y) - target_y,
            )

        terminal_error_limit_m = max(
            plan_tolerance,
            resolution * 2.0,
        )

        gates.update({
            "make_plan_call_ok": (
                plan.get("call_ok") is True
            ),
            "make_plan_path_available": (
                plan.get("path_available") is True
                and plan_pose_count > 0
            ),
            "plan_terminal_near_target": (
                terminal_error_m is not None
                and terminal_error_m
                <= terminal_error_limit_m
            ),
        })

        payload["plan"] = {
            "service_name": plan.get(
                "service_name"
            ),
            "call_ok": plan.get("call_ok"),
            "path_available": plan.get(
                "path_available"
            ),
            "plan_pose_count": plan_pose_count,
            "plan_last_x": plan_last_x,
            "plan_last_y": plan_last_y,
            "terminal_error_m": terminal_error_m,
            "terminal_error_limit_m": (
                terminal_error_limit_m
            ),
        }

        if not all(gates.values()):
            payload["message"] = (
                "preparation blocked: "
                "make_plan validation failed"
            )
            return _prepare_safe_navigation_response(
                payload
            )

        digest_source = {
            "target": payload["target"],
            "parameters": request_parameters,
            "costmap_stats": stats,
            "plan": payload["plan"],
        }

        validation_digest = hashlib.sha256(
            json.dumps(
                digest_source,
                ensure_ascii=False,
                sort_keys=True,
                separators=(",", ":"),
            ).encode("utf-8")
        ).hexdigest()

        token = secrets.token_urlsafe(24)

        now_epoch = time.time()
        now_monotonic = time.monotonic()

        expires_epoch = (
            now_epoch + token_ttl_sec
        )
        expires_monotonic = (
            now_monotonic + token_ttl_sec
        )

        token_record = {
            "token": token,
            "created_epoch": now_epoch,
            "created_monotonic": now_monotonic,
            "expires_epoch": expires_epoch,
            "expires_monotonic": expires_monotonic,
            "source": "prepare_safe_navigation",
            "execution_authorized": False,
            "executing": False,
            "used": False,
            "execution_claimed_epoch": None,
            "execution_claimed_monotonic": None,
            "execution_claim_id": None,
            "execution_deadline_monotonic": None,
            "used_epoch": None,
            "used_monotonic": None,
            "used_reason": None,
            "goal_publish_attempted": False,
            "goal_publish_completed": False,
            "goal_publish_call_returned": False,
            "goal_publish_outcome_unknown": False,
            "goal_topic_cleanup_attempted": False,
            "goal_topic_cleanup_completed": None,
            "goal_topic_cleanup_error_type": None,
            "target": {
                "frame_id": "map",
                "x": target_x,
                "y": target_y,
                "yaw_deg": normalized_yaw_deg,
            },
            "parameters": request_parameters,
            "validation_digest": validation_digest,
        }

        with _PREPARED_NAV_LOCK:
            _purge_prepared_navigation_tokens_locked(
                now_monotonic
            )
            _PREPARED_NAV_TOKENS[token] = (
                token_record
            )
            active_token_count = len(
                _PREPARED_NAV_TOKENS
            )

        payload["ok"] = True
        payload["prepared"] = True
        payload["validation_passed"] = True
        payload["token_issued"] = True
        payload["confirmation"] = {
            "token": token,
            "one_time": True,
            "issued_at_epoch": now_epoch,
            "expires_at_epoch": expires_epoch,
            "expires_in_sec": token_ttl_sec,
            "validation_digest": (
                validation_digest
            ),
            "active_token_count": (
                active_token_count
            ),
            "provider_restart_invalidates_token": (
                True
            ),
        }
        payload["message"] = (
            "candidate revalidated; "
            "short-lived one-time token issued; "
            "navigation has not been executed"
        )

    except Exception as exc:
        payload["ok"] = False
        payload["prepared"] = False
        payload["validation_passed"] = False
        payload["token_issued"] = False
        payload["error"] = str(exc)
        payload["traceback"] = (
            traceback.format_exc()
        )
        payload["message"] = (
            "safe navigation preparation failed; "
            "do not execute navigation"
        )

    return _prepare_safe_navigation_response(
        payload
    )


def _bounded_env_int(
    name: str,
    default: int,
    minimum: int,
    maximum: int,
) -> int:
    """Read and clamp a finite integer environment value."""
    raw_value = os.environ.get(name)

    if raw_value is None:
        return default

    try:
        value = int(raw_value)
    except (TypeError, ValueError):
        return default

    return min(max(value, minimum), maximum)


def _move_options_config() -> dict:
    """Return conservative server-controlled move-option settings."""
    max_prepare_distance_m = _bounded_env_float(
        "X3_MAX_PREPARE_DISTANCE_M",
        default=0.10,
        minimum=0.05,
        maximum=0.10,
    )
    hard_max_radius_m = min(
        _MOVE_OPTION_OPERATIONAL_MAX_RADIUS_M,
        max_prepare_distance_m,
    )
    min_radius_m = _bounded_env_float(
        "X3_MOVE_OPTIONS_MIN_RADIUS_M",
        default=hard_max_radius_m,
        minimum=0.05,
        maximum=hard_max_radius_m,
    )
    max_radius_m = _bounded_env_float(
        "X3_MOVE_OPTIONS_MAX_RADIUS_M",
        default=hard_max_radius_m,
        minimum=0.05,
        maximum=hard_max_radius_m,
    )

    if min_radius_m > max_radius_m:
        raise ValueError(
            "X3_MOVE_OPTIONS_MIN_RADIUS_M must not exceed "
            "X3_MOVE_OPTIONS_MAX_RADIUS_M"
        )

    return {
        "min_radius_m": min_radius_m,
        "max_radius_m": max_radius_m,
        "radius_step_m": _bounded_env_float(
            "X3_MOVE_OPTIONS_RADIUS_STEP_M",
            default=0.01,
            minimum=0.01,
            maximum=0.05,
        ),
        "angle_step_deg": _bounded_env_int(
            "X3_MOVE_OPTIONS_ANGLE_STEP_DEG",
            default=15,
            minimum=15,
            maximum=90,
        ),
        "max_options": _bounded_env_int(
            "X3_MOVE_OPTIONS_MAX_COUNT",
            default=5,
            minimum=1,
            maximum=5,
        ),
        "session_ttl_sec": _bounded_env_float(
            "X3_MOVE_OPTIONS_SESSION_TTL_SEC",
            default=60.0,
            minimum=30.0,
            maximum=120.0,
        ),
        "min_angle_separation_deg": _bounded_env_float(
            "X3_MOVE_OPTIONS_MIN_ANGLE_SEPARATION_DEG",
            default=30.0,
            minimum=15.0,
            maximum=90.0,
        ),
        "origin_translation_limit_m": _bounded_env_float(
            "X3_MOVE_OPTIONS_ORIGIN_TRANSLATION_LIMIT_M",
            default=0.03,
            minimum=0.02,
            maximum=0.03,
        ),
        # These match the frozen strict prepare baseline and cannot be
        # changed by a caller. A lower goal-cost environment override is
        # allowed, but the upper bound cannot exceed the existing limit.
        "window_m": 0.05,
        "goal_cost_limit": _bounded_env_int(
            "X3_MOVE_OPTIONS_GOAL_COST_LIMIT",
            default=80,
            minimum=1,
            maximum=80,
        ),
        "plan_tolerance_m": 0.02,
        "plan_timeout_sec": _bounded_env_float(
            "X3_MOVE_OPTIONS_PLAN_TIMEOUT_SEC",
            default=2.0,
            minimum=0.5,
            maximum=5.0,
        ),
        "max_plan_checks": _bounded_env_int(
            "X3_MOVE_OPTIONS_MAX_PLAN_CHECKS",
            default=16,
            minimum=5,
            maximum=24,
        ),
        "preview_time_budget_sec": _bounded_env_float(
            "X3_MOVE_OPTIONS_PREVIEW_TIME_BUDGET_SEC",
            default=20.0,
            minimum=5.0,
            maximum=30.0,
        ),
        "strict_high_cost": True,
        "max_prepare_distance_m": max_prepare_distance_m,
    }


def _purge_move_option_sessions_locked(
    now_monotonic: float,
) -> None:
    """Remove expired sessions while the session lock is held."""
    expired_session_ids = [
        session_id
        for session_id, record in _MOVE_OPTION_SESSIONS.items()
        if float(record.get("expires_monotonic", 0.0))
        <= now_monotonic
    ]

    for session_id in expired_session_ids:
        _MOVE_OPTION_SESSIONS.pop(session_id, None)

    while len(_MOVE_OPTION_SESSIONS) >= _MOVE_OPTION_SESSION_LIMIT:
        oldest_session_id = min(
            _MOVE_OPTION_SESSIONS,
            key=lambda item: float(
                _MOVE_OPTION_SESSIONS[item].get(
                    "created_monotonic",
                    0.0,
                )
            ),
        )
        _MOVE_OPTION_SESSIONS.pop(oldest_session_id, None)


def _amcl_pose_snapshot(amcl: dict, require_quality: bool = False) -> dict:
    """Validate AMCL data and return a finite map-frame pose."""
    frame_id = str(amcl.get("frame_id", ""))
    if frame_id != "map":
        raise RuntimeError("AMCL frame is not map")

    raw_pose = amcl.get("raw_pose", {})
    position = raw_pose.get("position", {})
    orientation = raw_pose.get("orientation", {})

    x = float(position.get("x"))
    y = float(position.get("y"))
    qx = float(orientation.get("x", 0.0) or 0.0)
    qy = float(orientation.get("y", 0.0) or 0.0)
    qz = float(orientation.get("z", 0.0) or 0.0)
    qw = float(orientation.get("w", 1.0))

    if not all(
        math.isfinite(value)
        for value in (x, y, qx, qy, qz, qw)
    ):
        raise RuntimeError("AMCL pose contains non-finite values")

    quaternion_norm = math.sqrt(
        qx * qx + qy * qy + qz * qz + qw * qw
    )
    if (
        not math.isfinite(quaternion_norm)
        or quaternion_norm <= 1e-6
    ):
        raise RuntimeError("AMCL orientation quaternion is invalid")

    qx /= quaternion_norm
    qy /= quaternion_norm
    qz /= quaternion_norm
    qw /= quaternion_norm

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw_rad = math.atan2(siny_cosp, cosy_cosp)

    quality = {
        "freshness_verified": bool(amcl.get("freshness_verified")),
        "stamp_sec": amcl.get("stamp_sec"),
        "message_age_sec": amcl.get("message_age_sec"),
        "position_variance_max": amcl.get("position_variance_max"),
        "yaw_variance": amcl.get("yaw_variance"),
        "max_message_age_sec": _AMCL_MAX_MESSAGE_AGE_SEC,
        "max_position_variance_m2": _AMCL_MAX_POSITION_VARIANCE_M2,
        "max_yaw_variance_rad2": _AMCL_MAX_YAW_VARIANCE_RAD2,
    }
    if require_quality:
        try:
            message_age_sec = float(quality["message_age_sec"])
            position_variance_max = float(
                quality["position_variance_max"]
            )
            yaw_variance = float(quality["yaw_variance"])
        except (TypeError, ValueError):
            raise RuntimeError("AMCL quality data is missing")
        if quality["freshness_verified"] is not True:
            raise RuntimeError("AMCL freshness was not verified")
        if not all(math.isfinite(value) for value in (
            message_age_sec,
            position_variance_max,
            yaw_variance,
        )):
            raise RuntimeError("AMCL quality data is non-finite")
        if message_age_sec > _AMCL_MAX_MESSAGE_AGE_SEC:
            raise RuntimeError("AMCL pose is stale")
        if position_variance_max > _AMCL_MAX_POSITION_VARIANCE_M2:
            raise RuntimeError("AMCL position covariance is too high")
        if yaw_variance > _AMCL_MAX_YAW_VARIANCE_RAD2:
            raise RuntimeError("AMCL yaw covariance is too high")

    return {
        "frame_id": "map",
        "x": x,
        "y": y,
        "yaw_deg": math.degrees(yaw_rad),
        "yaw_rad": yaw_rad,
        "quaternion_norm_input": quaternion_norm,
        "quality": quality,
        "orientation": {
            "x": qx,
            "y": qy,
            "z": qz,
            "w": qw,
        },
    }


def _fresh_amcl_pose_snapshot(b: X3Bridge) -> dict:
    """Request a no-motion AMCL update and enforce localization quality."""
    reader = getattr(b, "get_fresh_amcl_pose", None)
    if not callable(reader):
        # Offline fakes from the frozen Stage 1/2 suites do not implement the
        # ROS service. Production X3Bridge always takes the strict branch.
        return _amcl_pose_snapshot(b.get_amcl_pose())
    return _amcl_pose_snapshot(reader(timeout=5.0), require_quality=True)


def _global_costmap_context(grid: dict) -> dict:
    """Validate a complete global OccupancyGrid for strict checks."""
    if grid.get("ok") is not True:
        raise RuntimeError("global costmap validation failed")

    if grid.get("frame_id") != "map":
        raise RuntimeError("global costmap frame is not map")

    width = int(grid.get("width", 0))
    height = int(grid.get("height", 0))
    resolution = float(grid.get("resolution", 0.0))
    origin_x = float(grid.get("origin_x", 0.0))
    origin_y = float(grid.get("origin_y", 0.0))
    data = grid.get("data", [])

    if width <= 0 or height <= 0 or resolution <= 0.0:
        raise RuntimeError("global costmap metadata is invalid")

    if not all(
        math.isfinite(value)
        for value in (resolution, origin_x, origin_y)
    ):
        raise RuntimeError("global costmap metadata is non-finite")

    if len(data) != width * height:
        raise RuntimeError("global costmap data length mismatch")

    return {
        "topic": grid.get("topic"),
        "frame_id": "map",
        "width": width,
        "height": height,
        "resolution": resolution,
        "origin_x": origin_x,
        "origin_y": origin_y,
        "data": data,
    }


def _strict_costmap_window_stats(
    context: dict,
    target_x: float,
    target_y: float,
    window_m: float,
) -> dict:
    """Apply the frozen prepare-style strict target-window check."""
    width = context["width"]
    height = context["height"]
    resolution = context["resolution"]
    origin_x = context["origin_x"]
    origin_y = context["origin_y"]
    data = context["data"]

    map_x = int(math.floor((target_x - origin_x) / resolution))
    map_y = int(math.floor((target_y - origin_y) / resolution))
    inside = 0 <= map_x < width and 0 <= map_y < height

    stats = {
        "inside": inside,
        "goal_cost": None,
        "unknown": 0,
        "lethal_ge100": 0,
        "high_ge80": 0,
        "max_cost": None,
        "map_x": map_x,
        "map_y": map_y,
        "window_radius_cells": None,
    }

    if not inside:
        return stats

    radius_cells = max(
        1,
        int(math.ceil(window_m / resolution)),
    )
    unknown = 0
    lethal = 0
    high = 0
    max_cost = -999

    for yy in range(map_y - radius_cells, map_y + radius_cells + 1):
        for xx in range(map_x - radius_cells, map_x + radius_cells + 1):
            if xx < 0 or yy < 0 or xx >= width or yy >= height:
                unknown += 1
                continue

            value = int(data[yy * width + xx])
            if value < 0:
                unknown += 1
            if value >= 100:
                lethal += 1
            if value >= 80:
                high += 1
            if value > max_cost:
                max_cost = value

    stats.update({
        "goal_cost": int(data[map_y * width + map_x]),
        "unknown": unknown,
        "lethal_ge100": lethal,
        "high_ge80": high,
        "max_cost": max_cost,
        "window_radius_cells": radius_cells,
    })
    return stats


def _validate_move_option_costmap(
    target: dict,
    parameters: dict,
    current_pose: dict,
    costmap_context: dict,
) -> dict:
    """Apply finite, distance, and strict costmap gates without planning."""
    target_x = float(target["x"])
    target_y = float(target["y"])
    target_yaw_deg = float(target["yaw_deg"])

    if target.get("frame_id") != "map":
        return {"ok": False, "reason": "target_frame_invalid"}

    if not all(
        math.isfinite(value)
        for value in (target_x, target_y, target_yaw_deg)
    ):
        return {"ok": False, "reason": "target_non_finite"}

    distance_m = math.hypot(
        target_x - current_pose["x"],
        target_y - current_pose["y"],
    )
    if distance_m > float(parameters["max_prepare_distance_m"]) + 1e-9:
        return {"ok": False, "reason": "target_too_far"}

    stats = _strict_costmap_window_stats(
        costmap_context,
        target_x,
        target_y,
        float(parameters["window_m"]),
    )
    goal_cost = stats.get("goal_cost")
    costmap_ok = (
        stats["inside"] is True
        and goal_cost is not None
        and goal_cost < int(parameters["goal_cost_limit"])
        and stats["unknown"] == 0
        and stats["lethal_ge100"] == 0
        and stats["high_ge80"] == 0
        and parameters["strict_high_cost"] is True
    )
    if not costmap_ok:
        return {
            "ok": False,
            "reason": "costmap_check_failed",
            "distance_m": distance_m,
            "costmap_stats": stats,
        }

    return {
        "ok": True,
        "reason": "costmap_validated",
        "distance_m": distance_m,
        "costmap_stats": stats,
    }


def _validate_move_option_plan(
    b: X3Bridge,
    target: dict,
    parameters: dict,
    costmap_context: dict,
    costmap_validation: dict,
) -> dict:
    """Apply the bounded make_plan gate to one costmap-safe target."""
    target_x = float(target["x"])
    target_y = float(target["y"])
    target_yaw_deg = float(target["yaw_deg"])
    plan_timeout_sec = float(parameters["plan_timeout_sec"])

    try:
        plan = b.make_plan_to_pose(
            goal_x=target_x,
            goal_y=target_y,
            goal_yaw_deg=target_yaw_deg,
            tolerance=float(parameters["plan_tolerance_m"]),
            timeout=plan_timeout_sec,
        )
    except Exception:
        return {
            "ok": False,
            "reason": "make_plan_failed",
            "distance_m": costmap_validation["distance_m"],
            "costmap_stats": costmap_validation["costmap_stats"],
            "plan_timeout_sec": plan_timeout_sec,
        }

    plan_pose_count = int(plan.get("plan_pose_count", 0))
    plan_last_x = plan.get("plan_last_x")
    plan_last_y = plan.get("plan_last_y")
    terminal_error_m = None

    if plan_last_x is not None and plan_last_y is not None:
        terminal_error_m = math.hypot(
            float(plan_last_x) - target_x,
            float(plan_last_y) - target_y,
        )

    terminal_error_limit_m = max(
        float(parameters["plan_tolerance_m"]),
        float(costmap_context["resolution"]) * 2.0,
    )
    make_plan_ok = (
        plan.get("call_ok") is True
        and plan.get("path_available") is True
        and plan_pose_count > 0
        and terminal_error_m is not None
        and terminal_error_m <= terminal_error_limit_m
    )

    plan_evidence = {
        "service_name": plan.get("service_name"),
        "call_ok": plan.get("call_ok"),
        "path_available": plan.get("path_available"),
        "plan_pose_count": plan_pose_count,
        "plan_last_x": plan_last_x,
        "plan_last_y": plan_last_y,
        "terminal_error_m": terminal_error_m,
        "terminal_error_limit_m": terminal_error_limit_m,
        "timeout_sec": plan_timeout_sec,
    }

    if not make_plan_ok:
        return {
            "ok": False,
            "reason": "make_plan_failed",
            "distance_m": costmap_validation["distance_m"],
            "costmap_stats": costmap_validation["costmap_stats"],
            "plan": plan_evidence,
        }

    return {
        "ok": True,
        "reason": "validated",
        "distance_m": costmap_validation["distance_m"],
        "costmap_stats": costmap_validation["costmap_stats"],
        "plan": plan_evidence,
    }


def _direction_label(relative_angle_deg: float) -> str:
    """Return an eight-sector display label; never a safety signal."""
    angle = ((relative_angle_deg + 180.0) % 360.0) - 180.0
    magnitude = abs(angle)

    if magnitude < 22.5:
        return "正前方"
    if magnitude < 67.5:
        return "左前方" if angle > 0.0 else "右前方"
    if magnitude < 112.5:
        return "左侧" if angle > 0.0 else "右侧"
    if magnitude < 157.5:
        return "左后方" if angle > 0.0 else "右后方"
    return "正后方"


def _circular_angle_distance_deg(a: float, b: float) -> float:
    """Return the shortest absolute distance between two angles."""
    return abs(((a - b + 180.0) % 360.0) - 180.0)


def _navigation_ready_snapshot() -> dict:
    """Reuse the existing read-only navigation readiness capability."""
    request = rosmaster_x3_bridge_mcp.CheckNavReady_Request()
    response = check_nav_ready(request)
    return json.loads(response.result_json.data)


def _preview_move_options_response(
    payload: dict,
) -> rosmaster_x3_bridge_mcp.PreviewMoveOptions_Response:
    """Create the generated preview-move-options response."""
    return rosmaster_x3_bridge_mcp.PreviewMoveOptions_Response(
        result_json=std_msgs_mcp.String(
            data=json.dumps(payload, ensure_ascii=False)
        )
    )


@provider.mcp(
    "robonix/primitive/rosmaster_x3_bridge/preview_move_options"
)
def preview_move_options(
    msg: rosmaster_x3_bridge_mcp.PreviewMoveOptions_Request,
) -> rosmaster_x3_bridge_mcp.PreviewMoveOptions_Response:
    """Return up to five strict nearby options with bounded planning."""
    preview_started_monotonic = time.monotonic()
    payload = {
        "ok": False,
        "reason": "internal_error",
        "preview_only": True,
        "session_created": False,
        "session_id": None,
        "session_ttl_sec": None,
        "expires_epoch": None,
        "option_count": 0,
        "max_options": 5,
        "navigation_goal_sent": False,
        "direct_cmd_vel_published": False,
        "waypoint_file_modified": False,
        "direction_description_reliable": False,
        "current_pose": None,
        "search": {},
        "candidate_stats": {},
        "options": [],
        "preview_elapsed_sec": 0.0,
        "preview_time_budget_sec": None,
        "preview_time_budget_reached": False,
        "message": "move-option preview failed safely",
    }

    def finish_response():
        elapsed_sec = max(
            0.0,
            time.monotonic() - preview_started_monotonic,
        )
        payload["preview_elapsed_sec"] = elapsed_sec
        time_budget_sec = payload.get("preview_time_budget_sec")
        if (
            time_budget_sec is not None
            and elapsed_sec >= float(time_budget_sec)
        ):
            payload["preview_time_budget_reached"] = True
        return _preview_move_options_response(payload)

    try:
        requested_max_options = int(msg.max_options)
        if requested_max_options < 1 or requested_max_options > 5:
            payload["reason"] = "invalid_max_options"
            payload["message"] = "max_options must be between 1 and 5"
            return finish_response()

        config = _move_options_config()
        preview_time_budget_sec = float(
            config["preview_time_budget_sec"]
        )
        preview_deadline_monotonic = (
            preview_started_monotonic + preview_time_budget_sec
        )
        payload["preview_time_budget_sec"] = preview_time_budget_sec
        effective_max_options = min(
            requested_max_options,
            int(config["max_options"]),
            5,
        )
        payload["max_options"] = effective_max_options
        payload["session_ttl_sec"] = config["session_ttl_sec"]

        try:
            ready_snapshot = _navigation_ready_snapshot()
        except Exception:
            ready_snapshot = {"ok": False, "ready": False}

        payload["navigation_readiness"] = ready_snapshot
        if (
            ready_snapshot.get("ok") is not True
            or ready_snapshot.get("ready") is not True
        ):
            payload["reason"] = "nav_not_ready"
            payload["message"] = "navigation prerequisites are not ready"
            return finish_response()

        b = ensure_bridge()

        try:
            current_pose = _fresh_amcl_pose_snapshot(b)
        except Exception:
            payload["reason"] = "amcl_unavailable"
            payload["message"] = "AMCL pose is unavailable or invalid"
            return finish_response()

        payload["current_pose"] = current_pose

        remaining_preview_sec = (
            preview_deadline_monotonic - time.monotonic()
        )
        if remaining_preview_sec <= 0.0:
            payload["preview_time_budget_reached"] = True
            payload["reason"] = "no_safe_options"
            payload["message"] = (
                "preview time budget was reached before costmap validation; "
                "the robot will not move"
            )
            return finish_response()

        try:
            grid = b.get_global_costmap_grid(
                timeout=min(8.0, remaining_preview_sec)
            )
            costmap_context = _global_costmap_context(grid)
        except Exception:
            payload["reason"] = "costmap_unavailable"
            payload["message"] = "global costmap is unavailable or invalid"
            return finish_response()

        radii = []
        radius = float(config["min_radius_m"])
        while radius <= float(config["max_radius_m"]) + 1e-9:
            radii.append(round(radius, 6))
            radius += float(config["radius_step_m"])

        relative_angles_deg = []
        raw_angle = 0
        while raw_angle < 360:
            relative_angles_deg.append(
                ((float(raw_angle) + 180.0) % 360.0) - 180.0
            )
            raw_angle += int(config["angle_step_deg"])

        search_parameters = {
            **config,
            "radii_m": radii,
            "relative_angles_deg": relative_angles_deg,
            "candidate_total": len(radii) * len(relative_angles_deg),
            "requested_max_options": requested_max_options,
            "effective_max_options": effective_max_options,
        }
        payload["search"] = copy.deepcopy(search_parameters)

        failures = {
            "target_frame_invalid": 0,
            "target_non_finite": 0,
            "target_too_far": 0,
            "costmap_check_failed": 0,
            "make_plan_failed": 0,
        }
        costmap_candidates = []
        geometry_processed = 0

        for radius_m in radii:
            for relative_angle_deg in relative_angles_deg:
                if time.monotonic() >= preview_deadline_monotonic:
                    payload["preview_time_budget_reached"] = True
                    break

                geometry_processed += 1
                direction_rad = current_pose["yaw_rad"] + math.radians(
                    relative_angle_deg
                )
                target = {
                    "frame_id": "map",
                    "x": current_pose["x"]
                    + radius_m * math.cos(direction_rad),
                    "y": current_pose["y"]
                    + radius_m * math.sin(direction_rad),
                    "yaw_deg": current_pose["yaw_deg"],
                }
                costmap_validation = _validate_move_option_costmap(
                    target,
                    config,
                    current_pose,
                    costmap_context,
                )

                if costmap_validation.get("ok") is not True:
                    reason = costmap_validation.get(
                        "reason",
                        "costmap_check_failed",
                    )
                    failures[reason] = failures.get(reason, 0) + 1
                    continue

                stats = costmap_validation["costmap_stats"]
                # This deterministic score ranks only candidates that already
                # passed every strict costmap gate. Planning happens later and
                # is bounded independently.
                costmap_rank_score = (
                    float(stats["max_cost"]) * 1000.0
                    + float(stats["goal_cost"]) * 100.0
                    + abs(
                        radius_m - _MOVE_OPTION_OPERATIONAL_MAX_RADIUS_M
                    ) * 10.0
                    + abs(relative_angle_deg) * 0.0001
                )

                costmap_candidates.append({
                    "target": target,
                    "radius_m": radius_m,
                    "relative_angle_deg": relative_angle_deg,
                    "costmap_validation": costmap_validation,
                    "costmap_rank_score": costmap_rank_score,
                })

            if payload["preview_time_budget_reached"]:
                break

        costmap_candidates.sort(
            key=lambda candidate: (
                candidate["costmap_rank_score"],
                candidate["radius_m"],
                abs(candidate["relative_angle_deg"]),
                candidate["relative_angle_deg"],
                candidate["target"]["x"],
                candidate["target"]["y"],
            )
        )

        selected = []
        plan_checked = 0
        plan_passed = 0
        plan_check_limit = int(config["max_plan_checks"])
        plan_check_limit_reached = False
        minimum_separation = float(
            config["min_angle_separation_deg"]
        )

        for candidate in costmap_candidates:
            if len(selected) >= effective_max_options:
                break

            if time.monotonic() >= preview_deadline_monotonic:
                payload["preview_time_budget_reached"] = True
                break

            if plan_checked >= plan_check_limit:
                plan_check_limit_reached = True
                break

            remaining_preview_sec = (
                preview_deadline_monotonic - time.monotonic()
            )
            if remaining_preview_sec <= 0.0:
                payload["preview_time_budget_reached"] = True
                break

            plan_checked += 1
            plan_parameters = dict(config)
            plan_parameters["plan_timeout_sec"] = min(
                float(config["plan_timeout_sec"]),
                remaining_preview_sec,
            )
            plan_validation = _validate_move_option_plan(
                b,
                candidate["target"],
                plan_parameters,
                costmap_context,
                candidate["costmap_validation"],
            )
            if plan_validation.get("ok") is not True:
                failures["make_plan_failed"] += 1
                continue

            plan_passed += 1
            target = candidate["target"]
            stats = plan_validation["costmap_stats"]
            plan = plan_validation["plan"]
            plan_pose_count = int(plan["plan_pose_count"])
            score = (
                candidate["costmap_rank_score"]
                + min(plan_pose_count, 1000) * 0.01
            )
            digest_source = {
                "target": target,
                "radius_m": candidate["radius_m"],
                "relative_angle_deg": candidate[
                    "relative_angle_deg"
                ],
                "costmap_stats": stats,
                "plan": plan,
                "search_parameters": search_parameters,
            }
            validation_digest = hashlib.sha256(
                json.dumps(
                    digest_source,
                    ensure_ascii=False,
                    sort_keys=True,
                    separators=(",", ":"),
                ).encode("utf-8")
            ).hexdigest()
            option = {
                "frame_id": "map",
                "x": target["x"],
                "y": target["y"],
                "yaw_deg": target["yaw_deg"],
                "radius_m": candidate["radius_m"],
                "relative_angle_deg": candidate[
                    "relative_angle_deg"
                ],
                "direction_label": _direction_label(
                    candidate["relative_angle_deg"]
                ),
                "direction_description_reliable": False,
                "goal_cost": stats["goal_cost"],
                "window_max_cost": stats["max_cost"],
                "unknown_count": stats["unknown"],
                "lethal_count": stats["lethal_ge100"],
                "high_cost_count": stats["high_ge80"],
                "make_plan_ok": True,
                "path_pose_count": plan_pose_count,
                "plan_terminal_error_m": plan[
                    "terminal_error_m"
                ],
                "obstacle_margin_proxy": max(
                    0,
                    79 - int(stats["max_cost"]),
                ),
                "score": score,
                "validation_digest": validation_digest,
            }

            if all(
                _circular_angle_distance_deg(
                    option["relative_angle_deg"],
                    existing["relative_angle_deg"],
                )
                >= minimum_separation
                for existing in selected
            ):
                selected.append(option)

        for option_id, option in enumerate(selected, start=1):
            option["option_id"] = option_id

        payload["candidate_stats"] = {
            "candidate_total": search_parameters["candidate_total"],
            "strict_passed": plan_passed,
            "geometry_candidate_total": search_parameters[
                "candidate_total"
            ],
            "geometry_processed": geometry_processed,
            "costmap_passed": len(costmap_candidates),
            "plan_checked": plan_checked,
            "plan_passed": plan_passed,
            "plan_check_limit": plan_check_limit,
            "plan_check_limit_reached": plan_check_limit_reached,
            "selected_after_diversity": len(selected),
            "failures": failures,
        }
        payload["options"] = copy.deepcopy(selected)
        payload["option_count"] = len(selected)

        if not selected:
            payload["reason"] = "no_safe_options"
            payload["message"] = (
                "no safe move options passed every strict gate; "
                "the robot will not move"
            )
            return finish_response()

        session_id = secrets.token_urlsafe(24)
        now_epoch = time.time()
        now_monotonic = time.monotonic()
        expires_epoch = now_epoch + float(config["session_ttl_sec"])
        expires_monotonic = (
            now_monotonic + float(config["session_ttl_sec"])
        )
        session_record = {
            "session_id": session_id,
            "created_epoch": now_epoch,
            "created_monotonic": now_monotonic,
            "expires_epoch": expires_epoch,
            "expires_monotonic": expires_monotonic,
            "used": False,
            "selected": False,
            "selection_in_progress": False,
            "invalidated": False,
            "origin_pose": copy.deepcopy(current_pose),
            "search_parameters": copy.deepcopy(search_parameters),
            "options": copy.deepcopy(selected),
        }

        with _MOVE_OPTION_SESSIONS_LOCK:
            _purge_move_option_sessions_locked(now_monotonic)
            _MOVE_OPTION_SESSIONS[session_id] = session_record

        payload.update({
            "ok": True,
            "reason": "session_created",
            "session_created": True,
            "session_id": session_id,
            "expires_epoch": expires_epoch,
            "message": (
                "safe move options are ready for numbered selection; "
                "no navigation goal was sent"
            ),
        })

    except Exception as exc:
        payload["ok"] = False
        payload["reason"] = "internal_error"
        payload["error_type"] = type(exc).__name__
        payload["message"] = "move-option preview failed safely"

    return finish_response()


def _prepare_selected_move_response(
    payload: dict,
) -> rosmaster_x3_bridge_mcp.PrepareSelectedMove_Response:
    """Create the generated selected-move preparation response."""
    return rosmaster_x3_bridge_mcp.PrepareSelectedMove_Response(
        result_json=std_msgs_mcp.String(
            data=json.dumps(payload, ensure_ascii=False)
        )
    )


def _invalidate_move_option_session(
    session_id: str,
    reason: str,
) -> None:
    """Atomically invalidate a failed selection session."""
    with _MOVE_OPTION_SESSIONS_LOCK:
        record = _MOVE_OPTION_SESSIONS.get(session_id)
        if record is not None and record.get("used") is not True:
            record["selection_in_progress"] = False
            record["invalidated"] = True
            record["invalidated_reason"] = reason
            record["invalidated_epoch"] = time.time()


def _prepare_failure_reason(prepare_result: dict) -> str:
    """Map the existing preparation result to a stable selected reason."""
    text = " ".join([
        str(prepare_result.get("message", "")),
        str(prepare_result.get("error", "")),
    ]).lower()

    if "amcl" in text or "current pose" in text:
        return "amcl_check_failed"
    if "costmap" in text:
        return "costmap_check_failed"
    if "make_plan" in text or "path" in text:
        return "make_plan_failed"
    return "selected_target_invalidated"


@provider.mcp(
    "robonix/primitive/rosmaster_x3_bridge/prepare_selected_move"
)
def prepare_selected_move(
    msg: rosmaster_x3_bridge_mcp.PrepareSelectedMove_Request,
) -> rosmaster_x3_bridge_mcp.PrepareSelectedMove_Response:
    """Revalidate one internal session option and issue a prepared token."""
    payload = {
        "ok": False,
        "reason": "internal_error",
        "prepared": False,
        "validation_passed": False,
        "session_consumed": False,
        "selected_option_id": None,
        "token_issued": False,
        "token": None,
        "one_time": True,
        "confirmation_text": None,
        "navigation_goal_sent": False,
        "direct_cmd_vel_published": False,
        "waypoint_file_modified": False,
        "target": None,
        "current_pose": None,
        "costmap": None,
        "plan": None,
        "message": "selected move preparation failed safely",
    }
    claimed = False
    issued_token = None

    try:
        session_id = str(msg.session_id).strip()
        option_id = int(msg.option_id)

        if not session_id:
            payload["reason"] = "session_not_found"
            payload["message"] = "move-option session was not found"
            return _prepare_selected_move_response(payload)

        now_monotonic = time.monotonic()
        with _MOVE_OPTION_SESSIONS_LOCK:
            session = _MOVE_OPTION_SESSIONS.get(session_id)
            if session is None:
                payload["reason"] = "session_not_found"
                payload["message"] = "move-option session was not found"
                return _prepare_selected_move_response(payload)

            if float(session["expires_monotonic"]) <= now_monotonic:
                _MOVE_OPTION_SESSIONS.pop(session_id, None)
                payload["reason"] = "session_expired"
                payload["message"] = "move-option session has expired"
                return _prepare_selected_move_response(payload)

            if session.get("used") is True:
                payload["reason"] = "session_already_used"
                payload["message"] = "move-option session was already used"
                return _prepare_selected_move_response(payload)

            if session.get("selection_in_progress") is True:
                payload["reason"] = "session_selection_in_progress"
                payload["message"] = "another selection is in progress"
                return _prepare_selected_move_response(payload)

            if session.get("invalidated") is True:
                payload["reason"] = "selected_target_invalidated"
                payload["message"] = (
                    "move-option session was invalidated; request new options"
                )
                return _prepare_selected_move_response(payload)

            option = next(
                (
                    item
                    for item in session.get("options", [])
                    if int(item.get("option_id", -1)) == option_id
                ),
                None,
            )
            if option is None:
                payload["reason"] = "invalid_option_id"
                payload["message"] = "option_id is not in this session"
                return _prepare_selected_move_response(payload)

            session["selection_in_progress"] = True
            claimed = True
            session_snapshot = copy.deepcopy(session)
            option_snapshot = copy.deepcopy(option)

        b = ensure_bridge()
        try:
            latest_pose = _fresh_amcl_pose_snapshot(b)
        except Exception:
            _invalidate_move_option_session(
                session_id,
                "amcl_check_failed",
            )
            payload["reason"] = "amcl_check_failed"
            payload["message"] = "second AMCL validation failed"
            return _prepare_selected_move_response(payload)

        origin_pose = session_snapshot["origin_pose"]
        translation_from_origin_m = math.hypot(
            latest_pose["x"] - float(origin_pose["x"]),
            latest_pose["y"] - float(origin_pose["y"]),
        )
        translation_limit_m = float(
            session_snapshot["search_parameters"][
                "origin_translation_limit_m"
            ]
        )
        payload["origin_translation_m"] = translation_from_origin_m
        payload["origin_translation_limit_m"] = translation_limit_m

        if translation_from_origin_m > translation_limit_m:
            _invalidate_move_option_session(
                session_id,
                "robot_pose_changed",
            )
            payload["reason"] = "robot_pose_changed"
            payload["message"] = (
                "robot position changed after option preview; "
                "request new options"
            )
            return _prepare_selected_move_response(payload)

        search_parameters = session_snapshot["search_parameters"]
        prepare_request = (
            rosmaster_x3_bridge_mcp.PrepareSafeNavigation_Request(
                x=float(option_snapshot["x"]),
                y=float(option_snapshot["y"]),
                yaw_deg=float(option_snapshot["yaw_deg"]),
                window=float(search_parameters["window_m"]),
                goal_cost_limit=int(
                    search_parameters["goal_cost_limit"]
                ),
                plan_tolerance=float(
                    search_parameters["plan_tolerance_m"]
                ),
                strict_high_cost=True,
            )
        )
        prepare_response = prepare_safe_navigation(prepare_request)
        prepare_result = json.loads(
            prepare_response.result_json.data
        )

        if (
            prepare_result.get("ok") is not True
            or prepare_result.get("prepared") is not True
            or prepare_result.get("token_issued") is not True
        ):
            failure_reason = _prepare_failure_reason(prepare_result)
            _invalidate_move_option_session(session_id, failure_reason)
            payload.update({
                "reason": failure_reason,
                "target": prepare_result.get("target"),
                "current_pose": prepare_result.get("current_pose"),
                "costmap": prepare_result.get("costmap"),
                "plan": prepare_result.get("plan"),
                "message": (
                    "selected target failed second strict validation; "
                    "request new options"
                ),
            })
            return _prepare_selected_move_response(payload)

        confirmation = prepare_result.get("confirmation", {})
        issued_token = str(confirmation.get("token", ""))
        if not issued_token:
            raise RuntimeError("prepared token missing from response")

        confirmation_text = (
            "CONFIRM_MOVE_" + secrets.token_hex(4).upper()
        )
        finalize_reason = None
        now_epoch = time.time()
        now_monotonic = time.monotonic()

        # This is the only path that holds both locks. The fixed order is
        # session lock first, prepared-token lock second.
        with _MOVE_OPTION_SESSIONS_LOCK:
            current_session = _MOVE_OPTION_SESSIONS.get(session_id)
            with _PREPARED_NAV_LOCK:
                token_record = _PREPARED_NAV_TOKENS.get(issued_token)

                if current_session is None:
                    finalize_reason = "session_not_found"
                elif (
                    float(current_session["expires_monotonic"])
                    <= now_monotonic
                ):
                    finalize_reason = "session_expired"
                    _MOVE_OPTION_SESSIONS.pop(session_id, None)
                elif current_session.get("used") is True:
                    finalize_reason = "session_already_used"
                elif (
                    current_session.get("selection_in_progress")
                    is not True
                ):
                    finalize_reason = "session_selection_in_progress"
                elif token_record is None:
                    finalize_reason = "internal_error"

                if finalize_reason is not None:
                    _PREPARED_NAV_TOKENS.pop(issued_token, None)
                    if current_session is not None:
                        current_session["selection_in_progress"] = False
                else:
                    token_record.update({
                        "source": "prepare_selected_move",
                        "execution_authorized": True,
                        "move_option_session_id": session_id,
                        "selected_option_id": option_id,
                        "prepared_pose": copy.deepcopy(latest_pose),
                        "source_session_id": session_id,
                        "source_option_id": option_id,
                        "confirmation_text": confirmation_text,
                    })
                    current_session.update({
                        "used": True,
                        "selected": True,
                        "selection_in_progress": False,
                        "selected_option_id": option_id,
                        "used_epoch": now_epoch,
                        "used_monotonic": now_monotonic,
                        "prepared_token": issued_token,
                    })

        if finalize_reason is not None:
            issued_token = None
            payload["reason"] = finalize_reason
            payload["message"] = (
                "session changed before token finalization; "
                "no usable token was retained"
            )
            return _prepare_selected_move_response(payload)

        payload.update({
            "ok": True,
            "reason": "token_issued",
            "prepared": True,
            "validation_passed": True,
            "session_consumed": True,
            "selected_option_id": option_id,
            "token_issued": True,
            "token": issued_token,
            "confirmation_text": confirmation_text,
            "target": prepare_result.get("target"),
            "current_pose": prepare_result.get("current_pose"),
            "costmap": prepare_result.get("costmap"),
            "plan": prepare_result.get("plan"),
            "validation_digest": confirmation.get(
                "validation_digest"
            ),
            "expires_at_epoch": confirmation.get(
                "expires_at_epoch"
            ),
            "message": (
                "selected target was revalidated and a one-time token "
                "was issued; no navigation goal was sent"
            ),
        })

    except Exception as exc:
        if issued_token:
            with _PREPARED_NAV_LOCK:
                _PREPARED_NAV_TOKENS.pop(issued_token, None)
        if claimed:
            _invalidate_move_option_session(
                str(getattr(msg, "session_id", "")),
                "internal_error",
            )
        payload["ok"] = False
        payload["reason"] = "internal_error"
        payload["error_type"] = type(exc).__name__
        payload["message"] = "selected move preparation failed safely"

    return _prepare_selected_move_response(payload)


def _prepared_navigation_execution_gate_open() -> bool:
    """Accept only the dedicated, exact server-side enable value."""
    return os.environ.get(
        "X3_ALLOW_PREPARED_NAV_EXECUTION",
        "",
    ) == "1"


def _prepared_navigation_execution_budget_sec() -> float:
    """Return the bounded server-only execution freshness budget."""
    return _bounded_env_float(
        "X3_PREPARED_NAV_EXECUTION_BUDGET_SEC",
        default=20.0,
        minimum=5.0,
        maximum=30.0,
    )


def _prepared_navigation_acceptance_timeout_sec() -> float:
    """Return the bounded wait for a new move_base goal ID."""
    return _bounded_env_float(
        "X3_PREPARED_NAV_ACCEPTANCE_TIMEOUT_SEC",
        default=5.0,
        minimum=2.0,
        maximum=10.0,
    )


def _prepared_navigation_result_timeout_sec() -> float:
    """Return the bounded terminal navigation-result wait."""
    return _bounded_env_float(
        "X3_PREPARED_NAV_RESULT_TIMEOUT_SEC",
        default=15.0,
        minimum=10.0,
        maximum=30.0,
    )


def _consume_claimed_prepared_token(
    token: str,
    used_reason: str,
    *,
    goal_publish_attempted: bool = False,
    goal_publish_completed: bool = False,
) -> bool:
    """Consume a claimed token atomically and retain replay evidence."""
    now_epoch = time.time()
    now_monotonic = time.monotonic()
    with _PREPARED_NAV_LOCK:
        record = _PREPARED_NAV_TOKENS.get(token)
        if record is None:
            return False

        if record.get("used") is not True:
            record["used_epoch"] = now_epoch
            record["used_monotonic"] = now_monotonic

        record.update({
            "executing": False,
            "used": True,
            "used_reason": used_reason,
            "goal_publish_attempted": bool(
                goal_publish_attempted
            ),
            "goal_publish_completed": bool(
                goal_publish_completed
            ),
        })
        return True


def _finalize_prepared_execution_claim(
    token: str,
    execution_claim_id: str,
    execution_deadline_monotonic: float,
) -> dict:
    """Perform the final atomic freshness check and pre-publish consume."""
    now_epoch = time.time()
    now_monotonic = time.monotonic()
    with _PREPARED_NAV_LOCK:
        record = _PREPARED_NAV_TOKENS.get(token)
        if record is None:
            return {
                "ok": False,
                "reason": "execution_claim_lost",
                "token_consumed": False,
            }

        reason = None
        if (
            record.get("execution_claim_id") != execution_claim_id
            or record.get("executing") is not True
            or record.get("used") is True
        ):
            reason = "execution_claim_lost"
        elif (
            float(record.get("expires_monotonic", 0.0))
            <= now_monotonic
        ):
            reason = "token_expired_before_publish"
        elif now_monotonic >= execution_deadline_monotonic:
            reason = "execution_validation_timeout"
        elif not _prepared_navigation_execution_gate_open():
            reason = "execution_gate_closed_before_publish"

        if reason is not None:
            record.update({
                "executing": False,
                "used": True,
                "used_epoch": now_epoch,
                "used_monotonic": now_monotonic,
                "used_reason": reason,
                "goal_publish_attempted": False,
                "goal_publish_completed": False,
            })
            return {
                "ok": False,
                "reason": reason,
                "token_consumed": True,
            }

        record.update({
            "executing": False,
            "used": True,
            "used_epoch": now_epoch,
            "used_monotonic": now_monotonic,
            "used_reason": "goal_publish_attempted",
            "goal_publish_attempted": True,
            "goal_publish_completed": False,
        })
        return {
            "ok": True,
            "reason": "goal_publish_attempted",
            "token_consumed": True,
        }


def _record_prepared_goal_publish_result(
    token: str,
    *,
    used_reason: str,
    publish_call_returned: bool,
    publish_outcome_unknown: bool,
    cleanup_attempted: bool,
    cleanup_completed: bool,
    cleanup_error_type,
    navigation_result=None,
) -> None:
    """Retain publish and cleanup evidence without reopening the token."""
    with _PREPARED_NAV_LOCK:
        record = _PREPARED_NAV_TOKENS.get(token)
        if record is None:
            return
        record.update({
            "used_reason": used_reason,
            "goal_publish_completed": bool(publish_call_returned),
            "goal_publish_call_returned": bool(publish_call_returned),
            "goal_publish_outcome_unknown": bool(
                publish_outcome_unknown
            ),
            "goal_topic_cleanup_attempted": bool(cleanup_attempted),
            "goal_topic_cleanup_completed": bool(cleanup_completed),
            "goal_topic_cleanup_error_type": cleanup_error_type,
        })
        if isinstance(navigation_result, dict):
            record.update({
                "move_base_acceptance_known": bool(
                    navigation_result.get("move_base_acceptance_known")
                ),
                "move_base_goal_id": navigation_result.get(
                    "move_base_goal_id"
                ),
                "move_base_status": navigation_result.get(
                    "move_base_status"
                ),
                "move_base_status_name": navigation_result.get(
                    "move_base_status_name"
                ),
                "move_base_status_text": navigation_result.get(
                    "move_base_status_text"
                ),
                "terminal_status_known": bool(
                    navigation_result.get("terminal_status_known")
                ),
                "arrival_known": bool(
                    navigation_result.get("arrival_known")
                ),
                "arrived": bool(navigation_result.get("arrived")),
                "navigation_outcome": navigation_result.get(
                    "navigation_outcome"
                ),
            })


def _prepared_execution_parameters(token_record: dict) -> dict:
    """Load only frozen server-side validation parameters from a token."""
    parameters = token_record.get("parameters", {})
    window_m = float(parameters["window"])
    goal_cost_limit = int(parameters["goal_cost_limit"])
    plan_tolerance_m = float(parameters["plan_tolerance"])
    max_prepare_distance_m = float(
        parameters["max_prepare_distance_m"]
    )
    plan_timeout_sec = float(parameters["plan_timeout_sec"])
    strict_high_cost = parameters.get("strict_high_cost")

    if not all(math.isfinite(value) for value in (
        window_m,
        plan_tolerance_m,
        max_prepare_distance_m,
        plan_timeout_sec,
    )):
        raise ValueError("frozen execution parameters are non-finite")

    if not 0.05 <= window_m <= 0.5:
        raise ValueError("frozen costmap window is invalid")
    if not 1 <= goal_cost_limit <= 80:
        raise ValueError("frozen goal cost limit is invalid")
    if not 0.0 <= plan_tolerance_m <= 0.10:
        raise ValueError("frozen plan tolerance is invalid")
    if not 0.05 <= max_prepare_distance_m <= 0.10:
        raise ValueError("frozen distance limit is invalid")
    if not 0.5 <= plan_timeout_sec <= 8.0:
        raise ValueError("frozen plan timeout is invalid")
    if strict_high_cost is not True:
        raise ValueError("strict high-cost validation is required")

    return {
        "window_m": window_m,
        "goal_cost_limit": goal_cost_limit,
        "plan_tolerance_m": plan_tolerance_m,
        "max_prepare_distance_m": max_prepare_distance_m,
        "plan_timeout_sec": min(plan_timeout_sec, 3.0),
        "strict_high_cost": True,
    }


def _execute_prepared_navigation_response(
    payload: dict,
) -> rosmaster_x3_bridge_mcp.ExecutePreparedNavigation_Response:
    """Create the generated prepared-navigation execution response."""
    return rosmaster_x3_bridge_mcp.ExecutePreparedNavigation_Response(
        result_json=std_msgs_mcp.String(
            data=json.dumps(payload, ensure_ascii=False)
        )
    )


@provider.mcp(
    "robonix/primitive/rosmaster_x3_bridge/execute_prepared_navigation"
)
def execute_prepared_navigation(
    msg: rosmaster_x3_bridge_mcp.ExecutePreparedNavigation_Request,
) -> rosmaster_x3_bridge_mcp.ExecutePreparedNavigation_Response:
    """Revalidate and publish one server-authorized prepared goal once."""
    execution_time_budget_sec = (
        _prepared_navigation_execution_budget_sec()
    )
    acceptance_timeout_sec = (
        _prepared_navigation_acceptance_timeout_sec()
    )
    result_timeout_sec = _prepared_navigation_result_timeout_sec()
    payload = {
        "ok": False,
        "reason": "internal_error",
        "execution_requested": False,
        "execution_gate_open": (
            _prepared_navigation_execution_gate_open()
        ),
        "token_found": False,
        "token_execution_authorized": False,
        "token_claimed": False,
        "token_consumed": False,
        "one_time": True,
        "confirmation_matched": False,
        "third_validation_passed": False,
        "navigation_goal_sent": False,
        "navigation_goal_sent_known": True,
        "goal_publish_attempted": False,
        "goal_publish_completed": False,
        "goal_publish_call_returned": False,
        "goal_publish_outcome_unknown": False,
        "goal_publish_error_type": None,
        "goal_topic_cleanup_attempted": False,
        "goal_topic_cleanup_completed": None,
        "goal_topic_cleanup_error_type": None,
        "execution_started": False,
        "execution_elapsed_sec": 0.0,
        "execution_validation_elapsed_sec": 0.0,
        "execution_time_budget_sec": execution_time_budget_sec,
        "execution_time_budget_reached": False,
        "navigation_wait_elapsed_sec": 0.0,
        "navigation_acceptance_timeout_sec": acceptance_timeout_sec,
        "navigation_result_timeout_sec": result_timeout_sec,
        "move_base_acceptance_known": False,
        "move_base_goal_id": None,
        "move_base_status": None,
        "move_base_status_name": None,
        "move_base_status_text": None,
        "terminal_status_known": False,
        "arrival_known": False,
        "arrived": False,
        "navigation_outcome": None,
        "cmd_vel_observation": None,
        "amcl_observation": None,
        "odom_observation": None,
        "safety_watchdog_triggered": False,
        "safety_watchdog_reason": None,
        "safety_cancel_attempted": False,
        "safety_cancel_terminal_observed": False,
        "motion_evidence_confirmed": False,
        "motion_evidence_min_displacement_m": 0.05,
        "direct_cmd_vel_published": False,
        "target": None,
        "current_pose": None,
        "prepared_pose": None,
        "pose_translation_m": None,
        "pose_translation_limit_m": 0.03,
        "pose_yaw_change_deg": None,
        "pose_yaw_gate_applied": False,
        "costmap": None,
        "plan": None,
        "validation_digest": None,
        "message": "prepared navigation execution failed safely",
    }
    token = ""
    token_claimed = False
    execution_claim_id = ""
    execution_started_monotonic = None
    execution_deadline_monotonic = None
    navigation_wait_started_monotonic = None

    def finish_response():
        if execution_started_monotonic is not None:
            elapsed_sec = max(
                0.0,
                time.monotonic() - execution_started_monotonic,
            )
            payload["execution_elapsed_sec"] = elapsed_sec
        if navigation_wait_started_monotonic is not None:
            payload["navigation_wait_elapsed_sec"] = max(
                0.0,
                time.monotonic() - navigation_wait_started_monotonic,
            )
        return _execute_prepared_navigation_response(payload)

    def execution_budget_remaining_sec() -> float:
        if execution_deadline_monotonic is None:
            return execution_time_budget_sec
        return max(
            0.0,
            execution_deadline_monotonic - time.monotonic(),
        )

    def fail_after_claim(reason: str, message: str):
        payload["token_consumed"] = _consume_claimed_prepared_token(
            token,
            reason,
        )
        payload["reason"] = reason
        payload["message"] = message
        if reason == "execution_validation_timeout":
            payload["execution_time_budget_reached"] = True
        return finish_response()

    try:
        token = str(msg.token).strip()
        allow_execute = bool(msg.allow_execute)
        confirmation_text = str(msg.confirmation_text)
        payload["execution_requested"] = allow_execute

        if not token:
            payload["reason"] = "invalid_request"
            payload["message"] = "a non-empty prepared token is required"
            return finish_response()

        rejection = None
        token_snapshot = None
        now_epoch = time.time()
        now_monotonic = time.monotonic()

        with _PREPARED_NAV_LOCK:
            token_record = _PREPARED_NAV_TOKENS.get(token)
            if token_record is None:
                rejection = (
                    "token_not_found",
                    "prepared token was not found",
                )
            else:
                payload["token_found"] = True
                if (
                    float(token_record.get("expires_monotonic", 0.0))
                    <= now_monotonic
                ):
                    _PREPARED_NAV_TOKENS.pop(token, None)
                    rejection = (
                        "token_expired",
                        "prepared token has expired",
                    )
                else:
                    execution_authorized = (
                        token_record.get("execution_authorized") is True
                        and token_record.get("source")
                        == "prepare_selected_move"
                    )
                    payload["token_execution_authorized"] = (
                        execution_authorized
                    )

                    if not execution_authorized:
                        rejection = (
                            "token_not_execution_authorized",
                            "token was not issued by prepare_selected_move",
                        )
                    elif token_record.get("used") is True:
                        payload["token_consumed"] = True
                        payload["goal_publish_attempted"] = bool(
                            token_record.get("goal_publish_attempted")
                        )
                        prior_publish_returned = bool(
                            token_record.get("goal_publish_call_returned")
                        )
                        prior_outcome_unknown = bool(
                            token_record.get(
                                "goal_publish_outcome_unknown"
                            )
                        )
                        payload.update({
                            "goal_publish_completed": (
                                prior_publish_returned
                            ),
                            "goal_publish_call_returned": (
                                prior_publish_returned
                            ),
                            "goal_publish_outcome_unknown": (
                                prior_outcome_unknown
                            ),
                            "navigation_goal_sent": (
                                None
                                if prior_outcome_unknown
                                else prior_publish_returned
                            ),
                            "navigation_goal_sent_known": (
                                not prior_outcome_unknown
                            ),
                            "goal_topic_cleanup_attempted": bool(
                                token_record.get(
                                    "goal_topic_cleanup_attempted"
                                )
                            ),
                            "goal_topic_cleanup_completed": (
                                token_record.get(
                                    "goal_topic_cleanup_completed"
                                )
                            ),
                            "goal_topic_cleanup_error_type": (
                                token_record.get(
                                    "goal_topic_cleanup_error_type"
                                )
                            ),
                            "move_base_acceptance_known": bool(
                                token_record.get(
                                    "move_base_acceptance_known"
                                )
                            ),
                            "move_base_goal_id": token_record.get(
                                "move_base_goal_id"
                            ),
                            "move_base_status": token_record.get(
                                "move_base_status"
                            ),
                            "move_base_status_name": token_record.get(
                                "move_base_status_name"
                            ),
                            "move_base_status_text": token_record.get(
                                "move_base_status_text"
                            ),
                            "terminal_status_known": bool(
                                token_record.get("terminal_status_known")
                            ),
                            "arrival_known": bool(
                                token_record.get("arrival_known")
                            ),
                            "arrived": bool(token_record.get("arrived")),
                            "navigation_outcome": token_record.get(
                                "navigation_outcome"
                            ),
                        })
                        rejection = (
                            "token_already_used",
                            "prepared token was already consumed",
                        )
                    elif token_record.get("executing") is True:
                        rejection = (
                            "execution_in_progress",
                            "prepared token is already being executed",
                        )
                    elif not allow_execute:
                        rejection = (
                            "execution_not_allowed",
                            "allow_execute must be true",
                        )
                    else:
                        expected_confirmation = str(
                            token_record.get("confirmation_text", "")
                        )
                        payload["confirmation_matched"] = (
                            secrets.compare_digest(
                                confirmation_text,
                                expected_confirmation,
                            )
                        )
                        if not payload["confirmation_matched"]:
                            rejection = (
                                "confirmation_mismatch",
                                "confirmation text does not match the token",
                            )
                        else:
                            gate_open = (
                                _prepared_navigation_execution_gate_open()
                            )
                            payload["execution_gate_open"] = gate_open
                            if not gate_open:
                                rejection = (
                                    "execution_gate_closed",
                                    "dedicated prepared execution gate is closed",
                                )
                            else:
                                execution_started_monotonic = (
                                    time.monotonic()
                                )
                                execution_deadline_monotonic = (
                                    execution_started_monotonic
                                    + execution_time_budget_sec
                                )
                                execution_claim_id = (
                                    secrets.token_urlsafe(18)
                                )
                                token_record.update({
                                    "executing": True,
                                    "execution_claimed_epoch": time.time(),
                                    "execution_claimed_monotonic": (
                                        execution_started_monotonic
                                    ),
                                    "execution_claim_id": (
                                        execution_claim_id
                                    ),
                                    "execution_deadline_monotonic": (
                                        execution_deadline_monotonic
                                    ),
                                })
                                token_snapshot = copy.deepcopy(token_record)
                                token_claimed = True
                                payload["token_claimed"] = True

        if rejection is not None:
            payload["reason"], payload["message"] = rejection
            return finish_response()

        payload["target"] = copy.deepcopy(
            token_snapshot.get("target")
        )
        payload["prepared_pose"] = copy.deepcopy(
            token_snapshot.get("prepared_pose")
        )
        payload["validation_digest"] = token_snapshot.get(
            "validation_digest"
        )

        try:
            readiness = _navigation_ready_snapshot()
        except Exception:
            readiness = {"ok": False, "ready": False}
        payload["navigation_readiness"] = readiness
        if execution_budget_remaining_sec() <= 0.0:
            return fail_after_claim(
                "execution_validation_timeout",
                "execution budget expired during readiness validation",
            )
        if (
            readiness.get("ok") is not True
            or readiness.get("ready") is not True
        ):
            return fail_after_claim(
                "nav_not_ready",
                "navigation prerequisites failed third validation",
            )

        b = ensure_bridge()
        try:
            current_pose = _fresh_amcl_pose_snapshot(b)
        except Exception:
            if execution_budget_remaining_sec() <= 0.0:
                return fail_after_claim(
                    "execution_validation_timeout",
                    "execution budget expired during AMCL validation",
                )
            return fail_after_claim(
                "amcl_check_failed",
                "fresh AMCL pose failed third validation",
            )
        payload["current_pose"] = current_pose
        if execution_budget_remaining_sec() <= 0.0:
            return fail_after_claim(
                "execution_validation_timeout",
                "execution budget expired during AMCL validation",
            )

        try:
            prepared_pose = token_snapshot["prepared_pose"]
            prepared_frame = str(prepared_pose["frame_id"])
            prepared_x = float(prepared_pose["x"])
            prepared_y = float(prepared_pose["y"])
            prepared_yaw_deg = float(prepared_pose["yaw_deg"])
            if prepared_frame != "map" or not all(
                math.isfinite(value)
                for value in (
                    prepared_x,
                    prepared_y,
                    prepared_yaw_deg,
                )
            ):
                raise ValueError("prepared pose is invalid")
        except Exception:
            payload["prepared_pose"] = None
            return fail_after_claim(
                "target_invalid",
                "token prepared pose is invalid",
            )

        pose_translation_m = math.hypot(
            current_pose["x"] - prepared_x,
            current_pose["y"] - prepared_y,
        )
        payload["pose_translation_m"] = pose_translation_m
        payload["pose_yaw_change_deg"] = (
            _circular_angle_distance_deg(
                current_pose["yaw_deg"],
                prepared_yaw_deg,
            )
        )
        if pose_translation_m > 0.03:
            return fail_after_claim(
                "robot_pose_changed",
                "robot moved more than 0.03 m after preparation",
            )

        try:
            target = token_snapshot["target"]
            target_frame = str(target["frame_id"])
            target_x = float(target["x"])
            target_y = float(target["y"])
            target_yaw_deg = float(target["yaw_deg"])
            if target_frame != "map" or not all(
                math.isfinite(value)
                for value in (target_x, target_y, target_yaw_deg)
            ):
                raise ValueError("target is invalid")
            validation_parameters = _prepared_execution_parameters(
                token_snapshot
            )
        except Exception:
            payload["target"] = None
            return fail_after_claim(
                "target_invalid",
                "token target or frozen parameters are invalid",
            )

        remaining_budget_sec = execution_budget_remaining_sec()
        if remaining_budget_sec <= 0.0:
            return fail_after_claim(
                "execution_validation_timeout",
                "execution budget expired before costmap validation",
            )
        costmap_timeout_sec = min(8.0, remaining_budget_sec)
        try:
            grid = b.get_global_costmap_grid(
                timeout=costmap_timeout_sec
            )
            costmap_context = _global_costmap_context(grid)
        except Exception:
            if execution_budget_remaining_sec() <= 0.0:
                return fail_after_claim(
                    "execution_validation_timeout",
                    "execution budget expired during costmap validation",
                )
            return fail_after_claim(
                "costmap_check_failed",
                "global costmap failed third validation",
            )
        if execution_budget_remaining_sec() <= 0.0:
            return fail_after_claim(
                "execution_validation_timeout",
                "execution budget expired during costmap validation",
            )

        costmap_validation = _validate_move_option_costmap(
            target,
            validation_parameters,
            current_pose,
            costmap_context,
        )
        payload["costmap"] = {
            "topic": costmap_context.get("topic"),
            "frame_id": costmap_context.get("frame_id"),
            "resolution": costmap_context.get("resolution"),
            "width": costmap_context.get("width"),
            "height": costmap_context.get("height"),
            "origin_x": costmap_context.get("origin_x"),
            "origin_y": costmap_context.get("origin_y"),
            "timeout_sec": costmap_timeout_sec,
            "distance_m": costmap_validation.get("distance_m"),
            "stats": costmap_validation.get("costmap_stats"),
        }
        if costmap_validation.get("ok") is not True:
            failure_reason = costmap_validation.get("reason")
            if failure_reason in {
                "target_frame_invalid",
                "target_non_finite",
                "target_too_far",
            }:
                return fail_after_claim(
                    "target_invalid",
                    "token target failed frame, finite, or distance checks",
                )
            return fail_after_claim(
                "costmap_check_failed",
                "target failed strict costmap third validation",
            )

        remaining_budget_sec = execution_budget_remaining_sec()
        if remaining_budget_sec <= 0.0:
            return fail_after_claim(
                "execution_validation_timeout",
                "execution budget expired before make_plan validation",
            )
        validation_parameters["plan_timeout_sec"] = min(
            float(validation_parameters["plan_timeout_sec"]),
            3.0,
            5.0,
            remaining_budget_sec,
        )
        plan_validation = _validate_move_option_plan(
            b,
            target,
            validation_parameters,
            costmap_context,
            costmap_validation,
        )
        payload["plan"] = plan_validation.get("plan")
        if execution_budget_remaining_sec() <= 0.0:
            return fail_after_claim(
                "execution_validation_timeout",
                "execution budget expired during make_plan validation",
            )
        if plan_validation.get("ok") is not True:
            return fail_after_claim(
                "make_plan_failed",
                "target failed make_plan third validation",
            )

        payload["third_validation_passed"] = True
        final_claim = _finalize_prepared_execution_claim(
            token,
            execution_claim_id,
            execution_deadline_monotonic,
        )
        payload["token_consumed"] = final_claim["token_consumed"]
        if final_claim["ok"] is not True:
            payload["reason"] = final_claim["reason"]
            if (
                final_claim["reason"]
                == "execution_gate_closed_before_publish"
            ):
                payload["execution_gate_open"] = False
            payload["execution_time_budget_reached"] = (
                final_claim["reason"] == "execution_validation_timeout"
            )
            final_messages = {
                "token_expired_before_publish": (
                    "prepared token expired before goal publish"
                ),
                "execution_validation_timeout": (
                    "execution validation exceeded its server time budget"
                ),
                "execution_gate_closed_before_publish": (
                    "dedicated execution gate closed before goal publish"
                ),
                "execution_claim_lost": (
                    "execution claim changed or disappeared before publish"
                ),
            }
            payload["message"] = final_messages.get(
                final_claim["reason"],
                "final execution freshness check failed",
            )
            return finish_response()

        payload["execution_validation_elapsed_sec"] = max(
            0.0,
            time.monotonic() - execution_started_monotonic,
        )
        payload["goal_publish_attempted"] = True
        navigation_wait_started_monotonic = time.monotonic()

        try:
            publish_result = b.send_navigation_goal_pose(
                frame_id=target_frame,
                x=target_x,
                y=target_y,
                yaw_deg=target_yaw_deg,
                acceptance_timeout=acceptance_timeout_sec,
                result_timeout=result_timeout_sec,
                max_linear_mps=0.10,
                max_angular_rps=0.60,
                max_odom_path_m=0.10,
                max_amcl_displacement_m=0.10,
                cancel_timeout=4.0,
            )
        except Exception as exc:
            _record_prepared_goal_publish_result(
                token,
                used_reason="goal_publish_outcome_unknown",
                publish_call_returned=False,
                publish_outcome_unknown=True,
                cleanup_attempted=False,
                cleanup_completed=False,
                cleanup_error_type=None,
            )
            payload.update({
                "reason": "goal_publish_outcome_unknown",
                "navigation_goal_sent": None,
                "navigation_goal_sent_known": False,
                "goal_publish_outcome_unknown": True,
                "goal_publish_error_type": type(exc).__name__,
                "navigation_outcome": "publish_outcome_unknown",
            })
            payload["message"] = (
                "The goal may already have been delivered. "
                "Do not retry this token."
            )
            return finish_response()

        publish_attempted = bool(
            publish_result.get("publish_attempted")
        )
        publish_call_returned = bool(
            publish_result.get("publish_call_returned")
        )
        publish_outcome_unknown = bool(
            publish_result.get("publish_outcome_unknown")
        )
        cleanup_attempted = bool(
            publish_result.get("cleanup_attempted")
        )
        cleanup_completed = bool(
            publish_result.get("cleanup_completed")
        )
        cleanup_error_type = publish_result.get(
            "cleanup_error_type"
        )
        acceptance_known = bool(
            publish_result.get("move_base_acceptance_known")
        )
        terminal_status_known = bool(
            publish_result.get("terminal_status_known")
        )
        status_code = publish_result.get("move_base_status")
        status_name = publish_result.get("move_base_status_name")
        status_text = publish_result.get("move_base_status_text")
        goal_id = publish_result.get("move_base_goal_id")
        navigation_outcome = publish_result.get("navigation_outcome")
        cmd_vel_observation = {
            "messages": int(
                publish_result.get("cmd_vel_messages", 0) or 0
            ),
            "nonzero_messages": int(
                publish_result.get("nonzero_cmd_vel_messages", 0) or 0
            ),
            "max_linear_mps": float(
                publish_result.get("max_linear_mps", 0.0) or 0.0
            ),
            "max_angular_rps": float(
                publish_result.get("max_angular_rps", 0.0) or 0.0
            ),
        }
        amcl_last_pose = publish_result.get("amcl_last_pose")
        amcl_displacement_from_validation_m = None
        if isinstance(amcl_last_pose, dict):
            try:
                amcl_displacement_from_validation_m = math.hypot(
                    float(amcl_last_pose["x"]) - current_pose["x"],
                    float(amcl_last_pose["y"]) - current_pose["y"],
                )
            except (KeyError, TypeError, ValueError):
                amcl_displacement_from_validation_m = None
        amcl_observation = {
            "messages": int(
                publish_result.get("amcl_messages", 0) or 0
            ),
            "first_pose": publish_result.get("amcl_first_pose"),
            "last_pose": amcl_last_pose,
            "observer_displacement_m": publish_result.get(
                "amcl_displacement_m"
            ),
            "displacement_from_validation_m": (
                amcl_displacement_from_validation_m
            ),
        }
        odom_observation = {
            "messages": int(
                publish_result.get("odom_messages", 0) or 0
            ),
            "first_pose": publish_result.get("odom_first_pose"),
            "last_pose": publish_result.get("odom_last_pose"),
            "displacement_m": publish_result.get(
                "odom_displacement_m"
            ),
            "path_length_m": publish_result.get(
                "odom_path_length_m"
            ),
        }
        odom_displacement_m = publish_result.get("odom_displacement_m")
        try:
            odom_displacement_m = float(odom_displacement_m)
        except (TypeError, ValueError):
            odom_displacement_m = None
        motion_evidence_confirmed = (
            cmd_vel_observation["nonzero_messages"] > 0
            and amcl_displacement_from_validation_m is not None
            and amcl_displacement_from_validation_m
            >= payload["motion_evidence_min_displacement_m"]
            and amcl_displacement_from_validation_m <= 0.10
            and odom_displacement_m is not None
            and odom_displacement_m
            >= payload["motion_evidence_min_displacement_m"]
            and odom_displacement_m <= 0.10
        )

        payload.update({
            "goal_publish_attempted": publish_attempted,
            "goal_publish_call_returned": publish_call_returned,
            "goal_publish_completed": publish_call_returned,
            "goal_publish_outcome_unknown": publish_outcome_unknown,
            "goal_publish_error_type": publish_result.get(
                "publish_error_type"
            ),
            "goal_topic_cleanup_attempted": cleanup_attempted,
            "goal_topic_cleanup_completed": cleanup_completed,
            "goal_topic_cleanup_error_type": cleanup_error_type,
            "execution_started": publish_attempted,
            "move_base_acceptance_known": acceptance_known,
            "move_base_goal_id": goal_id,
            "move_base_status": status_code,
            "move_base_status_name": status_name,
            "move_base_status_text": status_text,
            "terminal_status_known": terminal_status_known,
            "arrival_known": bool(
                publish_result.get("arrival_known")
            ),
            "arrived": bool(publish_result.get("arrived")),
            "navigation_outcome": navigation_outcome,
            "cmd_vel_observation": cmd_vel_observation,
            "amcl_observation": amcl_observation,
            "odom_observation": odom_observation,
            "safety_watchdog_triggered": bool(
                publish_result.get("safety_watchdog_triggered")
            ),
            "safety_watchdog_reason": publish_result.get(
                "safety_watchdog_reason"
            ),
            "safety_cancel_attempted": bool(
                publish_result.get("safety_cancel_attempted")
            ),
            "safety_cancel_terminal_observed": bool(
                publish_result.get("safety_cancel_terminal_observed")
            ),
            "motion_evidence_confirmed": motion_evidence_confirmed,
        })

        if payload["safety_watchdog_triggered"]:
            reason = "navigation_safety_watchdog"
            _record_prepared_goal_publish_result(
                token,
                used_reason=reason,
                publish_call_returned=publish_call_returned,
                publish_outcome_unknown=False,
                cleanup_attempted=cleanup_attempted,
                cleanup_completed=cleanup_completed,
                cleanup_error_type=cleanup_error_type,
                navigation_result=publish_result,
            )
            payload.update({
                "reason": reason,
                "navigation_goal_sent": True,
                "navigation_goal_sent_known": True,
                "message": (
                    "navigation was cancelled by the short-move safety "
                    "watchdog: {}"
                ).format(payload["safety_watchdog_reason"]),
            })
            return finish_response()

        if not publish_call_returned:
            reason = (
                "goal_publish_outcome_unknown"
                if publish_attempted or publish_outcome_unknown
                else "goal_publish_failed"
            )
            outcome_unknown = reason == "goal_publish_outcome_unknown"
            _record_prepared_goal_publish_result(
                token,
                used_reason=reason,
                publish_call_returned=False,
                publish_outcome_unknown=outcome_unknown,
                cleanup_attempted=cleanup_attempted,
                cleanup_completed=cleanup_completed,
                cleanup_error_type=cleanup_error_type,
                navigation_result=publish_result,
            )
            payload.update({
                "reason": reason,
                "navigation_goal_sent": None if outcome_unknown else False,
                "navigation_goal_sent_known": not outcome_unknown,
                "goal_publish_outcome_unknown": outcome_unknown,
                "message": (
                    "The goal may already have been delivered. "
                    "Do not retry this token."
                    if outcome_unknown
                    else "goal setup failed before publish; token consumed"
                ),
            })
            return finish_response()

        if not acceptance_known:
            reason = "goal_acceptance_timeout"
            _record_prepared_goal_publish_result(
                token,
                used_reason=reason,
                publish_call_returned=True,
                publish_outcome_unknown=False,
                cleanup_attempted=cleanup_attempted,
                cleanup_completed=cleanup_completed,
                cleanup_error_type=cleanup_error_type,
                navigation_result=publish_result,
            )
            payload.update({
                "reason": reason,
                "navigation_goal_sent": None,
                "navigation_goal_sent_known": False,
                "message": (
                    "local publish returned but no new move_base goal ID "
                    "was observed; do not retry this token"
                ),
            })
            return finish_response()

        payload.update({
            "navigation_goal_sent": True,
            "navigation_goal_sent_known": True,
        })
        if navigation_outcome in {
            "result_timeout",
            "result_timeout_cancelled",
            "result_timeout_cancel_timeout",
        }:
            reason = "navigation_result_timeout"
            _record_prepared_goal_publish_result(
                token,
                used_reason=reason,
                publish_call_returned=True,
                publish_outcome_unknown=False,
                cleanup_attempted=cleanup_attempted,
                cleanup_completed=cleanup_completed,
                cleanup_error_type=cleanup_error_type,
                navigation_result=publish_result,
            )
            payload.update({
                "reason": reason,
                "message": (
                    "move_base did not finish within the short-move "
                    "deadline; correlated cancellation was requested"
                ),
            })
            return finish_response()
        if not terminal_status_known:
            reason = "navigation_result_timeout"
            _record_prepared_goal_publish_result(
                token,
                used_reason=reason,
                publish_call_returned=True,
                publish_outcome_unknown=False,
                cleanup_attempted=cleanup_attempted,
                cleanup_completed=cleanup_completed,
                cleanup_error_type=cleanup_error_type,
                navigation_result=publish_result,
            )
            payload.update({
                "reason": reason,
                "message": (
                    "move_base accepted the goal but no terminal status "
                    "arrived before timeout; the goal may still be active"
                ),
            })
            return finish_response()

        terminal_reasons = {
            2: "navigation_preempted",
            3: "navigation_succeeded",
            4: "navigation_aborted",
            5: "navigation_rejected",
            8: "navigation_recalled",
            9: "navigation_lost",
        }
        reason = terminal_reasons.get(
            status_code,
            "navigation_terminal_status_unknown",
        )
        if status_code == 3 and not motion_evidence_confirmed:
            reason = "navigation_succeeded_without_motion_evidence"
        ok = reason == "navigation_succeeded"
        _record_prepared_goal_publish_result(
            token,
            used_reason=reason,
            publish_call_returned=True,
            publish_outcome_unknown=False,
            cleanup_attempted=cleanup_attempted,
            cleanup_completed=cleanup_completed,
            cleanup_error_type=cleanup_error_type,
            navigation_result=publish_result,
        )
        payload.update({
            "ok": ok,
            "reason": reason,
            "message": (
                "move_base reported SUCCEEDED with nonzero velocity and "
                "AMCL displacement evidence"
                if ok
                else (
                    "move_base reported SUCCEEDED but physical motion "
                    "evidence was insufficient"
                    if status_code == 3
                    else "move_base finished with {}: {}".format(
                        status_name or status_code,
                        status_text or "no status text",
                    )
                )
            ),
        })
        return finish_response()

    except Exception as exc:
        if token_claimed and not payload["token_consumed"]:
            payload["token_consumed"] = (
                _consume_claimed_prepared_token(
                    token,
                    "internal_error",
                )
            )
        payload["ok"] = False
        payload["reason"] = "internal_error"
        payload["error_type"] = type(exc).__name__
        payload["message"] = (
            "prepared navigation execution failed safely"
        )

    return finish_response()


def yaw_deg_to_quat(yaw_deg: float) -> dict:
    """Convert planar yaw in degrees to ROS quaternion."""
    yaw = math.radians(float(yaw_deg))
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(yaw / 2.0),
        "w": math.cos(yaw / 2.0),
    }


def send_nav_goal(
    msg: rosmaster_x3_bridge_mcp.SendNavGoal_Request,
) -> rosmaster_x3_bridge_mcp.SendNavGoal_Response:
    """Send a ROSMASTER X3 navigation goal through ROS1 move_base.

    Safety:
    - This tool never publishes raw /cmd_vel.
    - If allow_execute is false, it only performs a dry run.
    - Even if allow_execute is true, environment variable
      X3_ALLOW_NAV_GOAL must be set to 1 before it actually sends.
    """
    try:
        x = float(msg.x)
        y = float(msg.y)
        yaw_deg = float(msg.yaw_deg)
        allow_execute = bool(msg.allow_execute)

        payload = {
            "ok": True,
            "dry_run": not allow_execute,
            "sent": False,
            "target": {
                "frame_id": "map",
                "x": x,
                "y": y,
                "yaw_deg": yaw_deg,
            },
            "safety": {
                "raw_cmd_vel": "not used",
                "requires_allow_execute": True,
                "requires_env_X3_ALLOW_NAV_GOAL": True,
            },
        }

        max_goal_distance_m = float(os.environ.get("X3_MAX_NAV_GOAL_DISTANCE_M", "5.0"))

        b = None
        current_amcl = None
        distance_from_current_m = None

        if allow_execute:
            b = ensure_bridge()
            current_amcl = b.get_amcl_pose()
            if current_amcl and "x" in current_amcl and "y" in current_amcl:
                dx = x - float(current_amcl["x"])
                dy = y - float(current_amcl["y"])
                distance_from_current_m = math.sqrt(dx * dx + dy * dy)

        payload["safety"]["max_goal_distance_m"] = max_goal_distance_m
        payload["distance_from_current_m"] = distance_from_current_m

        if not allow_execute:
            payload["message"] = "dry run only; set allow_execute=true to request execution"
        elif os.environ.get("X3_ALLOW_NAV_GOAL", "0") != "1":
            payload["ok"] = False
            payload["message"] = "blocked by safety gate; set X3_ALLOW_NAV_GOAL=1 before boot to allow execution"
        elif distance_from_current_m is None:
            payload["ok"] = False
            payload["message"] = "blocked because current AMCL pose could not be read"
        elif distance_from_current_m > max_goal_distance_m:
            payload["ok"] = False
            payload["message"] = "blocked because target is farther than max_goal_distance_m"
        else:
            if b is None:
                b = ensure_bridge()
            quat = yaw_deg_to_quat(yaw_deg)

            import roslibpy

            goal_topic = roslibpy.Topic(
                b.ros,
                "/move_base_simple/goal",
                "geometry_msgs/PoseStamped",
            )
            goal_topic.advertise()
            time.sleep(0.5)

            goal = roslibpy.Message({
                "header": {
                    "frame_id": "map",
                    "stamp": {"secs": 0, "nsecs": 0},
                },
                "pose": {
                    "position": {"x": x, "y": y, "z": 0.0},
                    "orientation": quat,
                },
            })

            goal_topic.publish(goal)
            time.sleep(0.2)
            goal_topic.unadvertise()

            payload["sent"] = True
            payload["dry_run"] = False
            payload["message"] = "navigation goal sent to /move_base_simple/goal"

    except Exception as exc:
        payload = {
            "ok": False,
            "sent": False,
            "error": str(exc),
            "traceback": traceback.format_exc(),
        }

    return rosmaster_x3_bridge_mcp.SendNavGoal_Response(
        result_json=std_msgs_mcp.String(
            data=json.dumps(payload, ensure_ascii=False)
        )
    )


@provider.mcp("robonix/primitive/rosmaster_x3_bridge/cancel_nav_goal")
def cancel_nav_goal(
    msg: rosmaster_x3_bridge_mcp.CancelNavGoal_Request,
) -> rosmaster_x3_bridge_mcp.CancelNavGoal_Response:
    """Cancel the current ROSMASTER X3 move_base navigation goal.

    Safety:
    - This tool publishes only /move_base/cancel.
    - It does not publish raw /cmd_vel.
    - It does not send a new navigation goal.
    """
    _ = msg

    try:
        b = ensure_bridge()

        import roslibpy

        cancel_topic = roslibpy.Topic(
            b.ros,
            "/move_base/cancel",
            "actionlib_msgs/GoalID",
        )
        cancel_topic.advertise()
        time.sleep(0.5)

        cancel_msg = roslibpy.Message({
            "stamp": {
                "secs": 0,
                "nsecs": 0,
            },
            "id": "",
        })

        cancel_topic.publish(cancel_msg)
        time.sleep(0.2)
        cancel_topic.unadvertise()

        payload = {
            "ok": True,
            "canceled": True,
            "topic": "/move_base/cancel",
            "safety": {
                "raw_cmd_vel": "not used",
                "new_nav_goal": "not sent",
            },
            "message": "cancel message sent to /move_base/cancel",
        }

    except Exception as exc:
        payload = {
            "ok": False,
            "canceled": False,
            "error": str(exc),
            "traceback": traceback.format_exc(),
        }

    return rosmaster_x3_bridge_mcp.CancelNavGoal_Response(
        result_json=std_msgs_mcp.String(
            data=json.dumps(payload, ensure_ascii=False)
        )
    )


def go_to_waypoint(
    msg: rosmaster_x3_bridge_mcp.GoToWaypoint_Request,
) -> rosmaster_x3_bridge_mcp.GoToWaypoint_Response:
    """Navigate ROSMASTER X3 to a named waypoint.

    This tool:
    - Loads a named waypoint from ~/rosmaster_x3_deploy/config/x3_waypoints.json.
    - Reuses send_nav_goal, so all existing safety gates still apply.
    - Does not publish raw /cmd_vel.
    """
    try:
        from pathlib import Path

        name = str(msg.name).strip()
        allow_execute = bool(msg.allow_execute)

        deploy_root = Path(__file__).resolve().parents[3]
        waypoint_file = Path(
            os.environ.get(
                "X3_WAYPOINTS_FILE",
                str(deploy_root / "config" / "x3_waypoints.json"),
            )
        )

        payload = {
            "ok": True,
            "dry_run": not allow_execute,
            "sent": False,
            "waypoint_name": name,
            "waypoint_file": str(waypoint_file),
            "safety": {
                "raw_cmd_vel": "not used",
                "uses_send_nav_goal": True,
                "requires_allow_execute": True,
                "requires_env_X3_ALLOW_NAV_GOAL": True,
                "uses_max_goal_distance_gate": True,
            },
        }

        if not name:
            payload["ok"] = False
            payload["message"] = "blocked because waypoint name is empty"
        elif not waypoint_file.exists():
            payload["ok"] = False
            payload["message"] = "blocked because waypoint file does not exist"
        else:
            data = json.loads(waypoint_file.read_text())
            waypoints = data.get("waypoints", {})

            if name not in waypoints:
                payload["ok"] = False
                payload["message"] = "blocked because waypoint name was not found"
                payload["available_waypoints"] = sorted(waypoints.keys())
            else:
                wp = waypoints[name]
                target = {
                    "frame_id": wp.get("frame_id", "map"),
                    "x": float(wp["x"]),
                    "y": float(wp["y"]),
                    "yaw_deg": float(wp.get("yaw_deg", 0.0)),
                }

                payload["target"] = target

                req = rosmaster_x3_bridge_mcp.SendNavGoal_Request(
                    x=target["x"],
                    y=target["y"],
                    yaw_deg=target["yaw_deg"],
                    allow_execute=allow_execute,
                )

                send_resp = send_nav_goal(req)
                send_data = json.loads(send_resp.result_json.data)

                payload["ok"] = bool(send_data.get("ok", False))
                payload["dry_run"] = bool(send_data.get("dry_run", not allow_execute))
                payload["sent"] = bool(send_data.get("sent", False))
                payload["send_nav_goal_result"] = send_data

                if payload["sent"]:
                    payload["message"] = "waypoint navigation goal sent"
                else:
                    payload["message"] = "waypoint navigation was not sent"

    except Exception as exc:
        payload = {
            "ok": False,
            "dry_run": False,
            "sent": False,
            "error": str(exc),
            "traceback": traceback.format_exc(),
        }

    return rosmaster_x3_bridge_mcp.GoToWaypoint_Response(
        result_json=std_msgs_mcp.String(
            data=json.dumps(payload, ensure_ascii=False)
        )
    )


if __name__ == "__main__":
    provider.run()
