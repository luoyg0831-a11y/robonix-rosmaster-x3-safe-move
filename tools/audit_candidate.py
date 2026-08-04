#!/usr/bin/env python3
"""Fail closed when the public candidate weakens a recorded safety boundary."""

from __future__ import annotations

import ast
import math
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAIN_PATH = ROOT / "jetson/main.py"
BRIDGE_PATH = ROOT / "jetson/x3_bridge.py"
CLI_PATH = ROOT / "jetson/scripts/move_base_cli.py"
MANIFEST_PATH = ROOT / "jetson/package_manifest.yaml"
TOML_ROOT = ROOT / "jetson/capabilities/primitive/rosmaster_x3_bridge"

EXPECTED_CAPABILITIES = {
    "get_status",
    "check_nav_ready",
    "preview_safe_waypoint",
    "prepare_safe_navigation",
    "preview_move_options",
    "prepare_selected_move",
    "execute_prepared_navigation",
    "cancel_nav_goal",
}
FORBIDDEN_CAPABILITIES = {"send_nav_goal", "go_to_waypoint"}


def fail(message: str) -> None:
    raise SystemExit("candidate_audit=FAIL: " + message)


def numeric_assignment(tree: ast.AST, name: str) -> float:
    for node in ast.walk(tree):
        if not isinstance(node, (ast.Assign, ast.AnnAssign)):
            continue
        targets = node.targets if isinstance(node, ast.Assign) else [node.target]
        if not any(isinstance(target, ast.Name) and target.id == name for target in targets):
            continue
        value = ast.literal_eval(node.value)
        if not isinstance(value, (int, float)):
            fail(f"{name} is not numeric")
        return float(value)
    fail(f"missing assignment for {name}")
    raise AssertionError("unreachable")


def provider_contracts(tree: ast.AST) -> set[str]:
    contracts = set()
    for node in ast.walk(tree):
        if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        for decorator in node.decorator_list:
            if not isinstance(decorator, ast.Call):
                continue
            function = decorator.func
            if not (
                isinstance(function, ast.Attribute)
                and function.attr == "mcp"
                and isinstance(function.value, ast.Name)
                and function.value.id == "provider"
            ):
                continue
            if len(decorator.args) != 1:
                fail(f"provider.mcp decorator on {node.name} is not literal")
            contract = ast.literal_eval(decorator.args[0])
            if not isinstance(contract, str):
                fail(f"provider.mcp decorator on {node.name} is not a string")
            contracts.add(contract.rsplit("/", 1)[-1])
    return contracts


def cmd_vel_publishers(tree: ast.AST) -> set[str]:
    topic_names = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign) or not isinstance(node.value, ast.Call):
            continue
        if not any(
            isinstance(argument, ast.Constant) and argument.value == "/cmd_vel"
            for argument in node.value.args
        ):
            continue
        for target in node.targets:
            if isinstance(target, ast.Name):
                topic_names.add(target.id)

    publishers = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
            continue
        if node.func.attr != "publish" or not isinstance(node.func.value, ast.Name):
            continue
        if node.func.value.id in topic_names:
            publishers.add(node.func.value.id)
    return publishers


def audit_confirmation_gate(cli_tree: ast.AST) -> None:
    workflow = next(
        (
            node
            for node in cli_tree.body
            if isinstance(node, ast.FunctionDef) and node.name == "run_workflow"
        ),
        None,
    )
    if workflow is None:
        fail("CLI run_workflow is missing")

    gate_lines = [
        node.lineno
        for node in ast.walk(workflow)
        if isinstance(node, ast.With)
        and any(
            isinstance(item.context_expr, ast.Call)
            and isinstance(item.context_expr.func, ast.Name)
            and item.context_expr.func.id == "prepared_execution_gate"
            for item in node.items
        )
    ]
    confirmation_checks = [
        node.lineno
        for node in ast.walk(workflow)
        if isinstance(node, ast.If)
        and any(
            isinstance(name, ast.Name) and name.id == "confirmation"
            for name in ast.walk(node.test)
        )
        and any(isinstance(child, ast.Return) for child in node.body)
    ]
    if len(gate_lines) != 1:
        fail(f"expected one prepared execution gate, found {len(gate_lines)}")
    if len(confirmation_checks) < 2:
        fail("CLI cancellation and mismatch checks are missing")
    if max(confirmation_checks) >= gate_lines[0]:
        fail("CLI confirmation checks do not precede the execution gate")


def main() -> None:
    main_source = MAIN_PATH.read_text(encoding="utf-8")
    bridge_source = BRIDGE_PATH.read_text(encoding="utf-8")
    cli_source = CLI_PATH.read_text(encoding="utf-8")
    main_tree = ast.parse(main_source, filename=str(MAIN_PATH))
    bridge_tree = ast.parse(bridge_source, filename=str(BRIDGE_PATH))
    cli_tree = ast.parse(cli_source, filename=str(CLI_PATH))

    radius = numeric_assignment(
        main_tree, "_MOVE_OPTION_OPERATIONAL_MAX_RADIUS_M"
    )
    if not math.isclose(radius, 0.08, abs_tol=1e-12):
        fail(f"candidate radius is {radius}, expected exactly 0.08 m")

    manifest_text = MANIFEST_PATH.read_text(encoding="utf-8")
    manifest_capabilities = {
        value.rsplit("/", 1)[-1]
        for value in re.findall(r"^\s*- name:\s*(\S+)\s*$", manifest_text, re.M)
    }
    if manifest_capabilities != EXPECTED_CAPABILITIES:
        fail(f"unexpected manifest capabilities: {sorted(manifest_capabilities)}")
    if manifest_capabilities & FORBIDDEN_CAPABILITIES:
        fail("legacy direct-goal capability is exposed")

    toml_capabilities = {
        path.name.removesuffix(".v1.toml") for path in TOML_ROOT.glob("*.toml")
    }
    if toml_capabilities != EXPECTED_CAPABILITIES:
        fail(f"manifest/TOML mismatch: {sorted(toml_capabilities)}")

    decorated = provider_contracts(main_tree)
    if decorated != EXPECTED_CAPABILITIES:
        fail(f"unexpected decorated provider functions: {sorted(decorated)}")

    if cmd_vel_publishers(main_tree) or cmd_vel_publishers(bridge_tree):
        fail("a topic bound to /cmd_vel has a publish call")
    if re.search(r"rostopic\s+pub\s+/cmd_vel", main_source + bridge_source):
        fail("raw /cmd_vel shell publication found")

    for call in ast.walk(main_tree):
        if not isinstance(call, ast.Call) or not isinstance(call.func, ast.Attribute):
            continue
        if call.func.attr != "send_navigation_goal_pose":
            continue
        limits = {
            keyword.arg: ast.literal_eval(keyword.value)
            for keyword in call.keywords
            if keyword.arg in {"max_odom_path_m", "max_amcl_displacement_m"}
        }
        if limits != {
            "max_odom_path_m": 0.10,
            "max_amcl_displacement_m": 0.10,
        }:
            fail(f"prepared execution watchdog arguments changed: {limits}")

    audit_confirmation_gate(cli_tree)

    print("candidate_radius_m=0.08")
    print("odom_path_limit_m=0.10")
    print("amcl_displacement_limit_m=0.10")
    print("public_capability_count=8")
    print("direct_cmd_vel_publishers=0")
    print("legacy_navigation_tools_exposed=0")
    print("confirmation_precedes_execution_gate=1")
    print("candidate_audit=PASS")


if __name__ == "__main__":
    main()
