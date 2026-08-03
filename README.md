# Robonix x ROSMASTER X3 短距离安全移动

[English](README.en.md)

这是 Yahboom ROSMASTER X3 在 Robonix 中执行短距离安全导航的 ROS1 工程总结。用户在 Jetson 的 `robonix` 环境中运行 `move_base`，系统扫描并显示最多 5 个安全候选点；用户选择编号后，系统完成多阶段安全校验，再向 ROS `move_base` 单次发送目标并返回可核验的运动结果。

> 当前版本：`0.1.0-local-preview`。这是从 Windows 本地快照整理的公开候选版，不是 Jetson 生产基线。0.08 m 方案已通过本地语法和离线单元测试，但尚未完成 Jetson 部署、真实 ROS 无运动回归和最终硬件验收。

## 用户流程

```text
move_base
  -> 读取 AMCL、全局 costmap 和 /move_base/make_plan
  -> 返回 1 至 5 个安全候选
  -> 用户输入候选编号或 q 取消
  -> 二次校验并创建一次性 token
  -> 打开受控执行门并执行第三次校验
  -> 向 /move_base_simple/goal 单次发布目标
  -> 关联 move_base 状态、/cmd_vel、/vel_raw、AMCL 和 odom
  -> 返回成功或明确失败原因
```

## 安全边界

- 不直接发布 `/cmd_vel`；控制速度只能由 ROS `move_base` 产生。
- 候选位置由服务端生成，调用者不能提交任意坐标。
- 选择后执行二次校验，发布前执行第三次独立校验。
- 使用一次性 token、原子 claim、防重放和执行时间预算。
- 目标发布结果未知时不重试同一 token。
- 监控目标接收、终态、速度、AMCL、odom 和最终静止。
- 本地候选半径为 0.08 m；受充电线约束的累计路径上限为 0.10 m。
- 旧 `send_nav_goal` 和 `go_to_waypoint` 未列入公开 capability manifest。

详细规则见 [安全模型](docs/safety-model.md)。

## 当前验证状态

| 项目 | 状态 |
|---|---|
| Python 语法检查 | 已在 Windows Python 3.13 通过 |
| Stage 1 候选生成单元测试 | 已通过 |
| Stage 2 token、并发和执行门测试 | 已通过 |
| ROS bridge 导航观察器单元测试 | 已通过 |
| CLI 单元测试 | 已通过 |
| 早期约 0.25 m 实车移动 | 有视频证据，不能替代当前版本验收 |
| 0.09 m 实车测试 | 产生实际运动，最终因 0.10 m 路径看门狗被取消 |
| 0.08 m Jetson 构建与 dry-run | 尚未完成 |
| 0.08 m 最终硬件验收 | 尚未完成 |

完整状态见 [验证状态](docs/validation-status.md)。

## 本地测试

在仓库根目录运行：

```bash
python -m py_compile jetson/main.py jetson/x3_bridge.py jetson/scripts/move_base_cli.py
python jetson/stage1_unit_test.py
python jetson/stage2_unit_test.py
python jetson/x3_bridge_navigation_unit_test.py
python jetson/tests/move_base_cli_unit_test.py
```

这些测试使用 fake ROS/Robonix 模块，只验证离线逻辑，不代表真实机器人通过。

## 目录

```text
jetson/
  main.py                         Robonix ROS1 primitive provider
  x3_bridge.py                    roslibpy/rosbridge 适配和导航观察器
  scripts/move_base_cli.py        用户交互闭环
  capabilities/                   公开安全 capability 和 IDL
  stage1_unit_test.py             候选生成测试
  stage2_unit_test.py             执行安全语义测试
  x3_bridge_navigation_unit_test.py
  tests/move_base_cli_unit_test.py
docs/                             架构、安全、验证和迁移说明
evidence/                         脱敏后的实验结论，不保存原始现场日志
```

## 部署说明

本地快照缺少 Jetson 当前生产包中的 `scripts/build.sh`、`scripts/start.sh`、生成代码和完整 ROS 配置来源，因此本版本用于代码审查和成果总结，不能直接作为全新设备的一键部署包。恢复 SSH 后需要从 Jetson 只读核对并补齐这些文件，再执行 `rbnx validate/build/validate` 和真实 ROS 验证。详见 [部署缺口](docs/deployment-gap.md)。

## 与上游 Robonix 的关系

本工程基于较早版本的 Python Robonix API、ROS1 Melodic、rosbridge 和 `move_base`。当前 [syswonder/robonix](https://github.com/syswonder/robonix) `dev` 仍使用 `package_manifest.yaml`，但其 Python API、生命周期和 capability contract 已演进，并以 ROS2 为主要集成路径。本仓库是兼容性项目总结，不宣称可直接安装到当前上游。迁移方案见 [上游对齐](docs/upstream-alignment.md)。

## 许可证

本地历史 manifest 将原创工程代码声明为 Apache-2.0。第三方 Robonix、ROS、Yahboom 文件和生成代码仍受各自许可证约束；当前快照没有收录来源尚未确认的 Yahboom ROS 配置全文。
