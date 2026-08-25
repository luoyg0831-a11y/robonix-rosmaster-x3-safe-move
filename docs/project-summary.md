# Robonix x ROSMASTER X3 项目总结

本项目将 ROSMASTER X3 的 ROS1 `move_base` 封装为受控移动流程。服务端从
`0.80 m` 到 `0.10 m` 由远到近搜索，只返回最远安全半径内最多 5 个前方候选；
用户选择后完成二次校验，CLI 展示目标并要求
准确输入“确认执行”，随后才允许打开执行门进行第三次校验和单次目标发布。

固定边界为：不直接发布 `/cmd_vel`，不接受任意坐标，不公开旧 direct-goal
工具，odom 路径与 AMCL 位移看门狗输入不超过 `1.00 m`，当前在 `0.90 m`
触发取消，token 单次 claim 且未知发布结果禁止重试。

2026-08-25 通过 SSH 重新获取当前部署文件并核对 SHA256。本仓库的 `main.py`、
`x3_bridge.py` 与 `move_base_cli.py` 已与小车当前文件逐字节一致；本地语法、四套
离线测试、安全审计、Catalog 元数据审计和公开快照审计均通过。

现场验证分层记录：此前 0.08 m 配置完成过 `move_base=SUCCEEDED` 的实车闭环；
0.90 m preview→prepare 完成无运动验证且未发送导航目标；当前部署的 0.80 m
候选尚未执行长距离实车验收。因此当前结论仍是“可审计候选”，不是长距离生产授权。

仓库现已按 Robonix Package Catalog 普通软件包流程提供根
`package_manifest.yaml`、发布元数据、能力路径、`config.spec`、构建/启动入口和
Catalog PR 片段。候选版本已推送到 GitHub 默认分支并通过仓库离线工作流；Catalog PR
已提交至 `syswonder/robonix-package-catalog#21`，Catalog CI 已通过并于 2026-08-05
合并。Catalog 条目仍引用同一仓库 URL，无需新增条目。
