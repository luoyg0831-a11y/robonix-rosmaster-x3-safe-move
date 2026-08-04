# Robonix x ROSMASTER X3 项目总结

本项目将 ROSMASTER X3 的 ROS1 `move_base` 封装为受控短距离移动流程。服务端
生成最多 5 个 `0.08 m` 内候选；用户选择后完成二次校验，CLI 展示目标并要求
准确输入“确认执行”，随后才允许打开执行门进行第三次校验和单次目标发布。

固定边界为：不直接发布 `/cmd_vel`，不接受任意坐标，不公开旧 direct-goal
工具，odom 路径与 AMCL 位移看门狗不超过 `0.10 m`，token 单次 claim 且未知
发布结果禁止重试。

2026-08-04 已完成生产 0.07 m 文件的完整下载、SHA256 和逐字节 diff，并在任何
候选上传前建立日期化远端备份。候选仅进入独立 staging；Jetson Python 3.10
语法、四套离线测试、`rbnx validate/build/validate` 均通过，生产 SHA 未变化。

真实 ROS 无目标 dry-run 启动了已核对的厂商栈，但候选 fresh-AMCL 检查返回
`amcl_unavailable`，因此按失败关闭。订阅 guard 记录目标数和非零速度数均为 0，
没有执行实车移动。当前结论是“可审计的 0.08 m 源码与软件包候选”，不是生产
部署版或硬件验收版。

仓库现已按 Robonix Package Catalog 普通软件包流程提供根
`package_manifest.yaml`、发布元数据、能力路径、`config.spec`、构建/启动入口和
Catalog PR 片段。候选版本已推送到 GitHub 默认分支并通过仓库离线工作流；Catalog PR
已提交至 `syswonder/robonix-package-catalog#21` 且 Catalog CI 已通过，当前等待维护者
合并；0.08 m 版本仍尚未完成生产部署与实车验收。
