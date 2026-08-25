# Robonix × ROSMASTER X3 安全移动

[English](README.en.md)

这是 Yahboom ROSMASTER X3 的 ROS1 导航组件，当前发布版本为
`0.3.0-candidate.1`。Robonix Catalog 包名：
`robonix.primitive.yahboom.rosmaster_x3.safe_move`。

## 使用方式

先做无运动检查：

```bash
move_base --dry-run
```

确认环境和候选点正常后，再运行：

```bash
move_base
```

程序会列出最多 5 个候选点。用户选择编号后，系统重新检查 AMCL、全局代价地图和
`/move_base/make_plan`，随后显示目标信息。只有完整输入“确认执行”，程序才会进行
第三次检查并向 `move_base` 发布一次目标。在确认提示处，输入 `q` 或任何不是
“确认执行”的内容都会取消本次操作。

## 运行限制

- 候选半径按 `0.80 m`、`0.70 m`……`0.10 m` 的顺序检查，只保留通过检查的最远一档。
- 候选方向限制在车头前方 `±30°`。
- 执行接口不接收目标坐标，只接受由 `prepare_selected_move` 为当前候选会话签发的一次性令牌。
- 项目代码不直接发布 `/cmd_vel`；速度指令由 ROS `move_base` 生成。
- odom 累计路径和 AMCL 位移的参数上限是 `1.00 m`。使用该上限时，桥接层在
  `0.90 m` 触发取消，预留 `0.10 m` 停车余量。

代码中的 `0.93 m` 是准备接口的目标距离硬上限，不是候选搜索半径。编号候选流程还会
单独执行两项限制：候选本身不超过 `0.80 m`，且预览后小车位置变化不得超过 `0.03 m`。
这三个数值分别用于不同检查，不能相加理解。

## 验证状态

| 项目 | 结果 |
|---|---|
| 本地语法检查、四套单元测试和仓库检查 | 通过 |
| Jetson Python 3.10.20 语法检查和 CLI 单元测试 | 通过 |
| 仓库中的 provider、bridge、CLI 与小车文件哈希对比 | 一致 |
| 早期 0.08 m 配置实车闭环 | `move_base=SUCCEEDED` |
| 较早的 0.90 m 配置 preview→prepare 无运动检查 | 连续 3 次通过，未发送目标 |
| 当前 0.80 m 配置长距离实车验收 | 尚未执行 |

详细记录见 [2026-08-25 验证报告](evidence/validation-2026-08-25.md)。目前不能把
较早的 0.90 m 无运动检查视为行驶测试，也不能据此认定 0.80 m 配置已完成实车验收。

## 演示视频

[audio-test01.mp4](https://github.com/luoyg0831-a11y/robonix-rosmaster-x3-safe-move/releases/download/v0.3.0-candidate.1/audio-test01.mp4)

- 发布版本：`v0.3.0-candidate.1`
- 文件大小：`65,499,360` 字节
- SHA256：`ccec8554a6a1a68f3af2f2e81b3ef410870907cda46a57e78dfed2872839d5c1`

## 开发检查

GitHub Actions 会在 push 和 pull request 时运行语法检查、四套单元测试、Catalog
元数据检查、运行限制检查和文件校验和检查。工作流定义见
[offline-tests.yml](.github/workflows/offline-tests.yml)。

## Catalog

[Robonix Catalog PR #21](https://github.com/syswonder/robonix-package-catalog/pull/21)
已于 2026-08-05 合并。Catalog 条目引用本仓库地址，版本信息由根目录的
`package_manifest.yaml` 提供。

## 许可证

本仓库原创代码采用 Apache-2.0。Robonix、ROS 和 Yahboom 组件仍按各自许可证发布。
厂商 ROS 文件和现场地图未提交到本仓库；这里只保留必要的参数说明与哈希。
