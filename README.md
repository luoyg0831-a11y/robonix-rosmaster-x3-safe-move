# Robonix x ROSMASTER X3 短距离安全移动

[English](README.en.md)

这是 Yahboom ROSMASTER X3 的 ROS1 短距离安全导航候选包。当前版本
`0.2.0-candidate.1` 按 [Robonix Package Catalog 发布流程](https://robonix-book.syswonder.org/integration-guide/package-catalog)
整理：仓库根目录提供 `package_manifest.yaml`，发布名为
`robonix.primitive.yahboom.rosmaster_x3.safe_move`。

## 操作流程

```text
扫描并展示最多 5 个服务端候选
  -> 用户选择编号
  -> 第二次安全校验并展示目标
  -> 用户必须准确输入“确认执行”
  -> 打开进程内执行门并完成第三次校验
  -> 单次发布 move_base 目标并关联观察结果
```

输入 `q` 或任何错误确认都不会进入执行门。`--dry-run` 完成候选与准备
校验后直接退出，不请求确认，也不发送目标。

## 固定安全边界

- 候选半径最大 `0.08 m`。
- odom 累计路径与 AMCL 位移看门狗参数均不得超过 `0.10 m`。
- 不直接发布 `/cmd_vel`；公开 manifest 只有 8 个受控 capability。
- 不公开旧 `send_nav_goal`、`go_to_waypoint` 工具。
- 测试、构建和包元数据不授权实车移动。

## 当前证据

| 项目 | 结果 |
|---|---|
| 完整生产 0.07 m `main.py` | SHA256 `9d8a40fab4c5e734d94ac79f31c53c9f06d139788c30f239d6d11a32e783508e` |
| 公开 0.08 m 候选 `main.py` | SHA256 `a30eef41f392b440d6902b00f05811b6d7ccd463813d292e9bd5c06de91274bd` |
| 生产到候选 diff | 1 个 hunk，2 行删除、3 行增加，仅注释与 `0.07 -> 0.08` |
| 日期化远端备份 | PASS，282 个文件，修改生产文件前完成 |
| Jetson Python 3.10 语法检查 | PASS |
| 四套 Jetson 离线测试 | PASS |
| `rbnx validate/build/validate` | PASS，独立 staging |
| 真实 ROS 无目标 dry-run | INCOMPLETE：CLI 返回 `amcl_unavailable` |
| 目标或非零速度 | 均未观察到；未执行实车移动 |

详见[验证证据](evidence/validation-2026-08-04.md)、[远端盘点](evidence/remote-inventory-2026-08-04.md)
和[部署映射](docs/deployment-map.md)。

## 演示视频

[audio-test01.mp4](https://github.com/luoyg0831-a11y/robonix-rosmaster-x3-safe-move/releases/download/v0.2.0-candidate.1/audio-test01.mp4)
作为 `v0.2.0-candidate.1` 预发布版本的 Release 资产提供。文件 SHA256 为
`c2a8dc1c3b187205f5efb5392562f0e1d41d5c58575656ca2cd10c18d8494e88`。
该视频用于项目演示，不替代 0.08 m 候选版本的生产部署与实车验收。

## 离线检查

```bash
python -m py_compile \
  jetson/main.py jetson/x3_bridge.py jetson/scripts/move_base_cli.py \
  jetson/stage1_unit_test.py jetson/stage2_unit_test.py \
  jetson/x3_bridge_navigation_unit_test.py \
  jetson/tests/move_base_cli_unit_test.py \
  tools/audit_candidate.py tools/audit_public_snapshot.py
python jetson/stage1_unit_test.py
python jetson/stage2_unit_test.py
python jetson/x3_bridge_navigation_unit_test.py
python jetson/tests/move_base_cli_unit_test.py
python tools/audit_candidate.py
python tools/audit_public_snapshot.py
```

`.github/workflows/offline-tests.yml` 在 push 和 pull request 时执行同一组离线检查。

## Catalog 发布

包仓库准备完成后，Catalog PR 只应向 `syswonder/robonix-package-catalog`
的 `catalog.yaml` 增加一条 `name + repo`。所需片段和发布前检查见
[Catalog 提交说明](docs/catalog-submission.md)。候选版本已推送到 GitHub 默认分支，
离线工作流已通过；Catalog PR 已提交为
[#21](https://github.com/syswonder/robonix-package-catalog/pull/21)，Catalog CI 已通过并于
2026-08-05 合并。

## 许可证

原创工程代码按 Apache-2.0 发布。Robonix、ROS、Yahboom 文件和 codegen
产物仍受各自许可证约束；厂商 ROS 配置只记录来源、参数结论与哈希，不复制全文。
