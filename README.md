# Robonix x ROSMASTER X3 短距离安全移动

[English](README.en.md)

这是 Yahboom ROSMASTER X3 的 ROS1 安全导航候选包。当前版本
`0.3.0-candidate.1` 按 [Robonix Package Catalog 发布流程](https://robonix-book.syswonder.org/integration-guide/package-catalog)
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

- 候选点从 `0.80 m` 到 `0.10 m` 由远到近搜索，只返回最远安全半径，
  且限制在车头前方 `±30°`。
- odom 累计路径与 AMCL 位移看门狗参数均不得超过 `1.00 m`；当前桥接层
  在 `0.90 m` 触发取消，为控制、通信和物理制动保留 `0.10 m`。
- 二次、三次复核最多接受 `0.93 m`，仅用于吸收 preview 后不超过
  `0.03 m` 的 AMCL 静止漂移，不会扩大候选搜索半径。
- 不直接发布 `/cmd_vel`；公开 manifest 只有 8 个受控 capability。
- 不公开旧 `send_nav_goal`、`go_to_waypoint` 工具。
- 测试、构建和包元数据不授权实车移动。

## 当前证据

| 项目 | 结果 |
|---|---|
| 当前 Jetson `main.py` | SHA256 `a3c2a5f4b94def0f84b981c8fa1837d99196622717cf576df9603b57cc932e4b`；与本仓库一致 |
| 当前 Jetson `x3_bridge.py` | SHA256 `425baf888847ca3b2e264ed2e3b50c65118a7ba77718931e55ddac8d503927aa`；与本仓库一致 |
| 当前 Jetson `move_base_cli.py` | SHA256 `4f2637d4bac646bf66ba52641e7511831909e42478b5d115798cff70f0997c9d`；与本仓库一致 |
| 离线语法、安全审计和四套测试 | PASS |
| 0.08 m 实车闭环 | PASS：`move_base=SUCCEEDED`，最终停止 |
| 0.90 m 预览/二次复核无运动验证 | PASS；未发送导航目标 |
| 当前 0.80 m 候选实车长距离验收 | 未执行 |

详见[最新验证证据](evidence/validation-2026-08-25.md)、[历史验证证据](evidence/validation-2026-08-04.md)
和[部署映射](docs/deployment-map.md)。

## 演示视频

[audio-test01.mp4](https://github.com/luoyg0831-a11y/robonix-rosmaster-x3-safe-move/releases/download/v0.3.0-candidate.1/audio-test01.mp4)
作为 `v0.3.0-candidate.1` 预发布版本的 Release 资产提供。文件大小
`65,499,360` 字节，SHA256 为
`ccec8554a6a1a68f3af2f2e81b3ef410870907cda46a57e78dfed2872839d5c1`。
视频用于项目演示，不替代当前 0.80 m 候选的生产授权与实车验收。

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
