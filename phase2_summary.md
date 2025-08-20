# Shadow Hand集成 - 第二阶段进度报告（Controller & Pipeline）

## 概览
- 分支: `replace_hand`
- 提交: “import shadowhand, no finger control” 已推送
- 目标: 将 Shadow Hand 接入 Where2Act 的数据收集与回放流程，验证最小可用控制（手的整体位姿 + 开/合占位），为后续手指独立控制打基础。

## 已完成
1. 控制器骨架
   - 新增 `code/robots/shadowhand_robot.py`：
     - 兼容 `panda_robot.py` 的接口：`move_to_target_pose()`、`open_gripper()`、`close_gripper()`、`wait_n_steps()`；
     - 识别 `palm` 作为 `hand_actor_id`，其它手指 link 作为 `gripper_actor_ids`；
     - 为可动关节设置 PD 驱动（stiffness/damping），并在开/合时设置目标与速度目标（0.0）。

2. 仿真与URDF适配
   - 默认切换至完整版 URDF：`shadowhand_ign_shadow_hand_fixed.urdf`；
   - 新增工具：
     - `tools/prepare_shadowhand_meshes.py`：为 IGN 版 URDF 创建 visual 链接，重写 collision 指向 visual；
     - `tools/normalize_shadowhand_scale.py`：统一 `<mesh>` scale=0.001（mm→m）。
   - 可在无头模式完成采集与回放；回放脚本支持 `robot_type` 选择并自动使用记录的 URDF。

3. Pipeline改动
   - `collect_data.py`：增加参数 `--robot_type` / `--shadowhand_urdf`，记录到 `result.json`；
   - `replay_data.py`：支持 headless 回放，按 `result.json` 或 CLI 选择 Shadow Hand，保存 `replay_start.png`/`replay_end.png`；
   - 新增调试开关 `--debug_dump_hand_qpos`：在关键时刻（到达目标位姿、开/合后）导出 `robot.get_qpos()` 与截图。

4. 验证产出
   - 采集与回放样例：
     - `results/40147_StorageFurniture_0_pushing_6/` 含 `hand_qpos_*.json` 与截图；
     - `results/40147_StorageFurniture_0_pushing_7/`、`_8/` 运行通过，生成常规图像与日志。
   - 预操作（远离物体）开/合验证：`results/40147_StorageFurniture_0_pushing_12/`
     - `hand_qpos_preop_after_open.json` 与 `hand_qpos_preop_after_close.json` 均已生成；
     - 关节最大变化：`MAX_ABS_DELTA_PREOP ≈ 3.52 rad`；
     - 可动关节数：24（来自 `hand_joint_info.json`）。

## 当前问题与分析
- 之前“开/合未产生显著 qpos 变化”的问题（如 `pushing_6` 的 `max_abs_delta=0`）在“预操作阶段”已复现并修复：
  - 处理：提升 PD（stiffness=400, damping=40），`wait_n_steps` 每步清零 `drive_velocity_target(0)`，并把开/合移动到接触检测前；
  - 结果：预操作阶段 qpos 显著变化（最大约 3.52 rad）。
  - 待确认：主流程（接触开启后）中开/合是否稳定执行并被 dump（可能仍受 `ContactError` 早退影响）。

## 下一步计划（短期修复）
1. 主流程内（接触开启后）的开/合可观测性
   - 保持接触检查，但在 try 块前后保留 dump，对 `after_close_for_push` / `after_open_for_pull` / `after_close_for_pull` 进行同样的 qpos 差异统计；
   - 若仍受 `ContactError` 影响，进一步放宽接触阈值或将关键 dump 置于触发接触之前。

2. 驱动细化
   - 仅对可动关节设目标；提高 PD（如 stiffness≈400、damping≈40）；
   - 在 `wait_n_steps` 中持续调用 `compute_passive_force`+`set_qf`；必要时延长等待步数（500→1000）。

3. 关节与限位诊断
   - 导出所有关节名称与 `limits`，统计不可动关节数；
   - 将关节按手指分组（TH/FF/MF/RF/LF），逐组测试：只动某一组的开/合目标，便于定位无响应关节。

4. 参考实现调研
   - 对比 Shadow Robot 官方/社区项目的开合实现（常见为 ROS 控制，或在仿真中为每个关节给定目标角并等待收敛）；
   - 若需要，考虑使用较简化的 `shadowhand_sapien_complete.urdf` 先验证 qpos 变化，再迁移至完整版。

## 风险与规避
- 物理不稳定/崩溃：优先在 headless 下减少渲染调用，必要时缩短步长或降低刚度。
- Mesh/尺度问题：已统一 scale，如视角遮挡再增大相机距离或调整姿态。

## 里程碑
- M2.1：预操作开/合 qpos 差异>0.02 rad，保存截图与统计（已完成，≈3.52 rad）。
- M2.2：在不接触物体的场景中，连续多次开/合均可重复（预计2-3天）。
- M2.3：带物体、放宽接触检查的采集中稳定记录到开/合差异（预计3-4天）。

## 结论
- 第二阶段已完成最小接入（控制器骨架+pipeline+可视化），当前主要任务是让开/合产生稳定的关节位姿变化并可被记录。上述短期计划将直接针对该问题迭代，完成后即可进入手指分组/抓取预成型等更高层动作的实现。
