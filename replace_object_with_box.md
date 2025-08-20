## 用简单 Box/Cube 替换当前物体（用于抓取数据收集）

### 背景与目标
- 现有 `collect_data.py`/`env.py` 基于 Where2Act 的关节物体范式（需要可动关节与可移动部件掩码）。
- 为了在 Shadow Hand 上快速尝试“抓取”数据采集，我们希望把被操作物体替换为一个简单的 Box/Cube（可无关节）。
- 要求：
  - 与当前仿真环境（SAPIEN）无缝；
  - 便于在无头服务器上运行（`xvfb-run -a`）；
  - 尽量少改动主流程，可独立开关。

---

### 方案总览（推荐路径）
1) 提供一个“简单立方体 URDF”（单链接、无关节）。
2) 对 `env.py` 做小幅兼容增强：若加载的物体无 1-DOF 关节，则将“整物体链接”视为可交互链接，掩码生成与接触检查按整物体处理。
3) 对 `collect_data.py` 增加对象替换与抓取原语选项：
   - `--object_urdf_override`: 指定自定义 URDF 路径（如简单 Box）；
   - `--object_mode {dataset,simple}`: 控制是否走原数据集或简单物体分支；
   - `--box_size`: 可选，控制立方体尺寸（米）。
4) 新增一个“grasping”原语：远离物体处开手、靠近目标位姿、闭手、可选轻微抬升/保持，记录抓取结果与手部关节轨迹。

---

### 1. 简单 Box 的 URDF 模板
建议将简单物体放在 `code/objects/simple/`（新建目录）下，示例：`box_06m.urdf`（边长 0.06m）。

```xml
<?xml version="1.0"?>
<robot name="simple_box">
  <link name="box_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.2"/>
      <!-- 简单近似惯性 -->
      <inertia ixx="1e-3" ixy="0" ixz="0" iyy="1e-3" iyz="0" izz="1e-3"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.06 0.06 0.06"/>
      </geometry>
      <material name="gray">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.06 0.06 0.06"/>
      </geometry>
    </collision>
  </link>
  <!-- 无关节（单链接物体） -->
</robot>
```

可按需复制多份（0.04m、0.08m 等），或在代码侧通过 `--box_size` 自动生成临时 URDF（注意按需清理临时文件）。

---

### 2. env.py 兼容增强（最小修改点）
现状：`Env.load_object()` 通过遍历关节（`get_dof()==1`）收集 `movable_link_ids`，并依赖这些 ID 生成可移动部件掩码、目标部件等。

建议改动：
- 当加载的 URDF 中不存在任意 `get_dof()==1` 的关节：
  - 将 `self.movable_link_ids` 设为“全部链接 ID”（或仅包含单链接 `box_link` 的 ID）；
  - `set_target_object_part_actor_id()` 正常设置为该链接；
  - `get_target_part_qpos()` 在无关节时返回 0.0（占位）；
  - `start_checking_contact()`/`check_contact_is_valid()` 在 simple 模式下：
    - 允许“任一箱体链接”与“任一手指链接”产生首次有效接触（非严格）；
    - 仍保留“第一步不应碰撞”的约束，避免初始交叠。

这样可以让 `Camera.get_movable_link_mask(object_link_ids)` 正常工作（整物体掩码=可交互区域），不破坏现有渲染与日志逻辑。

---

### 3. collect_data.py 扩展点
新增/调整 CLI：
- `--object_mode {dataset,simple}`：simple 时走自定义物体分支；
- `--object_urdf_override`：提供 simple 物体的 URDF 路径；
- `--box_size`（可选）：未提供 override 时，按该尺寸生成临时 URDF（运行结束清理）。

加载逻辑：
- 若 `object_mode==simple`：
  - 若提供 `--object_urdf_override`，直接使用；
  - 否则根据 `--box_size` 生成临时 box URDF（放在 `code/.tmp/` 下，运行完清理）。
- 其他流程保持不变：
  - 正常等待物体静止、采样像素（此时掩码=整物体）；
  - 生成 EE 目标位姿；
  - 进入主循环。

新增原语：`grasping`
- 典型步骤（Shadow Hand）：
  1) 预操作开手（远离物体，避免接触中断）；
  2) 逼近至目标位姿（物体表面法线反方向 8–12cm 处）；
  3) 闭手（抓取），等待稳定；
  4) 可选：小幅上抬（2–5cm）用于验证抓取稳定性；
  5) 记录 `qpos`、接触与是否成功等。

抽样策略：
- 对于 simple box，可从其包围盒 6 个面中随机选一面、随机一点，法线方向作为“up”；
- 也可直接采样整物体掩码中的像素位置（与现流程一致）。

---

### 4. 运行示例（headless，where2act 环境）
先准备一个简单 URDF：`code/objects/simple/box_06m.urdf`（如上模板）。

```bash
xvfb-run -a python code/collect_data.py simpleBox Box 0 grasping \
  --no_gui \
  --robot_type shadowhand \
  --object_mode simple \
  --object_urdf_override ./code/objects/simple/box_06m.urdf \
  --debug_dump_hand_qpos
```

说明：
- `shape_id/category/cnt_id` 目前用于结果路径命名，可自定义（如 `simpleBox Box 0`）。
- 若不提供 `--object_urdf_override`，可使用 `--box_size 0.06` 自动生成临时 box URDF（运行结束清理该临时文件）。

---

### 5. 结果与验证
- 期望在 `results/[...]` 目录下看到：
  - 常规 `rgb.png`、`interaction_mask.png`、`cam_XYZA.h5` 等；
  - Shadow Hand 的 `hand_qpos_*.json` 快照；
  - 若开启 grasping：抓取后抬升阶段的帧图和 qpos 变化；
  - `result.json` 中包含 `robot_type=shadowhand`、`robot_urdf`、`object_mode=simple`、`object_urdf_override`、`box_size` 等元信息。

---

### 6. 后续可选增强
- 加入多尺寸/多形状（圆柱、长方体）simple 物体库，统一放置于 `code/objects/simple/`；
- 对 `env.py` 新增 `simple_object=True` 内部标志，`check_contact_is_valid()` 在 simple 模式下更宽松；
- 为 grasping 添加稳定性判据（抬升后仍保持接触/相对位姿稳定若干步）；
- 批量采集脚本：按尺寸×姿态×接近面批量生成数据。

---

### 7. 改动清单（最小化改动）
1) 新增：`code/objects/simple/box_06m.urdf`（和可选的其它尺寸）。
2) 修改：`env.py`
   - `load_object()`：当无 1-DOF 关节时，`movable_link_ids = all_link_ids`；
   - `get_target_part_qpos()`：无关节返回 0.0；
   - `check_contact_is_valid()`：simple 模式下允许“任一箱体链接”与手指链接的首次有效接触。
3) 修改：`collect_data.py`
   - 新增参数：`--object_mode`、`--object_urdf_override`、`--box_size`；
   - 新增原语：`grasping`；
   - simple 模式下加载 override URDF 或临时 URDF。

以上改动即可在不破坏现有 Panda/Shadow Hand 与原数据集流程的前提下，增量支持“简单物体抓取数据”的采集。


