### 使用 SAPIEN 资产（PartNet-Mobility）加载对象并采集数据（以 suitcase 100550 为例）

参考：[`SAPIEN Asset Browse`](https://sapien.ucsd.edu/browse)

---

#### 目标
- 在不破坏现有 Where2Act 数据集流程的前提下，优先使用 SAPIEN 官方的 PartNet-Mobility 资产。
- 为 `collect_data.py` 增加一个“资产模式”，可直接通过资产 ID 在线下载并加载 URDF。
- 以 `Suitcase`（ID: `100550`）为首个验证对象，支持 Shadow Hand 抓取/交互试验。

---

#### 前置准备
1) 注册并登录 SAPIEN 资产平台，获取访问 token（建议使用 `.edu` 邮箱）：
   - 浏览页与说明见：[`https://sapien.ucsd.edu/browse`](https://sapien.ucsd.edu/browse)
2) 在运行环境中配置 token：
   - 推荐通过环境变量：`export SAPIEN_ASSET_TOKEN="<your_token_here>"`
   - 也可通过 CLI 参数传入（见下文）。

---

#### API 使用要点（内置下载 + 直接加载）
SAPIEN 提供官方下载接口（示例）：

```python
import os
import sapien

token = os.environ.get('SAPIEN_ASSET_TOKEN')
urdf_file = sapien.asset.download_partnet_mobility(100550, token)
# 然后用场景的 URDF loader 加载
# urdf_loader.load(urdf_file)
```

说明：
- 接口会返回本地 URDF 路径，内部含有缓存策略；可重复使用，无需每次下载。
- 对于 `Suitcase (100550)`：页面中标注 `remeshed: yes / partnet: no`，可直接尝试加载。

---

#### 代码改动规划（最小侵入）
在 `collect_data.py` 新增参数与分支，保持与原有 `dataset` 模式并存：

- 新增 CLI 参数（建议）：
  - `--object_mode {dataset,simple,asset}`：新增 `asset` 模式用于 SAPIEN 在线资产；
  - `--sapien_asset_id`：整数资产 ID（例如 `100550`）；
  - `--sapien_asset_token`：可选；未提供时从 `SAPIEN_ASSET_TOKEN` 环境变量读取；
  - `--sapien_asset_cache`：可选缓存目录（若 API 支持自定义）；
  - 结果命名约定：若 `object_mode=asset`，建议将 `shape_id` 用作 `asset<id>` 以便区分，如 `asset100550`。

- 加载分支伪码（仅说明，不改变现有逻辑）：
  1) 若 `object_mode == 'asset'`：
     - 读取 token（优先 CLI，其次环境变量）；
     - 调用 `sapien.asset.download_partnet_mobility(args.sapien_asset_id, token)` 获得 `object_urdf_fn`；
     - 其余流程（材质、state 随机、静止等待、相机、掩码、采样、控制）沿用原逻辑；
     - 在 `result.json` 记录元信息：`object_mode=asset`、`asset_id`、`asset_urdf`、`asset_category`（若可获）等。
  2) 否则与现有 `dataset`/`simple` 分支保持不变。

无需修改 `env.py`：
- 资产返回的是 URDF 文件，`Env.load_object()` 已支持通过 `urdf_loader.load(urdf_file, material)` 加载。
- 若资产对象无 1-DOF 关节，`movable_link_ids` 为空的情况在现流程会导致“没有可移动像素”；
  - 若要支持“非关节物体抓取”，可复用 `replace_object_with_box.md` 中的思路，让“整物体链接”作为可交互目标（可选改动）。

---

#### 运行示例（以 suitcase 100550，Shadow Hand，headless）
假设已在 shell 设置 token：`export SAPIEN_ASSET_TOKEN=...`

```bash
xvfb-run -a python code/collect_data.py asset100550 Suitcase 0 pushing \
  --no_gui \
  --robot_type shadowhand \
  --shadowhand_urdf ./robots/shadowhand/shadowhand_ign_shadow_hand_fixed.urdf \
  --object_mode asset \
  --sapien_asset_id 100550 \
  --debug_dump_hand_qpos
```

说明：
- 若你倾向显式传 token，可增加 `--sapien_asset_token $SAPIEN_ASSET_TOKEN`。
- `shape_id/category/cnt_id` 主要用于命名；`asset100550/Suitcase/0` 仅作结果区分。
- 运行完成后，结果位于 `code/results/asset100550_Suitcase_0_pushing_<trial>/`。

---

#### 验证与调试建议
- 确认 `log.txt` 中打印的 `object_urdf_fn` 指向资产下载的 URDF 路径；
- 若首次加载失败（权限或网络），检查：
  - token 是否有效、是否登录授权；
  - 服务器是否可访问外网；
  - 资产 ID 是否存在；
- 若资产为“非关节”对象而需要抓取：
  - 按 `replace_object_with_box.md` 的建议，放宽 `env.py` 对可移动部件的假设，或直接使用 `simple box` 测试抓取。

---

#### 数据记录扩展（建议）
在 `result.json` 增加以下字段，便于追溯：
- `object_mode`: "asset"
- `asset_provider`: "SAPIEN-PartNet-Mobility"
- `sapien_asset_id`: 100550
- `asset_urdf`: "<下载到的本地 URDF 路径>"
- `asset_url`: "浏览器可视链接（可选）"

---

#### 清理与缓存
- 资产下载文件用于复用，不建议在每次运行后删除（避免重复下载）。
- 如临时生成了中间文件（例如为了适配路径重写的临时 URDF），请在脚本结束时清理，避免污染仓库。

---

#### 参考链接
- SAPIEN 资产浏览与说明：[`https://sapien.ucsd.edu/browse`](https://sapien.ucsd.edu/browse)


