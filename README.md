# Multi-UAV-Path-Planning

城市复杂环境下多无人机（UAV）三维路径规划 MATLAB 工程。基于三维 B 样条 / Bézier 轨迹表示，结合 CA（Chronos / 文化算法 CAv9x）优化控制点，支持 P2P 静态障碍避障与 TOUR 多机任务点巡访两类场景，并提供交互式 App、收敛曲线与三维可视化。

## 功能特性

- 三维城市环境建模：cube / cylinder / sphere / row / tower / nofly / tree / terrain 等多类障碍，支持 easy / medium / hard 三档难度（城市密度与复杂度递增）。
- 多无人机协同：无人机数量可在 3~30 之间配置，支持机间最小间隔惩罚与任务点分配（tour 模式）。
- 轨迹优化：以 B 样条控制点为决策变量，CA 算法最小化路径长度、平滑度、障碍穿透、越界、机间分离等加权代价。
- 通信链路约束（阶段 E，可开关）与时变车辆碰撞惩罚（阶段 D）。
- 交互式 App（纯 MATLAB uifigure，无需 App Designer）：场景切换、参数调节、3D 可视化、收敛曲线、轨迹动画。

## 目录结构

| 文件 | 说明 |
| --- | --- |
| `mu_config.m` | 构造城市复杂环境场景配置（难度、机数、障碍、代价权重） |
| `mu_run_planner.m` | 规划主流程（场景构建 → CA 优化 → 轨迹生成） |
| `mu_bspline.m` / `mu_traj_gradient.m` | B 样条轨迹与梯度计算 |
| `mu_cost_*.m` | 路径 / 点对点（p2p） / 巡访（tour）代价函数 |
| `mu_obstacle_dist*.m` | 障碍距离与穿透惩罚 |
| `mu_city_layout.m` / `mu_geo*.m` | 城市布局与地理坐标变换 |
| `MUAVPlanner.m` | 交互式规划 App 入口 |
| `demo_muav.m` | 三套难度 + 多机数演示脚本，导出 EPS 与 XLSX |
| `*.bat` | Windows 一键运行 / 回归 / 验证脚本 |

## 环境要求

- MATLAB（推荐 R2020b 及以上，使用 `uifigure` 等 App 组件需对应版本支持）
- 依赖父目录中的 `CAv9x` 优化器（代码通过 `addpath(fileparts(appRoot))` 引用）

## 快速开始

```matlab
% 运行演示（生成 easy/medium/hard 三档难度 + tour 场景结果）
demo_muav

% 或直接启动交互式 App
MUAVPlanner
```

运行 `demo_muav` / `run_muav` 后，输出默认写入 `results/` 子目录，包含三类产物：

- **完整结果 `.mat`**（`muav_<tag>_<时间戳>.mat`）：自描述、可复现的规划结果，含 `scene`、每条 `trajs`、`bestX`（CA 最优决策变量，可重新解码轨迹）、`bestCost`、`curve`、`opt`、`metrics` 与 `meta`（`schemaVersion`/`savedAtUTC`/`savedAtLocal`/MATLAB 版本/字段清单等）。可经 `mu_load_result` 读回做轨迹回放与二次评估。
- **实验集合 `.mat`**（`exp_<name>.mat`，可选）：同一实验名下多次运行追加进单个文件的 `runs` 字段，便于跨用例批量对比（如超参扫描）。
- **可读索引** `results/index.json`：每次保存后自动追加一条记录（文件/标签/模式/代价），无需逐个 `load` 即可浏览全部结果。
- **多视角图** `run_<tag>_<视角>_<时间戳>.png/.eps`（300dpi），由 `mu_export_views` 生成，覆盖看清路径所需的互补视角：
  - `persp` 透视 3D 总体态势、`top` 正俯视（看清 XY 平面走向与绕障）、`sideX`/`sideY` 正侧视（看清高度分层与楼高关系）、`closeup` 近景放大（聚焦最复杂单机局部绕障）。
  - `run_<tag>_convergence_<时间戳>.png/.eps`：CA 收敛曲线（P2P 用例）。
- **汇总表** `run_muav_results_<时间戳>.xlsx`：各用例代价、穿透、长度等指标。

读取示例：

```matlab
[res, meta] = mu_load_result('latest', 'results');          % 读最新一份单用例
[paths, n]  = mu_load_result('list',   'results');          % 列出全部结果路径
idx         = mu_load_result('index',  'results');          % 读取可读索引（JSON）
runs        = mu_load_result('experiment', 'exp1', 'results'); % 读取实验集合
```

存储设计要点：保存时把 `terrainF` 等不可序列化的函数句柄替换为 `terrainFInfo` 并记录来源，读取端给出用 `mu_city_layout` 重建的提示；保存前校验 `trajs` 条数与 `scene.nUAV` 一致性；采用 `-v7.3` 格式（支持大数组与压缩），直接写最终文件名、零临时文件残留。


## 许可证

参见 [LICENSE](LICENSE) 文件。
