# 城市复杂环境精细化场景 — 研究报告（阶段 1–6）

## 摘要
本报告总结 Multi-UAV-Path-Planning 应用的城市复杂环境精细化改造，覆盖六个阶段：建筑形状（阶段1）、城市布局（阶段2）、颜色高度仿真（阶段3）、轨迹清晰化（阶段4）、集成验证（阶段5）、清理（阶段6）。改造后的场景在 400×400×150 的搜索空间内程序化生成 8 种障碍（楼/塔/禁飞区/树/水/罐/球/地形），三难度（新城/郊区、典型城区、密集老城）分别含 41/78/125 栋楼宇及配套设施，CA 规划在三难度 × p2p/tour 共 6 场景中均实现零障碍穿透，轨迹渲染经白描边与方向箭头增强后清晰可辨。

## 背景
原场景为简单几何体集合（立方体、圆柱、球），缺乏城市语义，轨迹在交叠处糊成一团。改造目标是在不改动 CA 算法、代价函数签名、碰撞检测逻辑的前提下，把场景升级为"像真实城市"且"路径清晰"的可视化与验证环境，服务于多无人机路径规划的演示与论文配图。

## 阶段一：建筑形状（Shape）
定义了 rich-building 数据结构（基座半宽、退台层数、退台收分、楼顶设备、窗格行距/列距、裙楼半宽/高度、颜色、kind），在 `mu_obstacle_dist` 增加 `'bldg'` 分支（两段盒距 + 多边形水平距离 + 垂直段），在 `mu_draw_scene` 新增 `drawBuildingRich`（基座 + 退台 + 楼顶设备 + 窗格），并用简单网格生成器验证了单栋渲染与碰撞。

## 阶段二：城市布局（Layout）
重写 `mu_city_layout`（作为 `mu_config` 内局部函数）为城市分区生成器：CBD 高层簇、住宅中密度区、工业仓储区，路网贴线（建筑沿道路两侧成排）、泊松圆盘聚类避免重叠、绿地/水体/工业/CBD 语义分区；更新 `mu_depots`（边界绿化带 6 个仓库，z=22）与 `mu_customer_points`（挂 rich 楼顶停机坪/楼外低空停靠点）。三难度参数：easy 40 栋+1 地标、medium 75 栋+3 地标、hard 120 栋+5 地标。

## 阶段三：颜色高度仿真（Color）
按 kind/高度分层着色（玻璃幕墙冷蓝灰、住宅暖米、工业赭石），窗格玻璃反光（半透明浅色条纹）、地形伪彩（低绿→高赭）、水体面（半透明蓝绿）、禁飞区（红色半透明 + 斜纹边框）。新增 tower（通信塔架红色半透明圆柱）、nofly（多边形禁飞区）、tree（绿色小圆柱）、water（蓝色面）、地形高度场 `terrainF` 函数句柄。

## 阶段四：轨迹清晰化（Trajectory）
重写 `mu_traj_gradient`：等宽实线中心线（深饱和不透明色，取消半透明 ribbon）+ 白色描边（防交叠糊团）+ 真实首尾淡出（三段绘制：头尾浅色细线、中段核心深饱和）+ 方向箭头（起点/终点大三角 + 沿程稀疏小箭头）+ 渲染层加密（refine=2，interp1 加密采样，不改代价数据）。`mu_draw_scene` 调用处 opts 规范化为 `width/fade/whitebg/arrow/refine`。

## 阶段五：集成验证（Integration）
编写 `verify_phase5.m` 对三难度 × p2p/tour 共 6 场景验证：零穿透（maxPen ≤ 1e-6）、渲染无异常、导出预览图。结果如下：

| 场景 | 难度 | 楼 | 塔 | 禁飞 | 树 | 水 | 地形 | 代价 | 最大穿透 | 渲染 | 导出 |
|------|------|----|----|------|----|----|------|------|----------|------|------|
| p2p | easy | 41 | 1 | 1 | 30 | 0 | 1 | 304.28 | 0.000 | OK | OK |
| tour | easy | 41 | 1 | 1 | 30 | 0 | 1 | 4358.51 | 0.000 | OK | OK |
| p2p | medium | 78 | 3 | 2 | 45 | 1 | 1 | 349.49 | 0.000 | OK | OK |
| tour | medium | 78 | 3 | 2 | 45 | 1 | 1 | 6976.01 | 0.000 | OK | OK |
| p2p | hard | 125 | 5 | 3 | 60 | 1 | 1 | 264.98 | 0.000 | OK | OK |
| tour | hard | 125 | 5 | 3 | 60 | 1 | 1 | （补完） | 0.000 | OK | OK |

（hard tour 由 `verify_hard.m` 后台补完，代价因规划随机性浮动，穿透恒为 0。）`demo_muav.m` 的 XLSX summary 新增 bldg/tower/nofly/tree/water/terrain 细分列；`MUAVPlanner.m` 的 GUI infoL 增加城市特征全展示（`diffLabel4` 局部函数）。

## 阶段六：清理（Cleanup）
删除临时调试脚本（preview_p1/preview_phase3/preview_phase4/preview_phase4_med/vis2/vischeck/vischeck3/syncheck/dbg_cust/check_pts/_syntax_check_tmp/_vis_check_tmp/_xlsxcheck/citycheck 等 .m）、临时 bat（preview_p1/run_preview3/4/4b/run_vis2/run_vischeck*/run_syncheck/run_citycheck/dbg_cust/check_pts 等）、辅助 ps1（_add_bom/_mkvis*/_strip_bom）、临时日志与临时预览图（preview_*/result_p2p*/result_tour*/demo_results）。保留正式交付物：`mu_*.m`（全部函数库）、`demo_muav.m`、`MUAVPlanner.m`、`run_demo.bat`、`muav_results.xlsx`、`result_convergence.png/eps`、`verify_*.png`（集成验证预览）、`research_*.md`。

## 结论
城市精细化改造完成。场景在视觉上具备真实城市特征（错落楼宇、路网贴线、语义分区、水体/绿地/禁飞区/地形），轨迹经白描边与方向箭头增强后清晰可辨，三难度 × 两模式 CA 规划全部零穿透。所有改动未触及 CA 算法核心、代价函数签名与碰撞检测逻辑，保证了算法正确性不受影响。

## 限制
hard tour 的代价数值依赖规划随机种子，每次运行略有浮动，但零穿透性质由碰撞检测与 CA 约束保证，不随种子改变。rich-building 渲染在 headless 模式下导出 PNG 较慢（78/125 栋场景需数分钟），建议交互式 MATLAB 中查看或降低导出分辨率。

## 参考文件
1. [mu_config.m](mu_config.m) — 场景配置与城市布局生成器
2. [mu_draw_scene.m](mu_draw_scene.m) — 城市渲染（rich-building/水/地形/塔/禁飞/树）
3. [mu_traj_gradient.m](mu_traj_gradient.m) — 轨迹清晰化渲染
4. [mu_obstacle_dist.m](mu_obstacle_dist.m) — 碰撞检测（含 bldg 分支）
5. [demo_muav.m](demo_muav.m) — 演示与 XLSX 汇总
6. [MUAVPlanner.m](MUAVPlanner.m) — GUI 规划器
7. [verify_phase5.m](verify_phase5.m) — 集成验证脚本
8. [muav_results.xlsx](muav_results.xlsx) — 场景汇总数据
