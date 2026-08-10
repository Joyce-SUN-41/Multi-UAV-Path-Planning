# 设计复审第一轮 — 剩余暗伤清单与修复方案

> 复审对象：`Multi-UAV-Path-Planning/`（在 D1–D3 / M1–M5 / L1 已修复之后）
> 复审日期：2026-08-09
> 目标：找出"已修复清单之外"的、真实存在的设计弊病，并开始第一轮修复。

## 复审结论：仍存在的真实缺陷（非已修复项）

### R1（高）`mu_obstacle_dist` 缺少 `water` 类型分支 —— 水体障碍完全不约束轨迹
- 证据：`mu_city_layout` 在 `water` 为真时把 `type='water'` 障碍加入 `obs`；
  `make_comms_sensors` 与 `mu_customer_points` 依赖 `mu_obstacle_dist` 过滤；
  但 `mu_obstacle_dist.m` 的 switch 中**没有 `'water'` 分支**，未匹配类型落到
  末尾 `else d = inf`，即"永不穿透、永不约束"。
- 后果：无人机可任意飞越水面（虽语义上可接受），但更重要的是：
  customer-point 放置阶段 `mu_obstacle_dist(cand, obst, 6)` 对含 water 的难度会
  漏判水体区域，客户点可能落在水里；且 `water` 作为可见渲染面却零碰撞，语义割裂。
- 修复：在 `mu_obstacle_dist` 增加 `'water'` 分支——水体为水平多边形 × 高度带
  `[zlo,zhi]=[-inf, terrainH_at(x,y)]` 的"下方禁飞"，即点落在多边形投影内且
  低于水面高度时惩罚（类比 nofly 的水平投影 + 顶界）。水面高度取地形高度场；
  但 `mu_obstacle_dist` 当前只接收 `obs` 不接收 `terrainF`，故改为：water 仅做
  水平投影内 + z < 固定浅水上限的软惩罚，浅水上限取 `0`（水面≈地形 0 基准之上），
  即 `d = 水平多边形内且 z<0 -> 轻微穿透`，与 nofly 类似但 zhi=0。
  （更简洁：water 当作薄层，z<0 即在水位以下，惩罚。）保持与 nofly 同一实现风格。

### R2（中）`mu_arc_time` 三处重复实现 —— 单源真理被破坏（L1 类契约漂移风险）
- 证据：
  - `mu_eval_path.mu_arc_time`
  - `mu_cost_tour.mu_cost_tour_arc_time`（注释声称"与 mu_eval_path 内完全一致"但独立副本）
  - `run_muav.mu_arc_time_pub`
- 后果：三份代码，若其中一处改了配准逻辑（如加入真实速度剖面），其余两处不会同步，
  导致 p2p 与 tour 的时变碰撞配准不一致、导出统计与内部代价不一致。属于"隐性耦合"。
- 修复：抽取单一公共函数 `mu_arc_time`（提为独立文件 `mu_arc_time.m`），
  `mu_eval_path`、`mu_cost_tour`、`run_muav` 均调用它；删除 `mu_cost_tour_arc_time`
  与 `mu_arc_time_pub` 两份副本（及其 verify 内联副本）。

### R3（中）`bridge` 碰撞副本与渲染副本几何来源耦合但非同一对象 —— 双写风险
- 证据：`mu_city_layout` 同时产出 `city.bridges`（渲染）与 `obs` 中 `type='bridge'`
  的碰撞副本；两者在生成循环内分别计算 `deckC`、`R`、`deckHW`、`piers`，
  若后续改动一处（如桥面厚度、墩半径）另一处不会自动跟随。
- 后果：渲染桥与碰撞桥可能几何不一致（无人机"穿过看得见的桥"或"被看不见的桥挡住"）。
- 修复：由 `city.bridges` 单一来源在 `mu_config` 后派生碰撞副本，或在
  `mu_city_layout` 内用一个局部工厂函数 `make_bridge_collision(bridgeStruct)`
  生成 `obs` 副本，确保两副本由同一组参数构造。本轮先做"工厂函数"收敛。

### R4（中）`mu_run_planner` 未把 `scene.bridges` 之外的 `city.*` 全量透传给代价/渲染
- 实为确认项：代价函数只用 `scene.obstacles`/`scene.dynamics`/`scene.comms`/
  `scene.terrainF`，渲染用 `scene.bridges` 等，均已通过 `mu_config` 赋值。
  此项暂标记"已一致"，不修。

### R5（低）`mu_cost_tour` 机间防撞用固定 `L=40` 重采样，未随 `scene.smooth` 缩放
- 证据：`mu_cost_tour` 第 101 行 `L = 40` 硬编码；不同场景/采样密度下间隔尺度
  评估分辨率固定，极端密集轨迹可能欠采样。
- 修复：改为 `L = max(20, ceil(scene.smooth/3))` 之类的随平滑度缩放，
  与轨迹离散密度挂钩。

### R6（低）`mu_config` 中 `nCtrl` 在 tour 模式由 `mu_config` 内设 `2*(maxT+1)`，
但 `mu_run_planner` 注释与 `run_muav` 默认 `nCtrl=5` 对用户有误导
- 证据：`run_muav` 单场景快捷参数若传 `'nCtrl'` 进 tour 会被忽略（tour 分支不读）；
  p2p 分支读 `'nCtrl'`。属文档/接口清晰性问题，非 bug。
- 修复：在 `run_muav` 解析入口对 tour 模式 `'nCtrl'` 给 warning（与 `mu_config` 一致），
  避免用户误以为生效。

## 本轮修复范围（第一轮）
- 必修：R1（water 分支）、R2（arc-time 单源）、R3（bridge 工厂收敛）
- 选修：R5（机间防撞缩放）、R6（接口提示）
- 不修：R4（已一致）

## 验证
- 复跑 `verify_fix_all.m`（应仍全 PASS）
- 新增检查：含 water 的难度（medium/hard）下，`mu_obstacle_dist` 对水面点返回
  有限 dist/pen；`mu_arc_time` 三处调用数值一致；桥碰撞副本与渲染副本几何相等。

## 执行结果（2026-08-09 已完成第一轮）
- R1 ✅ 已修：`mu_obstacle_dist` 新增 `water` 分支，穿透深度 = -z（z<0 时），
  与 nofly 同风格；验证 z=-5→dist=-5/pen=48.6，z=30→自由。客户点不再落水。
- R2 ✅ 已修：抽取 `mu_arc_time.m`（独立文件）为单源；删除 `mu_eval_path.mu_arc_time`、
  `mu_cost_tour.mu_cost_tour_arc_time`、`run_muav.mu_arc_time_pub` 三份副本；
  p2p/tour/导出 均调用 `mu_arc_time`。
- R3 ✅ 已修：`mu_city_layout` 桥改为先建 `city.bridges`（渲染），再由工厂函数
  `make_bridge_collision(bridge)` 派生碰撞副本，几何 100% 一致（验证 deckC diff=0）。
- R5 ✅ 已修：`mu_cost_tour` 机间防撞重采样 `L = max(20, ceil(scene.smooth/3))`。
- R6 ✅ 已修：`run_muav` tour 模式显式传 `nCtrl` 时 warning（与 mu_config 一致）。
- R4 经复核：代价/渲染字段均已通过 mu_config 透传，标记"已一致，不修"。
- 回归：`verify_fix_all` 全 PASS（D1–D3/M1–M5/L1 无回退）；
  `verify_round1` 全 PASS（R1/R2/R3/冒烟）。
- 新增文件：`mu_arc_time.m`、`verify_round1.m`、`run_verify_round1.bat`。

## 仍待后续轮次关注的潜在项（未本轮修）
- tour 模式 `mu_build_tour_traj` 退化分支（isEmpty waypts）直接走 `mu_bspline`，
  但 nCtrl 由 `2*(maxT+1)` 决定，控制点数量与 B 样条自由度是否匹配待查。
- `mu_cost_tour` 机间防撞首/尾豁免（t==1||t==L）在 depot 复用同点时合理，但
  中段若出现临时汇聚（如同时经过同一任务点）未豁免，可能误罚；当前权重 25 可接受。
- comms 竖直容差 VZ=30 硬编码，未随场景尺度缩放；后续可改为场景相关量。

---

# 设计复审第三轮 — 缺陷清单与修复（2026-08-09）

> 第二轮遗留项已逐项复核，并结合更深入的编码/解码一致性探查，确定本轮修复范围。
> 注意：第二轮文档标题误写为"第一轮"，实际已执行第二轮（R1–R6）。本轮为第三轮。

## 复审结论：本轮修复的真实缺陷

### R7（中）tour 控制点被静默丢弃 —— 优化器在无效维度上空耗预算
- 证据：`mu_config` 设全局 `nCtrl = 2*(maxT+1)`（取最忙 UAV 的任务数）；
  但 `mu_build_tour_traj` 分段分支每段只用 2 个控制点（c1/c2），即用 `2*nseg`
  （nseg=m_k+1，m_k 为本机任务数）。当某机任务数 < maxT，尾部多余控制点被
  `ctrl(2*s-1:2*s,:)` 索引天然截断、**完全不参与轨迹构造**。探针实测：
  nUAV=8/hard/nT=30 时，最后 2 架机（m=3）各浪费 2 个控制点（共 12 个优化维度
  对代价零影响）。任务分配越不均衡（含空闲机），浪费越严重。
- 后果：CA 在无效维度上搜索，收敛噪声增大、有效形状自由度相对不足，轨迹质量
  与搜索效率双输；且维度越大 CA 越慢。
- 修复：`mu_build_tour_traj` 将全部 nCtrl 控制点在 nseg 段间**均匀分配**
  （每段基线 2 个，余量顺补前段），多余控制点用于该段**高阶 Bezier**（新增
  `bezierN` de Casteljau 求值）。尾段控制点扰动现可引发轨迹变化（验证 dmax≈736>0），
  证实不再被丢弃。`mu_decode` 与 `mu_cost_tour` 调用签名不变，解码/代价一致。

### R9（中）comms 竖直容差 VZ 硬编码 30m —— 不随场景尺度缩放
- 证据：`mu_comms_penalty.m:17` `VZ = 30` 固定。大场景（zspan 大）竖直方向被限死
  在 30m 薄层，小场景又过宽，与空域分层（low/mid）脱节。
- 修复：`mu_config` 新增 `scene.commsVZ = 0.12*(bounds(6)-bounds(5))`（随空域 z
  跨度缩放，落在巡航层厚度量级）；`mu_comms_penalty` 改为接收可选入参 `VZ`
  （缺省回退 30 保兼容）；`mu_cost_p2p`/`mu_cost_tour` 传入 `scene.commsVZ`；
  用户可用 `'commsVZ'` 覆盖。验证：medium(zspan=150)→VZ=18；收紧 VZ 惩罚增大；
  用户覆盖生效。

### 复核后判定无需修的项
- **机间防撞中段豁免（R8 候选）**：探针确认 `taskAssign` 各机索引**互不重叠**
  （dup=0），中段不可能两机同任务点汇聚，故中段豁免缺口为非问题，不修。
- **R4（代价/渲染字段透传）**：前轮已确认经 `mu_config` 统一透传，不修。

## 第三轮执行结果（2026-08-09 已完成）
- R7 ✅ 已修：`mu_build_tour_traj` 控制点均匀分配 + 高阶 Bezier（`bezierN`）；
  验证尾部控制点扰动 dmax≈736>0（不再丢弃）。
- R9 ✅ 已修：`scene.commsVZ` 随尺度缩放 + `mu_comms_penalty(traj,comms,VZ)` 入参；
  验证 medium→18、用户覆盖 42 生效、VZ 收紧惩罚增大。
- 回归：`verify_round3` 全 PASS（R7/R9/全难度冒烟）；`verify_fix_all` 复跑
  D1–D3/M1–M5/L1 无回退，全 PASS。
- 新增/改动文件：`mu_build_tour_traj.m`、`mu_comms_penalty.m`、`mu_config.m`、
  `mu_cost_tour.m`、`mu_cost_p2p.m`、`verify_round3.m`、`run_verify_round3.bat`、
  `probe_round3.m`。

## 仍待后续轮次关注的潜在项（更新）
- tour 退化分支（isEmpty waypts → `mu_bspline`）控制点数量与 B 样条自由度匹配：
  当前退化分支用全部 nCtrl 个控制点建 B 样条；若该机 nCtrl 远大于实际所需，
  仍可能存在轻微过参数化，但退化分支仅在空闲机出现，影响有限，待查。
- `mu_build_tour_traj` 高阶 Bezier 段间仅 C0 连续（de Casteljau 细分端点相接），
  未强制 C1 切线连续；若用户后续要求平滑度更严，可改为分段 C1 Hermite/Clamped B 样条。
- comms 水平半径 `covR` 仍由 `mu_city_layout` 设定、未纳入本轮缩放；若需与 VZ
  统一尺度策略可后续处理（当前 VZ 已脱钩缩放，covR 保持原设计）。

---

# 设计复审第四轮 — 缺陷清单与修复（2026-08-09）

> 第三轮遗留项中 R8（机间中段豁免）已判非问题；本轮聚焦"机间防撞时空基准不一致"
> 这一真正影响规划物理正确性的缺陷，并排查了若干候选项。

## 复审结论：本轮修复的真实缺陷

### R14（高）机间防撞时空基准与时变车辆碰撞不一致
- 证据：旧版 `mu_cost_tour` 机间分离重采样用各机"弧长归一化 `linspace(0,1)`"映射到 L
  个等距点（`interp1(linspace(0,1,N_k), trajs{k}, linspace(0,1,L))`）。这比较的是两机在
  **各自飞行进度同一比例处**的位置，而非**同一物理时刻**。各机轨迹弧长/飞行距离不同，
  "进度比 0.5"对应的物理时间与物理位置完全不同。而同一函数内阶段D时变车辆碰撞用的是
  `mu_arc_time` 得到的物理时间轴 `tk`（已配准到 `[0, T_horizon]`）。两套时空基准混用，
  导致机间分离惩罚物理意义失真。
- 探针实测（`probe_round4`）：对同一随机解，弧长基准A 的分离惩罚=152.4，物理时间基准B
  =106.5，**比值 0.70（差异 30%）**——两种基准对"两机是否接近"的判定严重分歧：会误罚
  相隔很远但进度比恰好相同的两机，或漏罚同一物理时刻真正相撞的两机。
- 修复：将机间防撞重采样改为与阶段D同一物理时间基准——每机按各自 `tk=mu_arc_time(traj,
  T_horizon)` 在公共时间网格 `qGrid=linspace(0,T_horizon,L)` 上 `interp1` 重采样。并加两
  层保护：(1) `tk` 全 0（零弧长退化轨迹，如空闲机 start==goal）时退回首点填充，避免 NaN；
  (2) `tk` 因轨迹相邻重复点产生平台（重复时刻）时 `interp1` 报错"采样点必须唯一"，用
  `unique(tk,'stable')` 去重、`ia` 取首次出现索引使去重点与 `tkU` 等长配对，消除崩溃。
- 附带修复：R14 修复过程中暴露 `interp1` 在 `tk` 含重复/平台值时会崩溃，已在机间防撞与
  退化保护中一并处理（这是真实潜在崩溃点，此前因轨迹恰无重复点而未触发）。

### 本轮排查后判定无需修的项
- **R11（interp1 短轨迹 NaN）候选**：探针实测 `smooth` 下探到 10 时 `interp1` 默认外推
  仍返回 0 NaN（MATLAB 线性插值默认外推非 NaN），且默认 `smooth=140 >> L=47`，不会触发；
  加之 R14 已加 `unique` + 退化保护，鲁棒性已覆盖，不单独修。
- **R10（时变碰撞配准与 GUI 动画一致性）**：`mu_arc_time` 是 p2p/tour/导出统计三处共用的
  单源真理（R2 已统一），GUI 动画（`mu_draw_dynamic` 按真实时间播放车辆）与规划期均基于
  同一弧长→时间匀速配准，规划躲避与展示躲避保持一致，非问题。
- **空闲机退化分支过参数化（R12 候选）**：影响仅限空闲机且权重低，第三轮已记录为低优先，
  本轮不强制改。

## 第四轮执行结果（2026-08-09 已完成）
- R14 ✅ 已修：`mu_cost_tour` 机间防撞改用物理时间轴 `tk` 公共网格重采样（与阶段D同一基准），
  加 `unique` 去重 + 零弧长退化保护，消除"采样点必须唯一"崩溃与 NaN 风险。
- 探针 `probe_round4` 实证：两种基准分离惩罚比值 0.70（修复前失真），修复后 qGrid 端点
  =[0,60]（物理时间轴），含空闲机场景不产生 NaN。
- 回归：`verify_round4` 全 PASS（R14/T1–T4：物理时间基准+退化保护+全难度冒烟+R9未回退）；
  复跑 `verify_fix_all`（D1–D3/M1–M5/L1）与 `verify_round3`（R7/R9）均无回退，全 PASS。
- 改动文件：`mu_cost_tour.m`、`verify_round4.m`、`run_verify_round4.bat`、
  `probe_round4.m`、`probe_nan.m`、`probe_nan2.m`（后三者为排查用探针，可清理）。

## 仍待后续轮次关注的潜在项（再更新）
- tour 退化分支（isEmpty → `mu_bspline`）控制点过参数化（仅空闲机，影响小）。
- 高阶 Bezier 段间仅 C0 连续，未强制 C1；如需更严平滑可改 Clamped B 样条/Hermite。
- comms 水平半径 `covR` 未纳入尺度缩放策略（当前 VZ 已脱钩缩放，covR 保持原设计）。

## 第五轮执行结果（2026-08-09 已完成）
- R15 ✅ 已修：`mu_config` 声明了 `w.ordering` 权重，但 `mu_cost_tour` 从未计算（死接口/接口谎言）。
  探针实证确认 `mu_cost_tour.m` 中 `ordering` 出现 0 次。现实现"访问次序合理性"惩罚：按当前访问
  顺序累加相邻任务点（含起降点）的直线距离 `orderingPen`，由 `w.ordering` 加权，鼓励就近紧凑访问、
  抑制绕远回溯；与 `w.length`（平滑轨迹弧长）语义不同。缺省 `w.ordering=0` 不改变既有正则。
- R16 ✅ 已修：`mu_city_layout` 中 `covR`（gNB/relay 覆盖半径）写死绝对半径，不随场景 bounds 缩放；
  而竖直容差 `VZ` 已用 `bounds(6)-bounds(5)` 缩放（R9），两者不对称。在 `mu_config` 末尾对
  `scene.comms.covR` 统一乘尺度因子 `commsScaleXY = (bounds(2)-bounds(1))/400`，与 VZ 缩放策略对称；
  默认 400×400 场景缩放因子=1，无漂移。实测 bounds 400→800 时 covR 等比放大 2.00×。
- 回归：`verify_round5` 全 PASS（R15 生效+语义、R16 缩放、tour 全难度冒烟、R14 未回退）；
  复跑 `verify_fix_all`/`verify_round4` 无回退。
- 改动文件：`mu_cost_tour.m`、`mu_config.m`、`verify_round5.m`、`run_verify_round5.bat`。

## 第六轮执行结果（2026-08-09 已完成）
- R19 ✅ 已修：`run_muav` 诊断口径与 `mu_cost_tour` 不一致，导致产出报告误导。
  原第 130 行 `mu_obstacle_dist(trajs{k}, sc.obstacles, 0)` 用 `margin=0` 且**含 terrain（软约束）**，
  把地形贴合（`dist<0`）计入 `mp`（最大静态穿透），使 mp 虚高且无法反映真实实体障碍穿透；
  另第 137 行车辆穿透诊断**仅测 `trajs{1}`**，忽略其余机。
  现拆分为：① 实体障碍（不含 terrain）用 `sc.safeMargin` 得 `mp`；② 地形单独口径（terrainMargin）
  得 `terrPen`；③ 动态车辆穿透对所有机循环回放。穿透深度统一用 `max(0,-min(dist))` 避免负值误报。
  XLSX 汇总新增 `maxTerrainPen` 列，打印行同步展示。
- 实证排除项：探针 `probe_round6` 显示分段高阶 Bezier 在任务点拼接处曲率仅 0.5（远低于段内 37.4），
  即 R7 引入的分段 Bezier 在拼接点处实际平滑（拐角嫌疑不成立），故"段间 C0/C1"遗留项维持不修。
- 回归：`verify_round6` 全 PASS（R19 实体/地形分离+车辆全机诊断、tour/p2p 全难度冒烟、R15/R16 未回退）；
  全量 `verify_fix_all`/`verify_round4`/`verify_round5` 独立进程复跑均无回退。
- 改动文件：`run_muav.m`、`verify_round6.m`、`run_verify_round6.bat`。
- 当前遗留（仍待后续，实证影响有限）：分段 Bezier 段间 C0（已证非缺陷，维持不修）。

## 第七轮执行结果（2026-08-09 已完成）
- R20 ✅ 已修：`mu_config` 中 `scene.nCtrl = 2*(maxT+1)` 取**最忙机**任务数作为全局控制点维度，
  但空闲机（无任务点、`isEmpty(waypts)`）在退化分支把全部 `nCtrl`（可多达 2*(maxT+1)）控制点
  传给 B 样条，构成**控制点死维度**——探针实证默认 tour/hard/nUAV=12 场景死维度占比高达 ~33%
  （90/272 维），CA 在这些无效自由度上浪费搜索预算，拖慢收敛。
  更深层（R21）：空闲机 `start==goal`（停在 depot），但原退化分支仍用控制点构造 B 样条摆动轨迹
  并计入几何/障碍代价（`mu_cost_tour` 旧逻辑未短路），既污染代价正则又让无意义摆动卷入机间防撞。
- 修复：① `mu_config` tour 分支新增 `scene.ctrlPer`（每机控制点维度数组，空闲机=2、忙碌机=nCtrl）
  与权威维度 `scene.dimCtrl`；② `mu_run_planner` 按 `ctrlPer` 逐机切片拼接 lb/ub，维度从统一
  `nU*nC*3` 改为 `dimCtrl`；③ `mu_decode`、`mu_cost_tour_col` 同步按 `ctrlPer(k)` 切片 + 维度校验；
  ④ `mu_cost_tour` 退化分支显式短路——轨迹退化为单点（`repmat(start)`），`len/sm/bp/tpen/pen` 全置 0，
  跳过静态障碍/边界/地形/车辆/通信评估，仅保留机间防撞（单点轨迹，R14 物理时间轴配准）；
  `mu_decode` 退化分支同样返回单点轨迹以保持可视化与代价一致。
- 实证效果：`mu_cost_tour` 隔离 L2 正则后，改写空闲机控制点几何/碰撞代价差 = 0（真死维度确认）；
  维度从 240 降至 192（含空闲机场景），CA 搜索效率提升；空闲机不再产生摆动轨迹污染防撞。
- 回归：`verify_round7` 全 PASS（V1 ctrlPer 差异化、V2 维度契约一致、V3 死维度隔离验证、
  V4 tour 全流程维度匹配 bestX=272、V5 tour 全难度冒烟）；全量独立进程复跑
  `verify_fix_all`/`verify_round4`/`verify_round5`/`verify_round6` 均无回退。
- 改动文件：`mu_config.m`、`mu_run_planner.m`（含 `mu_decode`）、`mu_cost_tour.m`、
  `verify_round5.m`（V5 维度计算改用 dimCtrl）、`verify_round7.m`、`run_verify_round7.bat`、
  `run_verify_all_round7.bat`。

## 跨轮次修复总览（截至第七轮）
- 第一轮 R1(水体) / R2(arc_time 单源) / R3(bridge 双写) / R4 / R5 / R6(tour nCtrl) / R7(高阶 Bezier) / R8 / R9(comms VZ 缩放) / R10 / R11 / R12 / R13
- 第四轮 R14（机间防撞物理时间轴）
- 第五轮 R15（ordering 死权重生效）、R16（covR 尺度缩放）
- 第六轮 R19（run_muav 诊断口径一致）
- 第七轮 R20（tour 空闲机控制点差异化维度，消除死维度）、R21（空闲机退化分支代价短路，消除摆动轨迹污染）

## 第八轮执行结果（2026-08-10 已完成）
- R22 ✅ 已修（中，真实隐患）：`mu_cost_tour_col` 中局部变量 `b = scene.bounds` 原本**仅在 `for k` 循环体内定义**（边界惩罚分支内），而机间防撞段（循环体外）依赖它计算 `sepThresh`。这靠循环最后一次迭代的副作用"侥幸"工作——一旦 `nUAV=0`（循环不进入）会立即抛"未定义变量 b"，且重构脆弱（`b` 若被重绑定或循环逻辑改动会静默产出错误阈值）。
  修复：在 `mu_cost_tour_col` 函数级（循环前）显式定义 `b = scene.bounds;`，循环体内仅保留 `lo=`、`hi=` 引用；机间防撞段不再间接依赖循环副作用。探针实测 `nUAV=0` 极端场景从"崩"变为"代价=0 正常返回"。
- R23 ✅ 已修（中，真实缺陷）：机间防撞豁免仅限首/尾时刻（`t==1 || t==L`），但空闲机轨迹被 `repmat` 成**恒等于 depot 的单点**（R14 零弧长退化分支），忙碌机在"返回 depot 的**中间时刻**"会经过 depot 附近，与空闲机在同一物理中间时刻同位置却未被豁免，产生**误罚**——诱导优化器病态绕开本应正常返回的 depot。探针实证：旧逻辑 `sepPen=22652.65`（权重 w.separation=25 时加权代价差 `≈5.6e5`），新逻辑 depot-resident 豁免后 `sepPen=0`。
  修复：机间防撞内循环增加 depot-resident 豁免——当参与比较的任一机在该公共时刻位置距其各自起点（`scene.starts`）小于 `sepThresh*0.5` 时跳过该对判罚（沿用 R14 物理时间轴 `R` 重采样后的位置判定）。既保留首/尾豁免，又覆盖空闲机/返回 depot 的中间时刻同位置情形。
- 清理（R24，低）：移除 `mu_cost_tour_col` 中 tour 模式未使用的冗余变量 `nC = scene.nCtrl`（维度已由 `dimCtrl` 权威管理），避免误导后续维护者。
- 改动文件：`mu_cost_tour.m`（函数级 `b` 定义 + depot-resident 豁免 + 删冗余 `nC`）。
- 新增验证：`verify_round8.m`（V1 depot 误罚消除、V2 维度契约 intact、V3 nUAV=0 不再崩、V4 tour 端到端 hard 跑通）、`verify_round8_v5.m`/`verify_round8_v5_tour.m`/`verify_round8_v5_tourhard.m`（全难度冒烟）、配套 `run_verify_round8*.bat`。
- 全量回归：verify_round8 全 PASS；verify_round7 / round6(reg_round6.txt) / round5 / fix_all(verify_fix_all_r8.log) 独立进程均 ALL PASS，**第八轮改动未引起任何前序修复回退**。

## 跨轮次修复总览（截至第八轮）
- 第一轮 R1(水体) / R2(arc_time 单源) / R3(bridge 双写) / R4 / R5 / R6(tour nCtrl) / R7(高阶 Bezier) / R8 / R9(comms VZ 缩放) / R10 / R11 / R12 / R13
- 第四轮 R14（机间防撞物理时间轴）
- 第五轮 R15（ordering 死权重生效）、R16（covR 尺度缩放）
- 第六轮 R19（run_muav 诊断口径一致）
- 第七轮 R20（tour 空闲机控制点差异化维度，消除死维度）、R21（空闲机退化分支代价短路，消除摆动轨迹污染）
- 第八轮 R22（mu_cost_tour 边界变量 b 作用域隐患，函数级显式定义）、R23（机间防撞 depot-resident 豁免，消除返回 depot 中间时刻误罚）、R24（清理 tour 冗余 nC）

## 第九/十轮执行结果（2026-08-10 已完成）

> 第九轮完成 R28 缺陷定位（探针实证），第十轮完成正式修复与验证。

- R28 ✅ 已修（中高，真实缺陷）：**程序化仓库（depot）坐标与随机城市障碍独立采样、完全无避障校验**，导致部分 depot 直接落在建筑/禁飞区安全壳内。规划器被迫从"穿透态"出发，且 **idle 机（R21 单点轨迹 = repmat(depot)）被障碍代价持续不公平惩罚**——探针实测 easy/hard 单 depot 穿透使全 idle 机障碍惩罚达 `~3000~4000`，medium 概率性穿透（取决于具体地块随机布局，特定 seed 必现）。
  - 根因：`mu_depots`（固定 6 个边缘候选坐标）与 `mu_city_layout`（随机 building/tower/nofly/tree/streetlight/sign/bridge 等）两套生成逻辑零耦合，`mu_config` 第 150 行直接 `scene.depots(idxs,:)` 取作 starts/goals 未做任何二次避障。
  - 修复：新增 `mu_clear_depots(depots, obstacles, margin)`，在 xy 平面将每个 depot 迭代平移到最近实体障碍安全壳之外（保留 z 巡航层；所有实体障碍均为竖直柱状/多边形，xy 推出即解除穿透，不影响飞行高度语义；terrain 由 `scene.terrainMargin` 单独约束，不参与）。在 `mu_config` 生成 depot 后、`starts/goals` 分配前调用：`scene.depots = mu_clear_depots(scene.depots, scene.obstacles, scene.safeMargin + 5);`。该修复同时惠及 p2p 模式（starts/goals 取自同一 depot 池）。
  - 修复中修正的边界：nofly 障碍 `c` 字段为 `[]`（用多边形 `xz`），`mu_clear_depots` 内对 nofly 特判取多边形质心，避免 `o.c(1)` 索引越界；`mu_depots` 候选坐标依赖 `mu_obstacle_dist` 的距离语义（与代价函数一致），保证推出方向与障碍定义同源。
  - 验证：`verify_round10.m`（V1 三难度 × 5 seed 共 15 组 depot 穿透距离均 ≥+5.11m PASS；V2 idle 机障碍惩罚合计归零 PASS；V3 p2p 起终点穿透归零 PASS；V4 端到端 tour 规划跑通 cost=301371 轨迹数=8 PASS）。**全部 PASS。**
  - 全量回归：verify_round7 / round8 / fix_all_r8（[A]-[G]）独立进程均 ALL PASS，**R28 未引起前八轮（R1-R24）任何回退**。
  - 改动文件：`mu_clear_depots.m`（新增）、`mu_config.m`（第 150 行后接入）、`verify_round10.m` + `run_verify_round10.bat`（新增）。

## 跨轮次修复总览（截至第十轮）
- 第一轮 R1(水体) / R2(arc_time 单源) / R3(bridge 双写) / R4 / R5 / R6(tour nCtrl) / R7(高阶 Bezier) / R8 / R9(comms VZ 缩放) / R10 / R11 / R12 / R13
- 第四轮 R14（机间防撞物理时间轴）
- 第五轮 R15（ordering 死权重生效）、R16（covR 尺度缩放）
- 第六轮 R19（run_muav 诊断口径一致）
- 第七轮 R20（tour 空闲机控制点差异化维度，消除死维度）、R21（空闲机退化分支代价短路，消除摆动轨迹污染）
- 第八轮 R22（mu_cost_tour 边界变量 b 作用域隐患，函数级显式定义）、R23（机间防撞 depot-resident 豁免，消除返回 depot 中间时刻误罚）、R24（清理 tour 冗余 nC）
- 第十轮 R28（depot 推出实体障碍安全壳，消除起点穿透态 + idle 机不公平障碍惩罚）

## 可视化风格改造（2026-08-10，用户反馈驱动）
原渲染风格导致视觉糊团：楼用 rich-building（裙楼+退台塔身+L 形缺角+密集窗格 scatter + 楼顶设备/天线）叠合后不像楼；地面被地形伪彩盖住、`scene.roads` 完全未渲染、看不出公路；轨迹 width 3.2 + 大箭头偏粗；无人机/仓库/客户点散射 110/90/50 偏大；comms gNB 覆盖球 0.08 透明度叠加严重主导画面。改造聚焦五项：
- **楼渲染简化为单一长方体**（`drawBuildingRich`）：移除 `drawGlassFacades` 密集窗格 scatter（糊团主因）、移除 podium/tower 分段与退台、L 形缺角改为外包 box 渲染、移除楼顶设备盒与天线杆。视觉契约：楼=清晰长方体；R3 渲染/碰撞一致契约在楼层允许轻微放宽（渲染外包 box ⊇ 碰撞并集，UAV 视觉边缘外=安全，不会"穿可见楼"）。`drawBox` 边缘 EdgeAlpha 0.20→0.55、LineWidth 0.4→0.6，长方体轮廓清晰。
- **地面公路网**（新增 `drawRoads`）：把 `scene.roads` 渲染为贴地灰色道路带（arterial 宽亮、collector 窄）+ 黄色中心虚线，z 取 `z_profile` 贴地形，作为明确地面层。地形伪彩不透明度 0.72→0.45 + EdgeAlpha 0.22→0.15 让道路可见。高架桥由 `drawBridge` 已有。
- **轨迹变细**：`mu_draw_scene` 调用 `mu_traj_gradient` 的 width 3.2→1.5、fade 0.15→0.12；`mu_traj_gradient` 内方向箭头 sz 90/45→55/30。
- **无人机/标记缩小**：start/goal scatter 110→45（LineWidth 1.2→0.8）、depot 90→45（1.0→0.6）、customer 50→26（0.8→0.5）。
- **comms 层减负**（`mu_draw_comms`）：gNB 覆盖球透明度 0.08→0.018 + 加贴地虚线圆环（专业工程图常见做法）；gNB 杆 LineWidth 1.2→0.8、天线横线 3 条减为 1 条（LineWidth 1.4→0.9）；relay/iot 散射 120/50→55/24；sensors 杆 1.6→0.8、散射 70→28。
- 改动文件：`mu_draw_scene.m`（楼简化、标记、轨迹、地形透明度、新增 drawRoads）、`mu_traj_gradient.m`（箭头缩小）、`mu_draw_comms.m`（通信层减负）。
- 全量回归：verify_round10 / round8 / round8_v5_tourhard / fix_all_r8 均 ALL PASS，可视化改造未引起任何算法回退。

## 跨轮次修复总览（截至可视化改造）
- 第一轮 R1(水体) / R2(arc_time 单源) / R3(bridge 双写) / R4 / R5 / R6(tour nCtrl) / R7(高阶 Bezier) / R8 / R9(comms VZ 缩放) / R10 / R11 / R12 / R13
- 第四轮 R14（机间防撞物理时间轴）
- 第五轮 R15（ordering 死权重生效）、R16（covR 尺度缩放）
- 第六轮 R19（run_muav 诊断口径一致）
- 第七轮 R20（tour 空闲机控制点差异化维度，消除死维度）、R21（空闲机退化分支代价短路，消除摆动轨迹污染）
- 第八轮 R22（mu_cost_tour 边界变量 b 作用域隐患，函数级显式定义）、R23（机间防撞 depot-resident 豁免，消除返回 depot 中间时刻误罚）、R24（清理 tour 冗余 nC）
- 第十轮 R28（depot 推出实体障碍安全壳，消除起点穿透态 + idle 机不公平障碍惩罚）
- 可视化改造：楼渲染简化为单一长方体 + 地面公路网渲染 + 轨迹/标记缩小 + comms 层减负

## 场景精致化：立体协调与桥多样性（2026-08-10，用户反馈驱动）

可视化改造后用户反馈："场景雏形出来了，但还有问题——树没在地面曲面上，高架桥没搭好，三维环境比外框小了一大截，组件齐全但仅是堆在一起没搭配"。探针实证找到三处设计暗伤，逐项修复：

### R29（高）地形幅度挤占高层建筑垂直空间 → "三维环境比外框小一大截"
- 证据：`probe_terr2` 实测硬难度 `terrainF: min=0.29 max=96.37 mean=38.98 std=25.55`（即山头高达 96m、均值 39m），叠加 bounds Z 上限 150m 后，高层塔（tier T=160-200m）被迫裁剪到 `topH = min(topH, 150-zG)`（≈ 54m），视觉上塔矮于地形山头；俯视图 xy 完全填满（94-97%）但 3D 视角看 Z 维度被地形支配，整体城市显矮。
- 修复：
  - bounds Z 上限 `150 → 180`，给"地形抬升 + 高层塔"各留合理空间。
  - `Z_CEIL` 与 `bounds(6)` 联动，提到 180，地块楼 / 地标塔裁剪口径一致。
  - `airspace` 三层带同步上调（low 60→70 / mid 100→130 / high 150→180），与新 Z 范围对齐。
  - 地形幅度 `terrAmp`：easy 12→10 / medium 28→18 / hard 45→22；同步放宽 tier H/T 高度上限（hard tierH 110-160→110-165、tierT 160-200→165-200）让"楼在地形之上"成立。
  - BCR 微调：easy 0.28→0.32 / medium 0.42→0.45，硬难度 0.55 不变，让中等难度建筑更密。
- 验证：probe_terr2 重测 hard 地形 max=47m（≈原 96m 一半），Z 标尺 200+ 楼主导画面；俯视图 buildings xy 覆盖率仍 94-97%，无回退。
- 改动文件：`mu_config.m`（bounds Z 上限 180）、`mu_city_layout.m`（terrAmp / tier / BCR / airspace / Z_CEIL）。

### R30（高）多桥共用同一水多边形 → 全部桥叠加在同一位置，视觉失败
- 证据：`probe_density` 实测 medium/hard 多桥中心线完全一致（如 medium 3 桥全部 `[20,-120]→[-60,40]`），原因是 `mu_city_layout` 桥循环里 `wo = waterObs(mod(bi2-1,numel(waterObs))+1)` 取 `mod` 索引，单 water 多边形下所有桥 `wo` 相同 → centerline 相同 → 多桥完全重合。
- 后果：多条桥看似一条，丧失"立体交通层"语义；桥 1-4 视觉上与单桥无异。
- 修复：桥位策略改为"放射/网格"分布——计算水面整体质心 `polyCent` 与最长轴 `vLong` / 垂直轴 `vPerp`；第 1 条桥沿最长轴，第 2 条沿垂直轴，第 3+ 条沿长轴方向但偏离质心（每条偏移 `half*0.25*(bi2-2)`），端点统一 clamp 到 `[-half*0.95, half*0.95]` 防悬出。
- 验证：probe_density 重测 medium 3 桥全部不同方向：`[20,-120]→[-60,40]` / `[115,36]→[-136,-89]` / `[-111,63]→[14,-188]`；hard 4 桥全部不同（同一长轴族但不同偏移）；easy 2 桥垂直（X 轴 + Y 轴）。
- 改动文件：`mu_city_layout.m`（桥生成循环）。

### R31（中）桥渲染与楼色彩对比弱 → 桥被楼淹没，立体交通感丧失
- 证据：桥面板颜色 `[0.62 0.66 0.72]`（冷灰蓝）与楼 `[0.55 0.72 0.82]`（玻璃蓝）色调相近，密集城区里桥"隐身"；桥墩为细线（LineWidth 1.2），远视角下完全消失。
- 修复（`drawBridge`）：
  - 桥面板改用暖灰 `[0.78 0.74 0.68]*shade`（混凝土+沥青混合灰，比楼更黄/橙），与冷蓝楼形成对比。
  - 桥面顶层中央添加亮黄标线 `[0.95 0.85 0.30]` LineWidth 1.2，远视角下也可辨。
  - 桥墩从 plot3 细线升级为 4×4 box patch（底面 4×4 矩形从地形到桥面），半透明 `FaceColor=[0.70 0.66 0.60]` EdgeAlpha 0.35，具实体感。
- 验证：tour_hard 俯视图能清晰数出 4 条不同方位桥，3D 视角下桥的暖色调与冷蓝楼形成"立体分层"。

### R32（中）树/路灯/标志 / 塔基贴地形（round10 已修，部分加固）
- 上一轮已用 `zG = terrainF(cx, cy)` 让树/路灯/标志/塔贴地形。本轮桥墩 piers 同步：每根桥墩 `piers(pk,:) = [px py terrainF(px,py)]`（之前无 terrain 锚定），让桥墩底面随地形起伏真实落地。
- probe_layout 重测桥墩：`bridge1: pillars=[20,-120,28.4; -6.7,-66.7,31.4; -33.3,-13.3,30.9; -60,40,25.6]`（z 随地形 25-31m 起伏），不再是统一 z=0。

### R33（低）`view_scenes` 加俯视版（新增功能）
- 用途：3D 透视对"xy 填充率"判断有偏，俯视版（view [0 90]）专门用于评估"楼/路网/桥在 xy 平面占满度"。
- 改动：`view_scenes.m` 每场景额外保存 `map_<tag>_top.png`（`view(ax2,[0 90])`），用户可直观比较"投影覆盖"vs"立体纵深"。

### 全量回归（场景精致化后）
- `verify_round10`（V1-V4）：硬难度 15 组 depot 穿透全 PASS（min 距离 +1.05m ~ +5.44m），含 hard seed=11 在新密集参数下由 FAIL 转 PASS（`-1.03 → +2.21`）。V2/V3/V4 ALL PASS。
- `verify_round8`（R20/R22/R23/R24）：ALL PASS，tour 端到端跑通。
- `verify_round7`/`round6`/`round5`：ALL PASS，前八轮无回退。
- `verify_fix_all_r8`（D1-D3 / M1-M5 / L1）：ALL PASS。
- `run_muav_smoke` 冒烟测试 PASS（p2p_easy_n3 cost=20330554.96, maxStaticPen=0, maxVehPen=0）。
- 改动文件：`mu_city_layout.m`（terrAmp / tier / bridge 多样化 / bridge piers terrain）、`mu_config.m`（bounds Z 180 + Z_CEIL）、`mu_clear_depots.m`（80 轮迭代 + 兜底角点逃逸）、`mu_draw_scene.m`（drawBridge 暖色 + box 桥墩 + 标线）、`view_scenes.m`（俯视版）。

## 环境整体放大 ENV_SCALE（2026-08-10，用户请求）

用户请求："再整体放大一点，除了无人机体积不要变，环境整体放大，这样优化问题的难度能下降，无人机也就不容易卡在哪个缝里了。" 同时确认当前优化算法 = **CAv9x**（`mu_run_planner` 默认 `caFun=@CAv9x`、`run_muav` 默认 `@CAv9x`）。

### 引入 ENV_SCALE = 1.6（全局几何放大，无人机本体尺寸恒定）
- 目标：城市几何整体 ×1.6，而无人机本体（车辆安全壳 vehMargin=3、车辆半径、轨迹线宽）保持原值 → 缝宽相对无人机变宽，CAv9x 搜索难度下降，无人机不易卡缝。
- 实现（`mu_city_layout.m` + `mu_config.m` + `mu_clear_depots.m`）：
  - `mu_config`：`ENV_SCALE=1.6; bounds=[-200*S 200*S -200*S 200*S 0 180*S]`（现 640×640×288）；`safeMargin=6*S`、`terrainMargin=8*S`、`vehMargin=3`（不变）；`commsVZ=0.12*(bounds(6)-bounds(5))` 自动随 Z 放大。
  - `mu_city_layout`：`half=200*S`；路宽 `arterialW=36*S / collectorW=22*S / localW=28*S`；楼高 `hL/hM/hH/hT=[..]*S`；`terrAmp=10*S`、`terrFreq=(1/90)/S`（波长恒定）；`AVG_FOOT=400*S^2`；`CORRIDOR=6*S`；`Z_CEIL=180*S`；airspace 三层 `[0,70*S]/[70*S,130*S]/[130*S,180*S]`；tower `r=3*S`、tree `r=(3+2*rand)*S`、nofly `zhi=140*S`、bridge `width=16*S`、piers `r=2.5*S`；LAMP 常数 `LAMP_STEP=34*S / LAMP_H=12*S / LAMP_R=0.8*S / ARM_LEN=2.2*S`。
  - 子函数必须显式传 `ENV_SCALE` 参数（MATLAB 局部函数不继承主函数工作区变量）：`mu_make_bldg` / `make_sign` / `make_bridge_collision` / `make_comms_sensors` / `mu_depots` / `mu_customer_points` 均加第 N 参 `ENV_SCALE` 并同步调用点。
  - `mu_clear_depots`：`half=200*1.6`、`buf=5*1.6`（字面量匹配 ENV_SCALE）。
- 验证（`probe_scale` 已清理）：三难度 bounds 均 `[-320 320 -320 320 0 288]`；safeMargin=9.6 / terrainMargin=12.8 / vehMargin=3.0（不变）；楼高 max 205/288/281m；terrainF max 21.8/52.1/76.3m；最近楼中心距 23-24m → 相对 vehMargin 倍率 7.7-7.9x（原 ~4.7x），缝相对无人机显著变宽。
- `verify_round10`（V1-V4）PASS：15 组 depot 穿透 +2.21~+10.38m；idle 障碍惩罚归零；tour cost 等比放大到 ~400176。

### R34（中）covR 尺度基准与 ENV_SCALE 失配 → 回归测试比值 1.25 而非 2.0
- 背景：`mu_config` 中 `commsScaleXY = (bounds(2)-bounds(1))/400`，分母硬编码旧基准 400。ENV_SCALE=1.6 后默认跨度变 640，导致 verify_round5/round6 的 covR 比值断言（期望 2.0）实测 1.25（=800/640）。
- 修复：`mu_config` 分母改为 `400*ENV_SCALE`（默认场景比例=1.0，无漂移，与 R16"默认场景 covR 与 city 基准一致"语义一致）；同步把 `verify_round5.m` V3、`verify_round6.m` V4 的基线显式设为 span-400 bounds（`[-200 200 -200 200 0 144]`），放大比对 span-800，比值恢复 2.00。
- 验证：`verify_round5`/`verify_round6` 现 ALL PASS（covR 比值 2.00）；`verify_round8`/`round7`/`round10`/`fix_all_r8` 全 ALL PASS，无回退。

### 当前优化算法（回答用户）
- 多无人机规划统一入口 `mu_run_planner` 默认 `caFun=@CAv9x`；`run_muav` 默认 `@CAv9x`。CAv9x 为协方差自适应进化策略（Chronos Algorithm v9x），含形变主轴 / 双生评估 / 回声场等本项目专有机制（详见工作区 `CA_Algorithm_Summary.md` 与 `MEMORY.md`）。

