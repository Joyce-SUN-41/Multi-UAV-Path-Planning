# 多无人机路径规划模块 — 设计暗伤与弊病审查报告

审查日期：2026-08-09
审查范围：`Multi-UAV-Path-Planning/` 全部核心模块（规划入口、代价函数、轨迹构造、碰撞检测、动态交通层、通信约束、GUI 时间轴）

## 结论速览

模块在城市静态场景渲染（阶段A~E 白底工程风）上完成度高、视觉专业，但**动态避障与时空一致性是结构性的"假象"**，且存在若干会直接导致规划失败或约束失效的硬伤。按严重程度分三档：

- 高危（会导致崩溃或规划结果不可信）：D1 时变碰撞未接入代价、D2 TOUR 控制点越界崩溃、D3 通信链路约束失效
- 中危（数值/架构一致性问题，影响质量与可维护性）：M1 控制点过/欠控制、M2 B 样条节点向量退化、M3 双路径评估语义漂移、M4 动态惩罚尺度不一致、M5 车辆几何与地形脱节
- 低危（契约脆弱/次要）：L1 CA 接口列优先约定无保护

---

## 高危缺陷

### D1 — 动态车辆碰撞 `mu_obstacle_dist_t` 从未进入代价函数（规划对车流零避障）

证据：`mu_cost_p2p.m:43,56` 与 `mu_cost_tour.m:67,81` 只调用静态 `mu_obstacle_dist`；`mu_eval_path.m:37` 也只调静态版；全局搜索无一处 `mu_obstacle_dist_t` 被代价调用。该函数仅被 `verify_phaseD.m` 与 GUI 渲染 `mu_draw_dynamic.m` 使用。

后果：CA 优化出的轨迹对地面车流完全无避障。GUI 的 `playAnim`/`onTimeSlider` 却让车辆动起来、UAV 头点沿静态轨迹移动，观察者会看到"无人机径直穿过行驶中的车辆"，但穿透诊断仍报 0。这是时间轴/规划一致性缺陷，会让用户误以为系统已处理动态避障。

修复方向：给轨迹点配准时间向量 `tk`，在代价中按 `tk(i)` 调用 `mu_obstacle_dist_t(traj(i,:), scene, tk(i), margin)` 并入 `obstPen`，并新增权重 `w.vehicle`。

### D2 — TOUR 分段构造 `mu_build_tour_traj` 控制点越界会直接令 CA 抛异常

证据：`mu_build_tour_traj.m:16-27` 循环 `for s=1:nseg`，取 `c1=ctrl(2*s-1,:); c2=ctrl(2*s,:)`，需要控制点总数 `>= 2*nseg = 2*(m+1)`。第 12 行的截断保护**只作用于 `waypts` 为空的退化分支**，分段分支完全无校验。而 `nC = scene.nCtrl`（固定，`mu_run_planner.m:40`）与每机任务数 `m` 没有强制 `nC >= 2*(m+1)` 的约束。

后果：当某机分配任务数 `m` 满足 `2*(m+1) > nC` 时，`ctrl(2*s-1,:)` 索引越界，CA 主循环抛异常，`mu_run_planner` 的 `catch` 捕获后规划失败。在 hard 难度（30 客户点 / 多机）或大 `nUAV` 下极易触发。

修复方向：进入分段循环前校验并补齐控制点（`if size(ctrl,1) < 2*nseg, ctrl = [ctrl; repmat(mid,2*nseg-size(ctrl,1),3)]; end`）；或在 `mu_run_planner` 中强制 `scene.nCtrl = 2*(maxT+1)` 从源头保证。

### D3 — 通信链路约束 `mu_comms_penalty` 垂直容差随天线挂高无界放大 → 低空自动"被覆盖"

证据：`mu_comms_penalty.m:17` `nz(k) = comms(k).antH*0.6 + 20;` 垂直容差纯由天线挂高决定，与水平半径 `covR` 完全独立。`mu_city_layout.m` 中 gNB 挂塔顶 `antH≈50` ⇒ `nz≈50m`，节点天线若挂高 `c(3)=zg+50`，则竖直覆盖 `[zg, zg+100]`，几乎吞掉全部低空到中层（airspace.low `zhi=60`、mid `zhi=100`）。

后果：高楼/高塔上的 gNB 会让低空飞行的 UAV 自动被判为覆盖，链路约束形同虚设（`w.comms>0` 开启时也失效）。另外覆盖判定是硬 0/1（`mu_comms_penalty.m:27`），`bestGap` 在边界有折点、无梯度，CA 优化不稳定。

修复方向：垂直容差与挂高脱钩（如 `nz = max(15, 0.15*antH)` 或固定 ±30m）；覆盖判定改为连续可微（去掉硬 `break` 与 0/1 标签，用 `max(0,...)` 平滑求和）。

---

## 中危缺陷

### M1 — TOUR 每机控制点固定 `nC`，不随任务数变化（过控制 + 欠控制 + 注释矛盾）

证据：`mu_run_planner.m:33` 注释称 tour 的 `nCtrl` 由任务数自动决定 `2*(maxT+1)`，但第 40 行 `nC = scene.nCtrl` 取的是 `mu_config` 返回的固定值，`2*(maxT+1)` 在该文件从未出现。

后果：任务少的机仍带 `nC` 个控制点却只连 start→goal（退化 B 样条），冗余维度浪费且可能引入无意义弯曲；任务多的机若 `2*(m+1) > nC` 直接触发 D2 崩溃。注释与实现矛盾，后续维护易踩坑。

### M2 — `mu_bspline` 节点向量在 `nCtrl<4` 时退化、中间节点与 clamp 端点 0/1 重叠致过约束

证据：`mu_bspline.m:24` `U = [repmat(0,1,p+1), linspace(0,1,K-p-1), repmat(1,1,p+1)]`。nCtrl=5 时 `U` 长度正确(11)但中间段 `linspace(0,1,3)=[0,0.5,1]` 的 0/1 与首尾 clamp 重复，曲线在两端过约束、自由度被悄悄压低。nCtrl=2 时 `K-p-1=-1`，`linspace(0,1,-1)` 返回空，`U` 长度恰等于 `K+p+1` 使第 25 行保护被跳过（漏判），三次样条退化为单段、边界插值存在数值漏洞（`bspline_curve` 第 44-46 行 `idx` 未夹紧到 `[1,K]`）。

后果：低端 `nCtrl` 配置下轨迹插值偏差、曲线灵活性下降；GUI 若改用小控制点数会出问题。

### M3 — TOUR 绕过 `mu_eval_path` 自行内联评估，双路径语义漂移

证据：`mu_cost_tour.m:58-78` 复制了障碍/边界/地形/平滑逻辑，而非复用 `mu_eval_path.m`。两段 Bezier 拼接后 `size(traj,1)` 取决于段数，平滑度二阶差分与 p2p 的 `mu_eval_path`（固定 140 点 B 样条）不在同一度量尺度；常量 `SAFE_MARGIN=6`、`TERRAIN_MARGIN=8` 在多处硬编码（p2p 用 `TERRAIN_MARGIN=8`，tour 内联 `zg+8`），分散维护。

后果：跨模式公平比较困难，一方修改易与另一方脱节。

### M4 — 动态碰撞惩罚核 `(1-dist)^2` 与静态核 `t.^2*(1+5*exp(-t/3))` 形态不一致

证据：`mu_obstacle_dist_t.m:52` 近壁无指数放大、深穿透为纯二次；静态版 `mu_obstacle_dist.m:197-198` 有近壁 `1+5*exp(-t/3)` 超线性威慑。动态版还无独立权重（静态经 `w.obstacle=60` 加权）。

后果：即便按 D1 接入动态层，规划器对楼宇用强威慑、对车辆偏弱，会倾向"擦蹭车辆"；且动态近距软威慑 `0.15*(3-dist)^2` 仅对 `0<=dist<3` 生效，与静态 `SAFE_MARGIN=6` 不一致。

### M5 — 动态车辆几何与地形脱节（junction 自转、z 用两端常数均值）

证据：`mu_road_xy.m:6-17` 中 `dir==0` 车辆绕节心半径 3m 圆周自转（r=3m < 车长 4m），非真实路口行驶；车辆 `z` 取 `(z0+z1)/2` 常数（`mu_city_layout.m` `make_dynamics`），不沿 `z_profile` 随 `s` 插值，hard 难度 `terrAmp=45` 时车辆浮于地形上方或埋入地形。

后果：动态障碍 z 分量与地形标高不一致，碰撞检测在坡地失真。

---

## 低危 / 契约问题

### L1 — CA 调用列优先约定（`X'` 即 dim×pop）无保护

证据：`CAv9x.m:143-144` 以 `X'`（dim×pop）传入，`mu_cost_*` 的 `size(x,2)>1` 按列分发假设与之吻合（当前正确）。但 `mu_run_planner.m:27` 允许 `caFun` 被替换为任意算法，若新 CA 以行向量 `(pop×dim)` 传入，矩阵分支仍触发但会按列错误切分，静默产生错误解。

修复方向：在代价函数头部或 `mu_run_planner` 调用处加契约断言/注释，明确"CA 必须以 dim×pop 传入"。

---

## 优先级排序与建议

1. **立即修 D2**（最危险，会直接崩溃）—— 加控制点越界保护或强制 `nCtrl=2*(maxT+1)`。
2. **修 D1 + 时空配准**（决定模块可信度）—— 把 `mu_obstacle_dist_t` 接入代价并给轨迹配时间轴。
3. **修 D3**（链路约束失效）—— 修正 `nz` 公式并改连续覆盖判定。
4. **修 M1/M2/M3**（质量与可维护性）—— 每机独立控制点、B 样条节点向量收紧、tour 复用 `mu_eval_path`。
5. **修 M4/M5/L1**（一致性收尾）。

> 注：当前阶段A~E 的静态城市渲染与结构已较完善，上述缺陷主要集中在"动态层打通代价"与"TOUR 鲁棒性/约束有效性"两条线上，修复成本可控。

---

## 修复记录（2026-08-09，全部已解决并验证）

- **D1（时变碰撞接入代价）**：`mu_eval_path` / `mu_cost_tour` 现按弧长配准时间轴 `tk`（新增 `mu_arc_time`，`scene.T_horizon=60`），逐轨迹点调 `mu_obstacle_dist_t(traj(:,:), scene, tk, vehMargin)`，经 `w.vehicle=60` 加权（默认开）。规划出的轨迹对地面车流有真实避障，与 GUI 动画时空一致。验证：穿车点 pen≈30、远离车流 pen=0；三难度 p2p/tour 全 PASS。
- **D2（TOUR 控制点越界崩溃）**：`mu_build_tour_traj` 分段分支在进入循环前校验并补齐控制点（`need=2*nseg`，不足用已有控制点中点补），彻底杜绝 `ctrl(2*s-1,:)` 索引越界。验证：tour hard + nUAV=8 规划成功（nCtrl=10），不再抛异常。
- **D3（通信链路约束失效）**：`mu_comms_penalty` 竖直容差 `nz` 与天线挂高脱钩、固定 `VZ=30m`；覆盖判定改为"取最近节点覆盖缺口"的连续平滑求和（移除硬 0/1 break），梯度连续。验证：低空 z=10 被正确惩罚（pen=60），节点处 pen=0。
- **M1（控制点过/欠控制 + 注释矛盾）**：`mu_config` 已按 `2*(maxT+1)` 设 `nCtrl`；修正 `mu_run_planner` 注释明确"由 mu_config 自动决定"；多余截断逻辑已移除。
- **M2（B 样条节点向量退化）**：`mu_bspline` 内部节点改为严格落在开区间 `(0,1)` 的均匀点 `(1:mk)/mk1`，低控制点自动降阶；de Boor 对恰等于端点重复节点的参数做微小夹紧并在末尾强制对齐 start/goal。验证：nCtrl=2/3/4/5 端点误差均 0。
- **M3（双路径语义漂移）**：障碍/边界/地形/安全壳裕度集中为 `scene.safeMargin=6` / `scene.terrainMargin=8` / `scene.vehMargin=3`，p2p 与 tour 共用，消除硬编码漂移。
- **M4（动态惩罚核不一致）**：`mu_obstacle_dist_t` 惩罚核对齐静态版 `t^2*(1+5*exp(-t/3))`（近壁超线性威慑），并经 `w.vehicle` 独立加权。验证：深度越大 pen 超线性增长。
- **M5（车辆几何与地形脱节）**：`mu_road_xy` junction 车辆改为真实环道绕行（半径 2.4m，非原地自转）；道路车辆 `z` 存储为两端地形标高向量 `[z0 z1]`，位置 `z` 沿中心线插值；`mu_obstacle_dist_t`、`mu_draw_dynamic`、`verify_phaseD` 同步适配返回的 `zc`。验证：t=0 z=29.42、t=200 z=40.16（随地形变化）；动态层渲染三难度 PASS。
- **L1（CA 列优先契约无保护）**：`mu_cost_p2p` / `mu_cost_tour` 矩阵分支加契约断言（CA 须以 dim×pop 列优先传入）。

> 注：修复后模块在动态避障可信度、TOUR 鲁棒性、约束有效性三条线上均达到设计预期，并通过 `verify_fix_all` / `verify_phaseD` / demo 全套回归。

