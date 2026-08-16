# 场景模型维护性与隐患审查报告

审查日期：2026-08-13
审查范围：场景模型链路（`mu_config` / `mu_city_layout` / `mu_obstacle_dist` / `mu_obstacle_dist_t` / `mu_road_xy` / `mu_clear_depots` / `mu_cost_p2p` / `mu_cost_tour` / `mu_eval_path` / `mu_comms_penalty` 及其消费方）
审查方式：静态逐文件核对 + MATLAB R2024b 实跑验证（`-r` 表达式调用，规避本机 `-batch` 中文编码报错）。构造 easy/medium/hard 三档 p2p 与 medium tour 场景，逐项核对数据一致性、边界条件、模块衔接。

## 一、已修复的问题（安全、不改变既有行为）

### 问题 1：每个场景都含一个“幽灵障碍”元素
- 位置：`mu_city_layout.m` 原第 19 行 `obs = struct('type',[],'c',[],...)` 用占位标量结构体初始化 `obs`，随后所有真实障碍通过 `obs(end+1) = ...` 追加，导致 `obs(1)` 永远是一个 `type=[]`、各字段为空的垃圾元素。
- 影响：① `{scene.obstacles.type}` 类型列表污染（含空串），任何对类型做 `unique`/`strcmp` 聚合的消费者（如诊断脚本、`mu_obstacle_dist` 的 `obsSolid` 过滤）都会被空元素干扰；② 每次距离计算多一轮无效迭代（`type=[]` 落入 `else` 分支返回 `inf`）；③ 任何假定 `obstacles(1)` 为真实障碍的潜在消费者会读到空字段。
- 修复：将初始化改为 **0×1 空结构体数组** `obs = struct('type',{},'c',{},...)`，既消除垃圾元素，又保留 `obs(end+1)` 追加所需的 struct 类型（直接用 `[]` 会因 double→struct 类型不匹配而报错，已实测验证）。
- 验证：三档场景 `phantom empty=0`，障碍数精确减少 1（276→275 / 321→320 / 455→454）；距离计算与 p2p/tour 全链路无回归。

### 问题 2：`mu_obstacle_dist` 楼宇分支对空 `setback` 字段会崩溃
- 位置：`mu_obstacle_dist.m` 原 `hwT = hw * (1 - o.setback)`。
- 影响：当某个 `bldg` 障碍的 `setback` 为空（`[]`）时，`hw * (1 - [])` 触发“矩阵乘法维度不正确”错误。程序化场景经 `mu_make_bldg` 总会写入非空 `setback`，故当前场景安全；但这是潜在脆弱点——任何手工/外部构造的障碍物或后续重构都可能崩溃。同函数的 `cutFrac`/`hasL` 已有 `isfield && ~isempty` 守卫，唯独 `setback` 缺失。
- 修复：与 `cutFrac` 一致地加守卫：`sb = 0.1; if isfield(o,'setback') && ~isempty(o.setback), sb = o.setback; end; hwT = hw * (1 - sb);`。默认 0.1 与 `mu_make_bldg` 的常见取值同量级，行为对真实楼宇完全不变。

### 问题 3：死代码占位行
- 位置：`mu_obstacle_dist.m` 原 `cellx = (nb(1)>1) * (2*bx/(nb(1)-1)) : 0; % placeholder`。
- 影响：计算了一个从未被使用的变量（`row` 分支实际用 `stepx`/`stepy`）。无功能危害，但属误导性死代码，易诱导后续维护者误用。
- 修复：删除该行。

## 二、经核查确认“非缺陷 / 已有正确保护”的项（不建议改动）

1. **仓库/起点 z 高度 = 地形 + 12·ENV_SCALE（+30）**：初看疑似异常（`mu_depots` 字面 z=22），实为 `mu_config` 第 165–181 行 **R35 修复**——按地形标高抬升每个 depot 的 z 后再做避障推开，避免 depot/起点落入地下。diag 实测三档 depot 全部 `above=1`（在地面以上），正确且必要。

2. **tour 模式 `ctrlPer` / `dimCtrl` 维度一致性**：`mu_config` tour 分支按各机实际任务数 `2*(numel(taskAssign{k})+1)` 设置 `ctrlPer`，并 `dimCtrl = sum(ctrlPer*3)`；`mu_run_planner` 用 `dim = dimCtrl + keyDim`、`mu_cost_tour` 用 `expected = dimCtrl + nU*maxT` 校验。tour 模式实跑成功（cost 正常计算、CA 未报维度不匹配），证明三者一致。另：`mu_config` 第 122–128 行有一段基于空 `tasks` 的 `taskAssign` 初算（结果为全空 cell），随后第 194 行被真实分配覆盖——冗余但无害，未改动以免引入风险。

3. **`water` 障碍分支的 `if point(i,3)<0` 看似无效**：两分支当前都写 `d(i)=point(i,3)`，数值上对“水下(z<0)为负惩罚、水上(z>0)为正间隙”语义是正确的（负距离触发惩罚，正距离不触发）。该 `if` 属冗余/可读性瑕疵，但**改动会改变代价数值**，故保留原行为，仅作记录。

4. **`comms` 覆盖半径 `covR` 随难度递减**（easy≈327 / medium≈170 / hard≈108）：由 `weatherK`（1.0/0.82/0.62）与随机抖动驱动，符合“密集老城遮挡强、覆盖更紧”的设计；`mu_config` 的 `commsScaleXY` 仅在本场景 bounds 偏离默认基准时才缩放，默认档比例=1.0 无漂移。正确。

5. **地形高度场全域非负**：diag 实测三档 `min≥0`、`anyNeg=0`；建筑基底 `c(3)=terrainF` 贴合地形、楼顶裁剪 `zG+topH≤Z_CEIL` 防止穿出 bounds；树/塔基同样贴合地形（`mu_city_layout` 已修复悬空）。正确。

6. **动态交通层 `dynamics.vehicles` 为结构体数组**，`mu_road_xy(dyn,vi)` 用 `dyn.vehicles(vi)` 取元素（要求结构体数组而非 cell），diag 确认 `isStructArray=1`；junction 车辆 `dir=0` 在 `mu_road_xy` 有专用分支且提前 `return`，不与 `v.z` 标量（道路车辆 `v.z` 为 1×2）冲突。正确。

7. **`scene.terrainF` 以函数句柄存于结构体字段**、`scene.comms`/`sensors` 元数据字段齐全，`mu_cost_p2p`/`mu_cost_tour`/`mu_eval_path` 均通过 `isfield` 守卫访问 `dynamics`/`terrainF`。正确。

8. **边界条件**：`mu_obstacle_dist` 将 `inf` 距离裁剪为 `1e3`（自由空间），`t=max(0,-dist)` 保证正距离不产生惩罚；`nofly`/`water` 用 `mu_in_poly` 射线法（凸/近凸多边形有效）。

## 三、资源加载/释放与模块衔接结论
- 本工程为 MATLAB 脚本/函数，无显式句柄资源管理；“资源释放”主要体现在绘图：三档空地图与 p2p/tour 结果图均经 `mu_draw_scene` 成功生成（run_muav 输出 PNG 至 `results/`），未出现未关闭 figure 堆积或字段缺失导致的渲染崩溃。
- 模块衔接：场景字段（`bounds`/`starts`/`goals`/`obstacles`/`tasks`/`taskAssign`/`ctrlPer`/`dimCtrl`/`dynamics`/`comms`/`terrainF`/`w`/`safeMargin`/`terrainMargin`/`vehMargin`/`commsVZ`/`T_horizon`/`smooth`）在生成、代价、解码、绘制四端读取一致，p2p/tour 端到端跑通即证。

## 四、修改文件清单
- `mu_city_layout.m`：移除幽灵障碍占位（改为 0×1 空结构体数组）。
- `mu_obstacle_dist.m`：楼宇分支 `setback` 空值守卫；删除死代码 `cellx` 行。

## 五、遗留建议（未改动，供后续参考）
- `mu_config` 第 122–128 行对空 `tasks` 的 `taskAssign` 初算为冗余，可在重构时删除（当前无害）。
- `water` 分支的冗余 `if` 可在不影响代价数值的前提下改为单一 `d=z` 并补全注释，提升可读性。
- 验证用临时脚本（`diag_main.m`/`diag2_main.m`/`diag3_main.m` 及 `_*.bat`）建议清理；它们仅用于本次审查，不影响工程功能。
