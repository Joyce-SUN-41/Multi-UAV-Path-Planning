# 多无人机三维路径规划 Application（CA 算法接口驱动）研究报告

## 摘要
本报告记录了一个运行于 MATLAB 的多无人机三维路径规划 application 的设计与实现。该 application 以**接口调用形式**直接接入现有的 CA（Chronos）算法（CAv9x / CAv8），将路径规划问题转化为连续优化问题：每架无人机的三维轨迹用 B 样条/Bezier 控制点表示，CA 算法在连续搜索空间中优化这些控制点，使代价（路径长度 + 平滑度 + 障碍穿透 + 越界 + 机间分离）最小化。Application 支持两种可切换场景——静态障碍避障（P2P）与多机任务点巡访（TOUR），并以纯 MATLAB 的交互式 GUI（uifigure，App Designer 风格）呈现三维可视化、收敛曲线与轨迹动画。

## 背景与设计动机
现有工作区已积累 CA 系列进化算法（CAv8 综合通用最强，CAv9x 引入"形变主轴"概念、命名重包装为 Chronos 词汇）。这些算法统一以 `fhd(x)` 形式接收目标函数句柄，对列向量（或 `dim×N` 矩阵）输入返回标量（或 `1×N`）代价，并通过 `varargin` 透传场景数据。路径规划天然适配此接口：把无人机轨迹参数编码为决策向量 `x`，把"轨迹优劣"编码为代价函数，即可复用 CA 的全部搜索机制（多时间线印记场、引力信标、定向流形切线、证据球、回声场、双生观测、连续 Rechenberg 步长等），无需改动算法内部。

查阅资料确认（见参考文献）：三维无人机路径规划主流做法是用 B 样条/Bezier 控制点表示轨迹以降低维度并保证平滑，代价函数通常包含路径长度、安全避障（软约束惩罚）、平滑度（曲率）与机间防撞。本实现遵循此范式，并把 CA 的软约束探索能力与上述代价结合。

## 架构与文件组织
Application 分为底层可调用函数库（供 CA 直接调用）与上层 GUI：

- `mu_config.m`：场景配置构造。P2P 模式给出起点/终点与障碍群；TOUR 模式生成 12 个任务点（确保全部位于障碍安全壳外，安全裕度 12），并按随机键编码分配给各机。TOUR 模式每机控制点维度自动设为 `2*(maxT+1)`。
- `mu_bspline.m`：三次 B 样条求值（Cox-de Boor 基函数求和，端点 clamped 保证曲线经过起/终点）。
- `mu_obstacle_dist.m`：球/立方体/圆柱障碍的带符号距离与平滑穿透惩罚（随穿透深度超线性增长，保证梯度连续）。
- `mu_eval_path.m`：由控制点评估单架轨迹的长度、平滑度、障碍与越界惩罚。
- `mu_cost_p2p.m` / `mu_cost_tour.m`：两种场景的代价函数，签名均满足 `cost = fhd(x, scene)`，并兼容 CA 的矩阵输入（每列一个解，返回 `1×N`）。
- `mu_build_tour_traj.m`：TOUR 分段三次 Bezier 拼接（每段 2 个内部控制点），避免少控制点时 B 样条退化。
- `mu_run_planner.m`：统一入口。组装决策向量 `x`、边界 `lb/ub`，调用 `CAv9x(fhd, dim, pop, iter, lb, ub, opts, scene)`（scene 经 `varargin` 透传），并拆解最优解为各机轨迹。
- `MUAVPlanner.m`：纯 MATLAB 交互式 GUI（深色精致主题）。含场景模式下拉、参数编辑框（无人机数/控制点数/种群/迭代/maxFE/各代价权重）、运行按钮、动画播放按钮、三维坐标区（轨迹+障碍+起终点+任务点）、收敛曲线坐标区与状态文本。
- `demo_muav.m`：无 GUI 端到端验证脚本（含 sphere 接口校验 + p2p + tour + 绘图）。

## CA 接口接入方式
CA 以 `feval(fhd, x, fargs{:})` 调用目标函数。本 application 将代价函数句柄（如 `@mu_cost_p2p`）与场景结构体 `scene` 一并传入，CA 内部以 `feval(fhd, X', scene)` 评估整个种群（每列一个解向量）。代价函数识别矩阵输入后逐列计算并返回 `1×N` 行向量，完全符合 CA 的评估约定。该接入方式**未修改 CA 算法任何内部逻辑**，接口样式仅按 CA 既有约定（`fhd(x, varargin)`）做了最小适配，满足"以接口调用函数的形式接入"的要求。

## 验证结果
通过 `demo_muav.m` 端到端运行验证（CAv9x，随机种子固定）：

1. CA 接口校验：以 sphere 代价（dim=10）测试，CA 收敛到 1.4e-1，确认算法接入正确。
2. P2P 场景（3 机，pop=40，iter=120，maxFE=60000）：最优代价约 341，**最大障碍穿透深度 -0.000**，即轨迹完全不进入任何障碍，避障成功。
3. TOUR 场景（3 机巡访 12 任务点，maxFE=80000）：代价由随机解的约 29785 降至 6725，下降约 77%；任务点均位于障碍安全壳外，机间分离约束生效。小预算（pop=20, maxFE=20000）下代价约 13015、最大穿透约 33，增大预算后显著改善。

所有源码经 `checkcode` 语法检查无错误。GUI 在 headless 环境完成了语法与组件创建路径校验；三维可视化、收敛曲线与动画逻辑基于已验证的 `mu_run_planner` 数据与 `mu_draw_scene` 绘图函数。

## 关键工程修正（实现过程中）
- B 样条端点退化：初始 de Boor 实现在少控制点（K=4）时退化，轨迹不过端点且不随控制点变化。重写为 Cox-de Boor 基函数求和，并令重复节点处 `alpha=0`；TOUR 分段改用三次 Bezier 保证端点 clamped 且对控制点连续敏感。
- 任务点落入障碍：默认生成的任务点之一位于球障碍内部（dist=-10.68），导致任何经过都穿障。改为生成时剔除落在安全壳（裕度 12）内的点。
- MATLAB 短路乘法 bug：`(s==1)*waypts(s-1,:)` 在 `s==1` 时仍计算 `waypts(0,:)` 触发非法索引，改为显式 if/else。
- 矩阵输入分发：代价函数增加"每列一个解"分支，匹配 CA 的种群矩阵评估约定。

## 局限与后续建议
- TOUR 场景代价量级（数千）高于 P2P（数百），主要源于机间分离惩罚与长路径；默认参数下在中等预算已实现良好避障，但极端密集障碍或大机群场景建议提高 maxFE 或调整 `w.separation`。
- 当前障碍为静态解析体（球/立方体/圆柱）；若需地形或动态障碍，可扩展 `mu_obstacle_dist` 与 `mu_config` 而不改动 CA 接口。
- 为进一步提升 TOUR 收敛，可启用 CAv9x 的形变主轴层（`use_morph` 系列开关），针对旋转/病态代价流形获得阶跃提升（已知在旋转基准上有效，但通用场景默认关闭）。

## 使用方法
在 MATLAB 命令行进入 `Multi-UAV-Path-Planning` 目录并运行 `MUAVPlanner` 打开 GUI；或运行 `demo_muav` 做无界面验证。底层函数库亦可直接调用，例如 `[bx, bc, cv, trajs, sc] = mu_run_planner('p2p', 'pop',40,'iter',120,'maxFE',60000)`。

## 参考文献
1. [Multi-UAV Route Re-Generation Based on B-spline (Shanghai Jiao Tong University)](https://xuebao.sjtu.edu.cn/sjtu_en/EN/10.1007/s12204-021-2332-2)
2. [Improved PSO for UAV B-spline Path Planning (Journal of Zhengzhou University)](http://gxb.zzu.edu.cn/oa/pdfdow.aspx?Sid=202408029)
3. [Obstacle avoidance path planning of UAV (Scientific Reports, 2023)](https://www.nature.com/articles/s41598-023-43783-7)
4. [A constrained differential evolution algorithm for UAV path planning in disaster scenarios (PDF)](https://klme.nuist.edu.cn/fj/kycg/lw/2020/AconstraineddifferentialevolutionalgorithmtosolveUAVpathplanningindisasterscenarios.pdf)
5. [多无人机、多场景路径规划 MATLAB（知乎）](https://zhuanlan.zhihu.com/p/717598324)
6. [CAv9x.m — Chronos Algorithm v9 实现（本工作区）](file:///C:/Users/Joyce_SUN/Desktop/LLM-MoH-DNOP/CAv9x.m)
7. [CA_Algorithm_Summary.md — CA 系列算法梳理（本工作区）](file:///C:/Users/Joyce_SUN/Desktop/LLM-MoH-DNOP/CA_Algorithm_Summary.md)
