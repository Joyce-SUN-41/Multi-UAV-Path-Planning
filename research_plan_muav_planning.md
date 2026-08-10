# Research Plan: 多无人机三维路径规划 Application (CA-driven)

## 目标
在 Multi-UAV-Path-Planning 文件夹用 MATLAB 构建一个精致的多无人机三维路径规划 application：
- 直接以接口调用方式接入现有 CA 算法（CAv9x / CAv8）
- 三维路径规划，轨迹用 B 样条控制点表示（精致、低维、平滑）
- 场景可切换：(A) 静态障碍 + 起终点避障；(B) 多机任务点巡访（多目标）
- 形态：MATLAB App Designer GUI + 可调用函数库

## CA 接口（已确认）
`[Best_Score, Best_X, convergence_curve] = CAv9x(fhd, dim, pop_size, iter_max, lb, ub, opts, varargin...)`
- fhd 按列向量输入 x，多输出取第一个
- varargin 首参为 struct(选项)，其余透传给目标函数 → 用于透传 scene
- 默认已开双生观测(use_twin)与尺度中性各向异性(fix_aniso)

## 关键设计决策
1. 代价函数签名：`cost = fcn(x, scene)`；CA 调用 `feval(fhd, x, scene)`。
2. 轨迹：每架无人机 K 个控制点 (3K 维)，三次均匀 B 样条插值端点固定为起/终点。
3. 场景A 代价 = w_len*长度 + w_smooth*曲率 + w_obs*障碍软惩罚 + w_bnd*边界惩罚。
4. 场景B：随机键(random-key)编码排序各机任务点访问顺序；代价叠加机间最小距离防撞惩罚。
5. 障碍：球体(center,r) + 立方体(center,half) + 地形/圆柱可选；软惩罚随穿透深度指数增长。
6. GUI：场景下拉、参数滑块(pop/iter/控制点数/权重)、Run、3D axes(轨迹+障碍+起终点+动画播放)、收敛曲线 axes、状态文本。

## 文件清单
- mu_config.m        场景配置构造/默认场景
- mu_bspline.m       控制点→B样条轨迹
- mu_obstacles.m     障碍距离与惩罚
- mu_eval_path.m     采样轨迹、计算长度/平滑度/惩罚
- mu_cost_p2p.m      场景A代价函数 (fhd)
- mu_cost_tour.m     场景B代价函数 (fhd)
- mu_run_planner.m   统一入口(组装x/lb/ub, 调 CAv9x, 拆解)
- MUAVPlanner.m      纯 MATLAB uifigure 应用（App Designer 风格 GUI，可直接 matlab 运行）
- demo_muav.m        无 GUI 演示脚本(验证算法可运行)

## 验证（已完成 2026-08-07）
1. demo_muav.m Step1 sphere 验证 CA 接口接入正确（收敛到 1.4e-1）。
2. 场景A p2p：3 机避障，最大障碍穿透 -0.000（零穿透，完美避障），cost≈341。
3. 场景B tour：3 机巡访 12 任务点，cost 由 29785(随机)→6725(优化后)，穿透大幅降低。
4. GUI syntax check 全部通过（checkcode 无错误）。
5. B 样条实现已修正（初始 de Boor 退化/Bezier 端点问题修复，轨迹端点 clamped 且对控制点连续敏感）。

## 关键修正记录
- mu_bspline.m：重写 de Boor 为 Cox-de Boor 基函数求和，open uniform 节点 p+1 重复，denom=0 时 alpha=0 约定。
- mu_build_tour_traj.m：每段用三次 Bezier（4 控制点）避免 K=4 B 样条退化。
- mu_config.m(tour)：任务点生成后剔除落在障碍内的点（安全裕度 12），避免必经点穿障。
- 所有代价函数支持 CA 的矩阵输入（dim×N，每列一个解）分发到单列计算。
- 修复 MATLAB 短路乘法 `(s==1)*waypts(s-1,:)` 导致索引 0 非法的 bug → 改 if/else。
- tour 每机控制点维度自动 = 2*(maxT+1)，随机键排序任务访问顺序。
