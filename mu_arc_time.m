function tk = mu_arc_time(traj, T_horizon)
% mu_arc_time — 单源真理：把轨迹按弧长配准到 [0, T_horizon] 的时间向量。
%   匀速（弧长参数化）假设：规划阶段无真实速度剖面，按弧长归一化是最保守、
%   与 GUI 动画一致的时空配准方式，保证"车辆在 tk 的位置"与轨迹点对齐。
%   p2p（mu_eval_path）、tour（mu_cost_tour）与导出统计（run_muav）均调用本函数，
%   避免多份副本漂移导致时变碰撞配准不一致（R2 修复）。
n = size(traj,1);
if n < 2
    tk = zeros(n,1);
    return;
end
cum = [0; cumsum(sqrt(sum(diff(traj,1,1).^2,2)))];
tot = cum(end);
if tot < 1e-9
    tk = zeros(n,1);
else
    tk = (cum / tot) * T_horizon;
end
end
