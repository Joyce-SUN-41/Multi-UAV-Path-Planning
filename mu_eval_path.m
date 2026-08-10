function [len, smooth, obstPen, bndPen, traj, vehPen] = mu_eval_path(ctrl, scene, uavIdx)
% mu_eval_path — 由控制点评估单架无人机的轨迹质量
%   ctrl     : nCtrl x 3 内部控制点
%   scene    : 场景配置
%   uavIdx   : 无人机编号
% 返回：路径长度 len、平滑度(曲率积分) smooth、
%       障碍惩罚 obstPen、边界惩罚 bndPen、采样轨迹 traj(n x 3)、时变车辆惩罚 vehPen
%
% 时变碰撞（D1）：把轨迹按弧长配准到时间轴 tk（0..T_horizon），对每个轨迹点
% 在对应时刻 t 调用 mu_obstacle_dist_t 求车辆穿透惩罚；vehPen 由 w.vehicle 加权
% 在代价函数中累加。tk 也可由调用方预计算传入（cell 追加第4参数 tk）。

start = scene.starts(uavIdx,:);
goal  = scene.goals(uavIdx,:);
n = scene.smooth;

traj = mu_bspline(ctrl, start, goal, n);

% ---- 时间轴配准（弧长 -> 时刻，供时变碰撞 D1；单源：mu_arc_time）----
tk = mu_arc_time(traj, scene.T_horizon);

% ---- 路径长度（折线累加）----
seg = sqrt(sum(diff(traj,1,1).^2, 2));
len = sum(seg);

% ---- 平滑度：二阶差分（近似曲率）----
if n >= 3
    d2 = diff(traj,2,1);
    smooth = sum(sqrt(sum(d2.^2,2)));
else
    smooth = 0;
end

% ---- 障碍惩罚 ----
% 注意：障碍惩罚必须用 "穿透惩罚 pen"（仅当进入障碍安全壳才非零），绝不能用
% 有符号距离 dist 求和——dist 在自由空中为正且随间隙单调增大，sum(dist) 会
% 让"高空远离障碍"的代价远大于"贴楼低空"，导致优化器被反向诱导去贴楼/压低高度
% （已用 verify_cost 验证：高空 sum(dist)=2888 vs 贴楼 634，方向完全反了）。
% 正确语义：pen 在 dist<0（穿透）或 dist<安全裕度时增长，自由空中为 0。
% terrain 是"最低飞行高度"软约束，单独在代价函数中处理，不计入此处。
obTypes = {scene.obstacles.type};
obsSolid = scene.obstacles(~strcmp(obTypes, 'terrain'));
[~, op] = mu_obstacle_dist(traj, obsSolid, scene.safeMargin);
obstPen = sum(op);

% ---- 时变车辆碰撞惩罚（D1）：逐轨迹点按 tk 调 mu_obstacle_dist_t ----
vehPen = 0;
if isfield(scene,'dynamics') && ~isempty(scene.dynamics) && ...
   isfield(scene.dynamics,'vehicles') && ~isempty(scene.dynamics.vehicles)
    [~, vp] = mu_obstacle_dist_t(traj, scene, tk, scene.vehMargin);
    vehPen = sum(vp);
end

% ---- 边界惩罚 ----
b = scene.bounds;
lo = b([1 3 5]); hi = b([2 4 6]);
out = max(0, lo - traj) + max(0, traj - hi);
bndPen = sum(out(:).^2);
end
