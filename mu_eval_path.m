function [len, smooth, obstPen, bndPen, traj, vehPen] = mu_eval_path(ctrl, scene, uavIdx)
% mu_eval_path ??
%   ctrl     : nCtrl x 3 ??%   scene    : 
%   uavIdx   : ??% ??len() smooth??%        obstPen??bndPen??traj(n x 3)??vehPen
%
% D1 tk??..T_horizon
% ??t  mu_obstacle_dist_t vehPen ??w.vehicle 
% tk cell ?? tk??
start = scene.starts(uavIdx,:);
goal  = scene.goals(uavIdx,:);
n = scene.smooth;

traj = mu_bspline(ctrl, start, goal, n);

% ----  ->  D1mu_arc_time??---
tk = mu_arc_time(traj, scene.T_horizon);

% ---- ----
seg = sqrt(sum(diff(traj,1,1).^2, 2));
len = sum(seg);

% ---- ----
if n >= 3
    d2 = diff(traj,2,1);
    smooth = sum(sqrt(sum(d2.^2,2)));
else
    smooth = 0;
end

% ----  ----
%  "??pen"??% ??dist dist sum(dist) ??% ??"""/
% ??verify_cost ??sum(dist)=2888 vs  634??% pen ??dist<0??dist<??0??% terrain ??????obTypes = {scene.obstacles.type};
obTypes = {scene.obstacles.type};
obsSolid = scene.obstacles(~strcmp(obTypes, 'terrain'));
[~, op] = mu_obstacle_dist(traj, obsSolid, scene.safeMargin);
obstPen = sum(op);

% ---- D1??tk ??mu_obstacle_dist_t ----
vehPen = 0;
if isfield(scene,'dynamics') && ~isempty(scene.dynamics) && ...
   isfield(scene.dynamics,'vehicles') && ~isempty(scene.dynamics.vehicles)
    [~, vp] = mu_obstacle_dist_t(traj, scene, tk, scene.vehMargin);
    vehPen = sum(vp);
end

% ----  ----
b = scene.bounds;
lo = b([1 3 5]); hi = b([2 4 6]);
out = max(0, lo - traj) + max(0, traj - hi);
bndPen = sum(out(:).^2);
end
