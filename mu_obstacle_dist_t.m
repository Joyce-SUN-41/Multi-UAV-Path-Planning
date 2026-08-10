function [dist, pen] = mu_obstacle_dist_t(point, scene, t, margin)
% mu_obstacle_dist_t — 时变碰撞检测（阶段D）：在时刻 t 计算点到
%   所有动态障碍（地面车辆 dynamics.vehicles）的带符号距离与穿透惩罚。
%   静态障碍仍由 mu_obstacle_dist 负责，本函数只处理移动车辆层。
%
%   point : 1x3 或 Nx3 UAV 轨迹点
%   scene : mu_config 场景（须含 scene.dynamics.vehicles）
%   t     : 时刻（秒），车辆沿路网循环行驶
%   margin: 车辆安全壳外扩（米）
%   dist  : Nx1 到最近动态障碍表面的带符号距离（<0 表示穿透）
%   pen   : Nx1 穿透惩罚（深度越大指数增长），仅在 dist<margin 时非零
%
% 设计：每辆车在当前时刻的中心位置由 mu_road_xy(dyn, vi, t) 给出，
%   车体为绕 Z 轴旋转 phi 的长方体（长 length × 宽 width × 高 height），
%   底面贴地（z ∈ [vehZ, vehZ+height]）。将 UAV 点经 -phi 旋转到车辆
%   局部轴对齐系后，复用 mu_obstacle_dist 中的 mu_box_dist。
if nargin < 4, margin = 0; end
if size(point,2) ~= 3, point = point.'; end
N = size(point,1);
dist = inf(N,1);

% 场景须含 dynamics 层
if ~isfield(scene,'dynamics') || isempty(scene.dynamics) || ...
   ~isfield(scene.dynamics,'vehicles') || isempty(scene.dynamics.vehicles)
    pen = zeros(N,1);
    return;
end
dyn = scene.dynamics;
V = dyn.vehicles;
nv = numel(V);

for vi=1:nv
    v = V(vi);
    % 当前时刻车辆中心 xy 与地形标高（zc 沿中心线插值，坡地准确）
    [xy, zc] = mu_road_xy(dyn, vi, t);
    cz = zc;                      % 地面基准高度（随地形变化）
    % 旋转到车辆局部系（绕 Z 轴 -phi）
    dx = point(:,1) - xy(1);
    dy = point(:,2) - xy(2);
    c = cos(-v.phi); s = sin(-v.phi);
    lx =  dx*c - dy*s;
    ly =  dx*s + dy*c;
    % 轴对齐盒：局部 x 半长 = length/2，y 半宽 = width/2
    d = mu_box_dist_local(lx, ly, point(:,3), ...
                          v.length/2, v.width/2, cz, cz + v.height);
    dist = min(dist, d - margin);
end

% 穿透惩罚（仅在进入安全壳时非零，深度越大指数增长）
% M4 修复：核形态与静态版 mu_obstacle_dist 对齐 —— 采用
%   t = max(0,-dist); pen = t^2*(1+5*exp(-t/3))
% 使近壁有超线性威慑（避免规划器"擦蹭车辆"），深穿透超线性增长，且与 w.obstacle
% 同量级加权（M4 第二项：动态层此前无近壁指数放大、且未独立加权）。
t = max(0, -dist);
pen = t.^2 .* (1 + 5*exp(-t/3));
end

function d = mu_box_dist_local(lx, ly, pz, hwx, hwy, zlo, zhi)
% 局部轴对齐盒带符号距离（lx,ly 已旋转到车体系；pz 为全局 z，车体 z 范围 [zlo,zhi]）
qx = abs(lx) - hwx;
qy = abs(ly) - hwy;
qz = abs(pz - (zlo+zhi)/2) - (zhi-zlo)/2;
outside = sqrt(max(qx,0).^2 + max(qy,0).^2 + max(qz,0).^2);
inside  = min(max([qx qy qz],[],2), 0);
d = outside + inside;
end
