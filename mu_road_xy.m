function [xy, zc] = mu_road_xy(dyn, vi, t)
% mu_road_xy — 由动态车辆索引 vi 与时间 t 计算其中心 xy 坐标（阶段D）。
% 车辆沿所属道路中心线以速度 speed 循环行驶；junction 类车辆沿一个小环道
% 真实绕行（半径随车长，非原地自转）。返回中心 xy 与对应的地形标高 zc，
% 使动态障碍的 z 分量沿地形高程插值（修复 M5：车辆 z 不再用两端常数均值）。
% dyn.vehicles(vi) 须含字段：dir / phi / cl / roadLen / s0 / speed / lateral / lateralV。
v = dyn.vehicles(vi);
t = t(:).';                                    % 支持标量或行/列向量，统一为行向量
if v.dir == 0
    % junction 车辆：沿半径 ~半车长的小环道真实绕行（非原地自转），
    % 中心取节心 cl(1,:)，z 取节心地形标高。
    Rj = 2.4;                                   % 环道半径(m)，>0 且接近路口尺度
    ang = v.phi + t * 0.6;                       % 绕行角速度（rad/s），t 可为向量
    xy = v.cl(1,:) + Rj * [cos(ang).' sin(ang).'];   % Nx2
    zc = repmat(v.z, numel(ang), 1);             % Nx1，junction 的 z 已是节心地形标高
    return;
end
L = v.roadLen;
s = mod(v.s0 + v.dir*v.speed*t, L);             % 行向量 1xN
% 中心线线性插值（xy 与地形标高同步插值，避免坡地车辆浮空/埋地）
a = v.cl(1,:); b = v.cl(2,:);                    % 1x2
f = (s / L).';                                    % Nx1（列向量便于隐式扩展）
p = a + (b - a) .* f;                             % 1x2 .* Nx1 -> Nx2（隐式扩展）
zc = v.z(1) + (v.z(2) - v.z(1)) * f;             % Nx1，沿中心线两端地形标高插值
xy = p + v.lateral * v.lateralV;                  % Nx2 + 标量*1x2
end
