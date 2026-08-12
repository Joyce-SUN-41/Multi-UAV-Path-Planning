function [xy, zc] = mu_road_xy(dyn, vi, t)
% mu_road_xy ????vi ??t ??xy D??%  speed junction 
% ??xy  zc??%  z  M5??z ??% dyn.vehicles(vi) dir / phi / cl / roadLen / s0 / speed / lateral / lateralV
v = dyn.vehicles(vi);                           % 取出第 vi 辆车（D3 修复：此前该行被误注释导致 v 未定义）
t = t(:).';                                    % /
if v.dir == 0
    % junction  ~??    % ??cl(1,:)z ??
    Rj = 2.4;                                   % (m)??0 ?? 绕行半径
    ang = v.phi + t * 0.6;                       % rad/st 
    xy = v.cl(1,:) + Rj * [cos(ang).' sin(ang).'];   % Nx2
    zc = repmat(v.z, numel(ang), 1);             % Nx1junction ??z 
    return;
end
L = v.roadLen;
s = mod(v.s0 + v.dir*v.speed*t, L);             % ??1xN
% xy /??a = v.cl(1,:); b = v.cl(2,:);                    % 1x2
a = v.cl(1,:); b = v.cl(2,:);                    % 1x2 起/终点
f = (s / L).';                                    % Nx1
p = a + (b - a) .* f;                             % 1x2 .* Nx1 -> Nx2
zc = v.z(1) + (v.z(2) - v.z(1)) * f;             % Nx1
xy = p + v.lateral * v.lateralV;                  % Nx2 + *1x2
end
