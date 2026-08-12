function [dist, pen] = mu_obstacle_dist_t(point, scene, t, margin)
% mu_obstacle_dist_t : time-dependent obstacle distance for moving vehicles
%   point : Nx3 UAV positions
%   scene : mu_config scene with scene.dynamics.vehicles
%   t     : time
%   margin: safety margin
%   dist  : Nx1 signed distance (<0 inside)
%   pen   : Nx1 penalty when dist < margin
%   Uses mu_road_xy(dyn, vi, t) to get vehicle (x,y) and z-center.
%   Vehicle oriented by phi; box half-length = length/2, half-width = width/2.
%   Z extent is [vehZ, vehZ+height]; rotated by -phi.
%   Relies on mu_obstacle_dist / mu_box_dist logic.
if nargin < 4, margin = 0; end
if size(point,2) ~= 3, point = point.'; end
N = size(point,1);
dist = inf(N,1);

% dynamics check
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
    [xy, zc] = mu_road_xy(dyn, vi, t);
    cz = zc;
    dx = point(:,1) - xy(1);
    dy = point(:,2) - xy(2);
    c = cos(-v.phi); s = sin(-v.phi);
    lx =  dx*c - dy*s;
    ly =  dx*s + dy*c;
    d = mu_box_dist_local(lx, ly, point(:,3), ...
                          v.length/2, v.width/2, cz, cz + v.height);
    dist = min(dist, d - margin);
end

% M4 penalty (same as mu_obstacle_dist)
t = max(0, -dist);
pen = t.^2 .* (1 + 5*exp(-t/3));

end

function d = mu_box_dist_local(lx, ly, pz, hwx, hwy, zlo, zhi)
% lx,ly,pz in vehicle frame; z extent [zlo,zhi]
qx = abs(lx) - hwx;
qy = abs(ly) - hwy;
qz = abs(pz - (zlo+zhi)/2) - (zhi-zlo)/2;
outside = sqrt(max(qx,0).^2 + max(qy,0).^2 + max(qz,0).^2);
inside  = min(max([qx qy qz],[],2), 0);
d = outside + inside;

end
