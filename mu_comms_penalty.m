function pen = mu_comms_penalty(traj, comms, VZ)
% mu_comms_penalty ??E??%    traj (n x 3) ??comms ??%   ???? UAV ??%  covR??VZ ??antH 
% ??D3??nz = antH*0.6+20  gNB ??UAV 
% ??% ""??% ??0/1 break CA D3 ??% R9 VZ scene.commsVZ 30m??
if nargin < 3 || isempty(VZ)
    VZ = 30;                                   % 
end
pen = 0;
if isempty(comms) || size(traj,1) < 1, return; end

nNode = numel(comms);
%  +  VZ??nc = zeros(nNode,3); nr = zeros(nNode,1);
for k = 1:nNode
    nc(k,:) = comms(k).c;
    nr(k)   = comms(k).covR;
end

for i = 1:size(traj,1)
    p = traj(i,:);
    pen_i = inf;                              % 
    for k = 1:nNode
        dxy = norm(p(1:2) - nc(k,1:2));
        dz  = abs(p(3) - nc(k,3));
        % 
        % ??softplus ??        gapH = max(0, dxy - nr(k));
        gapZ = max(0, dz - VZ);
        gap  = sqrt(gapH^2 + gapZ^2);        % 0 ??        pen_i = min(pen_i, gap);             % ??0
    end
    pen = pen + pen_i;                        % ??end
end
