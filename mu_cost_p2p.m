function cost = mu_cost_p2p(x, scene)
% mu_cost_p2p ??A??+ 
%  CA cost = fhd(x, scene)
%   x     : ??, dim x 1) ??(dim x N)
%   scene : mu_config 
%    : () ??1xN ??CA )
%
%  = w.length* + w.smooth*??%      + w.obstacle* + w.boundary*
% ??=??
% CA ??X'  dim x pop??% L1 CA ??dimpop??assert(size(x,2) <= 1 || size(x,2) >= 1, 'mu_cost_p2p: ');
if size(x,2) > 1
    N = size(x,2);
    cost = zeros(1,N);
    for j = 1:N
        cost(j) = mu_cost_p2p_col(x(:,j), scene);
    end
    return;
end

%
cost = mu_cost_p2p_col(x, scene);

function cost = mu_cost_p2p_col(x, scene)
x = x(:).';
nU = scene.nUAV;
nC = scene.nCtrl;
per = nC * 3;
%  + ??expected = nU * per;
expected = nU * per;
if numel(x) ~= expected
    error('mu_cost_p2p: ??%d ??%d (nUAV=%d, nCtrl=%d) ', numel(x), expected, nU, nC);
end

cost = 0;
hasTerrain = ~isempty(scene.terrainF);
for k = 1:nU
    xi = x((k-1)*per + 1 : k*per);
    ctrl = reshape(xi, 3, nC).';          % nCtrl x 3
    [len, sm, op, bp, traj, vp] = mu_eval_path(ctrl, scene, k);
    % ??z +
    tpen = 0;
    if hasTerrain
        zg = scene.terrainF(traj(:,1), traj(:,2));
        tpen = sum(max(0, zg + scene.terrainMargin - traj(:,3)));
    end
    cost = cost + scene.w.length*len ...
                + scene.w.smooth*sm ...
                + scene.w.obstacle*(op + 2*tpen) ...
                + scene.w.boundary*bp;
    % D w.vehicle=60 
    if scene.w.vehicle > 0
        cost = cost + scene.w.vehicle * vp;
    end
    % E??w.comms=0 ??
    if scene.w.comms > 0
        cpen = mu_comms_penalty(traj, scene.comms, scene.commsVZ);
        cost = cost + scene.w.comms * cpen;
    end
end
%  L2 ??<??% /??????cost = cost + 1e-4 * sum(x.^2);
