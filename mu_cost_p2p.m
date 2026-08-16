function cost = mu_cost_p2p(x, scene)
% mu_cost_p2p : p2p-mode CA cost = fhd(x, scene)
%   x     : (dim x 1) or (dim x N)
%   scene : mu_config
% L1 列优先契约：CAv9x 以 dim x N 矩阵按列传入，代价须支持矩阵分发
if size(x,2) > 1
    N = size(x,2);
    cost = zeros(1,N);
    for j = 1:N
        cost(j) = mu_cost_p2p_col(x(:,j), scene);
    end
    assert(numel(cost)==N, 'mu_cost_p2p: 矩阵分发返回长度与列数 %d 不符 (%d)', N, numel(cost));
    return;
end

cost = mu_cost_p2p_col(x, scene);
end

function cost = mu_cost_p2p_col(x, scene)
x = x(:).';
nU = scene.nUAV;
nC = scene.nCtrl;
per = nC * 3;
expected = nU * per;
if numel(x) ~= expected
    error('mu_cost_p2p: dim %d expected %d (nUAV=%d, nCtrl=%d) ', numel(x), expected, nU, nC);
end

% 并行评估开关：逐 UAV 轨迹评估相互独立，可用 parfor 分摊到多 worker。
% scene.useParallel 为假、未装并行工具箱、或当前未开并行池时退化为普通 for，零风险。
hasPT = ~isempty(ver('parallel'));
usePar = isfield(scene,'useParallel') && scene.useParallel && hasPT && ~isempty(gcp('nocreate'));

cost = 0;
hasTerrain = ~isempty(scene.terrainF);
cpart = zeros(nU, 1);                     % 并行收集每机代价，避免 parfor 内累加竞争
if usePar
    parfor k = 1:nU
        cpart(k) = mu_cost_p2p_uav(x, k, (k-1)*per + 1 : k*per, scene, nC, hasTerrain);
    end
else
    for k = 1:nU
        cpart(k) = mu_cost_p2p_uav(x, k, (k-1)*per + 1 : k*per, scene, nC, hasTerrain);
    end
end
cost = sum(cpart);
% L2 正则项
cost = cost + 1e-4 * sum(x.^2);

end

function c = mu_cost_p2p_uav(x, k, idx, scene, nC, hasTerrain)
% 单架无人机代价（parfor 切片）：输入该机控制点区间，独立计算后返回标量代价值。
xi = x(idx);
ctrl = reshape(xi, 3, nC).';          % nCtrl x 3
[len, sm, op, bp, traj, vp] = mu_eval_path(ctrl, scene, k);
tpen = 0;
if hasTerrain
    zg = scene.terrainF(traj(:,1), traj(:,2));
    tpen = sum(max(0, zg + scene.terrainMargin - traj(:,3)));
end
c = scene.w.length*len ...
    + scene.w.smooth*sm ...
    + scene.w.obstacle*(op + 2*tpen) ...
    + scene.w.boundary*bp;
if scene.w.vehicle > 0
    c = c + scene.w.vehicle * vp;
end
if scene.w.comms > 0
    cpen = mu_comms_penalty(traj, scene.comms, scene.commsVZ);
    c = c + scene.w.comms * cpen;
end
if scene.w.height > 0
    hpen = sum(max(0, traj(:,3) - scene.flightCeiling).^2);
    c = c + scene.w.height * hpen;
end
end
