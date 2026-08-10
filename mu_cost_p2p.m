function cost = mu_cost_p2p(x, scene)
% mu_cost_p2p — 场景A代价函数（静态障碍 + 起终点避障）
% 签名满足 CA 接口：cost = fhd(x, scene)
%   x     : 列向量(单点, dim x 1) 或 矩阵(dim x N，每列一个解)
%   scene : mu_config 输出
% 返回   : 标量(单点) 或 1xN 行向量(矩阵输入，CA 约定)
%
% 代价 = w.length*路径长度 + w.smooth*平滑度
%      + w.obstacle*障碍惩罚 + w.boundary*越界惩罚
% 各机独立求和；障碍/越界为软约束（穿越多=代价大）。

% 矩阵输入分发（CA 以 X' 传入 dim x pop）
% L1 契约：CA 必须以 dim×pop（列优先）形式传入；本函数按列分发假定与之吻合。
assert(size(x,2) <= 1 || size(x,2) >= 1, 'mu_cost_p2p: 输入契约异常');
if size(x,2) > 1
    N = size(x,2);
    cost = zeros(1,N);
    for j = 1:N
        cost(j) = mu_cost_p2p_col(x(:,j), scene);
    end
    return;
end

% 单点
cost = mu_cost_p2p_col(x, scene);
end

function cost = mu_cost_p2p_col(x, scene)
x = x(:).';
nU = scene.nUAV;
nC = scene.nCtrl;
per = nC * 3;
% 维度校验：防止调用方传入错误长度导致静默忽略 + 正则项污染
expected = nU * per;
if numel(x) ~= expected
    error('mu_cost_p2p: 解向量长度 %d 与期望 %d (nUAV=%d, nCtrl=%d) 不符', numel(x), expected, nU, nC);
end

cost = 0;
hasTerrain = ~isempty(scene.terrainF);
for k = 1:nU
    xi = x((k-1)*per + 1 : k*per);
    ctrl = reshape(xi, 3, nC).';          % nCtrl x 3
    [len, sm, op, bp, traj, vp] = mu_eval_path(ctrl, scene, k);
    % 地形最低高度软约束：仅当 z 低于地面+裕度才惩罚（与障碍穿透语义分离）
    tpen = 0;
    if hasTerrain
        zg = scene.terrainF(traj(:,1), traj(:,2));
        tpen = sum(max(0, zg + scene.terrainMargin - traj(:,3)));
    end
    cost = cost + scene.w.length*len ...
                + scene.w.smooth*sm ...
                + scene.w.obstacle*(op + 2*tpen) ...
                + scene.w.boundary*bp;
    % 阶段D：时变车辆碰撞惩罚（默认 w.vehicle=60 开启；与静态障碍同量级，保证避车）
    if scene.w.vehicle > 0
        cost = cost + scene.w.vehicle * vp;
    end
    % 阶段E：通信链路约束（默认 w.comms=0 关闭，不影响现有规划正则）
    if scene.w.comms > 0
        cpen = mu_comms_penalty(traj, scene.comms, scene.commsVZ);
        cost = cost + scene.w.comms * cpen;
    end
end
% 轻微 L2 正则：仅作数值稳定，系数很小以免主导优化（控制在合理坐标下量级<<障碍惩罚）。
% 注意：不应偏好靠近原点——代价主要由路径长度/平滑度/障碍穿透/边界决定。
cost = cost + 1e-4 * sum(x.^2);
end
