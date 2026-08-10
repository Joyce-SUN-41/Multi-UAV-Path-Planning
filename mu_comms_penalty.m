function pen = mu_comms_penalty(traj, comms, VZ)
% mu_comms_penalty — 阶段E：通信链路约束软惩罚
%   轨迹 traj (n x 3) 上的点需落在至少一个 comms 节点的覆盖半径内，
%   否则按"超出覆盖的距离"累加平滑软惩罚（鼓励 UAV 保持在网络连通空域）。
% 覆盖用三维半径近似：水平半径 covR，竖直容差 VZ 与天线挂高 antH 脱钩
% （修复 D3：旧版 nz = antH*0.6+20 随挂高无界放大，高空 gNB 让低空 UAV 自动
% 判为覆盖，链路约束失效）。竖直容差固定为与场景尺度相关的小值，避免吞掉空域。
% 覆盖判定改为连续可微：对每个点取"距最近节点覆盖边界的平滑缺口"累加，
% 移除硬 0/1 break，使 CA 优化梯度连续、稳定（D3 第二项）。
% R9 修复：VZ 由调用方传入（scene.commsVZ，随空域尺度缩放），缺省回退 30m。

if nargin < 3 || isempty(VZ)
    VZ = 30;                                   % 缺省回退（与场景无关的小值）
end
pen = 0;
if isempty(comms) || size(traj,1) < 1, return; end

nNode = numel(comms);
% 预取节点位置与覆盖（水平半径 + 竖直容差 VZ）
nc = zeros(nNode,3); nr = zeros(nNode,1);
for k = 1:nNode
    nc(k,:) = comms(k).c;
    nr(k)   = comms(k).covR;
end

for i = 1:size(traj,1)
    p = traj(i,:);
    pen_i = inf;                              % 取所有节点中最小缺口（最近节点覆盖）
    for k = 1:nNode
        dxy = norm(p(1:2) - nc(k,1:2));
        dz  = abs(p(3) - nc(k,3));
        % 距该节点覆盖边界的缺口：水平与竖直方向分别平滑截断，
        % 用 softplus 形式保证梯度连续（边界处不折跃）。
        gapH = max(0, dxy - nr(k));
        gapZ = max(0, dz - VZ);
        gap  = sqrt(gapH^2 + gapZ^2);        % 该节点缺口（0 表示在覆盖内）
        pen_i = min(pen_i, gap);             % 覆盖内等价于某节点缺口=0
    end
    pen = pen + pen_i;                        % 全轨迹累加最近节点缺口之和
end
end
