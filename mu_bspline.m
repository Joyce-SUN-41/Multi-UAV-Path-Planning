function P = mu_bspline(ctrl, start, goal, n)
% mu_bspline — 由内部控制点生成三次 B 样条轨迹（端点 clamped，必过 start/goal）
%   ctrl   : m x 3 内部控制点（不含端点）
%   start  : 1 x 3
%   goal   : 1 x 3
%   n      : 采样点数
%   P      : n x 3 轨迹点
%
% 实现：控制多边形 C = [start; ctrl; goal]（K = m+2 个控制点），
%       三次 (p=3) 均匀 open 节点向量（端点重复 p+1 次以保证曲线经过端点），
%       用 Cox-de Boor 基函数求和求值。对单点坐标独立计算。

if isempty(ctrl) || size(ctrl,1) < 1
    P = [linspace(start(1),goal(1),n).', ...
         linspace(start(2),goal(2),n).', ...
         linspace(start(3),goal(3),n).'];
    return;
end

C = [start; ctrl; goal];        % K x 3，K = m+2
K = size(C,1);
% open uniform（clamped）节点向量：端点各重复 p+1 次保证曲线经过 start/goal，
% 中间均匀节点严格落在开区间 (0,1) 内 —— 这是 M2 修复的关键：旧版用
% linspace(0,1,K-p-1) 把中间节点取到 0/1，与 clamp 端点重合导致曲线过约束、
% 自由度被悄悄压低；nCtrl<4（K<5）时 K-p-1<=0 还会返回空节点向量。
% 修复：内部节点数 mk=K-p-1，位置 = (1:mk)/mk1，其中 mk1=K-p（落在 (0,1)）。
% 当 K<p+2 时，自动降阶 p=K-2 以保证至少有 1 个内部节点（mk=1 -> 0.5）。
p = 3;                                  % 默认三次
if K < p + 2
    p = max(1, K - 2);                   % 降阶到可支撑的最低次数
end
mk  = K - p - 1;                         % 内部节点个数（>=1）
mk1 = K - p;                             % 内部节点跨度的分母
inner = (1:mk) / mk1;                    % 严格落在 (0,1) 内，不与端点 clamp 重合
U = [zeros(1, p+1), inner, ones(1, p+1)];

u = linspace(0,1,n);
P = zeros(n,3);
for d = 1:3
    P(:,d) = bspline_curve(C(:,d), U, p, u);
end
% clamped 性质兜底：强制首末点精确等于 start/goal（消除端点数值退化残差）
P(1,:) = start;
P(end,:) = goal;
end

function y = bspline_curve(C, U, p, u)
% 对单维控制坐标 C(Kx1)，在参数 u(1xN) 上求三次 B 样条
K = numel(C);
y = zeros(size(u));
uMax = U(end); uMin = U(1);
for i = 1:numel(u)
    ui = u(i);
    % 端点夹紧保护（M2）：clamped B 样条在 u 恰等于端点重复节点时，
    % 度-0 基函数全为 0 导致 de Boor 数值退化（得出错值）。把 u 夹到
    % 开区间 (uMin, uMax) 内微小偏移，既保留 clamped 必过端点的性质，
    % 又避免端点退化。端点值由 mu_bspline 末尾强制对齐 start/goal 兜底。
    if ui <= uMin, ui = uMin + 1e-9; end
    if ui >= uMax, ui = uMax - 1e-9; end
    % 找到节点区间：U(k) <= ui < U(k+1)，k 属于 [p+1, K]
    k = find(U(1:end-1) <= ui + 1e-12, 1, 'last');
    if isempty(k) || k < p+1, k = p+1; end
    if k > K, k = K; end
    % 局部控制点索引 k-p .. k
    idx = (k-p) : k;
    d = C(idx).';                    % (p+1) x 1
    for r = 1:p
        for j = 1:(p+1-r)
            ii = idx(j);
            denom = U(ii + p - r + 1) - U(ii);
            if abs(denom) < 1e-12
                alpha = 0;   % 重复节点约定：分母为零时 alpha 取 0
            else
                alpha = (ui - U(ii)) / denom;
            end
            d(j) = (1 - alpha) * d(j) + alpha * d(j+1);
        end
    end
    y(i) = d(1);
end
end

function k = augknt(breaks, p)
% 构造 open uniform 节点向量（端点重复 p+1 次）
k = [repmat(breaks(1),1,p+1), breaks, repmat(breaks(end),1,p+1)];
end
