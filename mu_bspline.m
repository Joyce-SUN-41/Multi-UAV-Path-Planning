function P = mu_bspline(ctrl, start, goal, n)
% mu_bspline ?? B ??clamped??start/goal??%   ctrl   : m x 3 ??%   start  : 1 x 3
%   goal   : 1 x 3
%   n      : 
%   P      : n x 3 ??%
%  C = [start; ctrl; goal]K = m+2 
%        (p=3)  open ??p+1 
%       ??Cox-de Boor ??
if isempty(ctrl) || size(ctrl,1) < 1
    P = [linspace(start(1),goal(1),n).', ...
         linspace(start(2),goal(2),n).', ...
         linspace(start(3),goal(3),n).'];
    return;
end

C = [start; ctrl; goal];        % K x 3K = m+2
K = size(C,1);
% open uniformclamped??p+1 ??start/goal??%  (0,1) ???? M2 ??% linspace(0,1,K-p-1) ??0/1 clamp ??% nCtrl<4K<5 K-p-1<=0 ??%  mk=K-p-1??= (1:mk)/mk1??mk1=K-p??(0,1)??% ??K<p+2  p=K-2  1 mk=1 -> 0.5
p = 3;                                  % 样条阶数（三次 B 样条）
if K < p + 2
    p = max(1, K - 2);                   % ??
end
mk  = K - p - 1;
mk1 = K - p;
inner = (1:mk) / mk1;
U = [zeros(1, p+1), inner, ones(1, p+1)];

u = linspace(0,1,n);
P = zeros(n,3);
for d = 1:3
    P(:,d) = bspline_curve(C(:,d), U, p, u);
end
% clamped  start/goal
P(1,:) = start;
P(end,:) = goal;
end

function y = bspline_curve(C, U, p, u)
% Cox-de Boor 基函数直接求和（B样条定义式，零歧义）：
%   y(u) = sum_j N_{j,p}(u) * C(j)
% N_{j,p}(u) 由 Cox-de Boor 递归定义（见 bspline_basis）。
K = numel(C);
y = zeros(size(u));
for i = 1:numel(u)
    ui = u(i);
    N = zeros(K,1);
    for j = 1:K
        N(j) = bspline_basis(j, p, ui, U);
    end
    y(i) = sum(N .* C(:));
end
end

function N = bspline_basis(j, p, u, U)
% N_{j,p}(u)，j 为 1-based 控制点索引（对应 U 索引 j..j+p+1）
if j < 1 || j > numel(U) - p - 1
    N = 0; return;
end
if p == 0
    if u >= U(j) && u < U(j+1)
        N = 1;
    else
        N = 0;
    end
    % 右端点 clamp：u == U(end) 时最高次基取 1
    if abs(u - U(end)) < 1e-12 && j == numel(U) - p - 1
        N = 1;
    end
    return;
end
left  = bspline_basis(j,   p-1, u, U);
right = bspline_basis(j+1, p-1, u, U);
N = 0;
d1 = U(j+p)     - U(j);
d2 = U(j+p+1)   - U(j+1);
if d1 > 1e-12, N = N + (u - U(j))     / d1 * left;  end
if d2 > 1e-12, N = N + (U(j+p+1) - u) / d2 * right; end
end

function k = augknt(breaks, p)
% ??open uniform ??p+1 
k = [repmat(breaks(1),1,p+1), breaks, repmat(breaks(end),1,p+1)];
end
