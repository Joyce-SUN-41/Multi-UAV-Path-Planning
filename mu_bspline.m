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
% ??C(Kx1) u(1xN)  B 
K = numel(C);
y = zeros(size(u));
uMax = U(end); uMin = U(1);
for i = 1:numel(u)
    ui = u(i);
    % M2clamped B ??u ??    % ??0 ??0  de Boor  u 
    %  (uMin, uMax) ??clamped ??    %  mu_bspline  start/goal ??    if ui <= uMin, ui = uMin + 1e-9; end
    if ui >= uMax, ui = uMax - 1e-9; end
    % U(k) <= ui < U(k+1)k  [p+1, K]
    k = find(U(1:end-1) <= ui + 1e-12, 1, 'last');
    if isempty(k) || k < p+1, k = p+1; end
    if k > K, k = K; end
    %  k-p .. k
    idx = (k-p) : k;
    d = C(idx).';                    % (p+1) x 1
    for r = 1:p
        for j = 1:(p+1-r)
            ii = idx(j);
            denom = U(ii + p - r + 1) - U(ii);
            if abs(denom) < 1e-12
                alpha = 0;   %  alpha ??0
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
% ??open uniform ??p+1 
k = [repmat(breaks(1),1,p+1), breaks, repmat(breaks(end),1,p+1)];
end
