function traj = mu_build_tour_traj(ctrl, start, goal, waypts, n)
% mu_build_tour_traj : build a tour trajectory via piecewise Bezier / B-spline
%   ctrl   : (2*nseg) x 3  [c1_s1;c2_s1; c1_s2;c2_s2; ...]
%   start  : 1x3
%   goal   : 1x3
%   waypts : m x 3
%   n      : samples per trajectory
% (控制点数量由 ctrl 实际行数决定，不再需要 nCtrl 参数；R7/M1 冗余清理)
if isempty(waypts)
    if size(ctrl,1) < 2, ctrl = [ctrl; ctrl(1,:)]; end
    % B-spline（无途经点时退化为单段样条）
    traj = mu_bspline(ctrl, start, goal, n);
    return;
end
m = size(waypts,1);
nseg = m + 1;
need = 2 * nseg;                 % D2/R7 : baseline control points needed; pad below if short
% ---- D2/R7 CA nCtrl vs nseg : 2 control points per Bezier segment
% nCtrl = 2*(maxT+1) ; R7/D2
if size(ctrl,1) >= need
    % distribute the extra (nCtrl - 2*nseg) control points
    extra = size(ctrl,1) - need;        % >=0
    perSeg = 2 * ones(nseg,1);
    for s = 1:min(extra, nseg)
        perSeg(s) = perSeg(s) + 1;       % give depot/longer segments an extra point
    end
    % split ctrl by perSeg(s)
    ctrlBySeg = cell(nseg,1);
    off = 1;
    for s = 1:nseg
        ctrlBySeg{s} = ctrl(off : off+perSeg(s)-1, :);
        off = off + perSeg(s);
    end
else
    if size(ctrl,1) >= 1
        mid = mean(ctrl,1);
    else
        mid = (start + goal)/2;
    end
    ctrl = [ctrl; repmat(mid, need - size(ctrl,1), 3)];
    ctrlBySeg = cell(nseg,1);
    off = 1;
    for s = 1:nseg
        ctrlBySeg{s} = ctrl(off : off+1, :);
        off = off + 2;
    end
end
traj = [];
for s = 1:nseg
    if s==1, a = start; else a = waypts(s-1,:); end
    if s<nseg, b = waypts(s,:); else b = goal; end
    cs = ctrlBySeg{s};                   % cs(1)=c1, cs(2)=c2, Bezier segment
    if size(cs,1) == 2
        c1 = cs(1,:); c2 = cs(2,:);
        segtr = bezier3([a; c1; c2; b], max(6, floor(n/nseg)));
    else
        % general-degree Bezier (de Casteljau), C0 continuity
        % CP includes a/b endpoints
        CP = [a; cs; b];                 % (size(cs,1)+2) x 3
        segtr = bezierN(CP, max(6, floor(n/nseg)));
    end
    if s==1, traj = segtr; else traj = [traj; segtr(2:end,:)]; end
end
end

function P = bezierN(C, n)
% general-degree Bezier, C is Kx3, de Casteljau
t = linspace(0,1,n).';
K = size(C,1);
P = zeros(n,3);
for i = 1:n
    Q = C;
    for r = 1:K-1
        Q = (1-t(i))*Q(1:end-1,:) + t(i)*Q(2:end,:);
    end
    P(i,:) = Q(1,:);
end
end

function P = bezier3(C, n)
% cubic Bezier, C is 4x3
t = linspace(0,1,n).';
bt = [(1-t).^3, 3*(1-t).^2.*t, 3*(1-t).*t.^2, t.^3];
P = bt * C;
end
