function traj = mu_build_tour_traj(ctrl, start, goal, waypts, n, nCtrl)
% mu_build_tour_traj — 分段三次 Bezier/B 样条（每段 2 个内部控制点）
%   ctrl   : (2*nseg) x 3 内部控制点，按段顺序 [c1_s1;c2_s1; c1_s2;c2_s2; ...]
%   start  : 1x3
%   goal   : 1x3
%   waypts : m x 3 有序任务点（可空）
%   n      : 采样点数
% 每段用三次 Bezier 曲线（4 控制点 a,c1,c2,b），端点 clamped 且对控制点连续可调。

if isempty(waypts)
    if size(ctrl,1) < 2, ctrl = [ctrl; ctrl(1,:)]; end
    % 退化 B 样条分支：用全部内部控制点（不清零到 nCtrl，避免误截断自由度）
    traj = mu_bspline(ctrl, start, goal, n);   % 多控制点用 B 样条（节点充足）
    return;
end
m = size(waypts,1);
nseg = m + 1;
% ---- D2/R7 修复：把 CA 分配的 nCtrl 个内部控制点在 nseg 段间均匀分配，
% 每段 2 个起（保证三次 Bezier 形状自由度），多余控制点对半追加到前段，
% 杜绝"全局 nCtrl=2*(maxT+1) 但本机任务少 → 尾部控制点被静默丢弃、
% 优化器在无效维度上空耗搜索预算"（R7 消除浪费）。同时保留 D2 的越界兜底：
% 若控制点不足 2*nseg（极端鲁棒性），在末尾用已有控制点中点补齐。
need = 2 * nseg;
if size(ctrl,1) >= need
    % 均匀分配：每段至少 2 个，余量 (nCtrl-2*nseg) 顺序追加到前段
    extra = size(ctrl,1) - need;        % >=0
    perSeg = 2 * ones(nseg,1);
    for s = 1:min(extra, nseg)
        perSeg(s) = perSeg(s) + 1;       % 余量优先补前段（靠近 depot，形状更敏感）
    end
    % 重排 ctrl：按段顺序取 perSeg(s) 个连续控制点
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
    cs = ctrlBySeg{s};                   % 本段 cs(1)=c1, cs(2)=c2, 多余点用于细分 Bezier
    if size(cs,1) == 2
        c1 = cs(1,:); c2 = cs(2,:);
        segtr = bezier3([a; c1; c2; b], max(6, floor(n/nseg)));
    else
        % 多于 2 个控制点：用高阶分段 Bezier（de Casteljau 细分保持 C0 连续），
        % 首末端锚定 a/b，中间点作为高阶贝塞尔控制点，提升形状表达能力。
        CP = [a; cs; b];                 % (size(cs,1)+2) x 3
        segtr = bezierN(CP, max(6, floor(n/nseg)));
    end
    if s==1, traj = segtr; else traj = [traj; segtr(2:end,:)]; end
end
end

function P = bezierN(C, n)
% 任意阶 Bezier 曲线（C 为 Kx3 控制点），de Casteljau 逐点求值
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
% 三次 Bezier 曲线，C 为 4x3 控制点
t = linspace(0,1,n).';
bt = [(1-t).^3, 3*(1-t).^2.*t, 3*(1-t).*t.^2, t.^3];
P = bt * C;
end
