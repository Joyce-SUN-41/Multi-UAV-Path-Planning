function h = mu_traj_gradient(ax, T, baseColor, opts)
% mu_traj_gradient - draw a 3D trajectory as a CLEAR solid line (white-bg priority)
%   ax        target axes
%   T         Nx3 waypoints (time ordered)
%   baseColor 1x3 base color (per-UAV)
%   opts      struct: width (default 3.2), fade (head/tail dim frac), whitebg,
%                     arrow (沿程方向箭头), refine (渲染插值加密)
%
% 清晰优先：等宽实线中心线（不透明深饱和）+ 白色描边（防交叠糊团）。
% 仅首尾轻微淡出（真正生效：首尾 fade 比例内降亮度模拟淡出），核心连续清晰。
% 方向箭头：起点→终点 + 沿程稀疏小箭头，增强路径可读性。
n = size(T,1);
if n < 2, h = []; return; end
if nargin < 4, opts = struct(); end
if ~isfield(opts,'width'),   opts.width   = 3.2; end
if ~isfield(opts,'fade'),    opts.fade    = 0.15; end
if ~isfield(opts,'whitebg'), opts.whitebg = false; end
if ~isfield(opts,'arrow'),   opts.arrow   = true;  end
if ~isfield(opts,'refine'),  opts.refine  = 2;     end   % 渲染插值加密倍数

hold(ax,'on');

% ---- 渲染层加密（纯视觉，不改代价/数据）----
if opts.refine > 1 && n >= 2
    tt = (0:n-1)'/(n-1);
    ti = linspace(0,1,(n-1)*opts.refine+1)';
    T = [interp1(tt, T(:,1), ti), interp1(tt, T(:,2), ti), interp1(tt, T(:,3), ti)];
    n = size(T,1);
end

% 主体颜色：加深饱和（白底更清晰）
lineCol = min(1, baseColor .^ 0.55);

% 首尾淡出占比（按总点数）
fadeN = max(1, floor(n*opts.fade));
% 淡出段用更浅的颜色模拟淡出（line 不支持逐段 alpha 的稳健画法）
fadeCol = min(1, baseColor*0.72 + 0.20);   % 浅化（接近背景）
midCol  = lineCol;                          % 中段全不透明

if opts.whitebg
    % 白色描边（粗，放底层）防交叠糊团
    plot3(ax, T(:,1), T(:,2), T(:,3), 'Color',[1 1 1], 'LineWidth',opts.width+1.4, ...
        'LineStyle','-', 'Tag','trajHalo');
end

% ---- 分三段绘制：头淡出 / 中段核心 / 尾淡出 ----
% 头淡出段 [1, fadeN+1]
if fadeN >= 1 && n > 2*fadeN
    plot3(ax, T(1:fadeN+1,1), T(1:fadeN+1,2), T(1:fadeN+1,3), ...
        'Color',fadeCol, 'LineWidth',opts.width*0.9, 'LineStyle','-', 'Tag','trajFade');
    plot3(ax, T(fadeN:n-fadeN,1), T(fadeN:n-fadeN,2), T(fadeN:n-fadeN,3), ...
        'Color',midCol, 'LineWidth',opts.width, 'LineStyle','-', 'Tag','trajCore');
    plot3(ax, T(n-fadeN:end,1), T(n-fadeN:end,2), T(n-fadeN:end,3), ...
        'Color',fadeCol, 'LineWidth',opts.width*0.9, 'LineStyle','-', 'Tag','trajFade');
else
    h = plot3(ax, T(:,1), T(:,2), T(:,3), 'Color',midCol, 'LineWidth',opts.width, ...
        'LineStyle','-', 'Tag','trajCore');
end

% ---- 方向箭头 ----
if opts.arrow
    % 起点→终点主方向大箭头
    drawArrow(ax, T(1,:), T(min(3,n),:), midCol, 55);
    drawArrow(ax, T(end,:), T(max(1,n-2),:), midCol, 55);
    % 沿程稀疏小箭头（每 ~25% 一段一个，指示行进方向）
    step = max(1, floor(n/4));
    for a = step:step:n-1
        bIdx = min(n, floor(a + step/2));
        drawArrow(ax, T(a,:), T(bIdx,:), midCol, 30);
    end
end
end

function drawArrow(ax, p, q, col, sz)
% 在 p 处画一个指向 q 的小箭头标记（方向感）
hold(ax,'on');
d = q - p; nd = norm(d+1e-9); d = d/nd;
scatter3(ax, p(1), p(2), p(3), sz, col, 'filled', 'Marker','>', ...
    'MarkerEdgeColor',[0.25 0.25 0.25], 'LineWidth',0.6);
end
