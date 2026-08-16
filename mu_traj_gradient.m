function h = mu_traj_gradient(ax, T, baseColor, opts)
% mu_traj_gradient - draw a 3D trajectory as a CLEAR solid line (white-bg priority)
%   ax        target axes
%   T         Nx3 waypoints (time ordered)
%   baseColor 1x3 base color (per-UAV)
%   opts      struct: width, fade, whitebg, arrow, refine, gradient,
%                     startColor / goalColor
%
% White background design: faint gray halo instead of thick white halo to
% avoid white clutter when many trajectories overlap.

n = size(T,1);
if n < 2, h = []; return; end
if nargin < 4, opts = struct(); end
if ~isfield(opts,'width'),   opts.width   = 3.2; end
if ~isfield(opts,'fade'),    opts.fade    = 0.15; end
if ~isfield(opts,'whitebg'), opts.whitebg = false; end
if ~isfield(opts,'arrow'),   opts.arrow   = true;  end
if ~isfield(opts,'refine'),  opts.refine  = 2;     end
if ~isfield(opts,'startColor'), opts.startColor = [0.20 0.75 0.35]; end
if ~isfield(opts,'goalColor'),  opts.goalColor  = [0.95 0.35 0.55]; end

hold(ax,'on');

% optional interpolation refinement
if opts.refine > 1 && n >= 2
    tt = (0:n-1)'/(n-1);
    ti = linspace(0,1,(n-1)*opts.refine+1)';
    T = [interp1(tt, T(:,1), ti), interp1(tt, T(:,2), ti), interp1(tt, T(:,3), ti)];
    n = size(T,1);
end

lineCol = min(1, baseColor .^ 0.55);

if isfield(opts,'gradient') && opts.gradient
    colStart = opts.startColor;
    colGoal  = opts.goalColor;
    gradOn = true;
else
    colStart = lineCol; colGoal = lineCol; gradOn = false;
end

fadeN = max(1, floor(n*opts.fade));
fadeCol = min(1, baseColor*0.72 + 0.20);
if opts.fade <= 0
    midCol = baseColor;   % 纯色实线：保持原始高饱和度，不做提亮
else
    midCol = lineCol;
end

if opts.whitebg
    % faint gray halo, not thick white, to keep multi-UAV plots readable
    plot3(ax, T(:,1), T(:,2), T(:,3), 'Color',[0.92 0.92 0.94], 'LineWidth',opts.width*0.6, ...
        'LineStyle','-', 'Tag','trajHalo');
end

if ~gradOn
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
else
    segN = n - 1;
    cols = zeros(segN, 3);
    for s = 1:segN
        f = (s - 0.5) / segN;
        cols(s,:) = (1-f)*colStart + f*colGoal;
    end
    if fadeN >= 1 && n > 2*fadeN
        plot3(ax, T(1:fadeN+1,1), T(1:fadeN+1,2), T(1:fadeN+1,3), ...
            'Color',fadeCol, 'LineWidth',opts.width*0.9, 'LineStyle','-', 'Tag','trajFade');
        plot3(ax, T(n-fadeN:end,1), T(n-fadeN:end,2), T(n-fadeN:end,3), ...
            'Color',fadeCol, 'LineWidth',opts.width*0.9, 'LineStyle','-', 'Tag','trajFade');
    end
    for s = max(fadeN+1,1):min(n-fadeN, n-1)
        plot3(ax, T(s:s+1,1), T(s:s+1,2), T(s:s+1,3), ...
            'Color',cols(s,:), 'LineWidth',opts.width, 'LineStyle','-', 'Tag','trajCore');
    end
end

% direction arrows: smaller to reduce clutter in dense plots
if opts.arrow
    drawArrow(ax, T(1,:), T(min(3,n),:), colStart, 42);
    drawArrow(ax, T(end,:), T(max(1,n-2),:), colGoal, 42);
    step = max(1, floor(n/4));
    for a = step:step:n-1
        bIdx = min(n, floor(a + step/2));
        f = (a - 0.5) / (n-1);
        drawArrow(ax, T(a,:), T(bIdx,:), (1-f)*colStart + f*colGoal, 22);
    end
end
end

function drawArrow(ax, p, q, col, sz)
% direction arrow marker at point p looking toward q
hold(ax,'on');
d = q - p; nd = norm(d+1e-9); d = d/nd;
scatter3(ax, p(1), p(2), p(3), sz, col, 'filled', 'Marker','>', ...
    'MarkerEdgeColor',[0.25 0.25 0.25], 'LineWidth',0.5);
end
