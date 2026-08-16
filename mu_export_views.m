function mu_export_views(sc, trajs, mode, outDir, tag, ts, costStr, varargin)
% mu_export_views — 把同一份规划结果渲染为多个互补视角的图，便于看清路径
%
% 视角设计（针对"单视角看不清路径"的痛点）：
%   persp  透视 3D [38 26]      总体态势（默认）
%   top    正俯视   [0 90]       XY 平面走向、绕障、起终点/任务分布（最看清怎么绕）
%   sideX  沿 X 侧视 [0 0]       高度分层、爬升/下降、与楼高关系
%   sideY  沿 Y 侧视 [90 0]      另一方向高度剖面
%   close  近景放大              拉近单架最复杂 UAV，看清局部绕障细节
%
% 用法:
%   mu_export_views(sc, trajs, mode, outDir, tag, ts, costStr)
%   若不关心代价，costStr 可传 '' 或省略（标题用 tag 占位）
%   可选 Name/Value: 'views', {'top','sideX',...} 限定导出子集（默认全部）
%
% 每个视角输出 run_<tag>_<view>_<ts>.png 与 .eps（300dpi），复用 mu_savefig。

p = inputParser;
addParameter(p,'costStr', '', @ischar);
addParameter(p,'views',  {}, @iscell);   % 空 = 全部
parse(p, 'costStr', costStr, varargin{:});

if nargin < 6 || isempty(ts)
    ts = char(datetime('now','Format','yyyy-MM-dd_HHmmss'));
end
if nargin < 5 || isempty(tag)
    tag = matlab.lang.makeValidName(sprintf('%s_%s', mode, sc.difficulty));
end
if ~exist(outDir,'dir'), mkdir(outDir); end

% 统一标题前缀（含代价便于归档对照）
if isempty(p.Results.costStr)
    titleBase = sprintf('%s  (%s)', upper(tag), sc.difficulty);
else
    titleBase = sprintf('%s  (%s)  cost=%s', upper(tag), sc.difficulty, p.Results.costStr);
end

% 视角表：{viewName, azimuth, elevation}
allViews = {
    'persp', 38, 26;
    'top',   0, 90;
    'sideX', 0, 0;
    'sideY', 90, 0;
    };
sel = p.Results.views;
if isempty(sel)
    views = allViews;
else
    nA = size(allViews,1);
    keep = false(nA,1);
    for s = 1:numel(sel)
        keep = keep | strcmpi(allViews(:,1), sel{s});
    end
    if any(keep)
        views = allViews(keep,:);
    else
        views = allViews;   % 子集名均未匹配，回退到全部
    end
end

% ---- 1) 多视角标准渲染 ----
FIG_W = 760; FIG_H = 760;          % 正方形画布，避免存图被拉伸出多余边
for v = 1:size(views,1)
    vn = views{v,1}; az = views{v,2}; el = views{v,3};
    fig = figure('Color','w','Visible','off', 'Position',[100 100 FIG_W FIG_H]);
    ax = axes('Parent',fig,'Color','w');
    hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
    sc.tCur = 0;
    mu_draw_scene(sc, trajs, mode, ax);
    view(ax, [az el]);
    % 所有标准视角：XY 精确裁切到外框 [-500,500] 并强制正方形画布（含透视图留白），
    % Z 轴不锁死，交给 MATLAB 按内容自适应（楼高 ~100、封顶 100，留白更紧凑合适）。
    b = sc.bounds;
    ax.XLim = [b(1) b(2)]; ax.YLim = [b(3) b(4)];
    ax.ZLim = [-50 400];
    axis(ax,'square');
    title(ax, sprintf('%s  [%s]', titleBase, vn), 'Interpreter','none');
    drawnow;
    pngFile = fullfile(outDir, sprintf('run_%s_%s_%s.png', tag, vn, ts));
    epsFile = fullfile(outDir, sprintf('run_%s_%s_%s.eps', tag, vn, ts));
    mu_savefig(fig, pngFile, 'png', 300);
    mu_savefig(fig, epsFile, 'eps', 300);
    close(fig);
end

% ---- 2) 近景放大：聚焦轨迹最"绕"的一架 UAV（按弧长选最长）----
if ~isempty(trajs)
    nT = numel(trajs);
    lens = zeros(nT,1);
    for j = 1:nT
        Tj = trajs{j};
        if ~isempty(Tj)
            lens(j) = sum(sqrt(sum(diff(Tj,1,1).^2,2)));
        end
    end
    [~, ki] = max(lens);
    T = trajs{ki};
    cxm = mean(T(:,1)); cym = mean(T(:,2)); czm = mean(T(:,3));
    fig = figure('Color','w','Visible','off', 'Position',[100 100 760 760]);
    ax = axes('Parent',fig,'Color','w');
    hold(ax,'on'); grid(ax,'on'); axis(ax,'square');
    sc.tCur = 0;
    mu_draw_scene(sc, trajs, mode, ax);
    view(ax, [38 26]);
    ax.ZLim = [-50 400];
    camtarget(ax, [cxm cym czm]);                 % 对准该机质心
    camva(ax, 18);                                 % 窄视野 = 放大
    title(ax, sprintf('%s  [closeup UAV%d]', titleBase, ki), 'Interpreter','none');
    drawnow;
    pngFile = fullfile(outDir, sprintf('run_%s_closeup_%s.png', tag, ts));
    epsFile = fullfile(outDir, sprintf('run_%s_closeup_%s.eps', tag, ts));
    mu_savefig(fig, pngFile, 'png', 300);
    mu_savefig(fig, epsFile, 'eps', 300);
    close(fig);
end
end
