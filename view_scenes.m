function view_scenes(varargin)
% view_scenes — 只看场景"地图"本身（不跑优化，秒级出图）
%   默认渲染三张代表性场景地图到 results/ 目录：
%     p2p_medium / tour_medium / tour_hard
%   仅构造场景 + 渲染（含楼/公路/高架桥/仓库/客户点/通信/车辆），不规划轨迹，
%   因此无无人机航迹，适合单独查看城市布局。
%
% 自定义：传入 cell 数组覆盖默认场景，例如
%   view_scenes({struct('tag','my','mode','tour','diff','hard','nUAV',12,'seed',7)})
%
% 交互查看（MATLAB 命令行，可见窗口）：
%   sc = mu_config('tour','difficulty','medium','nUAV',8,'seed',3);
%   figure('Color','w'); hold on; axis equal; mu_draw_scene(sc, {}, 'tour');

appDir = fileparts(mfilename('fullpath'));
parentDir = fileparts(appDir);
addpath(appDir); addpath(parentDir);

% 默认三场景地图
DEFAULTS = { ...
    struct('tag','p2p_medium','mode','p2p','diff','medium','nUAV',10,'nCtrl',5,'seed',3), ...
    struct('tag','tour_medium','mode','tour','diff','medium','nUAV',8,'seed',3), ...
    struct('tag','tour_hard','mode','tour','diff','hard','nUAV',8,'seed',3) };

if nargin>0 && iscell(varargin{1}) && ~isempty(varargin{1})
    cases = varargin{1};
else
    cases = DEFAULTS;
end

outDir = fullfile(appDir,'results');
if ~exist(outDir,'dir'), mkdir(outDir); end

for ci=1:numel(cases)
    c = cases{ci};
    fprintf('[地图 %d/%d] %s (%s, %d 机)...\n', ci, numel(cases), c.tag, c.diff, c.nUAV);
    if strcmp(c.mode,'tour')
        sc = mu_config('tour','difficulty',c.diff,'nUAV',c.nUAV,'seed',c.seed);
    else
        sc = mu_config('p2p','difficulty',c.diff,'nUAV',c.nUAV,'nCtrl',c.nCtrl,'seed',c.seed);
    end
    fig = figure('Color','w','Visible','off');
    ax = axes('Parent',fig,'Color','w');
    hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
    mu_draw_scene(sc, {}, c.mode, ax);     % trajs={} 空：只画场景地图，不画航迹
    view(ax,[38 26]);
    png = fullfile(outDir, sprintf('map_%s.png', c.tag));
    saveas(fig, png, 'png');
    close(fig);
    % 第二视角：俯视（正上方 90° 看 xy 填充），评估建筑/路网在 xy 平面占满度
    fig2 = figure('Color','w','Visible','off');
    ax2 = axes('Parent',fig2,'Color','w');
    hold(ax2,'on'); grid(ax2,'on'); axis(ax2,'equal');
    mu_draw_scene(sc, {}, c.mode, ax2);
    view(ax2,[0 90]);
    png2 = fullfile(outDir, sprintf('map_%s_top.png', c.tag));
    saveas(fig2, png2, 'png');
    close(fig2);
    fprintf('  已保存: %s (+ 俯视版 %s)\n', png, png2);
end
fprintf('全部地图已导出至: %s\n', outDir);
end
