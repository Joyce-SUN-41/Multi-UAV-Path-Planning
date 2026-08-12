function view_maps(varargin)
% view_maps — 交互式查看三个难度(easy/medium/hard)的城市三维空地图（无轨迹规划）
% 用法：
%   view_maps          在 MATLAB 桌面打开 3 个可旋转的 3D 图形窗口
%   view_maps('png')   无头导出 map_empty_<diff>.png 到 results/ 并关闭窗口（便于无显示器查看）
%
% 空地图 = 仅渲染城市静态场景（楼/路/立交/地形/禁飞区/通信设施/起点终点标记），
% 不包含无人机轨迹，用于直观检查场景搭配、穿模与立交形态。
appDir = fileparts(mfilename('fullpath'));
parentDir = fileparts(appDir);
addpath(appDir); addpath(parentDir);

exportPng = false;
if nargin>0 && ischar(varargin{1}) && strcmpi(varargin{1},'png')
    exportPng = true;
end

diffs = {'easy','medium','hard'};
outDir = fullfile(appDir,'results');
if exportPng && ~exist(outDir,'dir'), mkdir(outDir); end

for di=1:numel(diffs)
    d = diffs{di};
    % 构造空场景（p2p 模式即可，nUAV 取小值仅用于起点/终点标记占位）
    sc = mu_config('p2p','difficulty',d,'nUAV',6,'seed',2);
    if exportPng
        fig = figure('Color','w','Visible','off');
    else
        fig = figure('Color','w','Name',sprintf('城市空地图 - %s',d),'NumberTitle','off');
    end
    mu_draw_scene(sc, {}, 'p2p');   % trajs={} 不画轨迹，仅场景
    title(sprintf('城市空地图 · %s（三维态势，无轨迹）', d'),'Color',[0.15 0.20 0.30],'FontSize',13,'FontWeight','bold');
    if exportPng
        png = fullfile(outDir, sprintf('map_empty_%s.png', d));
        saveas(fig, png, 'png');
        close(fig);
        fprintf('  导出 %s\n', png);
    end
end
if exportPng
    fprintf('完成，图片在: %s\n', outDir);
else
    fprintf('已打开 %d 个可旋转三维窗口：easy / medium / hard。\n', numel(diffs));
end
end
