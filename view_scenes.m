function view_scenes(varargin)
% view_scenes ??""??%    results/ ??%     p2p_medium / tour_medium / tour_hard
%   ??+ ??/??/??/
%   ??%
%  cell ??%   view_scenes({struct('tag','my','mode','tour','diff','hard','nUAV',12,'seed',7)})
%
% MATLAB 
%   sc = mu_config('tour','difficulty','medium','nUAV',8,'seed',3);
%   figure('Color','w'); hold on; axis equal; mu_draw_scene(sc, {}, 'tour');

appDir = fileparts(mfilename('fullpath'));
parentDir = fileparts(appDir);
addpath(appDir); addpath(parentDir);

% ??DEFAULTS = { ...
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
    fprintf('[ %d/%d] %s (%s, %d ??...\n', ci, numel(cases), c.tag, c.diff, c.nUAV);
    if strcmp(c.mode,'tour')
        sc = mu_config('tour','difficulty',c.diff,'nUAV',c.nUAV,'seed',c.seed);
    else
        sc = mu_config('p2p','difficulty',c.diff,'nUAV',c.nUAV,'nCtrl',c.nCtrl,'seed',c.seed);
    end
    fig = figure('Color','w','Visible','off');
    ax = axes('Parent',fig,'Color','w');
    hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
    mu_draw_scene(sc, {}, c.mode, ax);     % trajs={} ??    view(ax,[38 26]);
    png = fullfile(outDir, sprintf('map_%s.png', c.tag));
    saveas(fig, png, 'png');
    close(fig);
    % ??90 ??xy /??xy ??    fig2 = figure('Color','w','Visible','off');
    ax2 = axes('Parent',fig2,'Color','w');
    hold(ax2,'on'); grid(ax2,'on'); axis(ax2,'equal');
    mu_draw_scene(sc, {}, c.mode, ax2);
    view(ax2,[0 90]);
    png2 = fullfile(outDir, sprintf('map_%s_top.png', c.tag));
    saveas(fig2, png2, 'png');
    close(fig2);
    fprintf('  ?? %s (+ ??%s)\n', png, png2);
end
fprintf(': %s\n', outDir);
end
