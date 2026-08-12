function run_muav(varargin)
% run_muav ?? / ??%
%  1p2p ??+ tour??PNG/EPS/XLSX
%   run_muav
%
%  2 Name/Value 
%   run_muav('mode','tour','difficulty','hard','nUAV',8,'pop',60,'iter',140,'maxFE',150000,'seed',3)
%   run_muav('mode','p2p','difficulty','medium','nUAV',12,'nCtrl',5,'pop',50,'iter',120)
%
%  3 cell ??case  struct DEFAULT_CASES??%   cases = { struct('tag','my1','mode','p2p','diff','easy','nUAV',6,'nCtrl',5,'pop',40,'iter',100,'maxFE',60000,'seed',2) };
%   run_muav('cases', cases)
%
% 
%   'outDir'   ??Multi-UAV-Path-Planning/results??%   'comms'    true/false??false scene.w.comms=30??%   'wObstacle'/'wSmooth'/'wSeparation'/'wLength' 
%   'caFun'     CA ??@CAv9x??dimpop ??(L1)
%
%  mu_run_planner / mu_config / mu_draw_scene / mu_obstacle_dist /
%       mu_savefig??CAv9x??addpath??
% ----------  ----------
appDir = fileparts(mfilename('fullpath'));
parentDir = fileparts(appDir);
addpath(appDir); addpath(parentDir);

% ----------  ----------
p = inputParser;
addParameter(p,'cases',{}, @iscell);
addParameter(p,'outDir', fullfile(appDir,'results'));
addParameter(p,'comms', false, @islogical);
addParameter(p,'wObstacle', 60);
addParameter(p,'wSmooth', 0.15);
addParameter(p,'wSeparation', 25);
addParameter(p,'wLength', 1.0);
addParameter(p,'caFun', @CAv9x);
% ????
addParameter(p,'mode', '');
addParameter(p,'difficulty', '');
addParameter(p,'nUAV', []);
addParameter(p,'nCtrl', []);
addParameter(p,'pop', []);
addParameter(p,'iter', []);
addParameter(p,'maxFE', []);
addParameter(p,'seed', []);
parse(p, varargin{:});
opt = p.Results;

%  demo_muav ??DEFAULT_CASES = { ...
DEFAULT_CASES = { ...
    struct('tag','p2p_easy',   'mode','p2p',  'diff','easy',   'nUAV',6,  'nCtrl',5, 'pop',40, 'iter',100, 'maxFE',60000,  'seed',2), ...
    struct('tag','p2p_medium', 'mode','p2p',  'diff','medium', 'nUAV',12, 'nCtrl',5, 'pop',50, 'iter',120, 'maxFE',100000, 'seed',2), ...
    struct('tag','p2p_hard',   'mode','p2p',  'diff','hard',   'nUAV',20, 'nCtrl',5, 'pop',60, 'iter',140, 'maxFE',150000, 'seed',2), ...
    struct('tag','tour_medium','mode','tour', 'diff','medium', 'nUAV',12, 'nCtrl',[],'pop',50, 'iter',120, 'maxFE',110000, 'seed',3), ...
    struct('tag','tour_hard',  'mode','tour', 'diff','hard',   'nUAV',8,  'nCtrl',[],'pop',60, 'iter',140, 'maxFE',150000, 'seed',3), ...
    };

%  mode/difficulty ??case 
singleGiven = ~isempty(opt.mode) || ~isempty(opt.difficulty) || ...
              ~isempty(opt.nUAV) || ~isempty(opt.pop) || ~isempty(opt.maxFE);
if singleGiven
    sc = struct();
    sc.mode  = validOpt(opt.mode,  'p2p',  {'p2p','tour'});
    sc.diff  = validOpt(opt.difficulty, 'medium', {'easy','medium','hard'});
    sc.nUAV  = validOpt(opt.nUAV,  3,   [3 30]);
    sc.nCtrl = validOpt(opt.nCtrl, 5,   [2 12]);
    sc.pop   = validOpt(opt.pop,   40,  [10 200]);
    sc.iter  = validOpt(opt.iter,  120, [10 500]);
    sc.maxFE = validOpt(opt.maxFE,60000,[5000 300000]);
    sc.seed  = validOpt(opt.seed,  2,   [0 1e6]);
    sc.tag   = sprintf('%s_%s_n%d', sc.mode, sc.diff, sc.nUAV);
    cases = { sc };
elseif isempty(opt.cases)
    cases = DEFAULT_CASES;
else
    cases = opt.cases;
end

% 
outDir = opt.outDir;
if ~exist(outDir,'dir'), mkdir(outDir); end
runTS = datestr(now, 'yyyy-mm-dd_HHMMSS');

nC = numel(cases);
costAll=zeros(nC,1); penAll=zeros(nC,1); lenAll=zeros(nC,1);
nUAVAll=zeros(nC,1); nObsAll=zeros(nC,1); nTaskAll=zeros(nC,1);
nBldgAll=zeros(nC,1); nTowerAll=zeros(nC,1); nNoFlyAll=zeros(nC,1);
nTreeAll=zeros(nC,1); nWaterAll=zeros(nC,1); terrainAll=zeros(nC,1); terrPenAll=zeros(nC,1);
trajCell=cell(nC,1); sceneCell=cell(nC,1); modeCell=cell(nC,1); cvCell=cell(nC,1);

fprintf('============   run_muav ============\n');
fprintf('??%d ??| =%d | CA=%s\n', nC, opt.comms, func2str(opt.caFun));

for ci=1:nC
    c = cases{ci};
    fprintf('\n[%d/%d] %s (%s, %d ??...\n', ci, nC, c.tag, c.diff, c.nUAV);
    try
        if strcmp(c.mode,'tour')
            % R6tour  nCtrl 2*(maxT+1)??            % ??mu_config ??warning ??
            if ~isempty(opt.nCtrl)
                warning('run_muav: tour nCtrl ignored, got %g', opt.nCtrl);
            end
            [bx, cost, cv, trajs, sc] = mu_run_planner('tour', ...
                'pop',c.pop,'iter',c.iter,'maxFE',c.maxFE,'nUAV',c.nUAV,'seed',c.seed, ...
                'caFun',opt.caFun);
        else
            [bx, cost, cv, trajs, sc] = mu_run_planner('p2p', ...
                'pop',c.pop,'iter',c.iter,'maxFE',c.maxFE,'nUAV',c.nUAV,'nCtrl',c.nCtrl,'seed',c.seed, ...
                'caFun',opt.caFun);
        end
    catch ME
        fprintf('  FAIL: %s\n', ME.message);
        continue;
    end

    % ??    sc.w.obstacle   = opt.wObstacle;
    sc.w.smooth     = opt.wSmooth;
    sc.w.separation = opt.wSeparation;
    sc.w.length     = opt.wLength;
    sc.w.comms      = opt.comms * 30;   % ??30 0

    % ??mu_cost_tour ??    % R19 ??mu_obstacle_dist(trajs{k}, sc.obstacles, 0) ??terrain, ??
    %  margin=0  mp??mp 
    %   ??terrain safeMarginterrainMargin
    %    trajs{1}??    mp=0; terrPen=0; ln=0;
    mp=0; terrPen=0; ln=0;
    obsTypes = {sc.obstacles.type};
    obsEnt   = sc.obstacles(~strcmp(obsTypes,'terrain'));    % /??/????????
    obsTerr  = sc.obstacles(strcmp(obsTypes,'terrain'));     % 
    for k=1:numel(trajs)
        [~,o] = mu_obstacle_dist(trajs{k}, obsEnt, sc.safeMargin);
        mp = max(mp, max(0, -min(o)));   % ??0
        if ~isempty(obsTerr)
            [~,ot] = mu_obstacle_dist(trajs{k}, obsTerr, sc.terrainMargin);
            terrPen = max(terrPen, max(0, -min(ot)));
        end
        ln = ln + sum(sqrt(sum(diff(trajs{k},1,1).^2,2)));
    end
    % D
    vehMax = 0;
    if isfield(sc,'dynamics') && ~isempty(sc.dynamics) && isfield(sc.dynamics,'vehicles')
        for k=1:numel(trajs)
            tk = mu_arc_time(trajs{k}, sc.T_horizon);
            [~,vp] = mu_obstacle_dist_t(trajs{k}, sc, tk, sc.vehMargin);
            vehMax = max(vehMax, max(vp));
        end
    end

    obTypes = {sc.obstacles.type};
    costAll(ci)=cost; penAll(ci)=mp; terrAll(ci)=terrPen; lenAll(ci)=ln;
    nUAVAll(ci)=sc.nUAV; nObsAll(ci)=numel(sc.obstacles);
    nTaskAll(ci)=size(sc.tasks,1);
    nBldgAll(ci)  = sum(strcmp(obTypes,'bldg'));
    nTowerAll(ci) = sum(strcmp(obTypes,'tower'));
    nNoFlyAll(ci) = sum(strcmp(obTypes,'nofly'));
    nTreeAll(ci)  = sum(strcmp(obTypes,'tree'));
    nWaterAll(ci) = sum(strcmp(obTypes,'water'));
    terrainAll(ci)= ~isempty(sc.terrainF);
    terrPenAll(ci)= terrPen;
    trajCell{ci}=trajs; sceneCell{ci}=sc; modeCell{ci}=c.mode; cvCell{ci}=cv;
    fprintf('  cost=%.2f  maxStaticPen=%.3f  terrainPen=%.3f  maxVehPen=%.3f  len=%.1f  ??%d ??%d =%d ??%d ??%d =%d\n', ...
        cost, mp, terrPen, vehMax, ln, nBldgAll(ci), nTowerAll(ci), nNoFlyAll(ci), nTreeAll(ci), nWaterAll(ci), terrainAll(ci));
end

% ---------- ??----------
fprintf('\n=== ??===\n');
for ci=1:nC
    if isempty(trajCell{ci}), continue; end
    c = cases{ci}; sc = sceneCell{ci}; trajs = trajCell{ci};
    fig = figure('Name',c.tag,'Color','w','Visible','off');
    ax = axes('Parent',fig,'Color','w');
    hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
    % D tCur=0 ??GUIMUAVPlanner??    sc.tCur = 0;
    mu_draw_scene(sc, trajs, modeCell{ci}, ax);
    view(ax, [38 26]);
    title(ax, sprintf('%s  (%s)  cost=%.1f', upper(c.tag), c.diff, costAll(ci)), 'Interpreter','none');
    drawnow;
    pngFile = fullfile(outDir, sprintf('run_%s_%s.png', c.tag, runTS));
    epsFile = fullfile(outDir, sprintf('run_%s_%s.eps', c.tag, runTS));
    mu_savefig(fig, pngFile, 'png', 300);
    mu_savefig(fig, epsFile, 'eps', 300);
    close(fig);
end

%  p2p ??figC = figure('Name','MUAV convergence','Color','w','Visible','off');
figC = figure('Name','MUAV convergence','Color','w','Visible','off');
axC = axes('Parent',figC,'Color','w');
hold(axC,'on'); grid(axC,'on');
set(axC,'GridColor',[0.85 0.88 0.92],'XColor',[0.3 0.35 0.42],'YColor',[0.3 0.35 0.42]);
cols = [0.10 0.45 0.75; 0.85 0.45 0.10; 0.20 0.60 0.35];
lgd = {};
cnt=0;
for ci=1:nC
    if strcmp(modeCell{ci},'p2p') && cnt<3
        cnt=cnt+1;
        semilogy(axC, cvCell{ci}, 'Color',cols(cnt,:), 'LineWidth',1.8);
        lgd{end+1} = cases{ci}.tag;
    end
end
if ~isempty(lgd)
    legend(axC, lgd{:}, 'TextColor',[0.2 0.2 0.2], 'Color',[1 1 1], 'EdgeColor',[0.7 0.7 0.7], 'Interpreter','none');
end
title(axC, 'CA  (P2P )','Color',[0.15 0.20 0.30]);
xlabel(axC, '','Color',[0.3 0.35 0.42]); ylabel(axC, '??(log)','Color',[0.3 0.35 0.42]);
drawnow;
mu_savefig(figC, fullfile(outDir, sprintf('run_convergence_%s.png', runTS)), 'png', 300);
mu_savefig(figC, fullfile(outDir, sprintf('run_convergence_%s.eps', runTS)), 'eps', 300);
close(figC);

% ---------- XLSX ??----------
tagNames = cell(nC,1); diffAll = cell(nC,1);
for ci=1:nC
    if isempty(trajCell{ci}), tagNames{ci}=cases{ci}.tag; diffAll{ci}='-'; continue; end
    tagNames{ci} = cases{ci}.tag; diffAll{ci} = sceneCell{ci}.difficulty;
end
summaryT = table( ...
    tagNames, modeCell(:), diffAll(:), ...
    nUAVAll, nObsAll, nBldgAll, nTowerAll, nNoFlyAll, nTreeAll, nWaterAll, terrainAll, nTaskAll, ...
    round(costAll,4), round(penAll,4), round(terrPenAll,4), round(lenAll,3), ...
    'VariableNames', {'Case','Mode','Difficulty','nUAV','nObstacles','nBldg','nTower','nNoFly','nTree','nWater','terrainOn','nTasks','bestCost','maxStaticPen','maxTerrainPen','totalLength'});
xlsxFile = fullfile(outDir, sprintf('run_muav_results_%s.xlsx', runTS));
[dr, nm, ext] = fileparts(xlsxFile);
tmpX = fullfile(dr, [nm '.tmp' ext]);
if exist(tmpX,'file'), delete(tmpX); end
try
    writetable(summaryT, tmpX, 'Sheet', 'Summary');
    movefile(tmpX, xlsxFile, 'f');
catch ME
    if exist(tmpX,'file'), delete(tmpX); end
    rethrow(ME);
end

fprintf('\n\n');
fprintf('??PNG / EPS / XLSX ?? %s\n', outDir);
fprintf('?? %s\n', runTS);

% ???? def??function v = validOpt(val, def, allowed)
function v = validOpt(val, def, allowed)
if isempty(val)
    v = def; return;
end
if iscell(allowed)   % 
    if ~any(strcmpi(val, allowed))
        error('run_muav: ??"%s"??{%s}', val, strjoin(allowed,', '));
    end
    v = val;
else
    lo = allowed(1); hi = allowed(2);                % ??    lo/hi 范围下界/上界
    if val < lo || val > hi
        error('run_muav: ??%g  [%g, %g]', val, lo, hi);
    end
    v = val;
end
end

end   % 主函数 run_muav 结尾（嵌套函数 validOpt 已在其内定义）
