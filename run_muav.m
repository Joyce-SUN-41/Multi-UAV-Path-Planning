function run_muav(varargin)
% run_muav ?? / ??%
%  1p2p ??+ tour??PNG/EPS/XLSX
%   run_muav
%
%  2 Name/Value (示例；当前默认档位见下方 DEFAULT_CASES，方案B：nCtrl=12,pop/iter 大幅加,maxFE 顶满)
%   run_muav('mode','tour','difficulty','hard','nUAV',8,'pop',180,'iter',450,'maxFE',300000,'seed',3)
%   run_muav('mode','p2p','difficulty','medium','nUAV',12,'nCtrl',12,'pop',160,'iter',400,'maxFE',280000,'seed',2)
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
% 高维精细化档位：nCtrl 提升曲线节点自由度，pop/iter 提高搜索充分度，
% maxFE 按 pop*iter*2*1.5 配平（CAv9x 双生观测每次占 2 次评估，留余量防提前截断），
% seed 固定以便复现对照。tour 的 nCtrl 由任务数自动决定（mu_config），此处忽略。
% 更精细档位（方案B：激进档，逼近 validOpt 上限）：nCtrl=12, pop/iter 大幅加，maxFE 顶满。
DEFAULT_CASES = { ...
    struct('tag','p2p_easy',   'mode','p2p',  'diff','easy',   'nUAV',6,  'nCtrl',12,'pop',120, 'iter',300, 'maxFE',160000, 'seed',2), ...
    struct('tag','p2p_medium', 'mode','p2p',  'diff','medium', 'nUAV',12, 'nCtrl',12,'pop',160, 'iter',400, 'maxFE',280000, 'seed',2), ...
    struct('tag','p2p_hard',   'mode','p2p',  'diff','hard',   'nUAV',20, 'nCtrl',12,'pop',180, 'iter',450, 'maxFE',300000, 'seed',2), ...
    struct('tag','tour_medium','mode','tour', 'diff','medium', 'nUAV',12, 'nCtrl',[],'pop',160, 'iter',400, 'maxFE',280000, 'seed',3), ...
    struct('tag','tour_hard',  'mode','tour', 'diff','hard',   'nUAV',8,  'nCtrl',[],'pop',180, 'iter',450, 'maxFE',300000, 'seed',3), ...
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
runTS = char(datetime('now','Format','yyyy-MM-dd_HHmmSS'));
% 每组运行产物归档到带时间戳的二级目录，便于管理
runDir = fullfile(outDir, ['run_' runTS]);
if ~exist(runDir,'dir'), mkdir(runDir); end

nC = numel(cases);
costAll=zeros(nC,1); penAll=zeros(nC,1); lenAll=zeros(nC,1);
nUAVAll=zeros(nC,1); nObsAll=zeros(nC,1); nTaskAll=zeros(nC,1);
nBldgAll=zeros(nC,1); nTowerAll=zeros(nC,1); nNoFlyAll=zeros(nC,1);
nTreeAll=zeros(nC,1); nWaterAll=zeros(nC,1); terrainAll=zeros(nC,1); terrPenAll=zeros(nC,1);
trajCell=cell(nC,1); sceneCell=cell(nC,1); modeCell=cell(nC,1); cvCell=cell(nC,1); bxCell=cell(nC,1);

fprintf('============   run_muav ============\n');
fprintf('%d cases | comms=%d | CA=%s\n', nC, opt.comms, func2str(opt.caFun));

for ci=1:nC
    c = cases{ci};
    fprintf('\n[%d/%d] %s (%s, %d UAV)...\n', ci, nC, c.tag, c.diff, c.nUAV);
    try
        if strcmp(c.mode,'tour')
            % R6tour  nCtrl 2*(maxT+1)??            % ??mu_config ??warning ??
            if ~isempty(opt.nCtrl)
                warning('run_muav: tour nCtrl ignored, got %g', opt.nCtrl);
            end
            [bx, cost, cv, trajs, sc] = mu_run_planner('tour', ...
                'pop',c.pop,'iter',c.iter,'maxFE',c.maxFE,'nUAV',c.nUAV,'seed',c.seed, ...
                'difficulty',c.diff, 'caFun',opt.caFun);
        else
            [bx, cost, cv, trajs, sc] = mu_run_planner('p2p', ...
                'pop',c.pop,'iter',c.iter,'maxFE',c.maxFE,'nUAV',c.nUAV,'nCtrl',c.nCtrl,'seed',c.seed, ...
                'difficulty',c.diff, 'caFun',opt.caFun);
        end
    catch ME
        fprintf('  FAIL: %s\n', ME.message);
        continue;
    end
    bxCell{ci} = bx;  % 绑定当前 case 的 CA 最优决策变量，避免循环结束后 bx 错配

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
    fprintf('  cost=%.2f  maxStaticPen=%.3f  terrainPen=%.3f  maxVehPen=%.3f  len=%.1f  bldg=%d tower=%d nofly=%d tree=%d water=%d terrain=%d\n', ...
        cost, mp, terrPen, vehMax, ln, nBldgAll(ci), nTowerAll(ci), nNoFlyAll(ci), nTreeAll(ci), nWaterAll(ci), terrainAll(ci));
end

% ---------- 多视角图导出（透视/俯视/侧视/近景，看清路径）----------
fprintf('\n=== multi-view figures ===\n');
for ci=1:nC
    if isempty(trajCell{ci}), continue; end
    c = cases{ci}; sc = sceneCell{ci}; trajs = trajCell{ci};
    % cost 格式化：大数量级用科学计数法，避免标题被长数字撑满
    if abs(costAll(ci)) >= 1e6
        costStr = sprintf('%.2e', costAll(ci));
    else
        costStr = sprintf('%.1f', costAll(ci));
    end
    try
        mu_export_views(sc, trajs, modeCell{ci}, runDir, c.tag, runTS, costStr);
        fprintf('  %s views exported (persp/top/sideX/sideY/closeup)\n', c.tag);
    catch ME
        % 单个用例的视图渲染偶发失败不应连累整批：跳过其图，保存循环仍会生成 .mat
        fprintf('  WARN: %s view export failed (%s); skipping figures, results still saved.\n', c.tag, ME.message);
    end
end

% ---------- 保存完整结果为 .mat（精致存储：可复现 + 自描述）----------
for ci=1:nC
    if isempty(trajCell{ci}), continue; end
    c = cases{ci}; sc = sceneCell{ci}; trajs = trajCell{ci};
    % 组装单用例结果结构体；bestX 仅在 mu_run_planner 直接调用时可得，
    % run_muav 主循环未保留 bestX，这里用空占位，trajs 作为快查副本已足够回放。
    res = struct();
    res.tag        = c.tag;
    res.mode       = c.mode;
    res.difficulty = sc.difficulty;
    res.scene      = sc;
    res.trajs      = trajs;
    res.bestX      = bxCell{ci};   % mu_run_planner 第1输出即 CA 最优决策变量，可重新解码轨迹（按 case 绑定，防错配）
    res.bestCost   = costAll(ci);
    res.curve      = cvCell{ci};
    res.opt        = struct('pop',c.pop,'iter',c.iter,'maxFE',c.maxFE, ...
                        'nUAV',c.nUAV,'nCtrl',c.nCtrl,'seed',c.seed, ...
                        'wObstacle',opt.wObstacle,'wSmooth',opt.wSmooth, ...
                        'wSeparation',opt.wSeparation,'wLength',opt.wLength, ...
                        'comms',opt.comms);
    res.metrics    = struct('maxStaticPen',penAll(ci),'maxTerrainPen',terrPenAll(ci), ...
                        'totalLength',lenAll(ci),'maxVehPen',vehMax);
    mu_save_result(res, runDir, 'tag', c.tag);
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
        lgd{end+1} = sprintf('scenario%d', cnt);
    end
end
if ~isempty(lgd)
    lgdH = legend(axC, lgd{:}, 'TextColor',[0.2 0.2 0.2], 'Color',[1 1 1], 'EdgeColor',[0.7 0.7 0.7], 'Interpreter','none');
    lgdH.FontSize = 13; lgdH.FontWeight = 'bold';
end
title(axC, 'CA  (P2P )','Color',[0.15 0.20 0.30]);
xlabel(axC, '','Color',[0.3 0.35 0.42]); ylabel(axC, '??(log)','Color',[0.3 0.35 0.42]);
drawnow;
mu_savefig(figC, fullfile(runDir, sprintf('run_convergence_%s.png', runTS)), 'png', 300);
mu_savefig(figC, fullfile(runDir, sprintf('run_convergence_%s.eps', runTS)), 'eps', 300);
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
xlsxFile = fullfile(runDir, sprintf('run_muav_results_%s.xlsx', runTS));
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
fprintf('PNG / EPS / XLSX written to %s\n', runDir);
fprintf('run timestamp: %s\n', runTS);

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
