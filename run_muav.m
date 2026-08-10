function run_muav(varargin)
% run_muav — 城市多无人机路径规划统一主调入口（命令行 / 无头批量友好）
%
% 用法 1（默认演示）：运行一组预设场景（p2p 三难度 + tour），并导出 PNG/EPS/XLSX
%   run_muav
%
% 用法 2（单场景自定义）：通过 Name/Value 指定单个场景参数，仅规划该场景并绘图
%   run_muav('mode','tour','difficulty','hard','nUAV',8,'pop',60,'iter',140,'maxFE',150000,'seed',3)
%   run_muav('mode','p2p','difficulty','medium','nUAV',12,'nCtrl',5,'pop',50,'iter',120)
%
% 用法 3（批量自定义）：传入 cell 数组的 case 列表（每个为 struct，字段见下方 DEFAULT_CASES）
%   cases = { struct('tag','my1','mode','p2p','diff','easy','nUAV',6,'nCtrl',5,'pop',40,'iter',100,'maxFE',60000,'seed',2) };
%   run_muav('cases', cases)
%
% 通用可选参数：
%   'outDir'   结果输出目录（默认 Multi-UAV-Path-Planning/results）
%   'comms'    是否启用通信链路约束（true/false，默认 false；开启则 scene.w.comms=30）
%   'wObstacle'/'wSmooth'/'wSeparation'/'wLength' 覆盖默认代价权重
%   'caFun'    替换 CA 算法句柄（默认 @CAv9x），须满足"dim×pop 列优先"接口约定(L1)
%
% 依赖：本目录须含 mu_run_planner / mu_config / mu_draw_scene / mu_obstacle_dist /
%       mu_savefig，且父目录须含 CAv9x（自动 addpath）。

% ---------- 路径 ----------
appDir = fileparts(mfilename('fullpath'));
parentDir = fileparts(appDir);
addpath(appDir); addpath(parentDir);

% ---------- 解析参数 ----------
p = inputParser;
addParameter(p,'cases',{}, @iscell);
addParameter(p,'outDir', fullfile(appDir,'results'));
addParameter(p,'comms', false, @islogical);
addParameter(p,'wObstacle', 60);
addParameter(p,'wSmooth', 0.15);
addParameter(p,'wSeparation', 25);
addParameter(p,'wLength', 1.0);
addParameter(p,'caFun', @CAv9x);
% 单场景自定义快捷参数：提供任一即视为"只跑这一个场景"
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

% 默认演示场景（与 demo_muav 一致的五套配置）
DEFAULT_CASES = { ...
    struct('tag','p2p_easy',   'mode','p2p',  'diff','easy',   'nUAV',6,  'nCtrl',5, 'pop',40, 'iter',100, 'maxFE',60000,  'seed',2), ...
    struct('tag','p2p_medium', 'mode','p2p',  'diff','medium', 'nUAV',12, 'nCtrl',5, 'pop',50, 'iter',120, 'maxFE',100000, 'seed',2), ...
    struct('tag','p2p_hard',   'mode','p2p',  'diff','hard',   'nUAV',20, 'nCtrl',5, 'pop',60, 'iter',140, 'maxFE',150000, 'seed',2), ...
    struct('tag','tour_medium','mode','tour', 'diff','medium', 'nUAV',12, 'nCtrl',[],'pop',50, 'iter',120, 'maxFE',110000, 'seed',3), ...
    struct('tag','tour_hard',  'mode','tour', 'diff','hard',   'nUAV',8,  'nCtrl',[],'pop',60, 'iter',140, 'maxFE',150000, 'seed',3), ...
    };

% 单场景自定义：若用户显式给了 mode/difficulty 等，则构造单个 case 覆盖默认
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

% 输出目录
outDir = opt.outDir;
if ~exist(outDir,'dir'), mkdir(outDir); end
runTS = datestr(now, 'yyyy-mm-dd_HHMMSS');

nC = numel(cases);
costAll=zeros(nC,1); penAll=zeros(nC,1); lenAll=zeros(nC,1);
nUAVAll=zeros(nC,1); nObsAll=zeros(nC,1); nTaskAll=zeros(nC,1);
nBldgAll=zeros(nC,1); nTowerAll=zeros(nC,1); nNoFlyAll=zeros(nC,1);
nTreeAll=zeros(nC,1); nWaterAll=zeros(nC,1); terrainAll=zeros(nC,1); terrPenAll=zeros(nC,1);
trajCell=cell(nC,1); sceneCell=cell(nC,1); modeCell=cell(nC,1); cvCell=cell(nC,1);

fprintf('============ 城市多无人机路径规划 主调 run_muav ============\n');
fprintf('共 %d 个场景 | 通信约束=%d | CA=%s\n', nC, opt.comms, func2str(opt.caFun));

for ci=1:nC
    c = cases{ci};
    fprintf('\n[%d/%d] %s (%s, %d 机)...\n', ci, nC, c.tag, c.diff, c.nUAV);
    try
        if strcmp(c.mode,'tour')
            % R6：tour 模式 nCtrl 由客户点数自动决定（2*(maxT+1)），用户显式传入无效，
            % 给出提示避免误以为生效（与 mu_config 的 warning 一致）。
            if ~isempty(opt.nCtrl)
                warning('run_muav: tour 模式控制点数由客户点数自动决定，忽略 nCtrl=%g。', opt.nCtrl);
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

    % 覆盖代价权重（若用户指定）
    sc.w.obstacle   = opt.wObstacle;
    sc.w.smooth     = opt.wSmooth;
    sc.w.separation = opt.wSeparation;
    sc.w.length     = opt.wLength;
    sc.w.comms      = opt.comms * 30;   % 启用则 30，关闭则 0

    % 诊断：最大障碍穿透（与 mu_cost_tour 口径一致，避免误判）
    % R19 修复：此前 mu_obstacle_dist(trajs{k}, sc.obstacles, 0) 把地形(terrain, 软约束)
    % 也以 margin=0 计入 mp，导致 mp 虚高且无法反映真实实体障碍穿透。现拆分为：
    %   实体障碍（不含 terrain）用 safeMargin；地形贴合单独报告（terrainMargin）；
    %   动态车辆穿透对所有机做回放诊断（此前仅测 trajs{1}）。
    mp=0; terrPen=0; ln=0;
    obsTypes = {sc.obstacles.type};
    obsEnt   = sc.obstacles(~strcmp(obsTypes,'terrain'));    % 实体障碍（楼/塔/禁飞/树/水/桥/灯/牌）
    obsTerr  = sc.obstacles(strcmp(obsTypes,'terrain'));     % 地形（软约束，单独口径）
    for k=1:numel(trajs)
        [~,o] = mu_obstacle_dist(trajs{k}, obsEnt, sc.safeMargin);
        mp = max(mp, max(0, -min(o)));   % 实体穿透深度（安全时=0，避免负值误报）
        if ~isempty(obsTerr)
            [~,ot] = mu_obstacle_dist(trajs{k}, obsTerr, sc.terrainMargin);
            terrPen = max(terrPen, max(0, -min(ot)));
        end
        ln = ln + sum(sqrt(sum(diff(trajs{k},1,1).^2,2)));
    end
    % 阶段D：动态车辆穿透（接入代价后应有避让；对所有机做独立回放诊断）
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
    fprintf('  cost=%.2f  maxStaticPen=%.3f  terrainPen=%.3f  maxVehPen=%.3f  len=%.1f  楼=%d 塔=%d 禁飞=%d 树=%d 水=%d 地形=%d\n', ...
        cost, mp, terrPen, vehMax, ln, nBldgAll(ci), nTowerAll(ci), nNoFlyAll(ci), nTreeAll(ci), nWaterAll(ci), terrainAll(ci));
end

% ---------- 绘图与导出 ----------
fprintf('\n=== 绘图与导出 ===\n');
for ci=1:nC
    if isempty(trajCell{ci}), continue; end
    c = cases{ci}; sc = sceneCell{ci}; trajs = trajCell{ci};
    fig = figure('Name',c.tag,'Color','w','Visible','off');
    ax = axes('Parent',fig,'Color','w');
    hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
    % 阶段D：传 tCur=0 渲染静态初始车流（如需动态回放请用 GUI：MUAVPlanner）
    sc.tCur = 0;
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

% 收敛曲线（前三个 p2p 场景对比）
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
title(axC, 'CA 收敛曲线 (P2P 场景)','Color',[0.15 0.20 0.30]);
xlabel(axC, '迭代','Color',[0.3 0.35 0.42]); ylabel(axC, '最优代价 (log)','Color',[0.3 0.35 0.42]);
drawnow;
mu_savefig(figC, fullfile(outDir, sprintf('run_convergence_%s.png', runTS)), 'png', 300);
mu_savefig(figC, fullfile(outDir, sprintf('run_convergence_%s.eps', runTS)), 'eps', 300);
close(figC);

% ---------- XLSX 汇总 ----------
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

fprintf('\n全部场景完成。\n');
fprintf('已保存 PNG / EPS / XLSX 至: %s\n', outDir);
fprintf('本次运行时间戳: %s\n', runTS);
end

% 取值助手：用户未指定(空)时用默认 def；枚举类做合法性校验
function v = validOpt(val, def, allowed)
if isempty(val)
    v = def; return;
end
if iscell(allowed)   % 枚举校验
    if ~any(strcmpi(val, allowed))
        error('run_muav: 非法取值 "%s"，应为 {%s}', val, strjoin(allowed,', '));
    end
    v = val;
else                % 数值范围校验
    lo = allowed(1); hi = allowed(2);
    if val < lo || val > hi
        error('run_muav: 取值 %g 超出范围 [%g, %g]', val, lo, hi);
    end
    v = val;
end
end
