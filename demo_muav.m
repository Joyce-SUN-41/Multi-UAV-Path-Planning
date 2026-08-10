%% demo_muav — 验证多无人机路径规划 application（三套难度 + 多机数），并导出 EPS 与 XLSX
% 步骤1：用 sphere 类代价验证 CA 接口接入正确
% 步骤2：分别运行 easy(6机)/medium(12机)/hard(20机) 的 p2p 场景
% 步骤3：运行 medium 难度的 tour 场景（多机任务点巡访）
% 步骤4：绘制结果，保存 EPS 矢量图 + XLSX 汇总文档

appDir = fileparts(mfilename('fullpath'));   % 本文件所在目录
parentDir = fileparts(appDir);                 % 父目录（含 CAv9x）
addpath(appDir); addpath(parentDir);
% 结果统一放入 results/ 子目录，文件名带本次运行的时间戳（避免冒号，便于文件命名）
outDir = fullfile(appDir, 'results');
if ~exist(outDir,'dir'), mkdir(outDir); end
runTS = datestr(now, 'yyyy-mm-dd_HHMMSS');

fprintf('=== Step1: 验证 CA 接口 (sphere, dim=10) ===\n');
sphere_f = @(x) sum(x.^2, 1);   % 支持矩阵输入：每列一个解 -> 1xN
[bs, bx, cv] = CAv9x(sphere_f, 10, 30, 60, -5*ones(1,10), 5*ones(1,10), struct('maxFE',9000,'seed',1));
fprintf('  sphere best = %.3e, 收敛末值 = %.3e\n', bs, cv(end));

% 三套难度 + 多机演示配置（城市复杂环境配送）
cases = { ...
    struct('tag','p2p_easy',   'mode','p2p',  'diff','easy',   'nUAV',6,  'nCtrl',5, 'pop',40,'iter',100,'maxFE',60000,'seed',2), ...
    struct('tag','p2p_medium', 'mode','p2p',  'diff','medium', 'nUAV',12, 'nCtrl',5, 'pop',50,'iter',120,'maxFE',100000,'seed',2), ...
    struct('tag','p2p_hard',   'mode','p2p',  'diff','hard',   'nUAV',20, 'nCtrl',5, 'pop',60,'iter',140,'maxFE',150000,'seed',2), ...
    struct('tag','tour_medium','mode','tour', 'diff','medium', 'nUAV',12, 'nCtrl',[],'pop',50,'iter',120,'maxFE',110000,'seed',3), ...
    };

nC = numel(cases);
costAll = zeros(nC,1); penAll = zeros(nC,1); lenAll = zeros(nC,1);
nUAVAll = zeros(nC,1); nObsAll = zeros(nC,1); nTaskAll = zeros(nC,1);
nBldgAll = zeros(nC,1); nTowerAll = zeros(nC,1); nNoFlyAll = zeros(nC,1);
nTreeAll = zeros(nC,1); nWaterAll = zeros(nC,1); terrainAll = zeros(nC,1);
trajCell = cell(nC,1); sceneCell = cell(nC,1); modeCell = cell(nC,1); cvCell = cell(nC,1);

fprintf('\n=== Step2/3: 运行 %d 个场景 ===\n', nC);
for ci=1:nC
    c = cases{ci};
    fprintf('  [%d/%d] %s (%s, %d 机)...\n', ci, nC, c.tag, c.diff, c.nUAV);
    if strcmp(c.mode,'tour')
        [~, cost, cv, trajs, sc] = mu_run_planner('tour', ...
            'pop',c.pop,'iter',c.iter,'maxFE',c.maxFE,'nUAV',c.nUAV,'seed',c.seed);
    else
        [~, cost, cv, trajs, sc] = mu_run_planner('p2p', ...
            'pop',c.pop,'iter',c.iter,'maxFE',c.maxFE,'nUAV',c.nUAV,'nCtrl',c.nCtrl,'seed',c.seed);
    end
    mp=0; ln=0;
    for k=1:numel(trajs)
        [~,op] = mu_obstacle_dist(trajs{k}, sc.obstacles, 0);
        mp = max(mp, -min(op));
        ln = ln + sum(sqrt(sum(diff(trajs{k},1,1).^2,2)));
    end
    obTypes = {sc.obstacles.type};
    costAll(ci)=cost; penAll(ci)=mp; lenAll(ci)=ln;
    nUAVAll(ci)=sc.nUAV; nObsAll(ci)=numel(sc.obstacles);
    nTaskAll(ci)=size(sc.tasks,1);
    nBldgAll(ci)  = sum(strcmp(obTypes,'bldg'));
    nTowerAll(ci) = sum(strcmp(obTypes,'tower'));
    nNoFlyAll(ci) = sum(strcmp(obTypes,'nofly'));
    nTreeAll(ci)  = sum(strcmp(obTypes,'tree'));
    nWaterAll(ci) = sum(strcmp(obTypes,'water'));
    terrainAll(ci)= ~isempty(sc.terrainF);
    trajCell{ci}=trajs; sceneCell{ci}=sc; modeCell{ci}=c.mode; cvCell{ci}=cv;
    fprintf('    cost=%.2f  maxPen=%.3f  len=%.1f  楼=%d 塔=%d 禁飞=%d 树=%d 水=%d 地形=%d\n', ...
        cost, mp, ln, nBldgAll(ci), nTowerAll(ci), nNoFlyAll(ci), nTreeAll(ci), nWaterAll(ci), terrainAll(ci));
end

fprintf('\n=== Step4: 绘图与导出 ===\n');
for ci=1:nC
    c = cases{ci}; sc = sceneCell{ci}; trajs = trajCell{ci};
    % 显式创建 axes 并传给 mu_draw_scene（与 GUI 一致，避免 gca 歧义）；
    % 不在 figure 上锁死 opengl 渲染器——headless/-batch 下 opengl 硬件加速不可用，
    % 会与 print 冲突导致空白图或 "Unable to use OpenGL for printing" 报错。
    fig = figure('Name',c.tag,'Color','w','Visible','off');
    ax = axes('Parent',fig,'Color','w');
    hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
    mu_draw_scene(sc, trajs, modeCell{ci}, ax);
    view(ax, [38 26]);
    title(ax, sprintf('%s  (%s)  cost=%.1f', upper(c.tag), c.diff, costAll(ci)), 'Interpreter','none');
    drawnow;
    pngFile = fullfile(outDir, sprintf('result_%s_%s.png', c.tag, runTS));
    epsFile = fullfile(outDir, sprintf('result_%s_%s.eps', c.tag, runTS));
    % 原子写：先写 .tmp 再改名，进程被杀也不留损坏文件
    mu_savefig(fig, pngFile, 'png', 300);
    mu_savefig(fig, epsFile, 'eps', 300);
    close(fig);
end

% ---- 收敛曲线（前三个 p2p 场景对比难度）----
figC = figure('Name','MUAV convergence','Color','w','Visible','off');
axC = axes('Parent',figC,'Color','w');
hold(axC,'on'); grid(axC,'on');
set(axC,'GridColor',[0.85 0.88 0.92],'XColor',[0.3 0.35 0.42],'YColor',[0.3 0.35 0.42]);
cols = [0.10 0.45 0.75; 0.85 0.45 0.10; 0.20 0.60 0.35];
lgd = {};
for ci=1:3
    cv = cvCell{ci};
    semilogy(axC, cv, 'Color',cols(ci,:), 'LineWidth',1.8);
    lgd{end+1} = cases{ci}.tag;
end
legend(axC, lgd{:}, 'TextColor',[0.2 0.2 0.2], 'Color',[1 1 1], 'EdgeColor',[0.7 0.7 0.7], 'Interpreter','none');
title(axC, 'CA 收敛曲线 (P2P 三难度)','Color',[0.15 0.20 0.30]);
xlabel(axC, '迭代','Color',[0.3 0.35 0.42]); ylabel(axC, '最优代价 (log)','Color',[0.3 0.35 0.42]);
drawnow;
mu_savefig(figC, fullfile(outDir, sprintf('result_convergence_%s.png', runTS)), 'png', 300);
mu_savefig(figC, fullfile(outDir, sprintf('result_convergence_%s.eps', runTS)), 'eps', 300);
close(figC);

% ================= XLSX 文档 =================
tagNames = cell(nC,1);
for ci=1:nC, tagNames{ci} = cases{ci}.tag; end
diffAll = cell(nC,1);
for ci=1:nC, diffAll{ci} = sceneCell{ci}.difficulty; end
summaryT = table( ...
    tagNames, ...
    modeCell(:), ...
    diffAll(:), ...
    nUAVAll, nObsAll, nBldgAll, nTowerAll, nNoFlyAll, nTreeAll, nWaterAll, terrainAll, nTaskAll, ...
    round(costAll,4), round(penAll,4), round(lenAll,3), ...
    'VariableNames', {'Case','Mode','Difficulty','nUAV','nObstacles','nBldg','nTower','nNoFly','nTree','nWater','terrainOn','nTasks','bestCost','maxPenetration','totalLength'});

xlsxFile = fullfile(outDir, sprintf('muav_results_%s.xlsx', runTS));
% 原子写：先写 .tmp 再改名，避免 writetable 中途被杀留下损坏 xlsx
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

fprintf('\n全部步骤完成。\n');
fprintf('已保存 EPS / PNG / XLSX 至: %s\n', outDir);
fprintf('本次运行时间戳: %s\n', runTS);
