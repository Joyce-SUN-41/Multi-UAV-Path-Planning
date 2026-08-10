function MUAVPlanner
% MUAVPlanner — 多无人机三维路径规划 Application（接入 CA 算法）
% 运行：在 MATLAB 命令行输入 MUAVPlanner
% 纯 MATLAB (uifigure)，无需 App Designer 即可运行；亦可被 App Designer 工程引用。
%
% 功能：
%   - 场景可切换：P2P（静态障碍避障） / TOUR（多机任务点巡访）
%   - 三维 B 样条/Bezier 轨迹，CA 算法以接口调用形式优化控制点
%   - 参数交互、3D 可视化、收敛曲线、轨迹动画播放

% ---------- 路径 ----------
appRoot = fileparts(mfilename('fullpath'));
addpath(appRoot);
addpath(fileparts(appRoot));   % 父目录含 CAv9x

% ---------- 颜色主题（精致深色） ----------
C_BG      = [0.07 0.09 0.13];
C_PANEL   = [0.10 0.13 0.18];
C_ACCENT  = [0.20 0.80 0.75];
C_TEXT    = [0.85 0.90 0.95];
C_TEXT2   = [0.55 0.62 0.70];
UAV_COLORS = [0.30 0.75 0.90; 0.95 0.60 0.35; 0.70 0.85 0.40; ...
              0.90 0.45 0.70; 0.55 0.85 0.75; 0.95 0.80 0.40];

% ---------- 主窗口 ----------
fig = uifigure('Name','多无人机三维路径规划 — Chronos/CA 驱动', ...
               'Color',C_BG, 'Position',[60 60 1280 760], ...
               'Resize','on', 'AutoResizeChildren','off');

% 标题条
uititle = uilabel(fig, 'Text','多无人机三维路径规划  ·  CA (Chronos) 接口驱动', ...
    'Position',[20 720 700 30], 'FontSize',16, 'FontWeight','bold', ...
    'FontColor',C_ACCENT, 'BackgroundColor',C_BG);

% ---------- 左侧控制面板 ----------
pL = uipanel(fig, 'Title','控制面板', 'Position',[15 15 320 690], ...
    'BackgroundColor',C_PANEL, 'ForegroundColor',C_TEXT, ...
    'BorderColor',C_ACCENT, 'FontSize',11, 'FontWeight','bold');

y = 640;

% 场景模式
uilabel(pL, 'Text','场景模式', 'Position',[15 y 200 20], ...
    'FontColor',C_TEXT, 'BackgroundColor',C_PANEL);
modeDD = uidropdown(pL, 'Items',{'P2P 避障','TOUR 巡访'}, ...
    'Value','P2P 避障', 'Position',[15 y-28 290 26], ...
    'BackgroundColor',[0.15 0.19 0.25], 'FontColor',C_TEXT, ...
    'ValueChangedFcn',@onModeChanged);

y = y - 70;
% 难度级别
uilabel(pL, 'Text','场景难度', 'Position',[15 y 200 20], ...
    'FontColor',C_TEXT, 'BackgroundColor',C_PANEL);
diffDD = uidropdown(pL, 'Items',{'新城/郊区 (easy)','典型城区 (medium)','密集老城 (hard)'}, ...
    'Value','典型城区 (medium)', 'Position',[15 y-28 290 26], ...
    'BackgroundColor',[0.15 0.19 0.25], 'FontColor',C_TEXT);

y = y - 70;
% 无人机数
uilabel(pL, 'Text','无人机数量 (3~30)', 'Position',[15 y 200 20],'FontColor',C_TEXT2,'BackgroundColor',C_PANEL);
nUAVEd = uieditfield(pL, 'numeric', 'Value',3, 'Limits',[3 30], ...
    'Position',[15 y-22 130 24], 'BackgroundColor',[0.15 0.19 0.25],'FontColor',C_TEXT);
% 控制点数
uilabel(pL, 'Text','控制点数/机', 'Position',[160 y 130 18],'FontColor',C_TEXT2,'BackgroundColor',C_PANEL);
nCtrlEd = uieditfield(pL, 'numeric', 'Value',5, 'Limits',[2 12], ...
    'Position',[160 y-22 145 24], 'BackgroundColor',[0.15 0.19 0.25],'FontColor',C_TEXT);

y = y - 50;
% 种群
uilabel(pL,'Text','种群 pop', 'Position',[15 y 90 18],'FontColor',C_TEXT2,'BackgroundColor',C_PANEL);
popEd = uieditfield(pL,'numeric','Value',40,'Limits',[10 200], ...
    'Position',[15 y-22 130 24],'BackgroundColor',[0.15 0.19 0.25],'FontColor',C_TEXT);
uilabel(pL,'Text','迭代 iter', 'Position',[160 y 90 18],'FontColor',C_TEXT2,'BackgroundColor',C_PANEL);
iterEd = uieditfield(pL,'numeric','Value',120,'Limits',[10 500], ...
    'Position',[160 y-22 140 24],'BackgroundColor',[0.15 0.19 0.25],'FontColor',C_TEXT);

y = y - 50;
uilabel(pL,'Text','最大评估 maxFE', 'Position',[15 y 200 18],'FontColor',C_TEXT2,'BackgroundColor',C_PANEL);
maxFEEd = uieditfield(pL,'numeric','Value',60000,'Limits',[5000 300000], ...
    'Position',[15 y-22 290 24],'BackgroundColor',[0.15 0.19 0.25],'FontColor',C_TEXT);

y = y - 55;
uilabel(pL,'Text','代价权重', 'Position',[15 y 200 18],'FontColor',C_TEXT,'BackgroundColor',C_PANEL,'FontWeight','bold');
y = y - 8;
uilabel(pL,'Text','障碍', 'Position',[15 y-18 60 18],'FontColor',C_TEXT2,'BackgroundColor',C_PANEL);
wObsEd = uieditfield(pL,'numeric','Value',60,'Limits',[0 200], ...
    'Position',[15 y-40 130 24],'BackgroundColor',[0.15 0.19 0.25],'FontColor',C_TEXT);
uilabel(pL,'Text','平滑', 'Position',[160 y-18 60 18],'FontColor',C_TEXT2,'BackgroundColor',C_PANEL);
wSmEd = uieditfield(pL,'numeric','Value',0.15,'Limits',[0 5], ...
    'Position',[160 y-40 140 24],'BackgroundColor',[0.15 0.19 0.25],'FontColor',C_TEXT);
y = y - 50;
uilabel(pL,'Text','机间分离', 'Position',[15 y 80 18],'FontColor',C_TEXT2,'BackgroundColor',C_PANEL);
wSepEd = uieditfield(pL,'numeric','Value',25,'Limits',[0 100], ...
    'Position',[15 y-22 130 24],'BackgroundColor',[0.15 0.19 0.25],'FontColor',C_TEXT);
uilabel(pL,'Text','路径长度', 'Position',[160 y 80 18],'FontColor',C_TEXT2,'BackgroundColor',C_PANEL);
wLenEd = uieditfield(pL,'numeric','Value',1,'Limits',[0 10], ...
    'Position',[160 y-22 140 24],'BackgroundColor',[0.15 0.19 0.25],'FontColor',C_TEXT);

y = y - 55;
runBtn = uibutton(pL,'Text','▶ 运行规划','BackgroundColor',C_ACCENT, ...
    'FontColor',[0.05 0.08 0.10],'FontWeight','bold','Position',[15 y 200 34], ...
    'ButtonPushedFcn',@runPlanner);
animBtn = uibutton(pL,'Text','▶ 播放动画','BackgroundColor',[0.25 0.45 0.55], ...
    'FontColor',C_TEXT,'Position',[225 y 80 34],'ButtonPushedFcn',@playAnim, ...
    'Enable','off');
exportBtn = uibutton(pL,'Text','⤓ 导出图片','BackgroundColor',[0.30 0.55 0.40], ...
    'FontColor',C_TEXT,'Position',[315 y 80 34],'ButtonPushedFcn',@exportImg, ...
    'Enable','off');

% ---- 阶段D：时间轴（动态车流回放） ----
y = y - 52;
uilabel(pL,'Text','时间轴 t (s) · 地面车流动态', 'Position',[15 y 290 18], ...
    'FontColor',C_TEXT,'BackgroundColor',C_PANEL,'FontSize',10,'FontWeight','bold');
y = y - 26;
tSlider = uislider(pL, 'Limits',[0 220], 'Value',0, 'Position',[15 y 230 3], ...
    'ValueChangedFcn',@onTimeSlider);
tLabel = uilabel(pL,'Text','t = 0.0 s','Position',[250 y-6 60 18], ...
    'FontColor',C_TEXT2,'BackgroundColor',C_PANEL,'FontSize',10);

y = y - 50;
% ---- 阶段E：通信/传感器层控制 ----
uilabel(pL,'Text','阶段E · 通信/传感器', 'Position',[15 y 290 18], ...
    'FontColor',C_TEXT,'BackgroundColor',C_PANEL,'FontSize',10,'FontWeight','bold');
y = y - 24;
showCommsChk = uicheckbox(pL,'Text','显示通信网络 (gNB/中继/终端 + 覆盖)', ...
    'Value',true,'Position',[15 y 290 20],'FontColor',C_TEXT2, ...
    'FontSize',10);
y = y - 24;
commsLinkChk = uicheckbox(pL,'Text','启用链路约束 (UAV 须保持网络覆盖)', ...
    'Value',false,'Position',[15 y 290 20],'FontColor',C_TEXT2, ...
    'FontSize',10);

y = y - 30;
statusL = uilabel(pL,'Text','就绪。设置参数后点击「运行规划」。', ...
    'Position',[15 y 290 60],'FontColor',C_TEXT,'BackgroundColor',C_PANEL, ...
    'WordWrap','on','FontSize',10);

% ---------- 右侧 3D 可视化（白底） ----------
ax3 = uiaxes(fig, 'Position',[350 300 600 440], ...
    'Color',[1 1 1], 'GridColor',[0.80 0.84 0.90], ...
    'XColor',[0.35 0.40 0.48],'YColor',[0.35 0.40 0.48],'ZColor',[0.35 0.40 0.48]);
title(ax3,'三维路径规划 · 态势可视化','Color',[0.15 0.20 0.30],'FontSize',12,'FontWeight','bold');
xlabel(ax3,'X'); ylabel(ax3,'Y'); zlabel(ax3,'Z');
grid(ax3,'on'); axis(ax3,'equal'); hold(ax3,'on');
view(ax3, [38 26]);

% ---------- 下方收敛曲线 ----------
axC = uiaxes(fig,'Position',[970 300 290 440], ...
    'Color',[0.05 0.07 0.10],'GridColor',[0.25 0.30 0.38], ...
    'XColor',C_TEXT2,'YColor',C_TEXT2,'ZColor',C_TEXT2);
title(axC,'CA 收敛曲线','Color',C_ACCENT,'FontSize',12);
xlabel(axC,'迭代'); ylabel(axC,'代价'); grid(axC,'on'); hold(axC,'on');

% 信息文本
infoL = uilabel(fig,'Text','场景：—   最优代价：—   最大穿透：—', ...
    'Position',[350 270 910 22],'FontColor',C_TEXT,'BackgroundColor',C_BG, ...
    'FontSize',10,'WordWrap','off');

% ---------- 应用状态 ----------
APP.mode = 'p2p';
APP.scene = [];
APP.trajs = {};
APP.bestX = [];
APP.curve = [];
APP.bestCost = [];
APP.animHandles = [];
APP.tCur = 0;            % 阶段D：当前回放时刻（秒）
APP.dynPlaying = false;  % 阶段D：车流动画是否正在播放
APP.ax3 = ax3; APP.axC = axC; APP.infoL = infoL; APP.statusL = statusL;
APP.UAV_COLORS = UAV_COLORS;
APP.tSlider = tSlider; APP.tLabel = tLabel;

% ===================== 回调 =====================
function runPlanner(~,~)
    set(statusL,'Text','规划中…（CA 算法运行中，请稍候）');
    drawnow;
    % 读取参数
    if strcmp(modeDD.Value,'TOUR 巡访'), mode='tour'; else mode='p2p'; end
    % 难度映射
    ddv = diffDD.Value;
    if strcmp(ddv,'新城/郊区 (easy)'), diff='easy';
    elseif strcmp(ddv,'密集老城 (hard)'), diff='hard';
    else diff='medium'; end
    nUAV = nUAVEd.Value; nCtrl = nCtrlEd.Value;
    pop = popEd.Value; iter = iterEd.Value; maxFE = maxFEEd.Value;
    wObs = wObsEd.Value; wSm = wSmEd.Value; wSep = wSepEd.Value; wLen = wLenEd.Value;

    % 构造场景（带难度、权重覆盖）
    if strcmp(mode,'tour')
        scene = mu_config('tour','nUAV',nUAV,'difficulty',diff,'seed',2);
    else
        scene = mu_config('p2p','nUAV',nUAV,'nCtrl',nCtrl,'difficulty',diff,'seed',2);
    end
    scene.w.obstacle   = wObs;
    scene.w.smooth     = wSm;
    scene.w.separation = wSep;
    scene.w.length     = wLen;
    % 阶段E：链路约束（默认关；勾选后启用 UAV 须保持网络覆盖的软惩罚）
    if isfield(APP,'commsLinkChk') && APP.commsLinkChk.Value
        scene.w.comms = 30;   % 软惩罚权重，与 w.obstacle 同量级
    else
        scene.w.comms = 0;
    end

    try
        [bx, bc, cv, trajs, sc] = mu_run_planner(mode, ...
            'pop',pop,'iter',iter,'maxFE',maxFE,'nUAV',nUAV, ...
            'difficulty',diff,'nCtrl',nCtrl,'seed',2);
    catch ME
        set(statusL,'Text',['运行出错：' ME.message]);
        return;
    end

    APP.scene = sc; APP.trajs = trajs; APP.bestX = bx;
    APP.curve = cv; APP.bestCost = bc; APP.mode = mode;

    % 诊断穿透（penetration = 穿透深度，>=0 表示安全，>0 表示穿入障碍）
    mp = 0;
    for k=1:numel(trajs)
        [~,o] = mu_obstacle_dist(trajs{k}, sc.obstacles, 0);
        mp = max(mp, -min(o));   % 最深穿透深度（dist<0 才穿透）
    end
    % 绘制
    drawScene();
    cla(axC); hold(axC,'on'); grid(axC,'on');
    semilogy(axC, cv, 'Color',C_ACCENT, 'LineWidth',1.6);
    title(axC,'CA 收敛曲线','Color',C_ACCENT);
    xlabel(axC,'迭代'); ylabel(axC,'代价 (log)');
    % 城市特征统计
    ot = {sc.obstacles.type};
    nB = sum(strcmp(ot,'bldg')); nTw = sum(strcmp(ot,'tower'));
    nNf = sum(strcmp(ot,'nofly')); nTr = sum(strcmp(ot,'tree'));
    nWt = sum(strcmp(ot,'water')); tOn = ~isempty(sc.terrainF);
    set(infoL,'Text',sprintf('场景：%s/%s  无人机：%d  楼:%d 塔:%d 禁飞:%d 树:%d 水:%d 地形:%d  代价：%.2f  穿透：%.3f', ...
        mode, diff, nUAV, nB, nTw, nNf, nTr, nWt, tOn, bc, mp));
    set(statusL,'Text',sprintf('完成。%s/%s 城市环境，最优代价 %.2f，最大障碍穿透 %.3f（0 表示完全无碰撞）。', ...
        diffLabel4(diff), mode, bc, mp));
    set(animBtn,'Enable','on');
    set(exportBtn,'Enable','on');
    drawnow;
end

function exportImg(~,~)
    % 用 exportgraphics 保存 UIAxes（print/saveas 对 uiaxes 不支持，会导出空白）
    % 结果自动存入 results/ 子目录，文件名带时间戳，方便区分每次运行。
    if isempty(APP.trajs) || isempty(APP.scene)
        set(statusL,'Text','尚无结果可导出，请先运行规划。');
        return;
    end
    % results 子目录（与 demo 导出一致）
    appDir = fileparts(mfilename('fullpath'));
    outDir = fullfile(appDir, 'results');
    if ~exist(outDir,'dir'), mkdir(outDir); end
    runTS = datestr(now, 'yyyy-mm-dd_HHMMSS');
    % 场景标签：模式 + 难度
    sc = APP.scene;
    tag = sprintf('%s_%s', APP.mode, sc.difficulty);
    try
        % 3D 场景：PNG（位图 300dpi）+ EPS（矢量）；先写 .tmp 再原子改名防损坏
        f3 = fullfile(outDir, sprintf('muav_%s_3D_%s', tag, runTS));
        mu_save_ui(APP.ax3, [f3 '.png'], 'png');
        mu_save_ui(APP.ax3, [f3 '.eps'], 'eps');
        % 收敛曲线：PNG + EPS
        fC = fullfile(outDir, sprintf('muav_%s_convergence_%s', tag, runTS));
        mu_save_ui(APP.axC, [fC '.png'], 'png');
        mu_save_ui(APP.axC, [fC '.eps'], 'eps');
        set(statusL,'Text',sprintf('已导出至 results/：%s（3D + 收敛曲线，含 PNG/EPS）', [f3 '.png']));
    catch ME
        set(statusL,'Text',['导出失败：' ME.message]);
    end
end

function onModeChanged(~,~)
    % TOUR 模式控制点数由任务数自动决定，禁用 nCtrl 控件避免误导
    if strcmp(modeDD.Value,'TOUR 巡访')
        nCtrlEd.Enable = 'off';
        nCtrlEd.BackgroundColor = [0.10 0.12 0.15];
    else
        nCtrlEd.Enable = 'on';
        nCtrlEd.BackgroundColor = [0.15 0.19 0.25];
    end
end

function drawScene()
    ax = APP.ax3; cla(ax); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
    sc = APP.scene; trajs = APP.trajs;
    % 阶段D：将当前回放时刻传入场景，mu_draw_scene 据此渲染动态车流
    sc.tCur = APP.tCur;
    % 阶段E：按开关决定是否渲染通信/传感器层（临时清空 comms/sensors 字段以跳过渲染）
    if isfield(APP,'showCommsChk') && ~APP.showCommsChk.Value
        sc = rmfield(sc, 'comms');   % mu_draw_scene 内 isfield 判空跳过
        if isfield(sc,'sensors'), sc = rmfield(sc,'sensors'); end
    end
    % 调用共享精致渲染（自动识别 UIAxes）
    mu_draw_scene(sc, trajs, APP.mode, ax);
    % 重新收集动画头句柄（机头光点）
    APP.animHandles = gobjects(0);
    for k=1:numel(trajs)
        T = trajs{k};
        col = APP.UAV_COLORS(mod(k-1,size(APP.UAV_COLORS,1))+1,:);
        h = scatter3(ax, T(1,1), T(1,2), T(1,3), 130, col, 'filled', ...
            'Marker','o','MarkerEdgeColor','w','LineWidth',1.4);
        APP.animHandles(end+1) = h;
    end
    % 阶段D：重置时间轴滑块到 0（新车流场景）
    APP.tCur = 0;
    if isfield(APP,'tSlider') && ishandle(APP.tSlider)
        APP.tSlider.Value = 0;
        APP.tLabel.Text = 't = 0.0 s';
    end
    drawnow;
end

function playAnim(~,~)
    if isempty(APP.trajs), return; end
    trajs = APP.trajs;
    N = size(trajs{1},1);
    set(statusL,'Text','动画播放中…（无人机轨迹 + 地面车流动态）');
    % 阶段D：动画时间与车流时间轴同步推进
    tSpan = 220;
    for i=1:N
        % UAV 头点
        for k=1:numel(trajs)
            T = trajs{k};
            if i<=size(T,1)
                set(APP.animHandles(k),'XData',T(i,1),'YData',T(i,2),'ZData',T(i,3));
            end
        end
        % 车流时刻：将 UAV 帧序映射到 [0, tSpan]
        tC = tSpan * (i-1) / max(1, N-1);
        APP.tCur = tC;
        redrawDynamic(tC);
        % 同步滑块显示
        if isfield(APP,'tSlider') && ishandle(APP.tSlider)
            APP.tSlider.Value = tC; APP.tLabel.Text = sprintf('t = %.1f s', tC);
        end
        drawnow limitrate;
    end
    set(statusL,'Text','动画播放完成。');
end

function redrawDynamic(tC)
% 阶段D：按时刻 tC 重绘地面车流层（仅删除旧车辆句柄并重画，避免整体 cla）
    ax = APP.ax3;
    if ~isfield(ax.UserData,'dynHandles') || isempty(ax.UserData.dynHandles)
        return;
    end
    delete(ax.UserData.dynHandles(:));
    ax.UserData.dynHandles = gobjects(0);
    if isfield(APP.scene,'dynamics') && ~isempty(APP.scene.dynamics) && ...
       isfield(APP.scene.dynamics,'vehicles') && ~isempty(APP.scene.dynamics.vehicles)
        mu_draw_dynamic(ax, APP.scene.dynamics, tC);
    end
end

function onTimeSlider(~,~)
% 阶段D：时间轴滑块手动控制（暂停车流回放到指定时刻）
    if isempty(APP.scene) || ~isfield(APP.scene,'dynamics') || isempty(APP.scene.dynamics)
        return;
    end
    tC = APP.tSlider.Value;
    APP.tCur = tC;
    APP.tLabel.Text = sprintf('t = %.1f s', tC);
    redrawDynamic(tC);
    drawnow;
end

% 初始提示
set(statusL,'Text','就绪。选择场景模式与参数，点击「运行规划」。TOUR 场景每机控制点数由任务数自动决定。');
end

function lab = diffLabel4(diff)
if strcmpi(diff,'easy'), lab = '新城/郊区';
elseif strcmpi(diff,'hard'), lab = '密集老城';
else lab = '典型城区'; end
end
