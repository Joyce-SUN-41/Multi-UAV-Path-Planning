function scene = mu_config(mode, varargin)
% mu_config — 构造城市复杂环境多无人机配送场景配置
%   scene = mu_config('p2p')                 静态障碍避障（默认 3 机，medium）
%   scene = mu_config('tour')                多机客户点配送巡访（多目标）
%   scene = mu_config(mode, 'Name',Value,...) 覆盖默认参数
%
% 支持难度：'easy' | 'medium' | 'hard'（控制城市密度与复杂度）
%   easy   : 稀疏新城/郊区，低层为主，少量禁飞区，地形平缓
%   medium : 典型城区，中高层混合，含通信塔架 + 1~2 禁飞区 + 轻微地形起伏
%   hard   : 密集老城，高层簇 + 多禁飞区 + 通信塔群 + 明显地形（山丘/河谷）
%            + 狭窄街道峡谷，密度最高
%   无人机数量 nUAV 可在 3~30 之间选择。
%
% 场景字段（均为结构化、可被代价函数直接读取）：
%   mode         'p2p' | 'tour'
%   difficulty   'easy' | 'medium' | 'hard'
%   nUAV         无人机数量 (3~30)
%   nCtrl        每架无人机 B 样条控制点数量（不含起点/终点固定端）
%   bounds       1x6 [xmin xmax ymin ymax zmin zmax] 搜索空间（城市范围）
%   starts       nUAV x 3 起点（配送站/仓库 depot）
%   goals        nUAV x 3 终点
%   obstacles    struct 数组，含 cube/cylinder/sphere/row/tower/nofly/tree/terrain
%   depots       (p2p/tour) nDepot x 3 仓库坐标
%   tasks        (tour 模式) nTask x 3 客户点坐标
%   taskAssign   (tour 模式) 每架机分配的客户点索引 cell
%   w            struct 代价权重
%   smooth       样条采样点数（轨迹离散化密度）
%   seed         随机种子

% ---- 环境整体缩放因子 ENV_SCALE（须与 mu_city_layout 一致）----
% 城市几何整体放大 2.5 倍（X/Y 半范围 = 200*2.5 = 500），使三个地图 X/Y 均为 [-500, +500]。
% 无人机本体尺寸（vehMargin / 车辆半径 / 轨迹线宽）不变，
% 缝宽相对无人机变宽，优化（CAv9x）难度下降，无人机不易卡缝；
% 同时避免过度放大导致楼宇过高、飞行高度上限与空域尺度失配。
% 注意：本值仅控制"城市几何内部比例"，搜索外框 bounds 已独立锁死（见下）。
ENV_SCALE = 2.5;

% ---- 默认搜索空间（长方体外框，严格锁死，不随 ENV_SCALE 漂移）----
% 用户要求：X/Y 轴均为 [-500, +500]（跨度 1000），Z 轴 [-50, +300]。
% 这三个地图的外框必须恒定为此值——故直接硬编码，不再由 ENV_SCALE 派生，
% 任何分支都不能覆盖 scene.bounds 的 X/Y/z 范围（专项回归 verify_round5/6 例外，
% 它们故意传入非默认 bounds 以测试 covR 跟随逻辑，不影响默认路径）。
% 城市几何 XY 半范围 = 200*ENV_SCALE = 500，恰好填满此外框；Z 轴按实际地形/楼高生成，
% 楼顶上限由 mu_city_layout.Z_CEIL(=280) 裁剪以保证 <= 300。
BOUNDS_LOCKED = [-500 500 -500 500 -50 300];
bounds = BOUNDS_LOCKED;

% ---- 默认代价权重 ----
w = struct();
w.length   = 1.0;    % 路径长度
w.smooth   = 0.15;   % 曲率/平滑度（二阶差分）
w.obstacle = 60.0;   % 障碍穿透惩罚系数（软约束）
w.boundary = 40.0;   % 越界惩罚系数
w.separation = 25.0; % (tour) 机间最小间隔惩罚
w.ordering = 0.0;    % (tour) 顺序合理性
w.comms = 0.0;       % 阶段E：通信链路约束（UAV 须落在 comms 节点覆盖内；默认关，避免破坏现有规划正则）
w.vehicle = 60.0;    % 阶段D：时变车辆碰撞惩罚系数（与 w.obstacle 同量级，避免规划器"擦蹭车辆"）
w.height  = 1e5;     % 飞行高度硬上限：超过 flightCeiling(默认100m) 即施加超大惩罚(炸机)，远超其他项量级

scene = struct();
scene.mode      = mode;
scene.difficulty = 'medium';
scene.nUAV      = 3;
scene.nCtrl     = 5;
scene.ctrlPer    = [];   % (tour) 每机控制点维度数组；空闲机取最小2，忙碌机取 nCtrl。R20 差异化维度
scene.bounds    = bounds;
scene.smooth    = 140;
scene.seed      = 0;
scene.w         = w;

% 并行评估开关：代价函数逐 UAV 循环可用 parfor 加速（需已开并行池）。
% 默认开；若未开并行池或显式置 false，代价函数自动退化为普通 for，零风险。
scene.useParallel = true;
scene.T_horizon = 60;   % 单架机飞行时间预算(秒)，用于把轨迹弧长配准到时间轴以接入时变碰撞(D1)
scene.safeMargin   = 6*ENV_SCALE; % 障碍安全壳外扩(m)，随环境放大（p2p/tour 共用）
scene.terrainMargin= 8*ENV_SCALE; % 最低飞行高度裕度(m)，随环境放大（p2p/tour 共用）
scene.vehMargin    = 3; % 车辆安全壳外扩(m)，D1 时变碰撞共用（无人机本体尺寸，不乘 S）
% 飞行高度硬上限（米，绝对 z）。无人机飞行高度超过此值的轨迹段受 w.height(=1e5)
% 超大惩罚，等效"炸机"硬约束（禁飞）。用户规定：飞行高度超过 100m 即算炸机，
% 故此处固定为 100（绝对值，不随 ENV_SCALE 漂移）。必须 <= bounds(6)=300。
% 用户可用 'flightCeiling' 覆盖（绝对值，单位米），但任何覆盖都不得突破 300。
scene.flightCeiling = 100;   % = 100m 绝对封顶，超过即炸机（禁飞），同时缩小 CA 搜索空间
% 阶段E：通信链路竖直容差(m)，R9 修复——不再硬编码 30，改为随空域尺度缩放
% （旧版 mu_comms_penalty 内 VZ=30 固定，大场景下竖直方向限死、小场景又过宽）。
% 取空域 z 跨度的 ~12%，落在合理巡航层厚度量级；可被用户 'commsVZ' 覆盖。
scene.commsVZ = 0.12 * (bounds(6) - bounds(5));

scene.needPoints = true;

% 并行评估开关：代价函数逐 UAV 循环可用 parfor 加速（需已开并行池）。
% 默认开；若未开并行池或显式置 false，代价函数自动退化为普通 for，零风险。
scene.useParallel = true;

if strcmpi(mode, 'p2p')
    scene.nUAV  = 3;
    scene.starts = [];
    scene.goals  = [];
    scene.tasks = [];
    scene.taskAssign = {};
elseif strcmpi(mode, 'tour')
    scene.nUAV  = 3;
    scene.starts = [];
    scene.goals  = [];
    scene.tasks = [];
    scene.taskAssign = {};
else
    error('mu_config: 未知模式 "%s"，应为 "p2p" 或 "tour"', mode);
end

% ---- 用户覆盖 ----
if nargin > 1
    for i=1:2:numel(varargin)
        name = varargin{i}; val = varargin{i+1};
        if strcmpi(mode,'tour') && strcmpi(name,'nCtrl')
            warning('mu_config: tour mode nCtrl is auto-decided by task count; nCtrl ignored.');
            continue;
        end
        if strcmpi(name,'nUAV')
            val = max(3, min(30, val));
            scene.nUAV = val;
            scene.needPoints = true;

% 并行评估开关：代价函数逐 UAV 循环可用 parfor 加速（需已开并行池）。
% 默认开；若未开并行池或显式置 false，代价函数自动退化为普通 for，零风险。
scene.useParallel = true;
            if strcmpi(mode,'tour') && ~isempty(scene.tasks)
                nT = size(scene.tasks,1);
                scene.taskAssign = cell(val,1);
                for k=1:val
                    idx = k:val:nT;
                    scene.taskAssign{k} = idx(:)';
                end
                maxT = max(cellfun(@numel, scene.taskAssign));
                scene.nCtrl = 2 * (maxT + 1);
            end
            continue;
        end
        if isfield(scene, name)
            scene.(name) = val;
        elseif isfield(scene.w, name)
            scene.w.(name) = val;
        else
            warning('mu_config: ignoring unknown parameter "%s"', name);
        end
    end
end
if strcmpi(mode,'tour') && isempty(scene.taskAssign)
    nT = size(scene.tasks,1);
    scene.taskAssign = cell(scene.nUAV,1);
    for k=1:scene.nUAV
        scene.taskAssign{k} = (k:scene.nUAV:nT)';
    end
end

% ---- 生成城市障碍（依据最终 difficulty，程序化布局）----
city = mu_city_layout(scene.difficulty, scene.seed);
scene.obstacles = city.obstacles;
scene.terrainF  = city.terrainF;     % 地形高度函数句柄 z=z_ground(x,y)
scene.roads     = city.roads;        % 分级立体路网（arterial/collector，带 z_profile）
scene.junctions = city.junctions;    % 交叉口节点（含地形标高）
scene.bridges   = city.bridges;      % 跨江/跨谷大桥（L2 立体交通层，渲染字段；碰撞副本在 obstacles 中 type='bridge'）
scene.airspace  = city.airspace;     % 空域分层参数（L3：低空/中层/高空）
scene.geoRef    = city.geoRef;       % 环境参考系（WGS84 锚点 + heading + 尺度）
scene.atmosphere= city.atmosphere;   % 大气条件（温度/湿度/气压/风/能见度）
scene.envdyn    = city.envdyn;       % 环境动态（光照/天气，驱动渲染）
scene.dynamics  = city.dynamics;     % 阶段D：道路动态交通层（时变车辆障碍）
scene.comms     = city.comms;         % 阶段E：通信网络节点（gNB/relay/iot，含覆盖半径）
scene.sensors   = city.sensors;       % 阶段E：传感器挂载点（cam/met/noise/lidar 元数据）

% R16 修复：comms 水平覆盖半径 covR 此前在 mu_city_layout 中写死绝对半径
% （320/240/180 ...），不随场景 bounds 缩放；而竖直容差 VZ 已用
% bounds(6)-bounds(5) 缩放（R9），两者不对称。若场景 bounds 放大（如 800x800），
% covR 不缩放会让链路约束语义失真（覆盖相对变小、约束过严）。此处集中做尺度归一：
% 以锁死默认场景 x 跨度 (bounds(2)-bounds(1))=1000 为基准，covR 乘场景 x 跨度比例，
% 与 VZ 缩放策略对称。默认场景比例=1.0（无漂移，covR 与 city 原始基准一致）。
% 注：专项回归 verify_round5/6 故意传入非默认 bounds 以测试此跟随逻辑，此处一律按
% 传入的 scene.bounds 计算比例，与默认值一致。
commsScaleXY = (scene.bounds(2) - scene.bounds(1)) / (BOUNDS_LOCKED(2) - BOUNDS_LOCKED(1));
if commsScaleXY ~= 1 && ~isempty(scene.comms)
    for k = 1:numel(scene.comms)
        scene.comms(k).covR = scene.comms(k).covR * commsScaleXY;
    end
end

% ---- 仓库（depot）：地面固定配送站 ----
scene.depots = mu_depots(scene.difficulty, scene.seed, ENV_SCALE);
% R35 修复：仓库 z 原固定 22m，但地形在边缘可达 100m+，导致 depot/起点/终点
%   落入地下。先按地形标高抬升每个 depot 的 z（保证起点/终点均在地面以上），
%   再交给 mu_clear_depots 做 3D 避障推开，避障计算基于真实高度。
depotMinZ = 22;
for dk = 1:size(scene.depots,1)
    tz = scene.terrainF(scene.depots(dk,1), scene.depots(dk,2));
    scene.depots(dk,3) = max(depotMinZ, tz + 12*ENV_SCALE);
end
% 第十轮(R28)修复：程序化 depot 与随机城市障碍独立采样、无避障校验，
% 部分 depot 会落在建筑/禁飞区安全壳内(实测 easy/hard 必现)，导致规划器
% 从穿透态出发、idle 机被障碍代价持续不公平惩罚。此处把 depot 统一推出壳外。
scene.depots = mu_clear_depots(scene.depots, scene.obstacles, scene.safeMargin + 5);
% 推开后地形可能微变（XY 漂移），再次抬升 z 保证仍在地面以上。
for dk = 1:size(scene.depots,1)
    tz = scene.terrainF(scene.depots(dk,1), scene.depots(dk,2));
    scene.depots(dk,3) = max(scene.depots(dk,3), tz + 12*ENV_SCALE);
end

% ---- 随机生成起终点（仓库中选择 + 客户点选择，互不重叠、避开障碍）----
if scene.needPoints
    s = RandStream('mt19937ar','Seed', scene.seed + 1000);
    if strcmpi(mode,'p2p')
        % 起点=仓库子集，终点=仓库另子集（不同仓库），p2p 为站间配送
        P = mu_assign_depots(2*scene.nUAV, scene.depots, scene.obstacles, s);
        scene.starts = P(1:scene.nUAV,:);
        scene.goals  = P(scene.nUAV+1:end,:);
    else
        % tour：每机从同一仓库出发并返回（仓库作为起点/终点）
        idxs = mu_pick_depots(scene.nUAV, scene.depots, s);
        scene.starts = scene.depots(idxs,:);
        scene.goals  = scene.depots(idxs,:);
    end
    scene.needPoints = false;
end

% ---- tour 客户点：依据最终 difficulty 生成数量，挂在楼宇高度上 ----
if strcmpi(mode,'tour')
    if strcmpi(scene.difficulty,'easy')
        nT = 12;
    elseif strcmpi(scene.difficulty,'hard')
        nT = 30;
    else
        nT = 20;
    end
    scene.tasks = mu_customer_points(nT, scene.obstacles, scene.depots, scene.seed, ENV_SCALE);
    % 均分客户点给各机（最终 nUAV）
    nT = size(scene.tasks,1);
    scene.taskAssign = cell(scene.nUAV,1);
    for k=1:scene.nUAV
        idx = k:scene.nUAV:nT;
        scene.taskAssign{k} = idx(:)';
    end
    maxT = max(cellfun(@numel, scene.taskAssign));
    scene.nCtrl = 2 * (maxT + 1);
    % R20：差异化控制点维度。空闲机（无任务点）只需 start->goal 直线起降，
    % 取最小控制点数 2（B 样条直线插值），不再占用最忙机的 2*(maxT+1) 维度，
    % 消除 CA 搜索空间中约 1/3 的"死维度"（空闲机控制点不参与任何代价计算）。
    % 忙碌机仍取 nCtrl 以覆盖分段构造（D2/M1）。
    scene.ctrlPer = zeros(scene.nUAV,1);
    for k=1:scene.nUAV
        if isempty(scene.taskAssign{k})
            scene.ctrlPer(k) = 2;                       % 空闲机：start->goal 直线，最小维度
        else
            % R42：按该机实际任务数分配控制点，而非统一取 nCtrl=2*(maxT+1)。
            % 任务少的机若取统一 nCtrl，多余控制点会被 mu_build_tour_traj 静默丢弃，
            % 形成跨机"死维度"（R20 仅消除了空闲机死维度，此遗留补全）。
            scene.ctrlPer(k) = 2 * (numel(scene.taskAssign{k}) + 1);
        end
    end
    scene.dimCtrl = sum(scene.ctrlPer * 3);
end
end

% ============ 城市布局生成器（程序化，rich-building 版） ============
% 实现已抽取为独立文件 mu_city_layout.m（重庆式层次城市，阶段A 重构）。
% 本文件第 112 行直接调用同目录的 mu_city_layout（文件函数），此处不再内联。

% ============ 仓库 / 配送站 ============
function depots = mu_depots(difficulty, seed, ENV_SCALE)
% 仓库设在城市边缘/空旷处（避开建筑密集区），地面固定点
s = RandStream('mt19937ar','Seed', 50*seed + 3);
half = 200 * ENV_SCALE;   % 与 mu_city_layout / bounds 一致的放大半范围
nD = 6;   % 最多 6 个仓库，供 nUAV 截取
depots = zeros(nD,3);
% 放在四条边中点附近，留足空旷
candidates = [ ...
    -half+15,  0, 22; ...
     half-15,  0, 22; ...
     0, -half+15, 22; ...
     0,  half-15, 22; ...
     -half*0.6, -half*0.6, 22; ...
     half*0.6,  half*0.6, 22];
depots = candidates(1:nD,:);
end

function idxs = mu_pick_depots(n, depots, s)
% 从仓库中选取 n 个（可重复仓库由不同机共享，这里简单取前 n，循环）
idxs = mod((0:n-1), size(depots,1)) + 1;
end

function P = mu_assign_depots(n, depots, obst, s)
% 从仓库子集中为起点/终点分配 n 个点（前 n/2 为起点，后 n/2 为终点）
% 起点与终点取自不同仓库索引，保证起↔终不共仓库（nUAV<=nD 时严格不重叠）
nD = size(depots,1);
halfN = ceil(n/2);
starts = mod((0:halfN-1), nD) + 1;               % 起点仓库循环
goals  = mod((floor(nD/2):floor(nD/2)+halfN-1), nD) + 1;  % 终点错开半圈
P = zeros(n,3);
P(1:halfN,:)        = depots(starts,:);
P(halfN+1:n,:)      = depots(goals,:);
end

% ============ 客户点（tour） ============
function tasks = mu_customer_points(nT, obst, depots, seed, ENV_SCALE)
% 客户点挂在楼宇上：楼顶停机坪 或 楼外低空停靠点
% 约束：所有客户点必须落在锁死外框内（XY ∈ [-500,500]，Z ∈ [-50,300]），
% 故 z 上限钳到 295（留余量），XY 偏移后若越 ±500 则跳过该候选重选。
s = RandStream('mt19937ar','Seed', 200*seed + 11);
XY_LIM = 500; Z_MAX = 295; Z_MIN = -50;
tasks = zeros(nT,3);
placed = 0; attempts = 0;
bldgs = obst(strcmp({obst.type},'bldg'));
while placed < nT && attempts < 5000
    attempts = attempts + 1;
    if isempty(bldgs), break; end
    o = bldgs(randi(s,numel(bldgs)));
    cx = o.c(1); cy = o.c(2);
    topH = o.baseH + o.bodyH;
    zlev = rand(s,1);
    if zlev < 0.5
        % 楼顶停机坪（楼正上方，安全壳外，留足 >margin 余量）
        z = min(topH + 12*ENV_SCALE, Z_MAX);
    else
        % 楼外低空停靠点（xy 偏移出裙楼半宽+裕度）
        off = 8*ENV_SCALE;
        ang = 2*pi*rand(s,1);
        cx = cx + (o.hw(1)+o.pod(1)+off)*cos(ang);
        cy = cy + (o.hw(2)+o.pod(2)+off)*sin(ang);
        z = max(14*ENV_SCALE, topH*0.3);
        z = min(z, Z_MAX);
    end
    % 楼外停靠点偏移可能越过 ±500 外框，越界则放弃该候选重选
    if abs(cx) > XY_LIM || abs(cy) > XY_LIM, continue; end
    cand = [cx cy z];
    [dter,~] = mu_obstacle_dist(cand, obst(structcmp({obst.type},'terrain')), 8*ENV_SCALE);
    [dnf,~]  = mu_obstacle_dist(cand, obst(structcmp({obst.type},'nofly')), 0);
    [dall,~] = mu_obstacle_dist(cand, obst, 6*ENV_SCALE);
    if dter<=0 || dnf<=0 || dall<=0, continue; end
    if placed>0 && min(sum((tasks(1:placed,:)-cand).^2,2)) < 100*ENV_SCALE^2, continue; end
    placed = placed + 1;
    tasks(placed,:) = cand;
end
if placed < nT
    while placed < nT
        % 兜底随机点：范围收紧到外框内（留出安全壳余量），并钳制到锁死框
        cand = [(-XY_LIM+6*ENV_SCALE)+2*(XY_LIM-6*ENV_SCALE)*rand(s,1), ...
                (-XY_LIM+6*ENV_SCALE)+2*(XY_LIM-6*ENV_SCALE)*rand(s,1), ...
                max(Z_MIN+1, 20*ENV_SCALE+60*ENV_SCALE*rand(s,1))];
        cand(1:2) = max(-XY_LIM, min(XY_LIM, cand(1:2)));
        cand(3)   = min(Z_MAX, cand(3));
        [d,~] = mu_obstacle_dist(cand, obst, 6*ENV_SCALE);
        if d>0
            placed = placed+1; tasks(placed,:)=cand;
        end
    end
end
end

function tf = structcmp(cells, val)
tf = false(size(cells));
for i=1:numel(cells), tf(i) = strcmpi(cells(i), val); end
end
