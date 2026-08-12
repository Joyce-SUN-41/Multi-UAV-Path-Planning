function city = mu_city_layout(difficulty, seed)
% mu_city_layout — 重庆式层次城市布局生成器（阶段A 重构）
%   difficulty : 'easy'(半岛新城) | 'medium'(坡地主城) | 'hard'(立体老城)
%   seed       : 随机种子
% 返回 struct:
%   obstacles  : 障碍数组（bldg/tower/nofly/tree/water/terrain/streetlight/sign/bridge，语义一致）
%   terrainF   : 地形高度场函数句柄（保证全域 >=0）
%   roads      : 分级立体路网（arterial/collector，带 z_profile）
%   junctions  : 交叉口节点（含地形标高）
%   bridges    : 跨江/跨谷大桥（L2 立体交通层，渲染 + 碰撞双用：障碍副本含 deck/pier）
%   airspace   : 空域分层参数（L3：低空/中层/高空）
%   materials  : 材质查找表（介电常数/反射率/RGB），建筑按 tier 赋 materialId
%
% 设计：路网先行 -> 地块划分 -> 按 BCR 地块内密铺 + 四带高度分层(L/M/H/T)。
% 街宽与建筑密度解耦，密度由 BCR 控制，彻底解决旧版"贴线法"的过散/过挤失真。
% 阶段B：streetlight/sign 沿路网生成并参与碰撞；bridge 升级为可碰撞实体结构；
%        materials 表补全（concrete/glass/metal/asphalt/water/vegetation/bridgedeck）。

obs = struct('type',[],'c',[],'r',[],'half',[],'h',[],'grid',[],'gap',[], ...
             'hmin',[],'hmax',[],'nbx',[],'nby',[],'ball',[],'xz',[],'zlo',[],'zhi',[],'f',[], ...
             'hw',[],'pod',[],'baseH',[],'bodyH',[],'setback',[],'kind',[],'hue',[],'poly',[],'roof',[],'tier',[], ...
             'materialId',[],'pole',[],'arm',[],'post',[],'panel',[],'deck',[],'pier',[]);

s = RandStream('mt19937ar','Seed', 100*seed + 7);

% ===== 环境整体缩放因子 ENV_SCALE（2026-08-10 用户要求）=====
% 目标：把"城市几何"整体放大，无人机本体尺寸（车辆/无人机半径、vehMargin、轨迹线宽）
% 保持不变 —— 这样"楼间隙 / 安全壳 / 走廊"相对无人机显著变宽，无人机不易卡缝，
% 优化问题（CAv9x 搜索可行域）难度随之下降。
%   - half / bounds / 路宽 / 楼高 / 地形幅度 / footprint / 最小走廊 / 安全壳 / 桥宽 等均乘 S
%   - terrFreq 除以 S（保持地形波长在世界尺度不变，仅幅度放大）
%   - vehMargin / 车辆半径 / 轨迹线宽 不乘 S（无人机物理尺寸恒定）
% 当前选 S=2.5：环境放大 2.5 倍，缝宽相对无人机显著变宽，CAv9x 搜索难度下降，无人机不易卡缝。
% 注意：mu_config.m 中 ENV_SCALE 必须与此保持一致；修改缩放时两处须同步，或由统一常量管理。
ENV_SCALE = 2.5;
half = 200 * ENV_SCALE;   % 城市半范围（已含 ENV_SCALE，下游所有 half 引用自动放大）

% ===== 难度参数（重庆式三梯度：半岛新城 / 坡地主城 / 立体老城）=====
% BCR 建筑覆盖率（密度主参数）、各级路宽、四带高度占比 L/M/H/T、地形幅度
% 地形幅度 terrAmp 是城市立体感的"地基隆起"，须与建筑高度 tierRatio 匹配：
%   - 旧版 hard 45 / medium 28 在 bounds Z=150 下把"地形抬升 + 高层塔"挤入 ~150m，
%     视觉上塔楼矮于地形山头，"三维环境比外框小一大截"。
%   - 新版：缩 terrAmp 给楼让出垂直空间（hard 45→22、medium 28→18、easy 12→10），
%     同步扩 bounds Z 至 180，整体城市更"立体高耸"而非"贴地山谷"。
if strcmpi(difficulty,'easy')      % 半岛新城：缓坡、低中层为主、成片但开阔
    BCR = 0.32; arterialW = 36*ENV_SCALE; collectorW = 22*ENV_SCALE; localW = 28*ENV_SCALE;
    tierRatio = [0.45 0.35 0.18 0.02];
    hL = [18 32]*ENV_SCALE; hM = [32 70]*ENV_SCALE; hH = [70 120]*ENV_SCALE; hT = [120 150]*ENV_SCALE;
    terrAmp = 10*ENV_SCALE; terrFreq = (1/90)/ENV_SCALE; nB = 6; water = 0;
    towers = 1; nofly = 1; trees = 40;
elseif strcmpi(difficulty,'hard')  % 立体老城：陡坡深谷、超密高层簇、窄街
    BCR = 0.55; arterialW = 34*ENV_SCALE; collectorW = 20*ENV_SCALE; localW = 8*ENV_SCALE;
    tierRatio = [0.18 0.30 0.37 0.15];
    hL = [20 32]*ENV_SCALE; hM = [32 110]*ENV_SCALE; hH = [110 165]*ENV_SCALE; hT = [165 200]*ENV_SCALE;
    terrAmp = 22*ENV_SCALE; terrFreq = (1/70)/ENV_SCALE; nB = 10; water = 1;
    towers = 5; nofly = 3; trees = 75;
else                               % medium 坡地主城：中高混合、明显坡地
    BCR = 0.45; arterialW = 36*ENV_SCALE; collectorW = 22*ENV_SCALE; localW = 16*ENV_SCALE;
    tierRatio = [0.30 0.35 0.28 0.07];
    hL = [16 32]*ENV_SCALE; hM = [32 95]*ENV_SCALE; hH = [95 165]*ENV_SCALE; hT = [165 190]*ENV_SCALE;
    terrAmp = 18*ENV_SCALE; terrFreq = (1/80)/ENV_SCALE; nB = 8; water = 1;
    towers = 3; nofly = 2; trees = 56;
end
AVG_FOOT = 400 * ENV_SCALE^2;   % 单栋平均 footprint (m^2)，按 BCR 估算每地块目标栋数（面积随 S²）
CORRIDOR = 6 * ENV_SCALE;       % 最小可飞走廊宽度 (m)，保证楼间隙 >= 此值（缝随环境放大）
Z_CEIL = 180 * ENV_SCALE;       % 楼顶相对地形起算不超过此绝对高度（与 bounds z 上限一致，乘 S）
% 道路红线缓冲：楼中心到道路中心线距离必须 >= 路半宽 + 此留白，否则拒绝放置
%   （用户要求：有公路/立交的地方纵向不应有高楼，避免楼压路/穿模）。
% arterial 主干 + 立交沿线留白大（两侧成片低层/留白带）；collector 是街区次干，
%   两侧本就该有楼，故仅防楼中心压到路幅本身（留白很小），否则城市被路网切碎无楼可放。
ROAD_SETBACK_ART = 16 * ENV_SCALE;   % 主干 + 立交主线沿线留白（形成清晰廊道，纵向无高楼）
ROAD_SETBACK_COL = 8 * ENV_SCALE;    % 次干道两侧本就该有楼，仅防楼裙楼(podium)压到路幅，留白含典型半宽

% ---- 地形高度场（重庆式：谷地/台地/陡坎，保证全域 >=0 作为 L0 层次）----
if strcmpi(difficulty,'hard')
    baseF = @(x,y) terrAmp*(1.0 + 0.5*sin(x*terrFreq) + 0.4*cos(y*terrFreq*1.1) ...
                 + 0.3*sin((x+y)*terrFreq*0.7)) ...
                 - terrAmp*1.3*exp(-((x/70).^2+(y/80).^2));   % 中央河谷下凹
elseif strcmpi(difficulty,'medium')
    baseF = @(x,y) terrAmp*(0.6 + 0.5*sin(x*terrFreq) + 0.45*cos(y*terrFreq));
else
    baseF = @(x,y) terrAmp*(0.3 + 0.4*sin(x*terrFreq) + 0.35*cos(y*terrFreq));
end
% 平移使最小高度接近 0（建筑基底贴合地形不悬空），钳制非负
[xg,yg] = meshgrid(linspace(-half,half,80));
zminT = min(baseF(xg,yg),[],'all');
terrainF = @(x,y) max(0, baseF(x,y) - zminT);
obs(end+1) = struct('type','terrain','c',[],'r',[],'half',[],'h',[],'grid',[],'gap',[], ...
                    'hmin',[],'hmax',[],'nbx',[],'nby',[],'ball',[],'xz',[],'zlo',[],'zhi',[],'f',terrainF, ...
                    'hw',[],'pod',[],'baseH',[],'bodyH',[],'setback',[],'kind',[],'hue',[],'poly',[],'roof',[],'tier',[], ...
                    'materialId',[],'pole',[],'arm',[],'post',[],'panel',[],'deck',[],'pier',[]);

% ---- 路网（分级立体路网：arterial 十字主轴 + collector 街区网格）----
% 街宽由 class 决定，建筑密度由 BCR 决定，二者解耦（解决"密度不合适"）。
% collector 网格同时定义地块（block），建筑在地块内按 BCR 密铺。
roads = struct('id',{},'class',{},'centerline',{},'width',{},'z_profile',{});
rid = 0;
% arterial：两条十字主轴
arterial = { [-half 0; half 0], [0 -half; 0 half] };
for ai=1:numel(arterial)
    rid = rid+1;
    cl = arterial{ai};
    roads(rid) = struct('id',rid,'class','arterial','centerline',cl,'width',arterialW, ...
                         'z_profile', terrainF(cl(:,1),cl(:,2)));
end
% collector：平行于主轴的网格，围成 nB x nB 地块
bxl = linspace(-half, half, nB+1);   % 街区边界线
for gi=1:nB
    off = bxl(gi);   % 平行于 Y 的纵向 collector
    cl = [-half off; half off];
    rid = rid+1;
    roads(rid) = struct('id',rid,'class','collector','centerline',cl,'width',collectorW, ...
                         'z_profile', terrainF(cl(:,1),cl(:,2)));
    off = bxl(gi);   % 平行于 X 的横向 collector
    cl = [off -half; off half];
    rid = rid+1;
    roads(rid) = struct('id',rid,'class','collector','centerline',cl,'width',collectorW, ...
                         'z_profile', terrainF(cl(:,1),cl(:,2)));
end
% 交叉口（collector 网格交点，含地形标高 z）
junctions = [];
for ix=1:nB+1
    for iy=1:nB+1
        junctions = [junctions; bxl(ix) bxl(iy) terrainF(bxl(ix),bxl(iy))];
    end
end

% ---- 地块（block）划分：相邻 collector 线围成 ----
blocks = [];
for ix=1:nB
    for iy=1:nB
        bxc = (bxl(ix)+bxl(ix+1))/2; byc = (bxl(iy)+bxl(iy+1))/2;
        bw  = (bxl(ix+1)-bxl(ix));   bh = (bxl(iy+1)-bxl(iy));
        blocks = [blocks; bxc byc bw bh];
    end
end

% ===== 立体交通：城市立交（L2 层次，渲染 + 碰撞双用）=====
% 必须放在"建楼"之前，使建楼红线缓冲能一并排除立交沿线高楼
% （用户要求：有公路/立交的地方纵向不应有高楼，避免楼穿桥/立交斜跨穿楼）。
% 在 arterial 主干十字交叉口 (0,0) 建造真实互通式立交：
%   - 沿 X 的 arterial 在交叉口附近抬升为跨线高架（overpass），跨越沿 Y 的地面 arterial；
%   - 沿 Y 的 arterial 保持贴地（已作为地面路绘制）；
%   - 跨线高架两端 + 中点落桥墩（pier 接地形）；
%   - 4 条定向匝道（ramp）从跨线高架两端分别落到地面 Y 向路两侧，形成互通感。
% 立交对象结构复用 bridge（centerline/width/deckZ/pillars），drawBridge 与
% make_bridge_collision 直接复用，保证渲染与碰撞一致。
bridges = struct('id',{},'class',{},'centerline',{},'width',{},'deckZ',{},'pillars',{},'kind',{});
% 交叉口 (0,0) 处地形最高，桥面须高于地面路 + 下方可飞
zCross = terrainF(0,0);
ovHalf = 90*ENV_SCALE;                  % 跨线高架半长（覆盖交叉口 + 引道）
bid = 0;

% ============================================================
% 弧线匝道辅助：二次 Bezier 折线（N 段），从 pA 平滑弯到 pB，
% 控制点在中点法向偏移 side*bow，deckZ 由 [zHi zLo] 沿参数线性插值。
% 返回 cl(N+1 x 2) 与 pillars(K x 3, 接地形)。用于让"该做弧线的匝道"真正呈弧线。
% ============================================================
function [cl, piers] = curvedRamp(pA, pB, zHi, zLo, bow, nSeg, S)
    nSeg = max(6, nSeg);
    mid = (pA + pB)/2;
    dirv = (pB - pA)/norm(pB - pA);
    perpv = [-dirv(2) dirv(1)];
    ctrl = mid + perpv * bow;               % 外凸控制点 -> 平滑弧线
    cl = zeros(nSeg+1, 2);
    for k=0:nSeg
        t = k/nSeg;
        cl(k+1,:) = (1-t)^2*pA + 2*(1-t)*t*ctrl + t^2*pB;
    end
    % 墩：每隔若干段取折线点接地形（至少两端）
    stepP = max(1, floor(nSeg/3));
    idx = unique([1:stepP:nSeg+1, nSeg+1]);
    piers = zeros(numel(idx), 3);
    for j=1:numel(idx)
        x = cl(idx(j),1); y = cl(idx(j),2);
        piers(j,:) = [x y terrainF(x,y)];
    end
end

if ~strcmpi(difficulty,'easy')            % easy 仅平路，无高架/立交
    if strcmpi(difficulty,'medium')
        % ---- medium：单层跨线高架（沿 X）+ 4 条定向匝道（用户要求保持原状）----
        deckZ_main = zCross + 26*ENV_SCALE;
        deckZ_ramp = zCross + 11*ENV_SCALE;
        p1m = [-ovHalf 0]; p2m = [ovHalf 0];
        nPierM = 5;
        piersM = zeros(nPierM,3);
        for pk=1:nPierM
            f = (pk-1)/(nPierM-1);
            px = (1-f)*p1m(1) + f*p2m(1); py = (1-f)*p1m(2) + f*p2m(2);
            piersM(pk,:) = [px py terrainF(px,py)];
        end
        bid = bid + 1;
        bridges(end+1) = struct('id',bid,'class','bridge','centerline',[p1m;p2m], ...
            'width',arterialW,'deckZ',deckZ_main,'pillars',piersM,'kind','overpass');
        rampLen = 55*ENV_SCALE;
        rampPairs = [ -ovHalf,+rampLen; -ovHalf,-rampLen; +ovHalf,+rampLen; +ovHalf,-rampLen ];
        for rk=1:4
            ex = rampPairs(rk,1); ey = rampPairs(rk,2);
            pA = [ex 0]; pB = [0 ey];
            nPierR = 2; piersR = zeros(nPierR,3);
            for pk=1:nPierR
                f = (pk-1)/(nPierR-1);
                px = (1-f)*pA(1) + f*pB(1); py = (1-f)*pA(2) + f*pB(2);
                piersR(pk,:) = [px py terrainF(px,py)];
            end
            bid = bid + 1;
            bridges(end+1) = struct('id',bid,'class','bridge','centerline',[pA;pB], ...
                'width',12*ENV_SCALE,'deckZ',deckZ_ramp,'pillars',piersR,'kind','ramp');
        end
    else
        % ---- hard：多层复式互通立交（定慧桥式）----
        %   双层主线：overpassX 高层 / overpassY 低层；8 条弧线定向匝道；
        %   2 条层间螺旋连接；4 条 270° 盘桥(loop)。所有匝道/连接均为弧线。
        zHigh = zCross + 48*ENV_SCALE;
        zLow  = zCross + 24*ENV_SCALE;
        % 主线 overpassX（沿 X，高层）
        p1x = [-ovHalf 0]; p2x = [ovHalf 0];
        nPx = 5; piersX = zeros(nPx,3);
        for pk=1:nPx
            f = (pk-1)/(nPx-1);
            px = (1-f)*p1x(1)+f*p2x(1); py=(1-f)*p1x(2)+f*p2x(2);
            piersX(pk,:) = [px py terrainF(px,py)];
        end
        bid = bid + 1;
        bridges(end+1) = struct('id',bid,'class','bridge','centerline',[p1x;p2x], ...
            'width',arterialW,'deckZ',zHigh,'pillars',piersX,'kind','overpass');
        % 主线 overpassY（沿 Y，低层）
        p1y = [0 -ovHalf]; p2y = [0 ovHalf];
        nPy = 5; piersY = zeros(nPy,3);
        for pk=1:nPy
            f = (pk-1)/(nPy-1);
            px = (1-f)*p1y(1)+f*p2y(1); py=(1-f)*p1y(2)+f*p2y(2);
            piersY(pk,:) = [px py terrainF(px,py)];
        end
        bid = bid + 1;
        bridges(end+1) = struct('id',bid,'class','bridge','centerline',[p1y;p2y], ...
            'width',arterialW,'deckZ',zLow,'pillars',piersY,'kind','overpass');
        % 4 条弧线匝道：overpassX 两端 -> 地面 Y 路两侧
        rampLen = 60*ENV_SCALE;
        rampPairsX = [ -ovHalf,+rampLen; -ovHalf,-rampLen; +ovHalf,+rampLen; +ovHalf,-rampLen ];
        for rk=1:4
            side = sign(rampPairsX(rk,2));
            [clR, piersR] = curvedRamp([rampPairsX(rk,1) 0], [0 rampPairsX(rk,2)], ...
                zHigh, zCross+6*ENV_SCALE, side*42*ENV_SCALE, 12, ENV_SCALE);
            bid = bid + 1;
            bridges(end+1) = struct('id',bid,'class','bridge','centerline',clR, ...
                'width',12*ENV_SCALE,'deckZ',[zHigh zCross+6*ENV_SCALE],'pillars',piersR,'kind','ramp');
        end
        % 4 条弧线匝道：overpassY 两端 -> 地面 X 路两侧
        rampPairsY = [ +rampLen,-ovHalf; -rampLen,-ovHalf; +rampLen,+ovHalf; -rampLen,+ovHalf ];
        for rk=1:4
            side = sign(rampPairsY(rk,1));
            [clR, piersR] = curvedRamp([0 rampPairsY(rk,2)], [rampPairsY(rk,1) 0], ...
                zLow, zCross+6*ENV_SCALE, side*42*ENV_SCALE, 12, ENV_SCALE);
            bid = bid + 1;
            bridges(end+1) = struct('id',bid,'class','bridge','centerline',clR, ...
                'width',12*ENV_SCALE,'deckZ',[zLow zCross+6*ENV_SCALE],'pillars',piersR,'kind','ramp');
        end
        % 2 条层间螺旋连接：overpassX(高层) 弯下接入 overpassY(低层)
        connPairs = [ -ovHalf,-ovHalf; +ovHalf,+ovHalf ];
        for ck=1:2
            ax_ = connPairs(ck,1); ay_ = connPairs(ck,2);
            [clC, piersC] = curvedRamp([ax_ 0], [0 ay_], zHigh, zLow, ...
                sign(ax_)*38*ENV_SCALE, 12, ENV_SCALE);
            bid = bid + 1;
            bridges(end+1) = struct('id',bid,'class','bridge','centerline',clC, ...
                'width',10*ENV_SCALE,'deckZ',[zHigh zLow],'pillars',piersC,'kind','ramp');
        end
        % 4 条 270° 盘桥(loop)：从高层螺旋下降回到低层，形成定慧桥式环岛
        R_loop = 60*ENV_SCALE; loopSeg = 28;
        loopStart = zHigh; loopEnd = zLow;
        for q=1:4
            ang0 = (q-1)*pi/2 + pi/4;
            cxq = cos(ang0)*ovHalf*0.55; cyq = sin(ang0)*ovHalf*0.55;
            pts = zeros(loopSeg+1,2); piersL = zeros(loopSeg+1,3);
            for k=0:loopSeg
                th = ang0 + k/loopSeg*1.5*pi;          % 270° 螺旋
                x = cxq + R_loop*cos(th); y = cyq + R_loop*sin(th);
                pts(k+1,:) = [x y]; piersL(k+1,:) = [x y terrainF(x,y)];
            end
            bid = bid + 1;
            bridges(end+1) = struct('id',bid,'class','bridge','centerline',pts, ...
                'width',11*ENV_SCALE,'deckZ',[loopStart loopEnd],'pillars',piersL,'kind','loop');
        end
    end
end
% 由渲染桥（city.bridges）单一来源派生碰撞副本，确保几何一致（R3 修复）：
% 渲染与碰撞双用，任一参数变化只在 bridges 一处定义，碰撞副本由工厂函数生成，
% 杜绝"看得见的桥"与"挡得住的桥"几何错位（无人机穿可见桥/被不可见桥挡）。
% 多段折线桥（弧线匝道/盘桥）会在此拆成逐段碰撞盒，渲染与碰撞共用同一几何。
for bi2=1:numel(bridges)
    obs = [obs, make_bridge_collision(bridges(bi2), ENV_SCALE)];
end

% ---- 按 BCR 在地块内密铺建筑（Poisson-disk 风格 + 四带高度分层）----
placedXY = zeros(0,2); placedHW = zeros(0,2); placedB = [];
cumT = cumsum(tierRatio);
for bi=1:size(blocks,1)
    bxc = blocks(bi,1); byc = blocks(bi,2); bw = blocks(bi,3); bh = blocks(bi,4);
    blockArea = bw*bh;
    nTarget = min(6, max(1, floor(blockArea*BCR/AVG_FOOT)));   % 每地块目标栋数（cap 防过载）
    placedInBlock = 0; attempts = 0;
    while placedInBlock < nTarget && attempts < nTarget*40
        attempts = attempts + 1;
        % 地块内均匀抖动采样（留边给街道），保证密集但成片
        cx = bxc + (rand(s,1)*2-1)*bw*0.42;
        cy = byc + (rand(s,1)*2-1)*bh*0.42;
        % 四带高度分层抽样（L/M/H/T）
        ur = rand(s,1);
        if ur < cumT(1)
            tier='L'; topH = hL(1) + rand(s,1)*(hL(2)-hL(1)); kind='residential';
        elseif ur < cumT(2)
            tier='M'; topH = hM(1) + rand(s,1)*(hM(2)-hM(1)); kind='office';
        elseif ur < cumT(3)
            tier='H'; topH = hH(1) + rand(s,1)*(hH(2)-hH(1)); kind='office';
        else
            tier='T'; topH = hT(1) + rand(s,1)*(hT(2)-hT(1)); kind='landmark';
        end
        b = mu_make_bldg(cx, cy, topH, kind, s, ENV_SCALE);
        % 楼层基底贴合地形（zGround），解决"楼悬空/埋地"暗伤：
        %   c(3) 改为实际地面标高，碰撞/渲染均以此为零基准起建。
        zG = terrainF(cx, cy);
        % 楼高上限：保证 zG + topH <= Z_CEIL（空域/搜索空间 z 上限），避免楼顶伸出 bounds
        topH = min(topH, Z_CEIL - zG);
        if topH < 10, continue; end
        b = mu_make_bldg(cx, cy, topH, kind, s, ENV_SCALE);
        % 重叠拒绝：用裙楼半宽 hwP=hw+pod（碰撞/渲染真实盒宽）做 AABB 轴对齐判据，
        % 避免用塔身半宽 hw 导致真实裙楼重叠或走廊被吃掉 pod。
        hwP = b.hw + b.pod;
        okPlace = true;
        if ~isempty(placedXY)
            for q=1:size(placedXY,1)
                dx = abs(cx - placedXY(q,1)) - (hwP(1) + placedHW(q,1)) - CORRIDOR;
                dy = abs(cy - placedXY(q,2)) - (hwP(2) + placedHW(q,2)) - CORRIDOR;
                if dx < 0 && dy < 0, okPlace = false; break; end  % 矩形重叠（轴对齐）
            end
        end
        % 道路红线缓冲：候选点若落入任一道路/立交中心线 half+setback 内则拒绝
        %   （解决楼压路/穿模，保证公路与立交沿线纵向无高楼）。
        %   roads 按 class 定留白；bridges（立交）统一用 arterial 级大留白，
        %   使立交 overpass/ramp 沿线纵向成片无高楼，立交斜跨也不会穿楼。
        if okPlace
            for ri2=1:numel(roads)
                r2 = roads(ri2);
                dR = mu_pt_seg_dist(cx, cy, r2.centerline(1,:), r2.centerline(2,:));
                sb = ROAD_SETBACK_ART; if strcmpi(r2.class,'collector'), sb = ROAD_SETBACK_COL; end
                if dR < r2.width/2 + sb
                    okPlace = false; break;
                end
            end
        end
        if okPlace
            for bi2=1:numel(bridges)
                br2 = bridges(bi2);
                dR = mu_pt_seg_dist(cx, cy, br2.centerline(1,:), br2.centerline(2,:));
                % overpass（主线高架）用 arterial 级大留白形成清晰廊道；
                % ramp（匝道）较窄，留白减小以免把城市切块（匝道沿线仍纵向无高楼）。
                sbB = ROAD_SETBACK_ART; if strcmpi(br2.kind,'ramp'), sbB = 12*ENV_SCALE; end
                if dR < br2.width/2 + sbB
                    okPlace = false; break;
                end
            end
        end
        if ~okPlace, continue; end
        placedInBlock = placedInBlock + 1;
        placedXY = [placedXY; cx cy]; placedHW = [placedHW; hwP];
        matId = mu_tier_material(tier);   % 材质：L/M 混凝土, H/T 玻璃幕墙
        obs(end+1) = struct('type','bldg','c',[cx cy zG],'r',[],'half',[],'h',[],'grid',[],'gap',[], ...
            'hmin',[],'hmax',[],'nbx',[],'nby',[],'ball',[],'xz',[],'zlo',[],'zhi',[],'f',[], ...
            'hw',b.hw,'pod',b.pod,'baseH',b.baseH,'bodyH',b.bodyH,'setback',b.setback, ...
            'kind',b.kind,'hue',b.hue,'poly',b.poly,'roof',b.roof,'tier',tier, ...
            'materialId',matId,'pole',[],'arm',[],'post',[],'panel',[],'deck',[],'pier',[]);
    end
end

% ---- 地标超高层（CBD 焦点塔，明确超高，tier=T）----
% 与地块楼统一做 AABB 重叠拒绝（修复暗伤：地标塔原循环不做间距检查，
% 会与地块楼/彼此重叠），并裁剪楼高使楼顶不超出空域/搜索空间上界。
% Z_CEIL 在顶部参数区定义，此处复用，保证地块楼与地标塔裁剪口径一致。
for ti=1:towers
    % 落在随机地块中心附近，作为城市天际线焦点
    bi = randi(s, size(blocks,1));
    bxc = blocks(bi,1); byc = blocks(bi,2); bw = blocks(bi,3); bh = blocks(bi,4);
    cx = bxc + (rand(s,1)*2-1)*bw*0.30; cy = byc + (rand(s,1)*2-1)*bh*0.30;
    zG = terrainF(cx, cy);
    % 楼高上限：保证 zG + topH <= Z_CEIL
    topHreq = hT(1) + rand(s,1)*(hT(2)-hT(1));
    topH = min(topHreq, Z_CEIL - zG);
    if topH < 10, continue; end
    b = mu_make_bldg(cx, cy, topH, 'landmark', s, ENV_SCALE);
    hwP = b.hw + b.pod;
    % AABB 重叠拒绝（与所有已放置楼）
    okPlace = true;
    if ~isempty(placedXY)
        for q=1:size(placedXY,1)
            dx = abs(cx - placedXY(q,1)) - (hwP(1) + placedHW(q,1)) - CORRIDOR;
            dy = abs(cy - placedXY(q,2)) - (hwP(2) + placedHW(q,2)) - CORRIDOR;
            if dx < 0 && dy < 0, okPlace = false; break; end
        end
    end
    % 道路红线缓冲（与地块楼一致，含立交沿线）
    if okPlace
        for ri2=1:numel(roads)
            r2 = roads(ri2);
            dR = mu_pt_seg_dist(cx, cy, r2.centerline(1,:), r2.centerline(2,:));
            sb = ROAD_SETBACK_ART; if strcmp(r2.class,'collector'), sb = ROAD_SETBACK_COL; end
            if dR < r2.width/2 + sb
                okPlace = false; break;
            end
        end
    end
    if okPlace
        for bi2=1:numel(bridges)
            br2 = bridges(bi2);
            dR = mu_pt_seg_dist(cx, cy, br2.centerline(1,:), br2.centerline(2,:));
            sbB = ROAD_SETBACK_ART; if strcmpi(br2.kind,'ramp'), sbB = 12*ENV_SCALE; end
            if dR < br2.width/2 + sbB
                okPlace = false; break;
            end
        end
    end
    if ~okPlace, continue; end
    obs(end+1) = struct('type','bldg','c',[cx cy zG],'r',[],'half',[],'h',[],'grid',[],'gap',[], ...
        'hmin',[],'hmax',[],'nbx',[],'nby',[],'ball',[],'xz',[],'zlo',[],'zhi',[],'f',[], ...
        'hw',b.hw,'pod',b.pod,'baseH',b.baseH,'bodyH',b.bodyH,'setback',b.setback, ...
        'kind',b.kind,'hue',b.hue,'poly',b.poly,'roof',b.roof,'tier','T', ...
        'materialId','glass','pole',[],'arm',[],'post',[],'panel',[],'deck',[],'pier',[]);
    placedXY = [placedXY; cx cy]; placedHW = [placedHW; hwP];
end

% ---- 通信塔架（tower，工业/基础设施）----
for ti=1:towers
    ang = 2*pi*rand(s,1); rad = half*(0.3+0.6*rand(s,1));
    cx = rad*cos(ang); cy = rad*sin(ang); th = 50+50*rand(s,1);
    zG = terrainF(cx, cy);                       % 塔基贴合地形（与楼一致，修复悬空）
    obs(end+1) = struct('type','tower','c',[cx cy zG],'r',3*ENV_SCALE,'half',[],'h',th,'grid',[],'gap',[], ...
        'hmin',[],'hmax',[],'nbx',[],'nby',[],'ball',8,'xz',[],'zlo',[],'zhi',[],'f',[], ...
        'hw',[],'pod',[],'baseH',[],'bodyH',[],'setback',[],'kind',[],'hue',[],'poly',[],'roof',[],'tier',[], ...
        'materialId','metal','pole',[],'arm',[],'post',[],'panel',[],'deck',[],'pier',[]);
end

% ---- 禁飞区（nofly，机场/政府区）----
for nf=1:nofly
    sx = (rand(s,1)<0.5)*(-2)+1; sy = (rand(s,1)<0.5)*(-2)+1;
    ccx = half*(0.2+0.6*rand(s,1))*sx; ccy = half*(0.2+0.6*rand(s,1))*sy;
    R = 30+25*rand(s,1); M = 7; poly = zeros(M,2); base = 2*pi*rand(s,1);
    for j=1:M
        a = base + 2*pi*(j-1)/M; rr = R*(0.8+0.2*rand(s,1));
        poly(j,:) = [ccx+rr*cos(a), ccy+rr*sin(a)];
    end
    obs(end+1) = struct('type','nofly','c',[],'r',[],'half',[],'h',[],'grid',[],'gap',[], ...
        'hmin',[],'hmax',[],'nbx',[],'nby',[],'ball',[],'xz',poly,'zlo',20*ENV_SCALE,'zhi',140*ENV_SCALE,'f',[], ...
        'hw',[],'pod',[],'baseH',[],'bodyH',[],'setback',[],'kind',[],'hue',[],'poly',[],'roof',[],'tier',[], ...
        'materialId',[],'pole',[],'arm',[],'post',[],'panel',[],'deck',[],'pier',[]);
end

% ---- 低矮植被（tree，软障碍，公园/街道绿化）----
for tr=1:trees
    cx = -half+2*half*rand(s,1); cy = -half+2*half*rand(s,1); th = 6+8*rand(s,1);
    zG = terrainF(cx, cy);                       % 树根贴合地形曲面（修复"树没在曲面上"）
    obs(end+1) = struct('type','tree','c',[cx cy zG],'r',(3+2*rand(s,1))*ENV_SCALE,'half',[],'h',th,'grid',[],'gap',[], ...
        'hmin',[],'hmax',[],'nbx',[],'nby',[],'ball',[],'xz',[],'zlo',[],'zhi',[],'f',[], ...
        'hw',[],'pod',[],'baseH',[],'bodyH',[],'setback',[],'kind',[],'hue',[],'poly',[],'roof',[],'tier',[], ...
        'materialId','vegetation','pole',[],'arm',[],'post',[],'panel',[],'deck',[],'pier',[]);
end

% ---- 水体（water，沿河谷/湖，地形下凹 + 浅蓝面，无楼）----
if water
    obs(end+1) = struct('type','water','c',[],'r',[],'half',[],'h',[],'grid',[],'gap',[], ...
        'hmin',[],'hmax',[],'nbx',[],'nby',[],'ball',[],'xz',[ -half*0.2 -half*0.5; ...
         half*0.1 -half*0.6; half*0.3 -half*0.2; half*0.0 half*0.1; -half*0.3 half*0.2], ...
        'zlo',[],'zhi',[],'f',[],'hw',[],'pod',[],'baseH',[],'bodyH',[],'setback',[],'kind',[],'hue',[],'poly',[],'roof',[],'tier',[], ...
        'materialId','water','pole',[],'arm',[],'post',[],'panel',[],'deck',[],'pier',[]);
end

% ---- 基础设施：路灯（streetlight，沿 collector/local 路中线成排）+ 交通标志（sign，交叉口）----
% 沿 collector 网格线布灯：每侧每隔约 30m 一根，杆高 ~12m，灯臂水平悬挑 2m（暖橙灯具）。
% sign 放在交叉口附近，立柱 + 牌面（小 box）。
LAMP_STEP = 34*ENV_SCALE;       % 路灯间距 (m)（随环境放大）
LAMP_H = 12*ENV_SCALE;          % 灯杆高 (m)
LAMP_R = 0.8*ENV_SCALE;         % 灯杆半径 (m)
ARM_LEN = 2.2*ENV_SCALE;        % 灯臂悬挑 (m)
for gi=1:nB
    off = bxl(gi);
    % 平行于 Y 的纵向 collector（x=off）
    for yy = -half+LAMP_STEP : LAMP_STEP : half-LAMP_STEP
        comp = off + collectorW/2 - 2;        % 路缘靠建筑侧
        cc = [comp yy terrainF(comp, yy)];     % 灯杆底心贴合地形
        obs(end+1) = make_streetlight(cc, LAMP_H, LAMP_R, ARM_LEN, [1 0 0], s);
    end
    % 平行于 X 的横向 collector（y=off）
    for xx = -half+LAMP_STEP : LAMP_STEP : half-LAMP_STEP
        comp = off + collectorW/2 - 2;
        cc = [xx comp terrainF(xx, comp)];     % 灯杆底心贴合地形
        obs(end+1) = make_streetlight(cc, LAMP_H, LAMP_R, ARM_LEN, [0 1 0], s);
    end
end
% 交通标志：交叉口附近随机若干（难度越高越多）
nSign = 4 + 4*(strcmpi(difficulty,'hard')>0) + 2*(strcmpi(difficulty,'medium')>0);
for sg=1:nSign
    ji = randi(s, size(junctions,1));
    jx = junctions(ji,1); jy = junctions(ji,2); jz = junctions(ji,3);
    offd = (rand(s,1)<0.5)*(-1)+1;
    sc = [jx + offd*(collectorW/2+1) jy + (rand(s,1)*2-1)*3 jz];
    sc(3) = terrainF(sc(1), sc(2));            % 标志立柱底心贴合地形
    obs(end+1) = make_sign(sc, s, ENV_SCALE);
end

% ---- 空域分层参数（L3：低空街道峡谷带 / 中层可飞带 / 高空走廊）----
% Z 边界随 ENV_SCALE 放大（与 Z_CEIL / bounds(6) 一致）
airspace = struct('low', struct('zlo',0,'zhi',70*ENV_SCALE), ...
                  'mid', struct('zlo',70*ENV_SCALE,'zhi',130*ENV_SCALE), ...
                  'high',struct('zlo',130*ENV_SCALE,'zhi',180*ENV_SCALE));

% ---- 材质查找表（阶段B 补全：介电常数/反射率/RGB）----
materials = struct();
materials.concrete  = struct('permittivity',6.4,'reflect',0.35,'rgb',[0.72 0.72 0.75]);
materials.glass     = struct('permittivity',2.3,'reflect',0.65,'rgb',[0.55 0.72 0.82]);
materials.metal     = struct('permittivity',1.0,'reflect',0.85,'rgb',[0.60 0.63 0.68]);
materials.asphalt   = struct('permittivity',3.5,'reflect',0.20,'rgb',[0.30 0.32 0.35]);
materials.water     = struct('permittivity',80 ,'reflect',0.10,'rgb',[0.20 0.45 0.62]);
materials.vegetation= struct('permittivity',15 ,'reflect',0.30,'rgb',[0.40 0.58 0.36]);
materials.bridgedeck= struct('permittivity',5.0,'reflect',0.45,'rgb',[0.66 0.66 0.70]);

% ---- 环境参考系（阶段C 3.1：局部笛卡尔 ↔ 经纬度 Helmert 2D + 高程偏移）----
% 锚点取重庆解放碑一带（WGS84），局部原点 = 城市中心，Z 向上（米）。
geoRef = struct('datum','WGS84', ...
                'originLat', 29.5630, 'originLon', 106.5780, 'originAlt', 250.0, ...
                'heading', 0.0, ...                 % 局部 X 轴相对正北的偏航（度）
                'k', 1.0);                          % 尺度因子（平面投影近似）

% ---- 大气条件（阶段C 3.10：温度/湿度/气压/风/能见度）----
% 难度驱动天气：easy 晴昼、medium 多云多变、hard 雾/黄昏。
if strcmpi(difficulty,'easy')
    atmosphere = struct('tempC',24,'humidity',0.55,'pressure',1002, ...
        'wind',[1.5 0.8 0.0],'visibility',12000);
elseif strcmpi(difficulty,'medium')
    atmosphere = struct('tempC',21,'humidity',0.68,'pressure',1000, ...
        'wind',[3.0 1.5 0.0],'visibility',8000);
else
    atmosphere = struct('tempC',18,'humidity',0.82,'pressure',998, ...
        'wind',[4.5 2.5 0.0],'visibility',2500);   % 雾：低能见度
end

% ---- 环境动态（阶段C 3.11：光照/天气驱动渲染）----
% timeOfDay 决定太阳方位/仰角；weather 影响背景/可见度。
if strcmpi(difficulty,'easy')
    envdyn = struct('timeOfDay',12.0,'weather','clear', ...
        'sunAz',135,'sunEl',72,'skyTint',[1 1 1]);
elseif strcmpi(difficulty,'medium')
    envdyn = struct('timeOfDay',16.0,'weather','cloudy', ...
        'sunAz',255,'sunEl',35,'skyTint',[0.93 0.94 0.96]);
else
    envdyn = struct('timeOfDay',18.0,'weather','fog', ...
        'sunAz',282,'sunEl',12,'skyTint',[0.82 0.83 0.85]);
end

dynamics = make_dynamics(roads, junctions, terrainF, difficulty, s);

% ---- 阶段E：通信/传感器基础设施（comms + sensors）----
% comms 网格：基站(gNB) / 中继(relay) / 终端(iot) 三维节点，含射频与覆盖半径，
% 供渲染（天线+覆盖球）与链路约束（w.comms：UAV 轨迹须落在任一节点覆盖内）。
% sensors 元数据：相机/气象/噪声监测点，挂载于建筑/杆塔/专用杆，供渲染与感知接入标记。
[comms, sensors] = make_comms_sensors(obs, towers, terrainF, difficulty, s, ENV_SCALE);

city = struct('obstacles', obs, 'terrainF', terrainF, 'roads', roads, ...
              'junctions', junctions, 'bridges', bridges, 'airspace', airspace, ...
              'materials', materials, 'geoRef', geoRef, ...
              'atmosphere', atmosphere, 'envdyn', envdyn, 'dynamics', dynamics, ...
              'comms', comms, 'sensors', sensors);
end

% ============ 阶段E：通信网络 + 传感器元数据 ============
function [comms, sensors] = make_comms_sensors(obs, towers, terrainF, difficulty, s, ENV_SCALE)
% 生成通信基础设施节点（comms）与传感器挂载点（sensors）。
% comms 节点字段：
%   type  : 'gNB'(宏基站) | 'relay'(mesh 中继) | 'iot'(终端/感知节点)
%   c     : [x y z] 三维位置（z 为天线挂高）
%   freq  : 主频 (GHz)
%   bw    : 带宽 (MHz)
%   ptx   : 发射功率 (dBm)
%   antH  : 天线高度 (m)
%   covR  : 地面覆盖半径 (m)，由发射功率/频段与难度（遮挡/天气）推导
%   node  : 节点编号
%   apId  : 接入点标识（同频组网单元）
% sensors 字段：
%   type  : 'cam' | 'lidar' | 'met' | 'noise'
%   c     : [x y z] 位置
%   range : 感知半径 (m)
%   mount : 挂载对象描述
%   res   : 分辨率/精度标识

% 节点规模与特性按难度：难度越高（密集老城）遮挡越强、基站更密、覆盖半径更小。
if strcmpi(difficulty,'easy')
    nGNB = 4;  nRelay = 3;  nIoT = 8;
    covR_gnb = 320;  covR_relay = 140;
    weatherK = 1.0;                    % 覆盖折减因子（晴昼）
elseif strcmpi(difficulty,'hard')
    nGNB = 8;  nRelay = 7;  nIoT = 16;
    covR_gnb = 180;  covR_relay = 90;
    weatherK = 0.62;                   % 雾/密集遮挡显著折减
else
    nGNB = 6;  nRelay = 5;  nIoT = 12;
    covR_gnb = 240;  covR_relay = 110;
    weatherK = 0.82;                   % 多云轻微折减
end

half = 200 * ENV_SCALE;               % 与全城搜索空间一致 [-200*S, 200*S]
freq_gnb   = 3.5;  bw_gnb  = 100;  ptx_gnb  = 46;   % 3.5GHz 宏站 100MHz 40W
freq_relay = 5.8;  bw_relay= 80;  ptx_relay= 30;   % 5.8GHz 中继 mesh
freq_iot   = 2.4;  bw_iot  = 20;  ptx_iot  = 20;   % 2.4GHz 终端

% 找楼宇与杆塔位置（用于基站/传感器挂载；无则随机散布）
% bldgTop 取楼实际顶高 = c(3)+baseH+bodyH（贴合地形后的真实楼顶，修复暗伤：
%   之前基站挂高用 zGround+antH，等于把天线挂到楼底基座而非楼顶）。
bldgXY = [];  bldgTop = [];  towerXY = [];
for oi=1:numel(obs)
    if strcmp(obs(oi).type,'bldg') && ~isempty(obs(oi).hw)
        bldgXY = [bldgXY; obs(oi).c(1:2)];
        bldgTop = [bldgTop; obs(oi).c(3) + obs(oi).baseH + obs(oi).bodyH];
    elseif strcmp(obs(oi).type,'tower')
        towerXY = [towerXY; obs(oi).c(1:2)];
    end
end

comms = struct('type','','c',[0 0 0],'freq',0,'bw',0,'ptx',0,'antH',0,'covR',0,'node',0,'apId',0);
comms(1) = [];
nodeId = 0;

% ---- 宏基站 gNB：优先挂载于地标塔/高楼顶，否则地面杆 ----
for gi=1:nGNB
    nodeId = nodeId + 1;
    if ~isempty(towerXY) && rand(s,1)<0.5
        xy = towerXY(randi(s,size(towerXY,1)),:);
        zg = terrainF(xy(1),xy(2));
        antH = 35 + 15*rand(s,1);      % 塔顶加挂
        c = [xy zg + antH];
    elseif ~isempty(bldgXY)
        bi = randi(s,size(bldgXY,1));
        xy = bldgXY(bi,:);
        ztop = bldgTop(bi);            % 楼实际顶高（已含地形）
        antH = 25 + 20*rand(s,1);
        c = [xy ztop + antH];          % 挂楼顶，而非楼基座
    else
        xy = (rand(s,1)*2-1)*half*0.8*[1 1];
        zg = terrainF(xy(1),xy(2));
        antH = 30; c = [xy zg + antH];
    end
    covR = covR_gnb * weatherK * (0.85 + 0.3*rand(s,1));
    comms(end+1) = struct('type','gNB','c',c,'freq',freq_gnb,'bw',bw_gnb, ...
        'ptx',ptx_gnb,'antH',antH,'covR',covR,'node',nodeId,'apId',mod(nodeId-1,3)+1);
end

% ---- 中继 relay：沿街区网格散布，弥补盲区（mesh 自组网） ----
for ri=1:nRelay
    nodeId = nodeId + 1;
    xy = (rand(s,1)*2-1)*half*0.85*[1 1];
    zg = terrainF(xy(1),xy(2));
    antH = 15 + 10*rand(s,1);
    covR = covR_relay * weatherK * (0.85 + 0.3*rand(s,1));
    comms(end+1) = struct('type','relay','c',[xy zg+antH],'freq',freq_relay,'bw',bw_relay, ...
        'ptx',ptx_relay,'antH',antH,'covR',covR,'node',nodeId,'apId',mod(nodeId-1,3)+1);
end

% ---- 终端/感知节点 iot：贴近地面，低功率广覆盖末端 ----
for ii=1:nIoT
    nodeId = nodeId + 1;
    xy = (rand(s,1)*2-1)*half*0.9*[1 1];
    zg = terrainF(xy(1),xy(2));
    antH = 6 + 6*rand(s,1);
    covR = 60 * weatherK * (0.8 + 0.4*rand(s,1));
    comms(end+1) = struct('type','iot','c',[xy zg+antH],'freq',freq_iot,'bw',bw_iot, ...
        'ptx',ptx_iot,'antH',antH,'covR',covR,'node',nodeId,'apId',mod(nodeId-1,4)+1);
end

% ---- 传感器挂载点 sensors（元数据，渲染标记 + 感知接入）----
sensors = struct('type','','c',[0 0 0],'range',0,'mount','','res','');
sensors(1) = [];
% 高空广域相机：挂于地标塔/高楼顶，俯瞰城区
for si=1:min(4, max(1, floor(numel(bldgXY)/40)))
    if isempty(bldgXY), break; end
    xy = bldgXY(randi(s,size(bldgXY,1)),:);
    zg = terrainF(xy(1),xy(2));
    sensors(end+1) = struct('type','cam','c',[xy zg+40],'range',500,'mount','rooftop','res','4K');
end
% 气象站：固定地面点
for si=1:3
    xy = (rand(s,1)*2-1)*half*0.8*[1 1];
    zg = terrainF(xy(1),xy(2));
    sensors(end+1) = struct('type','met','c',[xy zg+8],'range',0,'mount','ground','res','1Hz');
end
% 噪声监测：贴近道路交叉口
for si=1:4
    xy = (rand(s,1)*2-1)*half*0.9*[1 1];
    zg = terrainF(xy(1),xy(2));
    sensors(end+1) = struct('type','noise','c',[xy zg+4],'range',120,'mount','pole','res','60dB');
end
% LiDAR 感知塔：挂于通信塔
for si=1:min(2, max(0, towers))
    if isempty(towerXY), break; end
    xy = towerXY(randi(s,size(towerXY,1)),:);
    zg = terrainF(xy(1),xy(2));
    sensors(end+1) = struct('type','lidar','c',[xy zg+30],'range',300,'mount','tower','res','0.1deg');
end
end

% ============ 阶段D：道路动态交通（时变障碍） ============
function dyn = make_dynamics(roads, junctions, terrainF, difficulty, s)
% 沿分级路网（arterial / collector）生成地面车辆动态层。
% 每条 road 按长度布设若干 vehicle，沿中心线以固定速度循环行驶。
% 返回 dyn 结构：dyn.vehicles = struct 数组；dyn.count；dyn.roadLen；
%   dyn.vmax（参考最高速）；dyn.tSpan（循环周期上限）。
% 每个 vehicle 字段：
%   roadId  : 所属道路 id
%   cls     : 'arterial' | 'collector'
%   s0      : 初始沿路距 [0, roadLen]（米）
%   speed   : 速度 (m/s，正方向沿 centerline 起点->终点)
%   length  : 车长 (m)
%   width   : 车宽 (m)
%   height  : 车高 (m)
%   dir     : +1 / -1（行驶方向，反向车走对侧车道，由 lateral 偏移体现）
%   lateral : 车道横向偏移（米，左/右各一车道）
%   phi     : 航向角（绕 Z，由 centerline 切线决定，预先计算）
%   z       : 地面高度（中心线上地形标高）
% 位置函数（时变）：s(t) = mod(s0 + speed*t, roadLen)，映射回 xy 由
%   mu_obstacle_dist_t 调用 mu_road_xy(dyn, vi, t) 完成。

% 车辆密度与速度按难度
if strcmpi(difficulty,'easy')
    perKm = 2.2; vmax = 14;
elseif strcmpi(difficulty,'hard')
    perKm = 6.0; vmax = 18;
else
    perKm = 4.0; vmax = 16;
end

vehicles = struct('roadId',0,'cls','','s0',0,'speed',0,'length',0, ...
                  'width',0,'height',0,'dir',0,'lateral',0,'phi',0,'z',0, ...
                  'cl',[0 0;0 0],'roadLen',0,'lateralV',[0 0]);
vehicles(1) = [];   % 清空占位，得到 0×1 结构数组，可直接 end+1 追加
roadLen = [];  % 每条 road 长度（与 roads 同序的临时）

nRoad = numel(roads);
for ri=1:nRoad
    cl = roads(ri).centerline;          % 2 x 2 端点
    seg = cl(2,:) - cl(1,:);
    L = norm(seg);
    if L < 1, continue; end
    tang = seg / L;                      % 单位切线
    phi = atan2(tang(2), tang(1));       % 航向角
    % 车道横向单位向量（垂直于切线，水平面内）
    lateralV = [-tang(2) tang(1)];
    % 该 road 车辆数
    nV = max(1, round(L/1000 * perKm));
    % 两车道：dir=+1 走 +lateralV 侧，dir=-1 走 -lateralV 侧
    for vi=1:nV
        dir = (mod(vi,2)==1) * 2 - 1;    % +1 / -1
        s0 = (vi-1) * (L/nV) + rand(s,1)* (L/nV)*0.5;
        spd = vmax * (0.55 + 0.45*rand(s,1));   % 速度抖动
        latOff = dir * (roads(ri).width/4 + 1.6); % 车道偏置（靠右行驶）
        % 车辆位于车道中心：xy = centerline(s) + latOff*lateralV
        % z 取中心线该处地形标高（近似：线性插值两端）
        z0 = roads(ri).z_profile(1); z1 = roads(ri).z_profile(2);
        vehicles(end+1) = struct( ...
            'roadId', ri, 'cls', roads(ri).class, ...
            's0', s0, 'speed', spd, 'length', 4.6, 'width', 1.9, 'height', 1.5, ...
            'dir', dir, 'lateral', latOff, 'phi', phi, 'z', [z0 z1], ...
            'cl', cl, 'roadLen', L, 'lateralV', lateralV);
    end
    roadLen(ri) = L;
end

% 交叉口右转/环流车辆（少量，体现节点动态）：在 junction 附近低速绕行
% 简化：在若干 junction 加 1 辆慢速车，作为 point-like 障碍（dir=0 原地微动）
nJ = size(junctions,1);
nJdyn = min(6, nJ);
for ji=1:nJdyn
    j = junctions(randi(s,nJ),:);
    vehicles(end+1) = struct( ...
        'roadId', -ji, 'cls', 'junction', ...
        's0', 0, 'speed', 2.0, 'length', 4.0, 'width', 1.9, 'height', 1.5, ...
        'dir', 0, 'lateral', 0, 'phi', 2*pi*rand(s,1), 'z', j(3), ...
        'cl', [j(1:2); j(1:2)], 'roadLen', 6.0, 'lateralV', [0 0]);
end

dyn = struct('vehicles', vehicles, 'count', numel(vehicles), ...
             'roadLen', roadLen, 'vmax', vmax, 'perKm', perKm, ...
             'tSpan', 220);   % 动画参考周期（秒）
end

% ============ 阶段B 辅助：路灯 / 交通标志 / 材质映射 ============
function o = make_streetlight(c, H, r, armLen, dirv, s)
% 路灯：圆柱杆 (pole) + 水平悬挑灯臂 box (arm，暖橙灯具占位)
%   c     : 灯杆底心 [x y z0]
%   dirv  : 灯臂指向（单位向量，[1 0 0] 或 [0 1 0]）
perp = [0 0 1];   % 灯臂沿竖直方向无效，这里 arm 仅水平悬挑
Rarm = [dirv(1) 0 0; dirv(2) 0 0; 0 0 1];   % 使 box 长边对齐 dirv
armC = [c(1)+dirv(1)*armLen/2, c(2)+dirv(2)*armLen/2, c(3)+H];
armHW = [armLen/2*abs(dirv(1))+0.15, armLen/2*abs(dirv(2))+0.15, 0.2];   % 悬挑长对应轴
o = struct('type','streetlight','c',c,'r',[],'half',[],'h',[],'grid',[],'gap',[], ...
    'hmin',[],'hmax',[],'nbx',[],'nby',[],'ball',[],'xz',[],'zlo',[],'zhi',[],'f',[], ...
    'hw',[],'pod',[],'baseH',[],'bodyH',[],'setback',[],'kind',[],'hue',[],'poly',[],'roof',[],'tier',[], ...
    'materialId','metal','pole',struct('c',c,'r',r,'h',H),'arm',[],'post',[],'panel',[],'deck',[],'pier',[]);
o.arm = struct('c',armC,'R',Rarm,'hw',armHW);
end

function o = make_sign(c, s, ENV_SCALE)
% 交通标志：立柱 box (post) + 牌面 box (panel，悬于柱顶)
postH = 3.2*ENV_SCALE; postHW = [0.15*ENV_SCALE 0.15*ENV_SCALE 1.6*ENV_SCALE];  % 立柱半高 1.6 -> 高 3.2
panelC = [c(1) c(2) c(3)+postH+0.6*ENV_SCALE]; panelHW = [1.6*ENV_SCALE 0.1*ENV_SCALE 0.6*ENV_SCALE]; % 牌面 3.2m 宽 x 1.2m 高
o = struct('type','sign','c',c,'r',[],'half',[],'h',[],'grid',[],'gap',[], ...
    'hmin',[],'hmax',[],'nbx',[],'nby',[],'ball',[],'xz',[],'zlo',[],'zhi',[],'f',[], ...
    'hw',[],'pod',[],'baseH',[],'bodyH',[],'setback',[],'kind',[],'hue',[],'poly',[],'roof',[],'tier',[], ...
    'materialId','metal','pole',[],'arm',[],'post',struct('c',[c(1) c(2) c(3)+postH/2],'hw',postHW), ...
    'panel',struct('c',panelC,'hw',panelHW),'deck',[],'pier',[]);
end

function mid = mu_tier_material(tier)
% tier -> 建筑材质：低层/中层混凝土，高层/地标玻璃幕墙
if strcmpi(tier,'L') || strcmpi(tier,'M')
    mid = 'concrete';
else
    mid = 'glass';
end
end

% ============ 阶段L2：由渲染桥派生碰撞副本（单一几何来源，R3 修复） ============
function o = make_bridge_collision(br, ENV_SCALE)
% 输入 br : city.bridges 渲染结构（含 centerline / deckZ / pillars）
% 输出 o  : obs 中 type='bridge' 的碰撞障碍副本（deck 旋转 box + pier 竖直圆柱）
%   几何完全由 br 推导，与 mu_draw_scene.drawBridge 渲染共用同一组参数，避免双写错位。
%   多段折线桥（弧线匝道/盘桥）：centerline 为 Nx2 (N>2)，返回 1x(N-1) obs 数组，
%   每段一个 deck box + 该段两端桥墩，deckZ 沿段线性插值。
cl = br.centerline; n = size(cl,1);
if n == 2
    % ---- 单段直线桥（overpass / medium 原逻辑）----
    p1 = cl(1,:); p2 = cl(2,:);
    deckZ = br.deckZ;
    dirv = (p2 - p1)/norm(p2 - p1);
    perp = [-dirv(2) dirv(1)];
    R = [dirv(1) dirv(2) 0; perp(1) perp(2) 0; 0 0 1];
    deckHW = [norm(p2-p1)/2, br.width/2, 1.6*ENV_SCALE];
    deckC = [ (p1(1)+p2(1))/2 (p1(2)+p2(2))/2 deckZ ];
    pcols = br.pillars;
    piers = struct('c',[],'r',[],'z1',[]);
    for pk=1:size(pcols,1)
        pp = pcols(pk,:);
        piers(pk) = struct('c',[pp(1) pp(2) 0],'r',2.5*ENV_SCALE,'z1',deckZ);
    end
    o = struct('type','bridge','c',deckC,'r',[],'half',[],'h',[],'grid',[],'gap',[], ...
        'hmin',[],'hmax',[],'nbx',[],'nby',[],'ball',[],'xz',[],'zlo',[],'zhi',[],'f',[], ...
        'hw',[],'pod',[],'baseH',[],'bodyH',[],'setback',[],'kind',[],'hue',[],'poly',[],'roof',[],'tier',[], ...
        'materialId','bridgedeck','pole',[],'arm',[],'post',[],'panel',[], ...
        'deck',struct('c',deckC,'R',R,'hw',deckHW),'pier',piers);
else
    % ---- 多段折线桥：逐段拆分 ----
    dZ = br.deckZ;
    if isscalar(dZ), zA = dZ; zB = dZ; else zA = dZ(1); zB = dZ(end); end
    tmp = {};
    for i=1:n-1
        p1 = cl(i,:); p2 = cl(i+1,:);
        tmid = (i-0.5)/(n-1);
        zSeg = zA + (zB - zA)*tmid;            % 沿桥长线性插值 deckZ
        dirv = (p2 - p1)/norm(p2 - p1);
        perp = [-dirv(2) dirv(1)];
        R = [dirv(1) dirv(2) 0; perp(1) perp(2) 0; 0 0 1];
        deckHW = [norm(p2-p1)/2, br.width/2, 1.6*ENV_SCALE];
        deckC = [ (p1(1)+p2(1))/2 (p1(2)+p2(2))/2 zSeg ];
        % 该段两端桥墩（接地形）
        piers = struct('c',[],'r',[],'z1',[]);
        piers(1) = struct('c',[p1(1) p1(2) 0],'r',2.5*ENV_SCALE,'z1',zSeg);
        piers(2) = struct('c',[p2(1) p2(2) 0],'r',2.5*ENV_SCALE,'z1',zSeg);
        tmp{end+1} = struct('type','bridge','c',deckC,'r',[],'half',[],'h',[],'grid',[],'gap',[], ...
            'hmin',[],'hmax',[],'nbx',[],'nby',[],'ball',[],'xz',[],'zlo',[],'zhi',[],'f',[], ...
            'hw',[],'pod',[],'baseH',[],'bodyH',[],'setback',[],'kind',[],'hue',[],'poly',[],'roof',[],'tier',[], ...
            'materialId','bridgedeck','pole',[],'arm',[],'post',[],'panel',[], ...
            'deck',struct('c',deckC,'R',R,'hw',deckHW),'pier',piers);
    end
    o = [tmp{:}];
end
end

% ============ 单栋 rich-building 生成（随 mu_city_layout 一并迁移） ============
function b = mu_make_bldg(cx, cy, topH, kind, s, ENV_SCALE)
% 生成单栋楼宇差异化参数：矩形/L形 foot、真实裙楼(podium)、退台塔身、楼顶设备、色相抖动
%   topH : 总高（米）
%   kind : 'residential'|'office'|'landmark'|'civic'
% 返回 struct 字段：
%   hw(1x2)   塔身半宽 [hwx hwy]
%   pod(1x2)  裙楼相对塔身的外扩半宽 [px py]（裙楼半宽 = hw + pod）
%   baseH     裙楼高（米）
%   bodyH     塔身高 = topH - baseH
%   setback   退台收进比例（塔身顶部再收进）
%   kind      类型
%   hue       明度抖动
%   poly      L 形 foot 相对中心 Nx2（空=矩形）；L 形时 hw 视为外接半宽，pod 同上
%   roof      楼顶设备配置 struct(have,ant)

% 平面体量（矩形或 L 形外接半宽）：宽深独立随机，比例 0.6~1.6
% ENV_SCALE：楼平面半宽随环境放大（与楼高同尺度），保持楼"瘦高比"不变
footScale = 0.55 + 0.5*rand(s,1);
hwx = ENV_SCALE * footScale * (0.8 + 0.8*rand(s,1));
hwy = ENV_SCALE * footScale * (0.8 + 0.8*rand(s,1));
% 裙楼：底部 2~4 层，平面比塔身更宽（podium 外扩 2~5m）
baseLayers = 2 + floor(3*rand(s,1));        % 2~4 层裙楼
baseH = baseLayers * 4.0 * ENV_SCALE;       % 层高 ~4m（随环境放大）
bodyH = max(6, topH - baseH);
pod = ENV_SCALE * [2 + 3*rand(s,1), 2 + 3*rand(s,1)];  % 裙楼外扩（随环境放大）
setback = 0.08 + 0.18*rand(s,1);            % 塔身退台收进 8%~26%

% 可选 L 形（约 25% 概率）：在主矩形一角切掉小矩形。
% cutFrac 为切角比例，输出为字段供碰撞(mu_lshape_dist)与渲染(drawLShape)共用，
% 消除两处硬编码 0.2 的隐性耦合（任一处不同步都会造成碰撞/渲染错位）。
cutFrac = 0.2;
if rand(s,1) < 0.25
    poly = [ -hwx -hwy; hwx -hwy; hwx hwy*cutFrac; hwx*cutFrac hwy*cutFrac; hwx*cutFrac hwy; -hwx hwy ];
    hasL = true;
else
    poly = [];
    hasL = false;
end

% 楼顶设备：机房小盒（概率）+ 天线细杆（概率），随 seed 可复现
roof.have = rand(s,1) < 0.85;
roof.ant  = rand(s,1) < 0.5;

% 色相抖动（明度 ±0.06）
hue = (rand(s,1)*2-1)*0.06;
b = struct('hw',[hwx hwy],'pod',pod,'baseH',baseH,'bodyH',bodyH,'setback',setback, ...
           'kind',kind,'hue',hue,'poly',poly,'roof',roof,'cutFrac',cutFrac,'hasL',hasL);
end

% ============ 点到线段最近距离（XY 平面，用于道路红线缓冲） ============
function d = mu_pt_seg_dist(px, py, p1, p2)
% 点 (px,py) 到线段 p1-p2（均 Nx2 或 1x2）的最近欧氏距离（逐行）。
% 用于判断楼候选点是否落入某条道路红线（中心线 half + setback）。
ax = p1(:,1); ay = p1(:,2); bx = p2(:,1); by = p2(:,2);
dx = bx - ax; dy = by - ay;
L2 = dx.^2 + dy.^2;
% 参数 t = clamp(((P-A)·(B-A)) / |B-A|^2, 0, 1)
if isscalar(L2) && L2 < 1e-9
    t = 0;
else
    t = ((px - ax).*dx + (py - ay).*dy) ./ max(L2, 1e-9);
    t = max(0, min(1, t));
end
cx = ax + t.*dx; cy = ay + t.*dy;
d = sqrt((px - cx).^2 + (py - cy).^2);
end
