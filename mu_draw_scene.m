function mu_draw_scene(scene, trajs, mode, varargin)
% mu_draw_scene - clean white-background 3D visualization for urban UAV delivery
%   mu_draw_scene(scene, trajs, mode)           uses current/ new figure
%   mu_draw_scene(scene, trajs, mode, ax)       draws into given axes (GUI)
%
% Visual design: white background + light floor grid + realistic city obstacles:
% buildings (cool grey-blue, height-graded, window texture), comm towers,
% no-fly zones (semi-transparent red), trees (soft), terrain surface (light).
% Trajectories drawn as along-path gradient ribbons with dark cores.

% ---------- parse args ----------
if nargin >= 4 && isa(varargin{1},'matlab.graphics.axis.Axes')
    ax = varargin{1};
    isGUI = isa(ax,'matlab.ui.control.UIAxes');
else
    ax = gca;
    isGUI = false;
end

% ---------- palette (white theme) ----------
BG        = [1 1 1];
GRID_COL  = [0.88 0.90 0.93];
BUILDING  = [0.55 0.62 0.72];   % cool grey-blue
BUILD_EDG = [0.35 0.43 0.55];
HAZARD    = [0.88 0.42 0.30];   % warm accent (no-fly)
HAZARD_ED = [0.72 0.25 0.18];
UAV_COLORS = [0.90 0.10 0.10; 0.00 0.20 0.90; 0.00 0.70 0.20; 0.85 0.00 0.70; ...
              1.00 0.50 0.00; 0.00 0.70 0.80; 0.50 0.00 0.90; 0.95 0.85 0.00];

b = scene.bounds;
terrainF = [];
if isfield(scene,'terrainF'), terrainF = scene.terrainF; end
ax.XLim = [b(1) b(2)]; ax.YLim = [b(3) b(4)]; ax.ZLim = [b(5) b(6)];

% ---------- background and grid ----------
if ~isGUI
    fig = ancestor(ax,'figure');
    if ~isempty(fig), fig.Color = BG; end
    ax.Color = BG;
else
    ax.Color = BG;
end
ax.GridColor = GRID_COL; ax.GridAlpha = 1;
ax.XColor = [0.35 0.40 0.48]; ax.YColor = [0.35 0.40 0.48]; ax.ZColor = [0.35 0.40 0.48];
ax.LineWidth = 0.8;
ax.Box = 'on'; ax.BoxStyle = 'full';
grid(ax,'on');

% floor grid lines (below zmin to avoid z-fighting with building bases)
hold(ax,'on');
gridZ = b(5) - 0.5;
gx = linspace(b(1),b(2),25); gy = linspace(b(3),b(4),25);
for i=1:numel(gx)
    plot3(ax,[gx(i) gx(i)],[b(3) b(4)],[gridZ gridZ],'Color',GRID_COL,'LineWidth',0.45);
end
for j=1:numel(gy)
    plot3(ax,[b(1) b(2)],[gy(j) gy(j)],[gridZ gridZ],'Color',GRID_COL,'LineWidth',0.45);
end

% ---------- 地面公路网（道路作为地面层，灰带贴地；高架桥见 bridges 循环）----------
if isfield(scene,'roads') && ~isempty(scene.roads)
    % 推导立交跨线高架覆盖半长（overpass 沿 X 的端点 |x| 最大值），
    % 使 drawRoads 在立交区省略地面带，凸显高架立体层次。
    ovHalfDraw = [];
    if isfield(scene,'bridges') && ~isempty(scene.bridges)
        for bi2=1:numel(scene.bridges)
            if strcmp(scene.bridges(bi2).kind,'overpass')
                cl = scene.bridges(bi2).centerline;
                ovHalfDraw = max(abs(cl(:,1)));
                break;
            end
        end
    end
    drawRoads(ax, scene.roads, scene.terrainF, ovHalfDraw);
end

% ---------- obstacles ----------
for k=1:numel(scene.obstacles)
    o = scene.obstacles(k);
    t = lower(o.type);
    if strcmp(t,'sphere')
        drawHazardSphere(ax, o, HAZARD, HAZARD_ED);
    elseif strcmp(t,'cube') || strcmp(t,'building')
        drawBuilding(ax, o, BUILDING, BUILD_EDG);
    elseif strcmp(t,'cylinder')
        drawTank(ax, o, BUILDING, BUILD_EDG);
    elseif strcmp(t,'bldg')
        drawBuildingRich(ax, o, BUILDING, BUILD_EDG);
    elseif strcmp(t,'tower')
        drawTower(ax, o, BUILD_EDG);
    elseif strcmp(t,'nofly')
        drawNoFly(ax, o, HAZARD, HAZARD_ED);
    elseif strcmp(t,'tree')
        drawTree(ax, o);
    elseif strcmp(t,'water')
        drawWater(ax, o, scene.bounds, scene.terrainF);
    elseif strcmp(t,'terrain')
        drawTerrain(ax, o, scene.bounds, GRID_COL);
    elseif strcmp(t,'streetlight')
        drawStreetlight(ax, o, BUILD_EDG);
    elseif strcmp(t,'sign')
        drawSign(ax, o, BUILD_EDG);
    % bridge 障碍副本仅用于碰撞，渲染走 scene.bridges 循环（避免重复）
    end
end

% ---------- bridges (L2 立体交通层，跨江/跨谷大桥，不参与障碍代价) ----------
if isfield(scene,'bridges') && ~isempty(scene.bridges)
    for bi2=1:numel(scene.bridges)
        drawBridge(ax, scene.bridges(bi2), BUILD_EDG);
    end
end

% ---------- depot markers (warehouses) ----------
if isfield(scene,'depots') && ~isempty(scene.depots)
    for i=1:size(scene.depots,1)
        p = scene.depots(i,:);
        scatter3(ax, p(1),p(2),p(3), 45, [0.20 0.65 0.35], 'filled', ...
            'Marker','s','MarkerEdgeColor','k','LineWidth',0.6);
    end
end

% ---------- tour customer points ----------
if strcmpi(mode,'tour') && isfield(scene,'tasks') && ~isempty(scene.tasks)
    % 颜色阈值随空域尺度缩放，保证低/中/高分层语义在不同 ENV_SCALE 下一致
    if isfield(scene,'airspace') && isfield(scene.airspace,'low') && isfield(scene.airspace,'mid')
        zLow = scene.airspace.low.zhi;
        zMid = scene.airspace.mid.zhi;
    else
        zLow = 45; zMid = 90;
    end
    for i=1:size(scene.tasks,1)
        p = scene.tasks(i,:);
        % 按高度分色：低=蓝 中=橙 高=红
        if p(3) < zLow
            cc = [0.20 0.45 0.75];
        elseif p(3) < zMid
            cc = [0.95 0.65 0.10];
        else
            cc = [0.85 0.30 0.30];
        end
        th = linspace(0,2*pi,30);
        plot3(ax, p(1)+3*cos(th), p(2)+3*sin(th), p(3)*ones(size(th)), ...
            'Color',cc,'LineWidth',1.2,'LineStyle','-');
        scatter3(ax, p(1),p(2),p(3), 26, cc, 'filled', ...
            'Marker','o','MarkerEdgeColor',[0.3 0.3 0.3],'LineWidth',0.5);
    end
end

% ---------- trajectories (gradient ribbons, dark-core for white bg) ----------
% 多机场景下轨迹易在 dense 城区重叠成"意大利面"，给每架 UAV 一个微小的 z 阶梯偏移
%（不改变物理轨迹，仅供显示），提升可辨识性。
trajH = [];   % 收集轨迹线句柄，供图例精确引用（避免误纳建筑/地形等）
for k=1:scene.nUAV
    if k > size(trajs,1), break; end
    T = trajs{k};
    if isempty(T), continue; end
    col = UAV_COLORS(mod(k-1,size(UAV_COLORS,1))+1,:);
    Tvis = T;
    if scene.nUAV > 1
        Tvis(:,3) = Tvis(:,3) + (k-1) * 0.6;   % 每机递增 0.6m 显示偏移
    end
    % 简单实线轨迹（清晰、无渐变/光晕/分段）：直接 plot3 单段实线
    h = plot3(ax, Tvis(:,1), Tvis(:,2), Tvis(:,3), ...
        'Color',col, 'LineWidth',2.2, 'LineStyle','-');
    trajH = [trajH; h];
end

% ---------- tour task-to-ground垂线（帮助识别每机任务归属） ----------
if strcmpi(mode,'tour') && isfield(scene,'tasks') && ~isempty(scene.tasks) && scene.nUAV > 0
    if isfield(scene,'taskAssign') && ~isempty(scene.taskAssign)
        taskUAV = zeros(size(scene.tasks,1),1);
        for ku=1:numel(scene.taskAssign)
            taskUAV(scene.taskAssign{ku}) = ku;
        end
        for ti=1:size(scene.tasks,1)
            ku = taskUAV(ti);
            if ku == 0, continue; end
            p = scene.tasks(ti,:);
            zG = 0;
            if ~isempty(terrainF)
                zG = terrainF(p(1), p(2));
            end
            col = UAV_COLORS(mod(ku-1,size(UAV_COLORS,1))+1,:);
            plot3(ax, [p(1) p(1)], [p(2) p(2)], [zG p(3)], ...
                'Color',[col 0.55],'LineWidth',1.0,'LineStyle','-');
        end
    end
end

% ---------- start / goal markers ----------
for k=1:scene.nUAV
    col = UAV_COLORS(mod(k-1,size(UAV_COLORS,1))+1,:);
    drawMarker(ax, scene.starts(k,:), 'start', col);
    drawMarker(ax, scene.goals(k,:),  'goal',  col);
end

% ---------- 环境动态（阶段C 3.11：光照/天气驱动渲染）----------
if isfield(scene,'envdyn') && ~isempty(scene.envdyn)
    env = scene.envdyn;
    skyTint = env.skyTint;
    % 三个场景背景统一为纯白（如 easy/clear）：多云/雾仅影响雾效浓度，
    % 不再对背景色做灰色偏移，使 medium/hard 与 easy 一致为纯白底。
    bgCol = skyTint;
    if ~isGUI
        fig = ancestor(ax,'figure');
        if ~isempty(fig), fig.Color = bgCol; end
    end
    ax.Color = bgCol;
    % 黄昏暖色标题（timeOfDay 晚于 17 时偏暖）
    if isfield(env,'timeOfDay') && env.timeOfDay >= 17
        titleCol = [0.45 0.25 0.15];
    else
        titleCol = [0.15 0.20 0.30];
    end
    title(ax,'城市多无人机配送 · 三维态势','Color',titleCol,'FontSize',13,'FontWeight','bold');
    % 雾效：基于能见度（阶段C 3.10 atmosphere.visibility）归一化到 0~1
    if isfield(scene,'atmosphere') && isfield(scene.atmosphere,'visibility')
        vis = scene.atmosphere.visibility;
        fogFrac = max(0, min(0.9, 1 - vis/12000));   % 能见度越低雾越浓
        if fogFrac > 0.02
            fogColor = bgCol;
            set(ax,'Visible','on');
            try
                ax.Fog = 'on'; ax.FogColor = fogColor; ax.FogDensity = fogFrac*0.012;
            catch
                % 旧版本不支持 Fog，跳过（不影响渲染）
            end
        end
    end
else
    title(ax,'城市多无人机配送 · 三维态势','Color',[0.15 0.20 0.30],'FontSize',13,'FontWeight','bold');
end

% ---- 阶段D：道路动态交通层（按当前时刻 tCur 渲染时变车辆） ----
% 车辆与 UAV 同处世界坐标系，贴地行驶，属于低空/地面冲突源。
% 若 scene 未携带 dynamics 或 tCur 未定义，则跳过本层。
if isfield(scene,'dynamics') && ~isempty(scene.dynamics) && ...
   isfield(scene.dynamics,'vehicles') && ~isempty(scene.dynamics.vehicles)
  tCur = 0;
  if isfield(scene,'tCur'), tCur = scene.tCur; end
  mu_draw_dynamic(ax, scene.dynamics, tCur);   % 独立文件 mu_draw_dynamic.m，支持逐帧调用
end

% ---- 阶段E：通信网络与传感器层（基站/中继/终端 + 覆盖球 + 传感器挂载点）----
% comms/sensors 由 mu_city_layout 生成、mu_config 接入 scene；渲染为静态基础设施层。
if isfield(scene,'comms') && ~isempty(scene.comms)
  mu_draw_comms(ax, scene.comms, scene.sensors);  % 独立文件 mu_draw_comms.m
end

xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z');
view(ax, [38 26]);
% 锁定画布尺寸：XY 严格裁切到外框 [-500,500]，Z 固定 [-50,400]；
% 非 GUI 强制 axis square，使窗口呈正方形、四周留白不变形；GUI(UIAxes) 保留 equal 填满面板。
if ~isGUI
    b = scene.bounds;
    ax.XLim = [b(1) b(2)]; ax.YLim = [b(3) b(4)];
    ax.ZLim = [-50 400];
    axis(ax,'square');
else
    axis(ax,'equal');
end

% ---------- UAV 轨迹图例（统一放底部、两行：规范化加粗放大、清晰可读）----------
if ~isempty(trajH) && ~isGUI
    lg = legend(ax, trajH, arrayfun(@(k)sprintf('UAV-%d',k),1:numel(trajH),'UniformOutput',false));
    if ~isempty(lg)
        lg.FontSize = 13;
        lg.FontWeight = 'bold';
        lg.TextColor = [0.15 0.20 0.30];
        lg.Color = [1 1 1];
        lg.EdgeColor = [0.7 0.7 0.7];
        lg.Location = 'southoutside';        % 统一放底部（坐标轴下方外侧）
        lg.Orientation = 'horizontal';
        lg.NumColumns = ceil(numel(trajH)/2); % 分两行排布
        lg.Interpreter = 'none';
        lg.Box = 'on';
    end
end
drawnow;
end

% ============ local helpers ============
function materialSimple(ax, h)
try
    if isprop(h,'AmbientStrength'), h.AmbientStrength = 0.7; end
    if isprop(h,'DiffuseStrength'), h.DiffuseStrength = 0.6; end
    if isprop(h,'SpecularStrength'), h.SpecularStrength = 0.3; end
catch
end
end

function drawBuilding(ax, o, baseC, edgeC)
hold(ax,'on');
h = o.half;
cx=o.c(1); cy=o.c(2); cz=o.c(3);
vx = [-h h h -h -h h h -h];
vy = [-h -h h h -h -h h h];
vz = [-h -h -h -h h h h h];
V = [cx+vx; cy+vy; cz+vz].';
V(:,3) = V(:,3) - 1;
facelist = {[5 6 7 8], [1 2 6 5], [2 3 7 6], [3 4 8 7], [4 1 5 8]};
shade = [1.10 0.94 0.82 0.88 0.96];
for f=1:5
    fc = min(1, baseC*shade(f));
    patch(ax, 'Vertices',V, 'Faces',facelist{f}, ...
        'FaceColor',fc, 'EdgeColor',edgeC, 'EdgeAlpha',0.30, ...
        'LineWidth',0.5, 'FaceLighting','none');
end
drawWindows(ax, V, edgeC);
roof = V(5:8,:); roof = [roof; roof(1,:)];
plot3(ax, roof(:,1), roof(:,2), roof(:,3), 'Color',[0.95 0.97 1.0], ...
    'LineWidth',1.0, 'LineStyle','-');
end

function drawWindows(ax, V, edgeC)
% 在四个侧壁绘制窗格纹理（细线），增强真实感
hold(ax,'on');
% 侧面墙顶点（前/右/后/左）：[1 2 6 5],[2 3 7 6],[3 4 8 7],[4 1 5 8]
walls = {[1 2 6 5], [2 3 7 6], [3 4 8 7], [4 1 5 8]};
nwx = 4; nwy = 6;
for w=1:4
    idx = walls{w};
    c1 = V(idx(1),:); c2 = V(idx(2),:);
    c3 = V(idx(3),:); c4 = V(idx(4),:);
    for i=1:nwx
        for j=1:nwy
            u = (i-0.5)/nwx; v = (j-0.5)/nwy;
            % 双线性插值求墙上一点
            p = (1-v)*((1-u)*c1 + u*c2) + v*((1-u)*c4 + u*c3);
            scatter3(ax, p(1),p(2),p(3), 4, [0.97 0.97 0.99], 'filled', ...
                'Marker','s','MarkerEdgeColor','none');
        end
    end
end
end

function drawBuildingRich(ax, o, baseC, edgeC)
% rich-building：裙楼基座 + 退台塔身 + 楼顶设备 + 玻璃幕堰反光窗格
% 阶段3 颜色高度仿真：按 kind 分层着色（强化四色差异）+ 高度亮度衰减
%   （越高略浅，模拟玻璃幕墙反光/大气透视）+ 楼顶/裙楼/塔身明度微差。
% pod：裙楼相对塔身的外扩半宽；roof：楼顶设备（have/ant 随 seed 可复现）
hold(ax,'on');
cx=o.c(1); cy=o.c(2); zB=o.c(3);  % zB=楼层基底（贴合地形，与碰撞一致）
hw=o.hw; pod=o.pod; baseH=o.baseH; bH=o.bodyH; topH=baseH+bH;
hwP=hw+pod;                       % 裙楼半宽（比塔身更宽）
sb = 0.1; if isfield(o,'setback') && ~isempty(o.setback), sb = o.setback; end
hwT=hw*(1-sb);                      % 塔身退台收进（守卫空 setback，与 mu_obstacle_dist 一致）
% ---- 颜色按类型（强化四色差异，贴近真实城市建材）----
switch lower(o.kind)
    case 'residential', baseCol=[0.66 0.62 0.55];   % 暖灰（住宅混凝土/暖石材）
    case 'office',      baseCol=[0.50 0.60 0.74];   % 冷灰蓝（玻璃办公塔）
    case 'landmark',    baseCol=[0.34 0.42 0.60];   % 深蓝灰（地标幕墙）
    case 'civic',       baseCol=[0.74 0.71 0.61];   % 浅米（公共/文化建筑）
    otherwise,          baseCol=baseC;
end
% 高度亮度衰减：以 150m 为参考，越高实体越浅（玻璃反光/大气透视）
hAtt = topH / 150;
hue = 0; if isfield(o,'hue') && ~isempty(o.hue), hue = o.hue; end
baseCol = min(1, baseCol + 0.10*hAtt + hue);   % 高度衰减 + 明度抖动（守卫空 hue）
% 层次性（L1）：按 tier 给竖向四带轻微色相偏移，强化"成片低层 + 簇拥高层"可读性
if isfield(o,'tier')
    switch o.tier
        case 'L', baseCol = min(1, baseCol + [0.04 -0.02 -0.02]);   % 低层：暖偏
        case 'M', baseCol = min(1, baseCol + [0.00 0.02 0.03]);     % 中层：冷偏
        case 'H', baseCol = min(1, baseCol + [-0.02 0.00 0.04]);    % 高层：蓝偏
        case 'T', baseCol = min(1, baseCol + [-0.04 0.01 0.06]);    % 地标：深蓝偏
    end
end
podiumCol = min(1, baseCol*0.92);                % 裙楼略深（基座厚重感）
towerCol  = min(1, baseCol*1.06);                % 塔身略浅（高空采光）

isL = isfield(o,'hasL') && o.hasL;
cutFrac = 0.2; if isfield(o,'cutFrac') && ~isempty(o.cutFrac), cutFrac = o.cutFrac; end
if isL
    % ---- L 形（缺角）：渲染为外包长方体（视觉简化）----
    % 视觉契约（用户诉求）：楼应为清晰长方体。碰撞侧 L 形由两盒并集精确刻画
    % （R3 渲染/碰撞一致契约在楼层允许轻微放宽——渲染几何为外包 box ⊇ 碰撞并集，
    % UAV 在视觉边缘外=安全，不会"穿可见楼"；底条/侧条细分不影响避障安全）。
    cutx = hwP(1)*cutFrac; cuty = hwP(2)*cutFrac;
    drawLShape(ax, cx, cy, hwP(1), hwP(2), cutx, cuty, zB, zB+topH, baseCol, edgeC);
else
    % ---- 单一外包长方体（视觉简化，去掉 podium/tower 分段与退台）----
    drawBox(ax, cx, cy, hwP(1), hwP(2), zB, zB+topH, baseCol, edgeC);
end
% ---- 楼顶设备（视觉简化移除：保持楼为纯净长方体轮廓）----
end

function drawLShape(ax, cx, cy, hwx, hwy, cutx, cuty, zlo, zhi, col, edgeC)
% L 形 foot 渲染：底条(全宽, y∈[-hwy,cuty]) + 侧条(x∈[-hwx,cutx], 余高)，与碰撞一致
hold(ax,'on');
% 底条
drawBox(ax, cx, cy, hwx, cuty, zlo, zhi, col, edgeC);
% 侧条
drawBox(ax, cx, cy, cutx, hwy, zlo, zhi, col, edgeC);
end

function drawBox(ax, cx, cy, hwx, hwy, zlo, zhi, col, edgeC)
% 轴对齐盒体（基座/塔身/设备）
hold(ax,'on');
V=[cx+[-hwx hwx hwx -hwx -hwx hwx hwx -hwx]; ...
   cy+[-hwy -hwy hwy hwy -hwy -hwy hwy hwy]; ...
   [zlo zlo zlo zlo zhi zhi zhi zhi]].';
facelist={[5 6 7 8],[1 2 6 5],[2 3 7 6],[3 4 8 7],[4 1 5 8]};
shade=[1.10 0.94 0.82 0.88 0.96];
for f=1:5
    patch(ax,'Vertices',V,'Faces',facelist{f},'FaceColor',min(1,col*shade(f)), ...
        'EdgeColor',edgeC,'EdgeAlpha',0.55,'LineWidth',0.6,'FaceLighting','none');
end
roof=V(5:8,:); roof=[roof; roof(1,:)];
plot3(ax, roof(:,1), roof(:,2), roof(:,3), 'Color',[0.95 0.97 1.0], 'LineWidth',1.0, 'LineStyle','-');
end

function drawGlassFacades(ax, cx, cy, hw, zlo, zhi, kind, hue)
% 玻璃幕堰反光：按 4 个朝向给整面不同明度（受光面亮、背光面暗），
% 面内再铺细窗格网 + 轻微反光点，模拟真实玻璃幕墙。
hold(ax,'on');
% 受光方向（固定光照，左上前方），4 个侧面明度系数：
%   +Y 面(前) / +X 面(右) 受光较强， -Y / -X 背光较弱
wallShade = [0.86 0.72 0.60 0.78];   % [+Y, +X, -Y, -X]
% office/landmark 玻璃感强（窗格明显），residential/civic 偏实墙（窗格弱）
switch lower(kind)
    case {'office','landmark'}, glassStr = 1.0; glint = [0.82 0.88 0.95];
    case 'residential',         glassStr = 0.55; glint = [0.78 0.80 0.84];
    otherwise,                  glassStr = 0.45; glint = [0.80 0.78 0.72];   % civic 浅米实墙
end
walls = {[1 2 6 5],[2 3 7 6],[3 4 8 7],[4 1 5 8]};   % 4 个侧面顶点索引（相对 drawBox 的 V）
V=[cx+[-hw(1) hw(1) hw(1) -hw(1) -hw(1) hw(1) hw(1) -hw(1)]; ...
   cy+[-hw(2) -hw(2) hw(2) hw(2) -hw(2) -hw(2) hw(2) hw(2)]; ...
   [zlo zlo zlo zlo zhi zhi zhi zhi]].';
nwx=max(2, round(hw(1)/4)); nwy=max(3, round((zhi-zlo)/6));
for w=1:4
    idx=walls{w}; c1=V(idx(1),:); c2=V(idx(2),:); c3=V(idx(3),:); c4=V(idx(4),:);
    % 整面明度（受光衰减）
    fc = min(1, glint*wallShade(w)*(1+hue*0.4));
    % 窗格网：沿面铺网格线（细，半透明），模拟幕墙分格
    for i=1:nwx
        for j=1:nwy
            u=(i-0.5)/nwx; v=(j-0.5)/nwy;
            p=(1-v)*((1-u)*c1+u*c2)+v*((1-u)*c4+u*c3);
            % 窗格中心反光点（玻璃感强弱决定亮度/密度）
            if mod(i+j,2)==0
                gc = min(1, fc*1.12);
            else
                gc = min(1, fc*0.78);
            end
            sz = 5 * glassStr + 1.5;
            scatter3(ax, p(1),p(2),p(3), sz, gc, 'filled', ...
                'Marker','s','MarkerEdgeColor','none');
        end
    end
end
end

function drawWater(ax, o, bnd, terrainF)
% 水体：浅蓝半透明面，贴合地形函数下凹（河/湖低洼处）
hold(ax,'on');
poly=o.xz;
nseg=size(poly,1);
if ~isempty(terrainF)
    zv = terrainF(poly(:,1), poly(:,2));   % 各顶点处地形高度
    z = min(zv) - 1.5;                      % 略低于周边地形，呈水洼/河道
else
    z = 0;
end
V=[poly ones(nseg,1)*z];   % 贴合地形下凹
% 水面（浅蓝半透明，带轻微波纹描边）
patch(ax,'Vertices',V,'Faces',(1:nseg),'FaceColor',[0.42 0.66 0.82], ...
    'FaceAlpha',0.55,'EdgeColor',[0.28 0.50 0.72],'EdgeAlpha',0.8,'LineWidth',0.8, ...
    'FaceLighting','none','LineStyle','-');
% 沿边限飞/水岸描边（双层，外圈更亮，模拟岸线）
patch(ax,'Vertices',V,'Faces',(1:nseg),'FaceColor','none', ...
    'EdgeColor',[0.30 0.55 0.78],'EdgeAlpha',0.9,'LineWidth',1.2,'LineStyle','-');
% 波纹线（水面平行短纹，增强水感）
for k=1:2
    off = k*2.5;
    Vr = [poly(:,1)+off, poly(:,2)+off, z*ones(nseg,1)];
    plot3(ax, Vr(:,1), Vr(:,2), Vr(:,3), 'Color',[0.60 0.78 0.90], ...
        'LineWidth',0.4,'LineStyle',':');
end
end

function drawTank(ax, o, baseC, edgeC)
hold(ax,'on');
[X,Y,Z]=cylinder(o.r,28); Z=Z*o.h;
surf(ax, o.c(1)+X, o.c(2)+Y, o.c(3)+Z, 'FaceColor',baseC, 'EdgeColor',edgeC, ...
    'EdgeAlpha',0.18, 'LineWidth',0.4, 'FaceLighting','none');
[XT,YT]=cylinder(o.r,28); ZT=ones(size(XT))*o.h;
surf(ax, o.c(1)+XT, o.c(2)+YT, o.c(3)+ZT, 'FaceColor',min(1,baseC*1.12), ...
    'EdgeColor',edgeC,'EdgeAlpha',0.3,'LineWidth',0.5,'FaceLighting','none');
plot3(ax, o.c(1)+o.r*cos(linspace(0,2*pi,40)), o.c(2)+o.r*sin(linspace(0,2*pi,40)), ...
    zeros(1,40), 'Color',edgeC,'LineWidth',0.8);
end

function drawTower(ax, o, edgeC)
% 通信塔架：细杆 + 顶部禁飞球
hold(ax,'on');
[X,Y,Z]=cylinder(o.r,12); Z=Z*o.h;
surf(ax, o.c(1)+X, o.c(2)+Y, o.c(3)+Z, 'FaceColor',[0.45 0.50 0.58], ...
    'EdgeColor',edgeC,'EdgeAlpha',0.5,'LineWidth',0.4,'FaceLighting','none');
% 顶部禁飞球
cb = o.c + [0 0 o.h/2];
[Xs,Ys,Zs]=sphere(18);
surf(ax, cb(1)+o.ball*Xs, cb(2)+o.ball*Ys, cb(3)+o.ball*Zs, ...
    'FaceAlpha',0.30,'EdgeAlpha',0.6,'FaceColor',[0.88 0.42 0.30], ...
    'EdgeColor',[0.72 0.25 0.18],'LineWidth',0.5);
scatter3(ax, cb(1)+o.ball*0.4, cb(2)+o.ball*0.4, cb(3)+o.ball*0.4, 20, ...
    [1 0.95 0.9], 'filled','Marker','o','MarkerEdgeColor','none');
end

function drawNoFly(ax, o, baseC, edgeC)
% 禁飞区：水平投影多边形半透明红填充 + 民航斜纹描边（侧壁 hatch）
hold(ax,'on');
poly=o.xz;
nseg=size(poly,1);
% 侧壁（从 zlo 到 zhi）：半透明红 + 斜纹线（民航禁飞区感）
for j=1:nseg
    p1=poly(j,:); p2=poly(mod(j,nseg)+1,:);
    V=[p1(1) p1(2) o.zlo; p2(1) p2(2) o.zlo; p2(1) p2(2) o.zhi; p1(1) p1(2) o.zhi];
    patch(ax,'Vertices',V,'Faces',[1 2 3 4],'FaceColor',baseC,'FaceAlpha',0.10, ...
        'EdgeColor',edgeC,'EdgeAlpha',0.35,'LineWidth',0.4,'FaceLighting','none');
    % 斜纹 hatch：沿侧壁对角线方向画短斜线
    ns=6;
    for q=1:ns
        a=q/(ns+1); b=1-a;
        x1=p1(1)*a+p2(1)*b; y1=p1(2)*a+p2(2)*b;
        z1=o.zlo; z2=o.zhi;
        plot3(ax, [x1 x1],[y1 y1],[z1 z2], 'Color',[0.80 0.30 0.22], ...
            'LineWidth',0.5,'LineStyle','--');
    end
end
% 顶/底盖
Vtop=[poly ones(nseg,1)*o.zhi]; Vbot=[poly ones(nseg,1)*o.zlo];
patch(ax,'Vertices',Vtop,'Faces',(1:nseg),'FaceColor',baseC,'FaceAlpha',0.14, ...
    'EdgeColor',edgeC,'EdgeAlpha',0.6,'LineWidth',0.8,'FaceLighting','none');
patch(ax,'Vertices',Vbot,'Faces',(1:nseg),'FaceColor',baseC,'FaceAlpha',0.10, ...
    'EdgeColor',edgeC,'EdgeAlpha',0.5,'LineWidth',0.6,'FaceLighting','none');
% 顶盖斜纹（禁飞区标识感）
patch(ax,'Vertices',Vtop,'Faces',(1:nseg),'FaceColor','none', ...
    'EdgeColor',[0.85 0.35 0.25],'EdgeAlpha',0.7,'LineWidth',0.7,'LineStyle','--');
end

function drawTree(ax, o)
% 低矮植被（软障碍）：树干 + 半球冠
hold(ax,'on');
% 树干
[X,Y,Z]=cylinder(o.r*0.3,10); Z=Z*(o.h*0.4);
surf(ax, o.c(1)+X, o.c(2)+Y, o.c(3)+Z, 'FaceColor',[0.45 0.35 0.25], ...
    'EdgeColor','none','FaceAlpha',0.9,'FaceLighting','none');
% 树冠（半球）
[Xs,Ys,Zs]=sphere(14); Zs=max(Zs,0);
hc=o.h*0.6;
surf(ax, o.c(1)+o.r*Xs, o.c(2)+o.r*Ys, o.c(3)+o.h*0.4+hc*Zs, ...
    'FaceColor',[0.45 0.62 0.40],'FaceAlpha',0.55,'EdgeColor',[0.30 0.45 0.28], ...
    'EdgeAlpha',0.3,'LineWidth',0.3,'FaceLighting','none');
end

function drawRoads(ax, roads, terrainF, overHalf)
% 地面公路网：把 scene.roads 渲染为贴地灰色道路带（arterial 宽亮、collector 窄），
% 作为明确的"地面层"，让地面呈现路网而非纯地形伪彩。道路沿 centerline 生成矩形带，
% 高度沿中心线插值 terrainF / z_profile，保证长坡路段真实贴地。
% 高架桥（L2）由 drawBridge 单独绘制，本函数只画地面道路。
% overHalf（可选）：X 向 arterial 在 |x|<=overHalf 立交高架覆盖区省略地面带，
%   避免地面带与跨线高架投影重叠，使"高架跨地面"的立交层次清晰可辨。
hold(ax,'on');
if nargin < 4 || isempty(overHalf), overHalf = 0; end
nR = numel(roads);
for ri=1:nR
    r = roads(ri);
    cl = r.centerline;                 % 2x2 端点 [x;y]
    w  = r.width;                      % 路宽（米），半宽 w/2
    p1 = cl(1,:); p2 = cl(2,:);
    % 是否为沿 X 的主干（y 恒定），需在其跨越立交高架区时拆分地面带
    isX = abs(p1(2)-p2(2)) < 1e-6;
    if isX && overHalf > 0
        xa = min(p1(1), p2(1)); xb = max(p1(1), p2(1));
        spans = [xa -overHalf; overHalf xb];   % 两段：高架外侧保留地面带
    else
        spans = [p1(1) p2(1)];
    end
    for sp=1:size(spans,1)
        xA = spans(sp,1); xB = spans(sp,2);
        if abs(xB-xA) < 1e-3, continue; end
        a = [xA p1(2)]; b = [xB p2(2)];
        dir = (b-a)/norm(b-a); perp = [-dir(2) dir(1)];
        % 沿中心线多段采样取地形高度，避免长坡路用单点均值而浮于/陷入地形
        nSeg = max(2, ceil(norm(b-a)/20));
        tSeg = linspace(0,1,nSeg);
        midZ = 0;
        if ~isempty(terrainF)
            for si=1:nSeg
                cseg = a + (b-a)*tSeg(si);
                midZ = midZ + terrainF(cseg(1), cseg(2));
            end
            midZ = midZ / nSeg;
        else
            midZ = 0;
        end
        zc = midZ + 0.4;
        hw = w/2;
        V = [a+perp*hw, zc; b+perp*hw, zc; b-perp*hw, zc; a-perp*hw, zc];
        if strcmpi(r.class,'arterial')
            fc = [0.52 0.55 0.60]; ec = [0.35 0.38 0.44]; lw = 0.8;  % 主干道：宽亮灰
        else
            fc = [0.62 0.65 0.70]; ec = [0.45 0.48 0.54]; lw = 0.6;  % 次干道：浅灰窄
        end
        patch(ax,'Vertices',V,'Faces',[1 2 3 4],'FaceColor',fc,'FaceAlpha',0.95, ...
            'EdgeColor',ec,'EdgeAlpha',0.8,'LineWidth',lw,'FaceLighting','none');
        plot3(ax, [a(1) b(1)], [a(2) b(2)], [zc+0.05 zc+0.05], ...
            'Color',[0.80 0.66 0.20],'LineWidth',0.5,'LineStyle','--');
    end
end
end

function drawTerrain(ax, o, bnd, gridCol)
% 地形高度场：伪彩色（低绿→中黄→高棕）+ 等高线感网格
hold(ax,'on');
f=o.f;
nx=48; ny=48;
xs=linspace(bnd(1),bnd(2),nx); ys=linspace(bnd(3),bnd(4),ny);
[X,Y]=meshgrid(xs,ys);
Z=f(X,Y);
% 伪彩映射：按 z 归一化到 [0,1]，低=绿 中=黄 高=棕
zmin=min(Z(:)); zmax=max(Z(:)); zr=zmax-zmin;
if zr<1e-6, zr=1; end
t=(Z-zmin)/zr;
% 三段线性：绿(0.30,0.55,0.30) → 黄(0.78,0.72,0.40) → 棕(0.55,0.42,0.30)
C=zeros(size(Z,1),size(Z,2),3);
lo=t<0.5; hi=~lo;
C(:,:,1)=lo.*(0.30+2*(0.78-0.30)*t) + hi.*(0.78+(0.55-0.78)*(t-0.5)/0.5);
C(:,:,2)=lo.*(0.55+2*(0.72-0.55)*t) + hi.*(0.72+(0.42-0.72)*(t-0.5)/0.5);
C(:,:,3)=lo.*(0.30+2*(0.40-0.30)*t) + hi.*(0.40+(0.30-0.40)*(t-0.5)/0.5);
surf(ax, X, Y, Z, 'CData',C, 'FaceColor','interp', 'EdgeColor',gridCol, ...
    'EdgeAlpha',0.15,'LineWidth',0.2,'FaceAlpha',0.45,'FaceLighting','none');
end

function drawHazardSphere(ax, o, baseC, edgeC)
hold(ax,'on');
[X,Y,Z] = sphere(24);
surf(ax, o.c(1)+1.12*o.r*X, o.c(2)+1.12*o.r*Y, o.c(3)+1.12*o.r*Z, ...
    'FaceAlpha',0.04,'EdgeAlpha',0,'FaceColor',baseC);
hs = surf(ax, o.c(1)+o.r*X, o.c(2)+o.r*Y, o.c(3)+o.r*Z, ...
    'FaceAlpha',0.30,'EdgeAlpha',0.6,'FaceColor',baseC, ...
    'EdgeColor',edgeC,'LineWidth',0.5);
hl = o.c + o.r*0.55*[0.6 0.6 0.55];
scatter3(ax, hl(1), hl(2), hl(3), 26, [1 0.95 0.9], 'filled', ...
    'Marker','o','MarkerEdgeColor','none');
plot3(ax, o.c(1)+1.12*o.r*cos(linspace(0,2*pi,48)), ...
    o.c(2)+1.12*o.r*sin(linspace(0,2*pi,48)), ...
    o.c(3)*ones(1,48), 'Color',baseC,'LineStyle',':','LineWidth',0.8);
end

function drawMarker(ax, p, kind, col)
hold(ax,'on');
if strcmpi(kind,'start')
    c = [0.15 0.70 0.35]; mk = '^';
else
    c = [0.85 0.30 0.55]; mk = 'v';
end
plot3(ax, [p(1) p(1)], [p(2) p(2)], [p(3) p(3)], ...
    'Color', [0.5 0.5 0.5], 'LineWidth',1.0, 'LineStyle',':');
scatter3(ax, p(1),p(2),p(3), 45, c, 'filled', ...
    'Marker',mk,'MarkerEdgeColor','k','LineWidth',0.8);
end

function drawBridge(ax, o, edgeC)
% 跨江/跨谷大桥（L2 立体交通层）：路面板（连续桥面带）+ 两侧栏杆 + 桥墩
% 桥身用偏暖色（混凝土灰+浅橙面层），与冷蓝色建筑形成色彩对比，使桥在密集城区
% 里"跳出来"，呈现立体交通的视觉层次。
% 直线桥（n=2）用单段 box；多段桥（弧线匝道/盘桥/立交）沿弧长重采样为连续曲面，
% 视觉上完全丝滑（碰撞仍用 make_bridge_collision 的逐段 box，保持判定一致）。
hold(ax,'on');
cl = o.centerline; n = size(cl,1);
w = o.width/2;
dZ = o.deckZ;
if isscalar(dZ), zA = dZ; zB = dZ; else zA = dZ(1); zB = dZ(end); end
% 桥墩（pillars 每行 = [x y baseZ]，柱从地形基底到桥面，用粗线+半透明矩形 box）。
% 墩顶高度按该节点处的实际 deckZ（对盘桥/匝道/立交曲线段，deckZ 沿桥长变化，
% 不再固定取起始高度，从而与桥面贴合，消除桥墩与弯曲桥面错位）。
for pi=1:size(o.pillars,1)
    pc = o.pillars(pi,:);
    if n == 2
        zTop = zA;                            % 直线桥：两端高度
    else
        zTop = zA + (zB - zA) * ((pi-1)/(n-1)); % 折线桥：节点处沿桥长插值的真实桥面高度
    end
    zTop = max(zTop, pc(3) + 1);              % 防止墩顶低于基底
    Vp = [pc(1)-3.2 pc(2)-3.2 pc(3); pc(1)+3.2 pc(2)-3.2 pc(3); ...
          pc(1)+3.2 pc(2)+3.2 pc(3); pc(1)-3.2 pc(2)+3.2 pc(3); ...
          pc(1)-3.2 pc(2)-3.2 zTop; pc(1)+3.2 pc(2)-3.2 zTop; ...
          pc(1)+3.2 pc(2)+3.2 zTop; pc(1)-3.2 pc(2)+3.2 zTop];
    facesP = [5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    patch(ax,'Vertices',Vp,'Faces',facesP,'FaceColor',[0.70 0.66 0.60], ...
        'EdgeColor',edgeC,'EdgeAlpha',0.35,'LineWidth',0.4,'FaceLighting','none');
end
% ---- 桥面：连续曲面（视觉丝滑）----
% 统一沿弧长重采样 centerline 为密集点（每 ~3m 一个点，最少 40 段），
% 桥面高度沿弧长线性插值（用原始节点 deckZ），法向由局部切线得到，
% 生成左/右缘点列后用 surf 连成无缝桥面带；标线=重采样中心线，栏杆=左右缘。
if n == 2
    % 直线桥：直接用端点，单段 box 即可（已平滑）
    c1 = cl(1,:); c2 = cl(2,:);
    cc = (c1+c2)/2; Lh = norm(c2-c1)/2;
    dir = (c2-c1)/norm(c2-c1); perp = [-dir(2) dir(1)];
    za = zA; zb = zB;
    Vloc = [-Lh -w za; Lh -w zb; Lh w zb; -Lh w za; -Lh -w za+2; Lh -w zb+2; Lh w zb+2; -Lh w za+2];
    R = [dir(1) dir(2) 0; perp(1) perp(2) 0; 0 0 1];
    V = (R * Vloc.').'; V(:,1)=V(:,1)+cc(1); V(:,2)=V(:,2)+cc(2);
    faces = [5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    shade = [1.05 0.92 0.84 0.90 0.98];
    for f=1:5
        fc = min(1, [0.78 0.74 0.68]*shade(f));
        patch(ax,'Vertices',V,'Faces',faces(f,:),'FaceColor',fc,'EdgeColor',edgeC, ...
            'EdgeAlpha',0.4,'LineWidth',0.5,'FaceLighting','none');
    end
    % 标线 + 栏杆
    plot3(ax, [c1(1) c2(1)], [c1(2) c2(2)], [(za+zb)/2+2.05 (za+zb)/2+2.05], ...
        'Color',[0.95 0.85 0.30],'LineWidth',1.0);
    for sgn=[-1 1]
        e1 = cc + dir*Lh + perp*(sgn*w); e2 = cc - dir*Lh + perp*(sgn*w);
        plot3(ax, [e1(1) e2(1)], [e1(2) e2(2)], [(za+zb)/2+2 (za+zb)/2+2], 'Color',edgeC,'LineWidth',0.8);
    end
else
    % 多段桥：沿弧长重采样为连续带
    segLen = sqrt(sum(diff(cl,1,1).^2, 2));
    cumLen = [0; cumsum(segLen)];
    totalLen = cumLen(end);
    nSamp = max(40, ceil(totalLen/3));        % 每 ~3m 一点，保证丝滑
    ts = linspace(0, totalLen, nSamp);
    % 重采样 XY（按弧长线性插值原始折线）
    xs = interp1(cumLen, cl(:,1), ts, 'linear', 'extrap');  xs = xs(:);
    ys = interp1(cumLen, cl(:,2), ts, 'linear', 'extrap');  ys = ys(:);
    % 重采样 deckZ：原始节点 deckZ 沿桥长（按节点参数 0..1）线性插值
    if isscalar(dZ)
        zs = repmat(dZ, 1, nSamp);
    else
        zNodes = linspace(0, 1, numel(dZ)).';
        zs = interp1(zNodes, dZ(:).', ts/totalLen, 'linear', 'extrap');  zs = zs(:);
    end
    % 局部切线 -> 法向（统一为列向量，避免行/列拼接维度不一致）
    tx = [diff(xs); xs(end)-xs(end-1)];  tx = tx(:);
    ty = [diff(ys); ys(end)-ys(end-1)];  ty = ty(:);
    tl = sqrt(tx.^2 + ty.^2); tx = tx./max(tl,1e-9); ty = ty./max(tl,1e-9);
    px = -ty; py = tx;                          % 法向
    % 左右缘点（带桥面厚度 2）
    Lx = xs + px*w; Ly = ys + py*w; Lz = zs;
    Rx = xs - px*w; Ry = ys - py*w; Rz = zs;
    % 连续桥面带（surf）：2 x nSamp 网格（左缘行 + 右缘行）
    Xb = [Lx, Rx].'; Yb = [Ly, Ry].'; Zb = [Lz, Rz+2].';
    surf(ax, Xb, Yb, Zb, 'FaceColor',[0.78 0.74 0.68], 'EdgeColor','none', ...
        'FaceAlpha',1, 'LineWidth',0.2, 'FaceLighting','none');
    % 标线（重采样中心线，沿桥脊）
    plot3(ax, xs, ys, zs+2.05, 'Color',[0.95 0.85 0.30], 'LineWidth',1.0);
    % 栏杆（左右缘连续线）
    plot3(ax, Lx, Ly, Lz+2, 'Color',edgeC, 'LineWidth',0.8);
    plot3(ax, Rx, Ry, Rz+2, 'Color',edgeC, 'LineWidth',0.8);
end
end

function drawStreetlight(ax, o, edgeC)
% 路灯：金属灯杆（圆柱）+ 水平悬挑灯具（暖橙发光 box）
hold(ax,'on');
pl = o.pole; ar = o.arm;
% 灯杆
[X,Y,Z]=cylinder(pl.r,12); Z=Z*pl.h;
surf(ax, pl.c(1)+X, pl.c(2)+Y, pl.c(3)+Z, 'FaceColor',[0.55 0.58 0.64], ...
    'EdgeColor',edgeC,'EdgeAlpha',0.4,'LineWidth',0.3,'FaceLighting','none');
% 灯臂（旋转 box，利用 arm.R / arm.hw 与碰撞一致）
Vloc = [-ar.hw(1) -ar.hw(2) -ar.hw(3); ar.hw(1) -ar.hw(2) -ar.hw(3); ...
        ar.hw(1)  ar.hw(2) -ar.hw(3); -ar.hw(1)  ar.hw(2) -ar.hw(3); ...
        -ar.hw(1) -ar.hw(2)  ar.hw(3); ar.hw(1) -ar.hw(2)  ar.hw(3); ...
        ar.hw(1)  ar.hw(2)  ar.hw(3); -ar.hw(1)  ar.hw(2)  ar.hw(3)];
V = (ar.R * Vloc.').'; V = V + ar.c;
faces = [5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
for f=1:5
    patch(ax,'Vertices',V,'Faces',faces(f,:),'FaceColor',[0.30 0.30 0.32], ...
        'EdgeColor',edgeC,'EdgeAlpha',0.3,'LineWidth',0.3,'FaceLighting','none');
end
% 灯具发光点（暖橙，呼应夜景路灯）
scatter3(ax, ar.c(1), ar.c(2), ar.c(3), 36, [1.0 0.72 0.30], ...
    'filled','Marker','o','MarkerEdgeColor','none');
end

function drawSign(ax, o, edgeC)
% 交通标志：金属立柱（box）+ 牌面（box，暖橙描边）
hold(ax,'on');
pst = o.post;
Vp = [pst.c(1)+[-pst.hw(1) pst.hw(1) pst.hw(1) -pst.hw(1) -pst.hw(1) pst.hw(1) pst.hw(1) -pst.hw(1)]; ...
      pst.c(2)+[-pst.hw(2) -pst.hw(2) pst.hw(2) pst.hw(2) -pst.hw(2) -pst.hw(2) pst.hw(2) pst.hw(2)]; ...
      [pst.c(3)-pst.hw(3) pst.c(3)-pst.hw(3) pst.c(3)-pst.hw(3) pst.c(3)-pst.hw(3) ...
       pst.c(3)+pst.hw(3) pst.c(3)+pst.hw(3) pst.c(3)+pst.hw(3) pst.c(3)+pst.hw(3)]].';
faces = [5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
for f=1:5
    patch(ax,'Vertices',Vp,'Faces',faces(f,:),'FaceColor',[0.55 0.58 0.64], ...
        'EdgeColor',edgeC,'EdgeAlpha',0.3,'LineWidth',0.3,'FaceLighting','none');
end
pnl = o.panel;
Vn = [pnl.c(1)+[-pnl.hw(1) pnl.hw(1) pnl.hw(1) -pnl.hw(1) -pnl.hw(1) pnl.hw(1) pnl.hw(1) -pnl.hw(1)]; ...
      pnl.c(2)+[-pnl.hw(2) -pnl.hw(2) pnl.hw(2) pnl.hw(2) -pnl.hw(2) -pnl.hw(2) pnl.hw(2) pnl.hw(2)]; ...
      [pnl.c(3)-pnl.hw(3) pnl.c(3)-pnl.hw(3) pnl.c(3)-pnl.hw(3) pnl.c(3)-pnl.hw(3) ...
       pnl.c(3)+pnl.hw(3) pnl.c(3)+pnl.hw(3) pnl.c(3)+pnl.hw(3) pnl.c(3)+pnl.hw(3)]].';
for f=1:5
    patch(ax,'Vertices',Vn,'Faces',faces(f,:),'FaceColor',[0.95 0.55 0.20], ...
        'EdgeColor',[0.72 0.35 0.12],'EdgeAlpha',0.5,'LineWidth',0.4,'FaceLighting','none');
end
end

