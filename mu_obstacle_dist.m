function [dist, pen] = mu_obstacle_dist(point, obs, margin)
% mu_obstacle_dist — 计算点到所有障碍的有符号距离与惩罚（城市真实场景版）
%   point : 1x3 或 Nx3 点
%   obs   : 障碍 struct 数组，支持类型：
%           'cube'       长方体楼宇    字段 c(中心) half(半边长3)
%           'cylinder'   储罐/圆柱     字段 c(底心) r h
%           'sphere'     禁飞球/树冠   字段 c r
%           'building'   单栋楼宇(同 cube，语义别名)
%           'row'        成排街区楼宇  字段 c(街区中心) grid(2x2半宽) gap(街道半宽)
%                                   hmin hmax nbx nby (行列数) seed(可复现)
%           'tower'      通信塔架      字段 c(底心) r(杆半径) h(总高) ball(顶部禁飞球半径)
%           'nofly'      禁飞区        字段 xz(多边形顶点 Nx2) zlo zhi (水平投影+高度范围)
%           'tree'       低矮植被(软)  字段 c r h(树高)
%           'terrain'    地形高度场    字段 f(函数句柄 z=z_ground(x,y)) 仅参与"最低飞行高度"
%   margin: 安全裕度（软约束边界外扩）
%   dist  : Nx1 到最近障碍表面的带符号距离（<0 表示在障碍内）
%   pen   : Nx1 穿透惩罚（深度越大指数增长）
if nargin<3, margin = 0; end
if size(point,2)~=3, point = point.'; end
N = size(point,1);
dist = inf(N,1);

for k=1:numel(obs)
    o = obs(k);
    t = lower(o.type);
    if strcmp(t,'sphere')
        d = sqrt(sum((point - o.c).^2, 2)) - (o.r + margin);
    elseif strcmp(t,'cube') || strcmp(t,'building')
        q = abs(point - o.c) - o.half;
        outside = sqrt(sum(max(q,0).^2,2));
        inside  = min(max(q,[],2), 0);
        d = outside + inside - margin;
    elseif strcmp(t,'cylinder')
        rho = sqrt((point(:,1)-o.c(1)).^2 + (point(:,2)-o.c(2)).^2) - (o.r+margin);
        zlim = abs(point(:,3)-o.c(3)) - (o.h/2);
        d = max(rho, zlim);
    elseif strcmp(t,'row')
        % 成排街区楼宇：在 [c(1,2)±grid] 区域内按 nbx×nby 网格排布楼宇，
        % 楼宇半宽 = (cell - gap)，cell = 2*grid/(nb-1) 间距。点到整排取最近单栋距离。
        bx = o.grid(1); by = o.grid(2);
        nb = [o.nbx o.nby];
        cx = o.c(1); cy = o.c(2);
        % 单栋中心坐标
        xs = cx + linspace(-bx, bx, nb(1));
        ys = cy + linspace(-by, by, nb(2));
        cellx = (nb(1)>1) * (2*bx/(nb(1)-1)) : 0; % placeholder
        stepx = (nb(1)>1) * (2*bx/(nb(1)-1));
        stepy = (nb(2)>1) * (2*by/(nb(2)-1));
        bw = max(8, stepx - 2*o.gap) / 2;   % 单栋半宽(x)
        bh = max(8, stepy - 2*o.gap) / 2;   % 单栋半宽(y)
        if nb(1)==1 && nb(2)==1
            bw = bx * 0.85; bh = by * 0.85;
        end
        dmin = inf(N,1);
        for iy=1:nb(2)
            for ix=1:nb(1)
                cc = [xs(ix) ys(iy) (o.hmin+o.hmax)/2];
                hh = [bw bh (o.hmax-o.hmin)/2];
                q = abs(point - cc) - hh;
                outside = sqrt(sum(max(q,0).^2,2));
                inside  = min(max(q,[],2), 0);
                di = outside + inside - margin;
                dmin = min(dmin, di);
            end
        end
        d = dmin;
    elseif strcmp(t,'tower')
        % 细长杆（圆柱）+ 顶部禁飞球；取两者最近
        rho = sqrt((point(:,1)-o.c(1)).^2 + (point(:,2)-o.c(2)).^2) - (o.r+margin);
        zlim = abs(point(:,3)-o.c(3)) - (o.h/2);
        dshaft = max(rho, zlim);
        % 顶部球心
        cball = o.c + [0 0 o.h/2];
        dball = sqrt(sum((point - cball).^2, 2)) - (o.ball + margin);
        d = min(dshaft, dball);
    elseif strcmp(t,'nofly')
        % 禁飞区：水平投影多边形内 且 z∈[zlo-margin, zhi+margin] 才惩罚
        in = mu_in_poly(point(:,1:2), o.xz);
        zlo = o.zlo - margin; zhi = o.zhi + margin;
        d = inf(N,1);
        for i=1:N
            if in(i)
                if point(i,3) < zlo
                    d(i) = point(i,3) - zlo;          % 在禁飞区下方，距下边界
                elseif point(i,3) > zhi
                    d(i) = zhi - point(i,3);          % 在禁飞区上方
                else
                    d(i) = -1e-3;                     % 区内，轻微穿透（强惩罚）
                end
            else
                d(i) = inf;
            end
        end
    elseif strcmp(t,'tree')
        % 低矮植被（软障碍）：树干圆柱 + 半球冠；穿透惩罚较弱（可低空掠过）
        rho = sqrt((point(:,1)-o.c(1)).^2 + (point(:,2)-o.c(2)).^2) - (o.r+margin);
        zlim = abs(point(:,3)-o.c(3)) - (o.h/2);
        d = max(rho, zlim);
    elseif strcmp(t,'terrain')
        % 地形：仅在 z 低于地面+margin 时惩罚（最低飞行高度约束）
        zg = o.f(point(:,1), point(:,2));
        d = point(:,3) - zg - margin;   % <0 表示低于地形安全高度
    elseif strcmp(t,'bldg')
        % rich-building：裙楼盒 + 上部塔身盒（退台收进），两段取最近。
        % 以 o.c(3) 为楼层基底（贴合地形，修复悬空/埋地暗伤），碰撞/渲染一致。
        % L 形 foot：poly 非空时，用两盒并集（底条 + 侧条）精确刻画缺角；
        %   cutFrac 由 mu_make_bldg 输出，避免与渲染端硬编码不同步。
        cx = o.c(1); cy = o.c(2); zB = o.c(3);  % 楼层基底（地面标高）
        hw = o.hw; pod = o.pod; baseH = o.baseH; topH = o.baseH + o.bodyH;
        hwP = hw + pod;                       % 裙楼半宽（比塔身更宽）
        hwT = hw * (1 - o.setback);           % 塔身退台收进
        cutFrac = 0.2; if isfield(o,'cutFrac') && ~isempty(o.cutFrac), cutFrac = o.cutFrac; end
        isL = isfield(o,'hasL') && o.hasL;
        if isL
            % L 形分解为两盒：底条(全宽, y∈[-hwy, hwy*cutFrac]) + 侧条(x∈[-hwx, hwx*cutFrac], 余高)
            dP = mu_lshape_dist(point, cx, cy, hwP(1), hwP(2), zB, zB+baseH, cutFrac);
            dT = mu_lshape_dist(point, cx, cy, hwT(1), hwT(2), zB+baseH, zB+topH, cutFrac);
            d = min(dP, dT) - margin;
        else
            % 裙楼（zB~zB+baseH，半宽 hwP）
            dbase = mu_box_dist(point, cx, cy, hwP(1), hwP(2), zB, zB+baseH);
            % 塔身（zB+baseH~zB+topH，半宽 hwT）
            dtop  = mu_box_dist(point, cx, cy, hwT(1), hwT(2), zB+baseH, zB+topH);
            d = min(dbase, dtop) - margin;
        end
    elseif strcmp(t,'bridge')
        % 桥面 deck：旋转 box（R 旋转，与 mu_draw_scene/drawBridge 一致）
        % 桥墩 pier：竖直圆柱（底在地面 0、顶到 deckZ），可能多个
        if isfield(o,'deck')
            dk = o.deck;                       % struct: c(3),R(3x3),hw(3)
            pc = dk.c; R = dk.R; hwx = dk.hw(1); hwy = dk.hw(2); hzh = dk.hw(3);
            t = point - pc; lp = (R.'*t.').';
            qx = abs(lp(:,1)) - hwx; qy = abs(lp(:,2)) - hwy; qz = abs(lp(:,3)) - hzh;
            outside = sqrt(sum(max([qx qy qz],0).^2, 2));
            inside  = min(max([qx qy qz],[],2), 0);
            dDeck = outside + inside - margin;
        else
            dDeck = inf(N,1);
        end
        if isfield(o,'pier')
            dPier = inf(N,1);
            for pi=1:numel(o.pier)
                pr = o.pier(pi);                % struct: c(3底心), r, z1(墩顶高)
                zc = pr.z1/2;                    % 圆柱中心高
                rho = sqrt((point(:,1)-pr.c(1)).^2 + (point(:,2)-pr.c(2)).^2) - (pr.r+margin);
                zlim = abs(point(:,3)-zc) - (pr.z1/2);
                dPier = min(dPier, max(rho, zlim));
            end
        else
            dPier = inf(N,1);
        end
        d = max(dDeck, dPier);                  % 桥面或任一桥墩，取最近
    elseif strcmp(t,'streetlight')
        % 灯杆 cylinder（自地面 c(3) 到 h）+ 顶灯水平短 box（悬挑 arm）
        if isfield(o,'pole')
            pl = o.pole;                        % struct: c(3底心), r, h
            zc = pl.c(3) + pl.h/2;
            rho = sqrt((point(:,1)-pl.c(1)).^2 + (point(:,2)-pl.c(2)).^2) - (pl.r+margin);
            zlim = abs(point(:,3)-zc) - (pl.h/2);
            dPole = max(rho, zlim);
        else
            dPole = inf(N,1);
        end
        if isfield(o,'arm')
            ar = o.arm;                         % struct: c(3),hw(3) 旋转 box（灯具悬挑）
            t = point - ar.c; lp = (ar.R.'*t.').';
            qx = abs(lp(:,1)) - ar.hw(1); qy = abs(lp(:,2)) - ar.hw(2); qz = abs(lp(:,3)) - ar.hw(3);
            outside = sqrt(sum(max([qx qy qz],0).^2, 2));
            inside  = min(max([qx qy qz],[],2), 0);
            dArm = outside + inside - margin;
        else
            dArm = inf(N,1);
        end
        d = max(dPole, dArm);
    elseif strcmp(t,'water')
        % 水体（water）：水平投影多边形内 且 z 低于水面（≈地形 0 基准之上）才惩罚，
        % 视为"水面以下禁飞"薄层。此前缺此分支会落到末尾 else -> d=inf，
        % 水体障碍永不约束轨迹、客户点可落在水里（R1 修复）。
        % 水面高度取 0（与 terrainF 平移后全域 >=0、水体位于河谷下凹处一致）；
        % 穿透深度 = 0 - z（z<0 时为正深度），dist = -depth 使 pen 随下潜深度超线性增长。
        in = mu_in_poly(point(:,1:2), o.xz);
        d = inf(N,1);
        for i=1:N
            if in(i)
                if point(i,3) < 0
                    d(i) = point(i,3);              % 水面下：dist 为负，深度 = -z
                else
                    d(i) = point(i,3);              % 水面上：dist 为正（自由），pen=0
                end
            end
        end
    elseif strcmp(t,'sign')
        % 立柱 post（box）+ 牌面 panel（box），均轴对齐
        if isfield(o,'post')
            dPost = mu_box_dist(point, o.post.c(1), o.post.c(2), o.post.hw(1), o.post.hw(2), o.post.c(3)-o.post.hw(3), o.post.c(3)+o.post.hw(3)) - margin;
        else
            dPost = inf(N,1);
        end
        if isfield(o,'panel')
            dPanel = mu_box_dist(point, o.panel.c(1), o.panel.c(2), o.panel.hw(1), o.panel.hw(2), o.panel.c(3)-o.panel.hw(3), o.panel.c(3)+o.panel.hw(3)) - margin;
        else
            dPanel = inf(N,1);
        end
        d = max(dPost, dPanel);
    else
        d = inf(N,1);
    end
    dist = min(dist, d);
end
% 若所有障碍都返回 inf（如点全在禁飞区外），给一个安全正值兜底
dist(dist==inf) = 1e3;

% 软惩罚：仅在 dist < 0（穿透）时增长；平滑近似使梯度连续
t = max(0, -dist);
pen = t.^2 .* (1 + 5*exp(-t/3));     % 平滑、连续、随深度超线性增长
end

function in = mu_in_poly(xy, poly)
% mu_in_poly — 射线法判断点是否在多边形内
%   xy  : Nx2    poly : Mx2
N = size(xy,1); M = size(poly,1);
in = false(N,1);
for i=1:N
    x = xy(i,1); y = xy(i,2);
    cnt = 0;
    for j=1:M
        x1=poly(j,1); y1=poly(j,2);
        x2=poly(mod(j,M)+1,1); y2=poly(mod(j,M)+1,2);
        if ((y1>y) ~= (y2>y))
            xint = (x2-x1)*(y-y1)/(y2-y1) + x1;
            if x < xint, cnt = cnt+1; end
        end
    end
    in(i) = (mod(cnt,2)==1);
end
end

function d = mu_box_dist(point, cx, cy, hwx, hwy, zlo, zhi)
% mu_box_dist — 轴对齐矩形柱（xy 半宽 hwx/hwy，z∈[zlo,zhi]）带符号距离
%   外部：到柱面最近距离；内部：最深穿透（<=0）
qx = abs(point(:,1)-cx) - hwx;
qy = abs(point(:,2)-cy) - hwy;
qz = abs(point(:,3)-(zlo+zhi)/2) - (zhi-zlo)/2;
outside = sqrt(sum(max([qx qy qz],0).^2, 2));
inside  = min(max([qx qy qz],[],2), 0);
d = outside + inside;
end

function d = mu_lshape_dist(point, cx, cy, hwx, hwy, zlo, zhi, cutFrac)
% mu_lshape_dist — L 形 foot（整矩形缺 +x,+y 角）柱体带符号距离
%   poly 切角比例 cutFrac（与 mu_make_bldg 输出一致，默认 0.2）
%   分解为两盒并集：底条(全宽, y∈[-hwy, hwy*cutFrac]) + 侧条(x∈[-hwx, hwx*cutFrac], 余高)
%   取并集 = min(盒距)，z 范围 [zlo,zhi]。
if nargin<8 || isempty(cutFrac), cutFrac = 0.2; end
cutx = hwx*cutFrac; cuty = hwy*cutFrac;
% 底条：x∈[-hwx,hwx], y∈[-hwy, cuty]
dB = mu_box_dist(point, cx, cy, hwx, cuty, zlo, zhi);
% 侧条：x∈[-hwx, cutx], y∈[cuty, hwy]
dS = mu_box_dist(point, cx, cy, cutx, hwy, zlo, zhi);
d = min(dB, dS);
end
