function [dist, pen] = mu_obstacle_dist(point, obs, margin)
% mu_obstacle_dist ????%   point : 1x3 ??Nx3 ??%   obs   :  struct 
%           'cube'       ??    c() half(??)
%           'cylinder'   /      c() r h
%           'sphere'     ??    c r
%           'building'   (??cube??
%           'row'           c() grid(2x2) gap()
%                                   hmin hmax nbx nby (?? seed(??
%           'tower'             c() r(?? h() ball(??
%           'nofly'      ??        xz(??Nx2) zlo zhi (+)
%           'tree'       (??   c r h()
%           'terrain'    ??    f( z=z_ground(x,y)) ????
%   margin: ??%   dist  : Nx1 <0 ??%   pen   : Nx1 ??if nargin<3, margin = 0; end
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
        % grid 中心 [c(1,2)]，nbx*nby 格
        bx = o.grid(1); by = o.grid(2);
        nb = [o.nbx o.nby];
        cx = o.c(1); cy = o.c(2);
        % 
        xs = cx + linspace(-bx, bx, nb(1));
        ys = cy + linspace(-by, by, nb(2));
        stepx = (nb(1)>1) * (2*bx/(nb(1)-1));
        stepy = (nb(2)>1) * (2*by/(nb(2)-1));
        bw = max(8, stepx - 2*o.gap) / 2;   % (x)
        bh = max(8, stepy - 2*o.gap) / 2;   % (y)
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
        % 通信塔：圆柱塔身 + 顶部球（与 cylinder 分支一致的 rho 计算）
        rho = sqrt((point(:,1)-o.c(1)).^2 + (point(:,2)-o.c(2)).^2) - (o.r+margin);
        zlim = abs(point(:,3)-o.c(3)) - (o.h/2);
        dshaft = max(rho, zlim);
        % 
        cball = o.c + [0 0 o.h/2];
        dball = sqrt(sum((point - cball).^2, 2)) - (o.ball + margin);
        d = min(dshaft, dball);
    elseif strcmp(t,'nofly')
        % z 区间 [zlo-margin, zhi+margin]
        in = mu_in_poly(point(:,1:2), o.xz);
        zlo = o.zlo - margin; zhi = o.zhi + margin;
        d = inf(N,1);
        for i=1:N
            if in(i)
                if point(i,3) < zlo
                    d(i) = point(i,3) - zlo;
                elseif point(i,3) > zhi
                    d(i) = zhi - point(i,3);
                else
                    d(i) = -1e-3;
                end
            else
                d(i) = inf;
            end
        end
    elseif strcmp(t,'tree')
        %  + 
        rho = sqrt((point(:,1)-o.c(1)).^2 + (point(:,2)-o.c(2)).^2) - (o.r+margin);
        zlim = abs(point(:,3)-o.c(3)) - (o.h/2);
        d = max(rho, zlim);
    elseif strcmp(t,'terrain')
        % ??z +margin 
        zg = o.f(point(:,1), point(:,2));
        d = point(:,3) - zg - margin;   % <0 
    elseif strcmp(t,'bldg')
        % rich-building + L-shape foot + setback
        cx = o.c(1); cy = o.c(2); zB = o.c(3);
        hw = o.hw; pod = o.pod; baseH = o.baseH; topH = o.baseH + o.bodyH;
        hwP = hw + pod;
        sb = 0.1; if isfield(o,'setback') && ~isempty(o.setback), sb = o.setback; end
        hwT = hw * (1 - sb);
        cutFrac = 0.2; if isfield(o,'cutFrac') && ~isempty(o.cutFrac), cutFrac = o.cutFrac; end
        isL = isfield(o,'hasL') && o.hasL;
        if isL
            % L ??, y[-hwy, hwy*cutFrac]) + (x[-hwx, hwx*cutFrac], )
            dP = mu_lshape_dist(point, cx, cy, hwP(1), hwP(2), zB, zB+baseH, cutFrac);
            dT = mu_lshape_dist(point, cx, cy, hwT(1), hwT(2), zB+baseH, zB+topH, cutFrac);
            d = min(dP, dT) - margin;
        else
            % zB~zB+baseH 用裙楼半宽 hwP
            dbase = mu_box_dist(point, cx, cy, hwP(1), hwP(2), zB, zB+baseH);
            % zB+baseH~zB+topH 用塔身半宽 hwT
            dtop  = mu_box_dist(point, cx, cy, hwT(1), hwT(2), zB+baseH, zB+topH);
            d = min(dbase, dtop) - margin;
        end
    elseif strcmp(t,'bridge')
        %  deck??boxR  mu_draw_scene/drawBridge 
        %  pier 0??deckZ
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
                pr = o.pier(pi);                % struct: c(3), r, z1
                zc = pr.z1/2;
                rho = sqrt((point(:,1)-pr.c(1)).^2 + (point(:,2)-pr.c(2)).^2) - (pr.r+margin);
                zlim = abs(point(:,3)-zc) - (pr.z1/2);
                dPier = min(dPier, max(rho, zlim));
            end
        else
            dPier = inf(N,1);
        end
        d = max(dDeck, dPier);
    elseif strcmp(t,'streetlight')
        %  cylinder c(3) pole + box arm
        if isfield(o,'pole')
            pl = o.pole;                        % struct: c(3), r, h
            zc = pl.c(3) + pl.h/2;
            rho = sqrt((point(:,1)-pl.c(1)).^2 + (point(:,2)-pl.c(2)).^2) - (pl.r+margin);
            zlim = abs(point(:,3)-zc) - (pl.h/2);
            dPole = max(rho, zlim);
        else
            dPole = inf(N,1);
        end
        if isfield(o,'arm')
            ar = o.arm;                         % struct: c(3),hw(3)  box
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
        % water ??z  0 ??        % "" else -> d=inf??        % R1 ??        % ??0 terrainF ??>=0??        % ??= 0 - zz<0 dist = -depth ??pen ??
        in = mu_in_poly(point(:,1:2), o.xz);
        d = inf(N,1);
        for i=1:N
            if in(i)
                if point(i,3) < 0
                    d(i) = point(i,3);              % dist ??= -z
                else
                    d(i) = point(i,3);              % dist pen=0
                end
            end
        end
    elseif strcmp(t,'sign')
        %  postbox??  panelbox
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
% 将 inf 距离裁剪为有限大惩罚
dist(dist==inf) = 1e3;
% dist < 0 时 t > 0
t = max(0, -dist);
pen = t.^2 .* (1 + 5*exp(-t/3));
end

function in = mu_in_poly(xy, poly)
% mu_in_poly ????%   xy  : Nx2    poly : Mx2
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
% mu_box_dist: xy 平面半宽 hwx/hwy，z 区间 [zlo,zhi]
qx = abs(point(:,1)-cx) - hwx;
qy = abs(point(:,2)-cy) - hwy;
qz = abs(point(:,3)-(zlo+zhi)/2) - (zhi-zlo)/2;
outside = sqrt(sum(max([qx qy qz],0).^2, 2));
inside  = min(max([qx qy qz],[],2), 0);
d = outside + inside;
end

function d = mu_lshape_dist(point, cx, cy, hwx, hwy, zlo, zhi, cutFrac)
% mu_lshape_dist ??L ??foot??+x,+y ??%   poly  cutFrac mu_make_bldg  0.2??%   (, y[-hwy, hwy*cutFrac]) + (x[-hwx, hwx*cutFrac], )
%   z 取 [zlo,zhi] 最小值
if nargin<8 || isempty(cutFrac), cutFrac = 0.2; end
cutx = hwx*cutFrac; cuty = hwy*cutFrac;
% x[-hwx,hwx], y[-hwy, cuty]
dB = mu_box_dist(point, cx, cy, hwx, cuty, zlo, zhi);
% x[-hwx, cutx], y[cuty, hwy]
dS = mu_box_dist(point, cx, cy, cutx, hwy, zlo, zhi);
d = min(dB, dS);
end
