function mu_draw_dynamic(ax, dyn, tCur)
% ??tCur ??% ??UAV /??% ????GUI ??hold(ax,'on');
hDyn = gobjects(0);
for vi=1:dyn.count
  v = dyn.vehicles(vi);
  [xy, zc] = mu_road_xy(dyn, vi, tCur);    %  xy ??zc
  zBox = zc + v.height/2;                    % M5 ??  h = drawVehicle(ax, xy(1), xy(2), zBox, v.length/2, v.width/2, v.height/2, v.phi);
  h = drawVehicle(ax, xy(1), xy(2), zBox, v.length/2, v.width/2, v.height/2, v.phi);
  hDyn = [hDyn; h(:)];
end
%  GUI 
ax.UserData.dynHandles = hDyn;
end

function h = drawVehicle(ax, cx, cy, cz, hwx, hwy, hz, phi)
% ??Z ??phi 8  + ??% cz=+z ??hold(ax,'on');
c=cos(phi); s=sin(phi);
% ??8 x[-hwx,hwx], y[-hwy,hwy], z[cz-hz,cz+hz]
lx=[-hwx hwx hwx -hwx -hwx hwx hwx -hwx];
ly=[-hwy -hwy hwy hwy -hwy -hwy hwy hwy];
lz=[cz-hz cz-hz cz-hz cz-hz cz+hz cz+hz cz+hz cz+hz];
% 
wx= cx + lx*c - ly*s;
wy= cy + lx*s + ly*c;
wz= lz;
bodyCol=[0.42 0.47 0.55];
edgeC=[0.30 0.34 0.40];      % 车辆描边色（此前仅写在注释里未赋值）
facelist={[5 6 7 8],[1 2 6 5],[2 3 7 6],[3 4 8 7],[4 1 5 8]};
shade=[1.10 0.94 0.82 0.88 0.96];
h = gobjects(0);
for f=1:5
  p = patch(ax,'Vertices',[wx; wy; wz].','Faces',facelist{f},'FaceColor',min(1,bodyCol*shade(f)), ...
        'EdgeColor',edgeC,'EdgeAlpha',0.25,'LineWidth',0.4,'FaceLighting','none');
  h(end+1) = p;
end
% ??lg = plot3(ax,[wx(5) wx(6) wx(7) wx(8) wx(5)],[wy(5) wy(6) wy(7) wy(8) wy(5)],[wz(5) wz(6) wz(7) wz(8) wz(5)], ...
%      'Color',[0.95 0.55 0.20],'LineWidth',1.4,'LineStyle','-');
% h(end+1) = lg;   % 弃用：顶部轮廓线（lg 定义行已注释），车辆 body 已由上面 patch 绘制
end
