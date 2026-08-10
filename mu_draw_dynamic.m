function mu_draw_dynamic(ax, dyn, tCur)
% 按当前时刻 tCur 渲染所有动态车辆（贴地行驶的长方体）。
% 与 UAV 同处世界坐标系，属于地面/低空冲突源，便于时间轴回放观察
% 无人机与车流的空间-时间冲突。独立文件以便 GUI 时间轴逐帧直接调用。
hold(ax,'on');
hDyn = gobjects(0);
for vi=1:dyn.count
  v = dyn.vehicles(vi);
  [xy, zc] = mu_road_xy(dyn, vi, tCur);    % 当前中心 xy 与地形标高 zc
  zBox = zc + v.height/2;                    % 盒中心高度（随地形，M5 修复）
  h = drawVehicle(ax, xy(1), xy(2), zBox, v.length/2, v.width/2, v.height/2, v.phi);
  hDyn = [hDyn; h(:)];
end
% 记录句柄，供 GUI 时间轴逐帧更新时删除并重绘
ax.UserData.dynHandles = hDyn;
end

function h = drawVehicle(ax, cx, cy, cz, hwx, hwy, hz, phi)
% 旋转长方体车辆（绕 Z 轴 phi）。用 8 顶点 + 旋转矩阵绘制，车体冷灰蓝，
% 顶部一道暖橙行车灯带以区别于建筑。底面贴地（cz=地面+z 半高）。
hold(ax,'on');
c=cos(phi); s=sin(phi);
% 局部 8 顶点（x∈[-hwx,hwx], y∈[-hwy,hwy], z∈[cz-hz,cz+hz]）
lx=[-hwx hwx hwx -hwx -hwx hwx hwx -hwx];
ly=[-hwy -hwy hwy hwy -hwy -hwy hwy hwy];
lz=[cz-hz cz-hz cz-hz cz-hz cz+hz cz+hz cz+hz cz+hz];
% 旋转到世界系
wx= cx + lx*c - ly*s;
wy= cy + lx*s + ly*c;
wz= lz;
bodyCol=[0.42 0.47 0.55];      % 冷灰蓝车体
edgeC=[0.30 0.34 0.40];
facelist={[5 6 7 8],[1 2 6 5],[2 3 7 6],[3 4 8 7],[4 1 5 8]};
shade=[1.10 0.94 0.82 0.88 0.96];
h = gobjects(0);
for f=1:5
  p = patch(ax,'Vertices',[wx; wy; wz].','Faces',facelist{f},'FaceColor',min(1,bodyCol*shade(f)), ...
        'EdgeColor',edgeC,'EdgeAlpha',0.25,'LineWidth',0.4,'FaceLighting','none');
  h(end+1) = p;
end
% 顶灯带（暖橙，标识动态体）
lg = plot3(ax,[wx(5) wx(6) wx(7) wx(8) wx(5)],[wy(5) wy(6) wy(7) wy(8) wy(5)],[wz(5) wz(6) wz(7) wz(8) wz(5)], ...
      'Color',[0.95 0.55 0.20],'LineWidth',1.4,'LineStyle','-');
h(end+1) = lg;
end
