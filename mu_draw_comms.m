function mu_draw_comms(ax, comms, sensors)
% drawCommsLayer — 阶段E：渲染通信网络（基站/中继/终端 + 覆盖球）与传感器挂载点
%   独立文件主函数，供 mu_draw_scene 调用；句柄汇入 ax.UserData.commsHandles 便于清理。
% 风格：白底专业工程风，与 mu_draw_scene 一致。
%   gNB  ：冷灰蓝竖杆 + 三向天线阵 + 半透明青色覆盖球
%   relay：橙黄菱形（mesh 节点）+ 细覆盖环
%   iot  ：暖绿小球（末端感知）
%   sensors：按类型用不同标记（相机=方块 / 气象=三角 / 噪声=菱形 / LiDAR=六边）

hold(ax,'on');
handles = gobjects(0);

if nargin < 3 || isempty(sensors), sensors = struct(); end

% ---------- 通信节点 ----------
if ~isempty(comms)
    for k = 1:numel(comms)
        nd = comms(k);
        cx = nd.c(1); cy = nd.c(2); cz = nd.c(3);
        if strcmp(nd.type,'gNB')
            % 竖杆
            h = plot3(ax, [cx cx], [cy cy], [cz-nd.antH cz], ...
                'Color',[0.30 0.45 0.65], 'LineWidth',0.8);
            handles(end+1) = h;
            % 天线阵（顶端单横，视觉减负）
            h = plot3(ax, [cx-1.5 cx+1.5], [cy cy], [cz cz], ...
                'Color',[0.20 0.35 0.55], 'LineWidth',0.9);
            handles(end+1) = h;
            % 覆盖球（更淡，避免覆盖全场景）
            [sx,sy,sz] = sphere(14);
            h = surf(ax, cx+sx*nd.covR, cy+sy*nd.covR, cz+sz*nd.covR*0.5, ...
                'FaceAlpha',0.018, 'EdgeAlpha',0.03, 'FaceColor',[0.25 0.70 0.80]);
            handles(end+1) = h;
            % 覆盖范围圆环（贴地，参考线，更专业）
            th = linspace(0,2*pi,48);
            zr = cz - nd.antH*0.5;
            h = plot3(ax, cx+nd.covR*cos(th), cy+nd.covR*sin(th), zr*ones(size(th)), ...
                'Color',[0.25 0.70 0.80],'LineWidth',0.35,'LineStyle',':');
            handles(end+1) = h;
        elseif strcmp(nd.type,'relay')
            % 菱形节点（落点）
            h = scatter3(ax, cx, cy, cz, 55, [0.95 0.70 0.25], 'filled', ...
                'Marker','diamond', 'MarkerEdgeColor',[0.6 0.45 0.1], 'LineWidth',0.6);
            handles(end+1) = h;
            % 覆盖环（贴地近似圆柱）
            th = linspace(0,2*pi,40);
            zr = cz - nd.antH*0.5;
            h = plot3(ax, cx+nd.covR*cos(th), cy+nd.covR*sin(th), zr*ones(size(th)), ...
                'Color',[0.95 0.70 0.25], 'LineWidth',0.6, 'LineStyle','--');
            handles(end+1) = h;
        else % iot
            h = scatter3(ax, cx, cy, cz, 24, [0.40 0.80 0.45], 'filled', ...
                'Marker','o', 'MarkerEdgeColor',[0.20 0.50 0.25]);
            handles(end+1) = h;
        end
    end
end

% ---------- 传感器挂载点 ----------
if ~isempty(sensors)
    for k = 1:numel(sensors)
        sn = sensors(k);
        cx = sn.c(1); cy = sn.c(2); cz = sn.c(3);
        if strcmp(sn.type,'cam')
            col = [0.55 0.55 0.60];
        elseif strcmp(sn.type,'met')
            col = [0.35 0.65 0.85];
        elseif strcmp(sn.type,'noise')
            col = [0.85 0.45 0.55];
        else % lidar
            col = [0.70 0.55 0.85];
        end
        % 杆 + 传感头
        h = plot3(ax, [cx cx], [cy cy], [cz-sn.range*0.05 cz], ...
            'Color',[0.45 0.48 0.52], 'LineWidth',0.8);
        handles(end+1) = h;
        h = scatter3(ax, cx, cy, cz, 28, col, 'filled', ...
            'Marker','^', 'MarkerEdgeColor',[0.2 0.2 0.25], 'LineWidth',0.5);
        handles(end+1) = h;
    end
end

ax.UserData.commsHandles = handles;
end
