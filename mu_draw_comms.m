function mu_draw_comms(ax, comms, sensors)
% drawCommsLayer ??E??/ + ??%   ??mu_draw_scene ??ax.UserData.commsHandles ??%  mu_draw_scene ??%   gNB   + ??+ ??%   relaymesh ?? 
%   iot  ??%   sensors= / = / = / LiDAR=??
hold(ax,'on');
handles = gobjects(0);

if nargin < 3 || isempty(sensors), sensors = struct(); end

% ----------  ----------
if ~isempty(comms)
    for k = 1:numel(comms)
        nd = comms(k);
        cx = nd.c(1); cy = nd.c(2); cz = nd.c(3);
        if strcmp(nd.type,'gNB')
            % 
            h = plot3(ax, [cx cx], [cy cy], [cz-nd.antH cz], ...
                'Color',[0.30 0.45 0.65], 'LineWidth',0.8);
            handles(end+1) = h;
            % 
            h = plot3(ax, [cx-1.5 cx+1.5], [cy cy], [cz cz], ...
                'Color',[0.20 0.35 0.55], 'LineWidth',0.9);
            handles(end+1) = h;
            [sx,sy,sz] = sphere(14);
            h = surf(ax, cx+sx*nd.covR, cy+sy*nd.covR, cz+sz*nd.covR*0.5, ...
                'FaceAlpha',0.018, 'EdgeAlpha',0.03, 'FaceColor',[0.25 0.70 0.80]);
            handles(end+1) = h;
            th = linspace(0,2*pi,48);
            zr = cz - nd.antH*0.5;
            h = plot3(ax, cx+nd.covR*cos(th), cy+nd.covR*sin(th), zr*ones(size(th)), ...
                'Color',[0.25 0.70 0.80],'LineWidth',0.35,'LineStyle',':');
            handles(end+1) = h;
        elseif strcmp(nd.type,'relay')
            % 
            h = scatter3(ax, cx, cy, cz, 55, [0.95 0.70 0.25], 'filled', ...
                'Marker','diamond', 'MarkerEdgeColor',[0.6 0.45 0.1], 'LineWidth',0.6);
            handles(end+1) = h;
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

% ----------  ----------
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
