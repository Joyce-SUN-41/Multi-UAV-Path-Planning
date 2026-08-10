function depots = mu_clear_depots(depots, obstacles, margin)
% mu_clear_depots —第十轮(R28)修复：将仓库(depot)推出所有实体障碍安全壳。
%   程序化仓库坐标(mu_depots)与随机城市障碍(mu_city_layout)独立采样，
%   无任何避障校验，导致部分 depot 落在建筑/禁飞区安全壳内——规划器被迫
%   从"穿透态"出发，且 idle 机(R21 单点轨迹)被障碍代价持续不公平惩罚
%   (实测 easy/hard 单 depot 穿透使全 idle 机障碍惩罚达 ~3000~4000)。
%   本函数在 xy 平面将每个 depot 平移到最近障碍安全壳之外，保留 z(巡航层)。
%   所有实体障碍(bldg/tower/nofly/tree/streetlight/sign/bridge/sphere/cylinder/row)
%   均为竖直柱状/多边形，xy 推出即可解除穿透，不影响飞行高度语义。
%   地形(terrain)不参与——它由 scene.terrainMargin 单独约束最低飞行高度。
%   margin 取 scene.safeMargin + 缓冲，确保 depot 退出后仍留足安全间距。
%
%   depots   : nD x 3 仓库坐标(前两列 xy、第三列 z)
%   obstacles: 障碍 struct 数组(须已含全部实体障碍，terrain 被忽略)
%   margin   : 安全壳外扩(m)
%   返回推出后的 nD x 3 仓库坐标。

obTypes = {obstacles.type};
obsSolid = obstacles(~strcmp(obTypes, 'terrain'));   % 地形不参与 xy 推出
if isempty(obsSolid)
    return;
end

half = 200 * 1.6;   % 与 mu_config bounds 一致的边界半宽（ENV_SCALE=1.6 与 mu_city_layout 同步）
buf  = 5 * 1.6;       % 退出壳后额外留白(m)（随环境放大）

for k = 1:size(depots, 1)
    p = depots(k, :);
    converged = false;
    for iter = 1:80  % 迭代收敛(多障碍叠加/边角卡死时需要更多轮)
        [d, ~] = mu_obstacle_dist(p, obsSolid, margin);
        if d >= 0
            converged = true;
            break;   % 已在所有安全壳外
        end
        % 逐障碍求"远离中心"的水平退出位移并累加
        dx = 0; dy = 0;
        for oi = 1:numel(obsSolid)
            o = obsSolid(oi);
            [do, ~] = mu_obstacle_dist(p, o, margin);
            if do < 0
                % 远离障碍中心(竖直障碍用底面中心；nofly 用多边形质心)
                if strcmp(lower(o.type), 'nofly') && ~isempty(o.xz)
                    cx = mean(o.xz(:, 1)); cy = mean(o.xz(:, 2));
                else
                    cx = o.c(1); cy = o.c(2);
                end
                vx = p(1) - cx; vy = p(2) - cy;
                nv = sqrt(vx^2 + vy^2) + 1e-6;
                step = (-do) + buf;            % 超出壳 buf 米
                dx = dx + (vx / nv) * step;
                dy = dy + (vy / nv) * step;
            end
        end
        p(1) = p(1) + dx;
        p(2) = p(2) + dy;
        % 约束在搜索空间内(留 5m 边距)
        p(1) = min(max(p(1), -half + 5), half - 5);
        p(2) = min(max(p(2), -half + 5), half - 5);
    end
    % 兜底：未收敛时把 depot 推到边界安全带角点(hard 难度密集区偶尔卡死)
    if ~converged
        % 沿对角线向最近角点(±half*0.85)逃逸，再跑一次推出
        corner = sign([p(1) p(2)]) .* (half - 20);
        p(1:2) = corner;
        for iter = 1:30
            [d, ~] = mu_obstacle_dist(p, obsSolid, margin);
            if d >= 0, break; end
            dx = 0; dy = 0;
            for oi = 1:numel(obsSolid)
                o = obsSolid(oi);
                [do, ~] = mu_obstacle_dist(p, o, margin);
                if do < 0
                    if strcmp(lower(o.type), 'nofly') && ~isempty(o.xz)
                        cx = mean(o.xz(:, 1)); cy = mean(o.xz(:, 2));
                    else
                        cx = o.c(1); cy = o.c(2);
                    end
                    vx = p(1) - cx; vy = p(2) - cy;
                    nv = sqrt(vx^2 + vy^2) + 1e-6;
                    step = (-do) + buf;
                    dx = dx + (vx / nv) * step;
                    dy = dy + (vy / nv) * step;
                end
            end
            p(1) = p(1) + dx;
            p(2) = p(2) + dy;
            p(1) = min(max(p(1), -half + 5), half - 5);
            p(2) = min(max(p(2), -half + 5), half - 5);
        end
    end
    depots(k, :) = p;
end
end
