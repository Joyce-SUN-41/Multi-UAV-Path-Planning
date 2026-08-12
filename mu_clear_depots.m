function depots = mu_clear_depots(depots, obstacles, margin)
% mu_clear_depots (R28)(depot)??%   ??mu_depots)??mu_city_layout)??%    depot /
%   ???? idle ??R21 )
%   ( easy/hard ??depot ??idle  ~3000~4000)??%    xy ??depot ??z(????%   ??bldg/tower/nofly/tree/streetlight/sign/bridge/sphere/cylinder/row)
%   /xy ??%   (terrain)??scene.terrainMargin ??%   margin ??scene.safeMargin + ??depot ??%
%   depots   : nD x 3 (??xy z)
%   obstacles:  struct (terrain ??
%   margin   : ??m)
%    nD x 3 ??
obTypes = {obstacles.type};
obsSolid = obstacles(~strcmp(obTypes, 'terrain'));   % ??xy 
if isempty(obsSolid)
    return;
end

half = max(abs(depots(:,1:2)), [], 'all');   % 从 depot 自身坐标推断半范围（depot 由 mu_depots 用 200*ENV_SCALE 生成，自动跟随缩放）
buf  = 5 * (half / 200);                        % 推出缓冲，随 ENV_SCALE 缩放（原 5*2.5 的尺度无关写法）
for k = 1:size(depots, 1)
    p = depots(k, :);
    converged = false;
    for iter = 1:80  % (??)
        [d, ~] = mu_obstacle_dist(p, obsSolid, margin);
        if d >= 0
            converged = true;
            break;   % ??        end
        % ""
        dx = 0; dy = 0;
        for oi = 1:numel(obsSolid)
            o = obsSolid(oi);
            [do, ~] = mu_obstacle_dist(p, o, margin);
            if do < 0
                % (nofly )
                if strcmp(lower(o.type), 'nofly') && ~isempty(o.xz)
                    cx = mean(o.xz(:, 1)); cy = mean(o.xz(:, 2));
                else
                    cx = o.c(1); cy = o.c(2);
                end
                vx = p(1) - cx; vy = p(2) - cy;
                nv = sqrt(vx^2 + vy^2) + 1e-6;
                step = (-do) + buf;            % ??buf ??                dx = dx + (vx / nv) * step;
                dy = dy + (vy / nv) * step;
            end
        end
        p(1) = p(1) + dx;
        p(2) = p(2) + dy;
        % (??5m )
        p(1) = min(max(p(1), -half + 5), half - 5);
        p(2) = min(max(p(2), -half + 5), half - 5);
    end
    %  depot ??hard ??
    if ~converged
        % ??half*0.85)??
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
