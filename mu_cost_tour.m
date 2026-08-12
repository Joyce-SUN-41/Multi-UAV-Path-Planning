function cost = mu_cost_tour(x, scene)
% mu_cost_tour : tour-mode CA cost = fhd(x, scene)
% x is (dim x N) or (dim x 1)
%   x = [ control segments | task-ordering keys ]
%   control: nUAV * nCtrl * 3
%   keys:    nUAV * maxTaskPerUAV
% B = boundary / tour
% L1 CA dim / pop assert
assert(size(x,2) <= 1 || size(x,2) >= 1, 'mu_cost_tour: ');
if size(x,2) > 1
    N = size(x,2);
    cost = zeros(1,N);
    for j = 1:N
        cost(j) = mu_cost_tour_col(x(:,j), scene);
    end
    return;
end
cost = mu_cost_tour_col(x, scene);
end

function cost = mu_cost_tour_col(x, scene)
x = x(:).';
nU = scene.nUAV;
maxT = max(cellfun(@numel, scene.taskAssign));
% R20 ctrlPer(k) = 2*(...) ; dimCtrl = sum(ctrlPer*3) ; set in mu_run_planner / mu_decode
dimCtrl = scene.dimCtrl;
expected = dimCtrl + nU*maxT;
if numel(x) ~= expected
    error('mu_cost_tour: dim %d expected %d (nUAV=%d, dimCtrl=%d, maxT=%d) ', ...
        numel(x), expected, nU, dimCtrl, maxT);
end

ctrlSeg = x(1:dimCtrl);
keySeg  = x(dimCtrl+1 : end);
keyMat  = reshape(keySeg, maxT, nU).';     % nU x maxT

% R22 bounds for separation ; b = scene.bounds
b = scene.bounds;
lo = b([1 3 5]); hi = b([2 4 6]);

trajs = cell(nU,1);
cost = 0;
off = 0;                                    % R20 control offset

for k = 1:nU
    ck = scene.ctrlPer(k);
    xi = ctrlSeg(off + 1 : off + ck*3);
    ctrl = reshape(xi, 3, ck).';           % ck x 3
    off = off + ck*3;

    tIdx = scene.taskAssign{k};
    orderingPen = 0;
    if isempty(tIdx)
        % start==goal : depot resident ; no cost
        idle = true;
        traj = repmat(scene.starts(k,:), max(2, scene.smooth), 1);
        len = 0; sm = 0; bp = 0; tpen = 0; pen = 0;
    else
        idle = false;
        keys = keyMat(k, 1:numel(tIdx));
        [~, ord] = sort(keys);
        waypts = scene.tasks(tIdx(ord), :);
        traj = mu_build_tour_traj(ctrl, scene.starts(k,:), scene.goals(k,:), ...
                                  waypts, scene.smooth, ck);
        % R15 w.ordering ; w.length ; w.ordering=0
        seq = [scene.starts(k,:); waypts; scene.goals(k,:)];
        dseq = sqrt(sum(diff(seq,1,1).^2, 2));
        orderingPen = sum(dseq);
    end
    trajs{k} = traj;

    % arc-time (mu_arc_time) for p2p-style timing
    tk = mu_arc_time(traj, scene.T_horizon);

    if ~idle
        seg = sqrt(sum(diff(traj,1,1).^2, 2));
        len = sum(seg);
        if size(traj,1)>=3
            d2 = diff(traj,2,1); sm = sum(sqrt(sum(d2.^2,2)));
        else, sm = 0; end
        % terrain obstacle ; penalty ; p2p/mu_eval_path
        % M3 scene.safeMargin
        obTypes = {scene.obstacles.type};
        obsSolid = scene.obstacles(~strcmp(obTypes, 'terrain'));
        [~, pen] = mu_obstacle_dist(traj, obsSolid, scene.safeMargin);
        lo=b([1 3 5]); hi=b([2 4 6]);   % b R22 bounds
        out = max(0, lo - traj) + max(0, traj - hi);
        bp = sum(out(:).^2);
        % z terrain penalty
        tpen = 0;
        if ~isempty(scene.terrainF)
            zg = scene.terrainF(traj(:,1), traj(:,2));
            tpen = sum(max(0, zg + scene.terrainMargin - traj(:,3)));
        end
        % DD1 tk -> mu_obstacle_dist_t ; w.vehicle
        if scene.w.vehicle > 0 && isfield(scene,'dynamics') && ~isempty(scene.dynamics) ...
           && isfield(scene.dynamics,'vehicles') && ~isempty(scene.dynamics.vehicles)
            [~, vpen] = mu_obstacle_dist_t(traj, scene, tk, scene.vehMargin);
            cost = cost + scene.w.vehicle * sum(vpen);
        end
        % E comms ; w.comms=0 disables
        if scene.w.comms > 0
            cpen = mu_comms_penalty(traj, scene.comms, scene.commsVZ);
            cost = cost + scene.w.comms * cpen;
        end
    end
    cost = cost + scene.w.length*len + scene.w.smooth*sm ...
                + scene.w.obstacle*(sum(pen(:)) + 2*tpen) + scene.w.boundary*bp ...
                + scene.w.ordering*orderingPen;
end

% R14 time grid tk ; qGrid linspace(0,1)
% [0,T_horizon] with 40 samples ; R5 interp1 'linear','extrap' NaN-safe
L = max(20, ceil(scene.smooth/3));
qGrid = linspace(0, scene.T_horizon, L);
R = zeros(L,3,nU);
for k=1:nU
    tk = mu_arc_time(trajs{k}, scene.T_horizon);
    if all(tk < 1e-9)                       % start==goal
        R(:,:,k) = repmat(trajs{k}(1,:), L, 1);   % interp1 needs >1 distinct x
    else
        % tk interp1 ; unique stable to avoid repeated x
        [tkU, ia] = unique(tk, 'stable');
        if numel(tkU) < 2
            R(:,:,k) = repmat(trajs{k}(1,:), L, 1);
        else
            R(:,:,k) = interp1(tkU, trajs{k}(ia,:), qGrid, 'linear', 'extrap');
        end
    end
end
% R22 separation threshold ; nUAV=0 -> "b" bounds
sepThresh = 0.06 * norm([b(2)-b(1), b(4)-b(3), b(6)-b(5)]);
sepPen = 0;
for t=1:L
    % skip endpoints
    if t==1 || t==L, continue; end
    for a=1:nU-1
        for b2=a+1:nU
            dd = norm(R(t,:,a) - R(t,:,b2));
            if dd < sepThresh
                % R23 depot-resident exemption
                dDepotA = norm(R(t,:,a) - scene.starts(a,:));
                dDepotB = norm(R(t,:,b2) - scene.starts(b2,:));
                if dDepotA < sepThresh*0.5 || dDepotB < sepThresh*0.5
                    continue;
                end
                sepPen = sepPen + (sepThresh - dd)^2;
            end
        end
    end
end
cost = cost + scene.w.separation * sepPen;
cost = cost + 1e-4 * sum(x.^2);   % L2 regularization

end
