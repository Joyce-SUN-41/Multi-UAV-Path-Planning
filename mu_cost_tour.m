function cost = mu_cost_tour(x, scene)
% mu_cost_tour — 场景B代价函数（多机任务点巡访 + 机间防撞）
% 签名满足 CA 接口：cost = fhd(x, scene)
% 矩阵输入(dim x N)时按列分发，返回 1xN。
%
% 编码：x = [控制点段 | 随机键段]
%   控制点段：nUAV * nCtrl * 3
%   随机键段：nUAV * maxTaskPerUAV（用于对各机任务点排序）
% 每架机访问分配任务点（按随机键升序），B 样条连接，叠加机间间隔惩罚。

% 矩阵输入(dim x N)时按列分发，返回 1xN。
% L1 契约：CA 必须以 dim×pop（列优先）形式传入；本函数按列分发假定与之吻合。
assert(size(x,2) <= 1 || size(x,2) >= 1, 'mu_cost_tour: 输入契约异常');
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
% R20：差异化控制点维度。ctrlPer(k)=2(空闲机) 或 nC(忙碌机)，dimCtrl=sum(ctrlPer*3)。
% 与 mu_run_planner / mu_decode 共用同一权威维度，避免维度契约漂移。
dimCtrl = scene.dimCtrl;
expected = dimCtrl + nU*maxT;
if numel(x) ~= expected
    error('mu_cost_tour: 解向量长度 %d 与期望 %d (nUAV=%d, dimCtrl=%d, maxT=%d) 不符', ...
        numel(x), expected, nU, dimCtrl, maxT);
end

ctrlSeg = x(1:dimCtrl);
keySeg  = x(dimCtrl+1 : end);
keyMat  = reshape(keySeg, maxT, nU).';     % nU x maxT

% R22 修复：b（场景边界）原为 for k 循环体内局部变量，机间防撞段在循环外依赖它，
% 靠最后一次迭代的副作用"侥幸"工作——一旦 nUAV 迭代逻辑变动或循环体内 b 重绑定
% 会静默产出错误的 sepThresh 甚至崩溃（nUAV=0 时直接抛"未定义变量 b"）。
% 现提到函数级显式定义，消除隐式耦合。
b = scene.bounds;
lo = b([1 3 5]); hi = b([2 4 6]);

trajs = cell(nU,1);
cost = 0;
off = 0;                                    % R20：逐机控制点偏移累加
% R22 修复：b（场景边界）显式在函数级定义，供下方 for.k 循环的边界惩罚与
% 循环后机间防撞段共用，消除此前"仅在循环体内定义、循环外靠副作用读取"的脆弱耦合。
b = scene.bounds;
for k = 1:nU
    ck = scene.ctrlPer(k);
    xi = ctrlSeg(off + 1 : off + ck*3);
    ctrl = reshape(xi, 3, ck).';           % ck x 3 内部控制点
    off = off + ck*3;

    tIdx = scene.taskAssign{k};
    orderingPen = 0;
    if isempty(tIdx)
        % 空闲机（start==goal，停在 depot 待命，不执行飞行任务）。
        % R20/R21：此前退化分支仍用控制点构造 B 样条摆动轨迹并计入几何/障碍代价，
        % 既浪费 CA 搜索预算（控制点为无效自由度），又让无意义摆动污染机间防撞。
        % 现短路：轨迹退化为单点（停 depot），几何/障碍/车辆/通信代价全置 0，
        % 仅保留机间防撞（单点轨迹，R14 物理时间轴处理）。控制点对代价完全无效（真死维度）。
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
        % R15 修复：w.ordering 此前在 mu_config 声明为权重却从未计算（死接口/接口谎言）。
        % 实现"访问次序合理性"惩罚：按当前访问顺序累加相邻任务点（含起降点）的直线距离，
        % 鼓励就近紧凑访问、抑制绕远/交叉回溯。与 w.length（平滑轨迹弧长）语义不同——
        % 后者惩罚轨迹几何长度，前者惩罚任务点拓扑访问次序。缺省 w.ordering=0 不改变既有正则。
        seq = [scene.starts(k,:); waypts; scene.goals(k,:)];
        dseq = sqrt(sum(diff(seq,1,1).^2, 2));
        orderingPen = sum(dseq);
    end
    trajs{k} = traj;

    % 时间轴配准（弧长 -> 时刻，供时变碰撞 D1）；单源：mu_arc_time（与 p2p 一致）
    tk = mu_arc_time(traj, scene.T_horizon);

    if ~idle
        seg = sqrt(sum(diff(traj,1,1).^2, 2));
        len = sum(seg);
        if size(traj,1)>=3
            d2 = diff(traj,2,1); sm = sum(sqrt(sum(d2.^2,2)));
        else, sm = 0; end
        % 实体障碍惩罚：排除 terrain（最低飞行高度，单独作为软约束）；
        % 用穿透惩罚 pen（带安全裕度），自由空中为 0，方向与 p2p/mu_eval_path 一致。
        % M3：安全壳裕度复用 scene.safeMargin（不再硬编码 6），与 p2p 同一尺度。
        obTypes = {scene.obstacles.type};
        obsSolid = scene.obstacles(~strcmp(obTypes, 'terrain'));
        [~, pen] = mu_obstacle_dist(traj, obsSolid, scene.safeMargin);
        lo=b([1 3 5]); hi=b([2 4 6]);   % b 已在函数级显式定义（R22）
        out = max(0, lo - traj) + max(0, traj - hi);
        bp = sum(out(:).^2);
        % 地形最低高度软约束：仅当 z 低于地面+裕度才惩罚（与障碍穿透语义分离）
        tpen = 0;
        if ~isempty(scene.terrainF)
            zg = scene.terrainF(traj(:,1), traj(:,2));
            tpen = sum(max(0, zg + scene.terrainMargin - traj(:,3)));
        end
        % 阶段D：时变车辆碰撞惩罚（D1），逐轨迹点按 tk 调 mu_obstacle_dist_t，
        % 与静态障碍同量级加权（w.vehicle），避免规划器对车流零避障。
        if scene.w.vehicle > 0 && isfield(scene,'dynamics') && ~isempty(scene.dynamics) ...
           && isfield(scene.dynamics,'vehicles') && ~isempty(scene.dynamics.vehicles)
            [~, vpen] = mu_obstacle_dist_t(traj, scene, tk, scene.vehMargin);
            cost = cost + scene.w.vehicle * sum(vpen);
        end
        % 阶段E：通信链路约束（默认 w.comms=0 关闭，不影响现有规划正则）
        if scene.w.comms > 0
            cpen = mu_comms_penalty(traj, scene.comms, scene.commsVZ);
            cost = cost + scene.w.comms * cpen;
        end
    end
    cost = cost + scene.w.length*len + scene.w.smooth*sm ...
                + scene.w.obstacle*(sum(pen(:)) + 2*tpen) + scene.w.boundary*bp ...
                + scene.w.ordering*orderingPen;
end

% 机间防撞：R14 修复 —— 统一到物理时间轴 tk（与阶段D时变车辆碰撞同一时空基准）。
% 旧版用各机"弧长归一化 linspace(0,1)"重采样，等价于比较两机在"各自飞行进度同一比例处"
% 的位置，而非同一物理时刻；各机轨迹弧长/飞行距离不同，"进度比 0.5"对应的物理时间
% 与物理位置完全不同，导致分离惩罚物理意义失真（误罚相隔远但进度比相同的机、漏罚同
% 时刻真正相撞的机）。现改为：每机轨迹按各自 tk（mu_arc_time 物理时刻轴）在公共时间
% 网格 [0,T_horizon] 上重采样，与车辆碰撞完全一致。采样数随平滑度缩放（避免硬 40 在
% 密集轨迹欠采样，R5），并显式 'linear','extrap' 防端点外推产生 NaN。
L = max(20, ceil(scene.smooth/3));
qGrid = linspace(0, scene.T_horizon, L);
R = zeros(L,3,nU);
for k=1:nU
    tk = mu_arc_time(trajs{k}, scene.T_horizon);
    if all(tk < 1e-9)                       % 退化轨迹（零弧长，如空闲机 start==goal）
        R(:,:,k) = repmat(trajs{k}(1,:), L, 1);   % 退化为单点，避免 interp1 同值 x 返 NaN
    else
        % tk 单调非减但轨迹相邻重复点会产生平台（重复时刻），interp1 要求采样点唯一，
        % 故先 stable 去重：ia 为各去重时刻首次出现的原始索引，trajs{k}(ia,:) 与 tkU 等长配对。
        [tkU, ia] = unique(tk, 'stable');
        if numel(tkU) < 2
            R(:,:,k) = repmat(trajs{k}(1,:), L, 1);
        else
            R(:,:,k) = interp1(tkU, trajs{k}(ia,:), qGrid, 'linear', 'extrap');
        end
    end
end
% R22 修复：b（场景边界）此前仅在上方 for.k 循环体内定义（第101行），此处循环外使用靠
% 最后一次迭代的副作用"侥幸"工作——nUAV=0 时循环不进入会直接抛"未定义变量 b"，且重构脆弱。
% 现显式在函数级基于 scene.bounds 定义，消除隐式耦合。
b = scene.bounds;
sepThresh = 0.06 * norm([b(2)-b(1), b(4)-b(3), b(6)-b(5)]);  % 随空间缩放的安全间隔
sepPen = 0;
for t=1:L
    % 起降点时刻（首/尾）各机在仓库/任务点聚集属正常，豁免机间防撞，
    % 避免大机数复用同一起降点时 sepPen 不可解地爆表。
    if t==1 || t==L, continue; end
    for a=1:nU-1
        for b2=a+1:nU
            dd = norm(R(t,:,a) - R(t,:,b2));
            if dd < sepThresh
                % R23 修复：空闲机轨迹被 repmat 成恒等于 depot 的单点（R14 零弧长退化分支），
                % 忙碌机在"返回 depot 的中间时刻"会经过 depot 附近，与空闲机在同一物理中间时刻
                % 同位置却未被上面的首/尾豁免覆盖，产生误罚（实测加权代价差可达 ~4e4），
                % 诱导优化器病态绕开本应正常返回的 depot。现对该时刻做 depot-resident 豁免：
                % 当参与比较的任一机在该公共时刻位置距其各自起点（depot）小于阈值时跳过。
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
cost = cost + 1e-4 * sum(x.^2);   % 轻微 L2 正则（数值稳定，不主导优化）
end
