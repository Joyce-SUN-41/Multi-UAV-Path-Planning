function [bestX, bestCost, curve, trajs, scene] = mu_run_planner(mode, varargin)
% mu_run_planner — 多无人机路径规划统一入口（接入 CA 算法）
%   [bestX, bestCost, curve, trajs, scene] = mu_run_planner(mode, ...)
%     mode     : 'p2p' | 'tour'
%   可选 Name/Value:
%     pop, iter, maxFE, nCtrl, nUAV, difficulty, seed, caFun (默认 @CAv9x)
%
% 返回值：
%   bestX   : 最优解向量（控制点+随机键）
%   bestCost: 最优代价
%   curve   : CA 收敛曲线
%   trajs   : cell{nUAV} 各机最优采样轨迹 (n x 3)
%   scene   : 使用的场景配置
%
% 该入口「以接口调用形式」接入 CA：把场景通过 varargin 透传给代价函数，
% CA 内部以 feval(fhd, x, scene) 调用，代价函数签名 = fhd(x, scene)。

% ---- 解析可选参数 ----
p = inputParser;
addParameter(p,'pop',40);
addParameter(p,'iter',120);
addParameter(p,'maxFE',60000);
addParameter(p,'nCtrl',5);
addParameter(p,'nUAV',3);
addParameter(p,'difficulty','medium');
addParameter(p,'seed',0);
addParameter(p,'caFun',@CAv9x);
parse(p, varargin{:});
opt = p.Results;

% ---- 构造场景 ----
if strcmpi(mode,'tour')
    % tour 模式 nCtrl 由 mu_config 依据任务数自动设为 2*(maxT+1)（见 mu_config 内部），
    % 此处忽略用户传入的 nCtrl，保证每机控制点足够覆盖分段构造（修复 M1/D2）。
    scene = mu_config('tour','nUAV',opt.nUAV,'difficulty',opt.difficulty,'seed',opt.seed);
else
    scene = mu_config('p2p','nUAV',opt.nUAV,'nCtrl',opt.nCtrl,'difficulty',opt.difficulty,'seed',opt.seed);
end

% ---- 组装维度与边界 ----
nU = scene.nUAV; nC = scene.nCtrl;
xyzLo = scene.bounds([1 3 5]);   % [xmin ymin zmin]
xyzHi = scene.bounds([2 4 6]);   % [xmax ymax zmax]
if strcmpi(mode,'tour')
    % R20：差异化控制点维度。按 scene.ctrlPer(k) 逐机切片拼接，
    % 空闲机(ctrlPer=2)不再占用最忙机的 2*(maxT+1) 死维度。
    ctrlLB = []; ctrlUB = [];
    for k=1:nU
        ck = scene.ctrlPer(k);
        ctrlLB = [ctrlLB, repmat(xyzLo, 1, ck)];
        ctrlUB = [ctrlUB, repmat(xyzHi, 1, ck)];
    end
    maxT = max(cellfun(@numel, scene.taskAssign));
    keyDim = nU*maxT;
    dim = scene.dimCtrl + keyDim;
    % 随机键边界 [0,1]
    lb = [ctrlLB, zeros(1,keyDim)];
    ub = [ctrlUB, ones(1,keyDim)];
else
    per = nC*3;
    ctrlDim = nU*per;
    dim = ctrlDim;
    lb = repmat(xyzLo, 1, nU*nC);
    ub = repmat(xyzHi, 1, nU*nC);
end

% ---- 选择代价函数句柄 ----
if strcmpi(mode,'tour')
    fhd = @mu_cost_tour;
else
    fhd = @mu_cost_p2p;
end

% ---- CA 选项 ----
caOpts = struct('maxFE', opt.maxFE, 'seed', opt.seed);

% ---- 调用 CA 算法（接口样式，scene 经 varargin 透传）----
if nargout >= 3
    [bestCost, bestX, curve] = opt.caFun(fhd, dim, opt.pop, opt.iter, lb, ub, caOpts, scene);
else
    [bestCost, bestX] = opt.caFun(fhd, dim, opt.pop, opt.iter, lb, ub, caOpts, scene);
    curve = [];
end

% ---- 拆解最优解为轨迹（供可视化）----
trajs = mu_decode(bestX, scene, mode);
end

function trajs = mu_decode(x, scene, mode)
x = x(:).';
nU = scene.nUAV; nC = scene.nCtrl;
maxT = max(cellfun(@numel, scene.taskAssign));
trajs = cell(nU,1);
if strcmpi(mode,'tour')
    % R20：差异化控制点维度，按 scene.ctrlPer(k) 逐机切片
    off = 0;
    for k=1:nU
        ck = scene.ctrlPer(k);
        xi = x(off + 1 : off + ck*3);
        ctrl = reshape(xi, 3, ck).';
        off = off + ck*3;
        tIdx = scene.taskAssign{k};
        ctrlTot = scene.dimCtrl;   % 控制点段总长（键段在之后）
        if isempty(tIdx)
            % 空闲机：停在 depot 的单点轨迹（与代价函数退化分支一致，R20/R21）
            trajs{k} = repmat(scene.starts(k,:), max(2, scene.smooth), 1);
        else
            keyMat = reshape(x(ctrlTot+1:end), maxT, nU).';
            keys = keyMat(k, 1:numel(tIdx));
            [~, ord] = sort(keys);
            waypts = scene.tasks(tIdx(ord), :);
            trajs{k} = mu_build_tour_traj(ctrl, scene.starts(k,:), scene.goals(k,:), ...
                                          waypts, scene.smooth, ck);
        end
    end
else
    per = nC*3;
    for k=1:nU
        xi = x((k-1)*per + 1 : k*per);
        ctrl = reshape(xi, 3, nC).';
        trajs{k} = mu_bspline(ctrl, scene.starts(k,:), scene.goals(k,:), scene.smooth);
    end
end
end
