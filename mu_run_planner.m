function [bestX, bestCost, curve, trajs, scene] = mu_run_planner(mode, varargin)
% mu_run_planner ????CA ??%   [bestX, bestCost, curve, trajs, scene] = mu_run_planner(mode, ...)
% ---- ???? CAv9x ????（父目录）----
appDir = fileparts(mfilename('fullpath'));
addpath(appDir); addpath(fileparts(appDir));   % CAV9x.m 在上一级目录
%     mode     : 'p2p' | 'tour'
%   ??Name/Value:
%     pop, iter, maxFE, nCtrl, nUAV, difficulty, seed, caFun ( @CAv9x)
%
% 
%   bestX   : +
%   bestCost: ??%   curve   : CA 
%   trajs   : cell{nUAV} ??(n x 3)
%   scene   : ??%
% ??CA varargin 
% CA ??feval(fhd, x, scene) ??= fhd(x, scene)??
% ---- ??----
p = inputParser;
addParameter(p,'pop',150);
addParameter(p,'iter',400);
addParameter(p,'maxFE',260000);
addParameter(p,'nCtrl',12);
addParameter(p,'nUAV',3);
addParameter(p,'difficulty','medium');
addParameter(p,'seed',2);
addParameter(p,'caFun',@CAv9x);
parse(p, varargin{:});
opt = p.Results;

% ---- ??----
if strcmpi(mode,'tour')
    % tour  nCtrl ??mu_config ??2*(maxT+1) mu_config 
    % ??nCtrl M1/D2??
    scene = mu_config('tour','nUAV',opt.nUAV,'difficulty',opt.difficulty,'seed',opt.seed);
else
    scene = mu_config('p2p','nUAV',opt.nUAV,'nCtrl',opt.nCtrl,'difficulty',opt.difficulty,'seed',opt.seed);
end

% ---- ??----
nU = scene.nUAV; nC = scene.nCtrl;
xyzLo = scene.bounds([1 3 5]);   % [xmin ymin zmin]
xyzHi = scene.bounds([2 4 6]);   % [xmax ymax zmax]
if strcmpi(mode,'tour')
    % R20 scene.ctrlPer(k) ??    % ??ctrlPer=2)??2*(maxT+1) ??
    ctrlLB = []; ctrlUB = [];
    for k=1:nU
        ck = scene.ctrlPer(k);
        ctrlLB = [ctrlLB, repmat(xyzLo, 1, ck)];
        ctrlUB = [ctrlUB, repmat(xyzHi, 1, ck)];
    end
    maxT = max(cellfun(@numel, scene.taskAssign));
    keyDim = nU*maxT;
    dim = scene.dimCtrl + keyDim;
    % ??[0,1]
    lb = [ctrlLB, zeros(1,keyDim)];
    ub = [ctrlUB, ones(1,keyDim)];
else
    per = nC*3;
    ctrlDim = nU*per;
    dim = ctrlDim;
    lb = repmat(xyzLo, 1, nU*nC);
    ub = repmat(xyzHi, 1, nU*nC);
end

% ----  ----
if strcmpi(mode,'tour')
    fhd = @mu_cost_tour;
else
    fhd = @mu_cost_p2p;
end

% ---- CA  ----
caOpts = struct('maxFE', opt.maxFE, 'seed', opt.seed);

% ---- 并行池：代价函数逐 UAV 用 parfor 加速（scene.useParallel 默认开）----
% 仅当开关开、装了并行工具箱、且当前未开池时启动默认规模池；否则自动退化为串行。
% parpool 启动用 try-catch 包裹：若 license/配置异常导致开池失败，降级串行而非崩溃。
if scene.useParallel && ~isempty(ver('parallel'))
    if isempty(gcp('nocreate'))
        try
            parpool;   % 使用默认 worker 数（通常 = 本地核心数）
        catch MEpool
            fprintf('[parallel] 并行池启动失败，降级串行 for：%s\n', MEpool.message);
        end
    end
    p = gcp('nocreate');
    if ~isempty(p)
        fprintf('[parallel] 已启用 parfor 逐UAV并行，worker 数 = %d\n', p.NumWorkers);
    else
        fprintf('[parallel] 未获得并行池，退化为串行 for\n');
    end
else
    fprintf('[parallel] 未启用（useParallel=false 或未装 Parallel Computing Toolbox），退化为串行 for\n');
end

% ----  CA scene ??varargin ??---
if nargout >= 3
    [bestCost, bestX, curve] = opt.caFun(fhd, dim, opt.pop, opt.iter, lb, ub, caOpts, scene);
else
    [bestCost, bestX] = opt.caFun(fhd, dim, opt.pop, opt.iter, lb, ub, caOpts, scene);
    curve = [];
end

% ---- ??---
trajs = mu_decode(bestX, scene, mode);
end

function trajs = mu_decode(x, scene, mode)
x = x(:).';
nU = scene.nUAV; nC = scene.nCtrl;
maxT = max(cellfun(@numel, scene.taskAssign));
trajs = cell(nU,1);
if strcmpi(mode,'tour')
    % R20??scene.ctrlPer(k) 
    off = 0;
    for k=1:nU
        ck = scene.ctrlPer(k);
        xi = x(off + 1 : off + ck*3);
        ctrl = reshape(xi, 3, ck).';
        off = off + ck*3;
        tIdx = scene.taskAssign{k};
        ctrlTot = scene.dimCtrl;   % ??        if isempty(tIdx)
            %  depot R20/R21??
        if isempty(tIdx)
            trajs{k} = repmat(scene.starts(k,:), max(2, scene.smooth), 1);
        else
            keyMat = reshape(x(ctrlTot+1:end), maxT, nU).';
            keys = keyMat(k, 1:numel(tIdx));
            [~, ord] = sort(keys);
            waypts = scene.tasks(tIdx(ord), :);
            trajs{k} = mu_build_tour_traj(ctrl, scene.starts(k,:), scene.goals(k,:), ...
                                          waypts, scene.smooth);
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
