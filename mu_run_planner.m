function [bestX, bestCost, curve, trajs, scene] = mu_run_planner(mode, varargin)
% mu_run_planner ????CA ??%   [bestX, bestCost, curve, trajs, scene] = mu_run_planner(mode, ...)
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
