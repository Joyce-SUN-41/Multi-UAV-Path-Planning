function tk = mu_arc_time(traj, T_horizon)
% mu_arc_time ?? [0, T_horizon] ??%   ??%   ??GUI ????tk ????%   p2pmu_eval_pathtourmu_cost_tourrun_muav
%   R2 ??n = size(traj,1);
n = size(traj,1);
if n < 2
    tk = zeros(n,1);
    return;
end
cum = [0; cumsum(sqrt(sum(diff(traj,1,1).^2,2)))];
tot = cum(end);
if tot < 1e-9
    tk = zeros(n,1);
else
    tk = (cum / tot) * T_horizon;
end
end
