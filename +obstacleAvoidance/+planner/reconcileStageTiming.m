function timing = reconcileStageTiming(timing, totalElapsedTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   timing = reconcileStageTiming(timing, totalElapsedTime_s)
% PURPOSE
%   Check exclusive stage accounting against the measured total.
% INPUTS
%   Stage timings and total elapsed seconds.
% OUTPUTS
%   timing: stage timings with unattributed and total durations.
% UNITS
%   Seconds.

%% Section 1: Reconcile Measured Time
validateattributes(totalElapsedTime_s, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
template      = obstacleAvoidance.planner.createStageTiming();
requiredNames = string(fieldnames(template));
if ~isstruct(timing) || ~isscalar(timing) || ~isequal(string(fieldnames(timing)), requiredNames)
    error("stageTiming:InvalidFormat", "timing must use the shared Az/El stage-timing format.");
end
exclusiveNames         = requiredNames(1:end - 2);
exclusiveElapsedTime_s = 0;

for name = reshape(exclusiveNames, 1, [])
    % Sum only exclusive stages; the final two fields are derived totals.
    elapsedTime_s = timing.(name);
    validateattributes(elapsedTime_s, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
    exclusiveElapsedTime_s = exclusiveElapsedTime_s + elapsedTime_s;
end
% Permit only clock-resolution and floating-point accumulation noise.
accountingTolerance_s = max(1e-6, 64 * eps(max(1, totalElapsedTime_s)));
if exclusiveElapsedTime_s > totalElapsedTime_s + accountingTolerance_s
    error("stageTiming:OverAttributed", "Exclusive stage time exceeds the measured total time.");
end
timing.UnattributedElapsedTime_s = max(0, totalElapsedTime_s - exclusiveElapsedTime_s);
timing.TotalElapsedTime_s        = totalElapsedTime_s;

end
