function value = stageTiming(value, totalOrTimer, timing)
%% Section 0: Header & Readme
% SYNTAX
%   timing = obstacleAvoidance.planner.stageTiming()
%   timing = obstacleAvoidance.planner.stageTiming(timing, totalElapsedTime_s)
%   result = obstacleAvoidance.planner.stageTiming(result, planningTimer, timing)
%**************************************************************************
% PURPOSE
%   - Define the shared planner-stage and accounting-diagnostic format.
%   - Reconcile exclusive stages to an independently measured total.
%   - Apply finalized timing consistently at a public planner boundary.
%**************************************************************************
% INPUTS
%   - value (scalar struct, optional), timing record or planner result.
%   - totalOrTimer (numeric scalar, optional), elapsed seconds or tic handle.
%   - timing (scalar struct, optional), accumulated exclusive stages.
%**************************************************************************
% OUTPUTS
%   - value (scalar struct), empty/final timing or finalized planner result.
%       TimingAccountingValid is false, without changing planner success, when
%       finite nonnegative exclusive stages exceed the measured total beyond
%       clock tolerance. TimingAccountingResidual_s is total minus exclusive.
%**************************************************************************
% UNITS
%   - All timing values are seconds.
%**************************************************************************

if nargin == 0
    % Start every exclusive stage at zero. Unattributed time later captures
    % validation, setup, or overhead that was not measured by a named timer.
    value = struct( ...
        "TopologyElapsedTime_s", 0, ...
        "CorridorConstructionElapsedTime_s", 0, ...
        "MotionSolvingElapsedTime_s", 0, ...
        "CollisionCheckingElapsedTime_s", 0, ...
        "FinalValidationElapsedTime_s", 0, ...
        "UnattributedElapsedTime_s", 0, ...
        "TotalElapsedTime_s", 0, ...
        "TimingAccountingValid", true, ...
        "TimingAccountingResidual_s", 0);
    return;
end
if nargin == 2
    % Two inputs reconcile an existing timing record to a measured total.
    timing = value;
    totalElapsedTime_s = totalOrTimer;
elseif nargin == 3
    % Three inputs finalize a public result using the original planning timer.
    result = value;
    totalElapsedTime_s = toc(totalOrTimer);
else
    error("stageTiming:InvalidCall", ...
        "Use zero inputs to create timing, two to reconcile it, or three to finalize a planner result.");
end

if ~(isnumeric(totalElapsedTime_s) && isreal(totalElapsedTime_s) && ...
        isscalar(totalElapsedTime_s) && isfinite(totalElapsedTime_s) && ...
        totalElapsedTime_s >= 0)
    error("stageTiming:InvalidTotalElapsedTime", ...
        "totalElapsedTime_s must be one finite nonnegative numeric scalar.");
end
template = obstacleAvoidance.planner.stageTiming();
requiredNames = string(fieldnames(template));
if ~isstruct(timing) || ~isscalar(timing) || ...
        ~isequal(string(fieldnames(timing)), requiredNames)
    error("stageTiming:InvalidFormat", ...
        "timing must use the shared Az/El stage-timing format.");
end
exclusiveNames = ["TopologyElapsedTime_s"; ...
    "CorridorConstructionElapsedTime_s"; ...
    "MotionSolvingElapsedTime_s"; ...
    "CollisionCheckingElapsedTime_s"; ...
    "FinalValidationElapsedTime_s"];
nonnegativeNames = [exclusiveNames; "UnattributedElapsedTime_s"; ...
    "TotalElapsedTime_s"];
exclusiveElapsedTime_s = 0;

for name = reshape(nonnegativeNames, 1, [])
    % Reject corrupt elapsed values even when reconciliation will replace a
    % derived field. Only the signed accounting residual may be negative.
    elapsedTime_s = timing.(name);
    if ~(isnumeric(elapsedTime_s) && isreal(elapsedTime_s) && ...
            isscalar(elapsedTime_s) && isfinite(elapsedTime_s) && ...
            elapsedTime_s >= 0)
        error("stageTiming:InvalidTimingValue", ...
            "%s must be one finite nonnegative numeric scalar.", name);
    end
    if any(name == exclusiveNames)
        exclusiveElapsedTime_s = exclusiveElapsedTime_s + elapsedTime_s;
    end
end
if ~(islogical(timing.TimingAccountingValid) && ...
        isscalar(timing.TimingAccountingValid))
    error("stageTiming:InvalidTimingValue", ...
        "TimingAccountingValid must be one logical scalar.");
end
residualInput_s = timing.TimingAccountingResidual_s;
if ~(isnumeric(residualInput_s) && isreal(residualInput_s) && ...
        isscalar(residualInput_s) && isfinite(residualInput_s))
    error("stageTiming:InvalidTimingValue", ...
        "TimingAccountingResidual_s must be one finite numeric scalar.");
end
% Permit only clock-resolution and floating-point accumulation noise.
accountingTolerance_s = max(1e-6, ...
    64 * eps(max(1, totalElapsedTime_s)));
accountingResidual_s = totalElapsedTime_s - exclusiveElapsedTime_s;
timing.UnattributedElapsedTime_s = max( ...
    0, accountingResidual_s);
timing.TotalElapsedTime_s = totalElapsedTime_s;
timing.TimingAccountingValid = ...
    accountingResidual_s >= -accountingTolerance_s;
timing.TimingAccountingResidual_s = accountingResidual_s;

if nargin == 2
    value = timing;
else
    result.SearchDiagnostics.StageTiming = timing;
    result.ElapsedPlanningTime_s = totalElapsedTime_s;
    value = result;
end
end
