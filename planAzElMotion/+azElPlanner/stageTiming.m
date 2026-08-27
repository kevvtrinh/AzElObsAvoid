function value = stageTiming(value, totalOrTimer, timing)
%% Section 0: Header & Readme
% SYNTAX
%   timing = azElPlanner.stageTiming()
%   timing = azElPlanner.stageTiming(timing, totalElapsedTime_s)
%   result = azElPlanner.stageTiming(result, planningTimer, timing)
%**************************************************************************
% PURPOSE
%   - Define the shared seven-field planner-stage schema.
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
%**************************************************************************
% UNITS
%   - All timing values are seconds.
%**************************************************************************

if nargin == 0
    value = struct( ...
        "TopologyElapsedTime_s", 0, ...
        "CorridorConstructionElapsedTime_s", 0, ...
        "MotionSolvingElapsedTime_s", 0, ...
        "CollisionCheckingElapsedTime_s", 0, ...
        "FinalValidationElapsedTime_s", 0, ...
        "UnattributedElapsedTime_s", 0, ...
        "TotalElapsedTime_s", 0);
    return;
end
if nargin == 2
    timing = value;
    totalElapsedTime_s = totalOrTimer;
elseif nargin == 3
    result = value;
    totalElapsedTime_s = toc(totalOrTimer);
else
    error("stageTiming:InvalidCall", ...
        "Use zero inputs to create timing, two to reconcile it, or three to finalize a planner result.");
end

validateattributes(totalElapsedTime_s, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'});
template = azElPlanner.stageTiming();
requiredNames = string(fieldnames(template));
if ~isstruct(timing) || ~isscalar(timing) || ...
        ~isequal(string(fieldnames(timing)), requiredNames)
    error("stageTiming:InvalidSchema", ...
        "timing must use the shared Az/El stage-timing schema.");
end
exclusiveNames = requiredNames(1:end - 2);
exclusiveElapsedTime_s = 0;

for name = reshape(exclusiveNames, 1, [])
    elapsedTime_s = timing.(name);
    validateattributes(elapsedTime_s, {'numeric'}, ...
        {'real', 'finite', 'scalar', 'nonnegative'});
    exclusiveElapsedTime_s = exclusiveElapsedTime_s + elapsedTime_s;
end
accountingTolerance_s = max(1e-6, ...
    64 * eps(max(1, totalElapsedTime_s)));
if exclusiveElapsedTime_s > totalElapsedTime_s + accountingTolerance_s
    error("stageTiming:OverAttributed", ...
        "Exclusive stage time exceeds the measured total time.");
end
timing.UnattributedElapsedTime_s = max( ...
    0, totalElapsedTime_s - exclusiveElapsedTime_s);
timing.TotalElapsedTime_s = totalElapsedTime_s;

if nargin == 2
    value = timing;
else
    result.SearchDiagnostics.StageTiming = timing;
    result.ElapsedPlanningTime_s = totalElapsedTime_s;
    value = result;
end
end
