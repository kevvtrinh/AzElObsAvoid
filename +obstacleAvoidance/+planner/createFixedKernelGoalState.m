function kernelGoalState = createFixedKernelGoalState(goalState, options)
%% Section 0: Header & Readme
% SYNTAX
%   kernelGoalState = ...
%       obstacleAvoidance.planner.createFixedKernelGoalState( ...
%       goalState, options)
%**************************************************************************
% PURPOSE
%   - Remove moving-target metadata only after a fixed trial freezes its goal.
%**************************************************************************
% INPUTS
%   - goalState (scalar normalized goal-state struct)
%       Requested endpoint and optional target history.
%   - options (resolved scalar struct)
%       GoalTimeMode determines whether the endpoint is physically frozen.
%**************************************************************************
% OUTPUTS
%   - kernelGoalState (scalar struct)
%       Goal understood by dimension-neutral motion kernels.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds.
%**************************************************************************

%% Section 1: Remove Inapplicable Target History

% A motion engine should not interpret obstacle-planner target history. It can
% be removed only when fixed-arrival resolution selected the same physical
% endpoint represented by goalState.position_deg.

kernelGoalState = goalState;
hasTargetHistory = isfield(goalState, "targetTime_s") && ...
    ~isempty(goalState.targetTime_s);
if ~hasTargetHistory || string(options.GoalTimeMode) ~= "fixedArrival"
    return;
end
targetPosition_deg = obstacleAvoidance.input.goalPositionAtTime( ...
    goalState, goalState.time_s);
coordinateScale_deg = bmtpEngine.createCoordinateTolerances( ...
    targetPosition_deg, goalState.position_deg);
if max(abs(targetPosition_deg - goalState.position_deg)) > ...
        256 * eps(coordinateScale_deg)
    return;
end
metadataFields = intersect(fieldnames(kernelGoalState), ...
    {'targetTime_s', 'targetPosition_deg', 'InterpolationMethod'});
kernelGoalState = rmfield(kernelGoalState, metadataFields);
end
