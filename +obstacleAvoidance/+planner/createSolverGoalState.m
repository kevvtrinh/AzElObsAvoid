function solverGoalState = createSolverGoalState(goalState, options)
%% Section 0: Header & Readme
% SYNTAX
%   solverGoalState = ...
%       obstacleAvoidance.planner.createSolverGoalState( ...
%       goalState, options)
%
% PURPOSE
%   - Remove moving-target metadata only after a fixed trial freezes its goal.
%
% INPUTS
%   - goalState (scalar normalized goal-state struct)
%       Requested endpoint and optional target history.
%   - options (resolved scalar struct)
%       GoalTimeMode determines whether the endpoint is physically frozen.
%
% OUTPUTS
%   - solverGoalState (scalar struct)
%       Goal understood by dimension-neutral motion kernels.
%
% UNITS
%   - Position is degrees and time is seconds.
%

%% Section 1: Remove Inapplicable Target History

% Remove target history only when fixed-arrival resolution has selected
% the same endpoint as goalState.position_deg.

solverGoalState = goalState;
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
metadataFields = intersect(fieldnames(solverGoalState), ...
    {'targetTime_s', 'targetPosition_deg', 'InterpolationMethod'});
solverGoalState = rmfield(solverGoalState, metadataFields);
end
