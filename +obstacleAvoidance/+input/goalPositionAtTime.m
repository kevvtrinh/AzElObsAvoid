function position_deg = goalPositionAtTime(goalState, time_s)
%% Section 0: Header & Readme
% SYNTAX
%   position_deg = obstacleAvoidance.input.goalPositionAtTime(goalState, time_s)
%**************************************************************************
% PURPOSE
%   - Evaluate one fixed or sampled moving goal at requested times.
%**************************************************************************
% INPUTS
%   - goalState (scalar normalized goal-state struct)
%       position_deg is the fixed goal. Optional targetTime_s,
%       targetPosition_deg, and InterpolationMethod define a moving goal.
%   - time_s (numeric scalar or vector)
%       Absolute query time accepted by the selected interpolation method.
%**************************************************************************
% OUTPUTS
%   - position_deg (numeric array)
%       Fixed 1-by-2 position or interpolated N-by-2 moving-goal positions.
%**************************************************************************
% UNITS
%   - Position is degrees. Time is seconds.
%**************************************************************************

%% Section 1: Evaluate The Selected Goal Representation

% A fixed goal has one position for all times. A moving goal stores sampled
% positions and times. Interpolate only for the moving case. If this function
% gives an unexpected point, inspect the goal time history and interpolation
% method before you inspect the planner.

% A moving goal is stored as a time history of [azimuth elevation] samples.
% interp1 evaluates both coordinate columns together. One query gives one row.
% A query vector gives one row for each requested time. Linear interpolation
% gives straight motion between samples. Pchip gives a smooth curve through all
% supplied samples.
%
% A fixed goal has no time history. In that case query time has no effect and
% the function returns the original 1-by-2 position. This function handles the
% goal type. Other planner functions do not need to handle the goal type.
if isfield(goalState, "targetTime_s") && ~isempty(goalState.targetTime_s)
    position_deg = interp1(goalState.targetTime_s, ...
        goalState.targetPosition_deg, time_s, ...
        goalState.InterpolationMethod);
else
    position_deg = goalState.position_deg;
end
end
