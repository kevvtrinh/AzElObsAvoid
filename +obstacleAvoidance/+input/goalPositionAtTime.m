function position_units = goalPositionAtTime(goalState, time_s)
%% Section 0: Header & Readme
% SYNTAX
%   position_units = obstacleAvoidance.input.goalPositionAtTime(goalState, time_s)
%
% PURPOSE
%   - Evaluate one fixed or sampled moving goal at requested times.
%
% INPUTS
%   - goalState (scalar normalized goal-state struct)
%       position_units is the fixed goal. Optional targetTime_s,
%       targetPosition_units, and InterpolationMethod define a moving goal.
%   - time_s (numeric scalar or vector)
%       Absolute query time accepted by the selected interpolation method.
%
% OUTPUTS
%   - position_units (numeric array)
%       Fixed 1-by-2 position or interpolated N-by-2 moving-goal positions.
%
% UNITS
%   - Position is coordinate units. Time is seconds.
%

%% Section 1: Evaluate The Selected Goal Representation

% Interpolate moving goals; return the stored position for fixed goals.

if isfield(goalState, "targetTime_s") && ~isempty(goalState.targetTime_s)
    position_units = interp1(goalState.targetTime_s, goalState.targetPosition_units, time_s, goalState.InterpolationMethod);
else
    position_units = goalState.position_units;
end
end
