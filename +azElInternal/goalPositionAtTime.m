function position_deg = goalPositionAtTime(goalState, time_s)
%% Section 0: Header & Readme
% SYNTAX
%   position_deg = azElInternal.goalPositionAtTime(goalState, time_s)
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
if isfield(goalState, "targetTime_s") && ~isempty(goalState.targetTime_s)
    position_deg = interp1( ...
        goalState.targetTime_s, goalState.targetPosition_deg, ...
        time_s, goalState.InterpolationMethod);
else
    position_deg = goalState.position_deg;
end
end
