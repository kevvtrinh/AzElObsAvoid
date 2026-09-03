function result = planCorridorQuintic( ...
        obstacles, initialState, goalState, limits, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = obstacleAvoidance.planner.planCorridorQuintic()
%   result = obstacleAvoidance.planner.planCorridorQuintic( ...
%       obstacles, initialState, goalState, limits)
%   result = obstacleAvoidance.planner.planCorridorQuintic( ...
%       obstacles, initialState, goalState, limits, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Provide a deprecated compatibility alias for planTrajectory.
%   - Keep one production planner implementation at the public entry point.
%**************************************************************************
% INPUTS
%   - obstacles (supported obstacle input or [])
%       Static or time-varying protected geometry.
%   - initialState, goalState (scalar structs)
%       Endpoint motion states and goal-time policy.
%   - limits (scalar struct)
%       Physical and workspace limits with units in field names.
%   - optionOverrides (scalar struct, optional; default struct())
%       Partial public options; omitted and empty fields use defaults.
%**************************************************************************
% OUTPUTS
%   - result (scalar planner result or options struct)
%       Unmodified output from obstacleAvoidance.planTrajectory.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds. Derivatives use deg/s,
%     deg/s^2, and deg/s^3.
%**************************************************************************

%% Section 1: Forward To The Production Planner

% Deprecated: all planning decisions live in planTrajectory so this alias
% cannot drift into a competing implementation.

if nargin == 0
    result = obstacleAvoidance.planTrajectory();
elseif nargin < 5
    result = obstacleAvoidance.planTrajectory( ...
        obstacles, initialState, goalState, limits);
else
    result = obstacleAvoidance.planTrajectory( ...
        obstacles, initialState, goalState, limits, optionOverrides);
end
end
