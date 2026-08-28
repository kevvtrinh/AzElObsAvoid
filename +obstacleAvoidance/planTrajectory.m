function result = planTrajectory( ...
        obstacles, initialState, goalState, limits, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = obstacleAvoidance.planTrajectory()
%   result = obstacleAvoidance.planTrajectory( ...
%       obstacles, initialState, goalState, limits)
%   result = obstacleAvoidance.planTrajectory( ...
%       obstacles, initialState, goalState, limits, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Plan collision-free Az/El motion through one public entry point.
%   - Use exact Ruckig switching for eligible obstacle-free motion and HS3
%     after obstacle topology constrains the path.
%**************************************************************************
% INPUTS
%   - obstacles (canonical protected obstacle array, nested cells, or [])
%       Use obstacleAvoidance.obstacles.createObstacle to add each safety
%       margin one time.
%   - initialState (scalar struct)
%       Initial time, position, and supported derivatives.
%   - goalState (scalar struct)
%       Fixed or moving-goal state accepted by the obstacle planner.
%   - limits (scalar struct)
%       Physical and workspace limits with units in field names.
%   - optionOverrides (scalar struct, optional; default struct())
%       Partial planner options. Empty fields use their documented defaults.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       The result contains success or failure data, motion, and diagnostics.
%   - options (scalar struct, zero-input call)
%       Fully resolved planner defaults.
%**************************************************************************
% UNITS
%   - Position is in degrees. Time is in seconds.
%   - Derivatives use deg/s, deg/s^2, and deg/s^3.
%   - Histories are N-by-2 [azimuth elevation] arrays.
%**************************************************************************

%% Section 1: Resolve Defaults Requests

% A call with no inputs requests the planner defaults. Use the internal planner
% for this request. This keeps the reported defaults equal to the values that
% the planner uses for a normal request.
if nargin == 0
    result = obstacleAvoidance.planner.plan();
    return;
end

% One input does not define a planning problem. Report this case here. The
% caller then gets a direct input error before input normalization starts.
if nargin == 1
    error("planTrajectory:MissingInputs", ...
        "Planning requires obstacles, initialState, goalState, and limits.");
end

%% Section 2: Resolve The Planner Request

% Keep the four physical inputs in one fixed order. They describe the
% environment, initial motion, required final motion, and physical limits.
if nargin < 4
    error("planTrajectory:MissingInputs", ...
        "obstacles, initialState, goalState, and limits are required.");
end
% An omitted or empty option structure selects all default values. The internal
% planner merges partial options and validates each value.
if nargin < 5 || isempty(optionOverrides)
    optionOverrides = struct();
end
if ~isstruct(optionOverrides) || ~isscalar(optionOverrides)
    error("planTrajectory:InvalidOptions", ...
        "optionOverrides must be a scalar struct.");
end

%% Section 3: Run The Obstacle Planner

% This public function contains no planning decisions. It provides one public
% entry point. The internal planner performs route search, polynomial motion
% optimization, collision checks, and result assembly.
% If a plan fails, inspect result.TerminationReason and SearchDiagnostics.
% Follow the reported stage into search, optimization, or final validation.
result = obstacleAvoidance.planner.plan( ...
    obstacles, initialState, goalState, limits, optionOverrides);
end
