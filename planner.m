function result = planner(obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   options = planner()
%   result = planner(obstacles, initialState, goalState, limits)
%   result = planner(obstacles, initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Prepare the obstacles, find a route, and use BMTP to calculate motion
%     along it. Choose the planning method from the requested arrival time,
%     endpoint motion, and whether obstacles change with time.
%   - Return success only after the complete motion passes the independent
%     validator's checks for motion limits, endpoints, and obstacle clearance.
%**************************************************************************
% INPUTS
%   - obstacles (struct array)
%       Static or time-varying polygons; [] requests obstacle-free planning.
%   - initialState (scalar struct)
%       Example: struct("time_s", 0, "position_units", [-4 0])
%   - goalState (scalar struct)
%       Example: struct("time_s", 12, "position_units", [4 0])
%   - limits (scalar struct)
%       Workspace intervals and scalar or per-axis motion limits.
%   - options (scalar struct, optional; default struct())
%       Controls arrival time, sampling, wrapping, endpoint matching, and
%       search limits. Call planner() to see all defaults.
%       ArrivalTimeTolerance_s allows small differences in time comparisons.
%       ConstraintTolerance sets the numerical tolerance for position and
%       motion constraints and the calculations used to check them.
%       SpatialProbeIterationLimit limits BMTP iterations for a route built
%       from obstacle positions at one time.
%       BestSoFarRefinementTrialLimit limits extra arrival-time trials after
%       the timed search fails but valid motion exists; zero keeps that
%       motion without extra trials.
%       WrapX and WrapY allow travel across the corresponding interval ends.
%       For example, on a 360-unit axis, travel from 350 to 10 can use 350 to
%       370. The planner copies obstacles throughout the possible travel
%       range and keeps a moving target's path continuous across the seam.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Includes the request, prepared obstacles, route, motion, solver
%       details, and Validation. Expected no-path or infeasible outcomes
%       return Success = false with Message and TerminationReason explaining
%       why. Invalid inputs throw an error.
%   - options (scalar struct, zero-input call)
%       Fully resolved planner defaults.
%**************************************************************************
% UNITS
%   - Positions are 1-by-2 [x y] rows in coordinate units. Time is seconds;
%     velocity, acceleration, and jerk use units/s, units/s^2, and units/s^3.
%**************************************************************************

%% Section 1: Prepare The Request

% MATLAB uses the first matching function on its search path. Put this
% checkout first so archived copies cannot replace its planning or validation.
plannerFolder  = fileparts(mfilename('fullpath'));
engineFolder   = fullfile(plannerFolder, 'trajectory');
productionPath = [plannerFolder pathsep engineFolder];
if ~startsWith(path, [productionPath pathsep])
    addpath(productionPath, '-begin');
end

if nargin == 0
    result = obstacleAvoidance.planning.prepareRequest();
    return
end
if nargin < 2
    initialState = [];
end
if nargin < 3
    goalState = [];
end
if nargin < 4
    limits = [];
end
if nargin < 5
    options = [];
end
request = obstacleAvoidance.planning.prepareRequest( ...
    obstacles, initialState, goalState, limits, options, []);

%% Section 2: Plan The Motion

% Wrapping gives several coordinates for the same goal, such as 10 and 370
% on a 360-unit axis. Try the relevant copies before selecting the motion.
if request.options.WrapX || request.options.WrapY
    result = obstacleAvoidance.planning.planWrappedMotion(request);
else
    result = obstacleAvoidance.planning.planMotion(request);
end
end
