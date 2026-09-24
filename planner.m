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
%   - options (scalar struct) [OPTIONAL, Default is struct()]
%       Planner settings. Omitted fields take the defaults from planner().
%       - GoalTimeMode (string) [Default is "fixedArrival"]
%           "fixedArrival" arrives at goalState.time_s; "earliestArrival"
%           arrives as early as the limits allow.
%       - SampleTime_s (positive scalar) [Default is 0.05]
%           Time step of the returned sampled motion.
%       - ConstraintTolerance (positive scalar) [Default is 1e-8]
%           Tolerance on position and motion constraints.
%       - CollisionClearanceTolerance_units (scalar) [Default is 1e-7]
%           Clearance the motion must keep from protected obstacles.
%       - ArrivalTimeTolerance_s (positive scalar) [Default is 1e-8]
%           Tolerance when comparing arrival times.
%       - WrapX, WrapY (string) [Default is "false"]
%           "false", "both", "forward", or "backward". Allows travel past
%           the interval ends. x copies shift by a whole turn: 350 to 10 on
%           a 360 axis can use 350 to 370. y copies mirror over a pole and
%           shift x by half a turn: on x [0 360], y [-90 90], (190, 89) is
%           also (10, 91).
%       - MatchTargetVelocity, MatchTargetAcceleration [Default is false]
%           Take the goal velocity or acceleration from the target's path
%           at the arrival time.
%       - TemporalResolution_s (positive scalar) [Default is 0.5]
%           Spacing of the arrival times tried in a search.
%       - SpatialProbeIterationLimit (integer, max 35) [Default is 2]
%           BMTP iterations for a route probed at one obstacle time.
%       - BestSoFarRefinementTrialLimit (integer) [Default is 0]
%           Extra arrival-time trials after valid motion is found.
%       - MaxArrivalTrials (integer) [Default is 100]
%           Arrival-time trials allowed per search.
%       - MaxArrivalCandidates (integer) [Default is 4096]
%           Cap on the arrival-time grid built per search.
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
if request.options.WrapX ~= "false" || request.options.WrapY ~= "false"
    result = obstacleAvoidance.planning.planWrappedMotion(request);
else
    result = obstacleAvoidance.planning.planMotion(request);
end
end
