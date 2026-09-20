function result = planner(obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   options = planner()
%   result = planner(obstacles, initialState, goalState, limits)
%   result = planner(obstacles, initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Prepare protected polygon histories and an exact visibility guide,
%     then construct independently certified C3 quintic BMTP motion.
%   - Fixed-arrival moving-obstacle requests use bounded initial- and
%     arrival-snapshot shortcuts, then one time-expanded next method. Failed
%     shortcuts never prove infeasibility, and every accepted motion passes
%     independent validation.
%   - Earliest-arrival requests use one capability-driven pipeline: an exact
%     static BMTP solve, or a departure best plan so far followed by one timed
%     timed search attempt, or arrival-time trials when required.
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
%       ArrivalTimeTolerance_s bounds every comparison in seconds and
%       ConstraintTolerance bounds coordinates, derivatives, and algebraic
%       residuals; the other options control the arrival mode, sampling,
%       wrapping, endpoint matching, and search. SpatialProbeIterationLimit
%       is the explicit BMTP budget for each fixed-arrival snapshot shortcut.
%       BestSoFarRefinementTrialLimit is how many arrival-time trials may
%       try to beat a valid earliest-arrival plan once one is found; zero
%       keeps that plan without trying. A wrapped axis (WrapX, WrapY) is
%       planned in plain unwrapped coordinates: obstacles are copied one
%       turn up and down, a moving target's path is unwrapped so it never
%       jumps at the seam, and every goal copy the vehicle can reach in time
%       is planned and accepted against the wrapped request.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Stable success-or-failure record containing resolved inputs, prepared
%       geometry, visibility data, BMTP diagnostics, and Validation. Expected
%       no-path or infeasible outcomes return Success = false; invalid inputs
%       throw an error.
%   - options (scalar struct, zero-input call)
%       Fully resolved planner defaults.
%**************************************************************************
% UNITS
%   - Positions are 1-by-2 [x y] rows in coordinate units. Time is seconds;
%     velocity, acceleration, and jerk use units/s, units/s^2, and units/s^3.
%**************************************************************************

%% Section 1: Resolve Inputs And Dispatch The Request

% Recursive user path setup can put archived benchmark packages ahead of this
% checkout's engine. Keep planning and validation bound to the same checkout.
plannerFolder  = fileparts(mfilename('fullpath'));
engineFolder   = fullfile(plannerFolder, 'trajectory');
productionPath = [plannerFolder pathsep engineFolder];
if ~startsWith(path, [productionPath pathsep])
    addpath(productionPath, '-begin');
end

if nargin == 0
    result = obstacleAvoidance.planning.planRequest();
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
result = obstacleAvoidance.planning.planRequest( ...
    obstacles, initialState, goalState, limits, options, []);
end
