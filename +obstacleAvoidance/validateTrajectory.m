function validation = validateTrajectory(trajectory, obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   validation = obstacleAvoidance.validateTrajectory()
%   validation = obstacleAvoidance.validateTrajectory(result)
%   validation = obstacleAvoidance.validateTrajectory( ...
%       trajectory, obstacles, initialState, goalState, limits, options)
%
% PURPOSE
%   - Validate one complete polynomial motion independently of planner status.
%   - Fail closed when a continuous collision interval cannot be resolved.
%
% INPUTS
%   - trajectory (scalar candidate or planner-result struct)
%       Must contain sampled histories and Polynomial segment coefficients.
%   - obstacles (canonical protected obstacle array, optional with result)
%   - initialState, goalState (normalized state structs, optional with result)
%   - limits (normalized physical limits struct, optional with result)
%   - options (resolved planner options, optional with result)
%
% OUTPUTS
%   - validation (scalar struct)
%       Stable checks, clearance, message, collision counts, and timings.
%
% UNITS
%   - Position and clearance are degrees. Derivatives use deg/s, deg/s^2,
%     and deg/s^3. Time is seconds.
%
if nargin == 0
    validation = obstacleAvoidance.validation.validatePreparedTrajectory();
    return;
end
validationTimer = tic;

%% Section 1: Resolve Inputs And Validate Histories

if nargin == 1
    requiredFields = {'Inputs', 'Options', 'Polynomial'};
    if ~isstruct(trajectory) || ~isscalar(trajectory) || ~all(isfield(trajectory, requiredFields))
        error("validateTrajectory:InvalidResult", "A one-input call requires a planner result with Inputs, " + "Options, and Polynomial fields.");
    end
    obstacles    = trajectory.Inputs.obstacles;
    initialState = trajectory.Inputs.initialState;
    goalState    = trajectory.Inputs.goalState;
    limits       = trajectory.Inputs.limits;
    options      = trajectory.Options;
elseif nargin ~= 6
    error("validateTrajectory:InvalidCall", "Use one planner result or all six explicit validation inputs.");
end
if isempty(obstacles) || ~isfield(obstacles, "InternalPreparation")
    obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacles);
end
obstacles  = obstacleAvoidance.obstacles.prepareObstacles(obstacles);
validation = obstacleAvoidance.validation.validatePreparedTrajectory(trajectory, obstacles, initialState, goalState, limits, options);
validation.ElapsedTime_s = toc(validationTimer);
end
