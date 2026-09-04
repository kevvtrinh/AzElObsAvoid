function result = exampleObstacleAvoidance(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleObstacleAvoidance()
%   result = exampleObstacleAvoidance(exampleOverrides)
%**************************************************************************
% PURPOSE
%   - Demonstrate deterministic side choice around one protected rectangle.
%**************************************************************************
% INPUTS
%   - exampleOverrides (scalar struct, optional; default struct())
%       Uniform display controls and public planner option overrides.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Unmodified public planner result.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s, deg/s^2,
%     and deg/s^3.
%**************************************************************************

%% Section 1: Resolve Example Controls

% Resolve common options before the scenario defines its physical inputs.

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
[options, displayOptions] = resolveExampleOptions( ...
    exampleOverrides, struct("GoalTimeMode", "earliestArrival"), [2 2]);

%% Section 2: Create Obstacles

% One protected rectangle blocks the direct line. Symmetric route choices test
% deterministic side selection from identical inputs.

obstacleTime_s = [0; 20];
obstacleAzimuth_deg = [-1; 1; 1; -1];
obstacleElevation_deg = [-2; -2; 2; 2];
safetyMargin_deg = 0.2;
obstacles = obstacleAvoidance.obstacles.createObstacle( ...
    "rectangle", obstacleTime_s, obstacleAzimuth_deg, obstacleElevation_deg, safetyMargin_deg);

%% Section 3: Create Planner Inputs

% The start and goal lie on opposite sides of the rectangle. The safety margin
% belongs to obstacle construction and is not added again by the planner.

initialState = struct("time_s", 0, "position_deg", [-5 0]);
goalState = struct("time_s", 12, "position_deg", [5 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], "maxAcceleration_deg_s2", [1 1], "maxJerk_deg_s3", displayOptions.MaxJerk_deg_s3);

%% Section 4: Run Planner

% Run the maintained planner without waypoints or a preferred detour side.

result = obstacleAvoidance.planTrajectory( obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result

% Verify endpoint agreement, limits, workspace bounds, and collision freedom.

exampleValidation = obstacleAvoidance.validateTrajectory(result);
if ~exampleValidation.Passed
    warning("exampleObstacleAvoidance:ValidationFailed", ...
        "%s", exampleValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion

% Show the original obstacle, protected obstacle, route, and timed motion.

if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory( ...
        result, displayOptions.PlotOptions);
end

end
