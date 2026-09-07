function [result, diagnosis] = exampleObstacleAvoidance(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleObstacleAvoidance()
%   result = exampleObstacleAvoidance(exampleOverrides)
%
% PURPOSE
%   - Demonstrate deterministic side choice around one protected rectangle.
%
% INPUTS
%   - exampleOverrides (scalar struct, optional; default struct())
%       Uniform display controls and public planner option overrides.
%
% OUTPUTS
%   - result (scalar struct)
%       Unmodified public planner result.
%   - diagnosis (optional second output): search attempts and solver details.
%
% UNITS
%   - Position is coordinate units; time is seconds; derivatives use units/s, units/s^2,
%     and units/s^3.
%

%% Section 1: Resolve Example Controls

% Resolve common options before the scenario defines its physical inputs.

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
[options, displayOptions] = resolveExampleOptions(exampleOverrides, struct("GoalTimeMode", "earliestArrival"), [2 2]);

%% Section 2: Create Obstacles

% One protected rectangle blocks the direct line. Symmetric route choices test
% deterministic side selection from identical inputs.

obstacleTime_s        = [0; 20];
obstacleX_units   = [-1; 1; 1; -1];
obstacleY_units = [-2; -2; 2; 2];
safetyMargin_units      = 0.2;
obstacles             = obstacleAvoidance.obstacles.createObstacle("rectangle", obstacleTime_s, obstacleX_units, obstacleY_units, safetyMargin_units);

%% Section 3: Create Planner Inputs

% The start and goal lie on opposite sides of the rectangle. The safety margin
% belongs to obstacle construction and is not added again by the planner.

initialState = struct();
initialState.time_s       = 0;
initialState.position_units = [-5 0];
goalState = struct();
goalState.time_s       = 12;
goalState.position_units = [5 0];
limits = struct("maxVelocity_units_s", [2 2], "maxAcceleration_units_s2", [1 1], "maxJerk_units_s3", displayOptions.MaxJerk_units_s3);

%% Section 4: Run Planner

% Run the maintained planner without waypoints or a preferred detour side.

[result, diagnosis] = obstacleAvoidance.planTrajectory(obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result

% Verify endpoint agreement, limits, workspace bounds, and collision freedom.

exampleValidation = obstacleAvoidance.validateTrajectory(result);
if ~exampleValidation.Passed
    warning("exampleObstacleAvoidance:ValidationFailed", "%s", exampleValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion

% Show the original obstacle, protected obstacle, route, and timed motion.

if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory(result, displayOptions.PlotOptions, diagnosis);
end

end
