function [result, diagnosis] = exampleAlternatingSlalom(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleAlternatingSlalom()
%   result = exampleAlternatingSlalom(exampleOverrides)
%
% PURPOSE
%   - Demonstrate one input-driven route through alternating static barriers.
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

% Start with the same display controls and planner defaults as other examples.
% Caller overrides can hide plots or change public planner options.

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
[options, displayOptions] = resolveExampleOptions(exampleOverrides, struct("GoalTimeMode", "earliestArrival"), [2 2]);

%% Section 2: Create Obstacles

% Four narrow rectangles cross the direct route. Their centers alternate above
% and below the route. This forces a repeated left-right slalom without stored
% waypoints or a preferred turn direction.

obstacleTime_s      = [0; 30];
centerX_units   = [-4; 0; 4];
centerY_units = [2.5; -2.5; 2.5];
obstacles           = obstacleAvoidance.obstacles.combineObstacles();

% Center each vertical barrier at its given offset. Keep the loop order stable
% so obstacle indices and diagnostics are reproducible.
for obstacleIndex = 1:numel(centerX_units)
    center_units    = [centerX_units(obstacleIndex), centerY_units(obstacleIndex)];
    rectangle_units = center_units + [ -0.7 -2.5; 0.7 -2.5; 0.7 2.5; -0.7 2.5];
    obstacle      = obstacleAvoidance.obstacles.createObstacle("barrier " + obstacleIndex, obstacleTime_s, rectangle_units(:, 1), rectangle_units(:, 2), 0.1);
    obstacles     = obstacleAvoidance.obstacles.combineObstacles(obstacles, obstacle);
end

%% Section 3: Create Planner Inputs

% The start and goal are on opposite sides of the barrier row. The time window
% and motion limits require one smooth, physically possible route.

initialState = struct();
initialState.time_s       = 0;
initialState.position_units = [-8 0];
goalState = struct();
goalState.time_s       = 22;
goalState.position_units = [8 0];
limits = struct("maxVelocity_units_s", [2 2], ...
    "maxAcceleration_units_s2", [1 1], "maxJerk_units_s3", displayOptions.MaxJerk_units_s3, "yInterval_units", [-5 5]);

%% Section 4: Run Planner

% Call the maintained planner once. The example does not add route hints.

[result, diagnosis] = obstacleAvoidance.planTrajectory(obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result

% Validate the returned motion independently. A failure can point to collision,
% endpoint, workspace, or motion-limit errors.

exampleValidation = obstacleAvoidance.validateTrajectory(result);
if ~exampleValidation.Passed
    warning("exampleAlternatingSlalom:ValidationFailed", "%s", exampleValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion

% Plot only from the returned result. A failed plan shows search diagnostics.

if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory(result, displayOptions.PlotOptions, diagnosis);
end

end
