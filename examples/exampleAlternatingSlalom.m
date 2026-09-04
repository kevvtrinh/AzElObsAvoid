function result = exampleAlternatingSlalom(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleAlternatingSlalom()
%   result = exampleAlternatingSlalom(exampleOverrides)
%**************************************************************************
% PURPOSE
%   - Demonstrate one input-driven route through alternating static barriers.
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

% Start with the same display controls and planner defaults as other examples.
% Caller overrides can hide plots or change public planner options.

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
[options, displayOptions] = resolveExampleOptions( ...
    exampleOverrides, struct("GoalTimeMode", "earliestArrival"), [2 2]);

%% Section 2: Create Obstacles

% Four narrow rectangles cross the direct route. Their centers alternate above
% and below the route. This forces a repeated left-right slalom without stored
% waypoints or a preferred turn direction.

obstacleTime_s = [0; 30];
centerAzimuth_deg = [-4; 0; 4];
centerElevation_deg = [2.5; -2.5; 2.5];
obstacles = obstacleAvoidance.obstacles.combineObstacles();

% Center each vertical barrier at its given offset. Keep the loop order stable
% so obstacle indices and diagnostics are reproducible.
for obstacleIndex = 1:numel(centerAzimuth_deg)
    center_deg = [centerAzimuth_deg(obstacleIndex), centerElevation_deg(obstacleIndex)];
    rectangle_deg = center_deg + [ -0.7 -2.5; 0.7 -2.5; 0.7 2.5; -0.7 2.5];
    obstacle = obstacleAvoidance.obstacles.createObstacle( ...
        "barrier " + obstacleIndex, obstacleTime_s, rectangle_deg(:, 1), rectangle_deg(:, 2), 0.1);
    obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacles, obstacle);
end

%% Section 3: Create Planner Inputs

% The start and goal are on opposite sides of the barrier row. The time window
% and motion limits require one smooth, physically possible route.

initialState = struct("time_s", 0, "position_deg", [-8 0]);
goalState = struct("time_s", 22, "position_deg", [8 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1], "maxJerk_deg_s3", displayOptions.MaxJerk_deg_s3, "elevationInterval_deg", [-5 5]);

%% Section 4: Run Planner

% Call the maintained planner once. The example does not add route hints.

result = obstacleAvoidance.planTrajectory( obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result

% Validate the returned motion independently. A failure can point to collision,
% endpoint, workspace, or motion-limit errors.

exampleValidation = obstacleAvoidance.validateTrajectory(result);
if ~exampleValidation.Passed
    warning("exampleAlternatingSlalom:ValidationFailed", ...
        "%s", exampleValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion

% Plot only from the returned result. A failed plan shows search diagnostics.

if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory( ...
        result, displayOptions.PlotOptions);
end

end
