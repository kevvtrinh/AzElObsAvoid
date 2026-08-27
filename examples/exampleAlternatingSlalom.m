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

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
[options, displayOptions] = resolveExampleOptions( ...
    exampleOverrides, struct( "GoalTimeMode", "earliestArrival", "MaximumSeedCount", 3), [2 2]);

%% Section 2: Create Obstacles

obstacleTime_s = [0; 30];
centerAzimuth_deg = [-4; 0; 4];
centerElevation_deg = [2.5; -2.5; 2.5];
obstacles = obstacleAvoidance.obstacles.combineObstacles();

% Center each vertical barrier at its configured offset to form the alternating slalom.
for obstacleIndex = 1:numel(centerAzimuth_deg)
    center_deg = [centerAzimuth_deg(obstacleIndex), centerElevation_deg(obstacleIndex)];
    rectangle_deg = center_deg + [ -0.7 -2.5; 0.7 -2.5; 0.7 2.5; -0.7 2.5];
    obstacle = obstacleAvoidance.obstacles.createObstacle( ...
        "barrier " + obstacleIndex, obstacleTime_s, rectangle_deg(:, 1), rectangle_deg(:, 2), 0.1);
    obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacles, obstacle);
end

%% Section 3: Create Planner Inputs

initialState = struct("time_s", 0, "position_deg", [-8 0]);
goalState = struct("time_s", 22, "position_deg", [8 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1], "maxJerk_deg_s3", displayOptions.MaxJerk_deg_s3, "elevationInterval_deg", [-5 5]);

%% Section 4: Run Planner

result = obstacleAvoidance.planTrajectory( obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result

exampleValidation = obstacleAvoidance.validateTrajectory(result);
if ~exampleValidation.Passed
    warning("exampleAlternatingSlalom:ValidationFailed", ...
        "%s", exampleValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion

if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory( ...
        result, displayOptions.PlotOptions);
end

end
