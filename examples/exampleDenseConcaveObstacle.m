function result = exampleDenseConcaveObstacle(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleDenseConcaveObstacle()
%   result = exampleDenseConcaveObstacle(exampleOverrides)
%**************************************************************************
% PURPOSE
%   - Demonstrate protected dense concave geometry without special waypoints.
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
    exampleOverrides, struct( "GoalTimeMode", "earliestArrival", "MaximumSeedCount", 2), [2 2]);

%% Section 2: Create Obstacles

vertexCount = 80;
angle_rad = (0:vertexCount - 1).' * (2 * pi / vertexCount);
radius_deg = 1.6 + 0.45 * cos(5 * angle_rad);
obstacleAzimuth_deg = radius_deg .* cos(angle_rad);
obstacleElevation_deg = radius_deg .* sin(angle_rad);
obstacleTime_s = [0; 20];
safetyMargin_deg = 0.1;
obstacles = obstacleAvoidance.obstacles.createObstacle( ...
    "dense concave polygon", obstacleTime_s, obstacleAzimuth_deg, obstacleElevation_deg, safetyMargin_deg);

%% Section 3: Create Planner Inputs

initialState = struct("time_s", 0, "position_deg", [-6 0]);
goalState = struct("time_s", 15, "position_deg", [6 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], "maxAcceleration_deg_s2", [1 1], "maxJerk_deg_s3", displayOptions.MaxJerk_deg_s3);

%% Section 4: Run Planner

result = obstacleAvoidance.planTrajectory( obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result

exampleValidation = obstacleAvoidance.validateTrajectory(result);
if ~exampleValidation.Passed
    warning("exampleDenseConcaveObstacle:ValidationFailed", ...
        "%s", exampleValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion

if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory( ...
        result, displayOptions.PlotOptions);
end

end
