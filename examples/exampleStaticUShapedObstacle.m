function result = exampleStaticUShapedObstacle(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleStaticUShapedObstacle()
%   result = exampleStaticUShapedObstacle(exampleOverrides)
%**************************************************************************
% PURPOSE
%   - Plan from the cavity of one protected U-shaped obstacle to an exterior
%     goal without waypoints or a directed route.
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

% Use fixed static geometry and common planner controls.

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
[options, displayOptions] = resolveExampleOptions( ...
    exampleOverrides, struct( "GoalTimeMode", "earliestArrival", "MaximumSeedCount", 3), [2.5 2.5]);

%% Section 2: Create Obstacles

% The start is inside the open cavity of a U shape. The planner must leave
% through the opening before it can travel toward the exterior goal.

missionEndTime_s = 120;
obstacleTime_s = [0; missionEndTime_s];
obstaclePosition_deg = [ -8 7; -5 7; -5 -4; 5 -4; 5 7; 8 7; 8 -7; -8 -7];
safetyMargin_deg = 0.20;
obstacles = obstacleAvoidance.obstacles.createObstacle( ...
    "Static U-shaped obstacle", obstacleTime_s, ...
    obstaclePosition_deg(:, 1), obstaclePosition_deg(:, 2), safetyMargin_deg);

%% Section 3: Create Planner Inputs

% No waypoint identifies the opening. The search must find it from the supplied
% protected boundary and endpoint positions.

initialState = struct("time_s", 0, "position_deg", [0 0]);
goalState = struct( "time_s", missionEndTime_s, "position_deg", [0 -10]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], "maxAcceleration_deg_s2", [0.75 0.75], "maxJerk_deg_s3", displayOptions.MaxJerk_deg_s3);

%% Section 4: Run Planner

% Run the public planner once with the complete scenario input.

result = obstacleAvoidance.planTrajectory( obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result

% Check the full path through the cavity opening. A direct segment through a U
% wall must fail collision validation.

exampleValidation = validateExampleResult( ...
    result, "single U", struct("RequireDirectBlocked", true));
if ~exampleValidation.Passed
    warning("exampleStaticUShapedObstacle:ValidationFailed", ...
        "%s", exampleValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion

% The workspace plot shows how the route leaves the concave cavity.

if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory( ...
        result, displayOptions.PlotOptions);
end

end
