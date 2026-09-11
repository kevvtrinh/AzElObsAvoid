function [result, diagnosis] = exampleStaticUShapedObstacle(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleStaticUShapedObstacle()
%   result = exampleStaticUShapedObstacle(exampleOverrides)
%
% PURPOSE
%   - Plan from the cavity of one protected U-shaped obstacle to an exterior
%     goal without waypoints or a directed route.
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

% Use fixed static geometry and common planner controls.

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
[options, displayOptions] = resolveExampleOptions(exampleOverrides, struct("GoalTimeMode", "earliestArrival"), [2.5 2.5]);

%% Section 2: Create Obstacles

% The start is inside the open cavity of a U shape. The planner must leave
% through the opening before it can travel toward the exterior goal.

missionEndTime_s     = 120;
obstacleTime_s       = [0; missionEndTime_s];
obstaclePosition_units = [ -8 7; -5 7; -5 -4; 5 -4; 5 7; 8 7; 8 -7; -8 -7];
safetyMargin_units     = 0.20;
obstacles            = obstacleAvoidance.obstacles.createObstacle("Static U-shaped obstacle", obstacleTime_s, obstaclePosition_units(:, 1), obstaclePosition_units(:, 2), safetyMargin_units);

%% Section 3: Create Planner Inputs

% No waypoint identifies the opening. The search must find it from the supplied
% protected boundary and endpoint positions.

initialState = struct();
initialState.time_s       = 0;
initialState.position_units = [0 0];
goalState = struct("time_s", missionEndTime_s, "position_units", [0 -10]);
limits    = struct("maxVelocity_units_s", [2 2], "maxAcceleration_units_s2", [0.75 0.75], "maxJerk_units_s3", displayOptions.MaxJerk_units_s3);

%% Section 4: Run Planner

% Run the public planner once with the complete scenario input.

[result, diagnosis] = planner(obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result

% Check the full path through the cavity opening. A direct segment through a U
% wall must fail collision validation.

exampleValidation = validateExampleResult(result, "single U", struct("RequireDirectBlocked", true), diagnosis);
if ~exampleValidation.Passed
    warning("exampleStaticUShapedObstacle:ValidationFailed", "%s", exampleValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion

% The workspace plot shows how the route leaves the concave cavity.

if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory(result, displayOptions.PlotOptions, diagnosis);
end

end
