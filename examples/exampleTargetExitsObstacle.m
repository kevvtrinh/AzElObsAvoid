function [result, diagnosis] = exampleTargetExitsObstacle(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleTargetExitsObstacle()
%   result = exampleTargetExitsObstacle(exampleOverrides)
%
% PURPOSE
%   - Intercept a sampled target that begins inside an obstacle and later
%     moves into free space.
%   - Route around a separate circular obstacle between the initial state
%     and the target's containing obstacle.
%
% INPUTS
%   - exampleOverrides (scalar struct, optional; default struct())
%       Planner overrides plus the shared FigureVisible, PlotOutputs,
%       ShowAnimation, ShowKinematicPlot, and MaxJerk_units_s3 controls.
%
% OUTPUTS
%   - result (scalar planner-result struct)
%       Validated specified-time intercept, target-occupancy history,
%       scenario inputs, and optional plot handles.
%
% UNITS
%   - Position is coordinate units; time is seconds; derivatives use units/s,
%     units/s^2, and units/s^3.
%

%% Section 1: Resolve Example Controls

% Use fixed arrival. The target must leave its containing obstacle before a
% valid intercept can occur.

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end

plannerOverrides = exampleOverrides;

[options, jerkConfiguration] = resolveExampleOptions(plannerOverrides, struct("GoalTimeMode", "fixedArrival", "SampleTime_s", 0.05, "WrapX", false, "FigureVisible", "on", "Title", "Target exits a containing obstacle"), [2.5 2.5]);

options.WrapX = false;

%% Section 2: Create Obstacles

% One circle contains the target at the start. A second circle blocks the gimbal
% route. This separates target visibility from route obstacle avoidance.

missionEndTime_s  = 24;
obstacleTime_s    = [0; missionEndTime_s];
circleVertexCount = 72;
circleAngle_rad   = (0:circleVertexCount - 1).' * (2 * pi / circleVertexCount);
unitCircle        = [cos(circleAngle_rad), sin(circleAngle_rad)];
safetyMargin_units  = 0.15;

transitCircleCenter_units   = [0 0];
transitCircleRadius_units   = 2.0;
transitCirclePosition_units = transitCircleCenter_units + transitCircleRadius_units * unitCircle;
transitCircle             = obstacleAvoidance.obstacles.createObstacle("Transit circle", obstacleTime_s, {transitCirclePosition_units(:, 1); transitCirclePosition_units(:, 1)}, {transitCirclePosition_units(:, 2); transitCirclePosition_units(:, 2)}, safetyMargin_units);

containingCircleCenter_units   = [8 0];
containingCircleRadius_units   = 2.0;
containingCirclePosition_units = containingCircleCenter_units + containingCircleRadius_units * unitCircle;
containingCircle             = obstacleAvoidance.obstacles.createObstacle("Target containment circle", obstacleTime_s, {containingCirclePosition_units(:, 1); containingCirclePosition_units(:, 1)}, {containingCirclePosition_units(:, 2); containingCirclePosition_units(:, 2)}, safetyMargin_units);

obstacles = obstacleAvoidance.obstacles.combineObstacles(transitCircle, containingCircle);

%% Section 3: Create Planner Inputs

% The target waits at the circle center and then moves outward. Positive x
% motion gives a deterministic exit from occupied space.

initialState = struct();
initialState.time_s              = 0;
initialState.position_units        = [-8 0];
initialState.velocity_units_s      = [0 0];
initialState.acceleration_units_s2 = [0 0];

targetTime_s          = (0:4:missionEndTime_s).';
targetWaitSampleCount = 3;
postWaitSampleCount   = numel(targetTime_s) - targetWaitSampleCount;

% The target first stays at the circle center. It then follows the given outward
% path. Positive x motion makes the exit deterministic.
postWaitXStep_units   = linspace(0.90, 1.15, postWaitSampleCount).';
postWaitYStep_units = linspace(0.22, 0.38, postWaitSampleCount).';
postWaitStep_units          = [postWaitXStep_units, postWaitYStep_units];
postWaitPosition_units      = containingCircleCenter_units + cumsum(postWaitStep_units, 1);
targetPosition_units        = [ repmat(containingCircleCenter_units, targetWaitSampleCount, 1); postWaitPosition_units];
targetMotion              = struct("time_s", targetTime_s, "position_units", targetPosition_units, "InterpolationMethod", "linear");

limits = struct("maxVelocity_units_s", [2 2], ...
    "maxAcceleration_units_s2", [0.8 0.8], "maxJerk_units_s3", jerkConfiguration.MaxJerk_units_s3);
goalState = struct("time_s", missionEndTime_s, "targetMotion", targetMotion);
options.GoalTimeMode = "fixedArrival";

%% Section 4: Run Planner

% Run the specified-time moving-target planner.

[result, diagnosis] = planner(obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result

% Confirm that the target starts blocked and ends clear. Confirm that the gimbal
% avoids both circles and reaches the target at the set time.

exampleValidation    = validateExampleResult(result, "target exits a containing obstacle", struct("RequireDirectBlocked", true), diagnosis);
obstacleQueryOptions = struct();

targetOccupied        = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(result.Inputs.obstacles, targetPosition_units(:, 1), targetPosition_units(:, 2), targetTime_s, obstacleQueryOptions);
firstClearSampleIndex = find(~targetOccupied, 1, "first");
targetStartsInside    = targetOccupied(1);
targetEventuallyExits = ~isempty(firstClearSampleIndex) && all(~targetOccupied(firstClearSampleIndex:end));
targetWaitedInside    = all(targetOccupied(1:targetWaitSampleCount)) && all(targetPosition_units(1:targetWaitSampleCount, :) == containingCircleCenter_units, "all");

targetIsClearAtIntercept = false;
if result.Success && all(isfinite(result.Intercept.TargetPosition_units))
    targetIsClearAtIntercept = ~obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(result.Inputs.obstacles, result.Intercept.TargetPosition_units(1), result.Intercept.TargetPosition_units(2), result.Intercept.Time_s, obstacleQueryOptions);
end

transitCircleIsBetween       = initialState.position_units(1) < transitCircleCenter_units(1) && transitCircleCenter_units(1) < containingCircleCenter_units(1);
targetTravel_units             = sum(vecnorm(diff(targetPosition_units, 1, 1), 2, 2));
successfulInterceptValidated = result.Success && targetIsClearAtIntercept;
scenarioValidation           = struct("Passed", targetStartsInside && targetWaitedInside && ...
        targetEventuallyExits && transitCircleIsBetween && ...
        targetTravel_units > 0 && successfulInterceptValidated, ...
    "TargetStartsInside", targetStartsInside, ...
    "TargetWaitedInside", targetWaitedInside, ...
    "TargetEventuallyExits", targetEventuallyExits, ...
    "TargetIsClearAtIntercept", targetIsClearAtIntercept, ...
    "TargetFrameFailureReported", false, ...
    "TransitCircleIsBetween", transitCircleIsBetween, ...
    "TargetOccupiedAtInputSamples", targetOccupied, ...
    "FirstClearTargetSampleIndex", firstClearSampleIndex, "TargetTravel_units", targetTravel_units);

if ~scenarioValidation.Passed
    exampleValidation.Passed  = false;
    exampleValidation.Message = "Target containment or obstacle-placement validation failed.";
end
if ~exampleValidation.Passed
    warning("exampleTargetExitsObstacle:ValidationFailed", "%s", exampleValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion

% Show the target exit and the gimbal detour on one time axis.

if jerkConfiguration.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory(result, jerkConfiguration.PlotOptions, diagnosis);
end

end
