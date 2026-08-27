function result = exampleTargetExitsObstacle(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleTargetExitsObstacle()
%   result = exampleTargetExitsObstacle(exampleOverrides)
%**************************************************************************
% PURPOSE
%   - Intercept a sampled target that begins inside an obstacle and later
%     moves into free space.
%   - Route around a separate circular obstacle between the initial state
%     and the target's containing obstacle.
%**************************************************************************
% INPUTS
%   - exampleOverrides (scalar struct, optional; default struct())
%       Planner overrides plus the shared FigureVisible, PlotOutputs,
%       ShowAnimation, ShowKinematicPlot, and MaxJerk_deg_s3 controls.
%**************************************************************************
% OUTPUTS
%   - result (scalar planner-result struct)
%       Validated specified-time intercept, target-occupancy history,
%       scenario inputs, and optional plot handles.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s,
%     deg/s^2, and deg/s^3.
%**************************************************************************

%% Section 1: Resolve Example Controls

% Use fixed arrival. The target must leave its containing obstacle before a
% valid intercept can occur.

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end

plannerOverrides = exampleOverrides;

[options, jerkConfiguration] = resolveExampleOptions( ...
    plannerOverrides, struct( ...
        "GoalTimeMode", "fixedArrival", ...
        "SampleTime_s", 0.05, ...
        "AllowAzimuthWrapping", false, ...
        "FigureVisible", "on", "Title", "Target exits a containing obstacle"), [2.5 2.5]);

options.AllowAzimuthWrapping = false;

%% Section 2: Create Obstacles

% One circle contains the target at the start. A second circle blocks the gimbal
% route. This separates target visibility from route obstacle avoidance.

missionEndTime_s = 24;
obstacleTime_s = [0; missionEndTime_s];
circleVertexCount = 72;
circleAngle_rad = (0:circleVertexCount - 1).' * (2 * pi / circleVertexCount);
unitCircle = [cos(circleAngle_rad), sin(circleAngle_rad)];
safetyMargin_deg = 0.15;

transitCircleCenter_deg = [0 0];
transitCircleRadius_deg = 2.0;
transitCirclePosition_deg = transitCircleCenter_deg + transitCircleRadius_deg * unitCircle;
transitCircle = obstacleAvoidance.obstacles.createObstacle( ...
    "Transit circle", obstacleTime_s, ...
    {transitCirclePosition_deg(:, 1); ...
        transitCirclePosition_deg(:, 1)}, ...
    {transitCirclePosition_deg(:, 2); transitCirclePosition_deg(:, 2)}, safetyMargin_deg);

containingCircleCenter_deg = [8 0];
containingCircleRadius_deg = 2.0;
containingCirclePosition_deg = containingCircleCenter_deg + containingCircleRadius_deg * unitCircle;
containingCircle = obstacleAvoidance.obstacles.createObstacle( ...
    "Target containment circle", obstacleTime_s, ...
    {containingCirclePosition_deg(:, 1); ...
        containingCirclePosition_deg(:, 1)}, ...
    {containingCirclePosition_deg(:, 2); containingCirclePosition_deg(:, 2)}, safetyMargin_deg);

obstacles = obstacleAvoidance.obstacles.combineObstacles(transitCircle, containingCircle);

%% Section 3: Create Planner Inputs

% The target waits at the circle center and then moves outward. Positive azimuth
% motion gives a deterministic exit from occupied space.

initialState = struct( "time_s", 0, "position_deg", [-8 0], "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);

targetTime_s = (0:4:missionEndTime_s).';
targetWaitSampleCount = 3;
postWaitSampleCount = numel(targetTime_s) - targetWaitSampleCount;

% The target first stays at the circle center. It then follows the given outward
% path. Positive azimuth motion makes the exit deterministic.
postWaitAzimuthStep_deg = linspace(0.90, 1.15, postWaitSampleCount).';
postWaitElevationStep_deg = linspace(0.22, 0.38, postWaitSampleCount).';
postWaitStep_deg = [postWaitAzimuthStep_deg, postWaitElevationStep_deg];
postWaitPosition_deg = containingCircleCenter_deg + cumsum(postWaitStep_deg, 1);
targetPosition_deg = [ repmat(containingCircleCenter_deg, targetWaitSampleCount, 1); postWaitPosition_deg];
targetMotion = struct( "time_s", targetTime_s, "position_deg", targetPosition_deg, "InterpolationMethod", "linear");

limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [0.8 0.8], "maxJerk_deg_s3", jerkConfiguration.MaxJerk_deg_s3);
interceptOptions = struct( ...
    "InterceptMode", "specifiedTime", ...
    "SpecifiedInterceptTime_s", missionEndTime_s, "MatchTargetVelocity", false, "PlannerOptions", options);

%% Section 4: Run Planner

% Run the specified-time moving-target planner.

result = obstacleAvoidance.planMovingTargetIntercept( obstacles, initialState, targetMotion, limits, interceptOptions);

%% Section 5: Validate Result

% Confirm that the target starts blocked and ends clear. Confirm that the gimbal
% avoids both circles and reaches the target at the set time.

exampleValidation = validateExampleResult( ...
    result, "target exits a containing obstacle", struct("RequireDirectBlocked", true));
obstacleQueryOptions = struct();

targetOccupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
    result.Inputs.obstacles, targetPosition_deg(:, 1), targetPosition_deg(:, 2), targetTime_s, obstacleQueryOptions);
firstClearSampleIndex = find(~targetOccupied, 1, "first");
targetStartsInside = targetOccupied(1);
targetEventuallyExits = ~isempty(firstClearSampleIndex) && all(~targetOccupied(firstClearSampleIndex:end));
targetWaitedInside = all(targetOccupied(1:targetWaitSampleCount)) && ...
    all(targetPosition_deg(1:targetWaitSampleCount, :) == containingCircleCenter_deg, "all");

targetIsClearAtIntercept = false;
if result.Success && all(isfinite(result.Intercept.TargetPosition_deg))
    targetIsClearAtIntercept = ~obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
        result.Inputs.obstacles, result.Intercept.TargetPosition_deg(1), ...
        result.Intercept.TargetPosition_deg(2), result.Intercept.Time_s, obstacleQueryOptions);
end

transitCircleIsBetween = initialState.position_deg(1) < transitCircleCenter_deg(1) && ...
    transitCircleCenter_deg(1) < containingCircleCenter_deg(1);
targetTravel_deg = sum(vecnorm(diff(targetPosition_deg, 1, 1), 2, 2));
successfulInterceptValidated = result.Success && targetIsClearAtIntercept;
scenarioValidation = struct( ...
    "Passed", targetStartsInside && targetWaitedInside && ...
        targetEventuallyExits && transitCircleIsBetween && ...
        targetTravel_deg > 0 && successfulInterceptValidated, ...
    "TargetStartsInside", targetStartsInside, ...
    "TargetWaitedInside", targetWaitedInside, ...
    "TargetEventuallyExits", targetEventuallyExits, ...
    "TargetIsClearAtIntercept", targetIsClearAtIntercept, ...
    "TargetFrameFailureReported", false, ...
    "TransitCircleIsBetween", transitCircleIsBetween, ...
    "TargetOccupiedAtInputSamples", targetOccupied, ...
    "FirstClearTargetSampleIndex", firstClearSampleIndex, "TargetTravel_deg", targetTravel_deg);

if ~scenarioValidation.Passed
    exampleValidation.Passed = false;
    exampleValidation.Message = "Target containment or obstacle-placement validation failed.";
end
if ~exampleValidation.Passed
    warning("exampleTargetExitsObstacle:ValidationFailed", ...
        "%s", exampleValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion

% Show the target exit and the gimbal detour on one time axis.

if jerkConfiguration.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory( ...
        result, jerkConfiguration.PlotOptions);
end

end
