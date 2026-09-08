function [result, diagnosis] = exampleStraightTargetAlternatingOcclusion(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleStraightTargetAlternatingOcclusion()
%   result = exampleStraightTargetAlternatingOcclusion(exampleOverrides)
%
% PURPOSE
%   - Move one target on a straight line through a square, circle,
%     12-point star, and U-shaped obstacle.
%   - Validate repeated blocked and unblocked target intervals while the
%     faster boresight catches the target in a gap between shapes.
%
% INPUTS
%   - exampleOverrides (scalar struct, optional; default struct())
%       Planner overrides plus the shared FigureVisible, PlotOutputs,
%       ShowAnimation, ShowKinematicPlot, and MaxJerk_units_s3 controls.
%
% OUTPUTS
%   - result (scalar planner-result struct)
%       Validated moving-target intercept, occupancy transitions, shape
%       probes, scenario inputs, and optional plot handles.
%
% UNITS
%   - Position is coordinate units; time is seconds; derivatives use units/s,
%     units/s^2, and units/s^3.
%

%% Section 1: Resolve Example Controls

% The intercept owner fixes each trial time. Use the smooth BMTP method so a
% route with more than two segments is velocity-carried rather than converted
% to an unsupported stop-at-waypoint Ruckig composition.

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end

[options, jerkConfiguration] = resolveExampleOptions(exampleOverrides, struct("GoalTimeMode", "fixedArrival", "SampleTime_s", 0.05, "WrapX", false, "FigureVisible", "on", "Title", "Straight target with alternating occlusion"), [2.5 2.5]);

options.WrapX = false;

%% Section 2: Create Obstacles

% Put four different shapes on the target line. The target enters and exits
% occupied regions many times. This tests convex, curved, star, and concave
% boundaries.

missionEndTime_s = 60;
obstacleTime_s   = [0; missionEndTime_s];
safetyMargin_units = 0.15;

squareCenter_units    = [-9 0];
squareHalfWidth_units = 1.5;
squarePosition_units  = squareCenter_units + [ -squareHalfWidth_units -squareHalfWidth_units; squareHalfWidth_units -squareHalfWidth_units; squareHalfWidth_units  squareHalfWidth_units; -squareHalfWidth_units  squareHalfWidth_units];
squareObstacle      = obstacleAvoidance.obstacles.createObstacle("Square", obstacleTime_s, {squarePosition_units(:, 1); squarePosition_units(:, 1)}, {squarePosition_units(:, 2); squarePosition_units(:, 2)}, safetyMargin_units);

circleCenter_units   = [-4 0];
circleRadius_units   = 1.5;
circleVertexCount  = 72;
circleAngle_rad    = (0:circleVertexCount - 1).' * (2 * pi / circleVertexCount);
circlePosition_units = circleCenter_units + circleRadius_units * [cos(circleAngle_rad), sin(circleAngle_rad)];
circleObstacle     = obstacleAvoidance.obstacles.createObstacle("Circle", obstacleTime_s, {circlePosition_units(:, 1); circlePosition_units(:, 1)}, {circlePosition_units(:, 2); circlePosition_units(:, 2)}, safetyMargin_units);

starCenter_units      = [2 0];
starPointCount      = 12;
starOuterRadius_units = 2.0;
starInnerRadius_units = 0.9;
starVertexCount     = 2 * starPointCount;
starAngle_rad       = (0:starVertexCount - 1).' * (2 * pi / starVertexCount);
starRadius_units      = repmat([starOuterRadius_units; starInnerRadius_units], starPointCount, 1);
starPosition_units    = starCenter_units + starRadius_units .* [cos(starAngle_rad), sin(starAngle_rad)];
starObstacle        = obstacleAvoidance.obstacles.createObstacle("12-point star", obstacleTime_s, {starPosition_units(:, 1); starPosition_units(:, 1)}, {starPosition_units(:, 2); starPosition_units(:, 2)}, safetyMargin_units);

uCenter_units   = [9 0];
uPosition_units = uCenter_units + [ -2.0  2.0; -1.2  2.0; -1.2 -1.2; 1.2 -1.2; 1.2  2.0; 2.0  2.0; 2.0 -2.0; -2.0 -2.0];
uObstacle     = obstacleAvoidance.obstacles.createObstacle("U shape", obstacleTime_s, {uPosition_units(:, 1); uPosition_units(:, 1)}, {uPosition_units(:, 2); uPosition_units(:, 2)}, safetyMargin_units);

obstacles = obstacleAvoidance.obstacles.combineObstacles(squareObstacle, circleObstacle, starObstacle, uObstacle);

%% Section 3: Create Planner Inputs

% Move the target at constant speed on a straight sampled track. The gimbal
% starts behind it and moves faster. The intercept must occur in a clear gap.

initialState = struct();
initialState.time_s              = 0;
initialState.position_units        = [-14 3];
initialState.velocity_units_s      = [0 0];
initialState.acceleration_units_s2 = [0 0];

targetTime_s                  = [0; missionEndTime_s];
targetPosition_units            = [squareCenter_units; 14 0];
targetMotion                  = struct("time_s", targetTime_s, "position_units", targetPosition_units, "InterpolationMethod", "linear");
specifiedInterceptX_units = -1;
specifiedInterceptTime_s      = missionEndTime_s * (specifiedInterceptX_units - targetPosition_units(1, 1)) / diff(targetPosition_units(:, 1));

limits = struct("maxVelocity_units_s", [2 2], ...
    "maxAcceleration_units_s2", [0.8 0.8], "maxJerk_units_s3", jerkConfiguration.MaxJerk_units_s3);
interceptOptions = struct("InterceptMode", "specifiedTime", ...
    "SpecifiedInterceptTime_s", specifiedInterceptTime_s, "MatchTargetVelocity", false, "PlannerOptions", options);

%% Section 4: Run Planner

% Run the moving-target planner without a stored intercept choice.

goalState = struct("time_s",interceptOptions.SpecifiedInterceptTime_s,"targetMotion",targetMotion);
plannerOptions = interceptOptions.PlannerOptions;
plannerOptions.GoalTimeMode = "fixedArrival";
[result, diagnosis] = planner(obstacles, initialState, goalState, limits, plannerOptions);

%% Section 5: Validate Result

% Check the trajectory and independently sample target occupancy. Extra probes
% show whether each shape blocks the intended part of the track.

exampleValidation    = validateExampleResult(result, "straight target with alternating occlusion", struct("RequireDirectBlocked", true), diagnosis);
obstacleQueryOptions = struct();

occupancySampleCount = 1201;
occupancyTime_s      = linspace(0, missionEndTime_s, occupancySampleCount).';
targetX_units    = interp1(targetTime_s, targetPosition_units(:, 1), occupancyTime_s, "linear");
targetY_units  = interp1(targetTime_s, targetPosition_units(:, 2), occupancyTime_s, "linear");
[targetOccupied, blockingObstacleIndex] = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(result.Inputs.obstacles, targetX_units, targetY_units, occupancyTime_s, obstacleQueryOptions);

blockedRunStart          = targetOccupied & [true; ~targetOccupied(1:end - 1)];
clearRunStart            = ~targetOccupied & [true; targetOccupied(1:end - 1)];
blockedRunCount          = nnz(blockedRunStart);
clearRunCount            = nnz(clearRunStart);
occupancyTransitionCount = nnz(diff(targetOccupied) ~= 0);

probeX_units   = [-9; -4; 2; 7.4; 9; 10.6; 14];
probeY_units = zeros(size(probeX_units));
probeTime_s        = missionEndTime_s * (probeX_units - targetPosition_units(1, 1)) / diff(targetPosition_units(:, 1));
[probeOccupied, probeBlockingObstacleIndex] = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(result.Inputs.obstacles, probeX_units, probeY_units, probeTime_s, obstacleQueryOptions);
expectedProbeOccupied              = logical([1; 1; 1; 1; 0; 1; 0]);
expectedProbeBlockingObstacleIndex = uint32([1; 2; 3; 4; 0; 4; 0]);

targetTrackIsStraight = all(abs(targetY_units) <= 1e-12);
targetVelocity_units_s  = diff(targetPosition_units, 1, 1) ./ diff(targetTime_s);
targetSpeed_units_s     = norm(targetVelocity_units_s);
boresightIsFaster     = targetSpeed_units_s < min(limits.maxVelocity_units_s);

interShapeGapBounds_units = [ ...
    max(squarePosition_units(:, 1)) + safetyMargin_units, circleCenter_units(1) - circleRadius_units - safetyMargin_units; circleCenter_units(1) + circleRadius_units + safetyMargin_units, starCenter_units(1) - starOuterRadius_units - safetyMargin_units; starCenter_units(1) + starOuterRadius_units + safetyMargin_units, min(uPosition_units(:, 1)) - safetyMargin_units];
interceptInInterShapeGap = false;
interceptTargetIsClear   = false;
if result.Success
    interceptX_units     = result.Intercept.TargetPosition_units(1);
    interceptInInterShapeGap = any(interceptX_units > interShapeGapBounds_units(:, 1) & interceptX_units < interShapeGapBounds_units(:, 2));
    interceptTargetIsClear   = ~obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(result.Inputs.obstacles, result.Intercept.TargetPosition_units(1), result.Intercept.TargetPosition_units(2), result.Intercept.Time_s, obstacleQueryOptions);
end
catchOccurredBeforeTrackEnd = result.Success && result.Intercept.Time_s < missionEndTime_s;
alternationIsPresent        = targetOccupied(1) && ~targetOccupied(end) && blockedRunCount >= 5 && clearRunCount >= 5 && occupancyTransitionCount >= 9;
shapeProbesPassed           = isequal(probeOccupied, expectedProbeOccupied) && isequal(probeBlockingObstacleIndex, expectedProbeBlockingObstacleIndex);
scenarioValidation          = struct("Passed", targetTrackIsStraight && alternationIsPresent && ...
        shapeProbesPassed && boresightIsFaster && ...
        interceptInInterShapeGap && interceptTargetIsClear && ...
        catchOccurredBeforeTrackEnd, ...
    "TargetTrackIsStraight", targetTrackIsStraight, ...
    "TargetSpeed_units_s", targetSpeed_units_s, ...
    "BoresightIsFaster", boresightIsFaster, ...
    "InterceptInInterShapeGap", interceptInInterShapeGap, ...
    "InterceptTargetIsClear", interceptTargetIsClear, ...
    "CatchOccurredBeforeTrackEnd", catchOccurredBeforeTrackEnd, ...
    "InterShapeGapBounds_units", interShapeGapBounds_units, ...
    "AlternationIsPresent", alternationIsPresent, ...
    "ShapeProbesPassed", shapeProbesPassed, ...
    "BlockedRunCount", blockedRunCount, ...
    "ClearRunCount", clearRunCount, ...
    "OccupancyTransitionCount", occupancyTransitionCount, ...
    "TargetOccupied", targetOccupied, ...
    "BlockingObstacleIndex", blockingObstacleIndex, ...
    "ProbeX_units", probeX_units, ...
    "ProbeTime_s", probeTime_s, ...
    "ProbeOccupied", probeOccupied, "ProbeBlockingObstacleIndex", probeBlockingObstacleIndex);

if ~scenarioValidation.Passed
    exampleValidation.Passed  = false;
    exampleValidation.Message = "Straight-target blocked/free sequence validation failed.";
end
if ~exampleValidation.Passed
    warning("exampleStraightTargetAlternatingOcclusion:ValidationFailed", "%s", exampleValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion

% Show the target track, blocked intervals, and selected intercept.

if jerkConfiguration.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory(result, jerkConfiguration.PlotOptions, diagnosis);
end

end
