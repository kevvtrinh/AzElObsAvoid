function result = exampleStraightTargetAlternatingOcclusion(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleStraightTargetAlternatingOcclusion()
%   result = exampleStraightTargetAlternatingOcclusion(exampleOverrides)
%**************************************************************************
% PURPOSE
%   - Move one target on a straight line through a square, circle,
%     12-point star, and U-shaped obstacle.
%   - Validate repeated blocked and unblocked target intervals while the
%     faster boresight catches the target in a gap between shapes.
%**************************************************************************
% INPUTS
%   - exampleOverrides (scalar struct, optional; default struct())
%       Planner overrides plus the shared FigureVisible, PlotOutputs,
%       ShowAnimation, ShowKinematicPlot, and MaxJerk_deg_s3 controls.
%**************************************************************************
% OUTPUTS
%   - result (scalar planner-result struct)
%       Validated moving-target intercept, occupancy transitions, shape
%       probes, scenario inputs, and optional plot handles.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s,
%     deg/s^2, and deg/s^3.
%**************************************************************************

%% Section 1: Resolve Example Controls

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end

[options, jerkConfiguration] = resolveAzElExampleOptions( ...
    exampleOverrides, struct( ...
        "GoalTimeMode", "fixedArrival", ...
        "SampleTime_s", 0.05, ...
        "AllowAzimuthWrapping", false, ...
        "FigureVisible", "on", "Title", "Straight target with alternating occlusion"), [2.5 2.5]);

options.AllowAzimuthWrapping = false;

%% Section 2: Create Obstacles

missionEndTime_s = 60;
obstacleTime_s = [0; missionEndTime_s];
safetyMargin_deg = 0.15;

squareCenter_deg = [-9 0];
squareHalfWidth_deg = 1.5;
squarePosition_deg = squareCenter_deg + [ ...
    -squareHalfWidth_deg -squareHalfWidth_deg; ...
     squareHalfWidth_deg -squareHalfWidth_deg; ...
     squareHalfWidth_deg  squareHalfWidth_deg; -squareHalfWidth_deg  squareHalfWidth_deg];
squareObstacle = makeAzElObstacleData( ...
    "Square", obstacleTime_s, ...
    {squarePosition_deg(:, 1); squarePosition_deg(:, 1)}, ...
    {squarePosition_deg(:, 2); squarePosition_deg(:, 2)}, safetyMargin_deg);

circleCenter_deg = [-4 0];
circleRadius_deg = 1.5;
circleVertexCount = 72;
circleAngle_rad = (0:circleVertexCount - 1).' * (2 * pi / circleVertexCount);
circlePosition_deg = circleCenter_deg + circleRadius_deg * [cos(circleAngle_rad), sin(circleAngle_rad)];
circleObstacle = makeAzElObstacleData( ...
    "Circle", obstacleTime_s, ...
    {circlePosition_deg(:, 1); circlePosition_deg(:, 1)}, ...
    {circlePosition_deg(:, 2); circlePosition_deg(:, 2)}, safetyMargin_deg);

starCenter_deg = [2 0];
starPointCount = 12;
starOuterRadius_deg = 2.0;
starInnerRadius_deg = 0.9;
starVertexCount = 2 * starPointCount;
starAngle_rad = (0:starVertexCount - 1).' * (2 * pi / starVertexCount);
starRadius_deg = repmat( [starOuterRadius_deg; starInnerRadius_deg], starPointCount, 1);
starPosition_deg = starCenter_deg + starRadius_deg .* [cos(starAngle_rad), sin(starAngle_rad)];
starObstacle = makeAzElObstacleData( ...
    "12-point star", obstacleTime_s, ...
    {starPosition_deg(:, 1); starPosition_deg(:, 1)}, ...
    {starPosition_deg(:, 2); starPosition_deg(:, 2)}, safetyMargin_deg);

uCenter_deg = [9 0];
uPosition_deg = uCenter_deg + [ -2.0  2.0; -1.2  2.0; -1.2 -1.2; 1.2 -1.2; 1.2  2.0; 2.0  2.0; 2.0 -2.0; -2.0 -2.0];
uObstacle = makeAzElObstacleData( ...
    "U shape", obstacleTime_s, ...
    {uPosition_deg(:, 1); uPosition_deg(:, 1)}, {uPosition_deg(:, 2); uPosition_deg(:, 2)}, safetyMargin_deg);

obstacles = combineAzElObstacles( squareObstacle, circleObstacle, starObstacle, uObstacle);

%% Section 3: Create Planner Inputs

initialState = struct( "time_s", 0, "position_deg", [-14 3], "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);

targetTime_s = [0; missionEndTime_s];
targetPosition_deg = [squareCenter_deg; 14 0];
targetMotion = struct( "time_s", targetTime_s, "position_deg", targetPosition_deg, "InterpolationMethod", "linear");
specifiedInterceptAzimuth_deg = -1;
specifiedInterceptTime_s = missionEndTime_s * ...
    (specifiedInterceptAzimuth_deg - targetPosition_deg(1, 1)) / diff(targetPosition_deg(:, 1));

limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [0.8 0.8], "maxJerk_deg_s3", jerkConfiguration.MaxJerk_deg_s3);
interceptOptions = struct( ...
    "InterceptMode", "specifiedTime", ...
    "SpecifiedInterceptTime_s", specifiedInterceptTime_s, "MatchTargetVelocity", false, "PlannerOptions", options);

%% Section 4: Run Planner

result = planAzElMovingTargetIntercept( obstacles, initialState, targetMotion, limits, interceptOptions);

%% Section 5: Validate Result

exampleValidation = validateAzElExampleResult( ...
    result, "straight target with alternating occlusion", struct("RequireDirectBlocked", true));
obstacleQueryOptions = struct("PlannerMethod", result.Options.PlannerMethod);

occupancySampleCount = 1201;
occupancyTime_s = linspace(0, missionEndTime_s, occupancySampleCount).';
targetAzimuth_deg = interp1(targetTime_s, targetPosition_deg(:, 1), occupancyTime_s, "linear");
targetElevation_deg = interp1(targetTime_s, targetPosition_deg(:, 2), occupancyTime_s, "linear");
[targetOccupied, blockingObstacleIndex] = queryAzElTimeObstacle( ...
    result.Inputs.obstacles, targetAzimuth_deg, targetElevation_deg, occupancyTime_s, obstacleQueryOptions);

blockedRunStart = targetOccupied & [true; ~targetOccupied(1:end - 1)];
clearRunStart = ~targetOccupied & [true; targetOccupied(1:end - 1)];
blockedRunCount = nnz(blockedRunStart);
clearRunCount = nnz(clearRunStart);
occupancyTransitionCount = nnz(diff(targetOccupied) ~= 0);

probeAzimuth_deg = [-9; -4; 2; 7.4; 9; 10.6; 14];
probeElevation_deg = zeros(size(probeAzimuth_deg));
probeTime_s = missionEndTime_s * (probeAzimuth_deg - targetPosition_deg(1, 1)) / diff(targetPosition_deg(:, 1));
[probeOccupied, probeBlockingObstacleIndex] = queryAzElTimeObstacle( ...
    result.Inputs.obstacles, probeAzimuth_deg, probeElevation_deg, probeTime_s, obstacleQueryOptions);
expectedProbeOccupied = logical([1; 1; 1; 1; 0; 1; 0]);
expectedProbeBlockingObstacleIndex = uint32([1; 2; 3; 4; 0; 4; 0]);

targetTrackIsStraight = all(abs(targetElevation_deg) <= 1e-12);
targetVelocity_deg_s = diff(targetPosition_deg, 1, 1) ./ diff(targetTime_s);
targetSpeed_deg_s = norm(targetVelocity_deg_s);
boresightIsFaster = targetSpeed_deg_s < min(limits.maxVelocity_deg_s);

interShapeGapBounds_deg = [ ...
    max(squarePosition_deg(:, 1)) + safetyMargin_deg, ...
        circleCenter_deg(1) - circleRadius_deg - safetyMargin_deg; ...
    circleCenter_deg(1) + circleRadius_deg + safetyMargin_deg, ...
        starCenter_deg(1) - starOuterRadius_deg - safetyMargin_deg; ...
    starCenter_deg(1) + starOuterRadius_deg + safetyMargin_deg, min(uPosition_deg(:, 1)) - safetyMargin_deg];
interceptInInterShapeGap = false;
interceptTargetIsClear = false;
if result.Success
    interceptAzimuth_deg = result.Intercept.TargetPosition_deg(1);
    interceptInInterShapeGap = any( ...
        interceptAzimuth_deg > interShapeGapBounds_deg(:, 1) & interceptAzimuth_deg < interShapeGapBounds_deg(:, 2));
    interceptTargetIsClear = ~queryAzElTimeObstacle( ...
        result.Inputs.obstacles, result.Intercept.TargetPosition_deg(1), ...
        result.Intercept.TargetPosition_deg(2), result.Intercept.Time_s, obstacleQueryOptions);
end
catchOccurredBeforeTrackEnd = result.Success && result.Intercept.Time_s < missionEndTime_s;
alternationIsPresent = targetOccupied(1) && ~targetOccupied(end) && ...
    blockedRunCount >= 5 && clearRunCount >= 5 && occupancyTransitionCount >= 9;
shapeProbesPassed = isequal(probeOccupied, expectedProbeOccupied) && ...
    isequal(probeBlockingObstacleIndex, expectedProbeBlockingObstacleIndex);
scenarioValidation = struct( ...
    "Passed", targetTrackIsStraight && alternationIsPresent && ...
        shapeProbesPassed && boresightIsFaster && ...
        interceptInInterShapeGap && interceptTargetIsClear && ...
        catchOccurredBeforeTrackEnd, ...
    "TargetTrackIsStraight", targetTrackIsStraight, ...
    "TargetSpeed_deg_s", targetSpeed_deg_s, ...
    "BoresightIsFaster", boresightIsFaster, ...
    "InterceptInInterShapeGap", interceptInInterShapeGap, ...
    "InterceptTargetIsClear", interceptTargetIsClear, ...
    "CatchOccurredBeforeTrackEnd", catchOccurredBeforeTrackEnd, ...
    "InterShapeGapBounds_deg", interShapeGapBounds_deg, ...
    "AlternationIsPresent", alternationIsPresent, ...
    "ShapeProbesPassed", shapeProbesPassed, ...
    "BlockedRunCount", blockedRunCount, ...
    "ClearRunCount", clearRunCount, ...
    "OccupancyTransitionCount", occupancyTransitionCount, ...
    "TargetOccupied", targetOccupied, ...
    "BlockingObstacleIndex", blockingObstacleIndex, ...
    "ProbeAzimuth_deg", probeAzimuth_deg, ...
    "ProbeTime_s", probeTime_s, ...
    "ProbeOccupied", probeOccupied, "ProbeBlockingObstacleIndex", probeBlockingObstacleIndex);

if ~scenarioValidation.Passed
    exampleValidation.Passed = false;
    exampleValidation.Message = "Straight-target blocked/free sequence validation failed.";
end

%% Section 6: Plot Diagnostics And Motion

result.PlotHandles = struct();
if jerkConfiguration.PlotOutputs
    result.PlotHandles = plotAzElMotion( result, jerkConfiguration.PlotOptions);
end

%% Section 7: Return Example Metadata

result.ExampleName = "exampleStraightTargetAlternatingOcclusion";
result.ExampleValidation = exampleValidation;
result.ScenarioValidation = scenarioValidation;
result.ExampleConfiguration = jerkConfiguration;
result.ExampleInputs = struct( ...
    "obstacles", obstacles, ...
    "initialState", initialState, "targetMotion", targetMotion, "limits", limits, "interceptOptions", interceptOptions);
result.targetTime_s = targetTime_s;
result.targetPosition_deg = targetPosition_deg;
result.occupancyTime_s = occupancyTime_s;
result.safetyMargin_deg = safetyMargin_deg;
result.shapeNames = ["Square" "Circle" "12-point star" "U shape"];
result.ExampleMetrics = computeAzElExampleMetrics(result);
end
