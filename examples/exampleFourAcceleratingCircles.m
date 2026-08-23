function result = exampleFourAcceleratingCircles(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleFourAcceleratingCircles()
%   result = exampleFourAcceleratingCircles(exampleOverrides)
%**************************************************************************
% PURPOSE
%   - Plan through four vertically moving circles: two rise and two fall.
%   - Intercept a sampled target trajectory while those obstacles move.
%   - Use a smooth rest-to-rest center profile instead of constant speed.
%   - Keep neighboring original circles tangent at their shared midpoint.
%**************************************************************************
% INPUTS
%   - exampleOverrides (scalar struct, optional; default struct())
%       Planner overrides plus the shared FigureVisible, PlotOutputs,
%       ShowAnimation, ShowKinematicPlot, and MaxJerk_deg_s3 controls.
%**************************************************************************
% OUTPUTS
%   - result (scalar planner-result struct)
%       Validated moving-target intercept, obstacle history, circle-center
%       kinematics, tangency checks, and optional plot handles.
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
        "FigureVisible", "on", "Title", "Moving target among four accelerating circles"), [2.5 2.5]);

% This example demonstrates interval-constrained motion. Custom search and
% display controls remain valid, but wrapping would change its meaning.
options.AllowAzimuthWrapping = false;

%% Section 2: Create Obstacles

missionEndTime_s = 22;
obstacleMotionDuration_s = 20;
obstacleSampleTime_s = 0.10;
obstacleTime_s = (0:obstacleSampleTime_s:missionEndTime_s).';

circleRadius_deg = 1.50;
safetyMargin_deg = 0.15;
circleCenterAzimuth_deg = [-4.5 -1.5 1.5 4.5];
circleDirection = [1 1 -1 -1];
circleTravel_deg = 7.0;

% A quintic smoothstep has zero velocity and acceleration at both ends.
% The circles accelerate into the scene, reach maximum speed halfway,
% decelerate out, and then remain at their terminal elevations.
normalizedTime = min(obstacleTime_s / obstacleMotionDuration_s, 1);
travelFraction = 10 * normalizedTime.^3 - 15 * normalizedTime.^4 + 6 * normalizedTime.^5;
travelRate_1_s = (30 * normalizedTime.^2 - 60 * normalizedTime.^3 + 30 * normalizedTime.^4) / obstacleMotionDuration_s;
travelAcceleration_1_s2 = (60 * normalizedTime - ...
    180 * normalizedTime.^2 + 120 * normalizedTime.^3) / obstacleMotionDuration_s^2;

motionIsComplete = obstacleTime_s >= obstacleMotionDuration_s;
travelRate_1_s(motionIsComplete) = 0;
travelAcceleration_1_s2(motionIsComplete) = 0;

initialCenterElevation_deg = -0.5 * circleTravel_deg .* circleDirection;
circleCenterElevation_deg = initialCenterElevation_deg + circleTravel_deg * travelFraction .* circleDirection;
circleCenterVelocity_deg_s = circleTravel_deg * travelRate_1_s .* circleDirection;
circleCenterAcceleration_deg_s2 = circleTravel_deg * travelAcceleration_1_s2 .* circleDirection;

circleAngle_rad = (0:71).' * (2 * pi / 72);
unitCircle = [cos(circleAngle_rad), sin(circleAngle_rad)];
obstacleByCircle = cell(1, numel(circleCenterAzimuth_deg));

for circleIndex = 1:numel(circleCenterAzimuth_deg)
    circleAzimuthByTime_deg = cell(numel(obstacleTime_s), 1);
    circleElevationByTime_deg = cell(numel(obstacleTime_s), 1);

    for sampleIndex = 1:numel(obstacleTime_s)
        center_deg = [circleCenterAzimuth_deg(circleIndex), circleCenterElevation_deg(sampleIndex, circleIndex)];
        circlePosition_deg = center_deg + circleRadius_deg * unitCircle;
        circleAzimuthByTime_deg{sampleIndex} = circlePosition_deg(:, 1);
        circleElevationByTime_deg{sampleIndex} = circlePosition_deg(:, 2);
    end
    obstacleByCircle{circleIndex} = makeAzElObstacleData( ...
        "Accelerating circle " + circleIndex, obstacleTime_s, ...
        circleAzimuthByTime_deg, circleElevationByTime_deg, safetyMargin_deg);
end

obstacles = combineAzElObstacles(obstacleByCircle{:});

%% Section 3: Create Planner Inputs

initialState = struct( "time_s", 0, "position_deg", [-10 0], "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);

% The target remains to the right of the obstacle field while following a
% visibly curved sampled trajectory. Position-only capture is intentional:
% the target is still moving at interception, while the gimbal request ends
% at rest after reaching the same azimuth/elevation point.
targetTime_s = [0; 4; 8; 12; 16; 20; missionEndTime_s];
targetPosition_deg = [ 8.0 -1.5; 8.3 -1.0; 8.8 0.2; 9.2 0.8; 9.5 0.5; 9.8 -0.4; 10.0 0.0];
targetMotion = struct( "time_s", targetTime_s, "position_deg", targetPosition_deg, "InterpolationMethod", "pchip");
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [0.75 0.75], "maxJerk_deg_s3", jerkConfiguration.MaxJerk_deg_s3);
interceptOptions = struct( ...
    "InterceptMode", "specifiedTime", ...
    "SpecifiedInterceptTime_s", missionEndTime_s, "MatchTargetVelocity", false, "PlannerOptions", options);

%% Section 4: Run Planner

result = planAzElMovingTargetIntercept( obstacles, initialState, targetMotion, limits, interceptOptions);

%% Section 5: Validate Result

exampleValidation = validateAzElExampleResult( ...
    result, "four accelerating circles", struct("RequireDirectBlocked", true));

midpointIndex = find( obstacleTime_s == 0.5 * obstacleMotionDuration_s, 1, "first");
motionEndIndex = find( obstacleTime_s == obstacleMotionDuration_s, 1, "first");
pairCenterDistance_deg = [ ...
    circleCenterAzimuth_deg(2) - circleCenterAzimuth_deg(1), circleCenterAzimuth_deg(4) - circleCenterAzimuth_deg(3)];
midpointCenterStep_deg = hypot( diff(circleCenterAzimuth_deg), diff(circleCenterElevation_deg(midpointIndex, :)));
tangencyTolerance_deg = 1e-12;

pairsRemainTangent = all(abs( pairCenterDistance_deg - 2 * circleRadius_deg) <= tangencyTolerance_deg);
allCirclesTangentAtMidpoint = all(abs( midpointCenterStep_deg - 2 * circleRadius_deg) <= tangencyTolerance_deg);
zeroEndpointVelocity = all(abs( circleCenterVelocity_deg_s([1 motionEndIndex], :)) <= 1e-12, "all");
zeroEndpointAcceleration = all(abs( circleCenterAcceleration_deg_s2([1 motionEndIndex], :)) <= 1e-12, "all");
hasAccelerationAndDeceleration = any(circleCenterAcceleration_deg_s2(2:midpointIndex - 1, 1) > 0) && ...
    any(circleCenterAcceleration_deg_s2( midpointIndex + 1:motionEndIndex - 1, 1) < 0);

circleMotionValidation = struct( ...
    "Passed", pairsRemainTangent && allCirclesTangentAtMidpoint && ...
        zeroEndpointVelocity && zeroEndpointAcceleration && ...
        hasAccelerationAndDeceleration, ...
    "PairsRemainTangent", pairsRemainTangent, ...
    "AllCirclesTangentAtMidpoint", allCirclesTangentAtMidpoint, ...
    "ZeroEndpointVelocity", zeroEndpointVelocity, ...
    "ZeroEndpointAcceleration", zeroEndpointAcceleration, ...
    "HasAccelerationAndDeceleration", hasAccelerationAndDeceleration, "TangencyTolerance_deg", tangencyTolerance_deg);

targetTravel_deg = sum(vecnorm(diff(targetPosition_deg, 1, 1), 2, 2));
targetEndpointError_deg = norm( result.Intercept.TargetPosition_deg - targetPosition_deg(end, :));
targetSpeedAtIntercept_deg_s = norm( ...
    (targetPosition_deg(end, :) - targetPosition_deg(end - 1, :)) / (targetTime_s(end) - targetTime_s(end - 1)));
movingTargetValidation = struct( ...
    "Passed", result.Success && result.Validation.Passed && ...
        targetTravel_deg > 0 && targetEndpointError_deg <= 1e-10 && ...
        targetSpeedAtIntercept_deg_s > 0, ...
    "InterceptPassed", result.Success && result.Validation.Passed, ...
    "TargetTravel_deg", targetTravel_deg, ...
    "TargetEndpointError_deg", targetEndpointError_deg, ...
    "TargetSpeedAtIntercept_deg_s", targetSpeedAtIntercept_deg_s, ...
    "PositionOnlyCapture", ~result.Intercept.Options.MatchTargetVelocity);

if ~circleMotionValidation.Passed
    exampleValidation.Passed = false;
    exampleValidation.Message = "Circle motion or tangency validation failed.";
end
if ~movingTargetValidation.Passed
    exampleValidation.Passed = false;
    exampleValidation.Message = "Moving-target intercept validation failed.";
end

%% Section 6: Plot Diagnostics And Motion

result.PlotHandles = struct();
if jerkConfiguration.PlotOutputs
    result.PlotHandles = plotAzElMotion( result, jerkConfiguration.PlotOptions);
end

%% Section 7: Return Example Metadata

result.ExampleValidation = exampleValidation;
result.CircleMotionValidation = circleMotionValidation;
result.MovingTargetValidation = movingTargetValidation;
result.ExampleConfiguration = jerkConfiguration;
result.ExampleInputs = struct( ...
    "obstacles", obstacles, ...
    "initialState", initialState, "targetMotion", targetMotion, "limits", limits, "interceptOptions", interceptOptions);
result.obstacleTime_s = obstacleTime_s;
result.obstacleMotionDuration_s = obstacleMotionDuration_s;
result.circleRadius_deg = circleRadius_deg;
result.safetyMargin_deg = safetyMargin_deg;
result.circleCenterAzimuth_deg = circleCenterAzimuth_deg;
result.circleCenterElevation_deg = circleCenterElevation_deg;
result.circleCenterVelocity_deg_s = circleCenterVelocity_deg_s;
result.circleCenterAcceleration_deg_s2 = circleCenterAcceleration_deg_s2;
result.targetTime_s = targetTime_s;
result.targetPosition_deg = targetPosition_deg;
result.ExampleMetrics = computeAzElExampleMetrics(result);
end
