function result = exampleFourAcceleratingCircles(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleFourAcceleratingCircles()
%   result = exampleFourAcceleratingCircles(exampleOverrides)
%**************************************************************************
% PURPOSE
%   - Plan through four vertically moving circles: two rise and two fall.
%   - Intercept a sampled target trajectory while those obstacles move.
%   - Demonstrate the shortest center-line route before the circles close it.
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

% Use fixed arrival and disable wrapping. This keeps the center route inside the
% stated workspace.

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end

[options, jerkConfiguration] = resolveExampleOptions( ...
    exampleOverrides, struct( ...
        "GoalTimeMode", "fixedArrival", ...
        "SampleTime_s", 0.05, ...
        "AllowAzimuthWrapping", false, ...
        "FigureVisible", "on", "Title", "Moving target among four accelerating circles"), [2.5 2.5]);

% This example demonstrates motion with time limits. Display and search
% overrides remain valid. Azimuth wrapping would change the scenario meaning.
options.AllowAzimuthWrapping = false;

%% Section 2: Create Obstacles

% Four tangent circles move vertically. Two move up and two move down. Their
% centers use smooth acceleration and deceleration.

missionEndTime_s = 22;
obstacleMotionDuration_s = 42;
obstacleSampleTime_s = 0.10;
obstacleTime_s = (0:obstacleSampleTime_s:missionEndTime_s).';

circleRadius_deg = 1.50;
safetyMargin_deg = 0.15;
circleCenterAzimuth_deg = [-4.5 -1.5 1.5 4.5];
circleDirection = [1 1 -1 -1];
circleTravel_deg = 7.0;

% Close the center line late enough for a direct pass. The remaining fixed-time
% slack lets the mechanism reach the target without extra path length.

% A quintic smoothstep has zero velocity and acceleration at both ends. The
% circles accelerate and then decelerate. They stay at their final elevations.
normalizedTime = min(obstacleTime_s / obstacleMotionDuration_s, 1);
travelFraction = 10 * normalizedTime.^3 - 15 * normalizedTime.^4 + 6 * normalizedTime.^5;
travelAcceleration_1_s2 = (60 * normalizedTime - ...
    180 * normalizedTime.^2 + 120 * normalizedTime.^3) / obstacleMotionDuration_s^2;

motionIsComplete = obstacleTime_s >= obstacleMotionDuration_s;
travelAcceleration_1_s2(motionIsComplete) = 0;

initialCenterElevation_deg = -0.5 * circleTravel_deg .* circleDirection;
circleCenterElevation_deg = initialCenterElevation_deg + circleTravel_deg * travelFraction .* circleDirection;
circleCenterAcceleration_deg_s2 = circleTravel_deg * travelAcceleration_1_s2 .* circleDirection;

circleAngle_rad = (0:71).' * (2 * pi / 72);
unitCircle = [cos(circleAngle_rad), sin(circleAngle_rad)];
obstacleByCircle = cell(1, numel(circleCenterAzimuth_deg));

% Build one sampled boundary history for each circle.
for circleIndex = 1:numel(circleCenterAzimuth_deg)
    circleAzimuthByTime_deg = cell(numel(obstacleTime_s), 1);
    circleElevationByTime_deg = cell(numel(obstacleTime_s), 1);

    % Move the shared circle outline to its center at each sample time.
    for sampleIndex = 1:numel(obstacleTime_s)
        center_deg = [circleCenterAzimuth_deg(circleIndex), circleCenterElevation_deg(sampleIndex, circleIndex)];
        circlePosition_deg = center_deg + circleRadius_deg * unitCircle;
        circleAzimuthByTime_deg{sampleIndex} = circlePosition_deg(:, 1);
        circleElevationByTime_deg{sampleIndex} = circlePosition_deg(:, 2);
    end
    obstacleByCircle{circleIndex} = obstacleAvoidance.obstacles.createObstacle( ...
        "Accelerating circle " + circleIndex, obstacleTime_s, ...
        circleAzimuthByTime_deg, circleElevationByTime_deg, safetyMargin_deg);
end

obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacleByCircle{:});

%% Section 3: Create Planner Inputs

% The target has a curved path beyond the obstacle field. Position-only capture
% lets the target keep moving when the gimbal reaches its position.

initialState = struct( "time_s", 0, "position_deg", [-10 0], "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);

% The target stays to the right of the obstacle field. It follows a curved path.
% The target can move at interception. The gimbal stops at the same position.
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

% Run the moving-target planner with the full obstacle history.

result = obstacleAvoidance.planMovingTargetIntercept( obstacles, initialState, targetMotion, limits, interceptOptions);

%% Section 5: Validate Result

% Check the intercept, circle tangency, center motion, collision freedom, and
% gimbal limits. These checks separate setup errors from planner errors.

exampleValidation = validateExampleResult( ...
    result, "four accelerating circles");

midpointIndex = find( obstacleTime_s == 0.5 * obstacleMotionDuration_s, 1, "first");
pairCenterDistance_deg = [ ...
    circleCenterAzimuth_deg(2) - circleCenterAzimuth_deg(1), circleCenterAzimuth_deg(4) - circleCenterAzimuth_deg(3)];
midpointCenterStep_deg = hypot( diff(circleCenterAzimuth_deg), diff(circleCenterElevation_deg(midpointIndex, :)));
tangencyTolerance_deg = 1e-12;

pairsRemainTangent = all(abs( pairCenterDistance_deg - 2 * circleRadius_deg) <= tangencyTolerance_deg);
allCirclesTangentAtMidpoint = all(abs( midpointCenterStep_deg - 2 * circleRadius_deg) <= tangencyTolerance_deg);
profileEndpointNormalizedTime = [0; 1];
profileEndpointRate_1_s = (30 * profileEndpointNormalizedTime .^ 2 - ...
    60 * profileEndpointNormalizedTime .^ 3 + ...
    30 * profileEndpointNormalizedTime .^ 4) / obstacleMotionDuration_s;
profileEndpointAcceleration_1_s2 = ( ...
    60 * profileEndpointNormalizedTime - ...
    180 * profileEndpointNormalizedTime .^ 2 + ...
    120 * profileEndpointNormalizedTime .^ 3) / ...
    obstacleMotionDuration_s ^ 2;
profileEndpointVelocity_deg_s = ...
    circleTravel_deg * profileEndpointRate_1_s .* circleDirection;
profileEndpointAcceleration_deg_s2 = ...
    circleTravel_deg * profileEndpointAcceleration_1_s2 .* circleDirection;
zeroEndpointVelocity = all( ...
    abs(profileEndpointVelocity_deg_s) <= 1e-12, "all");
zeroEndpointAcceleration = all( ...
    abs(profileEndpointAcceleration_deg_s2) <= 1e-12, "all");
hasAccelerationAndDeceleration = any(circleCenterAcceleration_deg_s2(2:midpointIndex - 1, 1) > 0) && ...
    any(circleCenterAcceleration_deg_s2(midpointIndex + 1:end, 1) < 0);

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

shortestRouteLength_deg = norm( ...
    targetPosition_deg(end, :) - initialState.position_deg);
sampledRouteLength_deg = sum(vecnorm( ...
    diff(result.position_deg, 1, 1), 2, 2));
maximumCenterLineError_deg = max(abs( ...
    result.position_deg(:, 2) - initialState.position_deg(2)));
shortestRouteTolerance_deg = 1e-6;
shortestRouteValidation = struct( ...
    "Passed", result.Success && ...
        abs(sampledRouteLength_deg - shortestRouteLength_deg) <= ...
        shortestRouteTolerance_deg && ...
        maximumCenterLineError_deg <= shortestRouteTolerance_deg, ...
    "TheoreticalMinimumLength_deg", shortestRouteLength_deg, ...
    "SampledRouteLength_deg", sampledRouteLength_deg, ...
    "MaximumCenterLineError_deg", maximumCenterLineError_deg, ...
    "Tolerance_deg", shortestRouteTolerance_deg);

if ~circleMotionValidation.Passed
    exampleValidation.Passed = false;
    exampleValidation.Message = "Circle motion or tangency validation failed.";
end
if ~movingTargetValidation.Passed
    exampleValidation.Passed = false;
    exampleValidation.Message = "Moving-target intercept validation failed.";
end
if ~shortestRouteValidation.Passed
    exampleValidation.Passed = false;
    exampleValidation.Message = ...
        "The selected motion is not the shortest center-line route.";
end
if ~exampleValidation.Passed
    warning("exampleFourAcceleratingCircles:ValidationFailed", ...
        "%s", exampleValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion

% Show the gimbal, target, and circles on one time base.

if jerkConfiguration.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory( ...
        result, jerkConfiguration.PlotOptions);
end

end
