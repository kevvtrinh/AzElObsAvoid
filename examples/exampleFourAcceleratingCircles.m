function [result, diagnosis] = exampleFourAcceleratingCircles(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleFourAcceleratingCircles()
%   result = exampleFourAcceleratingCircles(exampleOverrides)
%
% PURPOSE
%   - Plan through four vertically moving circles: two rise and two fall.
%   - Intercept a sampled target trajectory while those obstacles move.
%   - Demonstrate the shortest center-line route before the circles close it.
%   - Use a smooth rest-to-rest center profile instead of constant speed.
%   - Keep neighboring original circles tangent at their shared midpoint.
%
% INPUTS
%   - exampleOverrides (scalar struct, optional; default struct())
%       Planner overrides plus the shared FigureVisible, PlotOutputs,
%       ShowAnimation, ShowKinematicPlot, and MaxJerk_units_s3 controls.
%
% OUTPUTS
%   - result (scalar planner-result struct)
%       Validated moving-target intercept, obstacle history, circle-center
%       kinematics, tangency checks, and optional plot handles.
%
% UNITS
%   - Position is coordinate units; time is seconds; derivatives use units/s,
%     units/s^2, and units/s^3.
%

%% Section 1: Resolve Example Controls

% Use fixed arrival and disable wrapping. This keeps the center route inside the
% stated workspace.

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end

[options, jerkConfiguration] = resolveExampleOptions(exampleOverrides, struct("GoalTimeMode", "fixedArrival", "SampleTime_s", 0.05, "WrapX", false, "FigureVisible", "on", "Title", "Moving target among four accelerating circles"), [2.5 2.5]);

% This example demonstrates motion with time limits. Display and search
% overrides remain valid. X wrapping would change the scenario meaning.
options.WrapX = false;

%% Section 2: Create Obstacles

% Four tangent circles move vertically. Two move up and two move down. Their
% centers use smooth acceleration and deceleration.

missionEndTime_s         = 22;
obstacleMotionDuration_s = 42;
obstacleSampleTime_s     = 0.10;
obstacleTime_s           = (0:obstacleSampleTime_s:missionEndTime_s).';

circleRadius_units        = 1.50;
safetyMargin_units        = 0.15;
circleCenterX_units = [-4.5 -1.5 1.5 4.5];
circleDirection         = [1 1 -1 -1];
circleTravel_units        = 7.0;

% Close the center line late enough for a direct pass. The remaining fixed-time
% slack lets the mechanism reach the target without extra path length.

% A quintic smoothstep has zero velocity and acceleration at both ends. The
% circles accelerate and then decelerate. They stay at their final ys.
normalizedTime          = min(obstacleTime_s / obstacleMotionDuration_s, 1);
travelFraction          = 10 * normalizedTime.^3 - 15 * normalizedTime.^4 + 6 * normalizedTime.^5;
travelAcceleration_1_s2 = (60 * normalizedTime - 180 * normalizedTime.^2 + 120 * normalizedTime.^3) / obstacleMotionDuration_s^2;

motionIsComplete = obstacleTime_s >= obstacleMotionDuration_s;
travelAcceleration_1_s2(motionIsComplete) = 0;

initialCenterY_units      = -0.5 * circleTravel_units .* circleDirection;
circleCenterY_units       = initialCenterY_units + circleTravel_units * travelFraction .* circleDirection;
circleCenterAcceleration_units_s2 = circleTravel_units * travelAcceleration_1_s2 .* circleDirection;

circleAngle_rad  = (0:71).' * (2 * pi / 72);
unitCircle       = [cos(circleAngle_rad), sin(circleAngle_rad)];
obstacleByCircle = cell(1, numel(circleCenterX_units));

% Build one sampled boundary history for each circle.
for circleIndex = 1:numel(circleCenterX_units)
    circleXByTime_units   = cell(numel(obstacleTime_s), 1);
    circleYByTime_units = cell(numel(obstacleTime_s), 1);

    % Move the shared circle outline to its center at each sample time.
    for sampleIndex = 1:numel(obstacleTime_s)
        center_units         = [circleCenterX_units(circleIndex), circleCenterY_units(sampleIndex, circleIndex)];
        circlePosition_units = center_units + circleRadius_units * unitCircle;
        circleXByTime_units{sampleIndex} = circlePosition_units(:, 1);
        circleYByTime_units{sampleIndex} = circlePosition_units(:, 2);
    end
    obstacleByCircle{circleIndex} = obstacleAvoidance.obstacles.createObstacle("Accelerating circle " + circleIndex, obstacleTime_s, circleXByTime_units, circleYByTime_units, safetyMargin_units);
end

obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacleByCircle{:});

%% Section 3: Create Planner Inputs

% The target has a curved path beyond the obstacle field. Position-only capture
% lets the target keep moving when the gimbal reaches its position.

initialState = struct();
initialState.time_s              = 0;
initialState.position_units        = [-10 0];
initialState.velocity_units_s      = [0 0];
initialState.acceleration_units_s2 = [0 0];

% The target stays to the right of the obstacle field. It follows a curved path.
% The target can move at interception. The gimbal stops at the same position.
targetTime_s       = [0; 4; 8; 12; 16; 20; missionEndTime_s];
targetPosition_units = [ 8.0 -1.5; 8.3 -1.0; 8.8 0.2; 9.2 0.8; 9.5 0.5; 9.8 -0.4; 10.0 0.0];
targetMotion       = struct("time_s", targetTime_s, "position_units", targetPosition_units, "InterpolationMethod", "pchip");
limits             = struct("maxVelocity_units_s", [2 2], ...
    "maxAcceleration_units_s2", [0.75 0.75], "maxJerk_units_s3", jerkConfiguration.MaxJerk_units_s3);
interceptOptions = struct("InterceptMode", "specifiedTime", ...
    "SpecifiedInterceptTime_s", missionEndTime_s, "MatchTargetVelocity", false, "PlannerOptions", options);

%% Section 4: Run Planner

% Run the moving-target planner with the full obstacle history.

goalState = struct("time_s",interceptOptions.SpecifiedInterceptTime_s,"targetMotion",targetMotion);
plannerOptions = interceptOptions.PlannerOptions;
plannerOptions.GoalTimeMode = "fixedArrival";
[result, diagnosis] = planner(obstacles, initialState, goalState, limits, plannerOptions);

%% Section 5: Validate Result

% Check the intercept, circle tangency, center motion, collision freedom, and
% gimbal limits. These checks separate setup errors from planner errors.

exampleValidation = validateExampleResult(result, "four accelerating circles", struct(), diagnosis);

midpointIndex          = find(obstacleTime_s == 0.5 * obstacleMotionDuration_s, 1, "first");
pairCenterDistance_units = [ ...
    circleCenterX_units(2) - circleCenterX_units(1), circleCenterX_units(4) - circleCenterX_units(3)];
midpointCenterStep_units = hypot(diff(circleCenterX_units), diff(circleCenterY_units(midpointIndex, :)));
tangencyTolerance_units  = 1e-12;

pairsRemainTangent                 = all(abs(pairCenterDistance_units - 2 * circleRadius_units) <= tangencyTolerance_units);
allCirclesTangentAtMidpoint        = all(abs(midpointCenterStep_units - 2 * circleRadius_units) <= tangencyTolerance_units);
profileEndpointNormalizedTime      = [0; 1];
profileEndpointRate_1_s            = (30 * profileEndpointNormalizedTime .^ 2 - 60 * profileEndpointNormalizedTime .^ 3 + 30 * profileEndpointNormalizedTime .^ 4) / obstacleMotionDuration_s;
profileEndpointAcceleration_1_s2   = (60 * profileEndpointNormalizedTime - 180 * profileEndpointNormalizedTime .^ 2 + 120 * profileEndpointNormalizedTime .^ 3) / obstacleMotionDuration_s ^ 2;
profileEndpointVelocity_units_s      = circleTravel_units * profileEndpointRate_1_s .* circleDirection;
profileEndpointAcceleration_units_s2 = circleTravel_units * profileEndpointAcceleration_1_s2 .* circleDirection;
zeroEndpointVelocity               = all(abs(profileEndpointVelocity_units_s) <= 1e-12, "all");
zeroEndpointAcceleration           = all(abs(profileEndpointAcceleration_units_s2) <= 1e-12, "all");
hasAccelerationAndDeceleration     = any(circleCenterAcceleration_units_s2(2:midpointIndex - 1, 1) > 0) && any(circleCenterAcceleration_units_s2(midpointIndex + 1:end, 1) < 0);

circleMotionValidation = struct("Passed", pairsRemainTangent && allCirclesTangentAtMidpoint && ...
        zeroEndpointVelocity && zeroEndpointAcceleration && ...
        hasAccelerationAndDeceleration, ...
    "PairsRemainTangent", pairsRemainTangent, ...
    "AllCirclesTangentAtMidpoint", allCirclesTangentAtMidpoint, ...
    "ZeroEndpointVelocity", zeroEndpointVelocity, ...
    "ZeroEndpointAcceleration", zeroEndpointAcceleration, ...
    "HasAccelerationAndDeceleration", hasAccelerationAndDeceleration, "TangencyTolerance_units", tangencyTolerance_units);

targetTravel_units             = sum(vecnorm(diff(targetPosition_units, 1, 1), 2, 2));
targetEndpointError_units      = norm(result.Intercept.TargetPosition_units - targetPosition_units(end, :));
targetSpeedAtIntercept_units_s = norm((targetPosition_units(end, :) - targetPosition_units(end - 1, :)) / (targetTime_s(end) - targetTime_s(end - 1)));
movingTargetValidation       = struct("Passed", result.Success && result.Validation.Passed && ...
        targetTravel_units > 0 && targetEndpointError_units <= 1e-10 && ...
        targetSpeedAtIntercept_units_s > 0, ...
    "InterceptPassed", result.Success && result.Validation.Passed, ...
    "TargetTravel_units", targetTravel_units, ...
    "TargetEndpointError_units", targetEndpointError_units, ...
    "TargetSpeedAtIntercept_units_s", targetSpeedAtIntercept_units_s, ...
    "PositionOnlyCapture", result.Intercept.TerminalVelocityPolicy == "zero");

shortestRouteLength_units    = norm(targetPosition_units(end, :) - initialState.position_units);
sampledRouteLength_units     = sum(vecnorm(diff(result.position_units, 1, 1), 2, 2));
maximumCenterLineError_units = max(abs(result.position_units(:, 2) - initialState.position_units(2)));
shortestRouteTolerance_units = 1e-6;
shortestRouteValidation    = struct("Passed", result.Success && ...
        abs(sampledRouteLength_units - shortestRouteLength_units) <= shortestRouteTolerance_units && maximumCenterLineError_units <= shortestRouteTolerance_units, "TheoreticalMinimumLength_units", shortestRouteLength_units, "SampledRouteLength_units", sampledRouteLength_units, "MaximumCenterLineError_units", maximumCenterLineError_units, "Tolerance_units", shortestRouteTolerance_units);

if ~circleMotionValidation.Passed
    exampleValidation.Passed  = false;
    exampleValidation.Message = "Circle motion or tangency validation failed.";
end
if ~movingTargetValidation.Passed
    exampleValidation.Passed  = false;
    exampleValidation.Message = "Moving-target intercept validation failed.";
end
if ~shortestRouteValidation.Passed
    exampleValidation.Passed  = false;
    exampleValidation.Message = "The selected motion is not the shortest center-line route.";
end
if ~exampleValidation.Passed
    warning("exampleFourAcceleratingCircles:ValidationFailed", "%s", exampleValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion

% Show the gimbal, target, and circles on one time base.

if jerkConfiguration.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory(result, jerkConfiguration.PlotOptions, diagnosis);
end

end
