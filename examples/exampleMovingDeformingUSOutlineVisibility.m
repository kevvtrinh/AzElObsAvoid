function [result, diagnosis] = exampleMovingDeformingUSOutlineVisibility(options)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleMovingDeformingUSOutlineVisibility()
%   result = exampleMovingDeformingUSOutlineVisibility(options)
%
% PURPOSE
%   - Plan around a dense U.S. outline that starts at 8 percent scale,
%     grows, deforms, completes a 180-degree rotation, and disappears.
%   - Include a starburst sun that moves across the bottom of the scene.
%
% INPUTS
%   - options (scalar struct, optional; default struct())
%       Planner/display overrides plus the finite MaxJerk_units_s3 limit.
%
% OUTPUTS
%   - result (scalar struct)
%       Unmodified public planner result.
%   - diagnosis (optional second output): search attempts and solver details.
%
% UNITS
%   - Position is coordinate units, time is seconds, velocity is coordinate units per second,
%     acceleration is coordinate units per second squared, and jerk is coordinate units per
%     second cubed.
%

%% Section 1: Resolve Example Controls

% Set display and planner options before loading the large outline. A finite
% jerk limit requires smooth physical motion.

if nargin < 1 || isempty(options)
    options = struct();
end
[options, jerkConfiguration] = resolveExampleOptions(options, struct("GoalTimeMode", "earliestArrival", "MaximumDisplayedSlicesPerObstacle", 10, "ShowSweptSurfaces", true, "FigureVisible", "on", "Title", "Extreme growing/rotating U.S. with moving sun"), [12 12]);

%% Section 2: Create Obstacles

% Build two dynamic obstacles. The U.S. outline changes scale, shape, angle, and
% position. A star-shaped sun moves across the lower area.

missionEndTime_s   = 5 * 60;
obstacleTimeStep_s = 5;
uSDisappearTime_s  = 4 * 60;
uSTime_s           = (0:obstacleTimeStep_s:uSDisappearTime_s).';
% The private helper loads and joins more than 14,000 outline vertices. Keeping
% this work in the helper makes the scenario sequence easier to read.
[uSObstacle, uSHistory] = createContiguousUSObstacle(uSTime_s, 0.10, struct("MotionMode", "movingDeforming", "Verbose", jerkConfiguration.Verbose));

sunTime_s          = (0:obstacleTimeStep_s:missionEndTime_s).';
sunRayCount        = 16;
sunAngle_rad       = (0:2 * sunRayCount - 1).' * (pi / sunRayCount);
sunRadius_units      = repmat([2.6; 1.8], sunRayCount, 1);
sunSource_units      = sunRadius_units .* [cos(sunAngle_rad), sin(sunAngle_rad)];
sunStartCenter_units = [uSHistory.centroid_units(1, 1) - 16, 12];
sunEndCenter_units   = [uSHistory.centroid_units(1, 1) + 16, 12];
sunTransform       = @(sourcePosition_units, sampleTime_s, sampleIndex) moveSunSlice(sourcePosition_units, sampleTime_s, sampleIndex, missionEndTime_s, sunStartCenter_units, sunEndCenter_units);
[sunObstacle, sunHistory] = obstacleAvoidance.obstacles.createMovingObstacle("Moving sun", sunTime_s, sunSource_units(:, 1), sunSource_units(:, 2), sunTransform, 0.10, struct("Verbose", jerkConfiguration.Verbose));
obstacles = obstacleAvoidance.obstacles.combineObstacles(uSObstacle, sunObstacle);

%% Section 3: Create Planner Inputs

% Put the start and goal across the moving shapes. This request tests visibility
% candidates while dense geometry deforms and disappears.

routeX_units = uSHistory.centroid_units(1, 1);
initialState     = struct("time_s", 0, "position_units", [routeX_units 18]);
goalState = struct("time_s", missionEndTime_s, ...
    "position_units", [routeX_units 58]);
limits = struct("maxVelocity_units_s", [8 8], "maxAcceleration_units_s2", [3 3], "maxJerk_units_s3", jerkConfiguration.MaxJerk_units_s3);

%% Section 4: Run Planner

% Run the public planner with both obstacle histories.

[result, diagnosis] = planner(obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result

% Check the full trajectory and the outline history. For a failure, inspect
% topology-change diagnostics and collision subdivisions.

exampleValidation = validateExampleResult(result, "extreme moving/deforming U.S. with moving sun", struct("RequireDirectBlocked", true), diagnosis);

initialUS_units = [ ...
    uSHistory.xBySlice_units{1}, ...
    uSHistory.yBySlice_units{1}];
finalUS_units = [ ...
    uSHistory.xBySlice_units{end}, ...
    uSHistory.yBySlice_units{end}];
initialCenteredUS_units = initialUS_units - mean(initialUS_units, 1);
finalCenteredUS_units   = finalUS_units - mean(finalUS_units, 1);
rotationCosine        = sum(initialCenteredUS_units .* finalCenteredUS_units, "all") / (norm(initialCenteredUS_units, "fro") * norm(finalCenteredUS_units, "fro"));
initialAreaFraction   = uSHistory.area_units2(1) / max(uSHistory.area_units2);
[~, inactiveUSGeometry] = obstacleAvoidance.obstacles.shapeAtTime(uSObstacle, missionEndTime_s, true);
uSHistoryValidation = struct("Passed", initialAreaFraction <= 0.01 && ...
        max(uSHistory.scaleFactor) >= 1.35 - 1e-12 && rotationCosine <= -0.999 && ~inactiveUSGeometry.Active, "InitialAreaFraction", initialAreaFraction, "MaximumScaleFactor", max(uSHistory.scaleFactor), "CompletedRotation_deg", uSHistory.rotation_deg(end), "EndpointRotationCosine", rotationCosine, "DisappearTime_s", uSDisappearTime_s, "InactiveAtMissionEnd", ~inactiveUSGeometry.Active);

sunCentroidTravel_units = sum(vecnorm(diff(sunHistory.centroid_units, 1, 1), 2, 2));
sunStayedAtBottom     = max(sunHistory.bounds_units(:, 4)) < initialState.position_units(2);
sunMotionValidation   = struct("Passed", sunCentroidTravel_units >= 32 && sunStayedAtBottom, ...
    "CentroidTravel_units", sunCentroidTravel_units, ...
    "StayedBelowInitialY", sunStayedAtBottom, ...
    "MaximumBoundaryY_units", ...
        max(sunHistory.bounds_units(:, 4)));
if ~uSHistoryValidation.Passed || ~sunMotionValidation.Passed
    exampleValidation.Passed  = false;
    exampleValidation.Message = exampleValidation.Message + " Extreme obstacle-history validation failed.";
end
if ~exampleValidation.Passed
    warning("exampleMovingDeformingUSOutlineVisibility:ValidationFailed", "%s", exampleValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion

% Plot and animate the returned result with the supplied obstacle histories.

if jerkConfiguration.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory(result, jerkConfiguration.PlotOptions, diagnosis);
end

end

%% Section 8: Local Functions

function transformed_units = moveSunSlice(sourcePosition_units, sampleTime_s, ~, missionEndTime_s, startCenter_units, endCenter_units)
    % Move one sun outline smoothly across the lower part of the scene.
    missionProgress = min(max(sampleTime_s / missionEndTime_s, 0), 1);
    travelFraction  = 10 * missionProgress^3 - 15 * missionProgress^4 + 6 * missionProgress^5;
    center_units      = startCenter_units + travelFraction * (endCenter_units - startCenter_units);
    center_units(2) = center_units(2) + 2 * sin(2 * pi * missionProgress);
    transformed_units = sourcePosition_units + center_units;
end
