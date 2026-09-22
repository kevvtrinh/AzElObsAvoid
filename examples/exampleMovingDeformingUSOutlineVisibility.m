function result = exampleMovingDeformingUSOutlineVisibility(options)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleMovingDeformingUSOutlineVisibility()
%   result = exampleMovingDeformingUSOutlineVisibility(options)
%**************************************************************************
% PURPOSE
%   - Plan around a U.S. outline (reduced to at most 100 vertices) that
%     starts at 8 percent scale, grows, deforms, completes a 180-degree
%     rotation, and disappears.
%   - Include a starburst sun that moves across the bottom of the scene.
%**************************************************************************
% INPUTS
%   - options (scalar struct, optional; default struct())
%       Planner/display overrides plus the finite MaxJerk_units_s3 limit.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Unmodified public planner result. Ordinary planning failure returns
%       Success = false; invalid input throws an error.
%**************************************************************************
% UNITS
%   - Positions are 1-by-2 [x y] rows in coordinate units. Time is seconds;
%     velocity, acceleration, and jerk use units/s, units/s^2, and units/s^3.
%**************************************************************************

%% Section 1: Resolve Example Controls
% Set display and planner options before loading the large outline. A finite
% jerk limit requires smooth physical motion.
if nargin < 1 || isempty(options)
    options = struct();
end
scenarioDefaults = struct( ...
    "GoalTimeMode",  "earliestArrival", ...
    "FigureVisible", "on", ...
    "Title",         "Extreme growing/rotating U.S. with moving sun");
[options, displayOptions] = resolveExampleOptions( ...
    options, scenarioDefaults, [12 12]);

%% Section 2: Create Obstacles
% Build two dynamic obstacles. The U.S. outline changes scale, shape, angle, and
% position. A star-shaped sun moves across the lower area.

missionEndTime_s   = 5 * 60;
obstacleTimeStep_s = 5;
usDisappearTime_s  = 4 * 60;
usTime_s           = (0:obstacleTimeStep_s:usDisappearTime_s).';
% The private helper loads and joins more than 14,000 outline vertices. Keeping
% this work in the helper makes the scenario sequence easier to read.
% The supplied outline is capped at 100 vertices by a deterministic
% Douglas-Peucker reduction inside the helper; that reduced outline is the
% obstacle, and the planner treats it exactly.
[usObstacle, usHistory] = createContiguousUSObstacle( ...
    usTime_s, 0.10, struct( ...
        "MotionMode",             "movingDeforming", ...
        "Verbose",                displayOptions.Verbose, ...
        "MaximumOutlineVertices", 100));

sunTime_s            = (0:obstacleTimeStep_s:missionEndTime_s).';
sunRayCount          = 16;
sunAngle_rad         = (0:2 * sunRayCount - 1).' * (pi / sunRayCount);
sunRadius_units      = repmat([2.6; 1.8], sunRayCount, 1);
sunSource_units      = sunRadius_units .* [cos(sunAngle_rad), sin(sunAngle_rad)];
sunStartCenter_units = [usHistory.centroid_units(1, 1) - 16, 12];
sunEndCenter_units   = [usHistory.centroid_units(1, 1) + 16, 12];
sunTransform         = @(sourcePosition_units, sampleTime_s, sampleIndex) ...
    moveSunSlice(sourcePosition_units, sampleTime_s, sampleIndex, ...
    missionEndTime_s, sunStartCenter_units, sunEndCenter_units);
[sunObstacle, sunHistory] = obstacleAvoidance.obstacles.createMovingObstacle( ...
    "Moving sun", sunTime_s, sunSource_units(:, 1), sunSource_units(:, 2), ...
    sunTransform, 0.10, struct("Verbose", displayOptions.Verbose));
obstacles = obstacleAvoidance.obstacles.combineObstacles(usObstacle, sunObstacle);

%% Section 3: Create Planner Inputs
% Put the start and goal across the moving shapes. This request tests visibility
% candidates while dense geometry deforms and disappears.

routeX_units = usHistory.centroid_units(1, 1);
initialState = struct("time_s", 0, "position_units", [routeX_units 18]);
goalState = struct("time_s", missionEndTime_s, ...
    "position_units", [routeX_units 58]);
limits = struct( ...
    "maxVelocity_units_s",      [8 8], ...
    "maxAcceleration_units_s2", [3 3], ...
    "maxJerk_units_s3",         displayOptions.MaxJerk_units_s3);

%% Section 4: Run Planner
% Run the public planner with both obstacle histories.
result = planner(obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result
% Check the full trajectory and the outline history. For a failure, inspect
% topology-change diagnostics and collision subdivisions.

if result.TerminationReason == "unsupportedObstacleInterpolation"
    exampleValidation = struct( ...
        "Passed",  true, ...
        "Message", ...
        "Planner explicitly rejected a deformation without a proven continuous geometry model.");
else
    exampleValidation = validateExampleResult(result, ...
        "extreme moving/deforming U.S. with moving sun", ...
        struct("RequireDirectBlocked", true));
end

initialUs_units = [ ...
    usHistory.xBySlice_units{1}, ...
    usHistory.yBySlice_units{1}];
finalUs_units = [ ...
    usHistory.xBySlice_units{end}, ...
    usHistory.yBySlice_units{end}];
initialCenteredUs_units = initialUs_units - mean(initialUs_units, 1);
finalCenteredUs_units   = finalUs_units - mean(finalUs_units, 1);
rotationCosine          = sum(initialCenteredUs_units .* finalCenteredUs_units, "all") / ...
    (norm(initialCenteredUs_units, "fro") * norm(finalCenteredUs_units, "fro"));
initialAreaFraction     = usHistory.area_units2(1) / max(usHistory.area_units2);
[~, inactiveUsGeometry] = obstacleAvoidance.obstacles.preparedShapeAtTime( ...
    result.PreparedObstacles(1), missionEndTime_s, true);
usHistoryValidation = struct( ...
    "Passed", initialAreaFraction <= 0.01 && ...
    max(usHistory.scaleFactor) >= 1.35 - 1e-12 && ...
    rotationCosine <= -0.999 && ~inactiveUsGeometry.Active, ...
    "InitialAreaFraction",     initialAreaFraction, ...
    "MaximumScaleFactor",     max(usHistory.scaleFactor), ...
    "CompletedRotation_deg",  usHistory.rotation_deg(end), ...
    "EndpointRotationCosine", rotationCosine, ...
    "DisappearTime_s",        usDisappearTime_s, ...
    "InactiveAtMissionEnd",   ~inactiveUsGeometry.Active);

sunCentroidTravel_units = sum(vecnorm(diff(sunHistory.centroid_units, 1, 1), 2, 2));
sunStayedAtBottom       = max(sunHistory.bounds_units(:, 4)) < initialState.position_units(2);
sunMotionValidation     = struct( ...
    "Passed", sunCentroidTravel_units >= 32 && sunStayedAtBottom, ...
    "CentroidTravel_units",   sunCentroidTravel_units, ...
    "StayedBelowInitialY",    sunStayedAtBottom, ...
    "MaximumBoundaryY_units", max(sunHistory.bounds_units(:, 4)));
if ~usHistoryValidation.Passed || ~sunMotionValidation.Passed
    exampleValidation.Passed  = false;
    exampleValidation.Message = exampleValidation.Message + ...
        " Extreme obstacle-history validation failed.";
end
if ~exampleValidation.Passed
    warning("exampleMovingDeformingUSOutlineVisibility:ValidationFailed", ...
        "%s", exampleValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion
% Plot and animate the returned result with the supplied obstacle histories.
if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory(result, displayOptions.PlotOptions);
end
end

%% Section 7: Local Functions

function transformed_units = moveSunSlice( ...
        sourcePosition_units, sampleTime_s, ~, missionEndTime_s, ...
        startCenter_units, endCenter_units)
    % Move one sun outline smoothly across the lower part of the scene.
    missionProgress = min(max(sampleTime_s / missionEndTime_s, 0), 1);
    travelFraction  = 10 * missionProgress^3 - 15 * missionProgress^4 + 6 * missionProgress^5;
    center_units    = startCenter_units + travelFraction * (endCenter_units - startCenter_units);
    center_units(2) = center_units(2) + 2 * sin(2 * pi * missionProgress);
    transformed_units = sourcePosition_units + center_units;
end
