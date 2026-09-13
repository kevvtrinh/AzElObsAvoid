function [result, diagnosis] = exampleSpinningUAtStartAndGoal(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleSpinningUAtStartAndGoal()
%   result = exampleSpinningUAtStartAndGoal(exampleOverrides)
%
% PURPOSE
%   - Plan from the cavity of a spinning U-shaped obstacle to the cavity of
%     a second spinning U-shaped obstacle.
%   - Keep both obstacle motions deterministic and entirely input-driven.
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
%   - Position is coordinate units; time is seconds; derivatives use units/s,
%     units/s^2, and units/s^3. Rotation angles are radians.

%% Section 1: Resolve Example Controls

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
[options, displayOptions] = resolveExampleOptions(exampleOverrides, struct( ...
    "GoalTimeMode", "fixedArrival", "FixedArrivalSearch", "timeExpanded", ...
    "TemporalResolution_s", 1.0, "SampleTime_s", 0.05, "WrapX", false, ...
    "Title", "Spinning U at the start and goal"), [8 8]);

%% Section 2: Create The Spinning U Obstacles

% Each U rotates about the endpoint held in its open cavity. Initially the
% start U and goal U both open upward, so they do not face one another. Three
% convex bars represent each U so every between-sample rotation
% retains exact vertex correspondence instead of replacing a concave sweep by
% a hull.
missionEndTime_s   = 24;
obstacleTime_s     = linspace(0, missionEndTime_s, 17).';
spinAngle_rad      = linspace(0, 2 * pi, numel(obstacleTime_s)).';
startAngle_rad     = pi / 2 + spinAngle_rad;
goalAngle_rad      = -pi / 2 - spinAngle_rad;
startCenter_units  = [-8 0];
goalCenter_units   = [8 0];
safetyMargin_units = 0.12;

rightOpeningBars_units = { ...
    [-3.5 -3.0; -1.8 -3.0; -1.8 3.0; -3.5 3.0], ...
    [-1.8  1.6;  2.5  1.6;  2.5 3.0; -1.8 3.0], ...
    [-1.8 -3.0;  2.5 -3.0;  2.5 -1.6; -1.8 -1.6]};
leftOpeningBars_units = cellfun(@(bar_units) ...
    [-bar_units(:, 1), bar_units(:, 2)], rightOpeningBars_units, ...
    "UniformOutput", false);
[startUItems, startHistory] = createSpinningUComponents( ...
    "Start U", rightOpeningBars_units, startCenter_units, startAngle_rad, ...
    obstacleTime_s, safetyMargin_units);
[goalUItems, goalHistory] = createSpinningUComponents( ...
    "Goal U", leftOpeningBars_units, goalCenter_units, goalAngle_rad, ...
    obstacleTime_s, safetyMargin_units);
obstacles = obstacleAvoidance.obstacles.combineObstacles(startUItems{:}, goalUItems{:});

%% Section 3: Create Planner Inputs

initialState = struct("time_s", 0, "position_units", startCenter_units, ...
    "velocity_units_s", [0 0], "acceleration_units_s2", [0 0]);
goalState = struct("time_s", missionEndTime_s, "position_units", goalCenter_units, ...
    "velocity_units_s", [0 0], "acceleration_units_s2", [0 0]);
limits = struct("xInterval_units", [-13 13], "yInterval_units", [-8 8], ...
    "maxVelocity_units_s", [4 4], "maxAcceleration_units_s2", [4 4], ...
    "maxJerk_units_s3", displayOptions.MaxJerk_units_s3);

%% Section 4: Run Planner

[result, diagnosis] = planner(obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result And Scenario Motion

exampleValidation = validateExampleResult(result, ...
    "spinning U obstacles at both endpoints", struct(), diagnosis);
rotationValidation = validateSpinningUs(startHistory, goalHistory, ...
    startCenter_units, goalCenter_units, spinAngle_rad, obstacles, ...
    missionEndTime_s);
if ~rotationValidation.Passed
    exampleValidation.Passed = false;
    exampleValidation.Message = exampleValidation.Message + " " + rotationValidation.Message;
end
if ~exampleValidation.Passed
    warning("exampleSpinningUAtStartAndGoal:ValidationFailed", "%s", ...
        exampleValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion

if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory(result, displayOptions.PlotOptions, diagnosis);
end

end

%% Section 7: Local Functions

function transformed_units = rotateAboutEndpoint(sourcePosition_units, center_units, angle_rad)
    % Rotate the local U boundary and then place its cavity on the endpoint.
    rotation = [cos(angle_rad), -sin(angle_rad); sin(angle_rad), cos(angle_rad)];
    transformed_units = sourcePosition_units * rotation.' + center_units;
end

function [obstacleItems, histories] = createSpinningUComponents(name, bars_units, center_units, angle_rad, time_s, safetyMargin_units)
    % Keep each bar convex so its continuous affine rotation is represented.
    obstacleItems = cell(numel(bars_units), 1);
    histories = cell(numel(bars_units), 1);
    for barIndex = 1:numel(bars_units)
        boundary_units = bars_units{barIndex};
        transform = @(sourcePosition_units, ~, sampleIndex) rotateAboutEndpoint( ...
            sourcePosition_units, center_units, angle_rad(sampleIndex));
        [obstacleItems{barIndex}, histories{barIndex}] = ...
            obstacleAvoidance.obstacles.createMovingObstacle( ...
            name + " bar " + barIndex, time_s, boundary_units(:, 1), ...
            boundary_units(:, 2), transform, safetyMargin_units);
    end
end

function validation = validateSpinningUs(startHistory, goalHistory, startCenter_units, goalCenter_units, spinAngle_rad, obstacles, missionEndTime_s)
    % Confirm full revolutions while both endpoint cavities remain clear.
    angleTravel_rad = sum(abs(diff(spinAngle_rad)));
    radiusTolerance_units = 1e-10;
    radiiPreserved = all(cellfun(@(history) historyPreservesRadius( ...
        history, startCenter_units, radiusTolerance_units), startHistory)) && ...
        all(cellfun(@(history) historyPreservesRadius( ...
        history, goalCenter_units, radiusTolerance_units), goalHistory));
    completedRevolution = abs(angleTravel_rad - 2 * pi) <= 1e-12;
    queryTime_s = linspace(0, missionEndTime_s, 257).';
    occupancyOptions = struct("BoundaryIsOccupied", true);
    startOccupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
        obstacles, repmat(startCenter_units(1), size(queryTime_s)), ...
        repmat(startCenter_units(2), size(queryTime_s)), queryTime_s, occupancyOptions);
    goalOccupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
        obstacles, repmat(goalCenter_units(1), size(queryTime_s)), ...
        repmat(goalCenter_units(2), size(queryTime_s)), queryTime_s, occupancyOptions);
    endpointCavitiesRemainClear = ~any(startOccupied | goalOccupied);
    passed = completedRevolution && radiiPreserved && endpointCavitiesRemainClear;
    if passed
        message = "Both endpoint U obstacles complete one radius-preserving revolution with clear cavities.";
    else
        message = "The endpoint U rotation history is incomplete, changes radius, or closes an endpoint cavity.";
    end
    validation = struct("Passed", passed, "Message", message, ...
        "AngleTravel_rad", angleTravel_rad, "RadiiPreserved", radiiPreserved, ...
        "EndpointCavitiesRemainClear", endpointCavitiesRemainClear);
end

function preserved = historyPreservesRadius(history, center_units, tolerance_units)
    % Compare each component vertex radius before and after the revolution.
    initial_units = [history.xBySlice_units{1}, history.yBySlice_units{1}];
    final_units = [history.xBySlice_units{end}, history.yBySlice_units{end}];
    initialRadius_units = vecnorm(initial_units - center_units, 2, 2);
    finalRadius_units = vecnorm(final_units - center_units, 2, 2);
    preserved = max(abs(finalRadius_units - initialRadius_units)) <= tolerance_units;
end
