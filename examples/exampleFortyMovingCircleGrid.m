function result = exampleFortyMovingCircleGrid(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleFortyMovingCircleGrid()
%   result = exampleFortyMovingCircleGrid(exampleOverrides)
%**************************************************************************
% PURPOSE
%   - Plan the earliest validated diagonal motion through an equal 8-by-5
%     grid.
%   - Show the full visibility planner while every circle moves vertically.
%**************************************************************************
% INPUTS
%   - exampleOverrides (scalar struct, optional; default struct())
%       Planner overrides plus the shared FigureVisible, PlotOutputs,
%       ShowAnimation, ShowKinematicPlot, and MaxJerk_deg_s3 controls.
%**************************************************************************
% OUTPUTS
%   - result (scalar planner-result struct)
%       Validated motion, moving-grid history, scenario inputs, and
%       optional plot handles.
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
        "GoalTimeMode", "earliestArrival", ...
        "SampleTime_s", 0.05, ...
        "AllowAzimuthWrapping", false, ...
        "MaximumSeedCount", 2, ...
        "SeedClusterDistance_deg", 2.0, ...
        "FigureVisible", "on", "Title", "Diagonal motion through 40 moving circles"), [2.5 2.5]);

options.AllowAzimuthWrapping = false;

%% Section 2: Create Obstacles

missionEndTime_s = 200;
obstacleTime_s = [0; 15; 30; 45; 60; missionEndTime_s];
gridSpacing_deg = 8.0;
columnAzimuth_deg = (-3.5:3.5) * gridSpacing_deg;
rowElevation_deg = (-2:2).' * gridSpacing_deg;
verticalTranslation_deg = [0; 1.5; 0; -1.5; 0; 0];
circleRadius_deg = 3.0;
safetyMargin_deg = 0.10;
circleVertexCount = 16;
circleAngle_rad = (0:circleVertexCount - 1).' * (2 * pi / circleVertexCount);
unitCircle = [cos(circleAngle_rad), sin(circleAngle_rad)];

obstacleCount = numel(columnAzimuth_deg) * numel(rowElevation_deg);
obstacleByIndex = cell(obstacleCount, 1);
centerAzimuth_deg = zeros(obstacleCount, 1);
centerElevation_deg = zeros(numel(obstacleTime_s), obstacleCount);
obstacleIndex = 0;

% Walk the grid row by row so every moving circle receives a stable deterministic index.
for rowIndex = 1:numel(rowElevation_deg)

    % Pair every column with the current row and reuse that row's vertical motion history.
    for columnIndex = 1:numel(columnAzimuth_deg)
        obstacleIndex = obstacleIndex + 1;
        centerAzimuth_deg(obstacleIndex) = columnAzimuth_deg(columnIndex);
        centerElevation_deg(:, obstacleIndex) = rowElevation_deg(rowIndex) + verticalTranslation_deg;

        obstacleAzimuthByTime_deg = cell(numel(obstacleTime_s), 1);
        obstacleElevationByTime_deg = cell(numel(obstacleTime_s), 1);

        % Rebuild the circle at every time sample so its translation is explicit to the planner.
        for sampleIndex = 1:numel(obstacleTime_s)
            center_deg = [centerAzimuth_deg(obstacleIndex), centerElevation_deg(sampleIndex, obstacleIndex)];
            circlePosition_deg = center_deg + circleRadius_deg * unitCircle;
            obstacleAzimuthByTime_deg{sampleIndex} = circlePosition_deg(:, 1);
            obstacleElevationByTime_deg{sampleIndex} = circlePosition_deg(:, 2);
        end
        obstacleByIndex{obstacleIndex} = azElObstacles.makeAzElObstacleData( ...
            "Moving grid circle " + obstacleIndex, obstacleTime_s, ...
            obstacleAzimuthByTime_deg, obstacleElevationByTime_deg, safetyMargin_deg);
    end
end
obstacles = azElObstacles.combineAzElObstacles(obstacleByIndex{:});

%% Section 3: Create Planner Inputs

initialState = struct( "time_s", 0, "position_deg", [-36 -24], "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
goalState = struct( ...
    "time_s", missionEndTime_s, "position_deg", [36 24], "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [0.8 0.8], "maxJerk_deg_s3", jerkConfiguration.MaxJerk_deg_s3);

%% Section 4: Run Planner

result = planAzElMotion(obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result

exampleValidation = validateAzElExampleResult( result, "40-circle moving grid");

spacingTolerance_deg = 1e-12;
equalColumnSpacing = all(abs(diff(columnAzimuth_deg) - gridSpacing_deg) <= spacingTolerance_deg);
equalRowSpacing = all(abs(diff(rowElevation_deg) - gridSpacing_deg) <= spacingTolerance_deg);
startAndGoalSpanGridDiagonal = initialState.position_deg(1) < min(columnAzimuth_deg) && ...
    initialState.position_deg(2) < min(rowElevation_deg) && ...
    goalState.position_deg(1) > max(columnAzimuth_deg) && goalState.position_deg(2) > max(rowElevation_deg);
centerTranslation_deg = centerElevation_deg - centerElevation_deg(1, :);
relativeSpacingPreserved = all(abs(centerTranslation_deg - verticalTranslation_deg) <= spacingTolerance_deg, "all");
allObstaclesMoved = all(max(centerElevation_deg, [], 1) - min(centerElevation_deg, [], 1) > 0);

gridValidation = struct( ...
    "Passed", obstacleCount == 40 && equalColumnSpacing && ...
        equalRowSpacing && startAndGoalSpanGridDiagonal && ...
        relativeSpacingPreserved && allObstaclesMoved, ...
    "ObstacleCount", obstacleCount, ...
    "EqualColumnSpacing", equalColumnSpacing, ...
    "EqualRowSpacing", equalRowSpacing, ...
    "StartAndGoalSpanGridDiagonal", startAndGoalSpanGridDiagonal, ...
    "RelativeSpacingPreserved", relativeSpacingPreserved, "AllObstaclesMoved", allObstaclesMoved);

if ~gridValidation.Passed
    exampleValidation.Passed = false;
    exampleValidation.Message = "The 40-circle grid geometry or motion validation failed.";
end

%% Section 6: Plot Diagnostics And Motion

result.PlotHandles = struct();
if jerkConfiguration.PlotOutputs
    result.PlotHandles = azElPlotting.plotMotion( result, jerkConfiguration.PlotOptions);
end

%% Section 7: Return Example Metadata

result.ExampleName = "exampleFortyMovingCircleGrid";
result.ExampleValidation = exampleValidation;
result.GridValidation = gridValidation;
result.ExampleConfiguration = jerkConfiguration;
result.ExampleInputs = struct( ...
    "obstacles", obstacles, "initialState", initialState, "goalState", goalState, "limits", limits, "options", options);
result.obstacleTime_s = obstacleTime_s;
result.centerAzimuth_deg = centerAzimuth_deg;
result.centerElevation_deg = centerElevation_deg;
result.gridSpacing_deg = gridSpacing_deg;
result.circleRadius_deg = circleRadius_deg;
result.safetyMargin_deg = safetyMargin_deg;
result.ExampleMetrics = computeAzElExampleMetrics(result);
end
