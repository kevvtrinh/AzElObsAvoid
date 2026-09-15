function result = exampleTwoOpposingUVisibilityGraph(options)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleTwoOpposingUVisibilityGraph()
%   result = exampleTwoOpposingUVisibilityGraph(options)
%**************************************************************************
% PURPOSE
%   - Construct two protected opposing U obstacles and run the maintained
%     automatic visibility planner.
%**************************************************************************
% INPUTS
%   - options (scalar struct, optional; default struct())
%       Planner option overrides plus the finite MaxJerk_units_s3 limit.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Unmodified public planner result. Ordinary planning failure returns
%       Success = false; invalid input throws.
%**************************************************************************
% UNITS
%   - Position is coordinate units; time is seconds; derivatives use units/s,
%     units/s^2, and units/s^3.
%**************************************************************************

%% Section 1: Resolve Example Controls

% Resolve the same public options that all maintained examples use.

if nargin < 1 || isempty(options)
    options = struct();
end
scenarioDefaults = struct( ...
    "FigureVisible", "on", ...
    "GoalTimeMode",  "earliestArrival", ...
    "Title",         "Two opposing U-shaped x/y obstacles");
[options, displayOptions] = resolveExampleOptions(options, scenarioDefaults, [2.5 2.5]);

%% Section 2: Create Obstacles

% Two U shapes face in opposite directions. Their concave cavities create
% several visibility choices. No solver initialization changes the physical
% request.

missionEndTime_s      = 180;
safetyMargin_units    = 0.10;
firstUBoundary_units  = [-10, 8; 0, 8; 0, 5; -7, 5; -7, -5; 0, -5; 0, -8; -10, -8];
secondUBoundary_units = [5, 28; 15, 28; 15, 12; 5, 12; 5, 15; 12, 15; 12, 25; 5, 25];
obstacleTime_s        = [0; missionEndTime_s];
firstObstacle  = obstacleAvoidance.obstacles.createObstacle( ...
    "Right-opening U", obstacleTime_s, firstUBoundary_units(:, 1), ...
    firstUBoundary_units(:, 2), safetyMargin_units);
secondObstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "Left-opening U", obstacleTime_s, secondUBoundary_units(:, 1), ...
    secondUBoundary_units(:, 2), safetyMargin_units);
obstacles      = [firstObstacle; secondObstacle];

%% Section 3: Create Planner Inputs

% Put the endpoints beyond the two obstacles. The direct line is blocked by the
% protected shapes, so the visibility search must connect safe boundary views.

initialState = struct();
initialState.time_s         = 0;
initialState.position_units = [-4 0];

goalState = struct("time_s", missionEndTime_s, "position_units", [9 20]);

limits = struct( ...
    "maxVelocity_units_s",      [1 1], ...
    "maxAcceleration_units_s2", [0.75 0.75], ...
    "maxJerk_units_s3",         displayOptions.MaxJerk_units_s3);

%% Section 4: Run Planner

% Run the automatic visibility planner. Do not supply route directions.

result = planner(obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result

% Check that smoothing preserves the collision-free geometric route.

exampleValidation = validateExampleResult(result, "two opposing Us", struct("RequireDirectBlocked", true));
if ~exampleValidation.Passed
    warning("exampleTwoOpposingUVisibilityGraph:ValidationFailed", "%s", exampleValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion

% Plot visibility search data and the selected trajectory when enabled.

if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory(result, displayOptions.PlotOptions);
end

end
