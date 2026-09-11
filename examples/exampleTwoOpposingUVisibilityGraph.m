function [result, diagnosis] = exampleTwoOpposingUVisibilityGraph(options)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleTwoOpposingUVisibilityGraph()
%   result = exampleTwoOpposingUVisibilityGraph(options)
%
% PURPOSE
%   - Construct two protected opposing U obstacles and run the maintained
%     automatic visibility planner.
%
% INPUTS
%   - options (scalar struct, optional; default struct())
%       Planner option overrides plus the finite MaxJerk_units_s3 limit.
%
% OUTPUTS
%   - result (scalar struct)
%       Unmodified public planner result.
%   - diagnosis (optional second output): search attempts and solver details.
%
% UNITS
%   - Positions use coordinate units and time uses seconds.
%

%% Section 1: Resolve Example Controls

% Resolve the same public options that all maintained examples use.

if nargin < 1 || isempty(options)
    options = struct();
end
[options, jerkConfiguration] = resolveExampleOptions(options, struct("FigureVisible", "on", "GoalTimeMode", "earliestArrival", "Title", "Two opposing U-shaped x/y obstacles"), [2.5 2.5]);

%% Section 2: Create Obstacles

% Two U shapes face in opposite directions. Their concave cavities create
% several visibility choices. No solver initialization changes the physical
% request.
missionEndTime_s    = 180;
safetyMargin_units    = 0.10;
firstUBoundary_units  = [ -10, 8; 0, 8; 0, 5; -7, 5; -7,-5; 0,-5; 0,-8; -10,-8];
secondUBoundary_units = [ 5,28; 15,28; 15,12; 5,12; 5,15; 12,15; 12,25; 5,25];
time_s              = [0; missionEndTime_s];
obstacles           = [ ...
    obstacleAvoidance.obstacles.createObstacle("Right-opening U", time_s, firstUBoundary_units(:, 1), firstUBoundary_units(:, 2), safetyMargin_units); obstacleAvoidance.obstacles.createObstacle("Left-opening U", time_s, secondUBoundary_units(:, 1), secondUBoundary_units(:, 2), safetyMargin_units)];

%% Section 3: Create Planner Inputs

% Put the endpoints beyond the two obstacles. The direct line is blocked by the
% protected shapes, so the visibility search must connect safe boundary views.

initialState = struct();
initialState.time_s       = 0;
initialState.position_units = [-4 0];
goalState = struct("time_s", missionEndTime_s, "position_units", [9 20]);
limits    = struct("maxVelocity_units_s", [1 1], ...
    "maxAcceleration_units_s2", [0.75 0.75], "maxJerk_units_s3", jerkConfiguration.MaxJerk_units_s3);

%% Section 4: Run Planner

% Run the automatic visibility planner. Do not supply route directions.

[result, diagnosis] = planner(obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result

% Check that smoothing preserves the collision-free geometric route.

exampleValidation = validateExampleResult(result, "two opposing Us", struct("RequireDirectBlocked", true), diagnosis);
if ~exampleValidation.Passed
    warning("exampleTwoOpposingUVisibilityGraph:ValidationFailed", "%s", exampleValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion

% Plot visibility search data and the selected trajectory when enabled.

if jerkConfiguration.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory(result, jerkConfiguration.PlotOptions, diagnosis);
end

end
