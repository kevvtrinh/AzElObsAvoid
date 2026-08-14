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
%   - options (scalar struct, optional)
%       Planner option overrides plus EnableJerkConstraint and
%       MaxJerk_deg_s3 example controls.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Validated planner result and scenario geometry.
%**************************************************************************
% UNITS
%   - Angles are degrees and time is seconds.
%**************************************************************************

%% Section 1: Resolve Example Controls
if nargin < 1 || isempty(options)
    options = struct();
end
[options, jerkConfiguration] = resolveAzElExampleOptions( ...
    options, struct( ...
    "MotionType", "velocityCarrying", ...
    "Verbose", true, ...
    "FigureVisible", "on", ...
    "Title", "Two opposing U-shaped az/el obstacles"), [2.5 2.5]);

%% Section 2: Create Obstacles
% Keep the physical request identical when the example-only jerk switch is
% toggled so paired feasibility and timing comparisons remain meaningful.
missionEndTime_s = 180;
safetyMargin_deg = 0.10;
firstUBoundary_deg = [ ...
    -10, 8; 0, 8; 0, 5; -7, 5; ...
     -7,-5; 0,-5; 0,-8; -10,-8];
secondUBoundary_deg = [ ...
      5,28; 15,28; 15,12; 5,12; ...
      5,15; 12,15; 12,25; 5,25];
time_s = [0; missionEndTime_s];
obstacles = [ ...
    makeAzElObstacleData("Right-opening U", time_s, ...
        firstUBoundary_deg(:, 1), firstUBoundary_deg(:, 2), ...
        safetyMargin_deg); ...
    makeAzElObstacleData("Left-opening U", time_s, ...
        secondUBoundary_deg(:, 1), secondUBoundary_deg(:, 2), ...
        safetyMargin_deg)];

%% Section 3: Create Planner Inputs
initialState = struct("time_s", 0, "position_deg", [-4 0]);
goalState = struct( ...
    "time_s", missionEndTime_s, "position_deg", [9 20]);
limits = struct( ...
    "maxVelocity_deg_s", [1 1], ...
    "maxAcceleration_deg_s2", [0.75 0.75], ...
    "maxJerk_deg_s3", jerkConfiguration.MaxJerk_deg_s3);

%% Section 4: Run Planner
result = planAzElMotion( ...
    obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result
exampleValidation = validateAzElExampleResult( ...
    result, "two opposing Us", struct("RequireDirectBlocked", true));

%% Section 6: Plot Diagnostics And Motion
result.PlotHandles = struct();
if jerkConfiguration.PlotOutputs
    result.PlotHandles = plotAzElMotion( ...
        result, jerkConfiguration.PlotOptions);
end

%% Section 7: Return Example Metadata
result.ExampleValidation = exampleValidation;
result.firstUBoundary_deg = firstUBoundary_deg;
result.secondUBoundary_deg = secondUBoundary_deg;
result.ExampleConfiguration = jerkConfiguration;
end
