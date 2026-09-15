function result = exampleMovingObstacle220(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleMovingObstacle220()
%   result = exampleMovingObstacle220(exampleOverrides)
%**************************************************************************
% PURPOSE
%   - Demonstrate a narrow detour around a translating concave polygon with
%     exactly 220 vertices per snapshot and its complete source history.
%**************************************************************************
% INPUTS
%   - exampleOverrides (scalar struct, optional; default struct())
%       Uniform display controls and public planner option overrides.
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
if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
scenarioDefaults = struct( ...
    'GoalTimeMode', 'fixedArrival', ...
    'WrapX',        false, ...
    'Title',        'Moving concave obstacle: 220 vertices per snapshot');
[options, displayOptions] = resolveExampleOptions(exampleOverrides, scenarioDefaults, [2, 2]);

%% Section 2: Create Obstacles
[obstacles, initialState, goalState, limits] = createMovingObstacle220Scenario();

%% Section 3: Create Planner Inputs
limits.maxJerk_units_s3 = displayOptions.MaxJerk_units_s3;

%% Section 4: Run Planner
result = planner(obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result
validation = obstacleAvoidance.validateTrajectory(result);
if ~result.Success || ~validation.Passed
    warning('exampleMovingObstacle220:ValidationFailed', '%s; %s', ...
        result.Message, validation.Message);
end

%% Section 6: Plot Diagnostics And Motion
if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory(result, displayOptions.PlotOptions);
end
end
