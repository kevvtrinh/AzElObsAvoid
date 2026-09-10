function [result, diagnosis] = exampleMovingObstacle220(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX: [result, diagnosis] = exampleMovingObstacle220(exampleOverrides)
% PURPOSE: Demonstrate a narrow detour around a translating concave polygon
%   with exactly 220 vertices per snapshot and its complete source history.
% INPUTS: Optional scalar struct of uniform example display/planner overrides.
% OUTPUTS: Unmodified public planner result and compatibility diagnosis.
% UNITS: Planar coordinate units, seconds, and physical derivatives.

%% Section 1: Resolve Example Controls
if nargin < 1 || isempty(exampleOverrides), exampleOverrides = struct(); end
[options, displayOptions] = resolveExampleOptions(exampleOverrides, ...
    struct('GoalTimeMode','fixedArrival','WrapX',false, ...
    'Title','Moving concave obstacle: 220 vertices per snapshot'),[2,2]);

%% Section 2: Create The Benchmark Inputs
[obstacles, initialState, goalState, limits] = createMovingObstacle220Scenario();
limits.maxJerk_units_s3 = displayOptions.MaxJerk_units_s3;

%% Section 3: Plan And Independently Validate
[result, diagnosis] = planner(obstacles, initialState, goalState, limits, options);
validation = obstacleAvoidance.validateTrajectory(result);
if ~result.Success || ~validation.Passed
    warning('exampleMovingObstacle220:ValidationFailed','%s; %s', ...
        result.Message,validation.Message);
end

%% Section 4: Plot The Returned Motion
if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory(result, displayOptions.PlotOptions, diagnosis);
end
end
