function [result, diagnosis] = exampleVietnamBoundarySlew(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX: [result, diagnosis] = exampleVietnamBoundarySlew(exampleOverrides)
% PURPOSE: Solve the supplied Vietnam boundary fixture with one continuous BMTP motion.
% INPUTS: Optional uniform example display/planner overrides; default fixed arrival.
% OUTPUTS: Unmodified public planner result and compatibility diagnosis.
% UNITS: Degrees, seconds, and angular derivatives in degrees/s^order.

%% Section 1: Resolve Display And Physical Inputs
if nargin < 1 || isempty(exampleOverrides), exampleOverrides = struct(); end
[options, displayOptions] = resolveExampleOptions(exampleOverrides, ...
    struct('GoalTimeMode','fixedArrival','WrapX',false, ...
    'Title','Vietnam boundary: 921 slices with 280 vertices'),[2,2]);
[obstacles, initialState, goalState, limits] = createVietnamBoundaryScenario();
limits.maxJerk_units_s3 = displayOptions.MaxJerk_units_s3;

%% Section 2: Optimize And Independently Validate The Complete Motion
[result, diagnosis] = planner(obstacles, initialState, goalState, limits, options);
validation = obstacleAvoidance.validateTrajectory(result);
if ~result.Success || ~validation.Passed
    warning('exampleVietnamBoundarySlew:ValidationFailed','%s; %s', ...
        result.Message,validation.Message);
end

%% Section 3: Plot Only The Returned Core Results
if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory(result, displayOptions.PlotOptions, diagnosis);
end
end
