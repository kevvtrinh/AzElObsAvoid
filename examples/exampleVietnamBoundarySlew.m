function result = exampleVietnamBoundarySlew(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleVietnamBoundarySlew()
%   result = exampleVietnamBoundarySlew(exampleOverrides)
%**************************************************************************
% PURPOSE
%   - Exercise the supplied Vietnam boundary fixture without replacing its
%     declared normalized continuous deformation by a convex hull.
%**************************************************************************
% INPUTS
%   - exampleOverrides (scalar struct, optional; default struct())
%       Uniform display controls and public planner option overrides.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Unmodified public planner result. Ordinary planning failure returns
%       Success = false; invalid input throws.
%**************************************************************************
% UNITS
%   - Degrees and seconds; derivative limits use deg/s, deg/s^2, and deg/s^3.
%**************************************************************************

%% Section 1: Resolve Example Controls

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
scenarioDefaults = struct( ...
    'GoalTimeMode', 'fixedArrival', ...
    'WrapX',        false, ...
    'Title',        'Vietnam boundary: 921 slices with 275 normalized vertices');
[options, displayOptions] = resolveExampleOptions(exampleOverrides, ...
    scenarioDefaults, [2, 2]);

%% Section 2: Create Obstacles And Planner Inputs

[obstacles, initialState, goalState, limits] = createVietnamBoundaryScenario();
limits.maxJerk_units_s3 = displayOptions.MaxJerk_units_s3;

%% Section 3: Run Planner

result = planner(obstacles, initialState, goalState, limits, options);

%% Section 4: Validate Result

validation = obstacleAvoidance.validateTrajectory(result);
if ~(result.Success && validation.Passed)
    warning('exampleVietnamBoundarySlew:ValidationFailed', '%s; %s', ...
        result.Message, validation.Message);
end

%% Section 5: Plot Diagnostics And Motion

if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory(result, displayOptions.PlotOptions);
end
end
