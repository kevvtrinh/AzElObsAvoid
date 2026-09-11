function result = exampleRandomAzimuth(caseIndex, includeStaticObstacle, exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX: result = exampleRandomAzimuth(caseIndex, includeStaticObstacle, exampleOverrides)
% PURPOSE: Plot one reproducible wide-azimuth slew past a moving rectangle.
% INPUTS: caseIndex (default 1), logical includeStaticObstacle (default false),
%   and standard example display/planner overrides (default struct()).
% OUTPUTS: Unmodified public planner result, independently validated on success.
% UNITS: Degrees and seconds; derivative limits use deg/s, deg/s^2, deg/s^3.

%% Section 1: Create Inputs And Resolve Controls
if nargin < 1, caseIndex = 1; end
if nargin < 2, includeStaticObstacle = false; end
if nargin < 3, exampleOverrides = struct(); end
scenario = createRandomAzimuthScenario(caseIndex,includeStaticObstacle);
[options,displayOptions] = resolveExampleOptions(exampleOverrides,scenario.Options,[2,2]);
scenario.Limits.maxJerk_units_s3 = displayOptions.MaxJerk_units_s3;

%% Section 2: Plan, Validate, And Plot The Returned Core Result
result = planner(scenario.Obstacles,scenario.InitialState,scenario.GoalState,scenario.Limits,options);
if result.Success
    validation = obstacleAvoidance.validateTrajectory(result);
    assert(validation.Passed,validation.Message);
end
if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory(result,displayOptions.PlotOptions);
end
end
