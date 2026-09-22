function result = exampleRandomAzimuth(caseIndex, includeStaticObstacle, exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleRandomAzimuth()
%   result = exampleRandomAzimuth(caseIndex)
%   result = exampleRandomAzimuth(caseIndex, includeStaticObstacle)
%   result = exampleRandomAzimuth(caseIndex, includeStaticObstacle, exampleOverrides)
%**************************************************************************
% PURPOSE
%   - Plot one reproducible wide-azimuth slew past a moving rectangle.
%**************************************************************************
% INPUTS
%   - caseIndex (positive integer scalar, optional; default 1)
%       Selects the deterministic random scenario to reproduce.
%   - includeStaticObstacle (logical scalar, optional; default false)
%       Adds the second static obstacle to the scenario.
%   - exampleOverrides (scalar struct, optional; default struct())
%       Uniform display controls and public planner option overrides.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Unmodified public planner result, independently validated on success.
%       Ordinary planning failure returns Success = false; invalid input throws.
%**************************************************************************
% UNITS
%   - Degrees and seconds; derivative limits use deg/s, deg/s^2, and deg/s^3.
%**************************************************************************

%% Section 1: Resolve Example Controls

if nargin < 1
    caseIndex = 1;
end
if nargin < 2
    includeStaticObstacle = false;
end
if nargin < 3
    exampleOverrides = struct();
end

%% Section 2: Create Obstacles

% The scenario helper creates the complete deterministic physical request,
% including its obstacle histories.
scenario = createRandomAzimuthScenario(caseIndex, includeStaticObstacle);

%% Section 3: Create Planner Inputs

% Resolve display controls against the scenario's own planner options.
[options, displayOptions] = resolveExampleOptions(exampleOverrides, scenario.Options, [2, 2]);
scenario.Limits.maxJerk_units_s3 = displayOptions.MaxJerk_units_s3;

%% Section 4: Run Planner

result = planner(scenario.Obstacles, scenario.InitialState, scenario.GoalState, ...
    scenario.Limits, options);

%% Section 5: Validate Result

% Recheck every returned motion with the public independent validator.

if result.Success
    validation = obstacleAvoidance.validateTrajectory(result);
    assert(validation.Passed, validation.Message);
end

%% Section 6: Plot Diagnostics And Motion

if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory(result, displayOptions.PlotOptions);
end

end
