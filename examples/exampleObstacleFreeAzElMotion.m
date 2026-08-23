function result = exampleObstacleFreeAzElMotion(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleObstacleFreeAzElMotion()
%   result = exampleObstacleFreeAzElMotion(exampleOverrides)
%**************************************************************************
% PURPOSE
%   - Demonstrate earliest-arrival motion without obstacle constraints.
%**************************************************************************
% INPUTS
%   - exampleOverrides (scalar struct, optional; default struct())
%       Uniform display controls and public planner option overrides.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Planner result, independent validation, plots, and example metrics.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s, deg/s^2,
%     and deg/s^3.
%**************************************************************************

%% Section 1: Resolve Example Controls

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
[options, displayOptions] = resolveAzElExampleOptions( ...
    exampleOverrides, struct( ...
    "GoalTimeMode", "earliestArrival", ...
    "DirectSeedOnly", true, "MaximumSeedCount", 1), [2 2]);

%% Section 2: Create Obstacles

obstacles = [];

%% Section 3: Create Planner Inputs

initialState = struct( ...
    "time_s", 0, "position_deg", [0 0], ...
    "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
goalState = struct( ...
    "time_s", 8, "position_deg", [4 2], ...
    "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1], ...
    "maxJerk_deg_s3", displayOptions.MaxJerk_deg_s3);

%% Section 4: Run Planner

result = planAzElMotion( ...
    obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result

result.ExampleValidation = validateAzElTrajectory(result);

%% Section 6: Plot Diagnostics And Motion

result.PlotHandles = struct();
if displayOptions.PlotOutputs
    result.PlotHandles = plotAzElMotion( ...
        result, displayOptions.PlotOptions);
end

%% Section 7: Return Example Metadata

result.ExampleName = "exampleObstacleFreeAzElMotion";
result.ExampleMetrics = computeAzElExampleMetrics(result);
result.ExampleControls = displayOptions;
end
