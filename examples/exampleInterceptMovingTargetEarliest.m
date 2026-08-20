function result = exampleInterceptMovingTargetEarliest(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleInterceptMovingTargetEarliest()
%   result = exampleInterceptMovingTargetEarliest(exampleOverrides)
%**************************************************************************
% PURPOSE
%   - Demonstrate earliest interception through the single HS3 planner path.
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
[plannerOptions, displayOptions] = resolveAzElExampleOptions( ...
    exampleOverrides, struct( ...
    "DirectSeedOnly", true, "MaximumSeedCount", 1, ...
    "CollocationSegmentCount", 7), [2 2]);

%% Section 2: Create Obstacles

obstacles = [];

%% Section 3: Create Planner Inputs

initialState = struct( ...
    "time_s", 0, "position_deg", [0 0], ...
    "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
targetTime_s = (0:4:20).';
targetPosition_deg = [ ...
    6 + 0.2 * targetTime_s, ...
    1 + 0.02 * targetTime_s];
targetMotion = struct( ...
    "time_s", targetTime_s, ...
    "position_deg", targetPosition_deg, ...
    "InterpolationMethod", "linear");
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1], ...
    "maxJerk_deg_s3", displayOptions.MaxJerk_deg_s3);
interceptOptions = struct( ...
    "InterceptMode", "earliest", ...
    "MaximumSearchDuration_s", 20, ...
    "MatchTargetVelocity", false, ...
    "MatchTargetAcceleration", false, ...
    "PlannerOptions", plannerOptions);

%% Section 4: Run Planner

result = planAzElMovingTargetIntercept( ...
    obstacles, initialState, targetMotion, limits, interceptOptions);

%% Section 5: Validate Result

result.ExampleValidation = validateAzElTrajectory(result);

%% Section 6: Plot Diagnostics And Motion

result.PlotHandles = struct();
if displayOptions.PlotOutputs
    result.PlotHandles = plotAzElMotion( ...
        result, displayOptions.PlotOptions);
end

%% Section 7: Return Example Metadata

result.ExampleName = "exampleInterceptMovingTargetEarliest";
result.ExampleMetrics = computeAzElExampleMetrics(result);
result.ExampleControls = displayOptions;
result.ExampleInputs = struct( ...
    "targetTime_s", targetTime_s, ...
    "targetPosition_deg", targetPosition_deg, ...
    "interceptOptions", interceptOptions);
end
