function result = exampleInterceptMovingTargetAtSetTime( ...
        interceptTime_s, options)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleInterceptMovingTargetAtSetTime()
%   result = exampleInterceptMovingTargetAtSetTime(interceptTime_s)
%   result = exampleInterceptMovingTargetAtSetTime(options)
%   result = exampleInterceptMovingTargetAtSetTime( ...
%       interceptTime_s, options)
%**************************************************************************
% PURPOSE
%   - Intercept a curved target track supplied as time-indexed Az/El points
%     at a specified time in an obstacle-free environment.
%   - Demonstrate a position-only intercept for a general target tangent.
%**************************************************************************
% INPUTS
%   - interceptTime_s (positive scalar, optional; default 12)
%   - options (scalar struct, optional)
%       Planner option overrides plus the finite MaxJerk_deg_s3 limit.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Validated fixed-time intercept, target track, plots, and kinematic
%       diagnostics.
%**************************************************************************
% UNITS
%   - Angles are degrees and time is seconds.
%**************************************************************************

%% Section 1: Resolve Example Controls

if nargin < 1 || isempty(interceptTime_s)
    interceptTime_s = 12;
    options = struct();
elseif isstruct(interceptTime_s)
    options = interceptTime_s;
    interceptTime_s = 12;
elseif nargin < 2 || isempty(options)
    options = struct();
end
validateattributes(interceptTime_s, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'positive'});
[options, jerkConfiguration] = resolveAzElExampleOptions( ...
    options, struct( ...
    "Verbose", true, ...
    "FigureVisible", "on", ...
    "Title", sprintf( ...
    "Moving-target intercept at t = %.3f s", interceptTime_s)), ...
    [2.5 2.5]);

%% Section 2: Create Obstacles

obstacles = [];

%% Section 3: Create Planner Inputs

targetTime_s = linspace(0, max(30, interceptTime_s + 5), 7).';
targetMotion = struct( ...
    "time_s", targetTime_s, ...
    "position_deg", [ ...
        8 + 0.18 * targetTime_s - 0.006 * targetTime_s.^2, ...
        -3 + 0.02 * targetTime_s + 0.004 * targetTime_s.^2], ...
    "InterpolationMethod", "pchip");
initialState = struct( ...
    "time_s", 0, ...
    "position_deg", [0 0], ...
    "velocity_deg_s", [0 0], ...
    "acceleration_deg_s2", [0 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [0.8 0.8], ...
    "maxJerk_deg_s3", jerkConfiguration.MaxJerk_deg_s3);
interceptOptions = struct( ...
    "InterceptMode", "specifiedTime", ...
    "SpecifiedInterceptTime_s", interceptTime_s, ...
    "PlannerOptions", options);

%% Section 4: Run Planner

result = planAzElMovingTargetIntercept( ...
    initialState, targetMotion, limits, interceptOptions);

%% Section 5: Validate Result

exampleValidation = validateAzElExampleResult( ...
    result, "specified-time moving-target intercept");
specifiedTimeSatisfied = isempty(result.Inputs.obstacles) && ...
    result.Validation.Passed && ...
    abs(result.Intercept.Time_s - interceptTime_s) <= 1e-8;
exampleValidation.SpecifiedTimeSatisfied = specifiedTimeSatisfied;
exampleValidation.Passed = exampleValidation.Passed && ...
    specifiedTimeSatisfied;

%% Section 6: Plot Diagnostics And Motion

result.PlotHandles = struct();
if jerkConfiguration.PlotOutputs
    result.PlotHandles = plotAzElMotion( ...
        result, jerkConfiguration.PlotOptions);
end

%% Section 7: Return Example Metadata

result.ExampleValidation = exampleValidation;
result.obstacles = obstacles;
result.ExampleConfiguration = jerkConfiguration;
result.ExampleMetrics = computeAzElExampleMetrics(result);
end
