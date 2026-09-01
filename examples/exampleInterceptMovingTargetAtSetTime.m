function result = exampleInterceptMovingTargetAtSetTime( interceptTime_s, options)
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
%   - options (scalar struct, optional; default struct())
%       Planner option overrides plus the finite MaxJerk_deg_s3 limit.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Unmodified public moving-target planner result.
%**************************************************************************
% UNITS
%   - Angles are degrees and time is seconds.
%**************************************************************************

%% Section 1: Resolve Example Controls

% Accept a custom intercept time or an options structure. Fixed-arrival mode
% requires the gimbal to reach the target at exactly interceptTime_s.

if nargin < 1 || isempty(interceptTime_s)
    interceptTime_s = 12;
    options = struct();
elseif isstruct(interceptTime_s)
    options = interceptTime_s;
    interceptTime_s = 12;
elseif nargin < 2 || isempty(options)
    options = struct();
end
validateattributes(interceptTime_s, {'numeric'}, {'real', 'finite', 'scalar', 'positive'});
[options, jerkConfiguration] = resolveExampleOptions( ...
    options, struct( ...
    "FigureVisible", "on", "Title", sprintf( "Moving-target intercept at t = %.3f s", interceptTime_s)), [2.5 2.5]);

%% Section 2: Create Obstacles

% Use no obstacles. This isolates target interpolation and specified-time motion.

obstacles = [];

%% Section 3: Create Planner Inputs

% The sampled target follows a curved track. The intercept time must be inside
% this history. The planner does not estimate unknown target motion.

targetTime_s = linspace(0, max(30, interceptTime_s + 5), 7).';
targetMotion = struct( ...
    "time_s", targetTime_s, ...
    "position_deg", [ ...
        8 + 0.18 * targetTime_s - 0.006 * targetTime_s.^2, ...
        -3 + 0.02 * targetTime_s + 0.004 * targetTime_s.^2], "InterpolationMethod", "pchip");
initialState = struct( "time_s", 0, "position_deg", [0 0], "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [0.8 0.8], "maxJerk_deg_s3", jerkConfiguration.MaxJerk_deg_s3);
interceptOptions = struct( ...
    "InterceptMode", "specifiedTime", "SpecifiedInterceptTime_s", interceptTime_s, "PlannerOptions", options);

%% Section 4: Run Planner

% Evaluate the target at the set time and plan one position-only intercept.

result = obstacleAvoidance.planMovingTargetIntercept( ...
    obstacles, initialState, targetMotion, limits, interceptOptions);

%% Section 5: Validate Result

% Confirm that the final gimbal position equals the target position. Also check
% velocity, acceleration, and jerk limits.

exampleValidation = validateExampleResult( result, "specified-time moving-target intercept");
specifiedTimeSatisfied = isempty(result.Inputs.obstacles) && ...
    result.Validation.Passed && abs(result.Intercept.Time_s - interceptTime_s) <= 1e-8;
exampleValidation.SpecifiedTimeSatisfied = specifiedTimeSatisfied;
exampleValidation.Passed = exampleValidation.Passed && specifiedTimeSatisfied;
if ~exampleValidation.Passed
    warning("exampleInterceptMovingTargetAtSetTime:ValidationFailed", ...
        "%s", exampleValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion

% Plot the full target track and mark the requested meeting point.

if jerkConfiguration.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory( ...
        result, jerkConfiguration.PlotOptions);
end

end
