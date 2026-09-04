function result = exampleInterceptMovingTargetEarliest(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleInterceptMovingTargetEarliest()
%   result = exampleInterceptMovingTargetEarliest(exampleOverrides)
%**************************************************************************
% PURPOSE
%   - Demonstrate earliest interception through the maintained planner path.
%**************************************************************************
% INPUTS
%   - exampleOverrides (scalar struct, optional; default struct())
%       Uniform display controls and public planner option overrides.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Unmodified public moving-target planner result.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s, deg/s^2,
%     and deg/s^3.
%**************************************************************************

%% Section 1: Resolve Example Controls

% Select earliest-arrival mode. The planner can choose the first feasible
% intercept time instead of using the end of the target history.

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
[plannerOptions, displayOptions] = resolveExampleOptions( ...
    exampleOverrides, struct(), [2 2]);

%% Section 2: Create Obstacles

% Use an empty obstacle array. This isolates moving-target timing and motion
% limits from obstacle-search behavior.

obstacles = [];

%% Section 3: Create Planner Inputs

% Supply target positions at increasing times. Interpolation defines the target
% position between samples. The horizon is the latest allowed intercept time.

initialState = struct( "time_s", 0, "position_deg", [0 0], "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
targetTime_s = (0:4:20).';
targetPosition_deg = [ 6 + 0.2 * targetTime_s, 1 + 0.02 * targetTime_s];
targetMotion = struct( "time_s", targetTime_s, "position_deg", targetPosition_deg, "InterpolationMethod", "linear");
limits = struct( ...
    "maxVelocity_deg_s", [2 2], "maxAcceleration_deg_s2", [1 1], "maxJerk_deg_s3", displayOptions.MaxJerk_deg_s3);
interceptOptions = struct( ...
    "InterceptMode", "earliest", ...
    "MaximumSearchDuration_s", 20, ...
    "MatchTargetVelocity", false, "MatchTargetAcceleration", false, "PlannerOptions", plannerOptions);

%% Section 4: Run Planner

% Run the moving-target planner. It searches for the earliest valid meeting.

result = obstacleAvoidance.planMovingTargetIntercept( obstacles, initialState, targetMotion, limits, interceptOptions);

%% Section 5: Validate Result

% Confirm that the gimbal and target meet at the reported intercept time. Also
% check the returned motion against all physical limits.

exampleValidation = obstacleAvoidance.validateTrajectory(result);
if ~exampleValidation.Passed
    warning("exampleInterceptMovingTargetEarliest:ValidationFailed", ...
        "%s", exampleValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion

% Plot the target history and the returned intercept motion when enabled.

if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory( ...
        result, displayOptions.PlotOptions);
end

end
