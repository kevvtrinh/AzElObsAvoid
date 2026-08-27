function result = exampleObstacleFree(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleObstacleFree()
%   result = exampleObstacleFree(exampleOverrides)
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
%       Unmodified public planner result.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s, deg/s^2,
%     and deg/s^3.
%**************************************************************************

%% Section 1: Resolve Example Controls

% Select earliest-arrival mode and resolve shared display controls.

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
[options, displayOptions] = resolveExampleOptions( ...
    exampleOverrides, struct( ...
    "GoalTimeMode", "earliestArrival", "MaximumSeedCount", 1), [2 2]);

%% Section 2: Create Obstacles

% Use the standard empty obstacle array. This case gives a simple baseline for
% motion timing and smoothing without collision constraints.

obstacles = [];

%% Section 3: Create Planner Inputs

% Define rest-to-rest endpoint states and axis motion limits. The shortest path
% is the direct line because no obstacle blocks it.

initialState = struct( "time_s", 0, "position_deg", [0 0], "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
goalState = struct( "time_s", 8, "position_deg", [4 2], "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], "maxAcceleration_deg_s2", [1 1], "maxJerk_deg_s3", displayOptions.MaxJerk_deg_s3);

%% Section 4: Run Planner

% Run the public planner and let it find the minimum feasible arrival time.

result = obstacleAvoidance.planTrajectory( obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result

% Validate timing and kinematics. A failure here usually points to motion
% profiling or endpoint handling rather than obstacle geometry.

exampleValidation = obstacleAvoidance.validateTrajectory(result);
if ~exampleValidation.Passed
    warning("exampleObstacleFree:ValidationFailed", ...
        "%s", exampleValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion

% Use this plot as the simplest reference for more complex example plots.

if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory( ...
        result, displayOptions.PlotOptions);
end

end
