function [result, diagnosis] = exampleObstacleFree(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleObstacleFree()
%   result = exampleObstacleFree(exampleOverrides)
%
% PURPOSE
%   - Demonstrate earliest-arrival motion without obstacle constraints.
%
% INPUTS
%   - exampleOverrides (scalar struct, optional; default struct())
%       Uniform display controls and public planner option overrides.
%
% OUTPUTS
%   - result (scalar struct)
%       Unmodified public planner result.
%   - diagnosis (optional second output): search attempts and solver details.
%
% UNITS
%   - Position is coordinate units; time is seconds; derivatives use units/s, units/s^2,
%     and units/s^3.
%

%% Section 1: Resolve Example Controls

% Select earliest-arrival mode and resolve shared display controls.

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
[options, displayOptions] = resolveExampleOptions(exampleOverrides, struct("GoalTimeMode", "earliestArrival"), [2 2]);

%% Section 2: Create Obstacles

% Use the standard empty obstacle array. This case gives a simple baseline for
% motion timing and smoothing without collision constraints.

obstacles = [];

%% Section 3: Create Planner Inputs

% Define rest-to-rest endpoint states and axis motion limits. The shortest path
% is the direct line because no obstacle blocks it.

initialState = struct();
initialState.time_s              = 0;
initialState.position_units        = [0 0];
initialState.velocity_units_s      = [0 0];
initialState.acceleration_units_s2 = [0 0];
goalState = struct();
goalState.time_s              = 8;
goalState.position_units        = [4 2];
goalState.velocity_units_s      = [0 0];
goalState.acceleration_units_s2 = [0 0];
limits = struct("maxVelocity_units_s", [2 2], "maxAcceleration_units_s2", [1 1], "maxJerk_units_s3", displayOptions.MaxJerk_units_s3);

%% Section 4: Run Planner

% Run the public planner and let it find the minimum feasible arrival time.

[result, diagnosis] = planner(obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result

% Validate timing and kinematics. A failure here usually points to motion
% profiling or endpoint handling rather than obstacle geometry.

exampleValidation = obstacleAvoidance.validateTrajectory(result);
if ~exampleValidation.Passed
    warning("exampleObstacleFree:ValidationFailed", "%s", exampleValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion

% Use this plot as the simplest reference for more complex example plots.

if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory(result, displayOptions.PlotOptions, diagnosis);
end

end
