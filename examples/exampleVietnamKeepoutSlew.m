function [result, diagnosis] = exampleVietnamKeepoutSlew(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleVietnamKeepoutSlew()
%   result = exampleVietnamKeepoutSlew(exampleOverrides)
%
% PURPOSE
%   - Plan a fixed-arrival slew through eight time-varying protected regions.
%   - Exercise the maintained timed-visibility and timed-cell BMTP path.
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
%   - Position is coordinate units; time is seconds; derivatives use units/s,
%     units/s^2, and units/s^3.
%

%% Section 1: Resolve Example Controls

% Load the reviewed physical request, then allow only the same public planner
% and display overrides accepted by every maintained example.

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
exampleRoot = fileparts(mfilename("fullpath"));
source = load(fullfile(exampleRoot, "data", "vietnamKeepoutSlewInput.mat"));
scenarioDefaults = source.options;
scenarioDefaults.Title = "Vietnam keep-out slew";
[options, displayOptions] = resolveExampleOptions(exampleOverrides, scenarioDefaults, source.limits.maxJerk_units_s3);

%% Section 2: Create Obstacles

% The reviewed input contains eight protected obstacle histories sampled from
% 0 to 30 seconds. The planner preserves their original and protected geometry.

obstacles = source.protectedObstacles;

%% Section 3: Create Planner Inputs

% Preserve the supplied physical endpoints and per-axis derivative limits.
% MaxJerk_units_s3 remains a uniform example override for comparison runs.

initialState = source.initialState;
goalState    = source.goalState;
limits       = source.limits;
limits.maxJerk_units_s3 = displayOptions.MaxJerk_units_s3;

%% Section 4: Run Planner

% Use the public entry point so the example exercises normalization, timed
% search, motion construction, independent validation, and diagnostics.

[result, diagnosis] = planner(obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result

% Recheck the complete returned motion independently of planner acceptance.

exampleValidation = obstacleAvoidance.validateTrajectory(result);
if ~exampleValidation.Passed
    warning("exampleVietnamKeepoutSlew:ValidationFailed", "%s", exampleValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion

% Show the complete moving histories, selected route, search, and kinematics.

if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory(result, displayOptions.PlotOptions, diagnosis);
end

end
