function [result, diagnosis] = exampleNoPath(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleNoPath()
%   result = exampleNoPath(exampleOverrides)
%
% PURPOSE
%   - Demonstrate stable failure and plotted search diagnostics for no path.
%
% INPUTS
%   - exampleOverrides (scalar struct, optional; default struct())
%       Uniform display controls and public planner option overrides.
%
% OUTPUTS
%   - result (scalar struct)
%       Unmodified public planner failure result.
%
% UNITS
%   - Position is coordinate units; time is seconds; derivatives use units/s, units/s^2,
%     and units/s^3.
%

%% Section 1: Resolve Example Controls

% Use normal planner controls. This scenario expects a reported failure.

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
[options, displayOptions] = resolveExampleOptions(exampleOverrides, struct("GoalTimeMode", "earliestArrival", "WrapX", false), [2 2]);

%% Section 2: Create Obstacles

% The protected barrier spans the usable workspace. No route can go around it.
% This is an expected no-path case, not invalid input.

obstacleTime_s        = [0; 20];
obstacleX_units   = [-0.5; 0.5; 0.5; -0.5];
obstacleY_units = [-90; -90; 90; 90];
obstacles             = obstacleAvoidance.obstacles.createObstacle("full-height wall", obstacleTime_s, obstacleX_units, obstacleY_units, 0);

%% Section 3: Create Planner Inputs

% Put the endpoints on opposite sides of the complete barrier. The workspace
% bounds prevent an escape outside the demonstrated region.

initialState = struct();
initialState.time_s       = 0;
initialState.position_units = [-5 0];
goalState = struct();
goalState.time_s       = 12;
goalState.position_units = [5 0];
limits = struct("maxVelocity_units_s", [2 2], ...
    "maxAcceleration_units_s2", [1 1], ...
    "maxJerk_units_s3", displayOptions.MaxJerk_units_s3, "yInterval_units", [-10 10]);

%% Section 4: Run Planner

% The planner must return a failure result. It must not stop the example with an
% error for this expected planning outcome.

[result, diagnosis] = obstacleAvoidance.planTrajectory(obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result

% Check the failure reason, search counts, and diagnostic arrays. These outputs
% help a junior engineer find where and why the search ended.

recognizedReason   = result.TerminationReason == "noValidatedSeed";
diagnosticsPresent = isfield(diagnosis, "Search") && isfield(diagnosis.Search, "ExpandedCount");
exampleValidation  = struct("Passed", ~result.Success && isempty(result.time_s) && ...
    recognizedReason && diagnosticsPresent, ...
    "Message", "Expected failure must retain a recognized reason and " + ...
    "search diagnostics.", ...
    "PlannerReportedSuccess", result.Success, ...
    "RecognizedReason", recognizedReason, "DiagnosticsPresent", diagnosticsPresent);
if ~exampleValidation.Passed
    warning("exampleNoPath:ValidationFailed", "%s", exampleValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion

% Plot explored states, rejected transitions, and the best partial route. No
% final trajectory is available in this case.

if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory(result, displayOptions.PlotOptions, diagnosis);
end

end
