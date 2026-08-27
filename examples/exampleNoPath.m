function result = exampleNoPath(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleNoPath()
%   result = exampleNoPath(exampleOverrides)
%**************************************************************************
% PURPOSE
%   - Demonstrate stable failure and plotted search diagnostics for no path.
%**************************************************************************
% INPUTS
%   - exampleOverrides (scalar struct, optional; default struct())
%       Uniform display controls and public planner option overrides.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Unmodified public planner failure result.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s, deg/s^2,
%     and deg/s^3.
%**************************************************************************

%% Section 1: Resolve Example Controls

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
[options, displayOptions] = resolveExampleOptions( ...
    exampleOverrides, struct( ...
    "GoalTimeMode", "earliestArrival", "MaximumSeedCount", 3, "AllowAzimuthWrapping", false), [2 2]);

%% Section 2: Create Obstacles

obstacleTime_s = [0; 20];
obstacleAzimuth_deg = [-0.5; 0.5; 0.5; -0.5];
obstacleElevation_deg = [-90; -90; 90; 90];
obstacles = obstacleAvoidance.obstacles.createObstacle( "full-height wall", obstacleTime_s, obstacleAzimuth_deg, obstacleElevation_deg, 0);

%% Section 3: Create Planner Inputs

initialState = struct("time_s", 0, "position_deg", [-5 0]);
goalState = struct("time_s", 12, "position_deg", [5 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1], ...
    "maxJerk_deg_s3", displayOptions.MaxJerk_deg_s3, "elevationInterval_deg", [-10 10]);

%% Section 4: Run Planner

result = obstacleAvoidance.planTrajectory( obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result

recognizedReason = result.TerminationReason == "noValidatedSeed";
diagnosticsPresent = isfield(result.SearchDiagnostics, "Grid") && ...
    isfield(result.SearchDiagnostics.Grid, "ExpandedCount");
exampleValidation = struct( ...
    "Passed", ~result.Success && isempty(result.time_s) && ...
    recognizedReason && diagnosticsPresent, ...
    "Message", "Expected failure must retain a recognized reason and " + ...
    "search diagnostics.", ...
    "PlannerReportedSuccess", result.Success, ...
    "RecognizedReason", recognizedReason, "DiagnosticsPresent", diagnosticsPresent);
if ~exampleValidation.Passed
    warning("exampleNoPath:ValidationFailed", ...
        "%s", exampleValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion

if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory( ...
        result, displayOptions.PlotOptions);
end

end
