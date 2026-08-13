function result = exampleUSOutlineExtremeVisibility(options)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleUSOutlineExtremeVisibility()
%   result = exampleUSOutlineExtremeVisibility(options)
%**************************************************************************
% PURPOSE
%   - Plan Mexico-to-Canada around a dense static U.S. outline using bounded
%     extreme visibility candidates and full protected collision geometry.
%**************************************************************************
% INPUTS
%   - options (scalar struct, optional; default struct())
%       Planner/display overrides plus EnableJerkConstraint and
%       MaxJerk_deg_s3.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Stable planner result plus independent ExampleValidation,
%       ObstacleHistory, and ExampleConfiguration metadata.
%**************************************************************************
% UNITS
%   - Position is degrees, time is seconds, velocity is degrees per second,
%     acceleration is degrees per second squared, and jerk is degrees per
%     second cubed.
%**************************************************************************

%% Section 1: Resolve Example Controls
if nargin < 1 || isempty(options)
    options = struct();
end
[options, jerkConfiguration] = resolveAzElExampleOptions( ...
    options, struct( ...
    "MotionType", "velocityCarrying", ...
    "GoalTimeMode", "earliestArrival", ...
    "TurnRadius_deg", 0.50, ...
    "PolygonCandidateMode", "extreme", ...
    "ExtremeDirectionCount", 16, ...
    "MaximumTangenciesPerReference", 2, ...
    "VisibilitySampleStep_deg", 0.25, ...
    "MaximumDisplayedSlicesPerObstacle", 1, ...
    "ShowSweptSurfaces", false, ...
    "Verbose", true, ...
    "FigureVisible", "on", ...
    "Title", "Mexico-to-Canada route: reduced U.S. visibility nodes"), ...
    [12 12]);

%% Section 2: Create Obstacles
missionEndTime_s = 120;
% The private geometry helper is retained because shapefile union and
% 14,000-plus outline vertices would obscure the visible scenario flow.
[obstacle, obstacleHistory] = createContiguousUSObstacle( ...
    [0; missionEndTime_s], 0.15, struct( ...
    "MotionMode", "static", "UseParallel", false, ...
    "Verbose", options.Verbose));

%% Section 3: Create Planner Inputs
initialState = struct("time_s", 0, "position_deg", [-102 23]);
goalState = struct( ...
    "time_s", missionEndTime_s, "position_deg", [-102 52]);
limits = struct( ...
    "maxVelocity_deg_s", [8 8], ...
    "maxAcceleration_deg_s2", [3 3], ...
    "maxJerk_deg_s3", jerkConfiguration.MaxJerk_deg_s3);

%% Section 4: Run Planner
result = planAzElMotion( ...
    obstacle, initialState, goalState, limits, options);

%% Section 5: Validate Result
exampleValidation = validateAzElExampleResult( ...
    result, "static U.S. outline", ...
    struct("RequireDirectBlocked", true));

%% Section 6: Plot Diagnostics And Motion
% planAzElMotion already created the requested workspace, animation, and
% kinematic figures from the returned result without scenario knowledge.

%% Section 7: Return Example Metadata
result.ExampleValidation = exampleValidation;
result.ObstacleHistory = obstacleHistory;
result.ExampleConfiguration = jerkConfiguration;
end
