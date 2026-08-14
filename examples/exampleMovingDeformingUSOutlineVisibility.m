function result = exampleMovingDeformingUSOutlineVisibility(options)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleMovingDeformingUSOutlineVisibility()
%   result = exampleMovingDeformingUSOutlineVisibility(options)
%**************************************************************************
% PURPOSE
%   - Plan around a dense, moving, independently deforming U.S. outline over
%     five minutes with collision slices every five seconds while showing
%     only ten evenly distributed time-obstacle explorer slices.
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
    "BoundaryRouteReductionTolerance_deg", 0.25, ...
    "MaximumDisplayedSlicesPerObstacle", 10, ...
    "MaximumRetimedVisibilityRoutes", 12, ...
    "ShowSweptSurfaces", true, ...
    "Verbose", true, ...
    "FigureVisible", "on", ...
    "Title", "Moving/deforming U.S.: slice-local visibility reduction"), ...
    [12 12]);

%% Section 2: Create Obstacles

missionEndTime_s = 5 * 60;
obstacleTimeStep_s = 5;
obstacleTime_s = (0:obstacleTimeStep_s:missionEndTime_s).';
% The private geometry helper is retained because shapefile union and
% 14,000-plus outline vertices would obscure the visible scenario flow.
[obstacle, obstacleHistory] = createContiguousUSObstacle( ...
    obstacleTime_s, 0.10, struct( ...
    "MotionMode", "movingDeforming", ...
    "Verbose", options.Verbose));

%% Section 3: Create Planner Inputs

initialState = struct("time_s", 0, "position_deg", [-102 20]);
goalState = struct( ...
    "time_s", missionEndTime_s, "position_deg", [-102 55]);
limits = struct( ...
    "maxVelocity_deg_s", [8 8], ...
    "maxAcceleration_deg_s2", [3 3], ...
    "maxJerk_deg_s3", jerkConfiguration.MaxJerk_deg_s3);

%% Section 4: Run Planner

result = planAzElMotion( ...
    obstacle, initialState, goalState, limits, options);

%% Section 5: Validate Result

exampleValidation = validateAzElExampleResult( ...
    result, "moving/deforming U.S.", ...
    struct("RequireDirectBlocked", true));

%% Section 6: Plot Diagnostics And Motion

result.PlotHandles = struct();
if jerkConfiguration.PlotOutputs
    result.PlotHandles = plotAzElMotion( ...
        result, jerkConfiguration.PlotOptions);
end

%% Section 7: Return Example Metadata

result.ExampleValidation = exampleValidation;
result.ObstacleHistory = obstacleHistory;
result.ExampleConfiguration = jerkConfiguration;
end
