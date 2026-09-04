function result = examplePlannerStageInspection(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = examplePlannerStageInspection()
%   result = examplePlannerStageInspection(exampleOverrides)
%**************************************************************************
% PURPOSE
%   - Demonstrate inspection of the production planner's returned scene,
%     proposal, visibility, route, seed, candidate, and motion stages.
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

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
[options, displayOptions] = resolveExampleOptions( ...
    exampleOverrides, struct( ...
    "GoalTimeMode", "balancedArrival", ...
    "ShowAnimation", false, "ShowKinematics", false), [2 2]);

%% Section 2: Create Obstacles

% A static protected rectangle blocks the direct line, ensuring the normal
% production run creates proposal, visibility, route, and seed stages.

obstacleTime_s = [0; 20];
obstacleAzimuth_deg = [-1; 1; 1; -1];
obstacleElevation_deg = [-2; -2; 2; 2];
safetyMargin_deg = 0.2;
obstacles = obstacleAvoidance.obstacles.createObstacle( ...
    "rectangle", obstacleTime_s, obstacleAzimuth_deg, ...
    obstacleElevation_deg, safetyMargin_deg);

%% Section 3: Create Planner Inputs

initialState = struct("time_s", 0, "position_deg", [-5 0]);
goalState = struct("time_s", 12, "position_deg", [5 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1], ...
    "maxJerk_deg_s3", displayOptions.MaxJerk_deg_s3);

%% Section 4: Run Planner

% The public planner remains the sole orchestration entry point. Its returned
% stage records below are observations of that run, not a second planner.

result = obstacleAvoidance.planTrajectory( ...
    obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result

validation = obstacleAvoidance.validateTrajectory(result);
if ~validation.Passed
    warning("examplePlannerStageInspection:ValidationFailed", ...
        "%s", validation.Message);
end

%% Section 6: Plot Diagnostics And Motion

% Read each production stage in the same order as planTrajectory. Plotting
% consumes those records and never reconstructs nodes, routes, or candidates.

if displayOptions.PlotOutputs
    stages = result.SearchDiagnostics.StageOutputs;
    stageOptions = struct( ...
        "FigureVisible", displayOptions.FigureVisible);
    obstacleAvoidance.plotting.plotPreparedScene( ...
        stages.Scene, stageOptions);
    if ~isempty(fieldnames(stages.Proposal))
        obstacleAvoidance.plotting.plotProposalGeometry( ...
            stages.Proposal, stageOptions);
        obstacleAvoidance.plotting.plotVisibilityGraph( ...
            stages.VisibilityGraph, stages.Proposal, stageOptions);
        obstacleAvoidance.plotting.plotRouteSet( ...
            stages.RouteSet, stages.Scene, stageOptions);
        obstacleAvoidance.plotting.plotSeeds( ...
            stages.SeedSet, stages.Scene, stages.VisibilityGraph, ...
            stageOptions);
    end
    obstacleAvoidance.plotting.plotCandidateSet( ...
        stages.CandidateSet, stages.Scene, stageOptions);
    obstacleAvoidance.plotting.plotTrajectory( ...
        result, displayOptions.PlotOptions);
end
end
