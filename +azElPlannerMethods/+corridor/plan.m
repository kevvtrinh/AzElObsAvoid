function result = plan(obstacles, initialState, goalState, limits, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = planAzElMotion()
%   result = planAzElMotion( ...
%       obstacles, initialState, goalState, limits)
%   result = planAzElMotion( ...
%       obstacles, initialState, goalState, limits, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Orchestrate the complete planner without owning search, motion, or
%     collision algorithms. The visible flow is normalize -> reject invalid
%     endpoints -> generate topology seeds -> solve/validate/rank candidates.
%**************************************************************************
% INPUTS
%   - obstacles (canonical protected obstacle array, nested cells, or [])
%       Construct safety margins with makeAzElObstacleData exactly once.
%       Azimuth wrapping requires an empty obstacle array.
%   - initialState (scalar struct)
%       time_s and 1-by-2 position_deg are required. Velocity and
%       acceleration default to [0 0].
%   - goalState (scalar struct)
%       time_s is the fixed time or latest arrival. position_deg is 1-by-2.
%       Optional targetTime_s and targetPosition_deg define a moving goal.
%       Moving goals do not support azimuth wrapping.
%   - limits (scalar struct)
%       Positive 1-by-2 maxVelocity_deg_s, maxAcceleration_deg_s2, and
%       maxJerk_deg_s3 fields are required. Optional azimuthInterval_deg
%       and elevationInterval_deg define the workspace and default to
%       [-180 180] and [-90 90] degrees.
%   - optionOverrides (scalar struct, optional; default struct())
%       Partial options are accepted. Empty fields retain defaults.
%       MotionMethod is the maintained "corridorQuintic" implementation.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Stable success-or-failure motion, seed, validation, and diagnostics,
%       including five-stage timing, source, arrival, and first-valid time.
%       Invalid contracts throw. Expected planning failure returns
%       Success=false and preserves every attempted seed summary.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s, deg/s^2,
%     and deg/s^3. Histories are N-by-2 [azimuth elevation].
%**************************************************************************

%% Section 1: Validate Inputs And Apply Defaults

% Public data is normalized once here. Internal stages can therefore assume
% consistent shapes, units, option defaults, and protected obstacle records.
defaults = plannerDefaults();
if nargin == 0
    result = defaults;
    return;
end
if nargin < 4
    error("planAzElMotion:MissingInputs", "obstacles, initialState, goalState, and limits are required.");
end
planningTimer = tic;
if nargin < 5 || isempty(optionOverrides)
    optionOverrides = struct();
end
options = resolvePlannerOptions(defaults, optionOverrides);
[obstacles, initialState, goalState, limits] = ...
    azElInternal.normalizePlannerRequest( ...
    obstacles, initialState, goalState, limits, options);
[result, summaryTemplate] = azElInternal.emptyPlannerResult( ...
    obstacles, initialState, goalState, limits, options, ...
    azElPlannerMethods.corridor.validateTrajectory(), "corridorQuintic");
obstacles = azElInternal.obstacles.prepareDynamic(obstacles);
if options.Verbose
    fprintf("[AzEl] Planning started.\n");
    fprintf("[AzEl][setup] workspace az=[%.6g %.6g] deg, " + ...
        "el=[%.6g %.6g] deg.\n", limits.azimuthInterval_deg, limits.elevationInterval_deg);
    fprintf("[AzEl][setup] obstacles=%d, seeds<=%d, goalMode=%s.\n", ...
        numel(obstacles), options.MaximumSeedCount, options.GoalTimeMode);
end

%% Section 2: Validate Endpoint Feasibility

% Endpoint rejection is deliberately early: search cannot repair an endpoint
% that is outside the workspace or already inside protected geometry.
[endpointFeasible, endpointMessage, endpointReason] = ...
    azElInternal.validatePlannerEndpoints( ...
    obstacles, initialState, goalState, limits, options);
if ~endpointFeasible
    result.Message = endpointMessage;
    result.TerminationReason = endpointReason;
    result.SearchDiagnostics.TerminationReason = endpointReason;
    stageTiming = result.SearchDiagnostics.StageTiming;
    result = azElPlannerMethods.internal.stageTiming( ...
        result, planningTimer, stageTiming);
    if options.Verbose
        fprintf("[AzEl] Complete: success=0, reason=%s, elapsed=%.3f s. %s\n", ...
            endpointReason, result.ElapsedPlanningTime_s, endpointMessage);
    end
    return;
end

%% Section 3: Generate The Bounded Deterministic Seed Set

% Seeds describe route topology only. They are proposals, not valid motions;
% every generated candidate must still pass continuous validation.
seedTimer = tic;
if options.Verbose
    fprintf("[AzEl][seeds] generating topology proposals.\n");
end
[seeds, gridDiagnostics] = azElInternal.generateTopologySeeds( ...
    obstacles, initialState, goalState, limits, options);
gridDiagnostics.ElapsedTime_s = toc(seedTimer);
result.Seeds = seeds;
result.SearchDiagnostics.Grid = gridDiagnostics;
if options.Verbose
    fprintf("[AzEl][seeds] nodes=%d, visibleEdges=%d, rejectedEdges=%d, " + ...
        "expanded=%d, seeds=%d, elapsed=%.3f s.\n", ...
        gridDiagnostics.NodeCount, ...
        gridDiagnostics.VisibilityEdgeCount, ...
        gridDiagnostics.RejectedTransitionCount, ...
        gridDiagnostics.ExpandedCount, numel(seeds), gridDiagnostics.ElapsedTime_s);
end

%% Section 4: Solve, Validate, And Select A Candidate

% The runner tries the finite candidate families, preserves failure evidence,
% and returns the earliest independently validated motion with deterministic
% jerk and seed-index tie breakers.
result = azElPlannerMethods.corridor.internal.motion.runCorridorPlanner( ...
    result, summaryTemplate, seeds, gridDiagnostics, obstacles, initialState, ...
    goalState, limits, options, planningTimer);
stageTiming = result.SearchDiagnostics.StageTiming;
stageTiming.TopologyElapsedTime_s = gridDiagnostics.ElapsedTime_s;
result = azElPlannerMethods.internal.stageTiming( ...
    result, planningTimer, stageTiming);
return;
end


function options = plannerDefaults()
% Define the complete public planner option source of truth.
options = struct( ...
    "GoalTimeMode", "earliestArrival", ...
    "MotionMethod", "corridorQuintic", ...
    "SampleTime_s", 0.05, ...
    "AllowAzimuthWrapping", false, ...
    "MaximumSeedCount", 5, ...
    "DirectSeedOnly", false, ...
    "SeedClusterDistance_deg", 0, ...
    "ArrivalTimeTolerance_s", 1e-3, ...
    "ConstraintTolerance", 1e-7, ...
    "CollisionClearanceTolerance_deg", 1e-7, "CollisionMinimumTimeStep_s", 0.00025, "Verbose", false, "RandomSeed", 0);
end

function options = resolvePlannerOptions(defaults, overrides)
% Merge, normalize, and validate all public options once.
if ~isstruct(overrides) || ~isscalar(overrides)
    error("planAzElMotion:InvalidOptions", "optionOverrides must be a scalar struct.");
end
removedNames = ["MaximumPlanningTime_s", "AzimuthInterval_deg", "ElevationInterval_deg"];

% Reject each option that this replacement interface deliberately removed or relocated.
for removedName = removedNames
    if isfield(overrides, removedName)
        if removedName == "MaximumPlanningTime_s"
            error("planAzElMotion:RemovedMaximumPlanningTime", ...
                "MaximumPlanningTime_s has been removed. Use finite " + ...
                "algorithmic work limits and Verbose progress instead.");
        end
        replacementName = lower(extractBefore(removedName, "Interval")) + "Interval_deg";
        error("planAzElMotion:WorkspaceLimitMoved", ...
            "%s has moved from options to limits.%s.", removedName, replacementName);
    end
end
[options, unknownNames] = azElInternal.resolveOptions(defaults, overrides);
if ~isempty(unknownNames)
    warning("planAzElMotion:UnknownOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", strjoin(unknownNames, ", "));
end
options.GoalTimeMode = string(options.GoalTimeMode);
if ~isscalar(options.GoalTimeMode) || ~any(options.GoalTimeMode == ["earliestArrival", "fixedArrival"])
    error("planAzElMotion:InvalidGoalTimeMode", "GoalTimeMode must be 'earliestArrival' or 'fixedArrival'.");
end
options.MotionMethod = string(options.MotionMethod);
if ~isscalar(options.MotionMethod) || options.MotionMethod ~= "corridorQuintic"
    error("planAzElMotion:InvalidMotionMethod", "MotionMethod must be corridorQuintic on this replacement branch.");
end
logicalNames = ["AllowAzimuthWrapping", "DirectSeedOnly", "Verbose"];

% Normalize every logical option through the same scalar logical-or-binary contract.
for name = logicalNames
    options.(name) = azElInternal.normalizeLogicalScalar( ...
        options.(name), name, "planAzElMotion:InvalidLogicalOption");
end
validateattributes(options.SampleTime_s, {'numeric'}, {'real', 'finite', 'scalar', 'positive'});
validateattributes(options.MaximumSeedCount, {'numeric'}, {'real', 'finite', 'scalar', 'integer', '>=', 1, '<=', 9});
validateattributes(options.SeedClusterDistance_deg, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
positiveNames = [ "ArrivalTimeTolerance_s", "ConstraintTolerance", "CollisionMinimumTimeStep_s"];

% Validate each strictly positive tolerance with the same numeric contract.
for name = positiveNames
    validateattributes(options.(name), {'numeric'}, {'real', 'finite', 'scalar', 'positive'});
end
validateattributes(options.CollisionClearanceTolerance_deg, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
validateattributes(options.RandomSeed, {'numeric'}, {'real', 'finite', 'scalar', 'integer', 'nonnegative'});
end
