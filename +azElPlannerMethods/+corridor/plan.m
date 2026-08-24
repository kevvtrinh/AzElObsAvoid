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
obstacles = combineAzElObstacles(obstacles);
initialState = normalizeState(initialState, "initialState");
goalState = normalizeGoalState(goalState);
limits = normalizeLimits(limits);
if goalState.time_s <= initialState.time_s
    error("planAzElMotion:InvalidTimeWindow", "goalState.time_s must be greater than initialState.time_s.");
end
hasMovingGoal = isfield(goalState, "targetTime_s") && ~isempty(goalState.targetTime_s);
if options.AllowAzimuthWrapping && (~isempty(obstacles) || hasMovingGoal)
    error("planAzElMotion:UnsupportedWrappedGeometry", ...
        "AllowAzimuthWrapping is supported only for obstacle-free " + ...
        "fixed-position goals. Disable wrapping for this request.");
end
if options.AllowAzimuthWrapping
    turnCount = round((initialState.position_deg(1) - goalState.position_deg(1)) / 360);
    goalState.position_deg(1) = goalState.position_deg(1) + 360 * turnCount;
    if isfield(goalState, "targetPosition_deg") && ~isempty(goalState.targetPosition_deg)
        goalState.targetPosition_deg(:, 1) = goalState.targetPosition_deg(:, 1) + 360 * turnCount;
    end
end
[result, summaryTemplate] = azElPlannerMethods.corridor.internal.emptyAzElPlannerResult( obstacles, initialState, goalState, limits, options);
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
[endpointFeasible, endpointMessage, endpointReason] = validateEndpoints(obstacles, initialState, goalState, limits, options);
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
[seeds, gridDiagnostics] = azElPlannerMethods.corridor.internal.search.generateTopologySeeds( ...
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

function state = normalizeState(state, label)
% Normalize one fixed endpoint state and derivative defaults.
if ~isstruct(state) || ~isscalar(state) || ~all(isfield(state, {'time_s', 'position_deg'}))
    error("planAzElMotion:InvalidState", "%s must be a scalar struct with time_s and position_deg.", label);
end
validateattributes(state.time_s, {'numeric'}, {'real', 'finite', 'scalar'});
validateattributes(state.position_deg, {'numeric'}, {'real', 'finite', 'vector', 'numel', 2});
state.time_s = double(state.time_s);
state.position_deg = double(state.position_deg(:).');
state = defaultDerivative(state, "velocity_deg_s");
state = defaultDerivative(state, "acceleration_deg_s2");
end

function goalState = normalizeGoalState(goalState)
% Normalize a fixed goal or sampled moving-goal history.
goalState = normalizeState(goalState, "goalState");
if isfield(goalState, "targetTime_s") || isfield(goalState, "targetPosition_deg")
    if ~all(isfield(goalState, {'targetTime_s', 'targetPosition_deg'}))
        error("planAzElMotion:IncompleteMovingGoal", "targetTime_s and targetPosition_deg must be supplied together.");
    end
    validateattributes(goalState.targetTime_s, {'numeric'}, {'real', 'finite', 'vector', 'increasing'});
    goalState.targetTime_s = double(goalState.targetTime_s(:));
    if numel(goalState.targetTime_s) < 2
        error("planAzElMotion:MovingGoalHistoryTooShort", "targetTime_s must contain at least two increasing samples.");
    end
    validateattributes(goalState.targetPosition_deg, {'numeric'}, ...
        {'real', 'finite', '2d', 'ncols', 2, 'nrows', numel(goalState.targetTime_s)});
    goalState.targetPosition_deg = double(goalState.targetPosition_deg);
    if goalState.time_s < goalState.targetTime_s(1) || goalState.time_s > goalState.targetTime_s(end)
        error("planAzElMotion:MovingGoalHorizonOutsideHistory", "goalState.time_s must be inside targetTime_s.");
    end
    if ~isfield(goalState, "InterpolationMethod") || isempty(goalState.InterpolationMethod)
        goalState.InterpolationMethod = "linear";
    end
    goalState.InterpolationMethod = string(goalState.InterpolationMethod);
    if ~isscalar(goalState.InterpolationMethod) || ~any(goalState.InterpolationMethod == ["linear", "pchip"])
        error("planAzElMotion:InvalidGoalInterpolation", "InterpolationMethod must be 'linear' or 'pchip'.");
    end
end
end

function state = defaultDerivative(state, fieldName)
% Apply one two-axis zero derivative default at the public boundary.
if ~isfield(state, fieldName) || isempty(state.(fieldName))
    state.(fieldName) = [0 0];
else
    validateattributes(state.(fieldName), {'numeric'}, {'real', 'finite', 'vector', 'numel', 2});
    state.(fieldName) = double(state.(fieldName)(:).');
end
end

function limits = normalizeLimits(limits)
% Validate physical limits and own the workspace intervals.
requiredFields = ["maxVelocity_deg_s", "maxAcceleration_deg_s2", "maxJerk_deg_s3"];
if ~isstruct(limits) || ~isscalar(limits) || ~all(isfield(limits, cellstr(requiredFields)))
    error("planAzElMotion:InvalidLimits", "limits must contain velocity, acceleration, and jerk limits.");
end

% Normalize both axes of every required kinematic limit into finite row vectors.
for name = requiredFields
    validateattributes(limits.(name), {'numeric'}, {'real', 'finite', 'positive', 'vector', 'numel', 2});
    limits.(name) = double(limits.(name)(:).');
end
intervalDefaults = struct( "azimuthInterval_deg", [-180 180], "elevationInterval_deg", [-90 90]);
intervalNames = string(fieldnames(intervalDefaults));

% Fill and validate the azimuth and elevation workspace intervals independently.
for name = reshape(intervalNames, 1, [])
    if ~isfield(limits, name) || isempty(limits.(name))
        limits.(name) = intervalDefaults.(name);
    end
    validateattributes(limits.(name), {'numeric'}, ...
        {'real', 'finite', 'vector', 'numel', 2, 'increasing'}, "planAzElMotion", name);
    limits.(name) = double(limits.(name)(:).');
end
end

function [feasible, message, reason] = validateEndpoints( obstacles, initialState, goalState, limits, options)
% Return expected endpoint infeasibility without invoking the optimizer.
goalPosition_deg = azElInternal.goalPositionAtTime( ...
    goalState, goalState.time_s);
startIsBlocked = queryAzElTimeObstacle( ...
    obstacles, initialState.position_deg(1), initialState.position_deg(2), initialState.time_s);
fixedTerminalIsBlocked = false;
if options.GoalTimeMode == "fixedArrival"
    fixedTerminalIsBlocked = queryAzElTimeObstacle( ...
        obstacles, goalPosition_deg(1), goalPosition_deg(2), goalState.time_s);
end
if startIsBlocked || fixedTerminalIsBlocked
    feasible = false;
    message = "The protected geometry contains the start or fixed terminal point.";
    reason = "endpointBlocked";
    return;
end
derivativesWithinLimits = all(abs(initialState.velocity_deg_s) <= limits.maxVelocity_deg_s) && ...
    all(abs(goalState.velocity_deg_s) <= limits.maxVelocity_deg_s) && ...
    all(abs(initialState.acceleration_deg_s2) <= ...
    limits.maxAcceleration_deg_s2) && all(abs(goalState.acceleration_deg_s2) <= limits.maxAcceleration_deg_s2);
if ~derivativesWithinLimits
    feasible = false;
    message = "An endpoint derivative exceeds its physical limit.";
    reason = "dynamicEndpointInfeasible";
    return;
end
hasMovingGoal = isfield(goalState, "targetTime_s") && ~isempty(goalState.targetTime_s);
availableDuration_s = goalState.time_s - initialState.time_s;
if options.GoalTimeMode == "fixedArrival" || ~hasMovingGoal
    endpointDisplacement_deg = abs(goalPosition_deg - initialState.position_deg);
    minimumVelocityDuration_s = max(endpointDisplacement_deg ./ limits.maxVelocity_deg_s);
    if minimumVelocityDuration_s > availableDuration_s + options.ArrivalTimeTolerance_s
        feasible = false;
        message = sprintf( ...
            "The time window is too short for the endpoint displacement " + ...
            "at the configured velocity limits (minimum %.6g s, " + ...
            "available %.6g s). Increase goalState.time_s or the " + ...
            "velocity limits.", minimumVelocityDuration_s, availableDuration_s);
        reason = "timeWindowInfeasible";
        return;
    end
end
endpointPosition_deg = initialState.position_deg;
if options.GoalTimeMode == "fixedArrival" || ~hasMovingGoal
    endpointPosition_deg(end + 1, :) = goalPosition_deg;
end
positionWithinBounds = all(endpointPosition_deg(:, 2) >= limits.elevationInterval_deg(1)) && ...
    all(endpointPosition_deg(:, 2) <= limits.elevationInterval_deg(2));
if ~options.AllowAzimuthWrapping
    positionWithinBounds = positionWithinBounds && ...
        all(endpointPosition_deg(:, 1) >= limits.azimuthInterval_deg(1)) && ...
        all(endpointPosition_deg(:, 1) <= limits.azimuthInterval_deg(2));
end
if ~positionWithinBounds
    feasible = false;
    message = "An endpoint is outside the configured workspace.";
    reason = "endpointOutsideWorkspace";
    return;
end
feasible = true;
message = "";
reason = "";
end
