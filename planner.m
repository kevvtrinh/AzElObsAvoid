function result = planner(obstacles, initialState, goalState, limits, options, outerRequest)
%% Section 0: Header & Readme
% SYNTAX
%   options = planner()
%   result = planner(obstacles, initialState, goalState, limits)
%   result = planner(obstacles, initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Prepare protected polygon histories and an exact visibility guide,
%     then construct independently certified C3 quintic BMTP motion.
%   - Fixed-arrival moving-obstacle requests try the initial exact spatial
%     guide, a distinct arrival-snapshot guide, then one time-expanded guide.
%     Every accepted motion passes independent validation.
%**************************************************************************
% INPUTS
%   - obstacles (struct array)
%       Static or time-varying polygons; [] requests obstacle-free planning.
%   - initialState (scalar struct)
%       Example: struct("time_s", 0, "position_units", [-4 0])
%   - goalState (scalar struct)
%       Example: struct("time_s", 12, "position_units", [4 0])
%   - limits (scalar struct)
%       Workspace intervals and scalar or per-axis motion limits.
%   - options (scalar struct, optional; default struct())
%       ArrivalTimeTolerance_s bounds every comparison in seconds and
%       ConstraintTolerance bounds coordinates, derivatives, and algebraic
%       residuals; the other options control the arrival mode, sampling,
%       wrapping, endpoint matching, and search.
%       A wrapped axis is planned in the unwrapped frame inside the reach
%       band: obstacle images that meet the band, a target lifted by
%       continuity, and every goal image in the band planned and accepted
%       against the periodic request.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Stable success-or-failure record containing resolved inputs, prepared
%       geometry, visibility data, BMTP diagnostics, and Validation. Expected
%       no-path or infeasible outcomes return Success = false; invalid inputs
%       throw an error.
%   - options (scalar struct, zero-input call)
%       Fully resolved planner defaults.
%**************************************************************************
% UNITS
%   - Positions are 1-by-2 [x y] rows in coordinate units. Time is seconds;
%     velocity, acceleration, and jerk use units/s, units/s^2, and units/s^3.
%**************************************************************************

%% Section 1: Resolve Inputs And Validate The Request

% Recursive user path setup can put archived benchmark packages ahead of this
% checkout's engine. Keep planning and validation bound to the same checkout.
plannerFolder  = fileparts(mfilename('fullpath'));
engineFolder   = fullfile(plannerFolder, 'trajectory');
productionPath = [plannerFolder pathsep engineFolder];
if ~startsWith(path, [productionPath pathsep])
    addpath(productionPath, '-begin');
end

[defaultInitialState, defaultGoalState, defaultLimits, defaultOptions] = createDefaults();
if nargin == 0
    result = resolveOptions(struct(), defaultOptions);
    return
end
if nargin < 2 || isempty(initialState)
    initialState = defaultInitialState;
end
if nargin < 3 || isempty(goalState)
    goalState = defaultGoalState;
end
if nargin < 4 || isempty(limits)
    limits = defaultLimits;
end
if nargin < 5 || isempty(options)
    options = struct();
end
if nargin < 6
    outerRequest = [];
end
suppliedLimits     = limits;
suppliedGoalState  = goalState;
initialState       = normalizeState(initialState, defaultInitialState, "initialState");
goalState          = normalizeState(goalState, defaultGoalState, "goalState");
limits             = normalizeLimits(limits, defaultLimits);
options            = resolveOptions(options, defaultOptions);
requestedLimits    = limits;
requestedGoalState = goalState;

% Matched goal derivatives come from the target motion at the goal time.
if options.MatchTargetVelocity || options.MatchTargetAcceleration
    if isempty(goalState.targetMotion)
        error('planner:MissingTarget', 'Derivative matching requires goalState.targetMotion.');
    end
    derivativeNames     = ["velocity_units_s", "acceleration_units_s2"];
    matches             = [options.MatchTargetVelocity, options.MatchTargetAcceleration];
    [~, tgtVel, tgtAcc] = obstacleAvoidance.input.targetPositionAtTime(goalState.targetMotion, goalState.time_s);
    derivatives         = [tgtVel; tgtAcc];
    for derivativeIndex = find(matches)
        derivativeName   = derivativeNames(derivativeIndex);
        targetDerivative = derivatives(derivativeIndex, :);

        hasExplicitDerivative = isfield(suppliedGoalState, derivativeName) && ...
            ~isempty(suppliedGoalState.(derivativeName));
        if hasExplicitDerivative
            % Compare the normalized 1-by-2 row so a supplied column is not
            % broadcast into a 2-by-2 residual.
            derivativeResidual = abs(goalState.(derivativeName) - targetDerivative);
            if any(derivativeResidual > options.ConstraintTolerance)
                error('planner:ConflictingTargetDerivative', 'Explicit and matched target derivatives conflict.');
            end
        end
        goalState.(derivativeName) = targetDerivative;
    end
end

% A wrapped axis is planned in the unwrapped frame. A moving target is lifted
% by continuity from the initial position, a fixed goal takes its nearest
% image, and each wrapped interval becomes the reach band of the request.
wrapAxes = [options.WrapX options.WrapY];
if any(wrapAxes)
    intervalNames   = ["xInterval_units", "yInterval_units"];
    intervals_units = [limits.xInterval_units; limits.yInterval_units];
    if ~isempty(goalState.targetMotion)
        goalState.targetMotion   = obstacleAvoidance.input.liftPeriodicTarget( ...
            goalState.targetMotion, initialState.position_units, intervals_units, wrapAxes);
        goalState.position_units = obstacleAvoidance.input.targetPositionAtTime( ...
            goalState.targetMotion, goalState.time_s);
    end
    for axisIndex = find(wrapAxes)
        period_units = diff(intervals_units(axisIndex, :));
        start_units  = initialState.position_units(axisIndex);
        goal_units   = goalState.position_units(axisIndex);
        reach_units  = limits.maxVelocity_units_s(axisIndex) * (goalState.time_s - initialState.time_s);
        if isempty(goalState.targetMotion)
            periodOffset = floor((start_units - goal_units) / period_units + 0.5);
            goalState.position_units(axisIndex) = goal_units + period_units * periodOffset;
        end
        limits.(intervalNames(axisIndex)) = start_units + [-reach_units reach_units];
    end
end

if goalState.time_s <= initialState.time_s
    error("planTrajectory:InvalidTimeOrder", "goalState.time_s must be greater than initialState.time_s.");
end
% A moving target's deadline position only bounds the earliest-arrival
% search; a fixed goal or a fixed-arrival intercept is a required endpoint.
goalIsRequiredEndpoint = isempty(goalState.targetMotion) || options.GoalTimeMode == "fixedArrival";
endpointsCoincide      = norm(goalState.position_units - initialState.position_units) <= options.ConstraintTolerance;
if goalIsRequiredEndpoint && endpointsCoincide
    error("planTrajectory:CoincidentEndpoints", "Initial and goal positions must be distinct.");
end

% Every goal image inside the band is planned as a plain request in the
% unwrapped frame and accepted against this periodic request.
if any(wrapAxes)
    periodicRequest = struct( ...
        'SuppliedLimits',     suppliedLimits, ...
        'SuppliedGoalState',  suppliedGoalState, ...
        'RequestedLimits',    requestedLimits, ...
        'RequestedGoalState', requestedGoalState);
    result = obstacleAvoidance.input.planPeriodicRequest( ...
        obstacles, initialState, goalState, limits, options, periodicRequest);
    return
end

%% Section 2: Prepare Authoritative Geometry And Motion Coverage

totalTimer          = tic;
earliestTarget      = ~isempty(goalState.targetMotion) && options.GoalTimeMode == "earliestArrival";
requestedInterval_s = [initialState.time_s, goalState.time_s];
preparedObstacles   = obstacleAvoidance.obstacles.prepareObstacles(obstacles, requestedInterval_s, true);

% A request that leaves a multi-sample history also counts as dynamic.
isDynamic = false;
for obstacleIndex = 1:numel(preparedObstacles)
    obstacle = preparedObstacles(obstacleIndex);
    historyHasMultipleSamples = numel(obstacle.time_s) > 1;
    requestExceedsHistory     = historyHasMultipleSamples && ...
        (requestedInterval_s(1) < obstacle.time_s(1) || requestedInterval_s(2) > obstacle.time_s(end));
    obstacleChangesWithTime  = ~obstacle.InternalPreparation.IsTimeInvariant || requestExceedsHistory;
    if obstacleChangesWithTime
        isDynamic = true;
        break
    end
end

visibilityGraph = struct( ...
    'NodePosition_units',     zeros(0, 2), ...
    'AcceptedNodeIndex',      zeros(0, 2), ...
    'RejectedNodeIndex',      zeros(0, 2), ...
    'Route_units',            zeros(0, 2), ...
    'RouteLength_units',      Inf, ...
    'IsConnected',            false, ...
    'ExpandedCount',          0, ...
    'GraphIsFullyEnumerated', false, ...
    'SearchKind',             "notSearched");
result = createEmptyResult(obstacles, preparedObstacles, initialState, goalState, limits, options, visibilityGraph);
result.SuppliedLimits     = suppliedLimits;
result.RequestedLimits    = requestedLimits;
result.RequestedGoalState = requestedGoalState;
result.SuppliedGoalState  = suppliedGoalState;
if ~isempty(outerRequest)
    result.OuterRequest = outerRequest;
end

% Swept corresponding cells have a declared conservative continuous model.
% Only intervals without correspondence or any certificate stop preparation.
unsupportedObstacleIndex = [];
for obstacleIndex = 1:numel(preparedObstacles)
    preparation    = preparedObstacles(obstacleIndex).InternalPreparation;
    obstacleTime_s = preparedObstacles(obstacleIndex).time_s;

    intervalIsUnsupported   = preparation.IntervalPrepared & preparation.IntervalIsUnsupported;
    intervalOverlapsRequest = obstacleTime_s(1:end-1) < requestedInterval_s(2) & ...
        obstacleTime_s(2:end) > requestedInterval_s(1);

    unsupportedIntervalIndex = find(intervalIsUnsupported & intervalOverlapsRequest, 1);
    if ~isempty(unsupportedIntervalIndex)
        unsupportedObstacleIndex = obstacleIndex;
        break
    end
end

if ~isempty(unsupportedObstacleIndex)
    intervalTime_s = obstacleTime_s(unsupportedIntervalIndex:unsupportedIntervalIndex + 1);
    obstacleName   = string(preparedObstacles(unsupportedObstacleIndex).targetName);

    result.Message = sprintf(['Obstacle %d ("%s"), interval [%g, %g] s, has no ' ...
        'certified exact continuous interpolation.'], ...
        unsupportedObstacleIndex, obstacleName, ...
        intervalTime_s(1), intervalTime_s(2));

    hasCertificationReason = isfield(preparation, 'IntervalCertificationReason');
    if hasCertificationReason
        certificationReason = preparation.IntervalCertificationReason(unsupportedIntervalIndex);
        if certificationReason == "sweptEnvelopeExcludesProtectedSample"
            result.Message = result.Message + ...
                " The prescribed swept margin-square enclosure excludes " + ...
                "authoritative protected sample area.";
        end
    end

    result.TerminationReason = "unsupportedObstacleInterpolation";
    result.ElapsedTime_s     = toc(totalTimer);
    return
end

scene = obstacleAvoidance.obstacles.snapshot(preparedObstacles, initialState.time_s, ~isDynamic);
[endpointFeasible, result.Message, result.TerminationReason] = obstacleAvoidance.input.validatePlannerEndpoints( ...
    preparedObstacles, initialState, goalState, limits, options);
if ~endpointFeasible
    result.ElapsedTime_s = toc(totalTimer);
    return
end

% Dynamic scenes use exact time cells; static scenes use snapshot regions.
if isDynamic
    cells         = obstacleAvoidance.obstacles.createTimeCells(preparedObstacles, initialState.time_s, goalState.time_s);
    regions_units = cells.Regions_units;
else
    regionCount     = sum(arrayfun(@(obstacle) numel(obstacle.Regions_units), scene));
    regions_units   = cell(regionCount, 1);
    nextRegionIndex = 1;
    for obstacleIndex = 1:numel(scene)
        obstacleRegionCount          = numel(scene(obstacleIndex).Regions_units);
        targetIndices                = nextRegionIndex:nextRegionIndex + obstacleRegionCount - 1;
        regions_units(targetIndices) = scene(obstacleIndex).Regions_units;
        nextRegionIndex              = nextRegionIndex + obstacleRegionCount;
    end
end
coverage = struct("Passed", true, "ExactRegionCount", numel(regions_units));
if isDynamic
    coverage.ActiveTimeInterval_s = cells.ActiveTimeInterval_s;
    coverage.EndRegions_units     = cells.EndRegions_units;
    if options.GoalTimeMode == "fixedArrival"
        coverage.BreakTime_s = cells.BreakTime_s;
    end
end

%% Section 3: Plan Earliest-Arrival Motion

endpointDerivatives = [initialState.velocity_units_s, initialState.acceleration_units_s2, ...
    goalState.velocity_units_s, goalState.acceleration_units_s2];
isRest              = all(endpointDerivatives == 0);
needsTimedPlanning  = isDynamic || earliestTarget || ~isRest;
if options.GoalTimeMode == "earliestArrival" && needsTimedPlanning
    % The C3 chord is the retained analytic departure profile. A chord that
    % certifies with zero departure delay is accepted directly; a delayed
    % chord is only an incumbent that the single timed BMTP profile may beat.
    % Each candidate passes the public acceptance gate exactly once, and only
    % a candidate that passed it can be selected.
    departureResult   = result;
    departureAccepted = false;
    if isDynamic && ~earliestTarget && isRest
        departureRoute_units = [initialState.position_units; goalState.position_units];
        departureSeed        = struct('position_units', departureRoute_units, 'tau', [0; 1], 'Source', "departureSchedule");
        [departureCandidate, departureDiagnostics] = bmtpEngine.solve( ...
            departureSeed, regions_units, coverage, initialState, goalState, ...
            limits, options);
        if departureCandidate.Success
            departureResult = obstacleAvoidance.input.finalizeCandidate( ...
                result, departureCandidate, departureRoute_units, departureDiagnostics);
            departureResult.VisibilityGraph.SearchKind = "c3DepartureSchedule";
            departureAccepted = departureResult.Success;

            departureDelay_s = 0;
            if isfield(departureDiagnostics, 'DepartureSchedule')
                departureDelay_s = departureDiagnostics.DepartureSchedule.DepartureDelay_s;
            end
            if departureAccepted && departureDelay_s <= options.ArrivalTimeTolerance_s
                result               = departureResult;
                result.ElapsedTime_s = toc(totalTimer);
                return
            end
            if ~departureAccepted
                % A solver success the public gate rejected is reported to the
                % later stages as their prior outcome, never reselected.
                result.Message           = departureResult.Message;
                result.TerminationReason = departureResult.TerminationReason;
            end
        end
    end
    result.ElapsedTime_s         = toc(totalTimer);
    [timedResult, timedAccepted] = obstacleAvoidance.input.tryTimedArrival(result);
    if timedAccepted
        departureIsNoLater = departureAccepted && ...
            departureResult.ArrivalTime_s <= timedResult.ArrivalTime_s + options.ArrivalTimeTolerance_s;
        if departureIsNoLater
            result               = departureResult;
            result.ElapsedTime_s = toc(totalTimer);
        else
            result = timedResult;
        end
        return
    end
    if departureAccepted
        result               = departureResult;
        result.ElapsedTime_s = toc(totalTimer);
        return
    end
    result               = timedResult;
    result.ElapsedTime_s = toc(totalTimer);
    result               = obstacleAvoidance.input.searchArrivalTimes(result);
    return
end

%% Section 4: Construct A Spatial Guide And Solve C3 Quintic Motion

motionGoalState           = goalState;
route_units               = [initialState.position_units; goalState.position_units];
fixedArrivalDynamic       = isDynamic && options.GoalTimeMode == "fixedArrival";
initialSpatialAttempted   = false;
initialSpatialRoute_units = zeros(0, 2);
initialSpatialCandidate   = struct();
initialSpatialDiagnostics = struct();

if fixedArrivalDynamic
    initialVisibilityGraph = getVisibilityGraph( ...
        scene, initialState.position_units, goalState.position_units, ...
        limits, options, "initialSpatialSnapshot");
    if initialVisibilityGraph.IsConnected
        initialSpatialAttempted   = true;
        initialSpatialRoute_units = initialVisibilityGraph.Route_units;
        initialEdgeLength_units   = vecnorm(diff(initialSpatialRoute_units, 1, 1), 2, 2);
        initialSeed               = struct('position_units', initialSpatialRoute_units, ...
            'tau', [0; cumsum(initialEdgeLength_units)] / sum(initialEdgeLength_units), ...
            'Source', "initialSpatialSnapshot", ...
            'MaximumAlternatingIterations', 2);
        [initialSpatialCandidate, initialSpatialDiagnostics] = bmtpEngine.solve( ...
            initialSeed, regions_units, coverage, initialState, motionGoalState, ...
            limits, options);
        if initialSpatialCandidate.Success
            % A solver success that the public validator rejects is a defect
            % to diagnose upstream, not a reason to try another guide.
            result.VisibilityGraph = initialVisibilityGraph;
            result = obstacleAvoidance.input.finalizeCandidate( ...
                result, initialSpatialCandidate, initialSpatialRoute_units, ...
                initialSpatialDiagnostics);
            result.ElapsedTime_s = toc(totalTimer);
            return
        end
    end
end

% Choose the snapshot that seeds the guide route.
if fixedArrivalDynamic
    guideScene = obstacleAvoidance.obstacles.snapshot(preparedObstacles, motionGoalState.time_s, false);
    visibilityGraph = getVisibilityGraph( ...
        guideScene, initialState.position_units, goalState.position_units, ...
        limits, options, "arrivalSpatialSnapshot");
else
    visibilityGraph = getVisibilityGraph( ...
        scene, initialState.position_units, goalState.position_units, ...
        limits, options, "initialSpatialSnapshot");
end
if isDynamic && ~visibilityGraph.IsConnected
    % A disconnected spatial guide cannot rule out a later temporal opening.
    % This chord is only a seed; all time-dependent exclusions remain active.
    visibilityGraph.Route_units       = route_units;
    visibilityGraph.RouteLength_units = norm(diff(route_units));
    visibilityGraph.SearchKind        = "temporalDirectSeed";
end
result.VisibilityGraph = visibilityGraph;
if initialSpatialAttempted
    result.VisibilityGraph.InitialSpatialSeedDiagnostics = initialSpatialDiagnostics;
end
if ~isDynamic && ~visibilityGraph.IsConnected
    result.Message           = "The initial visibility graph contains no start-to-goal route.";
    result.TerminationReason = "noVisibilityRoute";
    result.ElapsedTime_s     = toc(totalTimer);
    return
end

route_units      = visibilityGraph.Route_units;
edgeLength_units = vecnorm(diff(route_units, 1, 1), 2, 2);
seed             = struct('position_units', route_units, ...
    'tau', [0; cumsum(edgeLength_units)] / sum(edgeLength_units), ...
    'Source', visibilityGraph.SearchKind);
if fixedArrivalDynamic
    % Give the exact spatial route its initial BMTP pass and one pass on the
    % resulting refined mesh. If neither pass certifies complete motion,
    % construct the exact timed route instead of repeatedly optimizing the
    % same failed homotopy.
    seed.MaximumAlternatingIterations = 2;
end
sameFailedSpatialRoute = fixedArrivalDynamic && initialSpatialAttempted && ...
    isequaln(route_units, initialSpatialRoute_units);
if sameFailedSpatialRoute
    candidate         = initialSpatialCandidate;
    solverDiagnostics = initialSpatialDiagnostics;
else
    [candidate, solverDiagnostics] = bmtpEngine.solve( ...
        seed, regions_units, coverage, initialState, motionGoalState, ...
        limits, options);
end

result = obstacleAvoidance.input.finalizeCandidate(result, candidate, route_units, solverDiagnostics);

% Only solver-level infeasibility of the spatial guide admits the timed guide.
% A motion the public validator rejects terminates here as a defect.
spatialFailureCanUseTimedGuide = ~candidate.Success && candidate.OptimizerIterateUnavailable;
if fixedArrivalDynamic && spatialFailureCanUseTimedGuide
    result.VisibilityGraph.SpatialSeedDiagnostics = solverDiagnostics;
    result.ElapsedTime_s                          = toc(totalTimer);
    [result, ~] = obstacleAvoidance.input.tryTimedArrival(result);
    return
end
result.ElapsedTime_s = toc(totalTimer);
end

%% Section 5: Local Functions

function graph = getVisibilityGraph(scene, start_units, goal_units, limits, options, kind)
    % Reuse only a graph with exactly identical geometry and public inputs.
    persistent previousInput previousGraph
    input = struct('Scene', scene, 'Start', start_units, 'Goal', goal_units, 'Limits', limits, 'Options', options);
    if isequaln(input, previousInput)
        graph = previousGraph;
    else
        graph         = obstacleAvoidance.search.createVisibilityGraph(scene, start_units, goal_units, limits, options);
        previousInput = input;
        previousGraph = graph;
    end
    graph.SearchKind = kind;
end

function [initialState, goalState, limits, options] = createDefaults()
    % Provide fallback state, limit, and option values for the public planner.
    initialState = struct( ...
        "time_s",                0, ...
        "position_units",        [-4 0], ...
        "velocity_units_s",      [0 0], ...
        "acceleration_units_s2", [0 0]);

    goalState = struct( ...
        "time_s",                12, ...
        "position_units",        [4 0], ...
        "velocity_units_s",      [0 0], ...
        "acceleration_units_s2", [0 0], ...
        "targetMotion",          []);

    limits = struct( ...
        "xInterval_units",         [-180 180], ...
        "yInterval_units",         [-90 90], ...
        "maxVelocity_units_s",      [2 2], ...
        "maxAcceleration_units_s2", [2 2], ...
        "maxJerk_units_s3",         [4 4]);

    options = struct( ...
        "GoalTimeMode",                      "fixedArrival", ...
        "SampleTime_s",                      0.05, ...
        "ConstraintTolerance",               1e-8, ...
        "CollisionClearanceTolerance_units", 1e-7, ...
        "ArrivalTimeTolerance_s",            1e-8, ...
        "WrapX",                             false, ...
        "WrapY",                             false, ...
        "MatchTargetVelocity",               false, ...
        "MatchTargetAcceleration",           false, ...
        "TemporalResolution_s",              0.5, ...
        "MaxArrivalTrials",                  100);
end

function state = normalizeState(state, defaults, argumentName)
    % Resolve omitted rest-to-rest fields and reject unsupported state data.
    if ~isstruct(state) || ~isscalar(state)
        error("planTrajectory:InvalidState", "%s must be a scalar struct.", argumentName);
    end
    allowedStateFields = string(fieldnames(defaults));
    unknownFields      = setdiff(string(fieldnames(state)), allowedStateFields);
    if ~isempty(unknownFields)
        error("planTrajectory:UnsupportedStateField", "%s contains unsupported fields: %s.", ...
            argumentName, strjoin(unknownFields, ", "));
    end
    for fieldName = reshape(allowedStateFields, 1, [])
        if ~isfield(state, fieldName) || isempty(state.(fieldName))
            state.(fieldName) = defaults.(fieldName);
        end
    end
    validateattributes(state.time_s, {'numeric'}, {'real', 'finite', 'scalar'});
    if isfield(state, 'targetMotion') && ~isempty(state.targetMotion)
        state.position_units = obstacleAvoidance.input.targetPositionAtTime(state.targetMotion, state.time_s);
    end
    for fieldName = ["position_units", "velocity_units_s", "acceleration_units_s2"]
        value = double(state.(fieldName));
        valueIsValid = isnumeric(state.(fieldName)) && isreal(value) && isvector(value) && ...
            numel(value) == 2 && all(isfinite(value));
        if ~valueIsValid
            error("planTrajectory:InvalidState", "%s.%s must be a finite 1-by-2 row.", argumentName, fieldName);
        end
        state.(fieldName) = reshape(value, 1, 2);
    end
    state.time_s = double(state.time_s);
end

function limits = normalizeLimits(limits, defaults)
    % Resolve the small fixed set of workspace and derivative limits.
    if ~isstruct(limits) || ~isscalar(limits)
        error("planTrajectory:InvalidLimits", "limits must be a scalar struct.");
    end
    limitNames    = string(fieldnames(defaults));
    unknownFields = setdiff(string(fieldnames(limits)), limitNames);
    if ~isempty(unknownFields)
        error("planTrajectory:UnsupportedLimitField", "Unsupported limit fields: %s.", strjoin(unknownFields, ", "));
    end
    for fieldName = reshape(limitNames, 1, [])
        if ~isfield(limits, fieldName) || isempty(limits.(fieldName))
            limits.(fieldName) = defaults.(fieldName);
        end
    end
    for fieldName = ["xInterval_units", "yInterval_units"]
        interval = double(limits.(fieldName));
        intervalIsValid = isnumeric(limits.(fieldName)) && isreal(interval) && isvector(interval) && ...
            numel(interval) == 2 && all(isfinite(interval)) && interval(2) > interval(1);
        if ~intervalIsValid
            error("planTrajectory:InvalidWorkspace", "%s must be a finite increasing 1-by-2 row.", fieldName);
        end
        limits.(fieldName) = reshape(interval, 1, 2);
    end
    derivativeLimitNames = ["maxVelocity_units_s", "maxAcceleration_units_s2", "maxJerk_units_s3"];
    limitSizes           = arrayfun(@(limitName) numel(limits.(limitName)), derivativeLimitNames);
    if any(limitSizes ~= limitSizes(1))
        error('planTrajectory:MixedLimitModes', ...
            ['Velocity, acceleration, and jerk limits must all be scalars ' ...
            'or all be two-element vectors.']);
    end
    for fieldName = derivativeLimitNames
        value = double(limits.(fieldName));
        if isscalar(value)
            value = [value value] / sqrt(2);
        end
        valueIsValid = isnumeric(limits.(fieldName)) && isreal(value) && isvector(value) && ...
            numel(value) == 2 && all(isfinite(value)) && all(value > 0);
        if ~valueIsValid
            error("planTrajectory:InvalidDerivativeLimit", "%s must be a positive scalar or finite 1-by-2 row.", fieldName);
        end
        limits.(fieldName) = reshape(value, 1, 2);
    end
end

function options = resolveOptions(options, defaults)
    % Resolve all BMTP controls in one place and warn once about unknown fields.
    if ~isstruct(options) || ~isscalar(options)
        error("planTrajectory:InvalidOptions", "options must be a scalar struct.");
    end

    [options, unknownFields] = obstacleAvoidance.input.resolveOptions(defaults, options);
    if ~isempty(unknownFields)
        warning("planTrajectory:UnknownOptions", "Ignoring unknown option fields: %s.", strjoin(unknownFields, ", "));
    end

    options.GoalTimeMode  = string(options.GoalTimeMode);
    allowedGoalTimeModes = ["fixedArrival", "earliestArrival"];
    goalTimeModeIsValid  = isscalar(options.GoalTimeMode) && any(options.GoalTimeMode == allowedGoalTimeModes);
    if ~goalTimeModeIsValid
        error("planner:UnsupportedGoalTimeMode", "GoalTimeMode must be fixedArrival or earliestArrival.");
    end

    numericOptionNames = ["SampleTime_s", "ConstraintTolerance", ...
        "CollisionClearanceTolerance_units", "ArrivalTimeTolerance_s", ...
        "TemporalResolution_s"];
    positiveScalarAttributes = {'real', 'finite', 'scalar', 'positive'};
    for fieldName = numericOptionNames
        validateattributes(options.(fieldName), {'numeric'}, positiveScalarAttributes);
        options.(fieldName) = double(options.(fieldName));
    end

    logicalOptionNames = ["WrapX", "WrapY", "MatchTargetVelocity", "MatchTargetAcceleration"];
    for optionName = logicalOptionNames
        options.(optionName) = obstacleAvoidance.input.normalizeLogicalScalar( ...
            options.(optionName), optionName, "planner:InvalidLogicalOption");
    end

    validateattributes(options.MaxArrivalTrials, {'numeric'}, {'scalar', 'finite', 'integer', 'positive'});
end

function result = createEmptyResult(obstacles, preparedObstacles, initialState, goalState, limits, options, visibilityGraph)
    % Keep one result schema for expected search and solver failures.
    result                              = struct();
    result.Success                      = false;
    result.Message                      = "Planning has not completed.";
    result.TerminationReason            = "notStarted";
    result.Inputs                       = struct("obstacles", {obstacles}, "initialState", initialState, ...
        "goalState", goalState);
    result.PreparedObstacles            = preparedObstacles;
    result.Limits                       = limits;
    result.Options                      = options;
    result.VisibilityGraph              = visibilityGraph;
    result.Route_units                  = zeros(0, 2);
    result.time_s                       = zeros(0, 1);
    result.position_units               = zeros(0, 2);
    result.velocity_units_s             = zeros(0, 2);
    result.acceleration_units_s2        = zeros(0, 2);
    result.jerk_units_s3                = zeros(0, 2);
    result.Polynomial                   = struct();
    result.PlaneCertificate             = struct();
    result.SolverDiagnostics            = struct();
    result.Validation                   = struct("Passed", false, "Message", "No motion is available.");
    result.ArrivalTime_s                = NaN;
    result.Intercept                    = struct('Time_s', NaN, 'TargetPosition_units', goalState.position_units, ...
        'TerminalVelocityPolicy', "zero");
    result.TrajectoryDuration_s         = NaN;
    result.ElapsedTime_s                = 0;
end
