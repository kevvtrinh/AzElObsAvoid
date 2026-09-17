function result = planner(obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   options = planner()
%   result = planner(obstacles, initialState, goalState, limits)
%   result = planner(obstacles, initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Prepare protected polygon histories and an exact visibility guide,
%     then construct independently certified C3 quintic BMTP motion.
%   - Fixed-arrival moving-obstacle requests use bounded initial- and
%     arrival-snapshot shortcuts, then one time-expanded fallback. Failed
%     shortcuts never prove infeasibility, and every accepted motion passes
%     independent validation.
%   - Earliest-arrival requests use one capability-driven pipeline: an exact
%     static BMTP solve, or a departure incumbent followed by one timed
%     challenger, or chronological fixed-clock trials when required.
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
%       wrapping, endpoint matching, and search. SpatialProbeIterationLimit
%       is the explicit BMTP budget for each fixed-arrival snapshot shortcut.
%       IncumbentRefinementTrialLimit optionally spends bounded chronological
%       trials below a validated earliest-arrival incumbent; zero skips that
%       secondary refinement without discarding the incumbent.
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

[~, ~, ~, defaultOptions] = createDefaults();
if nargin == 0
    result = resolveOptions(struct(), defaultOptions);
    return
end
if nargin < 2
    initialState = [];
end
if nargin < 3
    goalState = [];
end
if nargin < 4
    limits = [];
end
if nargin < 5
    options = struct();
end
result = plannerCore(obstacles, initialState, goalState, limits, options, []);
end

function result = plannerCore(obstacles, initialState, goalState, limits, options, outerRequest)
    % Carry private recursion context without extending the public API.

% Recursive user path setup can put archived benchmark packages ahead of this
% checkout's engine. Keep planning and validation bound to the same checkout.
plannerFolder  = fileparts(mfilename('fullpath'));
engineFolder   = fullfile(plannerFolder, 'trajectory');
productionPath = [plannerFolder pathsep engineFolder];
if ~startsWith(path, [productionPath pathsep])
    addpath(productionPath, '-begin');
end

[defaultInitialState, defaultGoalState, defaultLimits, defaultOptions] = createDefaults();
if isempty(initialState)
    initialState = defaultInitialState;
end
if isempty(goalState)
    goalState = defaultGoalState;
end
if isempty(limits)
    limits = defaultLimits;
end
if isempty(options)
    options = struct();
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
        obstacles, initialState, goalState, limits, options, periodicRequest, @plannerCore);
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
[endpointFeasible, endpointMessage, endpointReason] = obstacleAvoidance.input.validatePlannerEndpoints( ...
    preparedObstacles, initialState, goalState, limits, options);
if ~endpointFeasible
    result.Message           = endpointMessage;
    result.TerminationReason = endpointReason;
    result.ElapsedTime_s     = toc(totalTimer);
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
coverage = struct("ExactRegionCount", numel(regions_units));
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
if options.GoalTimeMode == "earliestArrival"
    result = planEarliestArrival(result, scene, regions_units, coverage, ...
        initialState, goalState, limits, options, totalTimer, ...
        isDynamic, earliestTarget, isRest, @plannerCore);
    return
end

%% Section 4: Construct A Spatial Guide And Solve C3 Quintic Motion

motionGoalState     = goalState;
fixedArrivalDynamic = isDynamic && options.GoalTimeMode == "fixedArrival";
if fixedArrivalDynamic
    result = planFixedArrivalDynamic(result, scene, regions_units, coverage, ...
        initialState, motionGoalState, limits, options, totalTimer);
    return
end

visibilityGraph = getVisibilityGraph( ...
    scene, initialState.position_units, goalState.position_units, ...
    limits, options, "initialSpatialSnapshot");
result.VisibilityGraph = visibilityGraph;
if ~visibilityGraph.IsConnected
    result.Message           = "The initial visibility graph contains no start-to-goal route.";
    result.TerminationReason = "noVisibilityRoute";
    result.FailureStage      = "search";
    result.FailureKind       = "noSpatialRoute";
    result.ElapsedTime_s     = toc(totalTimer);
    return
end

route_units      = visibilityGraph.Route_units;
edgeLength_units = vecnorm(diff(route_units, 1, 1), 2, 2);
seed             = struct('position_units', route_units, ...
    'tau', [0; cumsum(edgeLength_units)] / sum(edgeLength_units), ...
    'Source', visibilityGraph.SearchKind);
[candidate, solverDiagnostics] = bmtpEngine.solve( ...
    seed, regions_units, coverage, initialState, motionGoalState, ...
    limits, options);

result = obstacleAvoidance.input.finalizeCandidate(result, candidate, route_units, solverDiagnostics);
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

function result = planEarliestArrival(result, scene, regions_units, coverage, ...
        initialState, goalState, limits, options, totalTimer, ...
        isDynamic, earliestTarget, isRest, plannerCore)
    % Keep one truthful method cascade. A validated candidate is an incumbent;
    % only a public-validator pass can be selected, and acceptance defects are
    % terminal instead of being hidden by a later method.
    attempts = repmat(createAttemptRecord(0, "", "", ""), 0, 1);
    fixedRestGoal = ~earliestTarget && isRest;
    capabilities = struct( ...
        'StaticSpatialBmtp',      ~isDynamic && fixedRestGoal, ...
        'DepartureFamily',        isDynamic && fixedRestGoal, ...
        'TimedVariableClockBmtp', isDynamic && fixedRestGoal, ...
        'ChronologicalFixedClock', earliestTarget || ~isRest || isDynamic);
    necessaryArrivalTime_s = NaN;
    if ~earliestTarget
        necessaryArrivalTime_s = initialState.time_s + ...
            obstacleAvoidance.input.minimumTravelTime(initialState, goalState, limits);
    end

    %% Static Fixed-Position Rest Requests Use One Exact Spatial Proposal
    if capabilities.StaticSpatialBmtp
        attemptTimer = tic;
        graph = getVisibilityGraph(scene, initialState.position_units, ...
            goalState.position_units, limits, options, "initialSpatialSnapshot");
        result.VisibilityGraph = graph;
        attempt = createAttemptRecord(1, "spatialVisibility", "primary", ...
            "earliestArrivalPolicy");
        attempt.NecessaryArrivalBound_s = necessaryArrivalTime_s;
        attempt.GraphConnected           = graph.IsConnected;
        attempt.GraphIsFullyEnumerated   = graph.GraphIsFullyEnumerated;
        attempt.RouteNodeCount           = size(graph.Route_units, 1);
        attempt.RouteLength_units        = graph.RouteLength_units;
        attempt.ExpandedCount            = graph.ExpandedCount;
        if ~graph.IsConnected
            attempt.GuideStatus       = "noRoute";
            attempt.FailureStage      = "search";
            attempt.FailureKind       = "noSpatialRoute";
            attempt.MotionStatus      = "notRun";
            attempt.Outcome           = "terminalFailure";
            attempt.TerminationReason = "noVisibilityRoute";
            attempt.Message           = "The exhaustive static visibility graph has no route.";
            attempt.ElapsedTime_s     = toc(attemptTimer);
            result.Message            = attempt.Message;
            result.TerminationReason  = attempt.TerminationReason;
            result.FailureStage       = attempt.FailureStage;
            result.FailureKind        = attempt.FailureKind;
            result.Attempts           = attempt;
            result.ElapsedTime_s      = toc(totalTimer);
            result = finishEarliestArrival(result, attempt, capabilities, ...
                necessaryArrivalTime_s, "staticGraphDisconnected");
            return
        end

        route_units      = graph.Route_units;
        edgeLength_units = vecnorm(diff(route_units, 1, 1), 2, 2);
        seed = struct( ...
            'position_units', route_units, ...
            'tau', [0; cumsum(edgeLength_units)] / sum(edgeLength_units), ...
            'Source', graph.SearchKind);
        attempt.GuideStatus     = "connected";
        attempt.SolverAttempted = true;
        [candidate, diagnostics] = bmtpEngine.solve(seed, regions_units, coverage, ...
            initialState, goalState, limits, options);
        candidateResult = obstacleAvoidance.input.finalizeCandidate( ...
            result, candidate, route_units, diagnostics);
        attempt = populateMotionAttempt(attempt, candidateResult, candidate, diagnostics);
        attempt.ElapsedTime_s = toc(attemptTimer);
        if candidateResult.Success
            attempt.MotionStatus = "accepted";
            attempt.Selected     = true;
            attempt.Outcome      = "accepted";
            candidateResult.Attempts = attempt;
            candidateResult.ElapsedTime_s = toc(totalTimer);
            result = finishEarliestArrival(candidateResult, attempt, capabilities, ...
                necessaryArrivalTime_s, "staticSpatialBmtpAccepted");
            return
        end
        if candidate.Success
            attempt.MotionStatus = "validationFailure";
        else
            attempt.MotionStatus = attempt.FailureKind;
        end
        attempt.Outcome = "terminalFailure";
        candidateResult.Attempts = attempt;
        candidateResult.ElapsedTime_s = toc(totalTimer);
        result = finishEarliestArrival(candidateResult, attempt, capabilities, ...
            necessaryArrivalTime_s, "staticSpatialBmtpFailed");
        return
    end

    %% Dynamic Fixed-Position Rest Requests Try The Direct Departure Family
    incumbentResult       = result;
    incumbentAccepted     = false;
    incumbentAttemptIndex = 0;
    if capabilities.DepartureFamily
        attemptTimer = tic;
        departureRoute_units = [initialState.position_units; goalState.position_units];
        departureSeed = struct( ...
            'position_units', departureRoute_units, ...
            'tau', [0; 1], ...
            'Source', "departureSchedule");
        attempt = createAttemptRecord(1, "analyticDeparture", ...
            "heuristicShortcut", "earliestArrivalPolicy");
        attempt.IsHeuristic             = true;
        attempt.NecessaryArrivalBound_s = necessaryArrivalTime_s;
        attempt.GuideStatus             = "directFamily";
        attempt.GraphConnected          = true;
        attempt.RouteNodeCount          = 2;
        attempt.RouteLength_units       = norm(diff(departureRoute_units, 1, 1));
        attempt.SolverAttempted         = true;
        [candidate, diagnostics] = bmtpEngine.solve( ...
            departureSeed, regions_units, coverage, initialState, goalState, ...
            limits, options);
        departureResult = obstacleAvoidance.input.finalizeCandidate( ...
            result, candidate, departureRoute_units, diagnostics);
        departureResult.VisibilityGraph.SearchKind         = "c3DepartureSchedule";
        departureResult.VisibilityGraph.Route_units        = departureRoute_units;
        departureResult.VisibilityGraph.RouteLength_units  = attempt.RouteLength_units;
        departureResult.VisibilityGraph.IsConnected        = true;
        departureResult.VisibilityGraph.GraphIsFullyEnumerated = false;
        attempt = populateMotionAttempt(attempt, departureResult, candidate, diagnostics);
        attempt.ElapsedTime_s = toc(attemptTimer);

        if candidate.Success && ~departureResult.Success
            attempt.MotionStatus = "validationFailure";
            attempt.Outcome      = "terminalFailure";
            departureResult.Attempts = attempt;
            departureResult.ElapsedTime_s = toc(totalTimer);
            result = finishEarliestArrival(departureResult, attempt, capabilities, ...
                necessaryArrivalTime_s, "departureValidationFailure");
            return
        elseif departureResult.Success
            incumbentResult       = departureResult;
            incumbentAccepted     = true;
            incumbentAttemptIndex = 1;
            attempt.MotionStatus  = "accepted";
            attempt.Outcome       = "incumbentRetained";
            attempt.IncumbentArrival_s = departureResult.ArrivalTime_s;
            attempts(end + 1, 1) = attempt;
            if departureResult.ArrivalTime_s <= necessaryArrivalTime_s + ...
                    options.ArrivalTimeTolerance_s
                attempts(1).Selected = true;
                attempts(1).Outcome  = "accepted";
                departureResult.Attempts = attempts;
                departureResult.ElapsedTime_s = toc(totalTimer);
                result = finishEarliestArrival(departureResult, attempts, capabilities, ...
                    necessaryArrivalTime_s, "necessaryArrivalBoundAttained");
                return
            end
        else
            attempt.MotionStatus = attempt.FailureKind;
            attempt.MethodFallbackEligible = methodFallbackEligible(departureResult);
            attempt.MethodFallbackReason   = attempt.FailureKind;
            attempt.FallbackEligible       = attempt.MethodFallbackEligible;
            attempt.FallbackReason         = attempt.MethodFallbackReason;
            if attempt.MethodFallbackEligible
                attempt.Outcome = "nextMethodAdmitted";
                attempts(end + 1, 1) = attempt;
            else
                attempt.Outcome = "terminalFailure";
                departureResult.Attempts = attempt;
                departureResult.ElapsedTime_s = toc(totalTimer);
                result = finishEarliestArrival(departureResult, attempt, capabilities, ...
                    necessaryArrivalTime_s, "departureTerminalFailure");
                return
            end
        end
    end

    %% One Time-Expanded Variable-Clock Challenger
    timedAttempted = false;
    timedResult    = result;
    if capabilities.TimedVariableClockBmtp
        maximumArrivalTime_s = goalState.time_s;
        if incumbentAccepted
            maximumArrivalTime_s = min(maximumArrivalTime_s, ...
                incumbentResult.ArrivalTime_s - options.ArrivalTimeTolerance_s);
        end
        if maximumArrivalTime_s > initialState.time_s + ...
                options.ArrivalTimeTolerance_s
            timedAttempted = true;
            timedBase = result;
            timedBase.Attempts = attempts;
            timedBase.ElapsedTime_s = toc(totalTimer);
            priorElapsedTime_s = timedBase.ElapsedTime_s;
            [timedResult, timedAccepted] = obstacleAvoidance.input.tryTimedArrival( ...
                timedBase, maximumArrivalTime_s);
            trigger = "departureFamilyUnavailable";
            if incumbentAccepted
                trigger = "validatedDepartureIncumbent";
            elseif ~isempty(attempts)
                trigger = attempts(end).MethodFallbackReason;
            end
            timedAttempt = createTimedAttemptRecord(numel(attempts) + 1, ...
                "primary", trigger, timedResult, timedAccepted, priorElapsedTime_s);
            timedAttempt.NecessaryArrivalBound_s = necessaryArrivalTime_s;
            if incumbentAccepted
                timedAttempt.IncumbentArrival_s = incumbentResult.ArrivalTime_s;
            end

            if timedAccepted
                attempts(end + 1, 1) = timedAttempt;
                incumbentIsNoLater = incumbentAccepted && ...
                    incumbentResult.ArrivalTime_s <= timedResult.ArrivalTime_s + ...
                    options.ArrivalTimeTolerance_s;
                if incumbentIsNoLater
                    attempts(incumbentAttemptIndex).Selected = true;
                    attempts(incumbentAttemptIndex).Outcome  = "accepted";
                    attempts(end).Outcome = "superseded";
                    result = incumbentResult;
                    stoppingReason = "departureIncumbentNoLater";
                else
                    if incumbentAccepted
                        attempts(incumbentAttemptIndex).Outcome = "superseded";
                    end
                    attempts(end).Selected = true;
                    attempts(end).Outcome  = "accepted";
                    result = timedResult;
                    stoppingReason = "timedVariableClockAccepted";
                end
                result.Attempts = attempts;
                result.ElapsedTime_s = toc(totalTimer);
                result = finishEarliestArrival(result, attempts, capabilities, ...
                    necessaryArrivalTime_s, stoppingReason);
                return
            end

            if string(timedResult.TerminationReason) == "invalidMotion"
                timedAttempt.MotionStatus = "validationFailure";
                timedAttempt.Outcome      = "terminalFailure";
                attempts(end + 1, 1)      = timedAttempt;
                timedResult.Attempts      = attempts;
                timedResult.ElapsedTime_s = toc(totalTimer);
                result = finishEarliestArrival(timedResult, attempts, capabilities, ...
                    necessaryArrivalTime_s, "timedValidationFailure");
                return
            end

            timedAttempt.MethodFallbackEligible = methodFallbackEligible(timedResult);
            timedAttempt.MethodFallbackReason   = timedAttempt.FailureKind;
            timedAttempt.FallbackEligible       = timedAttempt.MethodFallbackEligible;
            timedAttempt.FallbackReason         = timedAttempt.MethodFallbackReason;
            if timedAttempt.MethodFallbackEligible
                timedAttempt.Outcome = "nextMethodAdmitted";
                attempts(end + 1, 1) = timedAttempt;
            else
                timedAttempt.Outcome = "terminalFailure";
                attempts(end + 1, 1) = timedAttempt;
                timedResult.Attempts = attempts;
                timedResult.ElapsedTime_s = toc(totalTimer);
                result = finishEarliestArrival(timedResult, attempts, capabilities, ...
                    necessaryArrivalTime_s, "timedTerminalFailure");
                return
            end
        end
    end

    %% Chronological Fixed-Clock Fallback Or Primary Method
    if incumbentAccepted
        searchBase = incumbentResult;
    elseif timedAttempted
        searchBase = timedResult;
    else
        searchBase = result;
    end
    searchBase.Attempts      = attempts;
    searchBase.ElapsedTime_s = toc(totalTimer);
    chronologicalTrialLimit = options.MaxArrivalTrials;
    if incumbentAccepted
        chronologicalTrialLimit = min(chronologicalTrialLimit, ...
            options.IncumbentRefinementTrialLimit);
        if chronologicalTrialLimit == 0
            attempts(incumbentAttemptIndex).Selected = true;
            attempts(incumbentAttemptIndex).Outcome  = "accepted";
            incumbentResult.Attempts = attempts;
            incumbentResult.ElapsedTime_s = toc(totalTimer);
            result = finishEarliestArrival(incumbentResult, attempts, ...
                capabilities, necessaryArrivalTime_s, ...
                "incumbentRetainedAfterTimedFailure");
            return
        end
    end
    result = obstacleAvoidance.input.searchArrivalTimes( ...
        searchBase, plannerCore, @createAttemptRecord, ...
        @methodFallbackEligible, chronologicalTrialLimit);
    result.ElapsedTime_s = toc(totalTimer);
    result = finishEarliestArrival(result, result.Attempts, capabilities, ...
        necessaryArrivalTime_s, chronologicalStoppingReason(result));
end

function attempt = populateMotionAttempt(attempt, candidateResult, candidate, diagnostics)
    % Translate one BMTP result into the stable planner-level attempt schema.
    attempt.IterationLimit = readDiagnosticScalar( ...
        diagnostics, "MaximumAlternatingIterations", NaN);
    attempt.IterationCount = readDiagnosticScalar(diagnostics, "IterationCount", 0);
    attempt.CandidateSuccess            = candidate.Success;
    attempt.OptimizerFeasible           = readLogicalField(candidate, "OptimizerFeasible");
    attempt.OptimizerIterateUnavailable = readLogicalField( ...
        candidate, "OptimizerIterateUnavailable");
    attempt.AlternativeGuideEligible = readLogicalField( ...
        candidate, "AlternativeGuideEligible");
    attempt.FailureStage = readStringField(candidate, "FailureStage");
    attempt.FailureKind  = readStringField(candidate, "FailureKind");
    attempt.ValidationStatus = "notRun";
    if candidate.Success
        attempt.ValidationStatus = "failed";
        if candidateResult.Validation.Passed
            attempt.ValidationStatus = "passed";
        end
    end
    attempt.Success           = candidateResult.Success;
    attempt.TerminationReason = candidateResult.TerminationReason;
    attempt.Message           = candidateResult.Message;
    if isfield(candidateResult, 'ArrivalTime_s') && ...
            isnumeric(candidateResult.ArrivalTime_s) && ...
            isscalar(candidateResult.ArrivalTime_s) && ...
            isfinite(candidateResult.ArrivalTime_s)
        attempt.CandidateArrival_s = candidateResult.ArrivalTime_s;
    end
end

function attempt = createTimedAttemptRecord(index, role, trigger, ...
        timedResult, timedAccepted, priorElapsedTime_s)
    % Record the one time-expanded method without retaining stale prior motion.
    attempt = createAttemptRecord(index, "timedVisibility", role, trigger);
    attempt.GraphIsFullyEnumerated = false;
    attempt.GraphConnected = timedResult.VisibilityGraph.IsConnected;
    attempt.GuideStatus = "noRoute";
    if attempt.GraphConnected
        attempt.GuideStatus = "connected";
    end
    attempt.RouteNodeCount    = size(timedResult.Route_units, 1);
    attempt.RouteLength_units = timedResult.VisibilityGraph.RouteLength_units;
    attempt.ExpandedCount     = timedResult.VisibilityGraph.ExpandedCount;
    attempt.SolverAttempted   = attempt.GraphConnected;
    attempt.IterationLimit = readDiagnosticScalar( ...
        timedResult.SolverDiagnostics, "MaximumAlternatingIterations", NaN);
    attempt.IterationCount = readDiagnosticScalar( ...
        timedResult.SolverDiagnostics, "IterationCount", 0);
    attempt.CandidateSuccess = readLogicalField( ...
        timedResult.SolverDiagnostics, "Accepted");
    attempt.OptimizerFeasible = readLogicalField(timedResult, "OptimizerFeasible");
    attempt.OptimizerIterateUnavailable = readLogicalField( ...
        timedResult, "OptimizerIterateUnavailable");
    attempt.AlternativeGuideEligible = readLogicalField( ...
        timedResult, "AlternativeGuideEligible");
    attempt.FailureStage = readStringField(timedResult, "FailureStage");
    attempt.FailureKind  = readStringField(timedResult, "FailureKind");
    attempt.ValidationStatus = "notRun";
    if attempt.CandidateSuccess
        attempt.ValidationStatus = "failed";
        if timedResult.Validation.Passed
            attempt.ValidationStatus = "passed";
        end
    end
    attempt.Success           = timedAccepted;
    attempt.TerminationReason = timedResult.TerminationReason;
    attempt.Message           = timedResult.Message;
    attempt.Outcome           = "terminalFailure";
    if timedAccepted
        attempt.MotionStatus = "accepted";
        attempt.Outcome      = "acceptedCandidate";
        attempt.CandidateArrival_s = timedResult.ArrivalTime_s;
    elseif attempt.GraphConnected
        attempt.MotionStatus = attempt.FailureKind;
    end
    attempt.ElapsedTime_s = max(0, timedResult.ElapsedTime_s - priorElapsedTime_s);
end

function eligible = methodFallbackEligible(candidateResult)
    % Admit another method only for a typed method-local miss or bounded
    % optimization exhaustion. Unknown and acceptance-defect outcomes stop.
    eligible = false;
    if string(candidateResult.TerminationReason) == "invalidMotion"
        return
    end
    failureStage = readStringField(candidateResult, "FailureStage");
    failureKind  = readStringField(candidateResult, "FailureKind");
    if failureStage == "" && isfield(candidateResult, 'Attempts') && ...
            ~isempty(candidateResult.Attempts)
        failureStage = string(candidateResult.Attempts(end).FailureStage);
        failureKind  = string(candidateResult.Attempts(end).FailureKind);
    end
    eligible = any(failureStage == ["search", "timing", "proposal", "optimization"]);
    if failureStage == "optimization"
        eligible = any(failureKind == ["iterationLimit", ...
            "trajectorySolverIterationLimit", "timedPairSetStalled"]);
    end
    if failureStage == ""
        eligible = any(string(candidateResult.TerminationReason) == ...
            ["noSpatialRoute", "noVisibilityRoute", "noTimedRoute", "noDepartureWindow", ...
            "timedMotionInfeasible", "noOptimizedFeasibleIterate"]);
    end
end

function result = finishEarliestArrival(result, attempts, capabilities, ...
        necessaryArrivalTime_s, stoppingReason)
    % Publish consistent selection evidence for every earliest-arrival path.
    result.Attempts = attempts;
    selectedAttemptIndex = find([attempts.Selected], 1, 'last');
    if isempty(selectedAttemptIndex)
        selectedAttemptIndex = 0;
    end
    incumbentArrivalTime_s = NaN;
    if selectedAttemptIndex > 0 && ...
            isfinite(attempts(selectedAttemptIndex).CandidateArrival_s)
        incumbentArrivalTime_s = attempts(selectedAttemptIndex).CandidateArrival_s;
    end
    globalEarliestProven = result.Success && isfinite(necessaryArrivalTime_s) && ...
        result.ArrivalTime_s <= necessaryArrivalTime_s + ...
        result.Options.ArrivalTimeTolerance_s;
    unsearchedInterval_s = [NaN, NaN];
    if result.Success && ~globalEarliestProven && isfinite(necessaryArrivalTime_s)
        unsearchedUpperTime_s = result.ArrivalTime_s - ...
            result.Options.ArrivalTimeTolerance_s;
        if unsearchedUpperTime_s > necessaryArrivalTime_s
            unsearchedInterval_s = [necessaryArrivalTime_s, unsearchedUpperTime_s];
        end
    end
    chronologicalSearchUsed = false;
    if ~isempty(attempts)
        chronologicalSearchUsed = any([attempts.Kind] == ...
            "chronologicalFixedArrival");
    end
    result.EarliestArrival = struct( ...
        'Capabilities',             capabilities, ...
        'NecessaryArrivalBound_s',  necessaryArrivalTime_s, ...
        'IncumbentArrival_s',       incumbentArrivalTime_s, ...
        'SelectedAttemptIndex',     selectedAttemptIndex, ...
        'StoppingReason',           string(stoppingReason), ...
        'GlobalEarliestProven',     globalEarliestProven, ...
        'UnsearchedInterval_s',     unsearchedInterval_s, ...
        'ChronologicalSearchUsed',  chronologicalSearchUsed, ...
        'AttemptCount',             numel(attempts));
    if isfield(result, 'TemporalSearch')
        result.TemporalSearch.GlobalEarliestProven = globalEarliestProven;
        result.TemporalSearch.SelectedAttemptIndex = selectedAttemptIndex;
        result.TemporalSearch.StoppingReason       = string(stoppingReason);
    end
end

function reason = chronologicalStoppingReason(result)
    % Distinguish a selected trial, a retained incumbent, and honest exhaustion.
    reason = string(result.TerminationReason);
    if isfield(result, 'TemporalSearch')
        reason = string(result.TemporalSearch.StoppingReason);
    elseif result.Success
        reason = "chronologicalFixedArrivalAccepted";
    end
end

function result = planFixedArrivalDynamic(result, scene, regions_units, coverage, ...
        initialState, goalState, limits, options, totalTimer)
    % Two cheap exact-snapshot guides are deterministic shortcuts. Their
    % bounded failures never prove infeasibility; eligible failures advance
    % to the next guide and ultimately to one clean timed fallback.
    snapshotProbeIterationLimit = options.SpatialProbeIterationLimit;
    snapshotKinds = ["initialSpatialSnapshot", "arrivalSpatialSnapshot"];
    snapshotTimes_s = [initialState.time_s, goalState.time_s];
    attempts = repmat(createAttemptRecord(0, "", "", ""), 0, 1);
    previousRoute_units = zeros(0, 2);
    fallbackReason = "snapshotGuidesExhausted";

    for snapshotIndex = 1:numel(snapshotKinds)
        attemptTimer = tic;
        trigger = "fixedArrivalPolicy";
        if ~isempty(attempts)
            trigger = attempts(end).FallbackReason;
        end
        attempt = createAttemptRecord(snapshotIndex, "spatialVisibility", ...
            "heuristicShortcut", trigger);
        attempt.IsHeuristic       = true;
        attempt.IterationLimit    = snapshotProbeIterationLimit;
        attempt.GraphSnapshotTime_s = snapshotTimes_s(snapshotIndex);

        snapshotScene = scene;
        if snapshotIndex == 2
            snapshotScene = obstacleAvoidance.obstacles.snapshot( ...
                result.PreparedObstacles, goalState.time_s, false);
        end
        graph = getVisibilityGraph(snapshotScene, initialState.position_units, ...
            goalState.position_units, limits, options, snapshotKinds(snapshotIndex));
        result.VisibilityGraph         = graph;
        attempt.GraphConnected         = graph.IsConnected;
        attempt.GraphIsFullyEnumerated = graph.GraphIsFullyEnumerated;
        attempt.RouteNodeCount         = size(graph.Route_units, 1);
        attempt.RouteLength_units      = graph.RouteLength_units;
        attempt.ExpandedCount          = graph.ExpandedCount;

        if ~graph.IsConnected
            attempt.GuideStatus       = "noRoute";
            attempt.MotionStatus      = "notRun";
            attempt.FallbackEligible  = true;
            attempt.FallbackReason    = "spatialGraphDisconnected";
            attempt.Outcome           = "nextGuideAdmitted";
            attempt.TerminationReason = "noSpatialRoute";
            attempt.Message = "This exact snapshot graph has no route; it cannot rule out a route at another time.";
        elseif ~isempty(previousRoute_units) && isequaln(graph.Route_units, previousRoute_units)
            % Identical routes produce identical BMTP requests because the
            % full time-cell coverage, not the snapshot label, is solved.
            attempt.GuideStatus       = "duplicateRoute";
            attempt.MotionStatus      = "notRun";
            attempt.FallbackEligible  = true;
            attempt.FallbackReason    = "duplicateSpatialGuide";
            attempt.Outcome           = "nextGuideAdmitted";
            attempt.TerminationReason = "duplicateSpatialGuide";
            attempt.Message           = "The snapshot reproduced the preceding guide, so the duplicate solve was skipped.";
        else
            route_units      = graph.Route_units;
            previousRoute_units = route_units;
            edgeLength_units = vecnorm(diff(route_units, 1, 1), 2, 2);
            seed = struct( ...
                'position_units', route_units, ...
                'tau', [0; cumsum(edgeLength_units)] / sum(edgeLength_units), ...
                'Source', graph.SearchKind, ...
                'MaximumAlternatingIterations', snapshotProbeIterationLimit);
            attempt.GuideStatus     = "connected";
            attempt.SolverAttempted = true;
            [candidate, diagnostics] = bmtpEngine.solve(seed, regions_units, coverage, ...
                initialState, goalState, limits, options);
            candidateResult = obstacleAvoidance.input.finalizeCandidate( ...
                result, candidate, route_units, diagnostics);

            attempt.IterationCount = readDiagnosticScalar( ...
                diagnostics, "IterationCount", 0);
            attempt.CandidateSuccess            = candidate.Success;
            attempt.OptimizerFeasible           = candidate.OptimizerFeasible;
            attempt.OptimizerIterateUnavailable = candidate.OptimizerIterateUnavailable;
            attempt.AlternativeGuideEligible = readLogicalField( ...
                candidate, "AlternativeGuideEligible");
            attempt.FailureStage = readStringField(candidate, "FailureStage");
            attempt.FailureKind  = readStringField(candidate, "FailureKind");
            attempt.ValidationStatus = "notRun";
            if candidate.Success
                attempt.ValidationStatus = "failed";
                if candidateResult.Validation.Passed
                    attempt.ValidationStatus = "passed";
                end
            end
            attempt.Success           = candidateResult.Success;
            attempt.TerminationReason = candidateResult.TerminationReason;
            attempt.Message           = candidateResult.Message;

            if candidateResult.Success
                attempt.MotionStatus = "accepted";
                attempt.Selected     = true;
                attempt.Outcome      = "accepted";
                attempt.ElapsedTime_s = toc(attemptTimer);
                candidateResult.Attempts = [attempts; attempt];
                candidateResult.ElapsedTime_s = toc(totalTimer);
                result = candidateResult;
                return
            elseif candidate.Success
                % A public-validator rejection is a terminal defect, never a
                % reason to conceal the candidate behind another guide.
                attempt.MotionStatus = "validationFailure";
                attempt.Outcome      = "terminalFailure";
                attempt.ElapsedTime_s = toc(attemptTimer);
                candidateResult.Attempts = [attempts; attempt];
                candidateResult.ElapsedTime_s = toc(totalTimer);
                result = candidateResult;
                return
            elseif attempt.AlternativeGuideEligible
                attempt.MotionStatus     = attempt.FailureKind;
                attempt.FallbackEligible = true;
                attempt.FallbackReason   = attempt.FailureKind;
                attempt.Outcome          = "nextGuideAdmitted";
                result                   = candidateResult;
            else
                attempt.MotionStatus = attempt.FailureKind;
                attempt.Outcome      = "terminalFailure";
                attempt.ElapsedTime_s = toc(attemptTimer);
                candidateResult.Attempts = [attempts; attempt];
                candidateResult.ElapsedTime_s = toc(totalTimer);
                result = candidateResult;
                return
            end
        end
        attempt.ElapsedTime_s = toc(attemptTimer);
        attempts(end + 1, 1)  = attempt; %#ok<AGROW>
        fallbackReason        = attempt.FallbackReason;
    end

    % The timed fallback starts from a clean motion and graph record. It is
    % the only non-snapshot proposal and runs with the normal solver budget.
    result.Attempts     = attempts;
    result.ElapsedTime_s = toc(totalTimer);
    priorElapsedTime_s = result.ElapsedTime_s;
    [timedResult, timedAccepted] = obstacleAvoidance.input.tryTimedArrival(result);
    timedAttempt = createAttemptRecord(numel(attempts) + 1, ...
        "timedVisibility", "fallback", fallbackReason);
    timedAttempt.GraphIsFullyEnumerated = false;
    timedAttempt.GraphConnected = timedResult.VisibilityGraph.IsConnected;
    timedAttempt.GuideStatus = "noRoute";
    if timedAttempt.GraphConnected
        timedAttempt.GuideStatus = "connected";
    end
    timedAttempt.RouteNodeCount    = size(timedResult.Route_units, 1);
    timedAttempt.RouteLength_units = timedResult.VisibilityGraph.RouteLength_units;
    timedAttempt.ExpandedCount     = timedResult.VisibilityGraph.ExpandedCount;
    timedAttempt.SolverAttempted   = timedAttempt.GraphConnected;
    timedAttempt.IterationLimit = readDiagnosticScalar( ...
        timedResult.SolverDiagnostics, "MaximumAlternatingIterations", NaN);
    timedAttempt.IterationCount = readDiagnosticScalar( ...
        timedResult.SolverDiagnostics, "IterationCount", 0);
    timedAttempt.CandidateSuccess = readLogicalField( ...
        timedResult.SolverDiagnostics, "Accepted");
    timedAttempt.OptimizerFeasible = readLogicalField(timedResult, "OptimizerFeasible");
    timedAttempt.OptimizerIterateUnavailable = readLogicalField( ...
        timedResult, "OptimizerIterateUnavailable");
    timedAttempt.AlternativeGuideEligible = readLogicalField( ...
        timedResult, "AlternativeGuideEligible");
    timedAttempt.FailureStage     = readStringField(timedResult, "FailureStage");
    timedAttempt.FailureKind      = readStringField(timedResult, "FailureKind");
    timedAttempt.ValidationStatus = "notRun";
    if timedAttempt.CandidateSuccess
        timedAttempt.ValidationStatus = "failed";
        if timedResult.Validation.Passed
            timedAttempt.ValidationStatus = "passed";
        end
    end
    timedAttempt.Success           = timedAccepted;
    timedAttempt.TerminationReason = timedResult.TerminationReason;
    timedAttempt.Message           = timedResult.Message;
    timedAttempt.Outcome           = "terminalFailure";
    if timedAccepted
        timedAttempt.MotionStatus = "accepted";
        timedAttempt.Selected     = true;
        timedAttempt.Outcome      = "accepted";
    elseif timedAttempt.GraphConnected
        timedAttempt.MotionStatus = timedAttempt.FailureKind;
    end
    timedAttempt.ElapsedTime_s = max(0, timedResult.ElapsedTime_s - priorElapsedTime_s);
    timedResult.Attempts       = [attempts; timedAttempt];
    result                     = timedResult;
end

function attempt = createAttemptRecord(index, kind, role, trigger)
    % Keep one compact, stable record for every guide proposal.
    attempt = struct( ...
        "Index",                        index, ...
        "Kind",                         string(kind), ...
        "Role",                         string(role), ...
        "Trigger",                      string(trigger), ...
        "IsHeuristic",                  false, ...
        "IterationLimit",               NaN, ...
        "GraphSnapshotTime_s",          NaN, ...
        "TrialTime_s",                  NaN, ...
        "CandidateArrival_s",           NaN, ...
        "NecessaryArrivalBound_s",      NaN, ...
        "IncumbentArrival_s",           NaN, ...
        "PrescreenStatus",              "notRun", ...
        "PrescreenReason",              "", ...
        "GuideStatus",                  "notRun", ...
        "GraphConnected",               false, ...
        "GraphIsFullyEnumerated",       false, ...
        "RouteNodeCount",               0, ...
        "RouteLength_units",            Inf, ...
        "ExpandedCount",                0, ...
        "SolverAttempted",              false, ...
        "MotionStatus",                 "notRun", ...
        "IterationCount",               0, ...
        "CandidateSuccess",             false, ...
        "OptimizerFeasible",            false, ...
        "OptimizerIterateUnavailable",  false, ...
        "AlternativeGuideEligible",     false, ...
        "FailureStage",                 "", ...
        "FailureKind",                  "", ...
        "ValidationStatus",             "notRun", ...
        "FallbackEligible",             false, ...
        "FallbackReason",               "", ...
        "MethodFallbackEligible",       false, ...
        "MethodFallbackReason",         "", ...
        "Success",                      false, ...
        "Selected",                     false, ...
        "Outcome",                      "notRun", ...
        "TerminationReason",            "", ...
        "Message",                      "", ...
        "ElapsedTime_s",                0, ...
        "ChildAttempts",                {repmat(struct(), 0, 1)});
end

function value = readLogicalField(record, name)
    % Read a diagnostic logical without letting diagnostics control planning.
    value = false;
    if isstruct(record) && isscalar(record) && isfield(record, name)
        value = logical(record.(name));
    end
end

function value = readStringField(record, name)
    % Read an optional diagnostic string.
    value = "";
    if isstruct(record) && isscalar(record) && isfield(record, name)
        value = string(record.(name));
    end
end

function value = readDiagnosticScalar(record, name, defaultValue)
    % Read one finite scalar diagnostic or keep its declared default.
    value = defaultValue;
    if isstruct(record) && isscalar(record) && isfield(record, name) && ...
            isnumeric(record.(name)) && isscalar(record.(name)) && isfinite(record.(name))
        value = double(record.(name));
    end
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
        "SpatialProbeIterationLimit",        2, ...
        "IncumbentRefinementTrialLimit",     0, ...
        "MaxArrivalTrials",                  100, ...
        "MaxArrivalCandidates",              4096);
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

    integerOptionNames = ["SpatialProbeIterationLimit", ...
        "MaxArrivalTrials", "MaxArrivalCandidates"];
    for optionName = integerOptionNames
        validateattributes(options.(optionName), {'numeric'}, ...
            {'scalar', 'finite', 'integer', 'positive'});
        options.(optionName) = double(options.(optionName));
    end
    validateattributes(options.IncumbentRefinementTrialLimit, {'numeric'}, ...
        {'scalar', 'finite', 'integer', 'nonnegative'});
    options.IncumbentRefinementTrialLimit = ...
        double(options.IncumbentRefinementTrialLimit);
    if options.SpatialProbeIterationLimit > 35
        error("planner:InvalidSpatialProbeIterationLimit", ...
            "SpatialProbeIterationLimit must not exceed the full BMTP limit of 35.");
    end
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
    result.Attempts                     = repmat(createAttemptRecord( ...
        0, "", "", ""), 0, 1);
    result.Validation                   = struct("Passed", false, "Message", "No motion is available.");
    result.ArrivalTime_s                = NaN;
    result.Intercept                    = struct('Time_s', NaN, 'TargetPosition_units', goalState.position_units, ...
        'TerminalVelocityPolicy', "zero");
    result.TrajectoryDuration_s         = NaN;
    result.ElapsedTime_s                = 0;
end
