function result = planNormalizedRequest(request, requestContext)
%% Section 0: Header & Readme
% SYNTAX
%   result = obstacleAvoidance.planning.planNormalizedRequest( ...
%       request, requestContext)
%**************************************************************************
% PURPOSE
%   - Plan one normalized, nonperiodic request through geometry preparation,
%     exact visibility search, BMTP motion generation, and validation.
%**************************************************************************
% INPUTS
%   - request (scalar struct)
%       Normalized planner states, limits, and options.
%   - requestContext (scalar struct)
%       Original inputs, provenance, and optional outer request.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Stable planner result. Expected planning failure returns
%       Success = false; invalid input throws an error.
%**************************************************************************
% UNITS
%   - Positions are coordinate units and time is seconds.
%**************************************************************************

obstacles    = requestContext.obstacles;
outerRequest = requestContext.outerRequest;

%% Section 2: Prepare Authoritative Geometry And Motion Coverage

totalTimer          = tic;
earliestTarget      = ~isempty(request.goalState.targetMotion) && ...
    request.options.GoalTimeMode == "earliestArrival";
requestedInterval_s = [request.initialState.time_s, request.goalState.time_s];
scene = struct('preparedObstacles', ...
    obstacleAvoidance.obstacles.prepareObstacles(obstacles, requestedInterval_s, true));

% A request that leaves a multi-sample history also counts as dynamic.
isDynamic = false;
for obstacleIndex = 1:numel(scene.preparedObstacles)
    obstacle = scene.preparedObstacles(obstacleIndex);
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
emptyAttempts = repmat(obstacleAvoidance.planning.createAttemptRecord(0, ""), 0, 1);
result = obstacleAvoidance.planning.createEmptyResult( ...
    scene.preparedObstacles, request, requestContext, visibilityGraph, emptyAttempts, 0);

% Swept corresponding cells have a declared conservative continuous model.
% Only intervals without correspondence or any certificate stop preparation.
unsupportedObstacleIndex = [];
for obstacleIndex = 1:numel(scene.preparedObstacles)
    preparation    = scene.preparedObstacles(obstacleIndex).InternalPreparation;
    obstacleTime_s = scene.preparedObstacles(obstacleIndex).time_s;

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
    obstacleName   = string(scene.preparedObstacles(unsupportedObstacleIndex).targetName);

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

snapshot = obstacleAvoidance.obstacles.snapshot( ...
    scene.preparedObstacles, request.initialState.time_s, ~isDynamic);
reuseEndpointValidation = false;
% Chronological trials carry a private prescreen pass. Wrapped recursion and
% any normalization difference fall through unless this complete key matches.
if isstruct(outerRequest) && isscalar(outerRequest) && ...
        isfield(outerRequest, 'EndpointValidation')
    endpointValidation = outerRequest.EndpointValidation;
    validationRecordIsComplete = isstruct(endpointValidation) && ...
        isscalar(endpointValidation) && ...
        all(isfield(endpointValidation, {'Key', 'Feasible', 'Message', 'Reason'})) && ...
        isequal(endpointValidation.Feasible, true);
    if validationRecordIsComplete
        endpointValidationOptions = struct( ...
            'GoalTimeMode',           request.options.GoalTimeMode, ...
            'WrapX',                  request.options.WrapX, ...
            'WrapY',                  request.options.WrapY, ...
            'ArrivalTimeTolerance_s', request.options.ArrivalTimeTolerance_s);
        initialEndpointState = struct( ...
            'time_s',                request.initialState.time_s, ...
            'position_units',        request.initialState.position_units, ...
            'velocity_units_s',      request.initialState.velocity_units_s, ...
            'acceleration_units_s2', request.initialState.acceleration_units_s2);
        goalEndpointState = struct( ...
            'time_s',                request.goalState.time_s, ...
            'position_units',        request.goalState.position_units, ...
            'velocity_units_s',      request.goalState.velocity_units_s, ...
            'acceleration_units_s2', request.goalState.acceleration_units_s2);
        endpointValidationKey = struct();
        endpointValidationKey.PreparedObstacles = scene.preparedObstacles;
        endpointValidationKey.RequestHorizon_s  = requestedInterval_s;
        endpointValidationKey.InitialState      = initialEndpointState;
        endpointValidationKey.GoalState         = goalEndpointState;
        endpointValidationKey.Limits            = request.limits;
        endpointValidationKey.Options           = endpointValidationOptions;
        reuseEndpointValidation = isequaln(endpointValidation.Key, endpointValidationKey);
    end
end
if reuseEndpointValidation
    endpointFeasible = endpointValidation.Feasible;
    endpointMessage  = endpointValidation.Message;
    endpointReason   = endpointValidation.Reason;
else
    [endpointFeasible, endpointMessage, endpointReason] = ...
        obstacleAvoidance.input.validatePlannerEndpoints( ...
        scene.preparedObstacles, request.initialState, request.goalState, ...
        request.limits, request.options);
end
if ~endpointFeasible
    result.Message           = endpointMessage;
    result.TerminationReason = endpointReason;
    result.ElapsedTime_s     = toc(totalTimer);
    return
end

% The initial-snapshot skeleton classifies every boundary vertex pair once.
% A chronological trial plans the same prepared geometry at the same initial
% time, so it receives the product under this key instead of rebuilding it.
skeletonKey = struct( ...
    'PreparedObstacles',   {scene.preparedObstacles}, ...
    'SnapshotTime_s',      request.initialState.time_s, ...
    'Limits',              request.limits, ...
    'ConstraintTolerance', request.options.ConstraintTolerance);
reuseSkeleton = isstruct(outerRequest) && isscalar(outerRequest) && ...
    isfield(outerRequest, 'InitialSkeleton') && ...
    isequaln(outerRequest.InitialSkeleton.Key, skeletonKey);
if reuseSkeleton
    skeleton = outerRequest.InitialSkeleton.Skeleton;
else
    skeleton = obstacleAvoidance.search.createVisibilitySkeleton( ...
        snapshot, request.limits, request.options);
end

% Dynamic scenes use exact time cells; static scenes use snapshot regions.
if isDynamic
    cells = obstacleAvoidance.obstacles.createTimeCells( ...
        scene.preparedObstacles, request.initialState.time_s, request.goalState.time_s);
    regions_units = cells.Regions_units;
    if request.options.GoalTimeMode == "fixedArrival"
        coverage = struct( ...
            'ExactRegionCount',       numel(regions_units), ...
            'ActiveTimeInterval_s',   cells.ActiveTimeInterval_s, ...
            'EndRegions_units',       {cells.EndRegions_units}, ...
            'BreakTime_s',            cells.BreakTime_s);
    else
        coverage = struct( ...
            'ExactRegionCount',       numel(regions_units), ...
            'ActiveTimeInterval_s',   cells.ActiveTimeInterval_s, ...
            'EndRegions_units',       {cells.EndRegions_units});
    end
    scene = struct( ...
        'preparedObstacles', scene.preparedObstacles, ...
        'snapshot',          snapshot, ...
        'skeleton',          skeleton, ...
        'regions_units',     {regions_units}, ...
        'coverage',          coverage);
else
    regionCount     = sum(arrayfun(@(obstacle) numel(obstacle.Regions_units), snapshot));
    regions_units   = cell(regionCount, 1);
    nextRegionIndex = 1;
    for obstacleIndex = 1:numel(snapshot)
        obstacleRegionCount          = numel(snapshot(obstacleIndex).Regions_units);
        targetIndices                = nextRegionIndex:nextRegionIndex + obstacleRegionCount - 1;
        regions_units(targetIndices) = snapshot(obstacleIndex).Regions_units;
        nextRegionIndex              = nextRegionIndex + obstacleRegionCount;
    end
    scene = struct( ...
        'preparedObstacles', scene.preparedObstacles, ...
        'snapshot',          snapshot, ...
        'skeleton',          skeleton, ...
        'regions_units',     {regions_units}, ...
        'coverage',          struct('ExactRegionCount', numel(regions_units)));
end

%% Section 3: Plan Earliest-Arrival Motion

endpointDerivatives = [request.initialState.velocity_units_s, ...
    request.initialState.acceleration_units_s2, ...
    request.goalState.velocity_units_s, request.goalState.acceleration_units_s2];
isRest              = all(endpointDerivatives == 0);
if request.options.GoalTimeMode == "earliestArrival"
    result = planEarliestArrival(result, scene, request, requestContext, totalTimer, ...
        isDynamic, earliestTarget, isRest);
    return
end

%% Section 4: Construct A Spatial Guide And Solve C3 Quintic Motion

motionGoalState     = request.goalState;
fixedArrivalDynamic = isDynamic && request.options.GoalTimeMode == "fixedArrival";
if fixedArrivalDynamic
    result = planFixedArrivalDynamic(result, scene, request, requestContext, totalTimer);
    return
end

visibilityGraph = getVisibilityGraph( ...
    scene.skeleton, request.initialState.position_units, request.goalState.position_units, ...
    "initialSpatialSnapshot");
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
    'tau', [0; cumsum(edgeLength_units)] / sum(edgeLength_units));
[candidate, solverDiagnostics] = bmtpEngine.solve( ...
    seed, scene.regions_units, scene.coverage, request.initialState, motionGoalState, ...
    request.limits, request.options, struct());

result = obstacleAvoidance.planning.finalizeCandidate( ...
    scene.preparedObstacles, request, requestContext, visibilityGraph, result.Attempts, ...
    result.ElapsedTime_s, false, struct(), candidate, route_units, solverDiagnostics);
result.ElapsedTime_s = toc(totalTimer);
end

%% Section 5: Local Functions

function graph = getVisibilityGraph(skeleton, start_units, goal_units, kind)
    % Attach this request's endpoints to a classified snapshot skeleton.
    graph = obstacleAvoidance.search.createVisibilityGraph(skeleton, start_units, goal_units);
    graph.SearchKind = kind;
end

function result = planEarliestArrival(result, scene, request, requestContext, totalTimer, ...
        isDynamic, earliestTarget, isRest)
    % Keep one truthful method cascade. A validated candidate is an incumbent;
    % only a public-validator pass can be selected, and acceptance defects are
    % terminal instead of being hidden by a later method.
    attempts = repmat(obstacleAvoidance.planning.createAttemptRecord(0, ""), 0, 1);
    fixedRestGoal = ~earliestTarget && isRest;
    capabilities = struct( ...
        'StaticSpatialBmtp',      ~isDynamic && fixedRestGoal, ...
        'DepartureFamily',        isDynamic && fixedRestGoal, ...
        'TimedVariableClockBmtp', isDynamic && fixedRestGoal, ...
        'ChronologicalFixedClock', earliestTarget || ~isRest || isDynamic);
    necessaryArrivalTime_s = NaN;
    if ~earliestTarget
        necessaryArrivalTime_s = request.initialState.time_s + ...
            obstacleAvoidance.input.minimumTravelTime( ...
            request.initialState, request.goalState, request.limits);
    end

    %% Static Fixed-Position Rest Requests Use One Exact Spatial Proposal
    if capabilities.StaticSpatialBmtp
        attemptTimer = tic;
        graph = getVisibilityGraph(scene.skeleton, request.initialState.position_units, ...
            request.goalState.position_units, "initialSpatialSnapshot");
        result.VisibilityGraph = graph;
        attempt = obstacleAvoidance.planning.createAttemptRecord(1, "spatialVisibility");
        attempt.NecessaryArrivalBound_s = necessaryArrivalTime_s;
        attempt.GraphConnected           = graph.IsConnected;
        attempt.GraphIsFullyEnumerated   = graph.GraphIsFullyEnumerated;
        attempt.RouteNodeCount           = size(graph.Route_units, 1);
        attempt.RouteLength_units        = graph.RouteLength_units;
        attempt.ExpandedCount            = graph.ExpandedCount;
        if ~graph.IsConnected
            attempt.FailureStage     = "search";
            attempt.FailureKind      = "noSpatialRoute";
            attempt.ElapsedTime_s    = toc(attemptTimer);
            result.Message           = "The exhaustive static visibility graph has no route.";
            result.TerminationReason = "noVisibilityRoute";
            result.FailureStage      = attempt.FailureStage;
            result.FailureKind       = attempt.FailureKind;
            result = finishEarliestArrival(result, attempt, capabilities, totalTimer, ...
                necessaryArrivalTime_s);
            return
        end

        route_units      = graph.Route_units;
        edgeLength_units = vecnorm(diff(route_units, 1, 1), 2, 2);
        seed = struct( ...
            'position_units', route_units, ...
            'tau', [0; cumsum(edgeLength_units)] / sum(edgeLength_units));
        attempt.SolverAttempted = true;
        [candidate, diagnostics] = bmtpEngine.solve(seed, scene.regions_units, scene.coverage, ...
            request.initialState, request.goalState, request.limits, request.options, struct());
        candidateResult = obstacleAvoidance.planning.finalizeCandidate( ...
            scene.preparedObstacles, request, requestContext, graph, result.Attempts, ...
            result.ElapsedTime_s, false, struct(), candidate, route_units, diagnostics);
        attempt = populateMotionAttempt(attempt, candidateResult, candidate, diagnostics);
        attempt.ElapsedTime_s = toc(attemptTimer);
        if candidateResult.Success
            attempt.Selected = true;
            result = finishEarliestArrival(candidateResult, attempt, capabilities, totalTimer, ...
                necessaryArrivalTime_s);
            return
        end
        result = finishEarliestArrival(candidateResult, attempt, capabilities, totalTimer, ...
            necessaryArrivalTime_s);
        return
    end

    %% Dynamic Fixed-Position Rest Requests Try The Direct Departure Family
    incumbentResult       = result;
    incumbentAccepted     = false;
    incumbentAttemptIndex = 0;
    if capabilities.DepartureFamily
        attemptTimer = tic;
        departureRoute_units = [request.initialState.position_units; ...
            request.goalState.position_units];
        departureSeed = struct( ...
            'position_units', departureRoute_units, ...
            'tau', [0; 1]);
        attempt = obstacleAvoidance.planning.createAttemptRecord(1, "analyticDeparture");
        attempt.IsHeuristic             = true;
        attempt.NecessaryArrivalBound_s = necessaryArrivalTime_s;
        attempt.GraphConnected          = true;
        attempt.RouteNodeCount          = 2;
        attempt.RouteLength_units       = norm(diff(departureRoute_units, 1, 1));
        attempt.SolverAttempted         = true;
        [candidate, diagnostics] = bmtpEngine.solve( ...
            departureSeed, scene.regions_units, scene.coverage, request.initialState, ...
            request.goalState, request.limits, request.options, struct());
        departureGraph = result.VisibilityGraph;
        departureGraph.SearchKind             = "c3DepartureSchedule";
        departureGraph.Route_units            = departureRoute_units;
        departureGraph.RouteLength_units      = attempt.RouteLength_units;
        departureGraph.IsConnected            = true;
        departureGraph.GraphIsFullyEnumerated = false;
        departureResult = obstacleAvoidance.planning.finalizeCandidate( ...
            scene.preparedObstacles, request, requestContext, departureGraph, result.Attempts, ...
            result.ElapsedTime_s, false, struct(), candidate, departureRoute_units, diagnostics);
        attempt = populateMotionAttempt(attempt, departureResult, candidate, diagnostics);
        attempt.ElapsedTime_s = toc(attemptTimer);

        if candidate.Success && ~departureResult.Success
            result = finishEarliestArrival(departureResult, attempt, capabilities, totalTimer, ...
                necessaryArrivalTime_s);
            return
        elseif departureResult.Success
            incumbentResult       = departureResult;
            incumbentAccepted     = true;
            incumbentAttemptIndex = 1;
            attempt.IncumbentArrival_s = departureResult.ArrivalTime_s;
            attempts(end + 1, 1) = attempt;
            if departureResult.ArrivalTime_s <= necessaryArrivalTime_s + ...
                    request.options.ArrivalTimeTolerance_s
                attempts(1).Selected = true;
                result = finishEarliestArrival(departureResult, attempts, capabilities, totalTimer, ...
                    necessaryArrivalTime_s);
                return
            end
        else
            attempt.MethodFallbackEligible = ...
                obstacleAvoidance.planning.methodFallbackEligible(departureResult);
            attempt.MethodFallbackReason   = attempt.FailureKind;
            attempt.FallbackEligible       = attempt.MethodFallbackEligible;
            if attempt.MethodFallbackEligible
                attempts(end + 1, 1) = attempt;
            else
                result = finishEarliestArrival(departureResult, attempt, capabilities, totalTimer, ...
                    necessaryArrivalTime_s);
                return
            end
        end
    end

    %% One Time-Expanded Variable-Clock Challenger
    timedAttempted = false;
    timedResult    = result;
    if capabilities.TimedVariableClockBmtp
        maximumArrivalTime_s = request.goalState.time_s;
        if incumbentAccepted
            maximumArrivalTime_s = min(maximumArrivalTime_s, ...
                incumbentResult.ArrivalTime_s - request.options.ArrivalTimeTolerance_s);
        end
        if maximumArrivalTime_s > request.initialState.time_s + ...
                request.options.ArrivalTimeTolerance_s
            timedAttempted = true;
            priorElapsedTime_s = toc(totalTimer);
            [timedResult, timedAccepted] = obstacleAvoidance.planning.tryTimedArrival( ...
                request, requestContext, scene.preparedObstacles, attempts, ...
                priorElapsedTime_s, struct(), maximumArrivalTime_s);
            timedAttempt = createTimedAttemptRecord(numel(attempts) + 1, ...
                timedResult, timedAccepted, priorElapsedTime_s);
            timedAttempt.NecessaryArrivalBound_s = necessaryArrivalTime_s;
            if incumbentAccepted
                timedAttempt.IncumbentArrival_s = incumbentResult.ArrivalTime_s;
            end

            if timedAccepted
                attempts(end + 1, 1) = timedAttempt;
                incumbentIsNoLater = incumbentAccepted && ...
                    incumbentResult.ArrivalTime_s <= timedResult.ArrivalTime_s + ...
                    request.options.ArrivalTimeTolerance_s;
                if incumbentIsNoLater
                    attempts(incumbentAttemptIndex).Selected = true;
                    result = incumbentResult;
                else
                    attempts(end).Selected = true;
                    result = timedResult;
                end
                result = finishEarliestArrival(result, attempts, capabilities, totalTimer, ...
                    necessaryArrivalTime_s);
                return
            end

            if string(timedResult.TerminationReason) == "invalidMotion"
                attempts(end + 1, 1)      = timedAttempt;
                result = finishEarliestArrival(timedResult, attempts, capabilities, totalTimer, ...
                    necessaryArrivalTime_s);
                return
            end

            timedAttempt.MethodFallbackEligible = ...
                obstacleAvoidance.planning.methodFallbackEligible(timedResult);
            timedAttempt.MethodFallbackReason   = timedAttempt.FailureKind;
            timedAttempt.FallbackEligible       = timedAttempt.MethodFallbackEligible;
            if incumbentAccepted
                incumbentResult.SolverDiagnostics.TimedChallenger = struct( ...
                    'AttemptIndex',                  timedAttempt.Index, ...
                    'Success',                       false, ...
                    'TerminationReason',             string(timedResult.TerminationReason), ...
                    'Message',                       string(timedResult.Message), ...
                    'FailureStage',                  timedAttempt.FailureStage, ...
                    'FailureKind',                   timedAttempt.FailureKind, ...
                    'OptimizerIterateUnavailable',   timedAttempt.OptimizerIterateUnavailable, ...
                    'MethodFallbackEligible',        timedAttempt.MethodFallbackEligible, ...
                    'VisibilityGraph',               timedResult.VisibilityGraph, ...
                    'Route_units',                   timedResult.Route_units, ...
                    'SolverDiagnostics',             timedResult.SolverDiagnostics);
            end
            if timedAttempt.MethodFallbackEligible
                attempts(end + 1, 1) = timedAttempt;
            else
                if incumbentAccepted
                    timedAttempt.Message = string(timedResult.Message);
                    timedAttempt.SolverExitFlag = readDiagnosticScalar( ...
                        timedResult.SolverDiagnostics, "LastTrajectoryExitFlag", NaN);
                end
                attempts(end + 1, 1) = timedAttempt;
                if incumbentAccepted
                    attempts(incumbentAttemptIndex).Selected = true;
                    result = finishEarliestArrival(incumbentResult, attempts, capabilities, ...
                        totalTimer, necessaryArrivalTime_s);
                    return
                end
                result = finishEarliestArrival(timedResult, attempts, capabilities, totalTimer, ...
                    necessaryArrivalTime_s);
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
    searchBase.ElapsedTime_s = toc(totalTimer);
    chronologicalTrialLimit = request.options.MaxArrivalTrials;
    if incumbentAccepted
        chronologicalTrialLimit = min(chronologicalTrialLimit, ...
            request.options.IncumbentRefinementTrialLimit);
        if chronologicalTrialLimit == 0
            attempts(incumbentAttemptIndex).Selected = true;
            result = finishEarliestArrival(incumbentResult, attempts, capabilities, ...
                totalTimer, necessaryArrivalTime_s);
            return
        end
    end
    result = obstacleAvoidance.planning.searchArrivalTimes( ...
        request, requestContext, scene, searchBase, attempts, ...
        chronologicalTrialLimit);
    result = finishEarliestArrival(result, result.Attempts, capabilities, totalTimer, ...
        necessaryArrivalTime_s);
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
    attempt.Success = candidateResult.Success;
    if isfield(candidateResult, 'ArrivalTime_s') && ...
            isnumeric(candidateResult.ArrivalTime_s) && ...
            isscalar(candidateResult.ArrivalTime_s) && ...
            isfinite(candidateResult.ArrivalTime_s)
        attempt.CandidateArrival_s = candidateResult.ArrivalTime_s;
    end
end

function attempt = createTimedAttemptRecord(index, timedResult, timedAccepted, ...
        priorElapsedTime_s)
    % Record the one time-expanded method without retaining stale prior motion.
    attempt = obstacleAvoidance.planning.createAttemptRecord(index, "timedVisibility");
    attempt.GraphIsFullyEnumerated = false;
    attempt.GraphConnected = timedResult.VisibilityGraph.IsConnected;
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
    attempt.Success = timedAccepted;
    if timedAccepted
        attempt.CandidateArrival_s = timedResult.ArrivalTime_s;
    end
    attempt.ElapsedTime_s = max(0, timedResult.ElapsedTime_s - priorElapsedTime_s);
end

function result = finishEarliestArrival(result, attempts, capabilities, totalTimer, ...
        necessaryArrivalTime_s)
    % Publish consistent selection evidence for every earliest-arrival path.
    result.Attempts      = attempts;
    result.ElapsedTime_s = toc(totalTimer);
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
        'GlobalEarliestProven',     globalEarliestProven, ...
        'UnsearchedInterval_s',     unsearchedInterval_s, ...
        'ChronologicalSearchUsed',  chronologicalSearchUsed, ...
        'AttemptCount',             numel(attempts));
    if isfield(result, 'TemporalSearch')
        result.TemporalSearch.GlobalEarliestProven = globalEarliestProven;
        result.TemporalSearch.SelectedAttemptIndex = selectedAttemptIndex;
    end
end

function result = planFixedArrivalDynamic(result, scene, request, requestContext, totalTimer)
    % Two cheap exact-snapshot guides are deterministic shortcuts. Their
    % bounded failures never prove infeasibility; eligible failures advance
    % to the next guide and ultimately to one clean timed fallback.
    snapshotProbeIterationLimit = request.options.SpatialProbeIterationLimit;
    snapshotKinds = ["initialSpatialSnapshot", "arrivalSpatialSnapshot"];
    snapshotTimes_s = [request.initialState.time_s, request.goalState.time_s];
    attempts = repmat(obstacleAvoidance.planning.createAttemptRecord(0, ""), 0, 1);
    previousRoute_units = zeros(0, 2);
    directMotion = struct();

    for snapshotIndex = 1:numel(snapshotKinds)
        attemptTimer = tic;
        attempt = obstacleAvoidance.planning.createAttemptRecord( ...
            snapshotIndex, "spatialVisibility");
        attempt.IsHeuristic       = true;
        attempt.IterationLimit    = snapshotProbeIterationLimit;
        attempt.GraphSnapshotTime_s = snapshotTimes_s(snapshotIndex);

        skeleton = scene.skeleton;
        if snapshotIndex == 2
            arrivalSnapshot = obstacleAvoidance.obstacles.snapshot( ...
                scene.preparedObstacles, request.goalState.time_s, false);
            skeleton = obstacleAvoidance.search.createVisibilitySkeleton( ...
                arrivalSnapshot, request.limits, request.options);
        end
        graph = getVisibilityGraph(skeleton, request.initialState.position_units, ...
            request.goalState.position_units, snapshotKinds(snapshotIndex));
        result.VisibilityGraph         = graph;
        attempt.GraphConnected         = graph.IsConnected;
        attempt.GraphIsFullyEnumerated = graph.GraphIsFullyEnumerated;
        attempt.RouteNodeCount         = size(graph.Route_units, 1);
        attempt.RouteLength_units      = graph.RouteLength_units;
        attempt.ExpandedCount          = graph.ExpandedCount;

        if ~graph.IsConnected
            attempt.FallbackEligible = true;
        elseif ~isempty(previousRoute_units) && isequaln(graph.Route_units, previousRoute_units)
            % Identical routes produce identical BMTP requests because the
            % full time-cell coverage, not the snapshot label, is solved.
            attempt.FallbackEligible = true;
        else
            route_units      = graph.Route_units;
            previousRoute_units = route_units;
            edgeLength_units = vecnorm(diff(route_units, 1, 1), 2, 2);
            seed = struct( ...
                'position_units', route_units, ...
                'tau', [0; cumsum(edgeLength_units)] / sum(edgeLength_units), ...
                'MaximumAlternatingIterations', snapshotProbeIterationLimit);
            attempt.SolverAttempted = true;
            [candidate, diagnostics, directMotion] = bmtpEngine.solve( ...
                seed, scene.regions_units, scene.coverage, ...
                request.initialState, request.goalState, request.limits, request.options, directMotion);
            candidateResult = obstacleAvoidance.planning.finalizeCandidate( ...
                scene.preparedObstacles, request, requestContext, graph, result.Attempts, ...
                result.ElapsedTime_s, false, struct(), candidate, route_units, diagnostics);

            attempt.IterationCount = readDiagnosticScalar( ...
                diagnostics, "IterationCount", 0);
            attempt.CandidateSuccess            = candidate.Success;
            attempt.OptimizerFeasible           = candidate.OptimizerFeasible;
            attempt.OptimizerIterateUnavailable = candidate.OptimizerIterateUnavailable;
            attempt.AlternativeGuideEligible = readLogicalField( ...
                candidate, "AlternativeGuideEligible");
            attempt.FailureStage = readStringField(candidate, "FailureStage");
            attempt.FailureKind  = readStringField(candidate, "FailureKind");
            attempt.Success = candidateResult.Success;

            if candidateResult.Success
                attempt.Selected = true;
                attempt.ElapsedTime_s = toc(attemptTimer);
                candidateResult.Attempts = [attempts; attempt];
                candidateResult.ElapsedTime_s = toc(totalTimer);
                result = candidateResult;
                return
            elseif candidate.Success
                % A public-validator rejection is a terminal defect, never a
                % reason to conceal the candidate behind another guide.
                attempt.ElapsedTime_s = toc(attemptTimer);
                candidateResult.Attempts = [attempts; attempt];
                candidateResult.ElapsedTime_s = toc(totalTimer);
                result = candidateResult;
                return
            elseif attempt.AlternativeGuideEligible
                attempt.FallbackEligible = true;
                result                   = candidateResult;
            else
                attempt.ElapsedTime_s = toc(attemptTimer);
                candidateResult.Attempts = [attempts; attempt];
                candidateResult.ElapsedTime_s = toc(totalTimer);
                result = candidateResult;
                return
            end
        end
        attempt.ElapsedTime_s = toc(attemptTimer);
        attempts(end + 1, 1)  = attempt; %#ok<AGROW>
    end

    % The timed fallback starts from a clean motion and graph record. It is
    % the only non-snapshot proposal and runs with the normal solver budget.
    priorElapsedTime_s = toc(totalTimer);
    [timedResult, timedAccepted, ~] = obstacleAvoidance.planning.tryTimedArrival( ...
        request, requestContext, scene.preparedObstacles, attempts, priorElapsedTime_s, directMotion);
    timedAttempt = obstacleAvoidance.planning.createAttemptRecord( ...
        numel(attempts) + 1, "timedVisibility");
    timedAttempt.GraphIsFullyEnumerated = false;
    timedAttempt.GraphConnected = timedResult.VisibilityGraph.IsConnected;
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
    timedAttempt.Success = timedAccepted;
    if timedAccepted
        timedAttempt.Selected = true;
    end
    timedAttempt.ElapsedTime_s = max(0, timedResult.ElapsedTime_s - priorElapsedTime_s);
    timedResult.Attempts       = [attempts; timedAttempt];
    result                     = timedResult;
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
