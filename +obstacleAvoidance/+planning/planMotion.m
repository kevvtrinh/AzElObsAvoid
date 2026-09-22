function result = planMotion(request)
%% Section 0: Header & Readme
% SYNTAX
%   result = obstacleAvoidance.planning.planMotion(request)
%**************************************************************************
% PURPOSE
%   - Prepare obstacles, find a route, and use BMTP to generate smooth motion.
%     Accept the motion only after it passes independent validation.
%**************************************************************************
% INPUTS
%   - request (scalar struct)
%       Checked states, limits, and options from prepareRequest, with obstacles,
%       originalInputs, and parentRequest. Wrapped coordinates must already
%       be converted to an unwrapped request by planWrappedMotion.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Stable planner result. Expected planning failure returns
%       Success = false; invalid input throws an error.
%**************************************************************************
% UNITS
%   - Positions are coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Read The Prepared Request

obstacles     = request.obstacles;
parentRequest = request.parentRequest;

%% Section 2: Prepare Obstacles And Check Endpoints

totalTimer          = tic;
earliestTarget      = ~isempty(request.goalState.targetMotion) && ...
    request.options.GoalTimeMode == "earliestArrival";
requestedInterval_s = [request.initialState.time_s, request.goalState.time_s];

planningEnvironment = struct('preparedObstacles', ...
    obstacleAvoidance.obstacles.prepareObstacles(obstacles, requestedInterval_s, true));

% Initialize the visiblity graph fields
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
emptyAttempts   = repmat(obstacleAvoidance.planning.createAttemptRecord(0, ""), 0, 1);
result          = obstacleAvoidance.planning.createEmptyResult(planningEnvironment.preparedObstacles, request, visibilityGraph, emptyAttempts, 0);

% Check whether obstacle geometry can be treated as fixed for the whole request.
% Use time-dependent planning if an obstacle changes or its samples do not
% cover the full time from start to arrival.
isDynamic = false;
for obstacleIndex = 1:numel(planningEnvironment.preparedObstacles)
    obstacle                  = planningEnvironment.preparedObstacles(obstacleIndex);
    historyHasMultipleSamples = numel(obstacle.time_s) > 1;
    requestExceedsHistory     = historyHasMultipleSamples && ...
        (requestedInterval_s(1) < obstacle.time_s(1) || requestedInterval_s(2) > obstacle.time_s(end));
    obstacleChangesWithTime   = ~obstacle.InternalPreparation.IsTimeInvariant || requestExceedsHistory;
    if obstacleChangesWithTime
        isDynamic = true;
        break
    end
end

% Stop if the obstacle geometry cannot be used for collision checking
% between two recorded times. Check only times from start to arrival.
unsupportedObstacleIndex = [];
for obstacleIndex = 1:numel(planningEnvironment.preparedObstacles)
    preparation    = planningEnvironment.preparedObstacles(obstacleIndex).InternalPreparation;
    obstacleTime_s = planningEnvironment.preparedObstacles(obstacleIndex).time_s;

    intervalIsUnsupported   = preparation.IntervalPrepared & preparation.IntervalIsUnsupported;
    intervalOverlapsRequest = obstacleTime_s(1:end - 1) < requestedInterval_s(2) & ...
        obstacleTime_s(2:end) > requestedInterval_s(1);

    unsupportedIntervalIndex = find(intervalIsUnsupported & intervalOverlapsRequest, 1);
    if ~isempty(unsupportedIntervalIndex)
        unsupportedObstacleIndex = obstacleIndex;
        break
    end
end

if ~isempty(unsupportedObstacleIndex)
    % Preparation found obstacle geometry it cannot use for collision checks.
    % Get the obstacle name and the two sample times around the problem.
    intervalTime_s = obstacleTime_s(unsupportedIntervalIndex:unsupportedIntervalIndex + 1);
    obstacleName   = string(planningEnvironment.preparedObstacles(unsupportedObstacleIndex).targetName);

    % Tell the caller which obstacle and time interval prevented planning.
    result.Message = sprintf(['Obstacle %d ("%s"), interval [%g, %g] s, has no ' ...
        'proven exact continuous interpolation.'], ...
        unsupportedObstacleIndex, obstacleName, ...
        intervalTime_s(1), intervalTime_s(2));

    % Include a more specific explanation if preparation recorded one.
    hasProofReason = isfield(preparation, 'IntervalProofReason');
    if hasProofReason
        proofReason = preparation.IntervalProofReason(unsupportedIntervalIndex);
        if proofReason == "movingCellsExcludeProtectedSample"
            % The calculated region misses part of a supplied obstacle shape
            % with its safety margin, so it cannot safely represent that shape.
            result.Message = result.Message + ...
                " The given moving-cell margin-square enclosure excludes " + ...
                "supplied protected sample area.";
        end
    end

    % Stop before route search and return the failure with elapsed time.
    % This does not prove there is no route; the geometry could not be checked.
    result.TerminationReason = "unsupportedObstacleInterpolation";
    result.ElapsedTime_s     = toc(totalTimer);
    return
end

%% Section 3: Check Endpoints And Prepare Route Search

% Get the obstacle shapes at the start time.
% Use this snapshot to build connections between obstacle vertices.
% Moving obstacles will also be checked over time during motion planning.
snapshot = obstacleAvoidance.obstacles.snapshot(planningEnvironment.preparedObstacles, request.initialState.time_s, ~isDynamic);

% Check the start and goal against obstacles, motion limits, and available time.
% A trial may have passed these checks already. Reuse that result only when
% every input to the check is unchanged; otherwise, run the checks again.
reuseEndpointValidation = false;
if ~isempty(parentRequest) && parentRequest.EndpointValidation.Feasible
    % Collect the options and endpoint values used by the earlier check.
    endpointValidationOptions = struct( ...
        'GoalTimeMode',           request.options.GoalTimeMode, ...
        'WrapX',                  request.options.WrapX, ...
        'WrapY',                  request.options.WrapY, ...
        'ArrivalTimeTolerance_s', request.options.ArrivalTimeTolerance_s);
    initialEndpointState      = struct( ...
        'time_s',                request.initialState.time_s, ...
        'position_units',        request.initialState.position_units, ...
        'velocity_units_s',      request.initialState.velocity_units_s, ...
        'acceleration_units_s2', request.initialState.acceleration_units_s2);
    goalEndpointState         = struct( ...
        'time_s',                request.goalState.time_s, ...
        'position_units',        request.goalState.position_units, ...
        'velocity_units_s',      request.goalState.velocity_units_s, ...
        'acceleration_units_s2', request.goalState.acceleration_units_s2);
    % The key is a record of the inputs, not a new feasibility check.
    % Compare it with the saved record before trusting the earlier result.
    endpointValidationKey     = struct();
    endpointValidationKey.PreparedObstacles = planningEnvironment.preparedObstacles;
    endpointValidationKey.RequestHorizon_s  = requestedInterval_s;
    endpointValidationKey.InitialState      = initialEndpointState;
    endpointValidationKey.GoalState         = goalEndpointState;
    endpointValidationKey.Limits            = request.limits;
    endpointValidationKey.Options           = endpointValidationOptions;
    reuseEndpointValidation = isequaln(parentRequest.EndpointValidation.Key, endpointValidationKey);
end

if reuseEndpointValidation
    % These exact inputs already passed, so use the saved result.
    endpointFeasible = parentRequest.EndpointValidation.Feasible;
    endpointMessage  = parentRequest.EndpointValidation.Message;
    endpointReason   = parentRequest.EndpointValidation.Reason;
else
    % Check the physical endpoints now. Valid input types alone do not prove
    % that a position is clear or that its velocity is within the limits.
    [endpointFeasible, endpointMessage, endpointReason] = ...
        obstacleAvoidance.input.validatePlannerEndpoints( ...
        planningEnvironment.preparedObstacles, request.initialState, request.goalState, ...
        request.limits, request.options);
end

if ~endpointFeasible
    % A failed endpoint check prevents planning. Passing this check still
    % does not guarantee that a complete route exists between the endpoints.
    result.Message           = endpointMessage;
    result.TerminationReason = endpointReason;
    result.ElapsedTime_s     = toc(totalTimer);
    return
end

% Find which obstacle corners can connect without crossing an obstacle.
% Reuse saved connections only when their inputs match. Start and goal
% are added later, when the planner searches for a complete route.
vertexVisibilityKey   = struct( ...
    'PreparedObstacles',   {planningEnvironment.preparedObstacles}, ...
    'SnapshotTime_s',      request.initialState.time_s, ...
    'Limits',              request.limits, ...
    'ConstraintTolerance', request.options.ConstraintTolerance);
reuseVertexVisibility = ~isempty(parentRequest) && ...
    isequaln(parentRequest.InitialVertexVisibility.Key, vertexVisibilityKey);
if reuseVertexVisibility
    vertexVisibility = parentRequest.InitialVertexVisibility.VertexVisibility;
else
    vertexVisibility = obstacleAvoidance.search.createVertexVisibility( ...
        snapshot, request.limits, request.options);
end

% Build the obstacle regions that the motion must avoid.
% For moving obstacles, store each region's time interval and ending shape.
% For static obstacles, use the same shapes for the whole trip.
if isDynamic
    cells         = obstacleAvoidance.obstacles.createTimeCells( ...
        planningEnvironment.preparedObstacles, request.initialState.time_s, request.goalState.time_s);
    regions_units = cells.Regions_units;
    % Keep the region count and timing information beside the regions so
    % BMTP knows which obstacle geometry applies during each part of the trip.
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
    planningEnvironment = struct( ...
        'preparedObstacles', planningEnvironment.preparedObstacles, ...
        'snapshot',          snapshot, ...
        'vertexVisibility',  vertexVisibility, ...
        'regions_units',     {regions_units}, ...
        'coverage',          coverage);
else
    % An obstacle may be split into several smaller regions. Collect all
    % of them into one list so BMTP checks every part of every obstacle.
    regionCount     = sum(arrayfun(@(obstacle) numel(obstacle.Regions_units), snapshot));
    regions_units   = cell(regionCount, 1);
    nextRegionIndex = 1;
    for obstacleIndex = 1:numel(snapshot)
        obstacleRegionCount          = numel(snapshot(obstacleIndex).Regions_units);
        targetIndices                = nextRegionIndex:nextRegionIndex + obstacleRegionCount - 1;
        regions_units(targetIndices) = snapshot(obstacleIndex).Regions_units;
        nextRegionIndex              = nextRegionIndex + obstacleRegionCount;
    end
    planningEnvironment = struct( ...
        'preparedObstacles', planningEnvironment.preparedObstacles, ...
        'snapshot',          snapshot, ...
        'vertexVisibility',  vertexVisibility, ...
        'regions_units',     {regions_units}, ...
        'coverage',          struct('ExactRegionCount', numel(regions_units)));
end

%% Section 4: Plan Earliest-Arrival Motion

endpointDerivatives = [request.initialState.velocity_units_s, ...
    request.initialState.acceleration_units_s2, ...
    request.goalState.velocity_units_s, request.goalState.acceleration_units_s2];
% Rest-to-rest means velocity = 0 and acceleration = 0 at both endpoints.
isRest = all(endpointDerivatives == 0);

if request.options.GoalTimeMode == "earliestArrival"
    result = planEarliestArrival(result, planningEnvironment, request, totalTimer, ...
        isDynamic, earliestTarget, isRest);
    return
end

%% Section 5: Plan Motion At The Fixed Arrival Time

motionGoalState     = request.goalState;
fixedArrivalDynamic = isDynamic && request.options.GoalTimeMode == "fixedArrival";

if fixedArrivalDynamic
    result = planFixedArrivalDynamic(result, planningEnvironment, request, totalTimer);
    return
end

% For static obstacles, find a collision-free route between the endpoints.
visibilityGraph = getVisibilityGraph( ...
    planningEnvironment.vertexVisibility, request.initialState.position_units, request.goalState.position_units, ...
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

% Use route distance to define progress from 0 to 1 along the starting path.
% Example: two equal-length edges give tau = [0; 0.5; 1]. These are not seconds.
route_units      = visibilityGraph.Route_units;
edgeLength_units = vecnorm(diff(route_units, 1, 1), 2, 2);
seed             = struct('position_units', route_units, ...
    'tau', [0; cumsum(edgeLength_units)] / sum(edgeLength_units));
[candidate, solverDiagnostics] = bmtpEngine.solve(seed, planningEnvironment, ...
    struct('initialState', request.initialState, ...
    'goalState', motionGoalState, ...
    'limits', request.limits, ...
    'options', request.options), struct());

% Assemble the returned motion and check it with the independent validator.
result = obstacleAvoidance.planning.finalizeCandidate( ...
    planningEnvironment.preparedObstacles, request, visibilityGraph, result, ...
    candidate, solverDiagnostics, struct());
result.ElapsedTime_s = toc(totalTimer);
end

%% Section 6: Local Functions

function graph = getVisibilityGraph(vertexVisibility, start_units, goal_units, kind)
    % Add start and goal to the saved obstacle-vertex connections, then find a route.
    graph = obstacleAvoidance.search.createVisibilityGraph(vertexVisibility, start_units, goal_units);
    graph.SearchKind = kind;
end

function result = planEarliestArrival(result, planningEnvironment, request, totalTimer, ...
        isDynamic, earliestTarget, isRest)
    % Choose the methods that apply to these obstacles and endpoint states.
    % Run them in order and keep the best motion that passes validation.
    % If the engine reports success but independent validation fails, stop.
    fixedRestGoal = ~earliestTarget && isRest;
    capabilities  = struct( ...
        'StaticSpatialBmtp',       ~isDynamic && fixedRestGoal, ...
        'DepartureFamily',         isDynamic && fixedRestGoal, ...
        'TimedVariableClockBmtp',  isDynamic && fixedRestGoal, ...
        'ArrivalTimeTrials',       earliestTarget || ~isRest || isDynamic);

    earliestPossibleArrival_s = NaN;
    if ~earliestTarget
        earliestPossibleArrival_s = request.initialState.time_s + ...
            obstacleAvoidance.input.minimumTravelTime( ...
            request.initialState, request.goalState, request.limits);
    end
    % Keep the current result and a record of each method tried.
    % Done tells the following stages whether the search should stop.
    search = struct( ...
        'Result',                result, ...
        'Attempts',              repmat(obstacleAvoidance.planning.createAttemptRecord(0, ""), 0, 1), ...
        'BestSoFarAttemptIndex', 0, ...
        'Done',                  false);

    if capabilities.StaticSpatialBmtp
        search = runStaticSpatialStage(search, planningEnvironment, request, ...
            earliestPossibleArrival_s);
    end
    if ~search.Done && capabilities.DepartureFamily
        search = runDepartureStage(search, planningEnvironment, request, ...
            earliestPossibleArrival_s);
    end
    if ~search.Done && capabilities.TimedVariableClockBmtp
        search = runTimedSearchStage(search, planningEnvironment, request, ...
            totalTimer, earliestPossibleArrival_s);
    end
    if ~search.Done && capabilities.ArrivalTimeTrials
        search = runArrivalTimeTrialStage(search, planningEnvironment, request, totalTimer);
    end
    result = finishEarliestArrival(search.Result, search.Attempts, capabilities, ...
        totalTimer, earliestPossibleArrival_s);
end

function search = runStaticSpatialStage(search, planningEnvironment, request, ...
        earliestPossibleArrival_s)
    % With static obstacles, a fixed goal, and zero endpoint velocity and
    % acceleration, find one route and use BMTP to turn it into motion.
    % This branch ends after that attempt, whether it succeeds or fails.
    attemptTimer = tic;
    result       = search.Result;
    graph        = getVisibilityGraph(planningEnvironment.vertexVisibility, request.initialState.position_units, ...
        request.goalState.position_units, "initialSpatialSnapshot");
    result.VisibilityGraph = graph;
    attempt = obstacleAvoidance.planning.createAttemptRecord(1, "spatialVisibility");
    attempt.EarliestPossibleArrival_s = earliestPossibleArrival_s;
    attempt.GraphConnected            = graph.IsConnected;
    attempt.GraphIsFullyEnumerated    = graph.GraphIsFullyEnumerated;
    attempt.RouteNodeCount            = size(graph.Route_units, 1);
    attempt.RouteLength_units         = graph.RouteLength_units;
    attempt.ExpandedCount             = graph.ExpandedCount;
    if graph.IsConnected
        route_units      = graph.Route_units;
        edgeLength_units = vecnorm(diff(route_units, 1, 1), 2, 2);
        seed             = struct( ...
            'position_units', route_units, ...
            'tau', [0; cumsum(edgeLength_units)] / sum(edgeLength_units));
        attempt.SolverAttempted = true;
        [candidate, diagnostics] = bmtpEngine.solve(seed, planningEnvironment, ...
            request, struct());
        result  = obstacleAvoidance.planning.finalizeCandidate( ...
            planningEnvironment.preparedObstacles, request, graph, result, ...
            candidate, diagnostics, struct());
        attempt = populateMotionAttempt(attempt, result, candidate, diagnostics);
        attempt.Selected = result.Success;
    else
        attempt.FailureStage = "search";
        attempt.FailureKind  = "noSpatialRoute";
        result.Message           = "The exhaustive static visibility graph has no route.";
        result.TerminationReason = "noVisibilityRoute";
        result.FailureStage      = attempt.FailureStage;
        result.FailureKind       = attempt.FailureKind;
    end
    attempt.ElapsedTime_s       = toc(attemptTimer);
    search.Result               = result;
    search.Attempts(end + 1, 1) = attempt;
    search.Done                 = true;
end

function search = runDepartureStage(search, planningEnvironment, request, ...
        earliestPossibleArrival_s)
    % For moving obstacles and a fixed goal, first try the direct start-to-goal path.
    % This stage requires zero velocity and acceleration at both endpoints.
    % Keep a valid result; stop if no earlier arrival is physically possible.
    % Otherwise, later stages may find an earlier arrival.
    attemptTimer = tic;
    result       = search.Result;
    route_units  = [request.initialState.position_units; request.goalState.position_units];
    seed         = struct('position_units', route_units, 'tau', [0; 1]);
    attempt      = obstacleAvoidance.planning.createAttemptRecord(1, "analyticDeparture");
    attempt.IsShortcut                = true;
    attempt.EarliestPossibleArrival_s = earliestPossibleArrival_s;
    attempt.GraphConnected            = true;
    attempt.RouteNodeCount            = 2;
    attempt.RouteLength_units         = norm(diff(route_units, 1, 1));
    attempt.SolverAttempted           = true;
    [candidate, diagnostics] = bmtpEngine.solve(seed, planningEnvironment, ...
        request, struct());
    graph = result.VisibilityGraph;
    graph.SearchKind             = "c3DepartureSchedule";
    graph.Route_units            = route_units;
    graph.RouteLength_units      = attempt.RouteLength_units;
    graph.IsConnected            = true;
    graph.GraphIsFullyEnumerated = false;
    departureResult = obstacleAvoidance.planning.finalizeCandidate( ...
        planningEnvironment.preparedObstacles, request, graph, result, ...
        candidate, diagnostics, struct());
    attempt         = populateMotionAttempt(attempt, departureResult, candidate, diagnostics);
    attempt.ElapsedTime_s = toc(attemptTimer);
    if departureResult.Success
        attempt.BestSoFarArrival_s = departureResult.ArrivalTime_s;
        attempt.Selected           = departureResult.ArrivalTime_s <= ...
            earliestPossibleArrival_s + request.options.ArrivalTimeTolerance_s;
        search.Result                = departureResult;
        search.BestSoFarAttemptIndex = 1;
        search.Done                  = attempt.Selected;
    elseif candidate.Success
        % The engine accepted a motion the public validator rejected.
        search.Result = departureResult;
        search.Done   = true;
    else
        attempt.NextMethodAllowed  = ...
            obstacleAvoidance.planning.nextMethodAllowed(departureResult);
        attempt.NextMethodReason   = attempt.FailureKind;
        attempt.NextAttemptAllowed = attempt.NextMethodAllowed;
        if ~attempt.NextMethodAllowed
            search.Result = departureResult;
        end
        search.Done = ~attempt.NextMethodAllowed;
    end
    search.Attempts(end + 1, 1) = attempt;
end

function search = runTimedSearchStage(search, planningEnvironment, request, ...
        totalTimer, earliestPossibleArrival_s)
    % Search once using both position and time, allowing segment durations to vary.
    % If a valid motion exists, search only for an earlier arrival.
    % Stop after selecting a valid result, or if a returned motion is invalid.
    % For other failures, nextMethodAllowed decides whether another method can run.
    hasBestSoFar         = search.BestSoFarAttemptIndex > 0;
    tolerance_s          = request.options.ArrivalTimeTolerance_s;
    maximumArrivalTime_s = request.goalState.time_s;

    if hasBestSoFar
        maximumArrivalTime_s = min(maximumArrivalTime_s, search.Result.ArrivalTime_s - tolerance_s);
    end
    if maximumArrivalTime_s <= request.initialState.time_s + tolerance_s
        return
    end
    priorElapsedTime_s = toc(totalTimer);
    [timedResult, timedAccepted] = obstacleAvoidance.planning.tryTimedArrival( ...
        request, planningEnvironment.preparedObstacles, search.Attempts, ...
        priorElapsedTime_s, struct(), maximumArrivalTime_s);
    attempt = createTimedAttemptRecord(numel(search.Attempts) + 1, ...
        timedResult, timedAccepted, priorElapsedTime_s);
    attempt.EarliestPossibleArrival_s = earliestPossibleArrival_s;
    if hasBestSoFar
        attempt.BestSoFarArrival_s = search.Result.ArrivalTime_s;
    end
    if timedAccepted
        bestSoFarIsNoLater = hasBestSoFar && ...
            search.Result.ArrivalTime_s <= timedResult.ArrivalTime_s + tolerance_s;
        if bestSoFarIsNoLater
            search.Attempts(search.BestSoFarAttemptIndex).Selected = true;
        else
            attempt.Selected = true;
            search.Result    = timedResult;
        end
        search.Done = true;
    elseif string(timedResult.TerminationReason) == "invalidMotion"
        search.Result = timedResult;
        search.Done   = true;
    else
        attempt.NextMethodAllowed  = ...
            obstacleAvoidance.planning.nextMethodAllowed(timedResult);
        attempt.NextMethodReason   = attempt.FailureKind;
        attempt.NextAttemptAllowed = attempt.NextMethodAllowed;
        if hasBestSoFar
            search.Result.SolverDiagnostics.TimedSearchAttempt = struct( ...
                'AttemptIndex',                  attempt.Index, ...
                'Success',                       false, ...
                'TerminationReason',             string(timedResult.TerminationReason), ...
                'Message',                       string(timedResult.Message), ...
                'FailureStage',                  attempt.FailureStage, ...
                'FailureKind',                   attempt.FailureKind, ...
                'OptimizerIterateUnavailable',   attempt.OptimizerIterateUnavailable, ...
                'NextMethodAllowed',        attempt.NextMethodAllowed, ...
                'VisibilityGraph',               timedResult.VisibilityGraph, ...
                'Route_units',                   timedResult.Route_units, ...
                'SolverDiagnostics',             timedResult.SolverDiagnostics);
            if ~attempt.NextMethodAllowed
                attempt.Message        = string(timedResult.Message);
                attempt.SolverExitFlag = readDiagnosticScalar( ...
                    timedResult.SolverDiagnostics, "LastTrajectoryExitFlag", NaN);
                search.Attempts(search.BestSoFarAttemptIndex).Selected = true;
            end
        else
            search.Result = timedResult;
        end
        search.Done = ~attempt.NextMethodAllowed;
    end
    search.Attempts(end + 1, 1) = attempt;
end

function search = runArrivalTimeTrialStage(search, planningEnvironment, request, totalTimer)
    % Try candidate arrival times for moving targets or nonzero endpoint motion.
    % If a valid motion already exists, limit extra trials to
    % BestSoFarRefinementTrialLimit; 0 keeps the existing motion without more trials.
    trialLimit = request.options.MaxArrivalTrials;
    if search.BestSoFarAttemptIndex > 0
        trialLimit = min(trialLimit, request.options.BestSoFarRefinementTrialLimit);
        if trialLimit == 0
            search.Attempts(search.BestSoFarAttemptIndex).Selected = true;
            search.Done = true;
            return
        end
    end
    searchBase               = search.Result;
    searchBase.ElapsedTime_s = toc(totalTimer);
    search.Result   = obstacleAvoidance.planning.searchArrivalTimes( ...
        request, planningEnvironment, searchBase, search.Attempts, trialLimit);
    search.Attempts = search.Result.Attempts;
    search.Done     = true;
end

function attempt = populateMotionAttempt(attempt, candidateResult, candidate, diagnostics)
    % Copy the solver outcome into the planner's attempt record.
    attempt.IterationLimit              = readDiagnosticScalar( ...
        diagnostics, "MaximumAlternatingIterations", NaN);
    attempt.IterationCount              = readDiagnosticScalar(diagnostics, "IterationCount", 0);
    attempt.CandidateSuccess            = candidate.Success;
    attempt.OptimizerFeasible           = readLogicalField(candidate, "OptimizerFeasible");
    attempt.OptimizerIterateUnavailable = readLogicalField( ...
        candidate, "OptimizerIterateUnavailable");
    attempt.AlternativeGuideEligible    = readLogicalField( ...
        candidate, "AlternativeGuideEligible");
    attempt.FailureStage                = readStringField(candidate, "FailureStage");
    attempt.FailureKind                 = readStringField(candidate, "FailureKind");
    attempt.Success                     = candidateResult.Success;
    if isfield(candidateResult, 'ArrivalTime_s') && ...
            isnumeric(candidateResult.ArrivalTime_s) && ...
            isscalar(candidateResult.ArrivalTime_s) && ...
            isfinite(candidateResult.ArrivalTime_s)
        attempt.CandidateArrival_s = candidateResult.ArrivalTime_s;
    end
end

function attempt = createTimedAttemptRecord(index, timedResult, timedAccepted, ...
        priorElapsedTime_s)
    % Record this position-and-time search using its own result and elapsed time.
    attempt = obstacleAvoidance.planning.createAttemptRecord(index, "timedVisibility");
    attempt.GraphIsFullyEnumerated      = false;
    attempt.GraphConnected              = timedResult.VisibilityGraph.IsConnected;
    attempt.RouteNodeCount              = size(timedResult.Route_units, 1);
    attempt.RouteLength_units           = timedResult.VisibilityGraph.RouteLength_units;
    attempt.ExpandedCount               = timedResult.VisibilityGraph.ExpandedCount;
    attempt.SolverAttempted             = attempt.GraphConnected;
    attempt.IterationLimit              = readDiagnosticScalar( ...
        timedResult.SolverDiagnostics, "MaximumAlternatingIterations", NaN);
    attempt.IterationCount              = readDiagnosticScalar( ...
        timedResult.SolverDiagnostics, "IterationCount", 0);
    attempt.CandidateSuccess            = readLogicalField( ...
        timedResult.SolverDiagnostics, "Accepted");
    attempt.OptimizerFeasible           = readLogicalField(timedResult, "OptimizerFeasible");
    attempt.OptimizerIterateUnavailable = readLogicalField( ...
        timedResult, "OptimizerIterateUnavailable");
    attempt.AlternativeGuideEligible    = readLogicalField( ...
        timedResult, "AlternativeGuideEligible");
    attempt.FailureStage                = readStringField(timedResult, "FailureStage");
    attempt.FailureKind                 = readStringField(timedResult, "FailureKind");
    attempt.Success                     = timedAccepted;
    if timedAccepted
        attempt.CandidateArrival_s = timedResult.ArrivalTime_s;
    end
    attempt.ElapsedTime_s = max(0, timedResult.ElapsedTime_s - priorElapsedTime_s);
end

function result = finishEarliestArrival(result, attempts, capabilities, totalTimer, ...
        earliestPossibleArrival_s)
    % Record which attempt was selected and whether an earlier arrival is
    % still possible. Finding a valid motion does not prove it is the fastest.
    result.Attempts      = attempts;
    result.ElapsedTime_s = toc(totalTimer);
    selectedAttemptIndex = find([attempts.Selected], 1, 'last');
    if isempty(selectedAttemptIndex)
        selectedAttemptIndex = 0;
    end
    bestSoFarArrivalTime_s = NaN;
    if selectedAttemptIndex > 0 && ...
            isfinite(attempts(selectedAttemptIndex).CandidateArrival_s)
        bestSoFarArrivalTime_s = attempts(selectedAttemptIndex).CandidateArrival_s;
    end
    % Compare the selected arrival with the earliest time allowed by the
    % physical limits. Treat them as equal within the arrival-time tolerance.
    globalEarliestProven = result.Success && isfinite(earliestPossibleArrival_s) && ...
        result.ArrivalTime_s <= earliestPossibleArrival_s + ...
        result.Options.ArrivalTimeTolerance_s;
    unsearchedInterval_s = [NaN, NaN];
    if result.Success && ~globalEarliestProven && isfinite(earliestPossibleArrival_s)
        unsearchedUpperTime_s = result.ArrivalTime_s - ...
            result.Options.ArrivalTimeTolerance_s;
        if unsearchedUpperTime_s > earliestPossibleArrival_s
            unsearchedInterval_s = [earliestPossibleArrival_s, unsearchedUpperTime_s];
        end
    end
    arrivalTimeSearchUsed = false;
    if ~isempty(attempts)
        arrivalTimeSearchUsed = any([attempts.Kind] == ...
            "arrivalTimeTrial");
    end
    result.EarliestArrival = struct( ...
        'Capabilities',             capabilities, ...
        'EarliestPossibleArrival_s',  earliestPossibleArrival_s, ...
        'BestSoFarArrival_s',       bestSoFarArrivalTime_s, ...
        'SelectedAttemptIndex',     selectedAttemptIndex, ...
        'GlobalEarliestProven',     globalEarliestProven, ...
        'UnsearchedInterval_s',     unsearchedInterval_s, ...
        'ArrivalTimeSearchUsed',    arrivalTimeSearchUsed, ...
        'AttemptCount',             numel(attempts));
    if isfield(result, 'TemporalSearch')
        result.TemporalSearch.GlobalEarliestProven = globalEarliestProven;
        result.TemporalSearch.SelectedAttemptIndex = selectedAttemptIndex;
    end
end

function result = planFixedArrivalDynamic(result, planningEnvironment, request, totalTimer)
    % Use obstacle shapes at the start and arrival times to suggest routes.
    % BMTP still checks motion against the obstacles over time.
    % If these attempts fail and another method is allowed, search in both
    % position and time. Failed snapshot routes do not prove that no route exists.
    snapshotProbeIterationLimit = request.options.SpatialProbeIterationLimit;
    snapshotKinds               = ["initialSpatialSnapshot", "arrivalSpatialSnapshot"];
    snapshotTimes_s             = [request.initialState.time_s, request.goalState.time_s];
    attempts                    = repmat(obstacleAvoidance.planning.createAttemptRecord(0, ""), 0, 1);
    previousRoute_units         = zeros(0, 2);
    directMotion                = struct();

    for snapshotIndex = 1:numel(snapshotKinds)
        attemptTimer = tic;
        attempt      = obstacleAvoidance.planning.createAttemptRecord( ...
            snapshotIndex, "spatialVisibility");
        attempt.IsShortcut          = true;
        attempt.IterationLimit      = snapshotProbeIterationLimit;
        attempt.GraphSnapshotTime_s = snapshotTimes_s(snapshotIndex);

        vertexVisibility = planningEnvironment.vertexVisibility;
        if snapshotIndex == 2
            arrivalSnapshot  = obstacleAvoidance.obstacles.snapshot( ...
                planningEnvironment.preparedObstacles, request.goalState.time_s, false);
            vertexVisibility = obstacleAvoidance.search.createVertexVisibility( ...
                arrivalSnapshot, request.limits, request.options);
        end
        graph = getVisibilityGraph(vertexVisibility, request.initialState.position_units, ...
            request.goalState.position_units, snapshotKinds(snapshotIndex));
        result.VisibilityGraph         = graph;
        attempt.GraphConnected         = graph.IsConnected;
        attempt.GraphIsFullyEnumerated = graph.GraphIsFullyEnumerated;
        attempt.RouteNodeCount         = size(graph.Route_units, 1);
        attempt.RouteLength_units      = graph.RouteLength_units;
        attempt.ExpandedCount          = graph.ExpandedCount;

        if ~graph.IsConnected
            attempt.NextAttemptAllowed = true;
        elseif ~isempty(previousRoute_units) && isequaln(graph.Route_units, previousRoute_units)
            % Skip a repeated route: BMTP receives the same path and full
            % obstacle motion, even though the snapshot time is different.
            attempt.NextAttemptAllowed = true;
        else
            route_units         = graph.Route_units;
            previousRoute_units = route_units;
            edgeLength_units    = vecnorm(diff(route_units, 1, 1), 2, 2);
            seed                = struct( ...
                'position_units', route_units, ...
                'tau', [0; cumsum(edgeLength_units)] / sum(edgeLength_units), ...
                'MaximumAlternatingIterations', snapshotProbeIterationLimit);
            attempt.SolverAttempted = true;
            [candidate, diagnostics, directMotion] = bmtpEngine.solve(seed, planningEnvironment, ...
                request, directMotion);
            candidateResult = obstacleAvoidance.planning.finalizeCandidate( ...
                planningEnvironment.preparedObstacles, request, graph, result, ...
                candidate, diagnostics, struct());

            attempt.IterationCount              = readDiagnosticScalar( ...
                diagnostics, "IterationCount", 0);
            attempt.CandidateSuccess            = candidate.Success;
            attempt.OptimizerFeasible           = candidate.OptimizerFeasible;
            attempt.OptimizerIterateUnavailable = candidate.OptimizerIterateUnavailable;
            attempt.AlternativeGuideEligible    = readLogicalField( ...
                candidate, "AlternativeGuideEligible");
            attempt.FailureStage                = readStringField(candidate, "FailureStage");
            attempt.FailureKind                 = readStringField(candidate, "FailureKind");
            attempt.Success                     = candidateResult.Success;

            if candidateResult.Success
                attempt.Selected      = true;
                attempt.ElapsedTime_s = toc(attemptTimer);
                candidateResult.Attempts      = [attempts; attempt];
                candidateResult.ElapsedTime_s = toc(totalTimer);
                result = candidateResult;
                return
            elseif candidate.Success
                % The engine reported success, but independent validation failed.
                % Return that failure instead of trying another route.
                attempt.ElapsedTime_s = toc(attemptTimer);
                candidateResult.Attempts      = [attempts; attempt];
                candidateResult.ElapsedTime_s = toc(totalTimer);
                result = candidateResult;
                return
            elseif attempt.AlternativeGuideEligible
                attempt.NextAttemptAllowed = true;
                result                   = candidateResult;
            else
                attempt.ElapsedTime_s = toc(attemptTimer);
                candidateResult.Attempts      = [attempts; attempt];
                candidateResult.ElapsedTime_s = toc(totalTimer);
                result = candidateResult;
                return
            end
        end
        attempt.ElapsedTime_s = toc(attemptTimer);
        attempts(end + 1, 1)  = attempt; %#ok<AGROW>
    end

    % Search using obstacle positions over time with the normal solver budget.
    % This attempt creates its own motion and graph result.
    priorElapsedTime_s = toc(totalTimer);
    [timedResult, timedAccepted, ~] = obstacleAvoidance.planning.tryTimedArrival( ...
        request, planningEnvironment.preparedObstacles, attempts, priorElapsedTime_s, directMotion, ...
        request.goalState.time_s);
    timedAttempt = obstacleAvoidance.planning.createAttemptRecord( ...
        numel(attempts) + 1, "timedVisibility");
    timedAttempt.GraphIsFullyEnumerated      = false;
    timedAttempt.GraphConnected              = timedResult.VisibilityGraph.IsConnected;
    timedAttempt.RouteNodeCount              = size(timedResult.Route_units, 1);
    timedAttempt.RouteLength_units           = timedResult.VisibilityGraph.RouteLength_units;
    timedAttempt.ExpandedCount               = timedResult.VisibilityGraph.ExpandedCount;
    timedAttempt.SolverAttempted             = timedAttempt.GraphConnected;
    timedAttempt.IterationLimit              = readDiagnosticScalar( ...
        timedResult.SolverDiagnostics, "MaximumAlternatingIterations", NaN);
    timedAttempt.IterationCount              = readDiagnosticScalar( ...
        timedResult.SolverDiagnostics, "IterationCount", 0);
    timedAttempt.CandidateSuccess            = readLogicalField( ...
        timedResult.SolverDiagnostics, "Accepted");
    timedAttempt.OptimizerFeasible           = readLogicalField(timedResult, "OptimizerFeasible");
    timedAttempt.OptimizerIterateUnavailable = readLogicalField( ...
        timedResult, "OptimizerIterateUnavailable");
    timedAttempt.AlternativeGuideEligible    = readLogicalField( ...
        timedResult, "AlternativeGuideEligible");
    timedAttempt.FailureStage                = readStringField(timedResult, "FailureStage");
    timedAttempt.FailureKind                 = readStringField(timedResult, "FailureKind");
    timedAttempt.Success                     = timedAccepted;
    if timedAccepted
        timedAttempt.Selected = true;
    end
    timedAttempt.ElapsedTime_s = max(0, timedResult.ElapsedTime_s - priorElapsedTime_s);
    timedResult.Attempts       = [attempts; timedAttempt];
    result                     = timedResult;
end

function value = readLogicalField(record, name)
    % Read an optional true/false field; use false when the field is missing.
    value = false;
    if isstruct(record) && isscalar(record) && isfield(record, name)
        value = logical(record.(name));
    end
end

function value = readStringField(record, name)
    % Read an optional text field; use empty text when the field is missing.
    value = "";
    if isstruct(record) && isscalar(record) && isfield(record, name)
        value = string(record.(name));
    end
end

function value = readDiagnosticScalar(record, name, defaultValue)
    % Read one finite numeric value; use the supplied default if it is unavailable.
    value = defaultValue;
    if isstruct(record) && isscalar(record) && isfield(record, name) && ...
            isnumeric(record.(name)) && isscalar(record.(name)) && isfinite(record.(name))
        value = double(record.(name));
    end
end
