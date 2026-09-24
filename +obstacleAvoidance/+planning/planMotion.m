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

%% Section 2: Prepare Obstacle Geometry And The Empty Result

totalTimer          = tic;
isEarliestIntercept = ~isempty(request.goalState.targetMotion) && ...
    request.options.GoalTimeMode == "earliestArrival";
requestedInterval_s = [request.initialState.time_s, request.goalState.time_s];

planningEnvironment = struct('preparedObstacles', ...
    obstacleAvoidance.obstacles.prepareObstacles(obstacles, requestedInterval_s, true));

% Start with empty route and attempt fields so an obstacle or endpoint
% failure returns the usual result fields. Route search fills these in later.
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
result        = obstacleAvoidance.planning.createEmptyResult( ...
    planningEnvironment.preparedObstacles, request, visibilityGraph, emptyAttempts, 0);

% Check whether obstacle geometry can be treated as fixed for the whole request.
% Use time-dependent planning if an obstacle changes or its samples do not
% cover the full time from start to arrival.
obstaclesNeedTimedChecks = false;
for obstacleIndex = 1:numel(planningEnvironment.preparedObstacles)
    obstacle = planningEnvironment.preparedObstacles(obstacleIndex);
    historyHasMultipleSamples = numel(obstacle.time_s) > 1;
    requestExceedsHistory = historyHasMultipleSamples && ...
        (requestedInterval_s(1) < obstacle.time_s(1) || requestedInterval_s(2) > obstacle.time_s(end));
    obstacleChangesWithTime = ~obstacle.InternalPreparation.IsTimeInvariant || requestExceedsHistory;
    if obstacleChangesWithTime
        obstaclesNeedTimedChecks = true;
        break
    end
end

% Stop if the obstacle geometry cannot be used for collision checking
% between two recorded times. Check only times from start to arrival.
unsupportedMessage = obstacleAvoidance.planning.describeUnsupportedInterval( ...
    planningEnvironment.preparedObstacles, requestedInterval_s);
if strlength(unsupportedMessage) > 0
    % Stop before route search and return the failure with elapsed time.
    % This does not prove there is no route; the geometry could not be checked.
    result.Message                   = unsupportedMessage;
    result.TerminationReason         = "unsupportedObstacleInterpolation";
    result.Diagnostics.ElapsedTime_s = toc(totalTimer);
    return
end

%% Section 3: Check Endpoints And Prepare Route Search

% Get the obstacle shapes at the start time.
% Use this snapshot to build connections between obstacle vertices.
% Moving obstacles will also be checked over time during motion planning.
obstacleSnapshot = obstacleAvoidance.obstacles.snapshot( ...
    planningEnvironment.preparedObstacles, request.initialState.time_s, ~obstaclesNeedTimedChecks);

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
    % The key is a record of the inputs, not a new feasibility check.
    % Compare it with the saved record before trusting the earlier result.
    endpointValidationKey = struct();
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
    endpointsAreFeasible = parentRequest.EndpointValidation.Feasible;
    endpointMessage      = parentRequest.EndpointValidation.Message;
    endpointReason       = parentRequest.EndpointValidation.Reason;
else
    % Check the physical endpoints now. Valid input types alone do not prove
    % that a position is clear or that its velocity is within the limits.
    [endpointsAreFeasible, endpointMessage, endpointReason] = ...
        obstacleAvoidance.input.validatePlannerEndpoints( ...
        planningEnvironment.preparedObstacles, request.initialState, request.goalState, ...
        request.limits, request.options);
end

if ~endpointsAreFeasible
    % A failed endpoint check prevents planning. Passing this check still
    % does not guarantee that a complete route exists between the endpoints.
    result.Message                   = endpointMessage;
    result.TerminationReason         = endpointReason;
    result.Diagnostics.ElapsedTime_s = toc(totalTimer);
    return
end

% Find which obstacle corners can connect without crossing an obstacle.
% Reuse saved connections only when their inputs match. Start and goal
% are added later, when the planner searches for a complete route.
vertexVisibilityKey = struct( ...
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
        obstacleSnapshot, request.limits, request.options);
end

% Build the obstacle regions that the motion must avoid.
% For moving obstacles, store each region's time interval and ending shape.
% For static obstacles, use the same shapes for the whole trip.
if obstaclesNeedTimedChecks
    timedRegions = obstacleAvoidance.obstacles.createTimeCells( ...
        planningEnvironment.preparedObstacles, request.initialState.time_s, request.goalState.time_s);
    regions_units = timedRegions.Regions_units;
    % Keep the region count and timing information beside the regions so
    % BMTP knows which obstacle geometry applies during each part of the trip.
    if request.options.GoalTimeMode == "fixedArrival"
        coverage = struct( ...
            'ExactRegionCount',     numel(regions_units), ...
            'ActiveTimeInterval_s', timedRegions.ActiveTimeInterval_s, ...
            'EndRegions_units',     {timedRegions.EndRegions_units}, ...
            'BreakTime_s',          timedRegions.BreakTime_s);
    else
        coverage = struct( ...
            'ExactRegionCount',     numel(regions_units), ...
            'ActiveTimeInterval_s', timedRegions.ActiveTimeInterval_s, ...
            'EndRegions_units',     {timedRegions.EndRegions_units});
    end
    planningEnvironment = struct( ...
        'preparedObstacles', planningEnvironment.preparedObstacles, ...
        'snapshot',          obstacleSnapshot, ...
        'vertexVisibility',  vertexVisibility, ...
        'regions_units',     {regions_units}, ...
        'coverage',          coverage);
else
    % An obstacle may be split into several smaller regions. Collect all
    % of them into one list so BMTP checks every part of every obstacle.
    regionCount     = sum(arrayfun(@(obstacle) numel(obstacle.Regions_units), obstacleSnapshot));
    regions_units   = cell(regionCount, 1);
    nextRegionIndex = 1;
    for obstacleIndex = 1:numel(obstacleSnapshot)
        obstacleRegionCount             = numel(obstacleSnapshot(obstacleIndex).Regions_units);
        regionRowIndices                = nextRegionIndex:nextRegionIndex + obstacleRegionCount - 1;
        regions_units(regionRowIndices) = obstacleSnapshot(obstacleIndex).Regions_units;
        nextRegionIndex                 = nextRegionIndex + obstacleRegionCount;
    end
    planningEnvironment = struct( ...
        'preparedObstacles', planningEnvironment.preparedObstacles, ...
        'snapshot',          obstacleSnapshot, ...
        'vertexVisibility',  vertexVisibility, ...
        'regions_units',     {regions_units}, ...
        'coverage',          struct('ExactRegionCount', numel(regions_units)));
end

%% Section 4: Choose The Planning Method

endpointVelocityAndAcceleration = [request.initialState.velocity_units_s, ...
    request.initialState.acceleration_units_s2, ...
    request.goalState.velocity_units_s, request.goalState.acceleration_units_s2];
% Rest-to-rest means velocity = 0 and acceleration = 0 at both endpoints.
endpointsAreAtRest = all(endpointVelocityAndAcceleration == 0);

if request.options.GoalTimeMode == "earliestArrival"
    result = planEarliestArrival(result, planningEnvironment, request, totalTimer, ...
        obstaclesNeedTimedChecks, isEarliestIntercept, endpointsAreAtRest);
elseif obstaclesNeedTimedChecks
    result = planFixedArrivalDynamic(result, planningEnvironment, request, totalTimer);
else
    result = planFixedArrivalStatic(result, planningEnvironment, request, totalTimer);
end
end

%% Section 5: Local Functions

function result = planFixedArrivalStatic(result, planningEnvironment, request, totalTimer)
    % Static obstacles need one visibility route followed by BMTP and validation.
    motionGoalState = request.goalState;

    % For static obstacles, find a collision-free route between the endpoints.
    visibilityGraph = obstacleAvoidance.search.createVisibilityGraph( ...
        planningEnvironment.vertexVisibility, ...
        request.initialState.position_units, request.goalState.position_units);
    visibilityGraph.SearchKind = "initialSpatialSnapshot";
    result.Diagnostics.VisibilityGraph     = visibilityGraph;
    if ~visibilityGraph.IsConnected
        result.Message                   = "The initial visibility graph contains no start-to-goal route.";
        result.TerminationReason         = "noVisibilityRoute";
        result.Diagnostics.FailureStage  = "search";
        result.Diagnostics.FailureKind   = "noSpatialRoute";
        result.Diagnostics.ElapsedTime_s = toc(totalTimer);
        return
    end

    % Use route distance to define progress from 0 to 1 along the starting path.
    % Example: two equal-length edges give tau = [0; 0.5; 1]. These are not seconds.
    route_units      = visibilityGraph.Route_units;
    edgeLength_units = vecnorm(diff(route_units, 1, 1), 2, 2);
    startingPath     = struct('position_units', route_units, ...
        'tau', [0; cumsum(edgeLength_units)] / sum(edgeLength_units));
    motionRequest = struct( ...
        'initialState', request.initialState, ...
        'goalState',    motionGoalState, ...
        'limits',       request.limits, ...
        'options',      request.options);
    [motionCandidate, solverDiagnostics] = bmtpEngine.solve( ...
        startingPath, planningEnvironment, motionRequest, struct());

    % Assemble the returned motion and check it with the independent validator.
    result = obstacleAvoidance.planning.finalizeCandidate( ...
        planningEnvironment.preparedObstacles, request, visibilityGraph, result, ...
        motionCandidate, solverDiagnostics, struct());
    result.Diagnostics.ElapsedTime_s = toc(totalTimer);
end

function result = planEarliestArrival(result, planningEnvironment, request, totalTimer, ...
        obstaclesNeedTimedChecks, isEarliestIntercept, endpointsAreAtRest)
    % Choose the methods that apply to these obstacles and endpoint states.
    % Run them in order and keep the best motion that passes validation.
    % If the engine reports success but independent validation fails, stop.
    fixedGoalWithRestEndpoints = ~isEarliestIntercept && endpointsAreAtRest;
    availablePlanningMethods   = struct( ...
        'StaticSpatialBmtp',      ~obstaclesNeedTimedChecks && fixedGoalWithRestEndpoints, ...
        'DepartureFamily',        obstaclesNeedTimedChecks && fixedGoalWithRestEndpoints, ...
        'TimedVariableClockBmtp', obstaclesNeedTimedChecks && fixedGoalWithRestEndpoints, ...
        'ArrivalTimeTrials',      isEarliestIntercept || ~endpointsAreAtRest || obstaclesNeedTimedChecks);

    earliestPossibleArrival_s = NaN;
    if ~isEarliestIntercept
        earliestPossibleArrival_s = request.initialState.time_s + ...
            obstacleAvoidance.input.minimumTravelTime( ...
            request.initialState, request.goalState, request.limits);
    end
    % Keep the current result and a record of each method tried.
    % IsComplete tells the following stages whether planning should stop.
    arrivalPlanningProgress = struct( ...
        'Result',                result, ...
        'Attempts',              repmat(obstacleAvoidance.planning.createAttemptRecord(0, ""), 0, 1), ...
        'BestSoFarAttemptIndex', 0, ...
        'IsComplete',            false);

    % Choose one branch from the physical request. Save the available methods
    % in the result so the caller can see which methods apply.
    if isEarliestIntercept || ~endpointsAreAtRest
        arrivalPlanningProgress = runArrivalTimeTrialStage( ...
            arrivalPlanningProgress, planningEnvironment, request, totalTimer);
    elseif ~obstaclesNeedTimedChecks
        arrivalPlanningProgress = runStaticSpatialStage(arrivalPlanningProgress, planningEnvironment, request, ...
            earliestPossibleArrival_s);
    else
        % Moving obstacles and a fixed rest-to-rest goal use this sequence.
        % Each stage decides whether another method is needed and allowed.
        arrivalPlanningProgress = runDepartureStage(arrivalPlanningProgress, planningEnvironment, request, ...
            earliestPossibleArrival_s);
        if ~arrivalPlanningProgress.IsComplete
            arrivalPlanningProgress = runTimedSearchStage(arrivalPlanningProgress, planningEnvironment, request, ...
                totalTimer, earliestPossibleArrival_s);
        end
        if ~arrivalPlanningProgress.IsComplete
            arrivalPlanningProgress = runArrivalTimeTrialStage( ...
                arrivalPlanningProgress, planningEnvironment, request, totalTimer);
        end
    end
    result = finishEarliestArrival( ...
        arrivalPlanningProgress.Result, arrivalPlanningProgress.Attempts, ...
        availablePlanningMethods, totalTimer, earliestPossibleArrival_s);
end

function arrivalPlanningProgress = runStaticSpatialStage(arrivalPlanningProgress, planningEnvironment, request, ...
        earliestPossibleArrival_s)
    % With static obstacles, a fixed goal, and zero endpoint velocity and
    % acceleration, find one route and use BMTP to turn it into motion.
    % This branch ends after that attempt, whether it succeeds or fails.
    attemptTimer    = tic;
    result          = arrivalPlanningProgress.Result;
    visibilityGraph = obstacleAvoidance.search.createVisibilityGraph( ...
        planningEnvironment.vertexVisibility, ...
        request.initialState.position_units, request.goalState.position_units);
    visibilityGraph.SearchKind = "initialSpatialSnapshot";
    result.Diagnostics.VisibilityGraph = visibilityGraph;
    attempt = obstacleAvoidance.planning.createAttemptRecord(1, "spatialVisibility");
    attempt.EarliestPossibleArrival_s = earliestPossibleArrival_s;
    attempt.GraphConnected            = visibilityGraph.IsConnected;
    attempt.GraphIsFullyEnumerated    = visibilityGraph.GraphIsFullyEnumerated;
    attempt.RouteNodeCount            = size(visibilityGraph.Route_units, 1);
    attempt.RouteLength_units         = visibilityGraph.RouteLength_units;
    attempt.ExpandedCount             = visibilityGraph.ExpandedCount;
    if visibilityGraph.IsConnected
        route_units      = visibilityGraph.Route_units;
        edgeLength_units = vecnorm(diff(route_units, 1, 1), 2, 2);
        startingPath     = struct( ...
            'position_units', route_units, ...
            'tau',            [0; cumsum(edgeLength_units)] / sum(edgeLength_units));
        attempt.SolverAttempted = true;
        [motionCandidate, solverDiagnostics] = bmtpEngine.solve(startingPath, planningEnvironment, ...
            request, struct());
        result = obstacleAvoidance.planning.finalizeCandidate( ...
            planningEnvironment.preparedObstacles, request, visibilityGraph, result, ...
            motionCandidate, solverDiagnostics, struct());
        attempt          = populateMotionAttempt(attempt, result, motionCandidate, solverDiagnostics);
        attempt.Selected = result.Success;
    else
        attempt.FailureStage = "search";
        attempt.FailureKind  = "noSpatialRoute";
        result.Message                  = "The exhaustive static visibility graph has no route.";
        result.TerminationReason        = "noVisibilityRoute";
        result.Diagnostics.FailureStage = attempt.FailureStage;
        result.Diagnostics.FailureKind  = attempt.FailureKind;
    end
    attempt.ElapsedTime_s = toc(attemptTimer);
    arrivalPlanningProgress.Result = result;
    arrivalPlanningProgress.Attempts(end + 1, 1) = attempt;
    arrivalPlanningProgress.IsComplete = true;
end

function arrivalPlanningProgress = runDepartureStage(arrivalPlanningProgress, planningEnvironment, request, ...
        earliestPossibleArrival_s)
    % For moving obstacles and a fixed goal, first try the direct start-to-goal path.
    % This stage requires zero velocity and acceleration at both endpoints.
    % Keep a valid result; stop if no earlier arrival is physically possible.
    % Otherwise, later stages may find an earlier arrival.
    attemptTimer = tic;
    result       = arrivalPlanningProgress.Result;
    route_units  = [request.initialState.position_units; request.goalState.position_units];
    startingPath = struct('position_units', route_units, 'tau', [0; 1]);
    attempt      = obstacleAvoidance.planning.createAttemptRecord(1, "analyticDeparture");

    attempt.IsShortcut                = true;
    attempt.EarliestPossibleArrival_s = earliestPossibleArrival_s;
    attempt.GraphConnected            = true;
    attempt.RouteNodeCount            = 2;
    attempt.RouteLength_units         = norm(diff(route_units, 1, 1));
    attempt.SolverAttempted           = true;
    [motionCandidate, solverDiagnostics] = bmtpEngine.solve(startingPath, planningEnvironment, ...
        request, struct());
    visibilityGraph = result.Diagnostics.VisibilityGraph;
    visibilityGraph.SearchKind             = "c3DepartureSchedule";
    visibilityGraph.Route_units            = route_units;
    visibilityGraph.RouteLength_units      = attempt.RouteLength_units;
    visibilityGraph.IsConnected            = true;
    visibilityGraph.GraphIsFullyEnumerated = false;
    departureResult = obstacleAvoidance.planning.finalizeCandidate( ...
        planningEnvironment.preparedObstacles, request, visibilityGraph, result, ...
        motionCandidate, solverDiagnostics, struct());
    attempt               = populateMotionAttempt(attempt, departureResult, motionCandidate, solverDiagnostics);
    attempt.ElapsedTime_s = toc(attemptTimer);
    % A valid departure may be improved unless it is already proven earliest.
    independentValidationFailed = motionCandidate.Success && ~departureResult.Success;
    arrivalPlanningProgress     = applyEarliestAttempt(arrivalPlanningProgress, departureResult, attempt, ...
        independentValidationFailed, false, earliestPossibleArrival_s, ...
        request.options.ArrivalTimeTolerance_s);
end

function arrivalPlanningProgress = runTimedSearchStage(arrivalPlanningProgress, planningEnvironment, request, ...
        totalTimer, earliestPossibleArrival_s)
    % Search once using both position and time, allowing segment durations to vary.
    % If a valid motion exists, search only for an earlier arrival.
    % After selecting a timed result, try arrival times only in the unsampled
    % interval just before its goal window. Stop if a returned motion is invalid.
    % For other failures, nextMethodAllowed decides whether another method can run.
    hasValidatedMotion     = arrivalPlanningProgress.BestSoFarAttemptIndex > 0;
    arrivalTimeTolerance_s = request.options.ArrivalTimeTolerance_s;
    maximumArrivalTime_s   = request.goalState.time_s;

    if hasValidatedMotion
        maximumArrivalTime_s = min( ...
            maximumArrivalTime_s, arrivalPlanningProgress.Result.ArrivalTime_s - arrivalTimeTolerance_s);
    end
    if maximumArrivalTime_s <= request.initialState.time_s + arrivalTimeTolerance_s
        return
    end
    priorElapsedTime_s           = toc(totalTimer);
    [timedResult, timedAccepted] = obstacleAvoidance.planning.tryTimedArrival( ...
        request, planningEnvironment.preparedObstacles, arrivalPlanningProgress.Attempts, ...
        priorElapsedTime_s, struct(), maximumArrivalTime_s);
    attempt = createTimedAttemptRecord(numel(arrivalPlanningProgress.Attempts) + 1, ...
        timedResult, timedAccepted, priorElapsedTime_s);
    if timedAccepted
        attempt.CandidateArrival_s = timedResult.ArrivalTime_s;
    end
    attempt.EarliestPossibleArrival_s = earliestPossibleArrival_s;
    if hasValidatedMotion
        attempt.BestSoFarArrival_s = arrivalPlanningProgress.Result.ArrivalTime_s;
    end
    % An accepted timed search settles the choice between it and the best
    % earlier result. Independent-validation failure always stops the search.
    independentValidationFailed = string(timedResult.TerminationReason) == "invalidMotion";
    arrivalPlanningProgress     = applyEarliestAttempt(arrivalPlanningProgress, timedResult, attempt, ...
        independentValidationFailed, true, earliestPossibleArrival_s, arrivalTimeTolerance_s);

    % The timed search samples time in layers spaced by the horizon, so its
    % arrival can sit on the first layer of the goal's clear window. When
    % waiting at the goal from the layer before was not clear, no layer
    % sampled the times in between, and the goal may have cleared anywhere
    % there. Try the arrival-time grid in that one interval; the grid does not
    % move with the horizon. Example: a goal that clears at 16.3 s gives
    % layers 15 and 18 s for a 24 s horizon but 15 and 18.75 s for 30 s.
    % Both try 15.5, 16, ... s. The budget is MaxArrivalTrials, and a failed
    % or exhausted search keeps the timed motion (a validator rejection does not).
    timedMotionSelected = arrivalPlanningProgress.Attempts(end).Selected;
    if timedMotionSelected
        timedSearchDetails = timedResult.Diagnostics.VisibilityGraph.TimedSearch.TimedSearch;
        if isfinite(timedSearchDetails.GoalWindowPreviousLayerTime_s)
            unsampledInterval_s = [timedSearchDetails.GoalWindowPreviousLayerTime_s, ...
                timedSearchDetails.SelectedGoalWindowStartTime_s];
            resultBeforeTrials               = arrivalPlanningProgress.Result;
            resultBeforeTrials.Diagnostics.ElapsedTime_s = toc(totalTimer);
            arrivalPlanningProgress.Result   = obstacleAvoidance.planning.searchArrivalTimes( ...
                request, planningEnvironment, resultBeforeTrials, arrivalPlanningProgress.Attempts, ...
                request.options.MaxArrivalTrials, unsampledInterval_s);
            arrivalPlanningProgress.Attempts = arrivalPlanningProgress.Result.Diagnostics.Attempts;
        end
    end
end

function arrivalPlanningProgress = applyEarliestAttempt(arrivalPlanningProgress, attemptResult, attempt, ...
        independentValidationFailed, stopAfterValidMotion, earliestPossibleArrival_s, arrivalTimeTolerance_s)
    % Handle a completed departure or timed-search attempt in one place.
    % Keep valid motion, stop on validation failure, or decide whether to continue.
    hasValidatedMotion = arrivalPlanningProgress.BestSoFarAttemptIndex > 0;

    if attempt.Success
        if stopAfterValidMotion
            % The timed search is complete. Keep the earlier valid arrival,
            % treating times within tolerance as equal.
            existingMotionArrivesNoLater = hasValidatedMotion && ...
                arrivalPlanningProgress.Result.ArrivalTime_s <= attemptResult.ArrivalTime_s + arrivalTimeTolerance_s;
            if existingMotionArrivesNoLater
                arrivalPlanningProgress.Attempts(arrivalPlanningProgress.BestSoFarAttemptIndex).Selected = true;
            else
                attempt.Selected = true;
                arrivalPlanningProgress.Result = attemptResult;
            end
            arrivalPlanningProgress.IsComplete = true;
        else
            % A departure result becomes the best so far. More methods may
            % run unless it reaches the earliest physically possible arrival.
            attempt.BestSoFarArrival_s = attemptResult.ArrivalTime_s;
            attempt.Selected           = attemptResult.ArrivalTime_s <= ...
                earliestPossibleArrival_s + arrivalTimeTolerance_s;
            arrivalPlanningProgress.Result                = attemptResult;
            arrivalPlanningProgress.BestSoFarAttemptIndex = attempt.Index;
            arrivalPlanningProgress.IsComplete            = attempt.Selected;
        end
    elseif independentValidationFailed
        % Do not hide an invalid returned motion behind a previous valid one.
        arrivalPlanningProgress.Result     = attemptResult;
        arrivalPlanningProgress.IsComplete = true;
    else
        attempt.NextMethodAllowed  = ...
            obstacleAvoidance.planning.nextMethodAllowed(attemptResult);
        attempt.NextMethodReason   = attempt.FailureKind;
        attempt.NextAttemptAllowed = attempt.NextMethodAllowed;

        if hasValidatedMotion
            % A failed timed search must not erase a valid departure result.
            % Keep its failure details beside the motion that remains available.
            arrivalPlanningProgress.Result.Diagnostics.SolverDiagnostics.TimedSearchAttempt = struct( ...
                'AttemptIndex',                attempt.Index, ...
                'Success',                     false, ...
                'TerminationReason',           string(attemptResult.TerminationReason), ...
                'Message',                     string(attemptResult.Message), ...
                'FailureStage',                attempt.FailureStage, ...
                'FailureKind',                 attempt.FailureKind, ...
                'OptimizerIterateUnavailable', attempt.OptimizerIterateUnavailable, ...
                'NextMethodAllowed',           attempt.NextMethodAllowed, ...
                'VisibilityGraph',             attemptResult.Diagnostics.VisibilityGraph, ...
                'Route_units',                 attemptResult.Diagnostics.Route_units, ...
                'SolverDiagnostics',           attemptResult.Diagnostics.SolverDiagnostics);
            if ~attempt.NextMethodAllowed
                attempt.Message        = string(attemptResult.Message);
                attempt.SolverExitFlag = readDiagnosticScalar( ...
                    attemptResult.Diagnostics.SolverDiagnostics, "LastTrajectoryExitFlag", NaN);
                arrivalPlanningProgress.Attempts(arrivalPlanningProgress.BestSoFarAttemptIndex).Selected = true;
            end
        elseif stopAfterValidMotion || ~attempt.NextMethodAllowed
            % Record timed-search failure, or a departure failure that ends
            % the sequence. Otherwise, keep the initial result for the next stage.
            arrivalPlanningProgress.Result = attemptResult;
        end
        arrivalPlanningProgress.IsComplete = ~attempt.NextMethodAllowed;
    end

    arrivalPlanningProgress.Attempts(end + 1, 1) = attempt;
end

function arrivalPlanningProgress = runArrivalTimeTrialStage( ...
    arrivalPlanningProgress, planningEnvironment, request, totalTimer)
    % Try candidate arrival times for moving targets or nonzero endpoint motion.
    % If a valid motion already exists, limit extra trials to
    % BestSoFarRefinementTrialLimit; 0 keeps the existing motion without more trials.
    maximumArrivalTrials = request.options.MaxArrivalTrials;
    if arrivalPlanningProgress.BestSoFarAttemptIndex > 0
        maximumArrivalTrials = min(maximumArrivalTrials, request.options.BestSoFarRefinementTrialLimit);
        if maximumArrivalTrials == 0
            arrivalPlanningProgress.Attempts(arrivalPlanningProgress.BestSoFarAttemptIndex).Selected = true;
            arrivalPlanningProgress.IsComplete = true;
            return
        end
    end
    resultBeforeTrials               = arrivalPlanningProgress.Result;
    resultBeforeTrials.Diagnostics.ElapsedTime_s = toc(totalTimer);
    arrivalPlanningProgress.Result     = obstacleAvoidance.planning.searchArrivalTimes( ...
        request, planningEnvironment, resultBeforeTrials, arrivalPlanningProgress.Attempts, maximumArrivalTrials);
    arrivalPlanningProgress.Attempts   = arrivalPlanningProgress.Result.Diagnostics.Attempts;
    arrivalPlanningProgress.IsComplete = true;
end

function attempt = populateMotionAttempt(attempt, attemptResult, motionCandidate, solverDiagnostics)
    % Copy the solver outcome into the planner's attempt record.
    attempt.IterationLimit              = readDiagnosticScalar( ...
        solverDiagnostics, "MaximumAlternatingIterations", NaN);
    attempt.IterationCount              = readDiagnosticScalar(solverDiagnostics, "IterationCount", 0);
    attempt.CandidateSuccess            = motionCandidate.Success;
    attempt.OptimizerFeasible           = readLogicalField(motionCandidate, "OptimizerFeasible");
    attempt.OptimizerIterateUnavailable = readLogicalField( ...
        motionCandidate, "OptimizerIterateUnavailable");
    attempt.AlternativeGuideEligible    = readLogicalField( ...
        motionCandidate, "AlternativeGuideEligible");
    attempt.FailureStage                = readStringField(motionCandidate, "FailureStage");
    attempt.FailureKind                 = readStringField(motionCandidate, "FailureKind");
    attempt.Success                     = attemptResult.Success;
    if isfield(attemptResult, 'ArrivalTime_s') && ...
            isnumeric(attemptResult.ArrivalTime_s) && ...
            isscalar(attemptResult.ArrivalTime_s) && ...
            isfinite(attemptResult.ArrivalTime_s)
        attempt.CandidateArrival_s = attemptResult.ArrivalTime_s;
    end
end

function attempt = createTimedAttemptRecord(attemptIndex, timedResult, timedAccepted, ...
        priorElapsedTime_s)
    % Record this position-and-time search using its own result and elapsed time.
    attempt = obstacleAvoidance.planning.createAttemptRecord(attemptIndex, "timedVisibility");
    attempt.GraphIsFullyEnumerated      = false;
    attempt.GraphConnected              = timedResult.Diagnostics.VisibilityGraph.IsConnected;
    attempt.RouteNodeCount              = size(timedResult.Diagnostics.Route_units, 1);
    attempt.RouteLength_units           = timedResult.Diagnostics.VisibilityGraph.RouteLength_units;
    attempt.ExpandedCount               = timedResult.Diagnostics.VisibilityGraph.ExpandedCount;
    attempt.SolverAttempted             = attempt.GraphConnected;
    attempt.IterationLimit              = readDiagnosticScalar( ...
        timedResult.Diagnostics.SolverDiagnostics, "MaximumAlternatingIterations", NaN);
    attempt.IterationCount              = readDiagnosticScalar( ...
        timedResult.Diagnostics.SolverDiagnostics, "IterationCount", 0);
    attempt.CandidateSuccess            = readLogicalField( ...
        timedResult.Diagnostics.SolverDiagnostics, "Accepted");
    attempt.OptimizerFeasible           = readLogicalField(timedResult.Diagnostics, "OptimizerFeasible");
    attempt.OptimizerIterateUnavailable = readLogicalField( ...
        timedResult.Diagnostics, "OptimizerIterateUnavailable");
    attempt.AlternativeGuideEligible    = readLogicalField( ...
        timedResult.Diagnostics, "AlternativeGuideEligible");
    attempt.FailureStage                = readStringField(timedResult.Diagnostics, "FailureStage");
    attempt.FailureKind                 = readStringField(timedResult.Diagnostics, "FailureKind");
    attempt.Success                     = timedAccepted;
    attempt.ElapsedTime_s               = max(0, timedResult.Diagnostics.ElapsedTime_s - priorElapsedTime_s);
end

function result = finishEarliestArrival(result, attempts, availablePlanningMethods, totalTimer, ...
        earliestPossibleArrival_s)
    % Record which attempt was selected and whether an earlier arrival is
    % still possible. Finding a valid motion does not prove it is the fastest.
    result.Diagnostics.Attempts      = attempts;
    result.Diagnostics.ElapsedTime_s = toc(totalTimer);
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
        result.ArrivalTime_s <= earliestPossibleArrival_s + result.Options.ArrivalTimeTolerance_s;
    unsearchedInterval_s = [NaN, NaN];
    if result.Success && ~globalEarliestProven && isfinite(earliestPossibleArrival_s)
        unsearchedUpperTime_s = result.ArrivalTime_s - result.Options.ArrivalTimeTolerance_s;
        if unsearchedUpperTime_s > earliestPossibleArrival_s
            unsearchedInterval_s = [earliestPossibleArrival_s, unsearchedUpperTime_s];
        end
    end
    arrivalTimeSearchUsed = false;
    if ~isempty(attempts)
        arrivalTimeSearchUsed = any([attempts.Kind] == "arrivalTimeTrial");
    end
    result.Diagnostics.EarliestArrival = struct( ...
        'Capabilities',              availablePlanningMethods, ...
        'EarliestPossibleArrival_s', earliestPossibleArrival_s, ...
        'BestSoFarArrival_s',        bestSoFarArrivalTime_s, ...
        'SelectedAttemptIndex',      selectedAttemptIndex, ...
        'GlobalEarliestProven',      globalEarliestProven, ...
        'UnsearchedInterval_s',      unsearchedInterval_s, ...
        'ArrivalTimeSearchUsed',     arrivalTimeSearchUsed, ...
        'AttemptCount',              numel(attempts));
    if isfield(result.Diagnostics, 'TemporalSearch')
        result.Diagnostics.TemporalSearch.GlobalEarliestProven = globalEarliestProven;
        result.Diagnostics.TemporalSearch.SelectedAttemptIndex = selectedAttemptIndex;
    end
end

function result = planFixedArrivalDynamic(result, planningEnvironment, request, totalTimer)
    % Use obstacle shapes at the start and arrival times to suggest routes.
    % BMTP still checks motion against the obstacles over time.
    % If these attempts fail and another method is allowed, search in both
    % position and time. Failed snapshot routes do not prove that no route exists.
    snapshotProbeIterationLimit = request.options.SpatialProbeIterationLimit;
    snapshotKinds              = ["initialSpatialSnapshot", "arrivalSpatialSnapshot"];
    snapshotTimes_s             = [request.initialState.time_s, request.goalState.time_s];

    attempts           = repmat(obstacleAvoidance.planning.createAttemptRecord(0, ""), 0, 1);
    previousRoute_units = zeros(0, 2);
    directMotion       = struct();

    for snapshotIndex = 1:numel(snapshotKinds)
        attemptTimer = tic;
        attempt      = obstacleAvoidance.planning.createAttemptRecord( ...
            snapshotIndex, "spatialVisibility");
        attempt.IsShortcut          = true;
        attempt.IterationLimit      = snapshotProbeIterationLimit;
        attempt.GraphSnapshotTime_s = snapshotTimes_s(snapshotIndex);

        vertexVisibility = planningEnvironment.vertexVisibility;
        if snapshotIndex == 2
            arrivalSnapshot = obstacleAvoidance.obstacles.snapshot( ...
                planningEnvironment.preparedObstacles, request.goalState.time_s, false);
            vertexVisibility = obstacleAvoidance.search.createVertexVisibility( ...
                arrivalSnapshot, request.limits, request.options);
        end
        visibilityGraph = obstacleAvoidance.search.createVisibilityGraph( ...
            vertexVisibility, request.initialState.position_units, request.goalState.position_units);
        visibilityGraph.SearchKind = snapshotKinds(snapshotIndex);
        result.Diagnostics.VisibilityGraph         = visibilityGraph;
        attempt.GraphConnected         = visibilityGraph.IsConnected;
        attempt.GraphIsFullyEnumerated = visibilityGraph.GraphIsFullyEnumerated;
        attempt.RouteNodeCount         = size(visibilityGraph.Route_units, 1);
        attempt.RouteLength_units      = visibilityGraph.RouteLength_units;
        attempt.ExpandedCount          = visibilityGraph.ExpandedCount;

        if ~visibilityGraph.IsConnected
            attempt.NextAttemptAllowed = true;
        elseif ~isempty(previousRoute_units) && isequaln(visibilityGraph.Route_units, previousRoute_units)
            % Skip a repeated route: BMTP receives the same path and full
            % obstacle motion, even though the snapshot time is different.
            attempt.NextAttemptAllowed = true;
        else
            route_units         = visibilityGraph.Route_units;
            previousRoute_units = route_units;
            edgeLength_units    = vecnorm(diff(route_units, 1, 1), 2, 2);
            startingPath        = struct( ...
                'position_units',               route_units, ...
                'tau',                          [0; cumsum(edgeLength_units)] / sum(edgeLength_units), ...
                'MaximumAlternatingIterations', snapshotProbeIterationLimit);
            attempt.SolverAttempted = true;
            [motionCandidate, solverDiagnostics, directMotion] = bmtpEngine.solve( ...
                startingPath, planningEnvironment, request, directMotion);
            attemptResult = obstacleAvoidance.planning.finalizeCandidate( ...
                planningEnvironment.preparedObstacles, request, visibilityGraph, result, ...
                motionCandidate, solverDiagnostics, struct());

            attempt.IterationCount              = readDiagnosticScalar( ...
                solverDiagnostics, "IterationCount", 0);
            attempt.CandidateSuccess            = motionCandidate.Success;
            attempt.OptimizerFeasible           = motionCandidate.OptimizerFeasible;
            attempt.OptimizerIterateUnavailable = motionCandidate.OptimizerIterateUnavailable;
            attempt.AlternativeGuideEligible    = readLogicalField( ...
                motionCandidate, "AlternativeGuideEligible");
            attempt.FailureStage                = readStringField(motionCandidate, "FailureStage");
            attempt.FailureKind                 = readStringField(motionCandidate, "FailureKind");
            attempt.Success                     = attemptResult.Success;

            if attemptResult.Success
                attempt.Selected      = true;
                attempt.ElapsedTime_s = toc(attemptTimer);
                attemptResult.Diagnostics.Attempts      = [attempts; attempt];
                attemptResult.Diagnostics.ElapsedTime_s = toc(totalTimer);
                result = attemptResult;
                return
            elseif motionCandidate.Success
                % The engine reported success, but independent validation failed.
                % Return that failure instead of trying another route.
                attempt.ElapsedTime_s = toc(attemptTimer);
                attemptResult.Diagnostics.Attempts      = [attempts; attempt];
                attemptResult.Diagnostics.ElapsedTime_s = toc(totalTimer);
                result = attemptResult;
                return
            elseif attempt.AlternativeGuideEligible
                attempt.NextAttemptAllowed = true;
                result = attemptResult;
            else
                attempt.ElapsedTime_s = toc(attemptTimer);
                attemptResult.Diagnostics.Attempts      = [attempts; attempt];
                attemptResult.Diagnostics.ElapsedTime_s = toc(totalTimer);
                result = attemptResult;
                return
            end
        end
        attempt.ElapsedTime_s = toc(attemptTimer);
        attempts(end + 1, 1)  = attempt; %#ok<AGROW>
    end

    % Search using obstacle positions over time with the normal solver budget.
    % This attempt creates its own motion and graph result.
    priorElapsedTime_s              = toc(totalTimer);
    [timedResult, timedAccepted, ~] = obstacleAvoidance.planning.tryTimedArrival( ...
        request, planningEnvironment.preparedObstacles, attempts, priorElapsedTime_s, directMotion, ...
        request.goalState.time_s);
    timedAttempt = createTimedAttemptRecord(numel(attempts) + 1, ...
        timedResult, timedAccepted, priorElapsedTime_s);
    timedAttempt.Selected = timedAccepted;
    timedResult.Diagnostics.Attempts  = [attempts; timedAttempt];
    result                = timedResult;
end

function fieldValue = readLogicalField(resultRecord, fieldName)
    % Read an optional true/false field; use false when the field is missing.
    fieldValue = false;
    if isstruct(resultRecord) && isscalar(resultRecord) && isfield(resultRecord, fieldName)
        fieldValue = logical(resultRecord.(fieldName));
    end
end

function fieldValue = readStringField(resultRecord, fieldName)
    % Read an optional text field; use empty text when the field is missing.
    fieldValue = "";
    if isstruct(resultRecord) && isscalar(resultRecord) && isfield(resultRecord, fieldName)
        fieldValue = string(resultRecord.(fieldName));
    end
end

function fieldValue = readDiagnosticScalar(resultRecord, fieldName, defaultValue)
    % Read one finite numeric value; use the supplied default if it is unavailable.
    fieldValue = defaultValue;
    if isstruct(resultRecord) && isscalar(resultRecord) && isfield(resultRecord, fieldName) && ...
            isnumeric(resultRecord.(fieldName)) && isscalar(resultRecord.(fieldName)) && ...
            isfinite(resultRecord.(fieldName))
        fieldValue = double(resultRecord.(fieldName));
    end
end
