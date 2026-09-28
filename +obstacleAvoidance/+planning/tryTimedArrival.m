function [result, accepted, directMotion] = tryTimedArrival( ...
        request, preparedObstacles, attempts, elapsedTime_s, ...
        directMotion, maximumArrivalTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   [result, accepted, directMotion] = obstacleAvoidance.planning.tryTimedArrival( ...
%       request, preparedObstacles, attempts, elapsedTime_s, ...
%       directMotion, maximumArrivalTime_s)
%**************************************************************************
% PURPOSE
%   - Find a route with a time at each point, then use BMTP to turn that
%     route into motion and check it with the independent validator.
%   - Support a required arrival time, or search for an earlier arrival
%     at a fixed position when the vehicle starts and finishes at rest.
%**************************************************************************
% INPUTS
%   - request (scalar struct)
%       Normalized initial state, goal state, limits, and resolved options.
%   - preparedObstacles (struct array)
%       Prepared obstacle geometry owned by the normalized request.
%   - attempts (struct array)
%       Planner-level attempt history to retain.
%   - elapsedTime_s (nonnegative scalar)
%       Planner time accumulated before this timed attempt.
%   - directMotion (scalar struct)
%       Saved direct-motion calculation and checks, or struct() when no
%       direct motion has been calculated for this request.
%   - maximumArrivalTime_s (finite scalar)
%       Latest time to search for an earliest arrival, capped at the request's
%       goal time. Fixed-arrival requests always use their required goal time.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Independently validated motion or a diagnostic failure with
%       Success = false. Invalid input throws an error.
%   - accepted (logical scalar)
%       True only when the timed candidate passes validation.
%   - directMotion (scalar struct)
%       Saved direct motion and checks, including any calculated by this call.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Initialize The Result And Check The Arrival Requirements

totalTimer   = tic;
accepted     = false;
initialState = request.initialState;
goalState    = request.goalState;

% Prepare the usual result fields before searching. If no route is found,
% the caller still receives the request, attempts, and reason for stopping.
timedVisibilityGraph = struct( ...
    'NodePosition_units',     zeros(0, 2), ...
    'AcceptedNodeIndex',      zeros(0, 2), ...
    'RejectedNodeIndex',      zeros(0, 2), ...
    'Route_units',            zeros(0, 2), ...
    'RouteTime_s',            zeros(0, 1), ...
    'RouteLength_units',      Inf, ...
    'IsConnected',            false, ...
    'ExpandedCount',          0, ...
    'GraphIsFullyEnumerated', false, ...
    'SearchKind',             "timeExpandedVisibilityGraph", ...
    'TimedSearch',            struct());
result = obstacleAvoidance.planning.createEmptyResult( ...
    preparedObstacles, request, timedVisibilityGraph, attempts, elapsedTime_s);
result.Message    = "The timed search has not completed.";
result.Diagnostics.Validation = struct( ...
    "Passed", false, "Message", "No timed motion is available.");
result.MotionLength_units                          = Inf;
result.Diagnostics.IntegratedSquaredJerk_units2_s5 = Inf;
result.Diagnostics.MaximumConstraintViolation      = Inf;
result.Diagnostics.OptimizerFeasible               = false;
result.Diagnostics.OptimizerIterateUnavailable     = false;
result.Diagnostics.AlternativeGuideEligible        = false;
result.Diagnostics.FailureStage                    = "notRun";
result.Diagnostics.FailureKind                     = "notRun";

% An earlier successful method may already give us an arrival time to beat.
% Shorten the search to that time only when arrival time is allowed to vary.
validateattributes(maximumArrivalTime_s, {'numeric'}, ...
    {'real', 'finite', 'scalar', '>', initialState.time_s});
maximumArrivalTime_s = min(maximumArrivalTime_s, goalState.time_s);
if request.options.GoalTimeMode == "earliestArrival"
    goalState.time_s = maximumArrivalTime_s;
end

% With a moving target or nonzero endpoint velocity or acceleration, changing
% arrival time needs separate fixed-time trials. This search cannot vary it.
endpointDerivativeValues = [initialState.velocity_units_s(:); initialState.acceleration_units_s2(:); ...
    goalState.velocity_units_s(:); goalState.acceleration_units_s2(:)];
endpointsAreAtRest             = all(endpointDerivativeValues == 0);
arrivalIsFixed                 = request.options.GoalTimeMode == "fixedArrival";
arrivalTimeSearchIsUnsupported = ~arrivalIsFixed && ...
    (~isempty(goalState.targetMotion) || ~endpointsAreAtRest);
if arrivalTimeSearchIsUnsupported
    result.Message = "Free-arrival timed visibility requires a fixed-position goal " + ...
        "with zero endpoint velocity and acceleration.";
    result.TerminationReason         = "unsupportedTimedRequest";
    result.Diagnostics.ElapsedTime_s = elapsedTime_s + toc(totalTimer);
    return
end

%% Section 2: Find A Route And A Time For Each Point

% Search positions and times together: an obstacle may block a connection
% at one time and move away later. BMTP will check whether the vehicle can
% follow the proposed route within its speed, acceleration, and jerk limits.
searchTimer = tic;

[route_units, routeTime_s, timedRouteDetails] = obstacleAvoidance.search.createTimedRouteProposal( ...
    preparedObstacles, initialState, goalState, request.originalInputs.requestedLimits, request.options);
timedRouteDetails.ElapsedTime_s = toc(searchTimer);

result.Diagnostics.VisibilityGraph.NodePosition_units = timedRouteDetails.Nodes_units;
result.Diagnostics.VisibilityGraph.TimedSearch        = timedRouteDetails;
result.Diagnostics.VisibilityGraph.ExpandedCount      = timedRouteDetails.TimedSearch.ExpandedCount;
if isempty(routeTime_s)
    result.Message                   = "No route reached the requested goal layer in the discrete timed graph.";
    result.TerminationReason         = "noTimedRoute";
    result.Diagnostics.FailureStage  = "search";
    result.Diagnostics.FailureKind   = "noTimedRoute";
    result.Diagnostics.ElapsedTime_s = elapsedTime_s + toc(totalTimer);
    return
end
if size(route_units, 1) > 2
    % A wait needs only its start and end times. For example, the same position
    % at t = 0, 1, and 2 s needs only t = 0 and 2 s. This preserves the wait
    % without adding motion segments for the intermediate time samples.
    segmentMoves         = any(diff(route_units, 1, 1) ~= 0, 2);
    routePointIsRetained = [true; segmentMoves(1:end - 1) | segmentMoves(2:end); true];
    route_units          = route_units(routePointIsRetained, :);
    routeTime_s          = routeTime_s(routePointIsRetained);
end
result.Diagnostics.VisibilityGraph.RouteTime_s       = routeTime_s;
result.Diagnostics.VisibilityGraph.Route_units       = route_units;
result.Diagnostics.VisibilityGraph.RouteLength_units = sum(vecnorm(diff(route_units), 2, 2));
result.Diagnostics.VisibilityGraph.IsConnected       = true;
result.Diagnostics.Route_units                       = route_units;

%% Section 3: Turn The Route And Its Times Into BMTP Motion

% For earliest arrival, BMTP may vary arrival within a time range when the
% goal position stays clear. The range must extend beyond the route's arrival
% by more than the time tolerance; otherwise, keep that one arrival time.
% A fixed-arrival request always keeps its required time.
timedSearchDetails         = timedRouteDetails.TimedSearch;
arrivalCanVaryWithinWindow = ~arrivalIsFixed;
if arrivalCanVaryWithinWindow
    hasGoalArrivalWindow       = isfield(timedSearchDetails, 'SelectedGoalWindowEndTime_s');
    arrivalCanVaryWithinWindow = hasGoalArrivalWindow && timedSearchDetails.SelectedGoalWindowEndTime_s > ...
        routeTime_s(end) + request.options.ArrivalTimeTolerance_s;
end
routeDuration_s = routeTime_s(end) - initialState.time_s;

% Keep the time assigned to each route point when creating the starting
% path. tau = 0 is the motion start and tau = 1 is the route's planned arrival.
normalizedRouteTime = (routeTime_s - initialState.time_s) / routeDuration_s;
startingPath        = struct( ...
    'position_units',       route_units, ...
    'tau',                  normalizedRouteTime, ...
    'UsesVariableClock',    false, ...
    'UsesTimeScopedSolver', false);
motionGoalState = goalState;
motionOptions   = request.options;
if arrivalCanVaryWithinWindow
    motionGoalState.time_s     = timedSearchDetails.SelectedGoalWindowEndTime_s;
    motionOptions.GoalTimeMode = "earliestArrival";

    % Arrival must be inside the clear time range and allow at least the
    % minimum travel time required by the vehicle's motion limits.
    minimumArrivalTime_s = max(timedSearchDetails.SelectedGoalWindowStartTime_s, ...
        timedSearchDetails.MinimumGoalArrivalTime_s);
    [obstacleRegions_units, obstacleTimeCoverage] = createTimedCoverage( ...
        preparedObstacles, initialState.time_s, motionGoalState.time_s);
    obstacleTimeCoverage.MinimumMotionDuration_s = minimumArrivalTime_s - initialState.time_s;
    obstacleTimeCoverage.SeedMotionDuration_s    = routeDuration_s;
    startingPath.UsesVariableClock               = true;
else
    motionGoalState.time_s     = routeTime_s(end);
    motionOptions.GoalTimeMode = "fixedArrival";

    [obstacleRegions_units, obstacleTimeCoverage] = createTimedCoverage( ...
        preparedObstacles, initialState.time_s, motionGoalState.time_s);
    startingPath.UsesTimeScopedSolver = true;
end

planningEnvironment = struct( ...
    'regions_units', {obstacleRegions_units}, ...
    'coverage',      obstacleTimeCoverage);
motionRequest = struct( ...
    'initialState', initialState, ...
    'goalState',    motionGoalState, ...
    'limits',       request.originalInputs.requestedLimits, ...
    'options',      motionOptions);
[motionCandidate, solverDiagnostics, directMotion] = bmtpEngine.solve( ...
    startingPath, planningEnvironment, motionRequest, directMotion);
if ~motionCandidate.Success
    result = obstacleAvoidance.planning.finalizeCandidate( ...
        preparedObstacles, request, result.Diagnostics.VisibilityGraph, result, ...
        motionCandidate, solverDiagnostics, struct());
    result.TerminationReason = "timedMotionInfeasible";
    result.Message           = "The timed route did not produce a feasible BMTP motion: " + ...
        motionCandidate.Message;
    result.Diagnostics.ElapsedTime_s = elapsedTime_s + toc(totalTimer);
    return
end

%% Section 4: Assemble And Independently Validate The Result

% Give the validator the same arrival requirements used by BMTP. It checks
% the actual motion times, not just the times proposed by the route search.
validationTimingFields = struct();
if arrivalCanVaryWithinWindow
    validationTimingFields.GoalArrivalWindow_s = ...
        [minimumArrivalTime_s, motionGoalState.time_s];
    % Retain obstacle coverage through the end of the allowed arrival range,
    % even when the returned motion reaches the goal earlier.
    validationTimingFields.TrajectoryCoverageEndTime_s = motionGoalState.time_s;
else
    % Store the required arrival time. The validator compares it with
    % the actual arrival time in the returned motion.
    validationTimingFields.FixedArrivalTrialTime_s = motionGoalState.time_s;
end
result = obstacleAvoidance.planning.finalizeCandidate( ...
    preparedObstacles, request, result.Diagnostics.VisibilityGraph, result, ...
    motionCandidate, solverDiagnostics, validationTimingFields);
result.Diagnostics.ElapsedTime_s = elapsedTime_s + toc(totalTimer);
accepted             = result.Success;
if ~accepted
    return
end
if arrivalIsFixed
    result.Message = "The given goal layer produced an independently validated timed BMTP motion.";
elseif arrivalCanVaryWithinWindow
    result.Message = "The first reachable goal window produced an independently validated free-clock BMTP motion.";
else
    result.Message = "The earliest reachable timed-route layer produced an independently validated BMTP motion.";
end
result.TerminationReason = "goalReached";
minimumTravelTime_s      = obstacleAvoidance.input.minimumTravelTime( ...
    initialState, goalState, request.limits);
earliestPossibleArrival_s = initialState.time_s + minimumTravelTime_s;

% Finding motion from selected route points and times does not prove that
% no earlier motion exists, so GlobalEarliestProven remains false.
result.Diagnostics.TemporalSearch = struct( ...
    'Resolution_s',              request.options.TemporalResolution_s, ...
    'TrialTime_s',               motionCandidate.ArrivalTime_s, ...
    'GlobalEarliestProven',      false, ...
    'EarliestPossibleArrival_s', earliestPossibleArrival_s, ...
    'BestSoFarArrival_s',        NaN, ...
    'BestSoFar',                 false);
result.Diagnostics.ElapsedTime_s = elapsedTime_s + toc(totalTimer);
end

%% Section 5: Local Functions

function [obstacleRegions_units, obstacleTimeCoverage] = createTimedCoverage(obstacles, startTime_s, finishTime_s)
    % Collect the obstacle regions and the times when each region applies.
    % BMTP uses these to check motion against moving obstacles.
    obstacleTimeCells     = obstacleAvoidance.obstacles.createTimeCells(obstacles, startTime_s, finishTime_s);
    obstacleRegions_units = obstacleTimeCells.Regions_units;
    obstacleTimeCoverage  = struct( ...
        'ExactRegionCount',     numel(obstacleRegions_units), ...
        'ActiveTimeInterval_s', obstacleTimeCells.ActiveTimeInterval_s, ...
        'EndRegions_units',     {obstacleTimeCells.EndRegions_units}, ...
        'BreakTime_s',          obstacleTimeCells.BreakTime_s);
end
