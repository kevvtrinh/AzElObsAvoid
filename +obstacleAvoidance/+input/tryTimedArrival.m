function [result, accepted] = tryTimedArrival(previous, maximumArrivalTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   [result, accepted] = obstacleAvoidance.input.tryTimedArrival(previous)
%   [result, accepted] = obstacleAvoidance.input.tryTimedArrival( ...
%       previous, maximumArrivalTime_s)
%**************************************************************************
% PURPOSE
%   - Use timed visibility and BMTP for a fixed-arrival endpoint or a
%     fixed-position free-arrival goal.
%**************************************************************************
% INPUTS
%   - previous (scalar struct)
%       Normalized planner result carrying the request and geometry.
%   - maximumArrivalTime_s (finite scalar, optional)
%       Upper search clock for earliest-arrival requests. Omission keeps the
%       request horizon. Fixed-arrival requests always keep their prescribed
%       clock.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Independently validated motion or a diagnostic failure with
%       Success = false. Invalid input throws an error.
%   - accepted (logical scalar)
%       True only when the timed candidate passes validation.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Check Eligibility And Create The Timed Route

totalTimer   = tic;
result       = resetTimedResult(previous);
accepted     = false;
initialState = previous.Inputs.initialState;
goalState    = previous.Inputs.goalState;
request = struct( ...
    'initialState', initialState, ...
    'goalState',    goalState, ...
    'limits',       previous.Limits, ...
    'options',      previous.Options);
outerRequest = [];
if isfield(previous, 'OuterRequest')
    outerRequest = previous.OuterRequest;
end
requestContext = struct( ...
    'obstacles',          {previous.Inputs.obstacles}, ...
    'suppliedLimits',     previous.SuppliedLimits, ...
    'requestedLimits',    previous.RequestedLimits, ...
    'suppliedGoalState',  previous.SuppliedGoalState, ...
    'requestedGoalState', previous.RequestedGoalState, ...
    'outerRequest',       {outerRequest});
if nargin < 2 || isempty(maximumArrivalTime_s)
    maximumArrivalTime_s = goalState.time_s;
end
validateattributes(maximumArrivalTime_s, {'numeric'}, ...
    {'real', 'finite', 'scalar', '>', initialState.time_s});
maximumArrivalTime_s = min(maximumArrivalTime_s, goalState.time_s);
if previous.Options.GoalTimeMode == "earliestArrival"
    goalState.time_s = maximumArrivalTime_s;
end

endpointDerivatives = [initialState.velocity_units_s(:); initialState.acceleration_units_s2(:); ...
    goalState.velocity_units_s(:); goalState.acceleration_units_s2(:)];
endpointsAreAtRest = all(endpointDerivatives == 0);
arrivalIsFixed     = previous.Options.GoalTimeMode == "fixedArrival";
freeClockIsUnsupported = ~arrivalIsFixed && ...
    (~isempty(goalState.targetMotion) || ~endpointsAreAtRest);
if freeClockIsUnsupported
    result.Message = "Free-arrival timed visibility requires a fixed-position goal " + ...
        "with zero endpoint velocity and acceleration.";
    result.TerminationReason = "unsupportedTimedRequest";
    result.ElapsedTime_s     = previous.ElapsedTime_s + toc(totalTimer);
    return
end
searchTimer = tic;
[route_units, routeTime_s, searchRecord] = obstacleAvoidance.search.createTimedRouteProposal( ...
    previous.PreparedObstacles, initialState, goalState, previous.RequestedLimits, previous.Options);
searchRecord.ElapsedTime_s = toc(searchTimer);
result.VisibilityGraph.NodePosition_units = searchRecord.Nodes_units;
result.VisibilityGraph.TimedSearch        = searchRecord;
result.VisibilityGraph.ExpandedCount      = searchRecord.TimedSearch.ExpandedCount;
if isempty(routeTime_s)
    result.Message                     = "No route reached the requested goal layer in the discrete timed graph.";
    result.TerminationReason           = "noTimedRoute";
    result.FailureStage                = "search";
    result.FailureKind                 = "noTimedRoute";
    result.ElapsedTime_s               = previous.ElapsedTime_s + toc(totalTimer);
    return
end
if size(route_units, 1) > 2
    % A constant-position run needs only its first and last times. Removing
    % interior wait knots preserves the complete piecewise-linear timed guide
    % and keeps the BMTP mesh independent of temporal-layer density.
    segmentMoves        = any(diff(route_units, 1, 1) ~= 0, 2);
    routeKnotIsRetained = [true; segmentMoves(1:end - 1) | segmentMoves(2:end); true];
    route_units         = route_units(routeKnotIsRetained, :);
    routeTime_s         = routeTime_s(routeKnotIsRetained);
end
result.VisibilityGraph.RouteTime_s       = routeTime_s;
result.VisibilityGraph.Route_units       = route_units;
result.VisibilityGraph.RouteLength_units = sum(vecnorm(diff(route_units), 2, 2));
result.VisibilityGraph.IsConnected       = true;
result.Route_units                        = route_units;

%% Section 2: Solve The Timed Route On Its Supplied Physical Clock

timedSearch          = searchRecord.TimedSearch;
freeGoalWindowIsUsed = ~arrivalIsFixed;
if freeGoalWindowIsUsed
    goalWindowIsDeclared = isfield(timedSearch, 'SelectedGoalWindowEndTime_s');
    freeGoalWindowIsUsed = goalWindowIsDeclared && timedSearch.SelectedGoalWindowEndTime_s > ...
        routeTime_s(end) + previous.Options.ArrivalTimeTolerance_s;
end
seedDuration_s = routeTime_s(end) - initialState.time_s;

% The producer declares the physical clock this guide was built on.
normalizedRouteTime = (routeTime_s - initialState.time_s) / seedDuration_s;
seed = struct( ...
    'position_units',      route_units, ...
    'tau',                 normalizedRouteTime, ...
    'UsesVariableClock',   false, ...
    'UsesTimeScopedSolver', false);
motionGoalState = goalState;
motionOptions   = previous.Options;
if freeGoalWindowIsUsed
    motionGoalState.time_s        = timedSearch.SelectedGoalWindowEndTime_s;
    motionOptions.GoalTimeMode    = "earliestArrival";
    minimumArrivalTime_s          = max(timedSearch.SelectedGoalWindowStartTime_s, ...
        timedSearch.MinimumGoalArrivalTime_s);
    [regions_units, coverage] = createTimedCoverage( ...
        previous.PreparedObstacles, initialState.time_s, motionGoalState.time_s);
    coverage.MinimumMotionDuration_s = minimumArrivalTime_s - initialState.time_s;
    coverage.SeedMotionDuration_s    = seedDuration_s;
    seed.UsesVariableClock           = true;
else
    motionGoalState.time_s        = routeTime_s(end);
    motionOptions.GoalTimeMode    = "fixedArrival";
    [regions_units, coverage] = createTimedCoverage( ...
        previous.PreparedObstacles, initialState.time_s, motionGoalState.time_s);
    seed.UsesTimeScopedSolver     = true;
end
[candidate, diagnostics] = bmtpEngine.solve(seed, regions_units, coverage, ...
    initialState, motionGoalState, previous.RequestedLimits, motionOptions);
if ~candidate.Success
    result = obstacleAvoidance.input.finalizeCandidate( ...
        previous.PreparedObstacles, request, requestContext, ...
        result.VisibilityGraph, result.Attempts, ...
        result.ElapsedTime_s, true, struct(), candidate, route_units, diagnostics);
    result.TerminationReason = "timedMotionInfeasible";
    result.Message = "The timed route did not produce a feasible BMTP motion: " + ...
        candidate.Message;
    result.ElapsedTime_s = previous.ElapsedTime_s + toc(totalTimer);
    return
end

%% Section 3: Assemble And Independently Validate The Result

validationDeclarations = struct();
if freeGoalWindowIsUsed
    validationDeclarations.GoalArrivalWindow_s = ...
        [minimumArrivalTime_s, motionGoalState.time_s];
    validationDeclarations.TrajectoryCoverageEndTime_s = motionGoalState.time_s;
else
    % Declare the prescribed clock the motion was solved on, not the
    % achieved arrival; the validator compares the two.
    validationDeclarations.FixedArrivalTrialTime_s = motionGoalState.time_s;
end
result = obstacleAvoidance.input.finalizeCandidate( ...
    previous.PreparedObstacles, request, requestContext, ...
    result.VisibilityGraph, result.Attempts, ...
    result.ElapsedTime_s, true, validationDeclarations, candidate, route_units, diagnostics);
result.ElapsedTime_s = previous.ElapsedTime_s + toc(totalTimer);
accepted             = result.Success;
if ~accepted
    return
end
if arrivalIsFixed
    result.Message = "The prescribed goal layer produced an independently validated timed BMTP motion.";
elseif freeGoalWindowIsUsed
    result.Message = "The first reachable goal window produced an independently validated free-clock BMTP motion.";
else
    result.Message = "The earliest reachable timed-route layer produced an independently validated BMTP motion.";
end
result.TerminationReason = "goalReached";
minimumTravelTime_s = obstacleAvoidance.input.minimumTravelTime( ...
    initialState, goalState, previous.Limits);
necessaryArrivalTime_s = initialState.time_s + minimumTravelTime_s;
result.TemporalSearch = struct( ...
    'Resolution_s',            previous.Options.TemporalResolution_s, ...
    'TrialTime_s',             candidate.ArrivalTime_s, ...
    'GlobalEarliestProven',    false, ...
    'NecessaryArrivalBound_s', necessaryArrivalTime_s, ...
    'IncumbentArrival_s',      NaN, ...
    'RetainedIncumbent',       false);
result.ElapsedTime_s = previous.ElapsedTime_s + toc(totalTimer);
end

%% Section 4: Local Functions

function [regions_units, coverage] = createTimedCoverage(obstacles, startTime_s, finishTime_s)
    % Give every timed BMTP path the same exact cells and reconstruction metadata.
    cells         = obstacleAvoidance.obstacles.createTimeCells(obstacles, startTime_s, finishTime_s);
    regions_units = cells.Regions_units;
    coverage = struct( ...
        'ExactRegionCount',     numel(regions_units), ...
        'ActiveTimeInterval_s', cells.ActiveTimeInterval_s, ...
        'EndRegions_units',     {cells.EndRegions_units}, ...
        'BreakTime_s',          cells.BreakTime_s);
end

function result = resetTimedResult(previous)
    % Start the timed attempt with no motion, graph, or solver state from a
    % failed spatial proposal. Request provenance and earlier attempts remain.
    result                              = previous;
    result.Success                      = false;
    result.Message                      = "The timed fallback has not completed.";
    result.TerminationReason            = "notStarted";
    result.Route_units                  = zeros(0, 2);
    result.time_s                       = zeros(0, 1);
    result.position_units               = zeros(0, 2);
    result.velocity_units_s             = zeros(0, 2);
    result.acceleration_units_s2        = zeros(0, 2);
    result.jerk_units_s3                = zeros(0, 2);
    result.Polynomial                   = struct();
    result.PlaneCertificate             = struct();
    result.SolverDiagnostics            = struct();
    result.Validation                   = struct( ...
        "Passed", false, "Message", "No timed motion is available.");
    result.ArrivalTime_s                = NaN;
    result.TrajectoryDuration_s         = NaN;
    result.MotionLength_units           = Inf;
    result.IntegratedSquaredJerk_units2_s5 = Inf;
    result.MaximumConstraintViolation   = Inf;
    result.OptimizerFeasible            = false;
    result.OptimizerIterateUnavailable  = false;
    result.AlternativeGuideEligible     = false;
    result.FailureStage                 = "notRun";
    result.FailureKind                  = "notRun";
    staleFieldNames = ["TemporalSearch", "GoalArrivalWindow_s", ...
        "FixedArrivalTrialTime_s", "TrajectoryCoverageEndTime_s"];
    for fieldName = staleFieldNames
        if isfield(result, fieldName)
            result = rmfield(result, fieldName);
        end
    end
    result.Intercept = struct( ...
        'Time_s', NaN, ...
        'TargetPosition_units', previous.Inputs.goalState.position_units, ...
        'TerminalVelocityPolicy', "zero");
    result.VisibilityGraph = struct( ...
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
end
