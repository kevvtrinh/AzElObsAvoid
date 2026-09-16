function [result, accepted] = tryTimedArrival(previous)
%% Section 0: Header & Readme
% SYNTAX
%   [result, accepted] = obstacleAvoidance.input.tryTimedArrival(previous)
%**************************************************************************
% PURPOSE
%   - Use timed visibility and BMTP for a fixed-arrival endpoint or a
%     fixed-position free-arrival goal.
%**************************************************************************
% INPUTS
%   - previous (scalar struct)
%       Normalized planner result carrying the request and geometry.
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

totalTimer     = tic;
result         = previous;
result.Success = false;
accepted       = false;
initialState   = previous.Inputs.initialState;
goalState      = previous.Inputs.goalState;

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
result.VisibilityGraph.SearchKind = "timeExpandedVisibilityGraph";
searchTimer = tic;
[route_units, routeTime_s, searchRecord] = obstacleAvoidance.search.createTimedRouteProposal( ...
    previous.PreparedObstacles, initialState, goalState, previous.RequestedLimits, previous.Options);
searchRecord.ElapsedTime_s = toc(searchTimer);
if isempty(routeTime_s)
    result.Message                     = "No route reached the requested goal layer in the discrete timed graph.";
    result.TerminationReason           = "noTimedRoute";
    result.VisibilityGraph.TimedSearch = searchRecord;
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
result.VisibilityGraph.TimedSearch       = searchRecord;
result.VisibilityGraph.RouteTime_s       = routeTime_s;
result.VisibilityGraph.Route_units       = route_units;
result.VisibilityGraph.RouteLength_units = sum(vecnorm(diff(route_units), 2, 2));
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

% The producer declares the physical clock this guide was built on. Source
% remains a diagnostic label and never selects a solver.
normalizedRouteTime = (routeTime_s - initialState.time_s) / seedDuration_s;
seed = struct( ...
    'position_units',      route_units, ...
    'tau',                 normalizedRouteTime, ...
    'Source',              "timeExpandedVisibilityGraph", ...
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
    result.Message           = "The timed route did not produce a feasible BMTP motion: " + candidate.Message;
    result.TerminationReason = "timedMotionInfeasible";
    result.SolverDiagnostics = diagnostics;
    result.ElapsedTime_s     = previous.ElapsedTime_s + toc(totalTimer);
    return
end

%% Section 3: Assemble And Independently Validate The Result

if freeGoalWindowIsUsed
    result.GoalArrivalWindow_s         = [minimumArrivalTime_s, motionGoalState.time_s];
    result.TrajectoryCoverageEndTime_s = motionGoalState.time_s;
else
    % Declare the prescribed clock the motion was solved on, not the
    % achieved arrival; the validator compares the two.
    result.FixedArrivalTrialTime_s = motionGoalState.time_s;
end
result = obstacleAvoidance.input.finalizeCandidate(result, candidate, route_units, diagnostics);
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
    'TrialTerminationReason',  "goalReached", ...
    'TrialStage',              string(candidate.SeedSource), ...
    'GlobalEarliestProven',    false, ...
    'NecessaryArrivalBound_s', necessaryArrivalTime_s, ...
    'IncumbentArrival_s',      NaN, ...
    'RetainedIncumbent',       false, ...
    'PriorTerminationReason',  previous.TerminationReason);
result.ElapsedTime_s = previous.ElapsedTime_s + toc(totalTimer);
end

%% Section 4: Local Functions

function [regions_units, coverage] = createTimedCoverage(obstacles, startTime_s, finishTime_s)
    % Give every timed BMTP path the same exact cells and reconstruction metadata.
    cells         = obstacleAvoidance.obstacles.createTimeCells(obstacles, startTime_s, finishTime_s);
    regions_units = cells.Regions_units;
    coverage = struct( ...
        'Passed',               true, ...
        'ExactRegionCount',     numel(regions_units), ...
        'ActiveTimeInterval_s', cells.ActiveTimeInterval_s, ...
        'EndRegions_units',     {cells.EndRegions_units}, ...
        'BreakTime_s',          cells.BreakTime_s);
end
