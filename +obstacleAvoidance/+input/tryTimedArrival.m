function [result, accepted] = tryTimedArrival(previous)
%% Section 0: Header & Readme
% SYNTAX: [result,accepted] = obstacleAvoidance.input.tryTimedArrival(previous)
% PURPOSE: Use a time-expanded visibility proposal and timed BMTP for a moving obstacle,
%   fixed-position, rest-to-rest fixed- or earliest-arrival request.
% INPUTS: A normalized planner result carrying the original request and geometry.
% OUTPUTS: Independently validated motion when accepted is true; otherwise a
%   diagnostic failure. Earliest-arrival callers may continue chronological search.
% UNITS: Position is coordinate units and time is seconds.

%% Section 1: Check Eligibility And Create The Timed Route
timer = tic;
result = previous;
result.Success = false;
accepted = false;
initialState = previous.Inputs.initialState;
goalState = previous.Inputs.goalState;
sourceIntervalCount = sum(arrayfun(@(obstacle) ...
    max(0,numel(obstacle.time_s)-1),previous.PreparedObstacles));
isRest = all([initialState.velocity_units_s(:);initialState.acceleration_units_s2(:); ...
    goalState.velocity_units_s(:);goalState.acceleration_units_s2(:)] == 0);
% The timed mesh compresses dense histories. Sparse histories remain faster
% through the exact chronological planner and its validated wait incumbent.
isFixedArrival = previous.Options.GoalTimeMode=="fixedArrival";
if ~isempty(goalState.targetMotion) || ~isRest || ...
    (~isFixedArrival && sourceIntervalCount < 16)
    result.Message = "Timed visibility requires a fixed-position goal and zero endpoint velocity and acceleration.";
    result.TerminationReason = "unsupportedTimedRequest";
    result.ElapsedTime_s = previous.ElapsedTime_s+toc(timer);
    return;
end
result.VisibilityGraph.SearchKind = "timeExpandedVisibilityGraph";
searchTimer = tic;
[route_units,routeTime_s,searchRecord] = ...
    obstacleAvoidance.search.createTimedRouteProposal( ...
    previous.PreparedObstacles,initialState,goalState, ...
    previous.RequestedLimits,previous.Options);
searchRecord.ElapsedTime_s = toc(searchTimer);
if isempty(routeTime_s)
    result.Message = "No route reached the requested goal layer in the discrete timed graph.";
    result.TerminationReason = "noTimedRoute";
    result.VisibilityGraph.TimedSearch = searchRecord;
    result.ElapsedTime_s = previous.ElapsedTime_s+toc(timer);
    return;
end
if isFixedArrival && size(route_units,1)>2
    % A constant-position run needs only its first and last times. Removing
    % interior wait knots preserves the complete piecewise-linear timed guide.
    moving = any(diff(route_units,1,1)~=0,2);
    keep = [true;moving(1:end-1)|moving(2:end);true];
    route_units = route_units(keep,:);
    routeTime_s = routeTime_s(keep);
end
result.VisibilityGraph.TimedSearch = searchRecord;
result.VisibilityGraph.RouteTime_s = routeTime_s;
result.VisibilityGraph.Route_units = route_units;
result.VisibilityGraph.RouteLength_units = sum(vecnorm(diff(route_units),2,2));
result.Route_units = route_units;

%% Section 2: Solve The Earliest Layer Then Its Window Wait Guide
fixedGoalState = goalState;
fixedGoalState.time_s = routeTime_s(end);
fixedOptions = previous.Options;
fixedOptions.GoalTimeMode = "fixedArrival";
[regions_units,coverage]=obstacleAvoidance.obstacles.createTimedBmtpCells( ...
    previous.PreparedObstacles,initialState.time_s,fixedGoalState.time_s,16);
seed = struct('position_units',route_units, ...
    'tau',(routeTime_s - initialState.time_s) / ...
    (fixedGoalState.time_s - initialState.time_s), ...
    'Index',1,'Source',"timeExpandedVisibilityGraph", ...
    'ObstacleEnvelope_units',zeros(0,2));
[candidate,diagnostics] = bmtpEngine.solve(seed,regions_units,coverage, ...
    initialState,fixedGoalState,previous.RequestedLimits,fixedOptions);
if candidate.Success
    [candidate,diagnostics] = certifyAgainstExactCells(candidate,diagnostics, ...
        seed,previous.PreparedObstacles,initialState,fixedGoalState, ...
        previous.RequestedLimits,fixedOptions);
end
usedWaitGuide=false;
if ~candidate.Success && ~isFixedArrival && ...
        isfield(searchRecord.TimedSearch,'WaitRouteTime_s')
    waitRouteTime_s=searchRecord.TimedSearch.WaitRouteTime_s;
    if ~isempty(waitRouteTime_s) && ...
            waitRouteTime_s(end)>routeTime_s(end)+fixedOptions.ArrivalTimeTolerance_s
        initialFailure=struct('Message',candidate.Message, ...
            'SolverDiagnostics',diagnostics);
        route_units=searchRecord.TimedSearch.WaitRoute_units;
        routeTime_s=waitRouteTime_s;
        result.VisibilityGraph.RouteTime_s=routeTime_s;
        result.VisibilityGraph.Route_units=route_units;
        result.VisibilityGraph.RouteLength_units=sum(vecnorm(diff(route_units),2,2));
        result.Route_units=route_units;
        fixedGoalState.time_s=routeTime_s(end);
        cells=obstacleAvoidance.obstacles.createTimeCells( ...
            previous.PreparedObstacles,initialState.time_s,fixedGoalState.time_s,true);
        regions_units=cells.Regions_units;
        coverage=struct('Passed',true,'ExactRegionCount',numel(regions_units), ...
            'SolverRegionCount',numel(regions_units), ...
            'AuthoritativeCoverageCheck','independentPlaneVerification', ...
            'ActiveTimeInterval_s',cells.ActiveTimeInterval_s, ...
            'EndRegions_units',{cells.EndRegions_units}, ...
            'BreakTime_s',cells.BreakTime_s, ...
            'ConvexMergeOrder',"longestSharedEdgeFirst");
        seed=struct('position_units',route_units, ...
            'tau',(routeTime_s-initialState.time_s)/ ...
            (fixedGoalState.time_s-initialState.time_s), ...
            'Index',1,'Source',"timeExpandedWaitGuide", ...
            'ObstacleEnvelope_units',zeros(0,2));
        [candidate,diagnostics]=bmtpEngine.solve(seed,regions_units,coverage, ...
            initialState,fixedGoalState,previous.RequestedLimits,fixedOptions);
        diagnostics.InitialTimedRouteFailure=initialFailure;
        usedWaitGuide=true;
    end
end
if ~candidate.Success
    result.Message="The timed route did not produce a feasible BMTP motion: "+ ...
        candidate.Message;
    result.TerminationReason="timedMotionInfeasible";
    result.VisibilityGraph.TimedSearch=searchRecord;
    result.SolverDiagnostics=diagnostics;
    result.ElapsedTime_s=previous.ElapsedTime_s+toc(timer);
    return;
end

%% Section 3: Assemble And Independently Validate The Result
for fieldName = reshape(string(fieldnames(candidate)),1,[])
    result.(fieldName) = candidate.(fieldName);
end
result.SolverDiagnostics = diagnostics;
result.FixedArrivalTrialTime_s = candidate.ArrivalTime_s;
result.Validation = obstacleAvoidance.validateTrajectory(result);
result.ElapsedTime_s = previous.ElapsedTime_s + toc(timer);
accepted = result.Validation.Passed;
result.Success = accepted;
if ~accepted
    result.Message = "Timed BMTP motion failed independent validation: "+result.Validation.Message;
    result.TerminationReason = "invalidTimedMotion";
    return;
end
result.Message = "The earliest reachable timed-route layer produced an independently validated BMTP motion.";
if usedWaitGuide
    result.Message="The first reachable goal window produced an independently validated BMTP motion from a near-goal wait guide.";
end
if isFixedArrival
    result.Message = "The prescribed goal layer produced an independently validated timed BMTP motion.";
end
result.TerminationReason = "goalReached";
necessaryArrival_s = initialState.time_s + ...
    obstacleAvoidance.input.minimumTravelTime(initialState,goalState, ...
    previous.Limits);
result.TemporalSearch = struct('Resolution_s',previous.Options.TemporalResolution_s, ...
    'TrialTime_s',candidate.ArrivalTime_s,'TrialTerminationReason',"goalReached", ...
    'TrialStage',string(seed.Source),'GlobalEarliestProven',false, ...
    'NecessaryArrivalBound_s',necessaryArrival_s,'IncumbentArrival_s',NaN, ...
    'RetainedIncumbent',false,'PriorTerminationReason',previous.TerminationReason);
result.ElapsedTime_s = previous.ElapsedTime_s + toc(timer);
end

%% Section 4: Local Functions
function [candidate,diagnostics] = certifyAgainstExactCells(candidate,diagnostics, ...
        seed,obstacles,initialState,goalState,limits,options)
    cells = obstacleAvoidance.obstacles.createTimeCells( ...
        obstacles,initialState.time_s,goalState.time_s,true);
    coverage = struct('Passed',true,'ExactRegionCount',numel(cells.Regions_units), ...
        'SolverRegionCount',numel(cells.Regions_units), ...
        'AuthoritativeCoverageCheck','independentPlaneVerification', ...
        'ActiveTimeInterval_s',cells.ActiveTimeInterval_s, ...
        'EndRegions_units',{cells.EndRegions_units},'BreakTime_s',cells.BreakTime_s);
    coverage.ConvexMergeOrder = "longestSharedEdgeFirst";
    request = bmtpEngine.createSolveRequest(seed,cells.Regions_units,coverage, ...
        initialState,goalState,limits,options);
    warmStart = bmtpEngine.createWarmStart(request);
    preparedMotion = struct('CertifiedControlPoint_units', ...
        powerToBernstein(candidate.Polynomial.positionPower_units), ...
        'SegmentTime_s',candidate.Polynomial.SegmentDuration_s);
    [~,~,reserve_units] = bmtpEngine.createCoordinateTolerances(seed.position_units, ...
        limits.xInterval_units,limits.yInterval_units, ...
        cells.Regions_units,cells.EndRegions_units);
    target_units = (1 + 2 ^ 20 * eps) * ...
        options.CollisionClearanceTolerance_units + reserve_units;
    certificate = bmtpEngine.checkFinalMotion(request,warmStart,preparedMotion, ...
        reserve_units,target_units);
    candidate.PlaneCertificate = certificate;
    diagnostics.PlaneCertificate = certificate;
    diagnostics.FinalCollisionPairCount = ...
        certificate.AllPairCount - certificate.VerifiedPairCount;
    candidate.Success = certificate.Passed;
end

function controlPoint_units = powerToBernstein(positionPower_units)
    degree = size(positionPower_units,3) - 1;
    controlPoint_units = zeros(size(positionPower_units));
    for controlIndex = 0:degree
        for powerIndex = 0:controlIndex
            controlPoint_units(:,:,controlIndex + 1) = ...
                controlPoint_units(:,:,controlIndex + 1) + ...
                nchoosek(controlIndex,powerIndex) / nchoosek(degree,powerIndex) * ...
                positionPower_units(:,:,powerIndex + 1);
        end
    end
    controlPoint_units = permute(controlPoint_units,[1 3 2]);
end
