function [candidate, checkResult, diagnostics, validationElapsedTime_s, stageTiming] = solveTimedBmtpTrajectory(seed, obstacles, initialState, goalState, limits, options, stageTiming)
%% Section 0: Header & Readme
% SYNTAX
%   [candidate, checkResult, diagnostics, ...
%       validationElapsedTime_s, stageTiming] = ...
%       obstacleAvoidance.planner.solveTimedBmtpTrajectory( ...
%       seed, obstacles, initialState, goalState, limits, options, ...
%       stageTiming)
%
% PURPOSE
%   - Adapt one timed multi-waypoint seed to the smooth BMTP engine without
%     stretching its proposed crossing times when trying a different arrival.
%   - Conservatively bind each moving-obstacle time cell to the polynomial
%     spans that overlap it, without constraining interior waypoints to rest.
%
% INPUTS
%   - seed (scalar struct)
%       position_units is N-by-2, tau increases zero to one, and
%       EstimatedDuration_s is the search-derived arrival estimate.
%   - obstacles (canonical or prepared obstacle struct array)
%       Static and time-varying protected geometry over the request horizon.
%   - initialState, goalState, limits, options (resolved scalar structs)
%       Normalized planner request and fully resolved planner options.
%   - stageTiming (scalar struct)
%       Accumulated planner timing before this timed solve.
%
% OUTPUTS
%   - candidate (scalar struct)
%       Smooth motion or stable expected-failure record for public validation.
%   - checkResult (scalar validation struct)
%       Authoritative validation for the accepted trial or stable failure.
%   - diagnostics (scalar struct)
%       Engine evidence plus every attempted fixed-arrival time-cell trial,
%       including the physical waypoint times used to initialize each solve.
%       Earliest-arrival trials use the shared discrete search time grid;
%       they do not certify a continuous-time global optimum.
%   - validationElapsedTime_s (nonnegative scalar)
%       Total authoritative-validation time nested inside this stage.
%   - stageTiming (scalar struct)
%       Timing updated by every authoritative trial check.
%
% UNITS
%   - Position is coordinate units and time is seconds. Derivatives use units/s,
%     units/s^2, and units/s^3. Histories and polygon vertices are N-by-2.
%

%% Section 1: Retain The Route Search's Physical Arrival Times

startTime_s     = initialState.time_s;
horizonTime_s   = goalState.time_s;
estimatedTime_s = startTime_s + double(seed.EstimatedDuration_s);
if options.GoalTimeMode == "fixedArrival"
    trialTime_s = horizonTime_s;
else
    % The route estimate ignores acceleration and jerk. It can suggest a
    % trial, but its failure cannot discard the times before the deadline.
    % Reuse every route-search layer and add the exact rest-to-rest lower
    % bound. Only that physical bound can exclude earlier arrivals.
    direct           = bmtpEngine.createDirectMotion(initialState, goalState, limits, options);
    lowerBoundTime_s = startTime_s;
    % A bound to one endpoint cannot exclude earlier intercepts of a moving
    % target. Keep all positive-time layers when no fixed-endpoint bound exists.
    goalIsFixed = ~isfield(goalState, 'targetTime_s') || isempty(goalState.targetTime_s);
    if goalIsFixed && all(isfinite(direct.MinimumAxisDuration_s))
        lowerBoundTime_s = startTime_s + max(direct.MinimumAxisDuration_s);
    end
    trialTime_s      = obstacleAvoidance.search.createTimeLayers(obstacles, startTime_s, horizonTime_s);
    trialTime_s      = unique([trialTime_s; estimatedTime_s; lowerBoundTime_s]);
    trialTime_s      = trialTime_s(isfinite(trialTime_s) & trialTime_s > startTime_s & trialTime_s >= lowerBoundTime_s & trialTime_s <= horizonTime_s);
    if isempty(trialTime_s)
        % Keep the original deadline attempt so an impossible request still
        % returns the usual construction failure and complete diagnostics.
        trialTime_s = horizonTime_s;
    end
end
trialTime_s              = unique(double(trialTime_s(:)), "stable");
maximumTimedSegmentCount = options.MaximumTimeLayerCount - 1;
timedSegmentCounts       = maximumTimedSegmentCount;
trialTemplate            = struct();
trialTemplate.FinalTime_s             = NaN;
trialTemplate.TimedSegmentCount       = 0;
trialTemplate.Coverage                = struct();
trialTemplate.Success                 = false;
trialTemplate.TerminationReason       = "notRun";
trialTemplate.ValidationPassed        = false;
trialTemplate.ValidationMessage       = "";
trialTemplate.ElapsedTime_s           = 0;
trialTemplate.ValidationElapsedTime_s = 0;
trialTemplate.WarmStartWaypointTime_s  = zeros(0, 1);
maximumTrialCount       = numel(trialTime_s);
trials                  = repmat(trialTemplate, maximumTrialCount, 1);
checkResult             = obstacleAvoidance.validation.validatePreparedTrajectory();
validationElapsedTime_s = 0;
totalTimer              = tic;

%% Section 2: Solve Each Full-Resolution Time-Cell Representation

fixedOptions = options;
fixedOptions.GoalTimeMode = "fixedArrival";
completedTrialCount = 0;
seedWaypointTime_s = startTime_s + double(seed.tau(:)) * double(seed.EstimatedDuration_s);
% Try arrivals in time order. A failed time does not exclude any later time:
% a moving obstacle can make one arrival unsafe and the next one safe.
for timeIndex = 1:numel(trialTime_s)
    fixedGoalState      = createFixedGoalState(goalState, trialTime_s(timeIndex));
    completedTrialCount = completedTrialCount + 1;
    [regions_units, coverage] = createTimeCellRegions(obstacles, startTime_s, trialTime_s(timeIndex), maximumTimedSegmentCount);
    % Preserve each interior waypoint's physical time when trying a later
    % arrival. Only the goal time changes; the optimizer can then move
    % the warm-start curve without silently delaying every obstacle crossing.
    % For an earlier arrival, omit later knots from this starting guess only.
    % Every solved motion must still reach the goal and pass full validation.
    timedSeed      = seed;
    interior       = find(seedWaypointTime_s > startTime_s & seedWaypointTime_s < trialTime_s(timeIndex));
    interior       = interior(interior < numel(seedWaypointTime_s));
    waypointTime_s = [startTime_s; seedWaypointTime_s(interior); trialTime_s(timeIndex)];
    timedSeed.position_units = seed.position_units([1; interior; size(seed.position_units, 1)], :);
    timedSeed.tau          = (waypointTime_s - startTime_s) / (trialTime_s(timeIndex) - startTime_s);
    trialTimer = tic;
    [trialCandidate, trialDiagnostics] = bmtpEngine.solve(timedSeed, regions_units, coverage, initialState, fixedGoalState, limits, fixedOptions);
    trials(completedTrialCount).FinalTime_s = trialTime_s(timeIndex);
    trials(completedTrialCount).TimedSegmentCount = maximumTimedSegmentCount;
    trials(completedTrialCount).Coverage = coverage;
    trials(completedTrialCount).WarmStartWaypointTime_s = waypointTime_s;
    trials(completedTrialCount).Success = trialCandidate.Success;
    trials(completedTrialCount).TerminationReason = trialCandidate.TerminationReason;
    trials(completedTrialCount).ElapsedTime_s = toc(trialTimer);
    candidate   = trialCandidate;
    checkResult = obstacleAvoidance.validation.validatePreparedTrajectory();
    diagnostics = trialDiagnostics;
    diagnostics.Identifier                    = "bmtpTimedCell";
    diagnostics.TimedSegmentCounts            = timedSegmentCounts;
    diagnostics.DynamicObstacleRepresentation = "perIntervalProtectedGeometryConvexHull";
    % Independently check constructed timed candidates; failed constructions proceed to the next trial time.
    if trialCandidate.Success
        [trialCandidate, trialCheck, trialValidationTime_s, ...
            stageTiming] = obstacleAvoidance.planner.checkCandidateMotion(trialCandidate, obstacles, initialState, goalState, limits, options, stageTiming, "The timed-cell BMTP kernel returned no trajectory.");
        validationElapsedTime_s = validationElapsedTime_s + trialValidationTime_s;
        trials(completedTrialCount).ValidationPassed = trialCheck.Passed;
        trials(completedTrialCount).ValidationMessage = trialCheck.Message;
        trials(completedTrialCount).ValidationElapsedTime_s = trialValidationTime_s;
        candidate   = trialCandidate;
        checkResult = trialCheck;
    end
    % Return immediately only when construction and independent checking both accept the timed candidate.
    if trialCandidate.Success && trialCheck.Passed
        diagnostics.Accepted          = true;
        diagnostics.TerminationReason = "goalReached";
        break;
    end
end
diagnostics.TimeCellTrials      = trials(1:completedTrialCount);
diagnostics.TrialCount          = completedTrialCount;
diagnostics.TimedCellTrialCount = completedTrialCount;
diagnostics.ElapsedTime_s       = toc(totalTimer);
candidate.SolverDiagnostics = diagnostics;
end

%% Section 3: Local Functions

function fixedGoalState = createFixedGoalState(goalState, finalTime_s)
    % Freeze the requested endpoint at one physical trial time.
    fixedGoalState = goalState;
    fixedGoalState.time_s       = finalTime_s;
    fixedGoalState.position_units = obstacleAvoidance.input.goalPositionAtTime(goalState, finalTime_s);
    metadataFields = intersect(fieldnames(fixedGoalState), {'targetTime_s', 'targetPosition_units', 'InterpolationMethod'});
    if ~isempty(metadataFields)
        fixedGoalState = rmfield(fixedGoalState, metadataFields);
    end
end

function [regions_units, coverage] = createTimeCellRegions(obstacles, startTime_s, finishTime_s, timedSegmentCount)
    % Cover static shapes exactly and movers by interval-wide convex supersets.
    regions_units         = cell(0, 1);
    activeTauInterval   = zeros(0, 2);
    sourceObstacleIndex = zeros(0, 1);
    sourceCellIndex     = zeros(0, 1);
    timeCellCount       = timedSegmentCount;
    baseEdges_s         = linspace(startTime_s, finishTime_s, timeCellCount + 1).';
    % Evaluate each obstacle against the current geometry or motion.
    for obstacleIndex = 1:numel(obstacles)
        obstacle = obstacles(obstacleIndex);
        [isStatic, staticShape] = obstacleAvoidance.obstacles.queryStaticHorizon(obstacle, startTime_s, finishTime_s);
        if isStatic
            exactRegions = obstacleAvoidance.geometry.convexPolygonRegions(staticShape);
            % Process each geometric region while constructing or checking the region topology.
            for regionIndex = 1:numel(exactRegions)
                vertices_units = finiteVertices(exactRegions(regionIndex).Vertices);
                if size(vertices_units, 1) >= 3
                    regions_units{end + 1, 1} = vertices_units; %#ok<AGROW>
                    activeTauInterval(end + 1, :) = [0 1]; %#ok<AGROW>
                    sourceObstacleIndex(end + 1, 1) = obstacleIndex; %#ok<AGROW>
                    sourceCellIndex(end + 1, 1) = 0; %#ok<AGROW>
                end
            end
            continue;
        end
        obstacleTimes_s = double(obstacle.time_s(:));
        internalEdges_s = obstacleTimes_s(obstacleTimes_s > startTime_s & obstacleTimes_s < finishTime_s);
        cellEdges_s     = snapCellEdgesToObstacleTimes([baseEdges_s; internalEdges_s], obstacleTimes_s);
        % Process each geometric cell while constructing or checking the region topology.
        for cellIndex = 1:numel(cellEdges_s) - 1
            cellStart_s  = cellEdges_s(cellIndex);
            cellFinish_s = cellEdges_s(cellIndex + 1);
            queryTime_s  = [cellStart_s; ...
                0.5 * (cellStart_s + cellFinish_s); cellFinish_s];
            vertices_units = zeros(0, 2);
            % Process each query in temporal order and accumulate its result.
            for queryIndex = 1:numel(queryTime_s)
                shape        = obstacleAvoidance.obstacles.preparedShapeAtTime(obstacle, queryTime_s(queryIndex));
                vertices_units = [vertices_units; ...
                    finiteVertices(shape.Vertices)]; %#ok<AGROW>
            end
            vertices_units = unique(vertices_units, "rows", "stable");
            if size(vertices_units, 1) < 3
                continue;
            end
            hullIndex = convhull(vertices_units(:, 1), vertices_units(:, 2));
            regions_units{end + 1, 1} = ...
                vertices_units(hullIndex(1:end - 1), :); %#ok<AGROW>
            activeTauInterval(end + 1, :) = ...
                ([cellStart_s cellFinish_s] - startTime_s) / ...
                (finishTime_s - startTime_s); %#ok<AGROW>
            sourceObstacleIndex(end + 1, 1) = obstacleIndex; %#ok<AGROW>
            sourceCellIndex(end + 1, 1) = cellIndex; %#ok<AGROW>
        end
    end
    coverage = struct("Passed", true, "ObstacleCount", numel(obstacles), ...
        "RegionCount", numel(regions_units), ...
        "ExactRegionCount", numel(regions_units), ...
        "SolverRegionCount", numel(regions_units), ...
        "RegionActiveTauInterval", activeTauInterval, ...
        "RegionSourceObstacleIndex", sourceObstacleIndex, ...
        "RegionSourceCellIndex", sourceCellIndex, ...
        "BaseTimeCellCount", timeCellCount, ...
        "TimedSegmentCount", timedSegmentCount, ...
        "TimeCellContainmentBasis", ...
        "convexHullOfProtectedIntervalEndpointAndMidpointGeometry", ...
        "AuthoritativeCoverageCheck", "publicDynamicValidation");
end

function cellEdges_s = snapCellEdgesToObstacleTimes(candidateEdges_s, obstacleTimes_s)
    % Merge event times that differ only by roundoff.
    timeScale_s     = max([1; abs(candidateEdges_s); abs(obstacleTimes_s)]);
    timeTolerance_s = 4096 * eps(timeScale_s);
    % Process each event in temporal order and accumulate its result.
    for eventIndex = 1:numel(obstacleTimes_s)
        nearEvent = abs(candidateEdges_s - obstacleTimes_s(eventIndex)) <= timeTolerance_s;
        candidateEdges_s(nearEvent) = obstacleTimes_s(eventIndex);
    end
    cellEdges_s = unique(candidateEdges_s, "sorted");
end

function vertices_units = finiteVertices(vertices_units)
    % Remove polyshape ring separators before exact decomposition or hulling.
    vertices_units = double(vertices_units);
    vertices_units = vertices_units(all(isfinite(vertices_units), 2), :);
end
