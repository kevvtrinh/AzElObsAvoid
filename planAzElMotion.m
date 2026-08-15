function result = planAzElMotion(obstacles, initialState, goalState, ...
        limits, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = planAzElMotion()
%   result = planAzElMotion(obstacles, initialState, goalState, limits)
%   result = planAzElMotion(obstacles, initialState, goalState, limits, ...
%       optionOverrides)
%**************************************************************************
% PURPOSE
%   - Plan one general collision-free azimuth/elevation motion through
%     adaptive visibility search and the requested spatial retimer.
%**************************************************************************
% INPUTS
%   - obstacles (canonical obstacle data, nested cell array, or [])
%       Static or time-varying protected polygon geometry.
%   - initialState, goalState (scalar structs)
%       time_s, position_deg, velocity_deg_s, and acceleration_deg_s2.
%   - limits (scalar struct)
%       Per-axis velocity, acceleration, and optional jerk limits.
%   - optionOverrides (scalar struct, optional; default struct())
%       Partial overrides of the zero-input defaults. UseParallel accepts
%       auto, on, off, or a logical scalar for independent polygon-search
%       tasks and falls back to serial execution when unavailable.
%       MaximumVisibilitySnapshotsPerObstacle controls moving-obstacle
%       baseline search slices. DetectSnapshotEvents scans every source
%       slice and promotes topology, edge-rotation, and boundary-motion
%       events before graph construction. CollisionValidationMode selects
%       hybrid prefiltering
%       or direct continuous checks. MinimumContinuouslyValidatedCandidates
%       retains alternative-route evidence before arrival-bound pruning.
%       ContinueAfterFirstFeasible controls improvement after the first
%       independently validated motion. MaximumTemporalRefinementSteps and
%       MaximumPlanningTime_s are explicit anytime-search budgets.
%       OptimalityTolerance_s prunes intervals that cannot improve the
%       incumbent by a meaningful amount. TemporalRefinementSampleTimes_s
%       optionally seeds additional source snapshots. Temporal refinement
%       always targets explicit unresolved intervals; a failed coarse
%       search refines automatically until success, exhaustion, or a
%       stated budget.
%       Every successful result receives the continuous between-sample
%       collision check in either mode.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Stable plan, trajectory, validation, and search record.
%       Expected infeasibility returns Success=false; invalid input throws.
%**************************************************************************
% UNITS
%   - Angles use degrees and time uses seconds. Derivative suffixes state
%     deg/s, deg/s^2, and deg/s^3.
%**************************************************************************

%% Section 1: Validate Inputs & Apply Defaults
defaults = plannerDefaults();

if nargin == 0
    result = defaults;
    return;
end

if nargin < 5 || isempty(optionOverrides)
    optionOverrides = struct();
end

planningTimer = tic;
options = resolveOptions(defaults, optionOverrides);
initialState = normalizeState(initialState, "initialState");
goalState = normalizeState(goalState, "goalState");
limits = normalizeLimits(limits);

if goalState.time_s <= initialState.time_s
    error("planAzElMotion:InvalidTimeWindow", ...
        "goalState.time_s must be greater than initialState.time_s.");
end

%% Section 2: Build Protected Geometry & Visibility Routes

% --- Normalize obstacle geometry ----------------------------------------
protectedAzElData = combineAzElObstacles(obstacles);
obstacleField = buildAzElTimeObstacleField(protectedAzElData, ...
    struct("Verbose", options.Verbose));
[originalObstacleField, obstacleSafetyMargins_deg] = ...
    recoverOriginalAzElObstacleField(obstacleField);

% --- Check the requested endpoints -------------------------------------
startBlocked = queryAzElTimeObstacle(obstacleField, ...
    initialState.position_deg(1), initialState.position_deg(2), initialState.time_s);
goalBlocked = queryAzElTimeObstacle(obstacleField, ...
    goalState.position_deg(1), goalState.position_deg(2), goalState.time_s);
endpointBlocked = logical(startBlocked || goalBlocked);

% --- Generate distinct geometric candidates ----------------------------
% Visibility vertices lie on the protected boundary. Offset them by a
% documented fraction of the search resolution so smoothing, sampled path
% chords, and floating-point contact do not turn a nominal tangent into an
% obstacle crossing.
routingClearance_deg = 0.25 * options.VisibilitySampleStep_deg;
searchOptions = struct(...
    "CandidateClearance_deg", routingClearance_deg, ...
    "MaximumSnapshotsPerObstacle", ...
    options.MaximumVisibilitySnapshotsPerObstacle, ...
    "RequiredSnapshotTimes_s", ...
    options.TemporalRefinementSampleTimes_s, ...
    "DetectSnapshotEvents", options.DetectSnapshotEvents, ...
    "SnapshotProbeEdgeCount", options.SnapshotProbeEdgeCount, ...
    "SnapshotCountChangeThreshold", ...
    options.SnapshotCountChangeThreshold, ...
    "SnapshotEdgeRotationThreshold_deg", ...
    options.SnapshotEdgeRotationThreshold_deg, ...
    "SnapshotBoundaryMotionThreshold_deg", ...
    options.SnapshotBoundaryMotionThreshold_deg, ...
    "VisibilitySampleStep_deg", options.VisibilitySampleStep_deg, ...
    "ConnectivityPathCostChangeThreshold", ...
    options.VisibilityGraphCostChangeThreshold, ...
    "UseParallel", options.UseParallel, ...
    "Verbose", options.Verbose);

search = buildAzElVisibilityRoutes(obstacleField, initialState, ...
    goalState, searchOptions);
[candidateRoutes_deg, snapshotTime_s, graphIndex, consolidation] = ...
    collectRoutes(search.VisibilityGraphs, initialState, goalState, ...
    options.MaximumRetimedVisibilityRoutes);
candidateCount = numel(candidateRoutes_deg);

departureSearchTime_s = initialState.time_s;
if options.GoalTimeMode == "earliestarrival" && ...
        ~isempty(search.VisibilityGraphs)
    graphTime_s = [search.VisibilityGraphs.Time_s].';
    graphTime_s = graphTime_s( ...
        graphTime_s >= initialState.time_s & ...
        graphTime_s <= goalState.time_s);
    departureSearchTime_s = unique([departureSearchTime_s; graphTime_s]);
end

%% Section 3: Smooth, Retime & Check Every Candidate

candidateTimedPaths = cell(candidateCount, 1);
candidateSmoothPaths = cell(candidateCount, 1);
retimed = false(candidateCount, 1);
collisionFree = false(candidateCount, 1);
collisionPrefilterApplied = false(candidateCount, 1);
collisionPrefilterPassed = false(candidateCount, 1);
collisionPrefilterSampleCount = zeros(candidateCount, 1);
continuousCollisionChecked = false(candidateCount, 1);
collisionSkippedByArrivalBound = false(candidateCount, 1);
collisionValidationStatus = repmat("notRetimed", candidateCount, 1);
firstBlockingTime_s = nan(candidateCount, 1);
arrivalTime_s = inf(candidateCount, 1);
attemptArrivalTime_s = inf(candidateCount, 1);
pathLength_deg = zeros(candidateCount, 1);
candidateMessage = strings(candidateCount, 1);
bestContinuouslyValidatedArrivalTime_s = Inf;

for candidateIndex = 1:candidateCount
    route_deg = candidateRoutes_deg{candidateIndex};
    pathLength_deg(candidateIndex) = sum(vecnorm( ...
        diff(route_deg, 1, 1), 2, 2));

    try
        if ~routeWithinAzimuthPolicy(route_deg, options)
            error("planAzElMotion:AzimuthPolicy", ...
                "Candidate violates the configured azimuth interval.");
        end
        departureCandidateTime_s = departureSearchTime_s;
        if graphIndex(candidateIndex) > 0
            departureCandidateTime_s = unique([ ...
                initialState.time_s; snapshotTime_s(candidateIndex)]);
        end
        smoothPath = emptySmoothPath(route_deg);
        timedPath = emptyTimedPath( ...
            limits, options, "No departure time was feasible.");
        bestArrivalTime_s = Inf;
        fallbackArrivalTime_s = Inf;
        fallbackSmoothPath = smoothPath;
        fallbackTimedPath = timedPath;
        collisionCheck = emptyCollisionCheck();
        fallbackCollisionCheck = collisionCheck;
        previousDepartureTime_s = NaN;
        previousDepartureWasBlocked = false;
        for departureIndex = 1:numel(departureCandidateTime_s)
            departureTime_s = departureCandidateTime_s(departureIndex);
            arrivalValidationBound_s = ...
                bestContinuouslyValidatedArrivalTime_s;
            if candidateIndex <= ...
                    options.MinimumContinuouslyValidatedCandidates
                arrivalValidationBound_s = Inf;
            end
            [attemptSmoothPath, attemptTimedPath, attemptBlocked, ...
                attemptCollisionCheck] = ...
                evaluateDeparture(route_deg, obstacleField, ...
                initialState, goalState, limits, options, departureTime_s, ...
                arrivalValidationBound_s);
            if attemptTimedPath.Success
                departureArrivalTime_s = ...
                    attemptTimedPath.GoalLineInterceptTime_s;
                if departureArrivalTime_s < fallbackArrivalTime_s
                    fallbackArrivalTime_s = departureArrivalTime_s;
                    fallbackSmoothPath = attemptSmoothPath;
                    fallbackTimedPath = attemptTimedPath;
                    fallbackCollisionCheck = attemptCollisionCheck;
                end
                if attemptCollisionCheck.ContinuousPassed && ...
                        departureArrivalTime_s < bestArrivalTime_s
                    if previousDepartureWasBlocked
                        [attemptSmoothPath, attemptTimedPath, ~, ...
                            attemptCollisionCheck] = ...
                            refineClearDeparture( ...
                            route_deg, obstacleField, initialState, ...
                            goalState, limits, options, ...
                            previousDepartureTime_s, departureTime_s, ...
                            attemptSmoothPath, attemptTimedPath, ...
                            attemptBlocked, attemptCollisionCheck, ...
                            arrivalValidationBound_s);
                        departureArrivalTime_s = ...
                            attemptTimedPath.GoalLineInterceptTime_s;
                    end
                    bestArrivalTime_s = departureArrivalTime_s;
                    smoothPath = attemptSmoothPath;
                    timedPath = attemptTimedPath;
                    collisionCheck = attemptCollisionCheck;
                    bestContinuouslyValidatedArrivalTime_s = min( ...
                        bestContinuouslyValidatedArrivalTime_s, ...
                        departureArrivalTime_s);
                    break;
                end
            elseif ~fallbackTimedPath.Success
                fallbackSmoothPath = attemptSmoothPath;
                fallbackTimedPath = attemptTimedPath;
            end
            previousDepartureTime_s = departureTime_s;
            previousDepartureWasBlocked = ...
                attemptTimedPath.Success && any(attemptBlocked);
        end
        if ~isfinite(bestArrivalTime_s)
            smoothPath = fallbackSmoothPath;
            timedPath = fallbackTimedPath;
            collisionCheck = fallbackCollisionCheck;
        end
    catch candidateError
        smoothPath = emptySmoothPath(route_deg);
        timedPath = emptyTimedPath(limits, options, ...
            string(candidateError.message));
        collisionCheck = emptyCollisionCheck();
    end

    candidateTimedPaths{candidateIndex} = timedPath;
    candidateSmoothPaths{candidateIndex} = smoothPath;
    retimed(candidateIndex) = timedPath.Success;
    candidateMessage(candidateIndex) = timedPath.Message;
    collisionPrefilterApplied(candidateIndex) = ...
        collisionCheck.PrefilterApplied;
    collisionPrefilterPassed(candidateIndex) = ...
        collisionCheck.PrefilterPassed;
    collisionPrefilterSampleCount(candidateIndex) = ...
        collisionCheck.PrefilterSampleCount;
    continuousCollisionChecked(candidateIndex) = ...
        collisionCheck.ContinuousChecked;
    collisionSkippedByArrivalBound(candidateIndex) = ...
        collisionCheck.SkippedByArrivalBound;
    firstBlockingTime_s(candidateIndex) = ...
        collisionCheck.FirstBlockingTime_s;

    if collisionCheck.SkippedByArrivalBound
        collisionValidationStatus(candidateIndex) = "arrivalBoundPruned";
    elseif collisionCheck.ContinuousPassed
        collisionValidationStatus(candidateIndex) = "continuousClear";
    elseif collisionCheck.ContinuousChecked
        collisionValidationStatus(candidateIndex) = "continuousCollision";
    elseif collisionCheck.PrefilterApplied && ...
            ~collisionCheck.PrefilterPassed
        collisionValidationStatus(candidateIndex) = "prefilterCollision";
    end

    if timedPath.Success
        attemptArrivalTime_s(candidateIndex) = timedPath.GoalLineInterceptTime_s;
    end

    collisionFree(candidateIndex) = timedPath.Success && ...
        collisionCheck.ContinuousPassed;

    if collisionFree(candidateIndex)
        arrivalTime_s(candidateIndex) = timedPath.GoalLineInterceptTime_s;
    elseif timedPath.Success
        if collisionCheck.SkippedByArrivalBound
            candidateMessage(candidateIndex) = ...
                "Continuous collision validation was unnecessary because " + ...
                "this arrival cannot beat an already validated candidate.";
        else
            candidateMessage(candidateIndex) = ...
                "The complete timed trajectory intersects protected geometry.";
        end
    end
end

% --- Evaluate the direct request for diagnostics -----------------------
directFraction = linspace(0, 1, 501).';
directPosition_deg = initialState.position_deg + directFraction .* ...
    (goalState.position_deg - initialState.position_deg);
directTime_s = initialState.time_s + directFraction .* ...
    (goalState.time_s - initialState.time_s);
directBlocked = queryAzElTimedPathCollision(obstacleField, ...
    directTime_s, directPosition_deg, struct(...
    "TimePaddingSamples", options.CollisionTimePaddingSamples));

% --- Record every candidate attempt ------------------------------------
source = repmat("visibilityGraph", candidateCount, 1);
source(1) = "direct";
candidateDiagnostics = table((1:candidateCount).', source, ...
    snapshotTime_s, graphIndex, pathLength_deg, retimed, collisionFree, ...
    collisionPrefilterApplied, collisionPrefilterPassed, ...
    collisionPrefilterSampleCount, continuousCollisionChecked, ...
    collisionSkippedByArrivalBound, collisionValidationStatus, ...
    firstBlockingTime_s, attemptArrivalTime_s, arrivalTime_s, ...
    candidateMessage, ...
    'VariableNames', {'Index','Source','SnapshotTime_s','GraphIndex', ...
    'PathLength_deg','Retimed','CollisionFree', ...
    'CollisionPrefilterApplied','CollisionPrefilterPassed', ...
    'CollisionPrefilterSampleCount','ContinuousCollisionChecked', ...
    'CollisionSkippedByArrivalBound','CollisionValidationStatus', ...
    'FirstBlockingTime_s', ...
    'AttemptArrivalTime_s','ArrivalTime_s','Message'});

%% Section 4: Select & Independently Validate One Result

% --- Select the fastest feasible candidate -----------------------------
feasibleIndex = find(isfinite(arrivalTime_s) & ~endpointBlocked);
planningSucceeded = ~isempty(feasibleIndex);

if planningSucceeded
    ranking = [arrivalTime_s(feasibleIndex), pathLength_deg(feasibleIndex), feasibleIndex];
    [~, order] = sortrows(ranking, [1 2 3]);
    selectedCandidateIndex = feasibleIndex(order(1));
else
    % A failed result still returns the most informative attempted route:
    % prefer retimed motion, then earlier arrival, shorter geometry, and
    % finally the direct route and stable candidate order.
    failedRanking = [~retimed(:), attemptArrivalTime_s(:), ...
        pathLength_deg(:), graphIndex(:) > 0, (1:candidateCount).'];
    [~, failedOrder] = sortrows(failedRanking, [1 2 3 4 5]);
    selectedCandidateIndex = failedOrder(1);
end

selectedRoute_deg = candidateRoutes_deg{selectedCandidateIndex};
timedSlopePath = candidateTimedPaths{selectedCandidateIndex};
smoothPath = candidateSmoothPaths{selectedCandidateIndex};

% --- Preserve the best available trajectory for failure diagnostics ----
bestAttemptPosition_deg = selectedRoute_deg;
bestAttemptTime_s = repmat(snapshotTime_s(selectedCandidateIndex), ...
    size(selectedRoute_deg, 1), 1);

if ~isempty(timedSlopePath.time_s)
    bestAttemptPosition_deg = timedSlopePath.position_deg;
    bestAttemptTime_s = timedSlopePath.time_s;
end

selectedCollisionWasAlreadyValidated = planningSucceeded && ...
    collisionFree(selectedCandidateIndex) && ...
    continuousCollisionChecked(selectedCandidateIndex);
if selectedCollisionWasAlreadyValidated
    % Candidate selection already used the complete continuous public query
    % with the same protected geometry and boundary policy. Repeating that
    % identical query cannot add independent evidence; maintained examples
    % still perform their own result-level collision validation.
    bestAttemptProtectedBlocked = false( ...
        numel(bestAttemptTime_s), 1);
else
    bestAttemptProtectedBlocked = queryAzElTimedPathCollision( ...
        obstacleField, bestAttemptTime_s, bestAttemptPosition_deg, struct( ...
        "TimePaddingSamples", options.CollisionTimePaddingSamples, ...
        "BoundaryIsOccupied", false));
end

% --- Independently validate the selected timed trajectory --------------
validation = validatePlan(planningSucceeded, endpointBlocked, ...
    timedSlopePath, bestAttemptProtectedBlocked, goalState, limits, options);
success = validation.Passed;

message = "Adaptive visibility and certified spatial retiming succeeded.";
terminationReason = "goalReached";

if endpointBlocked
    message = "The protected geometry contains the start or goal.";
    terminationReason = "endpointBlocked";
elseif ~planningSucceeded
    message = "No candidate satisfies collision and motion constraints.";
    terminationReason = "noFeasibleCandidate";
elseif ~success
    message = "Independent post-validation failed: " + validation.Message;
    terminationReason = "validationFailed";
end

%% Section 5: Assemble Stable Output

elapsedPlanningTime_s = toc(planningTimer);

searchDiagnostics = struct(...
    "VisibilityGraphCount", numel(search.VisibilityGraphs), ...
    "SuccessfulVisibilityGraphCount", nnz([search.VisibilityGraphs.Success]), ...
    "VisibilityGraphs", search.VisibilityGraphs, ...
    "ConnectivityDiagnostics", search.ConnectivityDiagnostics, ...
    "ConnectivityChangeIntervals", ...
    search.ConnectivityChangeIntervals, ...
    "SnapshotDiagnostics", search.SnapshotDiagnostics, ...
    "CandidateRouteCount", candidateCount, ...
    "FeasibleCandidateCount", nnz(collisionFree), ...
    "SelectedCandidateIndex", selectedCandidateIndex, ...
    "RouteConsolidation", consolidation, ...
    "VisibilitySearchOptions", search.Options, ...
    "TimedRouteOptimalityCertified", false, ...
    "TimedRouteOptimalityScope", ...
    "Best continuously validated route found at evaluated snapshots", ...
    "CollisionValidationMode", options.CollisionValidationMode, ...
    "MaximumCollisionPrefilterSamples", ...
    options.MaximumCollisionPrefilterSamples, ...
    "MinimumContinuouslyValidatedCandidates", ...
    options.MinimumContinuouslyValidatedCandidates, ...
    "ContinuousCollisionRequiredForSuccess", true, ...
    "PrefilterAppliedCandidateCount", nnz(collisionPrefilterApplied), ...
    "PrefilterPassedCandidateCount", nnz( ...
    collisionPrefilterApplied & collisionPrefilterPassed), ...
    "ContinuouslyCheckedCandidateCount", ...
    nnz(continuousCollisionChecked), ...
    "ArrivalBoundPrunedCandidateCount", ...
    nnz(collisionSkippedByArrivalBound), ...
    "DepartureTimeCandidates_s", departureSearchTime_s, ...
    "DepartureTimeCandidateCount", numel(departureSearchTime_s), ...
    "ParallelExecution", search.ParallelExecution, ...
    "BestAttemptPosition_deg", bestAttemptPosition_deg, ...
    "BestAttemptTime_s", bestAttemptTime_s, ...
    "ElapsedPlanningTime_s", elapsedPlanningTime_s, ...
    "TerminationReason", terminationReason);

if options.Verbose
    fprintf("[AzEl] %s Candidate %d; elapsed %.3f s.\n", ...
        message, selectedCandidateIndex, elapsedPlanningTime_s);
end

result = struct(...
    "Success", success, ...
    "Message", message, ...
    "TerminationReason", terminationReason, ...
    "Options", options, ...
    "azElData", protectedAzElData, ...
    "originalAzElData", originalObstacleField.SourceAzElData, ...
    "obstacleField", obstacleField, ...
    "initialState", initialState, ...
    "goalState", goalState, ...
    "limits", limits, ...
    "obstacleSafetyMargins_deg", obstacleSafetyMargins_deg, ...
    "candidateReductionDiagnostics", search.CandidateReductionDiagnostics, ...
    "directPosition_deg", directPosition_deg, ...
    "directTime_s", directTime_s, ...
    "directBlocked", logical(directBlocked(:)), ...
    "candidateRoutes_deg", {candidateRoutes_deg}, ...
    "candidateDiagnostics", candidateDiagnostics, ...
    "selectedCandidateIndex", selectedCandidateIndex, ...
    "selectedRoute_deg", selectedRoute_deg, ...
    "smoothPath", smoothPath, ...
    "timedSlopePath", timedSlopePath, ...
    "goalLineInterceptTime_s", timedSlopePath.GoalLineInterceptTime_s, ...
    "SearchDiagnostics", searchDiagnostics, ...
    "ElapsedPlanningTime_s", elapsedPlanningTime_s, ...
    "Validation", validation);

result = applyAdaptiveTemporalRefinement(result, obstacles, initialState, ...
    goalState, limits, options, planningTimer);
end

%% Section 6: Local Functions

function result = applyAdaptiveTemporalRefinement(result, obstacles, ...
        initialState, goalState, limits, options, planningTimer)
%% Section 0: Header & Readme
% SYNTAX
%   result = applyAdaptiveTemporalRefinement( ...
%       result, obstacles, initialState, ...
%       goalState, limits, options, planningTimer)
%**************************************************************************
% PURPOSE
%   - Refine only unresolved source-time intervals that can still produce
%     a feasible or faster validated motion.
%   - Preserve explicit unknown intervals when a time or step budget stops
%     search before source-snapshot exhaustion.
%**************************************************************************
% INPUTS
%   - result (scalar struct)
%       Completed result at the current visibility-snapshot density.
%   - obstacles, initialState, goalState, limits, options
%       Original resolved planner request for the next refinement pass.
%   - planningTimer (tic identifier)
%       Timer started by the outermost current planner call.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Best validated result plus explicit per-pass refinement history.
%**************************************************************************
% UNITS
%   - Snapshot limits are counts, angles are degrees, and time is seconds.
%**************************************************************************
[intervals, intervalSummary] = temporalIntervalsForResult( ...
    result, initialState, goalState, limits, options);
currentRecord = temporalRefinementRecord( ...
    result, intervals, intervalSummary, "initial");
history = currentRecord;

failureNeedsRefinement = ~result.Success && ...
    result.TerminationReason == "noFeasibleCandidate";
successNeedsRefinement = result.Success && ...
    options.ContinueAfterFirstFeasible;
refinementRequested = failureNeedsRefinement || successNeedsRefinement;

stepBudgetAvailable = ...
    options.MaximumTemporalRefinementSteps > 0;
timeBudgetAvailable = ...
    toc(planningTimer) < options.MaximumPlanningTime_s;
[selectedIntervalIndex, refinementTime_s] = ...
    selectTemporalIntervalForRefinement(intervals);
hasRefinableInterval = selectedIntervalIndex > 0;

if ~(refinementRequested && stepBudgetAvailable && ...
        timeBudgetAvailable && hasRefinableInterval)
    if ~refinementRequested && result.Success && ...
            ~options.ContinueAfterFirstFeasible
        stopReason = "firstFeasibleAccepted";
    elseif ~refinementRequested
        stopReason = "outcomeNotRefinable";
    elseif ~stepBudgetAvailable
        stopReason = "refinementStepLimit";
    elseif ~timeBudgetAvailable
        stopReason = "planningTimeLimit";
    elseif ~hasRefinableInterval && ...
            intervalSummary.OmittedSourceSampleCount == 0 && ...
            intervalSummary.ContinuousCoverageCertified
        stopReason = "continuousCoverageCertified";
    elseif ~hasRefinableInterval && ...
            intervalSummary.OmittedSourceSampleCount == 0
        stopReason = "sourceSnapshotsExhaustedContinuousCoverageUnknown";
    elseif ~hasRefinableInterval
        stopReason = "noCompetitiveRefinableInterval";
    else
        stopReason = "noRefinementNeeded";
    end
    result = attachTemporalSearchDiagnostics( ...
        result, intervals, history, false, stopReason, ...
        options, toc(planningTimer));
    if ~result.Success && ...
            ~intervalSummary.ContinuousCoverageCertified && ...
            result.TerminationReason == "noFeasibleCandidate"
        result.Message = "No validated route was found, but moving " + ...
            "obstacle intervals remain without a continuous visibility " + ...
            "certificate.";
        result.TerminationReason = "temporalSearchIncomplete";
        result.SearchDiagnostics.TerminationReason = ...
            result.TerminationReason;
    end
    return;
end

refinementTrigger = "automaticAfterFailure";
if result.Success
    refinementTrigger = "continueAfterFirstFeasible";
elseif intervals(selectedIntervalIndex).CollisionDirected
    refinementTrigger = "collisionDirected";
elseif intervals(selectedIntervalIndex).ConnectivityChanged
    refinementTrigger = "connectivityDirected";
end

refinedOptions = options;
refinedOptions.TemporalRefinementSampleTimes_s = unique([ ...
    options.TemporalRefinementSampleTimes_s; refinementTime_s]);
% Each snapshot graph is independent. A recovery pass therefore needs
% only the earliest baseline snapshot plus the locally requested source
% times; rebuilding the original coarse graph set would repeat work
% without adding a candidate that the completed pass did not already test.
refinedOptions.MaximumVisibilitySnapshotsPerObstacle = 1;
if isfinite(options.MaximumTemporalRefinementSteps)
    refinedOptions.MaximumTemporalRefinementSteps = ...
        options.MaximumTemporalRefinementSteps - 1;
end
if isfinite(options.MaximumPlanningTime_s)
    refinedOptions.MaximumPlanningTime_s = max(eps, ...
        options.MaximumPlanningTime_s - toc(planningTimer));
end

intervals(selectedIntervalIndex).SelectedForRefinement = true;
intervals(selectedIntervalIndex).RefinementTime_s = refinementTime_s;
currentRecord.Trigger = refinementTrigger;
currentRecord.RefinedIntervalIndex = selectedIntervalIndex;
currentRecord.RefinementTime_s = refinementTime_s;

if options.Verbose
    fprintf("[AzEl][temporal refinement] %s interval [%.3f, %.3f] " + ...
        "at t=%.3f s; %d unresolved source samples remain.\n", ...
        refinementTrigger, ...
        intervals(selectedIntervalIndex).StartTime_s, ...
        intervals(selectedIntervalIndex).EndTime_s, ...
        refinementTime_s, intervalSummary.OmittedSourceSampleCount);
end

refinedResult = planAzElMotion(obstacles, initialState, goalState, ...
    limits, refinedOptions);
refinedHistory = ...
    refinedResult.SearchDiagnostics.TemporalRefinementHistory;

keepRefinedResult = false;
if ~result.Success
    keepRefinedResult = true;
elseif refinedResult.Success
    % Compare the timing quantity controlled by GoalTimeMode, then use
    % route length only as a deterministic near-equal tie breaker.
    if options.GoalTimeMode == "fixedarrival"
        refinedTime_s = ...
            refinedResult.timedSlopePath.MinimumMotionDuration_s;
        currentTime_s = result.timedSlopePath.MinimumMotionDuration_s;
    else
        refinedTime_s = refinedResult.goalLineInterceptTime_s;
        currentTime_s = result.goalLineInterceptTime_s;
    end
    comparisonTolerance_s = 1e-9 * max( ...
        1, max(abs([refinedTime_s currentTime_s])));
    if refinedTime_s < currentTime_s - comparisonTolerance_s
        keepRefinedResult = true;
    elseif abs(refinedTime_s - currentTime_s) <= ...
            comparisonTolerance_s
        refinedLength_deg = sum(vecnorm(diff( ...
            refinedResult.selectedRoute_deg, 1, 1), 2, 2));
        currentLength_deg = sum(vecnorm(diff( ...
            result.selectedRoute_deg, 1, 1), 2, 2));
        keepRefinedResult = ...
            refinedLength_deg < currentLength_deg - 1e-9;
    end
end

if keepRefinedResult
    currentRecord.Selected = false;
    history = [currentRecord; refinedHistory];
    result = refinedResult;
else
    for recordIndex = 1:numel(refinedHistory)
        refinedHistory(recordIndex).Selected = false;
    end
    currentRecord.Selected = true;
    history = [currentRecord; refinedHistory];
end

for recordIndex = 1:numel(history)
    history(recordIndex).PassIndex = recordIndex;
end
result = attachTemporalSearchDiagnostics( ...
    result, refinedResult.SearchDiagnostics.TemporalIntervals, history, ...
    true, refinedResult.SearchDiagnostics.TemporalRefinementStoppedBecause, ...
    options, toc(planningTimer));
end

function record = temporalRefinementRecord( ...
        result, intervals, intervalSummary, trigger)
%% Section 0: Header & Readme
% SYNTAX
%   record = temporalRefinementRecord( ...
%       result, intervals, intervalSummary, trigger)
%**************************************************************************
% PURPOSE
%   - Summarize one complete adaptive temporal-search pass without
%     retaining duplicate graph payloads.
%**************************************************************************
% INPUTS
%   - result (scalar planner result)
%   - intervals (structure array)
%       Explicit clear, blocked, or unresolved temporal intervals.
%   - intervalSummary (scalar struct)
%       Counts and lower bounds derived from intervals.
%   - trigger (scalar string)
%       Reason that produced this planning pass.
%**************************************************************************
% OUTPUTS
%   - record (scalar struct)
%       Stable refinement-pass diagnostics and selection state.
%**************************************************************************
% UNITS
%   - Path length is degrees and elapsed values are seconds.
%**************************************************************************
arrivalTime_s = result.goalLineInterceptTime_s;
minimumMotionDuration_s = result.timedSlopePath.MinimumMotionDuration_s;
selectedPathLength_deg = Inf;
if size(result.selectedRoute_deg, 1) >= 2
    selectedPathLength_deg = sum(vecnorm( ...
        diff(result.selectedRoute_deg, 1, 1), 2, 2));
end

record = struct( ...
    "PassIndex", 1, ...
    "Trigger", string(trigger), ...
    "Success", result.Success, ...
    "TerminationReason", result.TerminationReason, ...
    "ArrivalTime_s", arrivalTime_s, ...
    "MinimumMotionDuration_s", minimumMotionDuration_s, ...
    "SelectedPathLength_deg", selectedPathLength_deg, ...
    "VisibilityGraphCount", ...
    result.SearchDiagnostics.VisibilityGraphCount, ...
    "CandidateRouteCount", ...
    result.SearchDiagnostics.CandidateRouteCount, ...
    "IntervalCount", numel(intervals), ...
    "UnresolvedIntervalCount", ...
    intervalSummary.UnresolvedIntervalCount, ...
    "RefinableIntervalCount", ...
    intervalSummary.RefinableIntervalCount, ...
    "CompetitiveIntervalCount", ...
    intervalSummary.CompetitiveIntervalCount, ...
    "ClearEvidenceIntervalCount", ...
    intervalSummary.ClearEvidenceIntervalCount, ...
    "BlockedEvidenceIntervalCount", ...
    intervalSummary.BlockedEvidenceIntervalCount, ...
    "UnknownEvidenceIntervalCount", ...
    intervalSummary.UnknownEvidenceIntervalCount, ...
    "OmittedSourceSampleCount", ...
    intervalSummary.OmittedSourceSampleCount, ...
    "ContinuousCoverageCertified", ...
    intervalSummary.ContinuousCoverageCertified, ...
    "RefinedIntervalIndex", 0, ...
    "RefinementTime_s", NaN, ...
    "ElapsedPlanningTime_s", result.ElapsedPlanningTime_s, ...
    "Selected", true);
end

function [intervals, summary] = temporalIntervalsForResult( ...
        result, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   [intervals, summary] = temporalIntervalsForResult( ...
%       result, initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Partition the request horizon at evaluated graph times and expose all
%     remaining source samples and continuous-time coverage uncertainty.
%**************************************************************************
% INPUTS
%   - result, initialState, goalState, limits, options (scalar structs)
%       Current result and resolved planning request.
%**************************************************************************
% OUTPUTS
%   - intervals (structure array)
%       Per-interval state, priority, and refinement seed.
%   - summary (scalar struct)
%       Aggregate coverage and admissible arrival-bound diagnostics.
%**************************************************************************
% UNITS
%   - Interval bounds, sample times, and arrival bounds are seconds.
%**************************************************************************
intervalTemplate = struct( ...
    "IntervalIndex", 0, ...
    "StartTime_s", NaN, ...
    "EndTime_s", NaN, ...
    "Status", "unknown", ...
    "CandidateEvidenceState", "unknown", ...
    "OmittedSourceTimes_s", zeros(0, 1), ...
    "OmittedSourceSampleCount", 0, ...
    "Competitive", false, ...
    "ConnectivityChanged", false, ...
    "CollisionDirected", false, ...
    "CollisionTime_s", NaN, ...
    "ArrivalLowerBound_s", Inf, ...
    "Priority", -Inf, ...
    "Refinable", false, ...
    "SelectedForRefinement", false, ...
    "RefinementTime_s", NaN);

evaluatedTimes_s = initialState.time_s;
visibilityGraphs = result.SearchDiagnostics.VisibilityGraphs;
if ~isempty(visibilityGraphs)
    evaluatedTimes_s = [evaluatedTimes_s; ...
        reshape([visibilityGraphs.Time_s], [], 1)];
end
evaluatedTimes_s = unique([evaluatedTimes_s; goalState.time_s]);
evaluatedTimes_s = evaluatedTimes_s( ...
    evaluatedTimes_s >= initialState.time_s & ...
    evaluatedTimes_s <= goalState.time_s);

sourceTimes_s = zeros(0, 1);
for obstacleIndex = 1:numel(result.obstacleField.Obstacles)
    obstacleTimes_s = double( ...
        result.obstacleField.Obstacles(obstacleIndex).TimeSeconds(:));
    sourceTimes_s = [sourceTimes_s; obstacleTimes_s( ...
        obstacleTimes_s >= initialState.time_s & ...
        obstacleTimes_s <= goalState.time_s)]; %#ok<AGROW>
end
sourceTimes_s = unique(sourceTimes_s);

isStaticField = obstacleFieldIsStatic(result.obstacleField);
intervalCount = max(0, numel(evaluatedTimes_s) - 1);
intervals = repmat(intervalTemplate, intervalCount, 1);
minimumDirectDuration_s = max(abs( ...
    goalState.position_deg - initialState.position_deg) ./ ...
    limits.maxVelocity_deg_s);
incumbentArrival_s = Inf;
if result.Success
    incumbentArrival_s = result.goalLineInterceptTime_s;
end

changeIntervals = result.SearchDiagnostics.ConnectivityChangeIntervals;
blockingTimes_s = zeros(0, 1);
if ~isempty(result.candidateDiagnostics) && ...
        ismember("FirstBlockingTime_s", ...
        string(result.candidateDiagnostics.Properties.VariableNames))
    blockingTimes_s = result.candidateDiagnostics.FirstBlockingTime_s;
    blockingTimes_s = blockingTimes_s(isfinite(blockingTimes_s));
end

for intervalIndex = 1:intervalCount
    startTime_s = evaluatedTimes_s(intervalIndex);
    endTime_s = evaluatedTimes_s(intervalIndex + 1);
    omittedMask = sourceTimes_s > startTime_s & ...
        sourceTimes_s < endTime_s;
    omittedTimes_s = sourceTimes_s(omittedMask);

    connectivityChanged = false;
    if ~isempty(changeIntervals)
        connectivityChanged = any( ...
            [changeIntervals.EarlierTime_s] <= endTime_s & ...
            [changeIntervals.LaterTime_s] >= startTime_s);
    end
    intervalBlockingTimes_s = blockingTimes_s( ...
        blockingTimes_s >= startTime_s & ...
        blockingTimes_s <= endTime_s);
    collisionDirected = ~isempty(intervalBlockingTimes_s);
    collisionTime_s = NaN;
    if collisionDirected
        collisionTime_s = intervalBlockingTimes_s(1);
    end

    arrivalLowerBound_s = max(initialState.time_s, startTime_s) + ...
        minimumDirectDuration_s;
    competitive = ~result.Success || ...
        arrivalLowerBound_s < ...
        incumbentArrival_s - options.OptimalityTolerance_s;
    refinable = ~isempty(omittedTimes_s) && competitive;

    if isStaticField
        status = "certifiedStatic";
    elseif isempty(omittedTimes_s)
        status = "unknownContinuousVisibility";
    else
        status = "unknownSourceSamples";
    end

    candidateEvidenceState = "unknown";
    if result.Success && ~isempty(result.timedSlopePath.time_s) && ...
            result.timedSlopePath.time_s(1) <= endTime_s && ...
            result.timedSlopePath.time_s(end) >= startTime_s
        candidateEvidenceState = "clear";
    elseif collisionDirected
        candidateEvidenceState = "blocked";
    end

    priority = double(competitive) * 10 + ...
        double(connectivityChanged) * 100 + ...
        double(collisionDirected) * 1000 + ...
        numel(omittedTimes_s);
    intervals(intervalIndex) = intervalTemplate;
    intervals(intervalIndex).IntervalIndex = intervalIndex;
    intervals(intervalIndex).StartTime_s = startTime_s;
    intervals(intervalIndex).EndTime_s = endTime_s;
    intervals(intervalIndex).Status = status;
    intervals(intervalIndex).CandidateEvidenceState = ...
        candidateEvidenceState;
    intervals(intervalIndex).OmittedSourceTimes_s = omittedTimes_s;
    intervals(intervalIndex).OmittedSourceSampleCount = ...
        numel(omittedTimes_s);
    intervals(intervalIndex).Competitive = competitive;
    intervals(intervalIndex).ConnectivityChanged = connectivityChanged;
    intervals(intervalIndex).CollisionDirected = collisionDirected;
    intervals(intervalIndex).CollisionTime_s = collisionTime_s;
    intervals(intervalIndex).ArrivalLowerBound_s = arrivalLowerBound_s;
    intervals(intervalIndex).Priority = priority;
    intervals(intervalIndex).Refinable = refinable;
end

unresolvedMask = string({intervals.Status}) ~= "certifiedStatic";
refinableMask = [intervals.Refinable];
competitiveMask = [intervals.Competitive];
candidateEvidenceState = string({intervals.CandidateEvidenceState});
summary = struct( ...
    "IntervalCount", intervalCount, ...
    "UnresolvedIntervalCount", nnz(unresolvedMask), ...
    "RefinableIntervalCount", nnz(refinableMask), ...
    "CompetitiveIntervalCount", nnz(competitiveMask), ...
    "ClearEvidenceIntervalCount", ...
    nnz(candidateEvidenceState == "clear"), ...
    "BlockedEvidenceIntervalCount", ...
    nnz(candidateEvidenceState == "blocked"), ...
    "UnknownEvidenceIntervalCount", ...
    nnz(candidateEvidenceState == "unknown"), ...
    "OmittedSourceSampleCount", sum( ...
    [intervals.OmittedSourceSampleCount]), ...
    "ContinuousCoverageCertified", isStaticField);
end

function isStatic = obstacleFieldIsStatic(obstacleField)
%% Section 0: Header & Readme
% SYNTAX
%   isStatic = obstacleFieldIsStatic(obstacleField)
%**************************************************************************
% PURPOSE
%   - Certify that every packed obstacle slice repeats identical geometry.
%**************************************************************************
% INPUTS
%   - obstacleField (scalar packed obstacle field)
%**************************************************************************
% OUTPUTS
%   - isStatic (logical scalar)
%**************************************************************************
% UNITS
%   - Geometry is compared exactly in degrees.
%**************************************************************************
isStatic = true;
for obstacleIndex = 1:numel(obstacleField.Obstacles)
    obstacle = obstacleField.Obstacles(obstacleIndex);
    for sampleIndex = 2:obstacle.SampleCount
        previousVertexRows = double(obstacle.SliceOffsets(sampleIndex)): ...
            double(obstacle.SliceOffsets(sampleIndex + 1) - 1);
        firstVertexRows = double(obstacle.SliceOffsets(1)): ...
            double(obstacle.SliceOffsets(2) - 1);
        previousEdgeRows = double(obstacle.EdgeOffsets(sampleIndex)): ...
            double(obstacle.EdgeOffsets(sampleIndex + 1) - 1);
        firstEdgeRows = double(obstacle.EdgeOffsets(1)): ...
            double(obstacle.EdgeOffsets(2) - 1);
        sameVertices = numel(previousVertexRows) == numel(firstVertexRows) && ...
            isequal(obstacle.AzimuthDeg(previousVertexRows), ...
            obstacle.AzimuthDeg(firstVertexRows)) && ...
            isequal(obstacle.ElevationDeg(previousVertexRows), ...
            obstacle.ElevationDeg(firstVertexRows));
        sameEdges = numel(previousEdgeRows) == numel(firstEdgeRows) && ...
            isequal(obstacle.EdgeStartAzimuthDeg(previousEdgeRows), ...
            obstacle.EdgeStartAzimuthDeg(firstEdgeRows)) && ...
            isequal(obstacle.EdgeStartElevationDeg(previousEdgeRows), ...
            obstacle.EdgeStartElevationDeg(firstEdgeRows)) && ...
            isequal(obstacle.EdgeEndAzimuthDeg(previousEdgeRows), ...
            obstacle.EdgeEndAzimuthDeg(firstEdgeRows)) && ...
            isequal(obstacle.EdgeEndElevationDeg(previousEdgeRows), ...
            obstacle.EdgeEndElevationDeg(firstEdgeRows));
        if ~(sameVertices && sameEdges)
            isStatic = false;
            return;
        end
    end
end
end

function [intervalIndex, refinementTime_s] = ...
        selectTemporalIntervalForRefinement(intervals)
%% Section 0: Header & Readme
% SYNTAX
%   [intervalIndex, refinementTime_s] = ...
%       selectTemporalIntervalForRefinement(intervals)
%**************************************************************************
% PURPOSE
%   - Choose the highest-value unresolved interval and one local source
%     snapshot, favoring the first exact collision when available.
%**************************************************************************
% INPUTS
%   - intervals (structure array)
%**************************************************************************
% OUTPUTS
%   - intervalIndex (nonnegative integer scalar)
%   - refinementTime_s (scalar seconds or NaN)
%**************************************************************************
% UNITS
%   - Refinement time is seconds.
%**************************************************************************
intervalIndex = 0;
refinementTime_s = NaN;
if isempty(intervals)
    return;
end
refinableIndex = find([intervals.Refinable]);
if isempty(refinableIndex)
    return;
end
[~, localIndex] = max([intervals(refinableIndex).Priority]);
intervalIndex = refinableIndex(localIndex);
interval = intervals(intervalIndex);
targetTime_s = 0.5 * (interval.StartTime_s + interval.EndTime_s);
if interval.CollisionDirected
    targetTime_s = interval.CollisionTime_s;
end
[~, sourceIndex] = min(abs( ...
    interval.OmittedSourceTimes_s - targetTime_s));
refinementTime_s = interval.OmittedSourceTimes_s(sourceIndex);
end

function result = attachTemporalSearchDiagnostics( ...
        result, intervals, history, refinementPerformed, stopReason, ...
        options, elapsedPlanningTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   result = attachTemporalSearchDiagnostics( ...
%       result, intervals, history, refinementPerformed, stopReason, ...
%       options, elapsedPlanningTime_s)
%**************************************************************************
% PURPOSE
%   - Publish stable anytime-search coverage, lower-bound, and stopping
%     diagnostics on both success and failure.
%**************************************************************************
% INPUTS
%   - result (scalar planner result)
%   - intervals, history (structure arrays)
%   - refinementPerformed (logical scalar)
%   - stopReason (scalar string)
%   - options (resolved planner options)
%   - elapsedPlanningTime_s (nonnegative scalar seconds)
%**************************************************************************
% OUTPUTS
%   - result (scalar planner result)
%**************************************************************************
% UNITS
%   - Elapsed, lower-bound, and gap values are seconds.
%**************************************************************************
unresolvedMask = string({intervals.Status}) ~= "certifiedStatic";
refinableMask = [intervals.Refinable];
competitiveMask = [intervals.Competitive];
candidateEvidenceState = string({intervals.CandidateEvidenceState});
omittedCount = sum([intervals.OmittedSourceSampleCount]);
continuousCertified = isempty(intervals) || ~any(unresolvedMask);
arrivalLowerBound_s = Inf;
if ~isempty(intervals)
    arrivalLowerBound_s = min([intervals.ArrivalLowerBound_s]);
end
bestArrival_s = Inf;
optimalityGap_s = Inf;
if result.Success
    bestArrival_s = result.goalLineInterceptTime_s;
    optimalityGap_s = max(0, bestArrival_s - arrivalLowerBound_s);
end

result.SearchDiagnostics.TemporalIntervals = intervals;
result.SearchDiagnostics.TemporalRefinementHistory = history;
result.SearchDiagnostics.TemporalRefinementPerformed = ...
    logical(refinementPerformed);
result.SearchDiagnostics.TemporalRefinementStoppedBecause = ...
    string(stopReason);
result.SearchDiagnostics.UnresolvedTemporalIntervalCount = ...
    nnz(unresolvedMask);
result.SearchDiagnostics.RefinableTemporalIntervalCount = ...
    nnz(refinableMask);
result.SearchDiagnostics.CompetitiveTemporalIntervalCount = ...
    nnz(competitiveMask);
result.SearchDiagnostics.ClearEvidenceIntervalCount = ...
    nnz(candidateEvidenceState == "clear");
result.SearchDiagnostics.BlockedEvidenceIntervalCount = ...
    nnz(candidateEvidenceState == "blocked");
result.SearchDiagnostics.UnknownEvidenceIntervalCount = ...
    nnz(candidateEvidenceState == "unknown");
result.SearchDiagnostics.OmittedSourceSampleCount = omittedCount;
result.SearchDiagnostics.AllSourceSnapshotsEvaluated = omittedCount == 0;
result.SearchDiagnostics.ContinuousTemporalCoverageCertified = ...
    continuousCertified;
result.SearchDiagnostics.TemporalCoverageComplete = ...
    continuousCertified;
result.SearchDiagnostics.OptimalityProven = false;
result.SearchDiagnostics.ArrivalLowerBound_s = arrivalLowerBound_s;
result.SearchDiagnostics.BestValidatedArrival_s = bestArrival_s;
result.SearchDiagnostics.OptimalityGap_s = optimalityGap_s;
historyTimes_s = reshape([history.RefinementTime_s], [], 1);
historyTimes_s = historyTimes_s(isfinite(historyTimes_s));
result.SearchDiagnostics.AdaptiveRefinementSampleTimes_s = unique([ ...
    options.TemporalRefinementSampleTimes_s; historyTimes_s]);
result.SearchDiagnostics.ElapsedPlanningTime_s = elapsedPlanningTime_s;
result.ElapsedPlanningTime_s = elapsedPlanningTime_s;
end

function [smoothPath, timedPath, blocked, collisionCheck] = ...
        evaluateDeparture( ...
        route_deg, obstacleField, initialState, goalState, limits, ...
        options, departureTime_s, arrivalValidationBound_s)
%% Section 0: Header & Readme
% SYNTAX
%   [smoothPath, timedPath, blocked, collisionCheck] = ...
%       evaluateDeparture( ...
%       route_deg, obstacleField, initialState, goalState, limits, ...
%       options, departureTime_s, arrivalValidationBound_s)
%**************************************************************************
% PURPOSE
%   - Smooth, retime, prefilter, and continuously validate one departure.
%**************************************************************************
% INPUTS
%   - route_deg (N-by-2 numeric), obstacleField (scalar struct)
%   - initialState, goalState, limits, options (scalar structs)
%       Candidate geometry, protected obstacles, and resolved request.
%   - departureTime_s, arrivalValidationBound_s (numeric scalars)
%       Motion start and incumbent continuously validated arrival.
%**************************************************************************
% OUTPUTS
%   - smoothPath, timedPath, collisionCheck (scalar structs)
%   - blocked (logical vector)
%       Candidate motion and complete collision-validation evidence.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds.
%**************************************************************************
try
    smoothPath = smoothRoute( ...
        route_deg, obstacleField, departureTime_s, options);
    timedPath = retimeSpatialPath( ...
        smoothPath, initialState, goalState, limits, options, ...
        departureTime_s);
catch departureError
    smoothPath = emptySmoothPath(route_deg);
    timedPath = emptyTimedPath( ...
        limits, options, string(departureError.message));
end

blocked = false(0, 1);
collisionCheck = emptyCollisionCheck();

if ~timedPath.Success
    return;
end

if timedPath.GoalLineInterceptTime_s >= arrivalValidationBound_s
    collisionCheck.SkippedByArrivalBound = true;
    collisionCheck.ArrivalValidationBound_s = arrivalValidationBound_s;
    return;
end

if options.CollisionValidationMode == "hybrid"
    collisionCheck.PrefilterApplied = true;
    pathSampleCount = numel(timedPath.time_s);
    retainedSampleCount = min(pathSampleCount, ...
        options.MaximumCollisionPrefilterSamples);
    retainedIndex = unique(round(linspace( ...
        1, pathSampleCount, retainedSampleCount))).';
    prefilterBlocked = queryAzElTimeObstacle(obstacleField, ...
        timedPath.position_deg(retainedIndex, 1), ...
        timedPath.position_deg(retainedIndex, 2), ...
        timedPath.time_s(retainedIndex), struct( ...
        "TimePaddingSamples", options.CollisionTimePaddingSamples, ...
        "BoundaryIsOccupied", false));
    collisionCheck.PrefilterSampleCount = numel(retainedIndex);
    collisionCheck.PrefilterPassed = ~any(prefilterBlocked);

    if ~collisionCheck.PrefilterPassed
        blocked = logical(prefilterBlocked(:));
        return;
    end
else
    collisionCheck.PrefilterPassed = true;
end

collisionCheck.ContinuousChecked = true;
[blocked, collisionDetails] = queryAzElTimedPathCollision( ...
    obstacleField, timedPath.time_s, timedPath.position_deg, struct( ...
    "TimePaddingSamples", options.CollisionTimePaddingSamples, ...
    "BoundaryIsOccupied", false));
collisionCheck.ContinuousPassed = ~any(blocked);
firstBlockedRow = find(blocked, 1, "first");
if ~isempty(firstBlockedRow)
    collisionCheck.FirstBlockingTime_s = ...
        collisionDetails.time_s(firstBlockedRow);
    collisionCheck.BlockingObstacleIndex = double( ...
        collisionDetails.BlockingObstacleIndex(firstBlockedRow));
    collisionCheck.BlockingSliceIndex = double( ...
        collisionDetails.BlockingSliceIndex(firstBlockedRow));
end
end

function collisionCheck = emptyCollisionCheck()
%% Section 0: Header & Readme
% SYNTAX
%   collisionCheck = emptyCollisionCheck()
%**************************************************************************
% PURPOSE
%   - Return the stable record for the two-stage collision policy.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - collisionCheck (scalar struct)
%       Prefilter and continuous-validation status for one attempt.
%**************************************************************************
% UNITS
%   - PrefilterSampleCount is a dimensionless count.
%**************************************************************************
collisionCheck = struct( ...
    "PrefilterApplied", false, ...
    "PrefilterSampleCount", 0, ...
    "PrefilterPassed", false, ...
    "ContinuousChecked", false, ...
    "ContinuousPassed", false, ...
    "SkippedByArrivalBound", false, ...
    "ArrivalValidationBound_s", Inf, ...
    "FirstBlockingTime_s", NaN, ...
    "BlockingObstacleIndex", 0, ...
    "BlockingSliceIndex", 0);
end

function [clearSmoothPath, clearTimedPath, clearBlocked, ...
        clearCollisionCheck] = ...
        refineClearDeparture(route_deg, obstacleField, initialState, ...
        goalState, limits, options, blockedDepartureTime_s, ...
        clearDepartureTime_s, clearSmoothPath, clearTimedPath, ...
        clearBlocked, clearCollisionCheck, arrivalValidationBound_s)
%% Section 0: Header & Readme
% SYNTAX
%   [clearSmoothPath, clearTimedPath, clearBlocked, ...
%       clearCollisionCheck] = ...
%       refineClearDeparture(route_deg, obstacleField, initialState, ...
%       goalState, limits, options, blockedDepartureTime_s, ...
%       clearDepartureTime_s, clearSmoothPath, clearTimedPath, ...
%       clearBlocked, clearCollisionCheck, arrivalValidationBound_s)
%**************************************************************************
% PURPOSE
%   - Bisect one adjacent blocked-to-clear departure interval so obstacle
%     sample times do not quantize the earliest usable motion start.
%**************************************************************************
% INPUTS
%   - route_deg, obstacleField, initialState, goalState, limits, options
%       Same candidate request consumed by evaluateDeparture.
%   - blockedDepartureTime_s, clearDepartureTime_s (numeric scalars)
%   - clearSmoothPath, clearTimedPath, clearBlocked, clearCollisionCheck
%   - arrivalValidationBound_s (numeric scalar)
%       Collision bracket, validated upper result, and incumbent arrival.
%**************************************************************************
% OUTPUTS
%   - clearSmoothPath, clearTimedPath, clearBlocked, clearCollisionCheck
%       Earliest clear result found to the stated time tolerance.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds.
%**************************************************************************
timeTolerance_s = max(1e-3, ...
    1e-8 * max(1, abs(clearDepartureTime_s)));
maximumRefinementCount = 24;

for refinementIndex = 1:maximumRefinementCount
    if clearDepartureTime_s - blockedDepartureTime_s <= timeTolerance_s
        break;
    end
    middleDepartureTime_s = 0.5 * ( ...
        blockedDepartureTime_s + clearDepartureTime_s);
    [middleSmoothPath, middleTimedPath, middleBlocked, ...
        middleCollisionCheck] = ...
        evaluateDeparture(route_deg, obstacleField, initialState, ...
        goalState, limits, options, middleDepartureTime_s, ...
        arrivalValidationBound_s);
    if middleTimedPath.Success && ~any(middleBlocked) && ...
            middleCollisionCheck.ContinuousPassed
        clearDepartureTime_s = middleDepartureTime_s;
        clearSmoothPath = middleSmoothPath;
        clearTimedPath = middleTimedPath;
        clearBlocked = middleBlocked;
        clearCollisionCheck = middleCollisionCheck;
    else
        blockedDepartureTime_s = middleDepartureTime_s;
    end
end
end

% --- Options And Input Normalization -----------------------------------
function options = plannerDefaults()
%% Section 0: Header & Readme
% SYNTAX
%   options = plannerDefaults()
%**************************************************************************
% PURPOSE
%   - Return the argument-independent public planner options.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - options (scalar struct)
%       Complete defaults for search, smoothing, timing, and diagnostics.
%**************************************************************************
% UNITS
%   - Angles are degrees and time is seconds.
%**************************************************************************
options = struct(...
    "GoalTimeMode", "earliestArrival", ...
    "SampleTime_s", 0.05, ...
    "TurnRadius_deg", 1.0, ...
    "CollisionTimePaddingSamples", 1, ...
    "AllowAzimuthWrapping", false, ...
    "AzimuthInterval_deg", [-180 180], ...
    "VisibilitySampleStep_deg", 0.10, ...
    "MaximumVisibilitySnapshotsPerObstacle", 4, ...
    "DetectSnapshotEvents", true, ...
    "SnapshotProbeEdgeCount", 4, ...
    "SnapshotCountChangeThreshold", 0.10, ...
    "SnapshotEdgeRotationThreshold_deg", 5, ...
    "SnapshotBoundaryMotionThreshold_deg", 1, ...
    "MaximumRetimedVisibilityRoutes", 12, ...
    "CollisionValidationMode", "hybrid", ...
    "MaximumCollisionPrefilterSamples", 80, ...
    "MinimumContinuouslyValidatedCandidates", 2, ...
    "ContinueAfterFirstFeasible", false, ...
    "VisibilityGraphCostChangeThreshold", 0.05, ...
    "MaximumTemporalRefinementSteps", Inf, ...
    "MaximumPlanningTime_s", Inf, ...
    "OptimalityTolerance_s", 0.05, ...
    "TemporalRefinementSampleTimes_s", zeros(0, 1), ...
    "UseParallel", false, ...
    "Verbose", false);
end

function options = resolveOptions(defaults, overrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = resolveOptions(defaults, overrides)
%**************************************************************************
% PURPOSE
%   - Merge partial overrides, warn once for unknown fields, and validate.
%**************************************************************************
% INPUTS
%   - defaults, overrides (scalar structs)
%       Complete defaults and a partial public option structure.
%**************************************************************************
% OUTPUTS
%   - options (scalar struct)
%       Fully resolved and normalized planner options.
%**************************************************************************
% UNITS
%   - Units follow the suffix of each option field.
%**************************************************************************
if ~isstruct(overrides) || ~isscalar(overrides)
    error("planAzElMotion:InvalidOptions", "optionOverrides must be a scalar struct.");
end

movedMarginNames = intersect(fieldnames(overrides), ...
    {'SafetyMarginDeg', 'RoundingClearance_deg'}, "stable");
if ~isempty(movedMarginNames)
    error("planAzElMotion:SafetyMarginMoved", ...
        "Safety margins belong to obstacle data. Remove %s and pass " + ...
        "each margin to makeAzElObstacleData.", ...
        strjoin(string(movedMarginNames), ", "));
end

options = defaults;
names = fieldnames(overrides);
unknown = strings(0, 1);

for optionIndex = 1:numel(names)
    name = names{optionIndex};
    if isfield(defaults, name)
        if ~isempty(overrides.(name))
            options.(name) = overrides.(name);
        end
    else
        unknown(end + 1, 1) = string(name); %#ok<AGROW>
    end
end

if ~isempty(unknown)
    warning("planAzElMotion:UnknownOptions", ...
        "Ignored unknown option fields: %s. No behavior changed.", strjoin(unknown, ", "));
end

options.GoalTimeMode = lower(string(options.GoalTimeMode));
if ~isscalar(options.GoalTimeMode) || ...
        ~any(options.GoalTimeMode == ["earliestarrival" "fixedarrival"])
    error("planAzElMotion:InvalidGoalTimeMode", ...
        "GoalTimeMode must be earliestArrival or fixedArrival.");
end

options.CollisionValidationMode = lower(string( ...
    options.CollisionValidationMode));
if ~isscalar(options.CollisionValidationMode) || ...
        ~any(options.CollisionValidationMode == ["hybrid" "continuous"])
    error("planAzElMotion:InvalidCollisionValidationMode", ...
        "CollisionValidationMode must be hybrid or continuous.");
end

options.UseParallel = normalizeParallelOption(options.UseParallel);

logicalNames = ["AllowAzimuthWrapping" "DetectSnapshotEvents" ...
    "ContinueAfterFirstFeasible" "Verbose"];
for optionIndex = 1:numel(logicalNames)
    name = logicalNames(optionIndex);
    value = options.(name);
    if ~(islogical(value) && isscalar(value)) && ~(isnumeric(value) && isscalar(value) && ...
            isfinite(value) && any(value == [0 1]))
        error("planAzElMotion:InvalidLogicalOption", ...
            "%s must be scalar logical or binary numeric.", name);
    end
    options.(name) = logical(value);
end

positiveNames = ["SampleTime_s" "TurnRadius_deg" ...
    "VisibilitySampleStep_deg"];
for optionIndex = 1:numel(positiveNames)
    validateattributes(options.(positiveNames(optionIndex)), {'numeric'}, ...
        {'real','finite','scalar','positive'});
end

validateattributes(options.MaximumRetimedVisibilityRoutes, ...
    {'numeric'}, {'real', 'scalar', 'positive'});
if isfinite(options.MaximumRetimedVisibilityRoutes) && ...
        fix(options.MaximumRetimedVisibilityRoutes) ~= ...
        options.MaximumRetimedVisibilityRoutes
    error("planAzElMotion:InvalidRouteLimit", ...
        "MaximumRetimedVisibilityRoutes must be a positive integer or Inf.");
end

validateattributes(options.MaximumVisibilitySnapshotsPerObstacle, ...
    {'numeric'}, {'real', 'scalar', 'positive'});
if isfinite(options.MaximumVisibilitySnapshotsPerObstacle) && ...
        fix(options.MaximumVisibilitySnapshotsPerObstacle) ~= ...
        options.MaximumVisibilitySnapshotsPerObstacle
    error("planAzElMotion:InvalidSnapshotLimit", ...
        "MaximumVisibilitySnapshotsPerObstacle must be a positive " + ...
        "integer or Inf.");
end

validateattributes(options.SnapshotProbeEdgeCount, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'positive'});
validateattributes(options.SnapshotCountChangeThreshold, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'});
validateattributes(options.SnapshotEdgeRotationThreshold_deg, ...
    {'numeric'}, {'real', 'scalar', 'nonnegative'});
validateattributes(options.SnapshotBoundaryMotionThreshold_deg, ...
    {'numeric'}, {'real', 'scalar', 'nonnegative'});


validateattributes(options.MaximumCollisionPrefilterSamples, ...
    {'numeric'}, {'real', 'scalar', '>=', 2});
if isfinite(options.MaximumCollisionPrefilterSamples) && ...
        fix(options.MaximumCollisionPrefilterSamples) ~= ...
        options.MaximumCollisionPrefilterSamples
    error("planAzElMotion:InvalidCollisionPrefilterLimit", ...
        "MaximumCollisionPrefilterSamples must be an integer of at " + ...
        "least two, or Inf, so both trajectory endpoints are retained.");
end


validateattributes(options.MinimumContinuouslyValidatedCandidates, ...
    {'numeric'}, {'real', 'scalar', 'positive'});
if isfinite(options.MinimumContinuouslyValidatedCandidates) && ...
        fix(options.MinimumContinuouslyValidatedCandidates) ~= ...
        options.MinimumContinuouslyValidatedCandidates
    error("planAzElMotion:InvalidContinuousValidationMinimum", ...
        "MinimumContinuouslyValidatedCandidates must be a positive " + ...
        "integer or Inf.");
end


validateattributes(options.VisibilityGraphCostChangeThreshold, ...
    {'numeric'}, {'real', 'finite', 'scalar', '>=', 0, '<=', 1});
validateattributes(options.MaximumTemporalRefinementSteps, ...
    {'numeric'}, {'real', 'scalar', 'nonnegative'});
if isfinite(options.MaximumTemporalRefinementSteps) && ...
        fix(options.MaximumTemporalRefinementSteps) ~= ...
        options.MaximumTemporalRefinementSteps
    error("planAzElMotion:InvalidTemporalRefinementStepLimit", ...
        "MaximumTemporalRefinementSteps must be a nonnegative integer " + ...
        "or Inf.");
end
validateattributes(options.MaximumPlanningTime_s, ...
    {'numeric'}, {'real', 'scalar', 'positive'});
validateattributes(options.OptimalityTolerance_s, ...
    {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
validateattributes(options.TemporalRefinementSampleTimes_s, ...
    {'numeric'}, {'real', 'finite', 'vector'});
options.TemporalRefinementSampleTimes_s = unique(double( ...
    options.TemporalRefinementSampleTimes_s(:)));

validateattributes(options.CollisionTimePaddingSamples, {'numeric'}, ...
    {'real','finite','scalar','integer','nonnegative'});
validateattributes(options.AzimuthInterval_deg, {'numeric'}, ...
    {'real','finite','vector','numel',2,'increasing'});
end

function mode = normalizeParallelOption(mode)
%% Section 0: Header & Readme
% SYNTAX
%   mode = normalizeParallelOption(mode)
%**************************************************************************
% PURPOSE
%   - Normalize the planner parallel control to auto, on, or off.
%**************************************************************************
% INPUTS
%   - mode (scalar text, logical scalar, or binary numeric scalar)
%**************************************************************************
% OUTPUTS
%   - mode (scalar string)
%**************************************************************************
% UNITS
%   - The mode is dimensionless.
%**************************************************************************
if (islogical(mode) || isnumeric(mode)) && isscalar(mode)
    validateattributes(mode, ...
        {'logical', 'numeric'}, {'real', 'finite', 'scalar'});
    if isnumeric(mode) && ~any(mode == [0 1])
        error("planAzElMotion:InvalidUseParallel", ...
            "Numeric UseParallel must be zero or one.");
    end
    if logical(mode)
        mode = "on";
    else
        mode = "off";
    end
else
    mode = lower(string(mode));
end
if ~isscalar(mode) || ~any(mode == ["auto" "on" "off"])
    error("planAzElMotion:InvalidUseParallel", ...
        "UseParallel must be auto, on, off, or a logical scalar.");
end
end

function state = normalizeState(state, label)
%% Section 0: Header & Readme
% SYNTAX
%   state = normalizeState(state, label)
%**************************************************************************
% PURPOSE
%   - Normalize one endpoint to scalar time and 1-by-2 state vectors.
%**************************************************************************
% INPUTS
%   - state (scalar struct), label (scalar text)
%       Endpoint state and its diagnostic input name.
%**************************************************************************
% OUTPUTS
%   - state (scalar struct)
%       Normalized time, position, velocity, and acceleration.
%**************************************************************************
% UNITS
%   - Time is seconds; position and derivatives use degree-based units.
%**************************************************************************
if ~isstruct(state) || ~isscalar(state) || ~all(isfield(state, ["time_s" "position_deg"]))
    error("planAzElMotion:InvalidState", "%s must contain time_s and position_deg.", label);
end

if ~isfield(state, "velocity_deg_s") || isempty(state.velocity_deg_s)
    state.velocity_deg_s = [0 0];
end
if ~isfield(state, "acceleration_deg_s2") || isempty(state.acceleration_deg_s2)
    state.acceleration_deg_s2 = [0 0];
end

validateattributes(state.time_s, {'numeric'}, {'real','finite','scalar'});
names = ["position_deg" "velocity_deg_s" "acceleration_deg_s2"];

for stateFieldIndex = 1:numel(names)
    name = names(stateFieldIndex);
    validateattributes(state.(name), {'numeric'}, {'real','finite','vector','numel',2});
    state.(name) = reshape(double(state.(name)), 1, 2);
end

state.time_s = double(state.time_s);
end

function limits = normalizeLimits(limits)
%% Section 0: Header & Readme
% SYNTAX
%   limits = normalizeLimits(limits)
%**************************************************************************
% PURPOSE
%   - Normalize physical limits to positive 1-by-2 vectors.
%**************************************************************************
% INPUTS
%   - limits (scalar struct)
%       Velocity and optional acceleration and jerk limits.
%**************************************************************************
% OUTPUTS
%   - limits (scalar struct)
%       Complete per-axis physical limits.
%**************************************************************************
% UNITS
%   - Limits use deg/s, deg/s^2, and deg/s^3.
%**************************************************************************
if ~isstruct(limits) || ~isscalar(limits) || ~isfield(limits, "maxVelocity_deg_s")
    error("planAzElMotion:InvalidLimits", "limits must contain maxVelocity_deg_s.");
end

names = ["maxVelocity_deg_s" "maxAcceleration_deg_s2" "maxJerk_deg_s3"];
defaults = {[], [Inf Inf], [Inf Inf]};

for limitIndex = 1:numel(names)
    name = names(limitIndex);
    if ~isfield(limits, name) || isempty(limits.(name))
        limits.(name) = defaults{limitIndex};
    end

    value = limits.(name);
    validateattributes(value, {'numeric'}, {'real','vector','nonempty','positive'});
    if any(isnan(value)) || ~(isscalar(value) || numel(value) == 2)
        error("planAzElMotion:InvalidLimits", ...
            "%s must be scalar or two-element and cannot contain NaN.", name);
    end

    value = reshape(double(value), 1, []);
    if isscalar(value)
        value = repmat(value, 1, 2);
    end

    limits.(name) = value;
end
end

% --- Candidate Collection And Selection --------------------------------
function [routes, snapshotTime_s, graphIndex, diagnostics] = ...
        collectRoutes(graphs, initialState, goalState, maximumCount)
%% Section 0: Header & Readme
% SYNTAX
%   [routes, snapshotTime_s, graphIndex, diagnostics] = ...
%       collectRoutes(graphs, initialState, goalState, maximumCount)
%**************************************************************************
% PURPOSE
%   - Collect distinct visibility paths and retain cost/time representatives.
%**************************************************************************
% INPUTS
%   - graphs (structure array), initialState, goalState (scalar structs)
%       Visibility results and requested endpoints.
%   - maximumCount (positive integer)
%       Maximum number of non-direct routes retained for retiming.
%**************************************************************************
% OUTPUTS
%   - routes (cell array), snapshotTime_s, graphIndex (column vectors)
%       Distinct candidate paths and their visibility-graph provenance.
%   - diagnostics (scalar struct)
%       Counts and indices describing route consolidation.
%**************************************************************************
% UNITS
%   - Route positions are degrees and snapshot times are seconds.
%**************************************************************************
routes = {[initialState.position_deg; goalState.position_deg]};
snapshotTime_s = initialState.time_s;
graphIndex = 0;
successful = find([graphs.Success]);
distinct = zeros(0, 1);

for visibilityGraphIndex = reshape(successful, 1, [])
    candidate = graphs(visibilityGraphIndex).PathPosition_deg;
    isDuplicate = false;

    for routeIndex = 1:numel(routes)
        route = routes{routeIndex};

        sameGeometry = isequal(size(route), size(candidate)) && ...
            max(abs(route(:) - candidate(:))) <= 1e-9;
        if sameGeometry
            isDuplicate = true;
            break;
        end
    end

    if ~isDuplicate
        routes{end + 1, 1} = candidate; %#ok<AGROW>
        snapshotTime_s(end + 1, 1) = ...
            graphs(visibilityGraphIndex).Time_s; %#ok<AGROW>
        graphIndex(end + 1, 1) = visibilityGraphIndex; %#ok<AGROW>
        distinct(end + 1, 1) = visibilityGraphIndex; %#ok<AGROW>
    end
end

if numel(routes) > maximumCount + 1
    availableRouteCount = numel(routes) - 1;
    cost = zeros(numel(routes) - 1, 1);

    for routeIndex = 2:numel(routes)
        cost(routeIndex - 1) = ...
            sum(vecnorm(diff(routes{routeIndex}), 2, 2));
    end

    [~, cheapest] = min(cost);
    retained = unique(round(linspace(1, numel(cost), maximumCount))).';
    retained = unique([cheapest; retained], "stable");
    if numel(retained) > maximumCount
        retained = retained(1:maximumCount);
    end

    keep = [1; retained + 1];
    routes = routes(keep);
    snapshotTime_s = snapshotTime_s(keep);
    graphIndex = graphIndex(keep);
    warning("planAzElMotion:RouteReduction", ...
        "Retained %d of %d distinct visibility routes for retiming. " + ...
        "The returned route is fastest only among retained candidates. " + ...
        "Set MaximumRetimedVisibilityRoutes=Inf to disable this explicit " + ...
        "runtime tradeoff.", numel(routes) - 1, availableRouteCount);
end

diagnostics = struct(...
    "SuccessfulGraphCount", numel(successful), ...
    "DistinctRouteCount", numel(distinct), ...
    "MaximumRetimedVisibilityRoutes", maximumCount, ...
    "SelectedRouteCount", numel(routes) - 1, ...
    "SelectedGraphIndices", graphIndex(graphIndex > 0));
end

function respects = routeWithinAzimuthPolicy(position_deg, options)
%% Section 0: Header & Readme
% SYNTAX
%   respects = routeWithinAzimuthPolicy(position_deg, options)
%**************************************************************************
% PURPOSE
%   - Check the configured non-wrapping azimuth interval.
%**************************************************************************
% INPUTS
%   - position_deg (N-by-2 numeric), options (scalar struct)
%       Candidate azimuth/elevation route and resolved planner options.
%**************************************************************************
% OUTPUTS
%   - respects (logical scalar)
%       True when the complete route obeys the azimuth policy.
%**************************************************************************
% UNITS
%   - Position and interval values are degrees.
%**************************************************************************
if options.AllowAzimuthWrapping
    respects = true;
    return;
end
azimuth_deg = position_deg(:, 1);
respects = all(azimuth_deg >= options.AzimuthInterval_deg(1) - 1e-9) && ...
    all(azimuth_deg <= options.AzimuthInterval_deg(2) + 1e-9) && ...
    all(abs(diff(azimuth_deg)) < 180);
end

% --- Geometric Smoothing And Path Sampling ------------------------------
function smoothPath = smoothRoute(route_deg, obstacleField, ...
        collisionTime_s, options)
%% Section 0: Header & Readme
% SYNTAX
%   smoothPath = smoothRoute(route_deg, obstacleField, ...
%       collisionTime_s, options)
%**************************************************************************
% PURPOSE
%   - Replace every resolvable polyline turn with a symmetric G3 blend.
%**************************************************************************
% INPUTS
%   - route_deg (N-by-2 numeric), obstacleField (packed obstacle struct)
%       Candidate polyline and the protected collision geometry.
%   - collisionTime_s (scalar), options (scalar struct)
%       Geometry snapshot time and resolved smoothing options.
%**************************************************************************
% OUTPUTS
%   - smoothPath (scalar struct)
%       Ordered line/quintic primitives, samples, stops, and diagnostics.
%**************************************************************************
% UNITS
%   - Positions and arc lengths are degrees; time is seconds.
%**************************************************************************

% --- Normalize the polyline --------------------------------------------
validateattributes(route_deg, {'numeric'}, {'real','finite','2d','ncols',2});
step_deg = diff(route_deg, 1, 1);
route_deg = route_deg([true; vecnorm(step_deg, 2, 2) > 1e-9], :);

if size(route_deg, 1) < 2
    error("planAzElMotion:ZeroLengthRoute", "A candidate route needs two distinct points.");
end

% --- Find the largest collision-free blend at every turn ---------------
cornerCount = size(route_deg, 1) - 2;
cornerTemplate = struct(...
    "PathPointIndex", 0, ...
    "Position_deg", zeros(1, 2), ...
    "DeflectionAngle_rad", 0, ...
    "AppliedRadius_deg", 0, ...
    "EntryPosition_deg", zeros(1, 2), ...
    "ExitPosition_deg", zeros(1, 2), ...
    "ControlPoints_deg", zeros(6, 2), ...
    "Smoothed", false, ...
    "MandatoryStop", false, ...
    "Reason", "");
corners = repmat(cornerTemplate, cornerCount, 1);
minimumRadius_deg = min(0.02, options.TurnRadius_deg);

for cornerIndex = 1:cornerCount
    pointIndex = cornerIndex + 1;
    corner = route_deg(pointIndex, :);
    incomingVector = corner - route_deg(pointIndex - 1, :);
    outgoingVector = route_deg(pointIndex + 1, :) - corner;
    incomingLength_deg = norm(incomingVector);
    outgoingLength_deg = norm(outgoingVector);
    incoming = incomingVector / incomingLength_deg;
    outgoing = outgoingVector / outgoingLength_deg;
    angle_rad = acos(min(1, max(-1, dot(incoming, outgoing))));
    turnCross = incoming(1) * outgoing(2) - incoming(2) * outgoing(1);

    diagnostic = cornerTemplate;
    diagnostic.PathPointIndex = pointIndex;
    diagnostic.Position_deg = corner;
    diagnostic.EntryPosition_deg = corner;
    diagnostic.ExitPosition_deg = corner;
    diagnostic.DeflectionAngle_rad = angle_rad;

    if angle_rad <= 1e-9
        diagnostic.Reason = "collinear";
        corners(cornerIndex) = diagnostic;
        continue;
    end

    if pi - angle_rad <= 1e-6 || abs(turnCross) <= 1e-12
        diagnostic.MandatoryStop = true;
        diagnostic.Reason = "unresolved reversal";
        corners(cornerIndex) = diagnostic;
        continue;
    end

    tangentScale = (384 / 125) * sin(angle_rad / 2) / cos(angle_rad / 2)^2;
    maximumRadius_deg = 0.45 * min(incomingLength_deg, outgoingLength_deg) / tangentScale;
    requestedRadius_deg = min(options.TurnRadius_deg, maximumRadius_deg);
    trialRadii_deg = requestedRadius_deg * 0.65 .^ (0:60);
    trialRadii_deg = trialRadii_deg(trialRadii_deg >= minimumRadius_deg);

    if requestedRadius_deg >= minimumRadius_deg && (isempty(trialRadii_deg) || ...
            trialRadii_deg(end) > minimumRadius_deg * (1 + eps))
        trialRadii_deg(end + 1) = minimumRadius_deg; %#ok<AGROW>
    end

    for radius_deg = trialRadii_deg
        trim_deg = radius_deg * tangentScale;
        % Repeated endpoint controls make q'' and q''' vanish at both
        % joins, so the line-to-curve transition remains G3.
        controlPoints_deg = [corner - trim_deg * incoming; ...
            corner - 0.5 * trim_deg * incoming; corner; corner; ...
            corner + 0.5 * trim_deg * outgoing; ...
            corner + trim_deg * outgoing];
        primitive = quinticLookup(controlPoints_deg);
        checkCount = max(21, ceil(primitive.Length_deg / 0.02) + 1);
        checkS_deg = linspace(0, primitive.Length_deg, checkCount).';
        checkParameter = interp1(primitive.ArcLengthGrid_deg, ...
            primitive.ParameterGrid, checkS_deg, "pchip");
        checkPosition_deg = evaluateQuintic(controlPoints_deg, ...
            min(max(checkParameter, 0), 1));
        blocked = queryAzElTimedPathCollision(obstacleField, ...
            collisionTime_s, checkPosition_deg, struct(...
            "TimePaddingSamples", options.CollisionTimePaddingSamples, ...
            "BoundaryIsOccupied", false));

        if any(blocked)
            continue;
        end

        diagnostic.AppliedRadius_deg = radius_deg;
        diagnostic.EntryPosition_deg = controlPoints_deg(1, :);
        diagnostic.ExitPosition_deg = controlPoints_deg(end, :);
        diagnostic.ControlPoints_deg = controlPoints_deg;
        diagnostic.Smoothed = true;
        diagnostic.Reason = "collision-free G3 blend";
        break;
    end

    if ~diagnostic.Smoothed
        diagnostic.MandatoryStop = true;
        diagnostic.Reason = "no collision-free blend";
    end

    corners(cornerIndex) = diagnostic;
end

% --- Assemble line and quintic primitives in path order ----------------
primitiveTemplate = struct(...
    "Type", "", ...
    "StartPosition_deg", zeros(1, 2), ...
    "EndPosition_deg", zeros(1, 2), ...
    "Direction", zeros(1, 2), ...
    "Length_deg", 0, ...
    "StartArcLength_deg", 0, ...
    "EndArcLength_deg", 0, ...
    "ControlPoints_deg", zeros(6, 2), ...
    "ParameterGrid", zeros(0, 1), ...
    "ArcLengthGrid_deg", zeros(0, 1), ...
    "CornerPathPointIndex", 0);
primitives = repmat(primitiveTemplate, 0, 1);
mandatoryStopArcLength_deg = zeros(0, 1);
currentPosition_deg = route_deg(1, :);
currentArcLength_deg = 0;

for cornerIndex = 1:cornerCount
    corner = corners(cornerIndex);
    [primitives, currentArcLength_deg] = appendLine(primitives, ...
        primitiveTemplate, currentPosition_deg, ...
        corner.EntryPosition_deg, currentArcLength_deg);

    if corner.Smoothed
        primitive = quinticLookup(corner.ControlPoints_deg);
        primitive.StartArcLength_deg = currentArcLength_deg;
        primitive.EndArcLength_deg = currentArcLength_deg + primitive.Length_deg;
        primitive.CornerPathPointIndex = corner.PathPointIndex;
        primitives(end + 1, 1) = primitive; %#ok<AGROW>
        currentArcLength_deg = primitive.EndArcLength_deg;
        currentPosition_deg = corner.ExitPosition_deg;
    else
        currentPosition_deg = corner.Position_deg;
        if corner.MandatoryStop
            mandatoryStopArcLength_deg(end + 1, 1) = currentArcLength_deg; %#ok<AGROW>
        end
    end
end

[primitives, currentArcLength_deg] = appendLine(primitives, ...
    primitiveTemplate, currentPosition_deg, route_deg(end, :), ...
    currentArcLength_deg);

if isempty(primitives)
    error("planAzElMotion:EmptySmoothPath", "Smoothing produced no nonzero primitive.");
end

% --- Sample the finished path and publish its diagnostics --------------
primitiveBoundaryS_deg = [primitives.EndArcLength_deg].';
sampleS_deg = unique([0; (0:0.05:currentArcLength_deg).'; ...
    primitiveBoundaryS_deg; mandatoryStopArcLength_deg; currentArcLength_deg]);
definition = struct( "Primitives", primitives, "TotalLength_deg", currentArcLength_deg);
samples = samplePath(definition, sampleS_deg);
mandatoryStop = false(size(sampleS_deg));

for stopIndex = 1:numel(mandatoryStopArcLength_deg)
    stopDistance_deg = abs(...
        sampleS_deg - mandatoryStopArcLength_deg(stopIndex));
    [~, sampleIndex] = min(stopDistance_deg);
    mandatoryStop(sampleIndex) = true;
end

smoothPath = struct(...
    "Success", true, ...
    "Message", sprintf("Rounded %d corners; %d stops remain.", ...
        nnz([corners.Smoothed]), nnz([corners.MandatoryStop])), ...
    "OriginalPathPosition_deg", route_deg, ...
    "Primitives", primitives, "TotalLength_deg", currentArcLength_deg, ...
    "SampleArcLength_deg", sampleS_deg, ...
    "position_deg", samples.position_deg, "tangent", samples.tangent, ...
    "secondDerivative_deg_inv", samples.secondDerivative_deg_inv, ...
    "thirdDerivative_deg_inv2", samples.thirdDerivative_deg_inv2, ...
    "curvature_deg_inv", samples.curvature_deg_inv, ...
    "PrimitiveIndex", samples.PrimitiveIndex, "PrimitiveType", samples.PrimitiveType, ...
    "MandatoryStop", mandatoryStop, ...
    "MandatoryStopArcLength_deg", mandatoryStopArcLength_deg, "CornerDiagnostics", corners, ...
    "RoundedCornerCount", nnz([corners.Smoothed]), ...
    "MandatoryStopCount", nnz([corners.MandatoryStop]), ...
    "Options", struct("TurnRadius_deg", options.TurnRadius_deg));
end

function primitive = quinticLookup(controlPoints_deg)
%% Section 0: Header & Readme
% SYNTAX
%   primitive = quinticLookup(controlPoints_deg)
%**************************************************************************
% PURPOSE
%   - Build one monotone parameter-to-arc-length lookup.
%**************************************************************************
% INPUTS
%   - controlPoints_deg (6-by-2 numeric)
%       Regular quintic Bezier control polygon.
%**************************************************************************
% OUTPUTS
%   - primitive (scalar struct)
%       Quintic geometry and its parameter/arc-length lookup arrays.
%**************************************************************************
% UNITS
%   - Positions and arc lengths are degrees; parameter is dimensionless.
%**************************************************************************
controlLength_deg = sum(vecnorm(diff(controlPoints_deg), 2, 2));
parameterGrid = linspace(0, 1, max(100, ceil(controlLength_deg / 0.004)) + 1).';
[~, firstDerivative] = evaluateQuintic(controlPoints_deg, parameterGrid);
parameterSpeed_deg = vecnorm(firstDerivative, 2, 2);
if any(parameterSpeed_deg <= 1e-10)
    error("planAzElMotion:DegenerateQuintic", ...
        "A quintic blend has a zero parameter derivative.");
end
arcLengthGrid_deg = cumtrapz(parameterGrid, parameterSpeed_deg);
primitive = struct( "Type", "quintic", "StartPosition_deg", controlPoints_deg(1, :), ...
    "EndPosition_deg", controlPoints_deg(end, :), "Direction", zeros(1, 2), ...
    "Length_deg", arcLengthGrid_deg(end), "StartArcLength_deg", 0, "EndArcLength_deg", 0, ...
    "ControlPoints_deg", controlPoints_deg, "ParameterGrid", parameterGrid, ...
    "ArcLengthGrid_deg", arcLengthGrid_deg, "CornerPathPointIndex", 0);
end

function [position_deg, firstDerivative, secondDerivative, ...
        thirdDerivative] = evaluateQuintic(controlPoints_deg, parameter)
%% Section 0: Header & Readme
% SYNTAX
%   [position_deg, firstDerivative, secondDerivative, thirdDerivative] = ...
%       evaluateQuintic(controlPoints_deg, parameter)
%**************************************************************************
% PURPOSE
%   - Evaluate a quintic Bezier and its first three parameter derivatives.
%**************************************************************************
% INPUTS
%   - controlPoints_deg (6-by-2), parameter (numeric vector)
%       Bezier control polygon and dimensionless evaluation parameters.
%**************************************************************************
% OUTPUTS
%   - position_deg, firstDerivative, secondDerivative, thirdDerivative
%       N-by-2 position and parameter-derivative arrays.
%**************************************************************************
% UNITS
%   - Position and parameter derivatives are degree-based.
%**************************************************************************
parameter = double(parameter(:));
oneMinus = 1 - parameter;
position_deg = [oneMinus.^5, 5 * oneMinus.^4 .* parameter, 10 * oneMinus.^3 .* parameter.^2, ...
    10 * oneMinus.^2 .* parameter.^3, ...
    5 * oneMinus .* parameter.^4, parameter.^5] * controlPoints_deg;
firstDerivative = [oneMinus.^4, 4 * oneMinus.^3 .* parameter, ...
    6 * oneMinus.^2 .* parameter.^2, 4 * oneMinus .* parameter.^3, parameter.^4] * ...
    (5 * diff(controlPoints_deg, 1, 1));
secondDerivative = [oneMinus.^3, 3 * oneMinus.^2 .* parameter, ...
    3 * oneMinus .* parameter.^2, parameter.^3] * (20 * diff(controlPoints_deg, 2, 1));
thirdDerivative = [oneMinus.^2, 2 * oneMinus .* parameter, parameter.^2] * ...
    (60 * diff(controlPoints_deg, 3, 1));
end

function [primitives, endS_deg] = appendLine(primitives, template, ...
        start_deg, goal_deg, startS_deg)
%% Section 0: Header & Readme
% SYNTAX
%   [primitives, endS_deg] = appendLine(primitives, template, ...
%       start_deg, goal_deg, startS_deg)
%**************************************************************************
% PURPOSE
%   - Append one nonzero straight primitive with exact arc metadata.
%**************************************************************************
% INPUTS
%   - primitives (structure array), template (scalar struct)
%       Existing path primitives and the stable primitive schema.
%   - start_deg, goal_deg (1-by-2), startS_deg (scalar)
%       Line endpoints and starting cumulative arc length.
%**************************************************************************
% OUTPUTS
%   - primitives (structure array), endS_deg (scalar)
%       Updated primitive sequence and cumulative arc length.
%**************************************************************************
% UNITS
%   - Positions and arc lengths are degrees.
%**************************************************************************
delta_deg = goal_deg - start_deg;
length_deg = norm(delta_deg);
endS_deg = startS_deg;
if length_deg <= 1e-9
    return;
end
primitive = template;
primitive.Type = "line";
primitive.StartPosition_deg = start_deg;
primitive.EndPosition_deg = goal_deg;
primitive.Direction = delta_deg / length_deg;
primitive.Length_deg = length_deg;
primitive.StartArcLength_deg = startS_deg;
endS_deg = startS_deg + length_deg;
primitive.EndArcLength_deg = endS_deg;
primitives(end + 1, 1) = primitive;
end

function samples = samplePath(smoothPath, arcLength_deg)
%% Section 0: Header & Readme
% SYNTAX
%   samples = samplePath(smoothPath, arcLength_deg)
%**************************************************************************
% PURPOSE
%   - Evaluate position and the first three arc derivatives on the path.
%**************************************************************************
% INPUTS
%   - smoothPath (scalar struct), arcLength_deg (numeric vector)
%       Ordered primitives and cumulative arc-length queries.
%**************************************************************************
% OUTPUTS
%   - samples (scalar struct)
%       Position, derivatives, curvature, and primitive provenance.
%**************************************************************************
% UNITS
%   - Position/arc length are degrees; derivatives use inverse degrees.
%**************************************************************************
queryS_deg = double(arcLength_deg(:));
totalLength_deg = smoothPath.TotalLength_deg;
tolerance_deg = 1e-10 * max(1, totalLength_deg);
if any(queryS_deg < -tolerance_deg) || any(queryS_deg > totalLength_deg + tolerance_deg)
    error("planAzElMotion:ArcLengthOutsidePath", ...
        "Arc-length queries must remain on the smooth path; " + ...
        "observed [%.17g, %.17g] deg, path [0, %.17g] deg.", ...
        min(queryS_deg), max(queryS_deg), totalLength_deg);
end
queryS_deg = min(max(queryS_deg, 0), totalLength_deg);
count = numel(queryS_deg);
position_deg = zeros(count, 2);
tangent = zeros(count, 2);
second = zeros(count, 2);
third = zeros(count, 2);
primitiveIndex = zeros(count, 1);
primitiveType = strings(count, 1);
primitives = smoothPath.Primitives;
for index = 1:numel(primitives)
    primitive = primitives(index);
    belongs = queryS_deg >= primitive.StartArcLength_deg - tolerance_deg;
    if index < numel(primitives)
        belongs = belongs & queryS_deg < primitive.EndArcLength_deg - tolerance_deg;
    else
        belongs = belongs & queryS_deg <= primitive.EndArcLength_deg + tolerance_deg;
    end
    belongs = belongs & primitiveIndex == 0;
    if ~any(belongs)
        continue;
    end
    localS_deg = min(max(queryS_deg(belongs) - ...
        primitive.StartArcLength_deg, 0), primitive.Length_deg);
    if primitive.Type == "line"
        position_deg(belongs, :) = primitive.StartPosition_deg + ...
            localS_deg .* primitive.Direction;
        tangent(belongs, :) = repmat(primitive.Direction, nnz(belongs), 1);
    else
        parameter = interp1(primitive.ArcLengthGrid_deg, ...
            primitive.ParameterGrid, localS_deg, "pchip");
        [position, first, parameterSecond, parameterThird] = ...
            evaluateQuintic(primitive.ControlPoints_deg, min(max(parameter, 0), 1));
        speed = vecnorm(first, 2, 2);
        firstSecond = sum(first .* parameterSecond, 2);
        speedSquared = speed.^2;
        secondValue = parameterSecond ./ speedSquared - first .* firstSecond ./ speed.^4;
        thirdValue = parameterThird ./ speed.^3 - ...
            3 * parameterSecond .* firstSecond ./ speed.^5 - ...
            first .* (sum(parameterSecond.^2, 2) + ...
            sum(first .* parameterThird, 2)) ./ speed.^5 + ...
            4 * first .* firstSecond.^2 ./ speed.^7;
        position_deg(belongs, :) = position;
        tangent(belongs, :) = first ./ speed;
        second(belongs, :) = secondValue;
        third(belongs, :) = thirdValue;
    end
    primitiveIndex(belongs) = index;
    primitiveType(belongs) = primitive.Type;
end
if any(primitiveIndex == 0)
    error("planAzElMotion:PrimitiveCoverage", ...
        "Smooth primitives do not cover every requested arc length.");
end
samples = struct( "arcLength_deg", queryS_deg, "position_deg", position_deg, ...
    "tangent", tangent, "secondDerivative_deg_inv", second, ...
    "thirdDerivative_deg_inv2", third, "curvature_deg_inv", vecnorm(second, 2, 2), ...
    "PrimitiveIndex", primitiveIndex, "PrimitiveType", primitiveType);
end

% --- Spatial Retiming And Motion Profiles -------------------------------
function timedPath = retimeSpatialPath( ...
        smoothPath, initialState, goalState, limits, options, ...
        departureCandidateTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   timedPath = retimeSpatialPath( ...
%       smoothPath, initialState, goalState, limits, options, ...
%       departureCandidateTime_s)
%**************************************************************************
% PURPOSE
%   - Retime one fixed G3 path with certified spatial limits in either mode.
%**************************************************************************
% INPUTS
%   - smoothPath, initialState, goalState, limits, options (scalar structs)
%       Fixed geometry, boundary states, physical limits, and time policy.
%   - departureCandidateTime_s (finite numeric scalar)
%       Earliest motion-start time for this independently checked schedule.
%**************************************************************************
% OUTPUTS
%   - timedPath (scalar struct)
%       Stable success/failure trajectory and constraint diagnostics.
%**************************************************************************
% UNITS
%   - Path is degrees; time and derivatives use degree-based SI time units.
%**************************************************************************

% --- Match the endpoint states to the fixed path ------------------------
tolerance = 1e-9;
validateattributes(departureCandidateTime_s, {'numeric'}, ...
    {'real', 'finite', 'scalar'});
totalLength_deg = smoothPath.TotalLength_deg;
endpoint = samplePath(smoothPath, [0; totalLength_deg]);

initialSpeed_deg_s = boundarySpeed(initialState.velocity_deg_s, ...
    endpoint.tangent(1, :), tolerance);
goalSpeed_deg_s = boundarySpeed(goalState.velocity_deg_s, ...
    endpoint.tangent(2, :), tolerance);

requiredInitialAcceleration_deg_s2 = ...
    endpoint.secondDerivative_deg_inv(1, :) * initialSpeed_deg_s^2;
requiredGoalAcceleration_deg_s2 = ...
    endpoint.secondDerivative_deg_inv(2, :) * goalSpeed_deg_s^2;

if norm(initialState.acceleration_deg_s2 - ...
        requiredInitialAcceleration_deg_s2) > tolerance || ...
        norm(goalState.acceleration_deg_s2 - requiredGoalAcceleration_deg_s2) > tolerance
    timedPath = emptyTimedPath(limits, options, ...
        "Endpoint acceleration does not match the path curvature.");
    return;
end

% --- Define the spatial constraint runs -------------------------------
jerkConstrained = any(isfinite(limits.maxJerk_deg_s3));
geometricPrimitiveCount = numel(smoothPath.Primitives);
runPrimitiveIndex = (1:geometricPrimitiveCount).';
boundaryS_deg = [0; [smoothPath.Primitives.EndArcLength_deg].'];

if ~jerkConstrained
    % The acceleration-only retimer needs local curvature envelopes. The
    % 0.1-degree cells vary the limits spatially without changing geometry.
    boundaryS_deg = 0;
    runPrimitiveIndex = zeros(0, 1);

    for primitiveIndex = 1:geometricPrimitiveCount
        primitive = smoothPath.Primitives(primitiveIndex);
        cellCount = max(2, ceil(primitive.Length_deg / 0.1));
        localS_deg = linspace(primitive.StartArcLength_deg, ...
            primitive.EndArcLength_deg, cellCount + 1).';
        boundaryS_deg = [boundaryS_deg; localS_deg(2:end)]; %#ok<AGROW>
        runPrimitiveIndex = [runPrimitiveIndex; ...
            repmat(primitiveIndex, cellCount, 1)]; %#ok<AGROW>
    end
end

runCount = numel(runPrimitiveIndex);
length_deg = diff(boundaryS_deg);

% --- Certify local path derivatives and derive scalar limits -----------
boundTemplate = derivativeBoundsTemplate();
bounds = repmat(boundTemplate, runCount, 1);
maximumSpeed_deg_s = zeros(runCount, 1);
maximumAcceleration_deg_s2 = zeros(runCount, 1);
maximumJerk_deg_s3 = zeros(runCount, 1);

effectiveLimits = limits;
unconstrainedAcceleration = ~isfinite(limits.maxAcceleration_deg_s2);
effectiveLimits.maxAcceleration_deg_s2(unconstrainedAcceleration) = ...
    100 * max(limits.maxVelocity_deg_s);

unconstrainedJerk = ~isfinite(limits.maxJerk_deg_s3);
if jerkConstrained
    % Mixed finite/infinite jerk axes need a finite scalar working value.
    % The original infinite axis remains unconstrained in final validation.
    effectiveLimits.maxJerk_deg_s3(unconstrainedJerk) = ...
        1000 * max(1, max(effectiveLimits.maxAcceleration_deg_s2));
end

for runIndex = 1:runCount
    primitiveIndex = runPrimitiveIndex(runIndex);
    primitive = smoothPath.Primitives(primitiveIndex);
    bounds(runIndex) = derivativeBounds(primitive, boundaryS_deg(runIndex), ...
        boundaryS_deg(runIndex + 1), runIndex);
    [maximumSpeed_deg_s(runIndex), ...
        maximumAcceleration_deg_s2(runIndex), ...
        maximumJerk_deg_s3(runIndex)] = scalarLimits(bounds(runIndex), ...
        effectiveLimits, tolerance);
end

% --- Mark stops that the geometry cannot smooth ------------------------
mandatoryStopNode = false(runCount + 1, 1);
mandatoryStopArcLength_deg = ...
    reshape(smoothPath.MandatoryStopArcLength_deg, 1, []);

for stopArcLength_deg = mandatoryStopArcLength_deg
    stopDistance_deg = abs(boundaryS_deg - stopArcLength_deg);
    [distance_deg, nodeIndex] = min(stopDistance_deg);

    if distance_deg > tolerance * max(1, totalLength_deg)
        error("planAzElMotion:MissingStopNode", ...
            "A mandatory stop does not coincide with a primitive join.");
    end

    mandatoryStopNode(nodeIndex) = true;
end

% --- Propagate the largest reachable speed at every run boundary -------
[nodeSpeed_deg_s, feasible, failureMessage] = ...
    accelerationNodeSpeeds(length_deg, maximumSpeed_deg_s, bounds, ...
    limits.maxAcceleration_deg_s2, initialSpeed_deg_s, ...
    goalSpeed_deg_s, mandatoryStopNode, tolerance);
if jerkConstrained
    [nodeSpeed_deg_s, feasible, failureMessage] = ...
        reachableNodeSpeeds(length_deg, maximumSpeed_deg_s, ...
        maximumAcceleration_deg_s2, maximumJerk_deg_s3, ...
        initialSpeed_deg_s, goalSpeed_deg_s, mandatoryStopNode, tolerance);
end
if ~feasible
    timedPath = emptyTimedPath(limits, options, failureMessage);
    return;
end

% --- Build one analytic motion profile for every run -------------------
profiles = repmat(profileTemplate(), runCount, 1);
runEndpoint = samplePath(smoothPath, boundaryS_deg);
uniformTimeScaleFactor = 1;

for runIndex = 1:runCount
    if jerkConstrained
        [profile, feasible, failureMessage] = minimumTimeProfile(...
            length_deg(runIndex), nodeSpeed_deg_s(runIndex), ...
            nodeSpeed_deg_s(runIndex + 1), ...
            maximumSpeed_deg_s(runIndex), ...
            maximumAcceleration_deg_s2(runIndex), ...
            maximumJerk_deg_s3(runIndex), tolerance);
    else
        profile = profileTemplate();
        profile.Length_deg = length_deg(runIndex);
        profile.StartSpeed_deg_s = nodeSpeed_deg_s(runIndex);
        profile.EndSpeed_deg_s = nodeSpeed_deg_s(runIndex + 1);
        profile.Duration_s = 2 * length_deg(runIndex) / ...
            (profile.StartSpeed_deg_s + profile.EndSpeed_deg_s);
        profile.TangentialAcceleration_deg_s2 = ...
            (profile.EndSpeed_deg_s^2 - profile.StartSpeed_deg_s^2) / ...
            (2 * length_deg(runIndex));
        profile.PeakSpeed_deg_s = max(profile.StartSpeed_deg_s, ...
            profile.EndSpeed_deg_s);
        profile.PeakAcceleration_deg_s2 = ...
            abs(profile.TangentialAcceleration_deg_s2);
        profile.PeakJerk_deg_s3 = NaN;
        profile.PhaseDuration_s(1) = profile.Duration_s;
        profile.PhaseStartTime_s(2:end) = profile.Duration_s;
        profile.PhaseStartPosition_deg(2:end) = profile.Length_deg;
        profile.PhaseStartSpeed_deg_s(1) = profile.StartSpeed_deg_s;
        profile.PhaseStartSpeed_deg_s(2:end) = profile.EndSpeed_deg_s;
        profile.PhaseStartAcceleration_deg_s2(1) = ...
            profile.TangentialAcceleration_deg_s2;
        feasible = isfinite(profile.Duration_s) && profile.Duration_s > 0;
        failureMessage = "A zero-speed spatial cell is infeasible.";
    end

    if ~feasible
        timedPath = emptyTimedPath(limits, options, ...
            "Primitive " + runIndex + ": " + failureMessage);
        return;
    end

    primitive = smoothPath.Primitives(runPrimitiveIndex(runIndex));
    profile.PrimitiveType = primitive.Type;
    profile.StartPosition_deg = runEndpoint.position_deg(runIndex, :);
    profile.EndPosition_deg = runEndpoint.position_deg(runIndex + 1, :);
    profile.StartArcLength_deg = boundaryS_deg(runIndex);
    profile.EndArcLength_deg = boundaryS_deg(runIndex + 1);
    profile.MaxSpeed_deg_s = maximumSpeed_deg_s(runIndex);
    profile.MaxAcceleration_deg_s2 = ...
        maximumAcceleration_deg_s2(runIndex);
    profile.MaxJerk_deg_s3 = maximumJerk_deg_s3(runIndex);
    [profile.PeakVelocityByAxis_deg_s, profile.PeakAccelerationByAxis_deg_s2, ...
        profile.PeakJerkByAxis_deg_s3] = ...
        cartesianBounds(bounds(runIndex), profile);
    profiles(runIndex) = profile;
end

if jerkConstrained
    [profiles, nodeSpeed_deg_s, uniformTimeScaleFactor, feasible, ...
        failureMessage] = enforceCoupledCartesianLimits( ...
        profiles, nodeSpeed_deg_s, bounds, limits, tolerance);
    if ~feasible
        timedPath = emptyTimedPath(limits, options, failureMessage);
        return;
    end
end

% --- Resolve arrival policy and assign absolute profile times -----------
minimumMotionDuration_s = sum([profiles.Duration_s]);
minimumWaitDuration_s = max( ...
    0, departureCandidateTime_s - initialState.time_s);
minimumArrivalTime_s = initialState.time_s + ...
    minimumWaitDuration_s + minimumMotionDuration_s;
timeTolerance_s = tolerance * max(1, abs(goalState.time_s));
if minimumArrivalTime_s > goalState.time_s + timeTolerance_s
    timedPath = emptyTimedPath(limits, options, sprintf( ...
        "Earliest arrival %.9g s exceeds goal time %.9g s.", ...
        minimumArrivalTime_s, goalState.time_s));
    return;
end

waitDuration_s = minimumWaitDuration_s;
if options.GoalTimeMode == "fixedarrival"
    waitDuration_s = max(0, ...
        goalState.time_s - initialState.time_s - ...
        minimumMotionDuration_s);
    if waitDuration_s + timeTolerance_s < minimumWaitDuration_s
        timedPath = emptyTimedPath(limits, options, sprintf( ...
            "Fixed arrival would require motion before %.9g s.", ...
            departureCandidateTime_s));
        return;
    end
end
requiresInitialHold = waitDuration_s > timeTolerance_s;
initialStateIsMoving = norm(initialState.velocity_deg_s) > tolerance || ...
    norm(initialState.acceleration_deg_s2) > tolerance;
if requiresInitialHold && initialStateIsMoving
    timedPath = emptyTimedPath(limits, options, ...
        "The timed route requires a hold, but the initial state is moving.");
    return;
end

motionStartTime_s = initialState.time_s + waitDuration_s;
startTime_s = motionStartTime_s + [0, cumsum([profiles(1:end - 1).Duration_s])];

for runIndex = 1:runCount
    profiles(runIndex).StartTime_s = startTime_s(runIndex);
    profiles(runIndex).EndTime_s = ...
        startTime_s(runIndex) + profiles(runIndex).Duration_s;
end

% --- Sample scalar motion and map it onto the fixed geometry ------------
[time_s, sampleS_deg, scalarSpeed_deg_s, scalarAcceleration_deg_s2, ...
    scalarJerk_deg_s3] = sampleProfiles(profiles, initialState.time_s, ...
    waitDuration_s, options.SampleTime_s);
geometry = samplePath(smoothPath, sampleS_deg);

position_deg = geometry.position_deg;
velocity_deg_s = geometry.tangent .* scalarSpeed_deg_s;
acceleration_deg_s2 = geometry.tangent .* scalarAcceleration_deg_s2 + ...
    geometry.secondDerivative_deg_inv .* scalarSpeed_deg_s.^2;
jerk_deg_s3 = geometry.tangent .* scalarJerk_deg_s3 + ...
    3 * geometry.secondDerivative_deg_inv .* scalarSpeed_deg_s .* ...
    scalarAcceleration_deg_s2 + geometry.thirdDerivative_deg_inv2 .* scalarSpeed_deg_s.^3;

% Endpoint values are part of the public request, so preserve them exactly.
position_deg(1, :) = initialState.position_deg;
velocity_deg_s(1, :) = initialState.velocity_deg_s;
acceleration_deg_s2(1, :) = initialState.acceleration_deg_s2;
position_deg(end, :) = goalState.position_deg;
velocity_deg_s(end, :) = goalState.velocity_deg_s;
acceleration_deg_s2(end, :) = goalState.acceleration_deg_s2;
if jerkConstrained
    jerk_deg_s3(1, :) = [0 0];
    jerk_deg_s3(end, :) = [0 0];
else
    jerk_deg_s3(:) = NaN;
end

% --- Certify limits and assemble the stable timed-path record -----------
peakVelocity_deg_s = max(vertcat(...
    profiles.PeakVelocityByAxis_deg_s), [], 1);
peakAcceleration_deg_s2 = max(vertcat(...
    profiles.PeakAccelerationByAxis_deg_s2), [], 1);
peakJerk_deg_s3 = max(vertcat(...
    profiles.PeakJerkByAxis_deg_s3), [], 1);

velocitySatisfied = all(peakVelocity_deg_s <= limits.maxVelocity_deg_s + tolerance);
accelerationSatisfied = all(peakAcceleration_deg_s2 <= ...
    limits.maxAcceleration_deg_s2 + tolerance);
jerkSatisfied = ~jerkConstrained || all(peakJerk_deg_s3 <= limits.maxJerk_deg_s3 + tolerance);
constraintsSatisfied = velocitySatisfied && accelerationSatisfied && jerkSatisfied;

joinSpeed_deg_s = nodeSpeed_deg_s(2:end - 1);
geometricJoin = ismember(boundaryS_deg(2:end - 1), ...
    [smoothPath.Primitives(1:end - 1).EndArcLength_deg].');
ordinaryJoin = geometricJoin & ~mandatoryStopNode(2:end - 1);
ordinaryJoinSpeed_deg_s = joinSpeed_deg_s(ordinaryJoin);
minimumJoinSpeed_deg_s = NaN;
if ~isempty(ordinaryJoinSpeed_deg_s)
    minimumJoinSpeed_deg_s = min(ordinaryJoinSpeed_deg_s);
end

diagnostics = struct(...
    "PeakVelocity_deg_s", peakVelocity_deg_s, ...
    "PeakAcceleration_deg_s2", peakAcceleration_deg_s2, ...
    "PeakJerk_deg_s3", peakJerk_deg_s3, ...
    "VelocityMargin_deg_s", limits.maxVelocity_deg_s - peakVelocity_deg_s, ...
    "AccelerationMargin_deg_s2", limits.maxAcceleration_deg_s2 - peakAcceleration_deg_s2, ...
    "JerkMargin_deg_s3", limits.maxJerk_deg_s3 - peakJerk_deg_s3, ...
    "VelocitySatisfied", velocitySatisfied, ...
    "AccelerationSatisfied", accelerationSatisfied, ...
    "JerkSatisfied", jerkSatisfied, ...
    "JerkConstrained", jerkConstrained, ...
    "FiniteJerkCertified", jerkConstrained && constraintsSatisfied, ...
    "FiniteJerkNumericallyVerified", jerkConstrained && jerkSatisfied, ...
    "ContinuousJerkCertified", false, ...
    "G3JoinCount", nnz(ordinaryJoin), ...
    "MinimumG3JoinSpeed_deg_s", minimumJoinSpeed_deg_s, ...
    "VelocityCarriedAcrossG3Joins", isempty(ordinaryJoinSpeed_deg_s) || ...
        all(ordinaryJoinSpeed_deg_s > tolerance), ...
    "JoinContinuityOrder", "G3", ...
    "GeometryDerivativeBounds", bounds, ...
    "SpatiallyVaryingLimits", true, ...
    "UniformTimeScaleFactor", uniformTimeScaleFactor, ...
    "SpatialRetimingCellCount", runCount, ...
    "ExecutedMotionProfileCount", runCount, ...
    "MandatoryStopCount", nnz(mandatoryStopNode), ...
    "MandatoryStopArcLength_deg", smoothPath.MandatoryStopArcLength_deg, ...
    "CurvatureDiscontinuityStopCount", nnz(mandatoryStopNode), ...
    "RoundedVelocityCarried", isempty(ordinaryJoinSpeed_deg_s) || ...
        all(ordinaryJoinSpeed_deg_s > tolerance), ...
    "MinimumArcSpeed_deg_s", minimumJoinSpeed_deg_s, ...
    "Satisfied", constraintsSatisfied);

curveNodeTime_s = [profiles.StartTime_s, profiles(end).EndTime_s].';

timedPath = struct(...
    "Success", constraintsSatisfied, ...
    "Message", "Certified spatial retiming succeeded.", ...
    "time_s", time_s, ...
    "position_deg", position_deg, ...
    "velocity_deg_s", velocity_deg_s, ...
    "acceleration_deg_s2", acceleration_deg_s2, ...
    "jerk_deg_s3", jerk_deg_s3, ...
    "PathPosition_deg", smoothPath.position_deg, ...
    "WaypointTime_s", curveNodeTime_s, ...
    "DepartureCandidateTime_s", departureCandidateTime_s, ...
    "MotionStartTime_s", motionStartTime_s, ...
    "WaitDuration_s", waitDuration_s, ...
    "MinimumMotionDuration_s", minimumMotionDuration_s, ...
    "GoalLineInterceptTime_s", time_s(end), ...
    "SegmentProfiles", profiles, ...
    "Limits", limits, ...
    "Options", struct("GoalTimeMode", options.GoalTimeMode, ...
        "SampleTime_s", options.SampleTime_s), ...
    "ConstraintDiagnostics", diagnostics, ...
    "SmoothPath", smoothPath, ...
    "CurveArcLength_deg", boundaryS_deg, ...
    "CurveNodeTime_s", curveNodeTime_s, ...
    "CurveSpeed_deg_s", nodeSpeed_deg_s, ...
    "CurveSpeedSquared_deg2_s2", nodeSpeed_deg_s.^2, ...
    "CurveTangentialAcceleration_deg_s2", nan(size(nodeSpeed_deg_s)), ...
    "CurveTangentialJerk_deg_s3", nan(size(nodeSpeed_deg_s)), ...
    "SampleArcLength_deg", sampleS_deg, ...
    "SampleSpeed_deg_s", scalarSpeed_deg_s, ...
    "SampleTangentialAcceleration_deg_s2", scalarAcceleration_deg_s2, ...
    "SampleTangentialJerk_deg_s3", scalarJerk_deg_s3, ...
    "CurvatureDiscontinuityStopCount", nnz(mandatoryStopNode), ...
    "RetimerType", "certifiedAnalyticSpatialJerk", ...
    "MotionType", "velocityCarrying");

if ~jerkConstrained
    timedPath.Message = "Certified acceleration-only spatial retiming succeeded.";
    timedPath.RetimerType = "certifiedSpatialAccelerationForwardBackward";
end

if ~constraintsSatisfied
    timedPath.Message = "Internal continuous constraint certification failed.";
end
end

function speed_deg_s = boundarySpeed(velocity_deg_s, tangent, tolerance)
%% Section 0: Header & Readme
% SYNTAX
%   speed_deg_s = boundarySpeed(velocity_deg_s, tangent, tolerance)
%**************************************************************************
% PURPOSE
%   - Project a boundary velocity onto the path and reject lateral motion.
%**************************************************************************
% INPUTS
%   - velocity_deg_s, tangent (1-by-2), tolerance (scalar)
%       Cartesian velocity, unit path tangent, and numerical tolerance.
%**************************************************************************
% OUTPUTS
%   - speed_deg_s (nonnegative scalar)
%       Forward scalar path speed.
%**************************************************************************
% UNITS
%   - Velocity and returned speed are degrees per second.
%**************************************************************************
speed_deg_s = dot(velocity_deg_s, tangent);
if speed_deg_s < -tolerance || norm(velocity_deg_s - speed_deg_s * tangent) > tolerance
    error("planAzElMotion:BoundaryVelocityMismatch", ...
        "Endpoint velocity must be nonnegative and tangent to the path.");
end
speed_deg_s = max(0, speed_deg_s);
end

function bounds = derivativeBoundsTemplate()
%% Section 0: Header & Readme
% SYNTAX
%   bounds = derivativeBoundsTemplate()
%**************************************************************************
% PURPOSE
%   - Return one stable continuous derivative-certificate record.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - bounds (scalar struct)
%       Empty certified and sampled derivative-envelope fields.
%**************************************************************************
% UNITS
%   - Derivative fields use dimensionless, 1/deg, and 1/deg^2 units.
%**************************************************************************
bounds = struct( "RunIndex", 0, "StartArcLength_deg", 0, ...
    "EndArcLength_deg", 0, "TangentByAxis", zeros(1, 2), ...
    "SecondDerivativeByAxis_deg_inv", zeros(1, 2), ...
    "ThirdDerivativeByAxis_deg_inv2", zeros(1, 2), "CertifiedTangentByAxis", zeros(1, 2), ...
    "CertifiedSecondDerivativeByAxis_deg_inv", zeros(1, 2), ...
    "CertifiedThirdDerivativeByAxis_deg_inv2", zeros(1, 2), ...
    "NumericalTangentByAxis", zeros(1, 2), ...
    "NumericalSecondDerivativeByAxis_deg_inv", zeros(1, 2), ...
    "NumericalThirdDerivativeByAxis_deg_inv2", zeros(1, 2), ...
    "EnvelopeInflationFactor", 1, "SampleCount", 0, "CertificateSubdivisionCount", 0, ...
    "CertificateFallbackCount", 0, "CertificatePrimitiveCount", 1, ...
    "SampledBoundsWithinCertificate", true, "Method", "continuousAnalyticEnvelope");
end

function bounds = derivativeBounds(primitive, startS_deg, endS_deg, index)
%% Section 0: Header & Readme
% SYNTAX
%   bounds = derivativeBounds(primitive, startS_deg, endS_deg, index)
%**************************************************************************
% PURPOSE
%   - Certify the first three arc derivatives over a primitive subinterval.
%**************************************************************************
% INPUTS
%   - primitive (scalar struct), startS_deg, endS_deg (scalars)
%       Path primitive and global arc-length interval.
%   - index (positive integer)
%       Retiming-run index stored for diagnostic provenance.
%**************************************************************************
% OUTPUTS
%   - bounds (scalar struct)
%       Continuous certificate plus independent diagnostic samples.
%**************************************************************************
% UNITS
%   - Arc length is degrees; derivatives use 1/deg and 1/deg^2.
%**************************************************************************
localStartS_deg = max(0, startS_deg - primitive.StartArcLength_deg);
localEndS_deg = min(primitive.Length_deg, endS_deg - primitive.StartArcLength_deg);
if primitive.Type == "line"
    tangent = abs(primitive.Direction);
    second = [0 0];
    third = [0 0];
    subdivisionCount = 0;
    fallbackCount = 0;
    method = "exactLine";
else
    startLookupIndex = find(primitive.ArcLengthGrid_deg <= localStartS_deg, 1, "last");
    endLookupIndex = find(primitive.ArcLengthGrid_deg >= localEndS_deg, 1, "first");
    parameterInterval = primitive.ParameterGrid( [startLookupIndex, endLookupIndex]);
    certificate = azElInternal.certifyQuinticArcDerivatives( ...
        primitive.ControlPoints_deg, parameterInterval);
    tangent = min(1, (1 + 1e-12) * certificate.TangentByAxis);
    second = (1 + 1e-12) * certificate.SecondDerivativeByAxis_deg_inv;
    third = (1 + 1e-12) * certificate.ThirdDerivativeByAxis_deg_inv2;
    subdivisionCount = certificate.SubdivisionCount;
    fallbackCount = certificate.FallbackCount;
    method = certificate.Method;
end
localPrimitive = primitive;
localPrimitive.StartArcLength_deg = 0;
localPrimitive.EndArcLength_deg = primitive.Length_deg;
sample = samplePath(struct("Primitives", localPrimitive, ...
    "TotalLength_deg", primitive.Length_deg), linspace(localStartS_deg, localEndS_deg, 17).');
numericalTangent = max(abs(sample.tangent), [], 1);
numericalSecond = max(abs(sample.secondDerivative_deg_inv), [], 1);
numericalThird = max(abs(sample.thirdDerivative_deg_inv2), [], 1);
comparisonTolerance = 1e-10 * max(1, max([tangent second third]));
within = all(numericalTangent <= tangent + comparisonTolerance) && ...
    all(numericalSecond <= second + comparisonTolerance) && ...
    all(numericalThird <= third + comparisonTolerance);
if ~within
    error("planAzElMotion:CertificateViolation", ...
        "A derivative sample exceeded the continuous certificate.");
end
bounds = derivativeBoundsTemplate();
bounds.RunIndex = index;
bounds.StartArcLength_deg = startS_deg;
bounds.EndArcLength_deg = endS_deg;
bounds.TangentByAxis = tangent;
bounds.SecondDerivativeByAxis_deg_inv = second;
bounds.ThirdDerivativeByAxis_deg_inv2 = third;
bounds.CertifiedTangentByAxis = tangent;
bounds.CertifiedSecondDerivativeByAxis_deg_inv = second;
bounds.CertifiedThirdDerivativeByAxis_deg_inv2 = third;
bounds.NumericalTangentByAxis = numericalTangent;
bounds.NumericalSecondDerivativeByAxis_deg_inv = numericalSecond;
bounds.NumericalThirdDerivativeByAxis_deg_inv2 = numericalThird;
bounds.SampleCount = 17;
bounds.CertificateSubdivisionCount = subdivisionCount;
bounds.CertificateFallbackCount = fallbackCount;
bounds.SampledBoundsWithinCertificate = within;
bounds.Method = method;
end

function [maximumSpeed_deg_s, maximumAcceleration_deg_s2, ...
        maximumJerk_deg_s3] = scalarLimits(bounds, limits, tolerance)
%% Section 0: Header & Readme
% SYNTAX
%   [maximumSpeed_deg_s, maximumAcceleration_deg_s2, ...
%       maximumJerk_deg_s3] = scalarLimits(bounds, limits, tolerance)
%**************************************************************************
% PURPOSE
%   - Derive conservative scalar budgets from coupled Cartesian limits.
%**************************************************************************
% INPUTS
%   - bounds, limits (scalar structs), tolerance (scalar)
%       Path derivative certificate, per-axis limits, and tolerance.
%**************************************************************************
% OUTPUTS
%   - maximumSpeed_deg_s, maximumAcceleration_deg_s2, ...
%       maximumJerk_deg_s3 (positive scalars)
%       Safe scalar path limits for one retiming run.
%**************************************************************************
% UNITS
%   - Outputs use deg/s, deg/s^2, and deg/s^3.
%**************************************************************************
tangent = bounds.CertifiedTangentByAxis;
second = bounds.CertifiedSecondDerivativeByAxis_deg_inv;
third = bounds.CertifiedThirdDerivativeByAxis_deg_inv2;
maximumSpeed_deg_s = Inf;
maximumAcceleration_deg_s2 = Inf;
maximumJerk_deg_s3 = Inf;
hasCurvature = any(second > tolerance) || any(third > tolerance);
for axisIndex = 1:2
    if tangent(axisIndex) > tolerance
        maximumSpeed_deg_s = min(maximumSpeed_deg_s, ...
            limits.maxVelocity_deg_s(axisIndex) / tangent(axisIndex));
        maximumAcceleration_deg_s2 = min(maximumAcceleration_deg_s2, ...
            limits.maxAcceleration_deg_s2(axisIndex) / tangent(axisIndex));
        maximumJerk_deg_s3 = min(maximumJerk_deg_s3, ...
            limits.maxJerk_deg_s3(axisIndex) / tangent(axisIndex));
    end
    if hasCurvature && second(axisIndex) > tolerance
        maximumSpeed_deg_s = min(maximumSpeed_deg_s, sqrt( ...
            limits.maxAcceleration_deg_s2(axisIndex) / ...
            second(axisIndex)));
    end
    if hasCurvature && third(axisIndex) > tolerance
        maximumSpeed_deg_s = min(maximumSpeed_deg_s, nthroot( ...
            limits.maxJerk_deg_s3(axisIndex) / ...
            third(axisIndex), 3));
    end
end
if ~all(isfinite([maximumSpeed_deg_s, maximumAcceleration_deg_s2])) || ...
        isnan(maximumJerk_deg_s3) || any([maximumSpeed_deg_s, maximumAcceleration_deg_s2, ...
        maximumJerk_deg_s3] <= 0)
    error("planAzElMotion:InvalidScalarLimits", ...
        "The path produced unusable scalar retiming limits.");
end
end

function [speed_deg_s, feasible, message] = ...
        accelerationNodeSpeeds(length_deg, runSpeedCap_deg_s, bounds, ...
        accelerationLimit_deg_s2, ...
        initialSpeed_deg_s, goalSpeed_deg_s, mandatoryStopNode, tolerance)
%% Section 0: Header & Readme
% SYNTAX
%   [speed_deg_s, feasible, message] = accelerationNodeSpeeds(...
%       length_deg, runSpeedCap_deg_s, bounds, accelerationLimit_deg_s2, ...
%       initialSpeed_deg_s, goalSpeed_deg_s, mandatoryStopNode, tolerance)
%**************************************************************************
% PURPOSE
%   - Carry maximum speed while reserving curvature-dependent acceleration.
%**************************************************************************
% INPUTS
%   - length_deg, runSpeedCap_deg_s (vectors), bounds (structure array)
%       Spatial-run lengths, scalar speed caps, and derivative bounds.
%   - accelerationLimit_deg_s2 (1-by-2), boundary speeds (scalars)
%       Per-axis limits and requested endpoint scalar speeds.
%   - mandatoryStopNode (logical vector), tolerance (scalar)
%       Forced zero-speed nodes and numerical tolerance.
%**************************************************************************
% OUTPUTS
%   - speed_deg_s (column vector), feasible (logical), message (string)
%       Reachable node speeds and failure diagnostics.
%**************************************************************************
% UNITS
%   - Length is degrees; speed and acceleration use deg/s and deg/s^2.
%**************************************************************************
runCount = numel(length_deg);
cap_deg_s = [runSpeedCap_deg_s(1); min(runSpeedCap_deg_s(1:end - 1), ...
    runSpeedCap_deg_s(2:end)); runSpeedCap_deg_s(end)];
cap_deg_s(mandatoryStopNode) = 0;
speedTolerance_deg_s = tolerance * max(1, max(runSpeedCap_deg_s));
if initialSpeed_deg_s > cap_deg_s(1) + speedTolerance_deg_s || ...
        goalSpeed_deg_s > cap_deg_s(end) + speedTolerance_deg_s
    speed_deg_s = zeros(runCount + 1, 1);
    feasible = false;
    message = "An endpoint speed exceeds its local path limit.";
    return;
end
speedSquared_deg2_s2 = cap_deg_s.^2;
speedSquared_deg2_s2([1 end]) = [initialSpeed_deg_s^2; goalSpeed_deg_s^2];
for passIndex = 1:max(8, 2 * (runCount + 1))
    previous = speedSquared_deg2_s2;
    for runIndex = 1:runCount
        speedSquared_deg2_s2(runIndex + 1) = min(...
            speedSquared_deg2_s2(runIndex + 1), ...
            accelerationReachableSquared(speedSquared_deg2_s2(runIndex), ...
            cap_deg_s(runIndex + 1)^2, length_deg(runIndex), ...
            bounds(runIndex), accelerationLimit_deg_s2, tolerance));
    end
    for runIndex = runCount:-1:1
        speedSquared_deg2_s2(runIndex) = min(...
            speedSquared_deg2_s2(runIndex), ...
            accelerationReachableSquared(...
            speedSquared_deg2_s2(runIndex + 1), cap_deg_s(runIndex)^2, ...
            length_deg(runIndex), bounds(runIndex), accelerationLimit_deg_s2, tolerance));
    end
    speedSquared_deg2_s2(mandatoryStopNode) = 0;
    speedSquared_deg2_s2([1 end]) = [initialSpeed_deg_s^2; goalSpeed_deg_s^2];
    if max(abs(speedSquared_deg2_s2 - previous)) <= speedTolerance_deg_s^2
        break;
    end
end
feasible = true;
for runIndex = 1:runCount
    maximumSquaredSpeed = max(speedSquared_deg2_s2(runIndex:runIndex + 1));
    allowance_deg_s2 = scalarAccelerationAllowance(maximumSquaredSpeed, ...
        bounds(runIndex), accelerationLimit_deg_s2, tolerance);
    feasible = feasible && abs(diff(...
        speedSquared_deg2_s2(runIndex:runIndex + 1))) <= ...
        2 * allowance_deg_s2 * length_deg(runIndex) + tolerance;
end
speed_deg_s = sqrt(max(0, speedSquared_deg2_s2));
message = "Acceleration boundary speeds are not mutually reachable.";
end

function reachableSquaredSpeed = accelerationReachableSquared(...
        startSquaredSpeed, capSquaredSpeed, distance_deg, bounds, ...
        accelerationLimit_deg_s2, tolerance)
%% Section 0: Header & Readme
% SYNTAX
%   reachableSquaredSpeed = accelerationReachableSquared(...
%       startSquaredSpeed, capSquaredSpeed, distance_deg, bounds, ...
%       accelerationLimit_deg_s2, tolerance)
%**************************************************************************
% PURPOSE
%   - Solve one monotone squared-speed reachability step by bisection.
%**************************************************************************
% INPUTS
%   - startSquaredSpeed, capSquaredSpeed, distance_deg (scalars)
%       Boundary speed squared, local cap squared, and run length.
%   - bounds (scalar struct), accelerationLimit_deg_s2 (1-by-2)
%       Derivative certificate and per-axis acceleration limits.
%   - tolerance (scalar)
%       Numerical feasibility tolerance.
%**************************************************************************
% OUTPUTS
%   - reachableSquaredSpeed (nonnegative scalar)
%       Largest conservatively reachable squared speed.
%**************************************************************************
% UNITS
%   - Squared speed is deg^2/s^2 and distance is degrees.
%**************************************************************************
if capSquaredSpeed <= startSquaredSpeed
    reachableSquaredSpeed = capSquaredSpeed;
    return;
end
lower = startSquaredSpeed;
upper = capSquaredSpeed;
for iteration = 1:60
    middle = 0.5 * (lower + upper);
    allowance_deg_s2 = scalarAccelerationAllowance(middle, ...
        bounds, accelerationLimit_deg_s2, tolerance);
    if middle - startSquaredSpeed <= 2 * allowance_deg_s2 * distance_deg
        lower = middle;
    else
        upper = middle;
    end
end
reachableSquaredSpeed = lower;
end

function acceleration_deg_s2 = scalarAccelerationAllowance(...
        squaredSpeed_deg2_s2, bounds, accelerationLimit_deg_s2, tolerance)
%% Section 0: Header & Readme
% SYNTAX
%   acceleration_deg_s2 = scalarAccelerationAllowance(...
%       squaredSpeed_deg2_s2, bounds, accelerationLimit_deg_s2, tolerance)
%**************************************************************************
% PURPOSE
%   - Intersect certified per-axis acceleration intervals at one path speed.
%**************************************************************************
% INPUTS
%   - squaredSpeed_deg2_s2 (scalar), bounds (scalar struct)
%       Scalar speed squared and certified path derivatives.
%   - accelerationLimit_deg_s2 (1-by-2), tolerance (scalar)
%       Per-axis limits and numerical tolerance.
%**************************************************************************
% OUTPUTS
%   - acceleration_deg_s2 (nonnegative scalar)
%       Safe tangential acceleration magnitude.
%**************************************************************************
% UNITS
%   - Input squared speed is deg^2/s^2; output is deg/s^2.
%**************************************************************************
tangent = bounds.CertifiedTangentByAxis;
second = bounds.CertifiedSecondDerivativeByAxis_deg_inv;
acceleration_deg_s2 = Inf;
for axisIndex = 1:2
    if ~isfinite(accelerationLimit_deg_s2(axisIndex))
        continue;
    end
    remaining_deg_s2 = accelerationLimit_deg_s2(axisIndex) - ...
        second(axisIndex) * squaredSpeed_deg2_s2;
    if remaining_deg_s2 < -tolerance
        acceleration_deg_s2 = 0;
        return;
    elseif tangent(axisIndex) > tolerance
        acceleration_deg_s2 = min(acceleration_deg_s2, ...
            max(0, remaining_deg_s2) / tangent(axisIndex));
    end
end
end

function [nodeSpeed_deg_s, feasible, message] = ...
        reachableNodeSpeeds(length_deg, maximumSpeed_deg_s, ...
        maximumAcceleration_deg_s2, ...
        maximumJerk_deg_s3, initialSpeed_deg_s, goalSpeed_deg_s, mandatoryStopNode, tolerance)
%% Section 0: Header & Readme
% SYNTAX
%   [nodeSpeed_deg_s, feasible, message] = reachableNodeSpeeds(...
%       length_deg, maximumSpeed_deg_s, maximumAcceleration_deg_s2, ...
%       maximumJerk_deg_s3, initialSpeed_deg_s, goalSpeed_deg_s, ...
%       mandatoryStopNode, tolerance)
%**************************************************************************
% PURPOSE
%   - Carry the largest mutually reachable zero-acceleration node speeds.
%**************************************************************************
% INPUTS
%   - length_deg and maximum speed/acceleration/jerk vectors
%       Spatial run lengths and conservative scalar limits.
%   - initialSpeed_deg_s, goalSpeed_deg_s (scalars)
%       Requested scalar endpoint speeds.
%   - mandatoryStopNode (logical vector), tolerance (scalar)
%       Forced stops and numerical tolerance.
%**************************************************************************
% OUTPUTS
%   - nodeSpeed_deg_s (column vector), feasible (logical), message (string)
%       Mutually reachable node speeds and failure diagnostics.
%**************************************************************************
% UNITS
%   - Length is degrees; derivatives use degree-based per-second units.
%**************************************************************************
runCount = numel(length_deg);
cap_deg_s = Inf(runCount + 1, 1);
cap_deg_s(1) = maximumSpeed_deg_s(1);
cap_deg_s(end) = maximumSpeed_deg_s(end);

for nodeIndex = 2:runCount
    cap_deg_s(nodeIndex) = min(...
        maximumSpeed_deg_s(nodeIndex - 1), ...
        maximumSpeed_deg_s(nodeIndex));
end

cap_deg_s(mandatoryStopNode) = 0;
speedTolerance_deg_s = tolerance * max(1, max(maximumSpeed_deg_s));

if initialSpeed_deg_s > cap_deg_s(1) + speedTolerance_deg_s || ...
        goalSpeed_deg_s > cap_deg_s(end) + speedTolerance_deg_s
    nodeSpeed_deg_s = zeros(runCount + 1, 1);
    feasible = false;
    message = "An endpoint speed exceeds its local path limit.";
    return;
end

nodeSpeed_deg_s = cap_deg_s;
nodeSpeed_deg_s(1) = initialSpeed_deg_s;
nodeSpeed_deg_s(end) = goalSpeed_deg_s;

for passIndex = 1:max(8, 2 * (runCount + 1))
    previous = nodeSpeed_deg_s;

    for runIndex = 1:runCount
        reachable = reachableSpeed(nodeSpeed_deg_s(runIndex), ...
            length_deg(runIndex), maximumSpeed_deg_s(runIndex), ...
            maximumAcceleration_deg_s2(runIndex), ...
            maximumJerk_deg_s3(runIndex), tolerance);

        if runIndex == runCount && ...
                goalSpeed_deg_s > reachable + speedTolerance_deg_s
            feasible = false;
            message = "The goal speed is not forward reachable.";
            return;
        elseif runIndex < runCount
            nodeSpeed_deg_s(runIndex + 1) = min(...
                nodeSpeed_deg_s(runIndex + 1), reachable);
        end
    end

    for runIndex = runCount:-1:1
        reachable = reachableSpeed(nodeSpeed_deg_s(runIndex + 1), ...
            length_deg(runIndex), maximumSpeed_deg_s(runIndex), ...
            maximumAcceleration_deg_s2(runIndex), ...
            maximumJerk_deg_s3(runIndex), tolerance);

        if runIndex == 1 && ...
                initialSpeed_deg_s > reachable + speedTolerance_deg_s
            feasible = false;
            message = "The initial speed cannot brake to path limits.";
            return;
        elseif runIndex > 1
            nodeSpeed_deg_s(runIndex) = min(...
                nodeSpeed_deg_s(runIndex), reachable);
        end
    end

    nodeSpeed_deg_s(mandatoryStopNode) = 0;
    nodeSpeed_deg_s(1) = initialSpeed_deg_s;
    nodeSpeed_deg_s(end) = goalSpeed_deg_s;
    if max(abs(nodeSpeed_deg_s - previous)) <= speedTolerance_deg_s
        break;
    end
end

feasible = true;
message = "";
end

function speed_deg_s = reachableSpeed(boundarySpeed_deg_s, distance_deg, ...
        speedLimit_deg_s, accelerationLimit_deg_s2, ...
        jerkLimit_deg_s3, tolerance)
%% Section 0: Header & Readme
% SYNTAX
%   speed_deg_s = reachableSpeed(boundarySpeed_deg_s, distance_deg, ...
%       speedLimit_deg_s, accelerationLimit_deg_s2, ...
%       jerkLimit_deg_s3, tolerance)
%**************************************************************************
% PURPOSE
%   - Find the greatest zero-acceleration speed reachable through one run.
%**************************************************************************
% INPUTS
%   - boundarySpeed_deg_s, distance_deg, speedLimit_deg_s (scalars)
%       Boundary speed, run length, and scalar speed cap.
%   - accelerationLimit_deg_s2, jerkLimit_deg_s3, tolerance (scalars)
%       Scalar derivative caps and numerical tolerance.
%**************************************************************************
% OUTPUTS
%   - speed_deg_s (nonnegative scalar)
%       Greatest reachable zero-acceleration boundary speed.
%**************************************************************************
% UNITS
%   - Distance is degrees; derivatives use degree-based per-second units.
%**************************************************************************
boundarySpeed_deg_s = min(max(boundarySpeed_deg_s, 0), speedLimit_deg_s);

if transitionDistance(boundarySpeed_deg_s, speedLimit_deg_s, ...
        accelerationLimit_deg_s2, jerkLimit_deg_s3) <= ...
        distance_deg + tolerance * max(1, distance_deg)
    speed_deg_s = speedLimit_deg_s;
    return;
end

lower = boundarySpeed_deg_s;
upper = speedLimit_deg_s;

for iteration = 1:70
    middle = 0.5 * (lower + upper);
    if transitionDistance(boundarySpeed_deg_s, middle, ...
            accelerationLimit_deg_s2, jerkLimit_deg_s3) <= distance_deg
        lower = middle;
    else
        upper = middle;
    end
end

speed_deg_s = lower;
end

function [profile, feasible, message] = minimumTimeProfile(length_deg, ...
        startSpeed_deg_s, endSpeed_deg_s, ...
        speedLimit_deg_s, accelerationLimit_deg_s2, jerkLimit_deg_s3, tolerance)
%% Section 0: Header & Readme
% SYNTAX
%   [profile, feasible, message] = minimumTimeProfile(length_deg, ...
%       startSpeed_deg_s, endSpeed_deg_s, speedLimit_deg_s, ...
%       accelerationLimit_deg_s2, jerkLimit_deg_s3, tolerance)
%**************************************************************************
% PURPOSE
%   - Build the minimum-time seven-phase scalar S-curve for one path run.
%**************************************************************************
% INPUTS
%   - length_deg and start/end/maximum speed values (scalars)
%       Run length, boundary speeds, and scalar speed cap.
%   - accelerationLimit_deg_s2, jerkLimit_deg_s3, tolerance (scalars)
%       Scalar derivative caps and numerical tolerance.
%**************************************************************************
% OUTPUTS
%   - profile (scalar struct), feasible (logical), message (string)
%       Analytic phase record and feasibility diagnostics.
%**************************************************************************
% UNITS
%   - Length is degrees; derivatives use degree-based per-second units.
%**************************************************************************
profile = profileTemplate();
profile.Length_deg = length_deg;
profile.StartSpeed_deg_s = startSpeed_deg_s;
profile.EndSpeed_deg_s = endSpeed_deg_s;

if min([length_deg, speedLimit_deg_s, accelerationLimit_deg_s2, jerkLimit_deg_s3]) <= 0 || ...
        max(startSpeed_deg_s, endSpeed_deg_s) > speedLimit_deg_s + tolerance
    feasible = false;
    message = "Invalid scalar S-curve inputs.";
    return;
end

minimumDistance_deg = transitionDistance(startSpeed_deg_s, endSpeed_deg_s, ...
    accelerationLimit_deg_s2, jerkLimit_deg_s3);

if minimumDistance_deg > length_deg + tolerance * max(1, length_deg)
    feasible = false;
    message = "The run is too short for its boundary speeds.";
    return;
end

peakSpeed_deg_s = speedLimit_deg_s;
distanceAtLimit_deg = transitionDistance(startSpeed_deg_s, ...
    peakSpeed_deg_s, accelerationLimit_deg_s2, jerkLimit_deg_s3) + ...
    transitionDistance(peakSpeed_deg_s, endSpeed_deg_s, ...
    accelerationLimit_deg_s2, jerkLimit_deg_s3);

if distanceAtLimit_deg > length_deg
    lower = max(startSpeed_deg_s, endSpeed_deg_s);
    upper = speedLimit_deg_s;

    for iteration = 1:80
        middle = 0.5 * (lower + upper);
        distance = transitionDistance(startSpeed_deg_s, middle, ...
            accelerationLimit_deg_s2, jerkLimit_deg_s3) + ...
            transitionDistance(middle, endSpeed_deg_s, ...
            accelerationLimit_deg_s2, jerkLimit_deg_s3);
        if distance <= length_deg
            lower = middle;
        else
            upper = middle;
        end
    end

    peakSpeed_deg_s = lower;
end

[firstDuration_s, firstJerk_deg_s3] = transitionPhases(...
    startSpeed_deg_s, peakSpeed_deg_s, ...
    accelerationLimit_deg_s2, jerkLimit_deg_s3);
[lastDuration_s, lastJerk_deg_s3] = transitionPhases(...
    peakSpeed_deg_s, endSpeed_deg_s, ...
    accelerationLimit_deg_s2, jerkLimit_deg_s3);
transitionDistance_deg = transitionDistance(startSpeed_deg_s, ...
    peakSpeed_deg_s, accelerationLimit_deg_s2, jerkLimit_deg_s3) + ...
    transitionDistance(peakSpeed_deg_s, endSpeed_deg_s, ...
    accelerationLimit_deg_s2, jerkLimit_deg_s3);
cruiseDuration_s = max(0, length_deg - transitionDistance_deg) / max(peakSpeed_deg_s, eps);

phaseDuration_s = [firstDuration_s; cruiseDuration_s; lastDuration_s];
phaseJerk_deg_s3 = [firstJerk_deg_s3; 0; lastJerk_deg_s3];
phaseStartTime_s = [0; cumsum(phaseDuration_s(1:end - 1))];
phaseStartPosition_deg = zeros(7, 1);
phaseStartSpeed_deg_s = zeros(7, 1);
phaseStartAcceleration_deg_s2 = zeros(7, 1);
phaseStartSpeed_deg_s(1) = startSpeed_deg_s;

for index = 1:6
    duration_s = phaseDuration_s(index);
    jerk_deg_s3 = phaseJerk_deg_s3(index);
    phaseStartPosition_deg(index + 1) = phaseStartPosition_deg(index) + ...
        phaseStartSpeed_deg_s(index) * duration_s + ...
        0.5 * phaseStartAcceleration_deg_s2(index) * duration_s^2 + ...
        jerk_deg_s3 * duration_s^3 / 6;
    phaseStartSpeed_deg_s(index + 1) = phaseStartSpeed_deg_s(index) + ...
        phaseStartAcceleration_deg_s2(index) * duration_s + 0.5 * jerk_deg_s3 * duration_s^2;
    phaseStartAcceleration_deg_s2(index + 1) = ...
        phaseStartAcceleration_deg_s2(index) + jerk_deg_s3 * duration_s;
end

profile.Duration_s = sum(phaseDuration_s);
profile.PeakSpeed_deg_s = peakSpeed_deg_s;
profile.PeakAcceleration_deg_s2 = max(abs([ phaseStartAcceleration_deg_s2; ...
    phaseStartAcceleration_deg_s2 + phaseJerk_deg_s3 .* phaseDuration_s]));
profile.PeakJerk_deg_s3 = max(abs(phaseJerk_deg_s3));
profile.PhaseDuration_s = phaseDuration_s;
profile.PhaseJerk_deg_s3 = phaseJerk_deg_s3;
profile.PhaseStartTime_s = phaseStartTime_s;
profile.PhaseStartPosition_deg = phaseStartPosition_deg;
profile.PhaseStartSpeed_deg_s = phaseStartSpeed_deg_s;
profile.PhaseStartAcceleration_deg_s2 = phaseStartAcceleration_deg_s2;
profile.TangentialAcceleration_deg_s2 = 0;
feasible = true;
message = "";
end

function profile = profileTemplate()
%% Section 0: Header & Readme
% SYNTAX
%   profile = profileTemplate()
%**************************************************************************
% PURPOSE
%   - Return one stable scalar S-curve profile record.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - profile (scalar struct)
%       Empty geometry, timing, phase, limit, and peak fields.
%**************************************************************************
% UNITS
%   - Field suffixes define degree-based position and derivative units.
%**************************************************************************
profile = struct( "PrimitiveType", "", "StartPosition_deg", zeros(1, 2), ...
    "EndPosition_deg", zeros(1, 2), "Length_deg", 0, ...
    "StartArcLength_deg", 0, "EndArcLength_deg", 0, ...
    "StartSpeed_deg_s", 0, "EndSpeed_deg_s", 0, ...
    "Duration_s", 0, "StartTime_s", 0, "EndTime_s", 0, ...
    "MaxSpeed_deg_s", 0, "MaxAcceleration_deg_s2", 0, ...
    "MaxJerk_deg_s3", 0, "PeakSpeed_deg_s", 0, ...
    "PeakAcceleration_deg_s2", 0, "PeakJerk_deg_s3", 0, ...
    "PeakVelocityByAxis_deg_s", zeros(1, 2), "PeakAccelerationByAxis_deg_s2", zeros(1, 2), ...
    "PeakJerkByAxis_deg_s3", zeros(1, 2), "PhaseDuration_s", zeros(7, 1), ...
    "PhaseJerk_deg_s3", zeros(7, 1), "PhaseStartTime_s", zeros(7, 1), ...
    "PhaseStartPosition_deg", zeros(7, 1), "PhaseStartSpeed_deg_s", zeros(7, 1), ...
    "PhaseStartAcceleration_deg_s2", zeros(7, 1), "TangentialAcceleration_deg_s2", 0);
end

function distance_deg = transitionDistance(firstSpeed_deg_s, secondSpeed_deg_s, ...
        accelerationLimit_deg_s2, jerkLimit_deg_s3)
%% Section 0: Header & Readme
% SYNTAX
%   distance_deg = transitionDistance(firstSpeed_deg_s, ...
%       secondSpeed_deg_s, accelerationLimit_deg_s2, jerkLimit_deg_s3)
%**************************************************************************
% PURPOSE
%   - Return the exact distance of a zero-acceleration S-curve transition.
%**************************************************************************
% INPUTS
%   - firstSpeed_deg_s, secondSpeed_deg_s (scalars)
%       Boundary speeds for one monotone transition.
%   - accelerationLimit_deg_s2, jerkLimit_deg_s3 (positive scalars)
%       Scalar derivative limits.
%**************************************************************************
% OUTPUTS
%   - distance_deg (nonnegative scalar)
%       Exact transition distance.
%**************************************************************************
% UNITS
%   - Distance is degrees; derivatives use degree-based per-second units.
%**************************************************************************
[duration_s, ~] = transitionPhases(firstSpeed_deg_s, ...
    secondSpeed_deg_s, accelerationLimit_deg_s2, jerkLimit_deg_s3);
distance_deg = 0.5 * (firstSpeed_deg_s + secondSpeed_deg_s) * sum(duration_s);
end

function [duration_s, jerk_deg_s3] = transitionPhases(firstSpeed_deg_s, ...
        secondSpeed_deg_s, ...
        accelerationLimit_deg_s2, jerkLimit_deg_s3)
%% Section 0: Header & Readme
% SYNTAX
%   [duration_s, jerk_deg_s3] = transitionPhases(firstSpeed_deg_s, ...
%       secondSpeed_deg_s, accelerationLimit_deg_s2, jerkLimit_deg_s3)
%**************************************************************************
% PURPOSE
%   - Return three phases for one monotone zero-acceleration transition.
%**************************************************************************
% INPUTS
%   - firstSpeed_deg_s, secondSpeed_deg_s (scalars)
%       Boundary speeds for one monotone transition.
%   - accelerationLimit_deg_s2, jerkLimit_deg_s3 (positive scalars)
%       Scalar derivative limits.
%**************************************************************************
% OUTPUTS
%   - duration_s, jerk_deg_s3 (3-by-1 vectors)
%       Phase durations and signed constant jerk values.
%**************************************************************************
% UNITS
%   - Duration is seconds and jerk is degrees per second cubed.
%**************************************************************************
velocityChange_deg_s = secondSpeed_deg_s - firstSpeed_deg_s;
direction = sign(velocityChange_deg_s);
changeMagnitude_deg_s = abs(velocityChange_deg_s);
if changeMagnitude_deg_s == 0
    duration_s = zeros(3, 1);
    jerk_deg_s3 = zeros(3, 1);
    return;
end

jerkRampDuration_s = min(sqrt(changeMagnitude_deg_s / jerkLimit_deg_s3), ...
    accelerationLimit_deg_s2 / jerkLimit_deg_s3);
constantAccelerationDuration_s = max(0, ...
    changeMagnitude_deg_s / (jerkLimit_deg_s3 * jerkRampDuration_s) - jerkRampDuration_s);
duration_s = [jerkRampDuration_s; constantAccelerationDuration_s; jerkRampDuration_s];
jerk_deg_s3 = direction * [jerkLimit_deg_s3; 0; -jerkLimit_deg_s3];
end

function [position_deg, speed_deg_s, acceleration_deg_s2, ...
        jerk_deg_s3] = sampleProfile(time_s, profile)
%% Section 0: Header & Readme
% SYNTAX
%   [position_deg, speed_deg_s, acceleration_deg_s2, jerk_deg_s3] = ...
%       sampleProfile(time_s, profile)
%**************************************************************************
% PURPOSE
%   - Evaluate one analytic scalar profile at local times.
%**************************************************************************
% INPUTS
%   - time_s (numeric vector), profile (scalar struct)
%       Local query times and analytic phase record.
%**************************************************************************
% OUTPUTS
%   - position_deg, speed_deg_s, acceleration_deg_s2, jerk_deg_s3
%       Scalar motion histories at every query time.
%**************************************************************************
% UNITS
%   - Time is seconds; outputs use degree-based derivative units.
%**************************************************************************
time_s = min(max(double(time_s(:)), 0), profile.Duration_s);
count = numel(time_s);
position_deg = zeros(count, 1);
speed_deg_s = zeros(count, 1);
acceleration_deg_s2 = zeros(count, 1);
jerk_deg_s3 = zeros(count, 1);
for index = 1:7
    if index < 7
        belongs = time_s >= profile.PhaseStartTime_s(index) & ...
            time_s < profile.PhaseStartTime_s(index + 1);
    else
        belongs = time_s >= profile.PhaseStartTime_s(index);
    end
    localTime_s = time_s(belongs) - profile.PhaseStartTime_s(index);
    jerk = profile.PhaseJerk_deg_s3(index);
    position_deg(belongs) = profile.PhaseStartPosition_deg(index) + ...
        profile.PhaseStartSpeed_deg_s(index) .* localTime_s + ...
        0.5 * profile.PhaseStartAcceleration_deg_s2(index) .* ...
        localTime_s.^2 + jerk .* localTime_s.^3 / 6;
    speed_deg_s(belongs) = profile.PhaseStartSpeed_deg_s(index) + ...
        profile.PhaseStartAcceleration_deg_s2(index) .* localTime_s + ...
        0.5 * jerk .* localTime_s.^2;
    acceleration_deg_s2(belongs) = profile.PhaseStartAcceleration_deg_s2(index) + ...
        jerk .* localTime_s;
    jerk_deg_s3(belongs) = jerk;
end
if isinf(profile.MaxJerk_deg_s3)
    jerk_deg_s3(:) = NaN;
end
atEnd = time_s == profile.Duration_s;
position_deg(atEnd) = profile.Length_deg;
speed_deg_s(atEnd) = profile.EndSpeed_deg_s;
if isfinite(profile.MaxJerk_deg_s3)
    acceleration_deg_s2(atEnd) = 0;
else
    acceleration_deg_s2(atEnd) = ...
        profile.TangentialAcceleration_deg_s2;
end
end

function [time_s, arcLength_deg, speed_deg_s, acceleration_deg_s2, ...
        jerk_deg_s3] = sampleProfiles(profiles, initialTime_s, ...
        waitDuration_s, sampleTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   [time_s, arcLength_deg, speed_deg_s, acceleration_deg_s2, ...
%       jerk_deg_s3] = sampleProfiles(profiles, initialTime_s, ...
%       waitDuration_s, sampleTime_s)
%**************************************************************************
% PURPOSE
%   - Sample every profile while preserving strict absolute timestamps.
%**************************************************************************
% INPUTS
%   - profiles (structure array)
%       Ordered analytic scalar profiles.
%   - initialTime_s, waitDuration_s, sampleTime_s (scalars)
%       Absolute start, optional hold, and regular sample interval.
%**************************************************************************
% OUTPUTS
%   - time_s, arcLength_deg, speed_deg_s, acceleration_deg_s2, jerk_deg_s3
%       Strictly ordered scalar motion histories.
%**************************************************************************
% UNITS
%   - Time is seconds; motion uses degree-based derivative units.
%**************************************************************************
time_s = zeros(0, 1);
arcLength_deg = zeros(0, 1);
speed_deg_s = zeros(0, 1);
acceleration_deg_s2 = zeros(0, 1);
jerk_deg_s3 = zeros(0, 1);
if waitDuration_s > 1e-12
    localTime_s = unique([0; ...
        (0:sampleTime_s:waitDuration_s).'; waitDuration_s]);
    time_s = initialTime_s + localTime_s;
    arcLength_deg = zeros(size(time_s));
    speed_deg_s = zeros(size(time_s));
    acceleration_deg_s2 = zeros(size(time_s));
    jerk_deg_s3 = zeros(size(time_s));
end
for index = 1:numel(profiles)
    profile = profiles(index);
    phaseEnd_s = cumsum(profile.PhaseDuration_s);
    phaseStart_s = [0; phaseEnd_s(1:end - 1)];
    events_s = unique([phaseStart_s; phaseStart_s + 0.5 * profile.PhaseDuration_s; phaseEnd_s]);
    localTime_s = unique([0; ...
        (0:sampleTime_s:profile.Duration_s).'; ...
        events_s(:); profile.Duration_s]);
    localTime_s = localTime_s( ...
        localTime_s >= 0 & localTime_s <= profile.Duration_s);
    [distance_deg, velocity_deg_s, accel_deg_s2, localJerk_deg_s3] = ...
        sampleProfile(localTime_s, profile);
    if any(distance_deg < -1e-10) || any(distance_deg > profile.Length_deg + 1e-10)
        error("planAzElMotion:ProfileDistanceOutsideRun", ...
            "Profile %d sampled [%.17g, %.17g] deg for length %.17g deg.", ...
            index, min(distance_deg), max(distance_deg), profile.Length_deg);
    end
    absoluteTime_s = profile.StartTime_s + localTime_s;
    previousTime_s = [];
    if ~isempty(time_s)
        previousTime_s = time_s(end);
    end
    % Floating-point addition can collapse neighboring local samples onto
    % one absolute timestamp. Keep the last member of each collapsed group
    % and discard every sample at or before the previous profile endpoint.
    keep = [diff(absoluteTime_s(:)) > 0; true];
    if ~isempty(previousTime_s)
        keep = keep & absoluteTime_s(:) > previousTime_s;
    end
    time_s = [time_s; absoluteTime_s(keep)]; %#ok<AGROW>
    arcLength_deg = [arcLength_deg; ...
        profile.StartArcLength_deg + distance_deg(keep)]; %#ok<AGROW>
    speed_deg_s = [speed_deg_s; velocity_deg_s(keep)]; %#ok<AGROW>
    acceleration_deg_s2 = [acceleration_deg_s2; accel_deg_s2(keep)]; %#ok<AGROW>
    jerk_deg_s3 = [jerk_deg_s3; localJerk_deg_s3(keep)]; %#ok<AGROW>
end
if numel(time_s) < 2 || any(diff(time_s) <= 0)
    error("planAzElMotion:NonIncreasingTime", ...
        "Profile assembly must produce strictly increasing time.");
end
end

function [velocityBound_deg_s, accelerationBound_deg_s2, ...
        jerkBound_deg_s3] = cartesianBounds(bounds, profile)
%% Section 0: Header & Readme
% SYNTAX
%   [velocityBound_deg_s, accelerationBound_deg_s2, jerkBound_deg_s3] = ...
%       cartesianBounds(bounds, profile)
%**************************************************************************
% PURPOSE
%   - Map scalar peaks through the certified path derivative envelope.
%**************************************************************************
% INPUTS
%   - bounds, profile (scalar structs)
%       Certified geometry derivatives and one scalar motion profile.
%**************************************************************************
% OUTPUTS
%   - velocityBound_deg_s, accelerationBound_deg_s2, jerkBound_deg_s3
%       Conservative 1-by-2 per-axis motion bounds.
%**************************************************************************
% UNITS
%   - Outputs use deg/s, deg/s^2, and deg/s^3.
%**************************************************************************
tangent = bounds.CertifiedTangentByAxis;
second = bounds.CertifiedSecondDerivativeByAxis_deg_inv;
third = bounds.CertifiedThirdDerivativeByAxis_deg_inv2;
velocityBound_deg_s = tangent * profile.PeakSpeed_deg_s;
accelerationBound_deg_s2 = tangent * profile.PeakAcceleration_deg_s2 + ...
    second * profile.PeakSpeed_deg_s^2;
jerkBound_deg_s3 = tangent * profile.PeakJerk_deg_s3 + ...
    3 * second * profile.PeakSpeed_deg_s * profile.PeakAcceleration_deg_s2 + ...
    third * profile.PeakSpeed_deg_s^3;
end

function [profiles, nodeSpeed_deg_s, scaleFactor, feasible, message] = ...
        enforceCoupledCartesianLimits(profiles, nodeSpeed_deg_s, bounds, ...
        limits, tolerance)
%% Section 0: Header & Readme
% SYNTAX
%   [profiles, nodeSpeed_deg_s, scaleFactor, feasible, message] = ...
%       enforceCoupledCartesianLimits(profiles, nodeSpeed_deg_s, bounds, ...
%       limits, tolerance)
%**************************************************************************
% PURPOSE
%   - Uniformly dilate a finite-jerk schedule until its certified coupled
%     Cartesian velocity, acceleration, and jerk bounds satisfy every axis.
%**************************************************************************
% INPUTS
%   - profiles (structure array), nodeSpeed_deg_s (numeric vector)
%       Analytic scalar profiles and their shared boundary speeds.
%   - bounds (structure array), limits (scalar struct), tolerance (scalar)
%       Certified path derivatives, Cartesian limits, and comparison margin.
%**************************************************************************
% OUTPUTS
%   - profiles, nodeSpeed_deg_s
%       Feasible profiles and node speeds after any uniform time dilation.
%   - scaleFactor (scalar), feasible (logical), message (string)
%       Applied time factor and an actionable feasibility result.
%**************************************************************************
% UNITS
%   - Time is seconds; motion uses degree-based derivative units.
%**************************************************************************
profileCount = numel(profiles);
velocityPeak_deg_s = zeros(profileCount, 2);
accelerationPeak_deg_s2 = zeros(profileCount, 2);
jerkPeak_deg_s3 = zeros(profileCount, 2);

for profileIndex = 1:profileCount
    [velocityPeak_deg_s(profileIndex, :), ...
        accelerationPeak_deg_s2(profileIndex, :), ...
        jerkPeak_deg_s3(profileIndex, :)] = ...
        cartesianBounds(bounds(profileIndex), profiles(profileIndex));
end

velocityRatio = maxConstraintRatio(velocityPeak_deg_s, ...
    limits.maxVelocity_deg_s);
accelerationRatio = maxConstraintRatio(accelerationPeak_deg_s2, ...
    limits.maxAcceleration_deg_s2);
jerkRatio = maxConstraintRatio(jerkPeak_deg_s3, ...
    limits.maxJerk_deg_s3);
scaleFactor = max([1, velocityRatio, sqrt(accelerationRatio), ...
    nthroot(jerkRatio, 3)]);

if ~isfinite(scaleFactor) || scaleFactor <= 0
    feasible = false;
    message = "Certified coupled path limits produced an invalid time scale.";
    return;
end

if scaleFactor > 1 + tolerance
    endpointSpeedScale_deg_s = 1;
    finiteVelocityLimit_deg_s = limits.maxVelocity_deg_s( ...
        isfinite(limits.maxVelocity_deg_s));
    if ~isempty(finiteVelocityLimit_deg_s)
        endpointSpeedScale_deg_s = max( ...
            endpointSpeedScale_deg_s, max(finiteVelocityLimit_deg_s));
    end
    endpointSpeedTolerance_deg_s = tolerance * endpointSpeedScale_deg_s;
    hasFixedMovingEndpoint = ...
        abs(profiles(1).StartSpeed_deg_s) > endpointSpeedTolerance_deg_s || ...
        abs(profiles(end).EndSpeed_deg_s) > endpointSpeedTolerance_deg_s;
    if hasFixedMovingEndpoint
        feasible = false;
        message = "Coupled path limits require time dilation, but a nonzero " + ...
            "endpoint speed is fixed by the request.";
        return;
    end

    % A small roundoff guard keeps the certificate inside the public limits.
    scaleFactor = scaleFactor * (1 + 32 * eps(scaleFactor));
    for profileIndex = 1:profileCount
        profile = profiles(profileIndex);
        profile.Duration_s = profile.Duration_s * scaleFactor;
        profile.StartSpeed_deg_s = profile.StartSpeed_deg_s / scaleFactor;
        profile.EndSpeed_deg_s = profile.EndSpeed_deg_s / scaleFactor;
        profile.PeakSpeed_deg_s = profile.PeakSpeed_deg_s / scaleFactor;
        profile.PeakAcceleration_deg_s2 = ...
            profile.PeakAcceleration_deg_s2 / scaleFactor^2;
        profile.PeakJerk_deg_s3 = profile.PeakJerk_deg_s3 / scaleFactor^3;
        profile.PhaseDuration_s = profile.PhaseDuration_s * scaleFactor;
        profile.PhaseStartTime_s = profile.PhaseStartTime_s * scaleFactor;
        profile.PhaseStartSpeed_deg_s = ...
            profile.PhaseStartSpeed_deg_s / scaleFactor;
        profile.PhaseStartAcceleration_deg_s2 = ...
            profile.PhaseStartAcceleration_deg_s2 / scaleFactor^2;
        profile.PhaseJerk_deg_s3 = ...
            profile.PhaseJerk_deg_s3 / scaleFactor^3;
        profile.TangentialAcceleration_deg_s2 = ...
            profile.TangentialAcceleration_deg_s2 / scaleFactor^2;
        [profile.PeakVelocityByAxis_deg_s, ...
            profile.PeakAccelerationByAxis_deg_s2, ...
            profile.PeakJerkByAxis_deg_s3] = ...
            cartesianBounds(bounds(profileIndex), profile);
        profiles(profileIndex) = profile;
    end
    nodeSpeed_deg_s = nodeSpeed_deg_s / scaleFactor;
end

feasible = true;
message = "";
end

function ratio = maxConstraintRatio(peakByAxis, limitByAxis)
%% Section 0: Header & Readme
% SYNTAX
%   ratio = maxConstraintRatio(peakByAxis, limitByAxis)
%**************************************************************************
% PURPOSE
%   - Return the largest finite-axis peak-to-limit ratio.
%**************************************************************************
% INPUTS
%   - peakByAxis (N-by-2 numeric), limitByAxis (1-by-2 numeric)
%       Nonnegative certified peaks and positive per-axis limits.
%**************************************************************************
% OUTPUTS
%   - ratio (nonnegative scalar)
%       Largest constrained-axis ratio, or zero when every axis is infinite.
%**************************************************************************
% UNITS
%   - The ratio is dimensionless; inputs have matching derivative units.
%**************************************************************************
finiteAxis = isfinite(limitByAxis);
if ~any(finiteAxis)
    ratio = 0;
    return;
end
ratio = max(peakByAxis(:, finiteAxis) ./ limitByAxis(finiteAxis), [], "all");
end

% --- Stable Failure Records And Independent Validation ------------------
function smoothPath = emptySmoothPath(route_deg)
%% Section 0: Header & Readme
% SYNTAX
%   smoothPath = emptySmoothPath(route_deg)
%**************************************************************************
% PURPOSE
%   - Return the stable empty smooth-path schema for candidate failures.
%**************************************************************************
% INPUTS
%   - route_deg (N-by-2 numeric)
%       Candidate polyline retained for failure diagnostics.
%**************************************************************************
% OUTPUTS
%   - smoothPath (scalar struct)
%       Stable unsuccessful smooth-path record.
%**************************************************************************
% UNITS
%   - Route positions are degrees.
%**************************************************************************
smoothPath = struct( "Success", false, "Message", "No smooth path was produced.", ...
    "OriginalPathPosition_deg", route_deg, "Primitives", struct([]), "TotalLength_deg", 0, ...
    "SampleArcLength_deg", zeros(0, 1), "position_deg", zeros(0, 2), "tangent", zeros(0, 2), ...
    "secondDerivative_deg_inv", zeros(0, 2), "thirdDerivative_deg_inv2", zeros(0, 2), ...
    "curvature_deg_inv", zeros(0, 1), "PrimitiveIndex", zeros(0, 1), ...
    "PrimitiveType", strings(0, 1), "MandatoryStop", false(0, 1), ...
    "MandatoryStopArcLength_deg", zeros(0, 1), ...
    "CornerDiagnostics", struct([]), "RoundedCornerCount", 0, ...
    "MandatoryStopCount", 0, "Options", struct());
end

function timedPath = emptyTimedPath(limits, options, message)
%% Section 0: Header & Readme
% SYNTAX
%   timedPath = emptyTimedPath(limits, options, message)
%**************************************************************************
% PURPOSE
%   - Return the stable timed-path failure schema without hiding diagnostics.
%**************************************************************************
% INPUTS
%   - limits, options (scalar structs), message (scalar text)
%       Resolved physical limits, timing options, and failure explanation.
%**************************************************************************
% OUTPUTS
%   - timedPath (scalar struct)
%       Stable unsuccessful trajectory and constraint record.
%**************************************************************************
% UNITS
%   - Empty scientific arrays retain degree-based field units.
%**************************************************************************
diagnostics = struct( "PeakVelocity_deg_s", [NaN NaN], ...
    "PeakAcceleration_deg_s2", [NaN NaN], "PeakJerk_deg_s3", [NaN NaN], ...
    "VelocityMargin_deg_s", [NaN NaN], "AccelerationMargin_deg_s2", [NaN NaN], ...
    "JerkMargin_deg_s3", [NaN NaN], ...
    "VelocitySatisfied", false, "AccelerationSatisfied", false, "JerkSatisfied", false, ...
    "JerkConstrained", any(isfinite(limits.maxJerk_deg_s3)), "FiniteJerkCertified", false, ...
    "FiniteJerkNumericallyVerified", false, "ContinuousJerkCertified", false, ...
    "G3JoinCount", 0, "MinimumG3JoinSpeed_deg_s", NaN, ...
    "VelocityCarriedAcrossG3Joins", false, "JoinContinuityOrder", "G3", ...
    "GeometryDerivativeBounds", repmat(derivativeBoundsTemplate(), 0, 1), ...
    "SpatiallyVaryingLimits", true, "UniformTimeScaleFactor", NaN, ...
    "SpatialRetimingCellCount", 0, ...
    "ExecutedMotionProfileCount", 0, "MandatoryStopCount", 0, ...
    "MandatoryStopArcLength_deg", zeros(0, 1), "CurvatureDiscontinuityStopCount", 0, ...
    "RoundedVelocityCarried", false, "MinimumArcSpeed_deg_s", NaN, "Satisfied", false);
timedPath = struct( "Success", false, "Message", string(message), ...
    "time_s", zeros(0, 1), "position_deg", zeros(0, 2), "velocity_deg_s", zeros(0, 2), ...
    "acceleration_deg_s2", zeros(0, 2), "jerk_deg_s3", zeros(0, 2), ...
    "PathPosition_deg", zeros(0, 2), "WaypointTime_s", zeros(0, 1), ...
    "DepartureCandidateTime_s", NaN, "MotionStartTime_s", NaN, ...
    "WaitDuration_s", NaN, "MinimumMotionDuration_s", NaN, ...
    "GoalLineInterceptTime_s", NaN, "SegmentProfiles", repmat(profileTemplate(), 0, 1), ...
    "Limits", limits, "Options", struct("GoalTimeMode", options.GoalTimeMode, ...
        "SampleTime_s", options.SampleTime_s), ...
    "ConstraintDiagnostics", diagnostics, "SmoothPath", struct(), ...
    "CurveArcLength_deg", zeros(0, 1), "CurveNodeTime_s", zeros(0, 1), ...
    "CurveSpeed_deg_s", zeros(0, 1), "CurveSpeedSquared_deg2_s2", zeros(0, 1), ...
    "CurveTangentialAcceleration_deg_s2", zeros(0, 1), ...
    "CurveTangentialJerk_deg_s3", zeros(0, 1), "SampleArcLength_deg", zeros(0, 1), ...
    "SampleSpeed_deg_s", zeros(0, 1), "SampleTangentialAcceleration_deg_s2", zeros(0, 1), ...
    "SampleTangentialJerk_deg_s3", zeros(0, 1), "CurvatureDiscontinuityStopCount", 0, ...
    "RetimerType", "certifiedAnalyticSpatialJerk", "MotionType", "velocityCarrying");
end

function validation = validatePlan(planningSucceeded, endpointBlocked, ...
        timedPath, blocked, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   validation = validatePlan(planningSucceeded, endpointBlocked, ...
%       timedPath, blocked, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Independently check the returned state and certified motion bounds.
%**************************************************************************
% INPUTS
%   - planningSucceeded, endpointBlocked (logical scalars)
%       Search outcome and endpoint occupancy status.
%   - timedPath, goalState, limits, options (scalar structs)
%       Selected trajectory, request, physical limits, and time policy.
%   - blocked (logical vector)
%       Independent protected-geometry collision query result.
%**************************************************************************
% OUTPUTS
%   - validation (scalar struct)
%       Named checks, overall pass flag, and actionable message.
%**************************************************************************
% UNITS
%   - Tolerances use seconds and degree-based derivative units.
%**************************************************************************
hasMotion = timedPath.Success && numel(timedPath.time_s) >= 2;
finiteJerkAxis = isfinite(limits.maxJerk_deg_s3);
finiteJerkHistory = ~any(finiteJerkAxis) || ...
    all(isfinite(timedPath.jerk_deg_s3(:, finiteJerkAxis)), "all");
finiteState = hasMotion && all(isfinite(timedPath.time_s)) && ...
    all(isfinite(timedPath.position_deg(:))) && ...
    all(isfinite(timedPath.velocity_deg_s(:))) && ...
    all(isfinite(timedPath.acceleration_deg_s2(:))) && finiteJerkHistory;
strictTime = hasMotion && all(diff(timedPath.time_s) > 0);
endpointMatched = hasMotion && ...
    norm(timedPath.position_deg(end, :) - goalState.position_deg) <= 1e-7 && ...
    norm(timedPath.velocity_deg_s(end, :) - goalState.velocity_deg_s) <= 1e-7 && ...
    norm(timedPath.acceleration_deg_s2(end, :) - goalState.acceleration_deg_s2) <= 1e-7;
goalTimeSatisfied = hasMotion && timedPath.time_s(end) <= goalState.time_s + 1e-7;
if options.GoalTimeMode == "fixedarrival"
    goalTimeSatisfied = hasMotion && abs(timedPath.time_s(end) - goalState.time_s) <= 1e-7;
end
sampleVelocitySatisfied = hasMotion && all(max(abs(timedPath.velocity_deg_s), [], 1) <= ...
    limits.maxVelocity_deg_s + 1e-7);
sampleAccelerationSatisfied = hasMotion && all(max( ...
    abs(timedPath.acceleration_deg_s2), [], 1) <= limits.maxAcceleration_deg_s2 + 1e-7);
sampleJerkSatisfied = hasMotion && (~any(finiteJerkAxis) || ...
    all(max(abs(timedPath.jerk_deg_s3(:, finiteJerkAxis)), [], 1) <= ...
    limits.maxJerk_deg_s3(finiteJerkAxis) + 1e-7));
certificateSatisfied = hasMotion && timedPath.ConstraintDiagnostics.Satisfied;
collisionFree = hasMotion && ~any(blocked);
passed = planningSucceeded && ~endpointBlocked && finiteState && ...
    strictTime && endpointMatched && goalTimeSatisfied && ...
    sampleVelocitySatisfied && sampleAccelerationSatisfied && ...
    sampleJerkSatisfied && certificateSatisfied && collisionFree;
failedChecks = strings(0, 1);
checks = [planningSucceeded, ~endpointBlocked, finiteState, strictTime, ...
    endpointMatched, goalTimeSatisfied, sampleVelocitySatisfied, ...
    sampleAccelerationSatisfied, sampleJerkSatisfied, certificateSatisfied, collisionFree];
names = ["planningSucceeded" "endpointsClear" "finiteState" ...
    "strictTime" "endpointMatched" "goalTimeSatisfied" ...
    "sampleVelocitySatisfied" "sampleAccelerationSatisfied" ...
    "sampleJerkSatisfied" "certificateSatisfied" "collisionFree"];
for index = 1:numel(checks)
    if ~checks(index)
        failedChecks(end + 1, 1) = names(index); %#ok<AGROW>
    end
end
message = "All independent checks passed.";
if ~passed
    message = "Failed checks: " + strjoin(failedChecks, ", ");
end
validation = struct( "Passed", passed, "Message", message, ...
    "PlanningSucceeded", planningSucceeded, "EndpointBlocked", endpointBlocked, ...
    "EndpointClear", ~endpointBlocked, "ProtectedGeometryClear", collisionFree, ...
    "AzimuthPolicySatisfied", hasMotion && ...
        routeWithinAzimuthPolicy(timedPath.position_deg, options), ...
    "GoalReached", endpointMatched, "ArrivalWithinGoalTime", goalTimeSatisfied, ...
    "FiniteState", finiteState, "TimeStrictlyIncreasing", strictTime, ...
    "EndpointMatched", endpointMatched, "GoalTimeSatisfied", goalTimeSatisfied, ...
    "VelocitySatisfied", sampleVelocitySatisfied, ...
    "AccelerationSatisfied", sampleAccelerationSatisfied, ...
    "JerkSatisfied", sampleJerkSatisfied, "VelocityWithinLimits", sampleVelocitySatisfied, ...
    "AccelerationWithinLimits", sampleAccelerationSatisfied, ...
    "JerkWithinLimits", sampleJerkSatisfied, "CertificateSatisfied", certificateSatisfied, ...
    "CollisionFree", collisionFree);
end
