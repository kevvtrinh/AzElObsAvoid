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
evaluation = azElInternal.evaluateAzElMotionCandidates( ...
    search, obstacleField, initialState, goalState, limits, options, ...
    endpointBlocked);

%% Section 3: Unpack Candidate Evaluation

candidateCount = evaluation.CandidateCount;
candidateRoutes_deg = evaluation.CandidateRoutes_deg;
candidateDiagnostics = evaluation.CandidateDiagnostics;
collisionFree = evaluation.CollisionFree;
collisionPrefilterApplied = evaluation.CollisionPrefilterApplied;
collisionPrefilterPassed = evaluation.CollisionPrefilterPassed;
continuousCollisionChecked = evaluation.ContinuousCollisionChecked;
collisionSkippedByArrivalBound = ...
    evaluation.CollisionSkippedByArrivalBound;
departureSearchTime_s = evaluation.DepartureSearchTime_s;
consolidation = evaluation.Consolidation;
bestAttemptPosition_deg = evaluation.BestAttemptPosition_deg;
bestAttemptTime_s = evaluation.BestAttemptTime_s;
selectedCandidateIndex = evaluation.SelectedCandidateIndex;
selectedRoute_deg = evaluation.SelectedRoute_deg;
timedSlopePath = evaluation.TimedSlopePath;
smoothPath = evaluation.SmoothPath;
directPosition_deg = evaluation.DirectPosition_deg;
directTime_s = evaluation.DirectTime_s;
directBlocked = evaluation.DirectBlocked;
validation = evaluation.Validation;
success = evaluation.Success;
message = evaluation.Message;
terminationReason = evaluation.TerminationReason;

%% Section 4: Select & Independently Validate One Result

% Selection and independent validation are performed together inside the
% candidate-evaluation module so every retained candidate follows the same
% collision and kinematic acceptance policy.

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
