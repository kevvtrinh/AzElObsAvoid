function evaluation = evaluateAzElMotionCandidates( ...
        search, obstacleField, initialState, goalState, limits, options, ...
        endpointBlocked)
%% Section 0: Header & Readme
% SYNTAX
%   evaluation = azElInternal.evaluateAzElMotionCandidates( ...
%       search, obstacleField, initialState, goalState, limits, options, ...
%       endpointBlocked)
%**************************************************************************
% PURPOSE
%   - Evaluate, collision-check, rank, and validate every retained
%     visibility route and useful departure time.
%**************************************************************************
% INPUTS
%   - search (scalar struct)
%       Visibility graphs and diagnostics from buildAzElVisibilityRoutes.
%   - obstacleField (scalar packed obstacle field)
%       Complete protected obstacle history used for collision validation.
%   - initialState, goalState, limits, options (scalar structs)
%       Normalized request, physical limits, and resolved planner options.
%   - endpointBlocked (logical scalar)
%       Whether protected geometry contains either requested endpoint.
%**************************************************************************
% OUTPUTS
%   - evaluation (scalar struct)
%       Candidate trajectories, diagnostics, selected motion, and
%       independent validation evidence.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds. Derivative field suffixes
%     state degrees per second, squared second, or cubed second.
%**************************************************************************

%% Section 1: Collect Candidate Routes

routeGraphs = search.VisibilityGraphs;
if isfield(search, "MergedVisibilityGraphs") && ...
        ~isempty(search.MergedVisibilityGraphs)
    routeGraphs = [routeGraphs; search.MergedVisibilityGraphs(:)];
end
[candidateRoutes_deg, snapshotTime_s, graphIndex, consolidation] = ...
    collectRoutes(routeGraphs, initialState, goalState, ...
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

%% Section 2: Smooth, Retime & Check Every Candidate

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
        canReuseDirectTiming = size(route_deg, 1) == 2;
        cachedSmoothPath = struct();
        cachedTimedPath = struct();
        if canReuseDirectTiming
            [cachedSmoothPath, cachedTimedPath] = retimeCandidate( ...
                route_deg, obstacleField, initialState, goalState, ...
                limits, options, initialState.time_s);
        end
        if canReuseDirectTiming
            smoothPath = cachedSmoothPath;
            timedPath = rescheduleCoppPath( ...
                cachedTimedPath, initialState, goalState, options, ...
                departureCandidateTime_s(1));
        else
            [smoothPath, timedPath] = retimeCandidate( ...
                route_deg, obstacleField, initialState, goalState, ...
                limits, options, departureCandidateTime_s(1), ...
                "No departure time was feasible.");
        end
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
                initialState, goalState, limits, options, ...
                departureTime_s, arrivalValidationBound_s, ...
                cachedSmoothPath, cachedTimedPath);
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
                            attemptCollisionCheck] = refineClearDeparture( ...
                            route_deg, obstacleField, initialState, ...
                            goalState, limits, options, ...
                            previousDepartureTime_s, departureTime_s, ...
                            attemptSmoothPath, attemptTimedPath, ...
                            attemptBlocked, attemptCollisionCheck, ...
                            arrivalValidationBound_s, ...
                            cachedSmoothPath, cachedTimedPath);
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
        [smoothPath, timedPath] = retimeCandidate( ...
            route_deg, obstacleField, initialState, goalState, ...
            limits, options, initialState.time_s, ...
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
for candidateIndex = 2:candidateCount
    if graphIndex(candidateIndex) > 0 && ...
            isfield(routeGraphs, "Representation")
        representation = ...
            routeGraphs(graphIndex(candidateIndex)).Representation;
        if representation == "mergedObstacleGraph"
            source(candidateIndex) = representation;
        end
    end
end
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

%% Section 3: Select & Independently Validate One Result

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

message = ...
    "Adaptive visibility and numerically verified COPP retiming succeeded.";
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


%% Section 4: Assemble Candidate Evaluation

evaluation = struct( ...
    "CandidateCount", candidateCount, ...
    "CandidateRoutes_deg", {candidateRoutes_deg}, ...
    "CandidateDiagnostics", candidateDiagnostics, ...
    "CandidateTimedPaths", {candidateTimedPaths}, ...
    "CandidateSmoothPaths", {candidateSmoothPaths}, ...
    "CollisionFree", collisionFree, ...
    "CollisionPrefilterApplied", collisionPrefilterApplied, ...
    "CollisionPrefilterPassed", collisionPrefilterPassed, ...
    "ContinuousCollisionChecked", continuousCollisionChecked, ...
    "CollisionSkippedByArrivalBound", collisionSkippedByArrivalBound, ...
    "DepartureSearchTime_s", departureSearchTime_s, ...
    "Consolidation", consolidation, ...
    "BestAttemptPosition_deg", bestAttemptPosition_deg, ...
    "BestAttemptTime_s", bestAttemptTime_s, ...
    "SelectedCandidateIndex", selectedCandidateIndex, ...
    "SelectedRoute_deg", selectedRoute_deg, ...
    "TimedSlopePath", timedSlopePath, ...
    "SmoothPath", smoothPath, ...
    "DirectPosition_deg", directPosition_deg, ...
    "DirectTime_s", directTime_s, ...
    "DirectBlocked", logical(directBlocked(:)), ...
    "Validation", validation, ...
    "Success", success, ...
    "Message", message, ...
    "TerminationReason", terminationReason);
end

%% Section 5: Local Functions

function [smoothPath, timedPath, blocked, collisionCheck] = ...
        evaluateDeparture( ...
        route_deg, obstacleField, initialState, goalState, limits, ...
        options, departureTime_s, arrivalValidationBound_s, ...
        cachedSmoothPath, cachedTimedPath)
%% Section 0: Header & Readme
% SYNTAX
%   [smoothPath, timedPath, blocked, collisionCheck] = ...
%       evaluateDeparture( ...
%       route_deg, obstacleField, initialState, goalState, limits, ...
%       options, departureTime_s, arrivalValidationBound_s)
%   [smoothPath, timedPath, blocked, collisionCheck] = ...
%       evaluateDeparture( ...
%       route_deg, obstacleField, initialState, goalState, limits, ...
%       options, departureTime_s, arrivalValidationBound_s, ...
%       cachedSmoothPath, cachedTimedPath)
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
%   - cachedSmoothPath, cachedTimedPath (scalar structs, optional)
%       Reusable COPP result for a two-point path whose geometry cannot
%       change with departure time.
%**************************************************************************
% OUTPUTS
%   - smoothPath, timedPath, collisionCheck (scalar structs)
%   - blocked (logical vector)
%       Candidate motion and complete collision-validation evidence.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds.
%**************************************************************************
if nargin >= 10 && isstruct(cachedTimedPath) && ...
        isfield(cachedTimedPath, "Success")
    smoothPath = cachedSmoothPath;
    timedPath = rescheduleCoppPath( ...
        cachedTimedPath, initialState, goalState, options, ...
        departureTime_s);
else
    [smoothPath, timedPath] = retimeCandidate( ...
        route_deg, obstacleField, initialState, goalState, limits, ...
        options, departureTime_s);
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
    "BoundaryIsOccupied", false, ...
    "StopAtFirstCollision", true));
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

function [smoothPath, timedPath] = retimeCandidate( ...
        route_deg, obstacleField, initialState, goalState, limits, options, ...
        departureTime_s, failureMessage)
%% Section 0: Header & Readme
% SYNTAX
%   [smoothPath, timedPath] = retimeCandidate( ...
%       route_deg, obstacleField, initialState, goalState, limits, options, ...
%       departureTime_s)
%   [smoothPath, timedPath] = retimeCandidate( ...
%       route_deg, obstacleField, initialState, goalState, limits, options, ...
%       departureTime_s, failureMessage)
%**************************************************************************
% PURPOSE
%   - Build fixed collision-checked geometry, then retime it with COPP
%     TOPP3 without executing the superseded custom timing algorithm.
%**************************************************************************
% INPUTS
%   - route_deg (N-by-2 numeric), obstacleField (scalar packed field)
%   - initialState, goalState, limits, options (scalar structs)
%   - departureTime_s (finite scalar)
%   - failureMessage (scalar text, optional; default "")
%**************************************************************************
% OUTPUTS
%   - smoothPath, timedPath (stable scalar structs)
%**************************************************************************
% UNITS
%   - Positions are degrees, time is seconds, and derivative suffixes state
%     their degree-based units.
%**************************************************************************
if nargin < 8
    failureMessage = "";
end
smoothPath = azElInternal.buildAzElSmoothPath( ...
    route_deg, obstacleField, departureTime_s, options, failureMessage);
timedPath = azElExperimental.retimeCoppSmoothPath( ...
    smoothPath, initialState, goalState, limits, options, ...
    departureTime_s, failureMessage);
end

function timedPath = rescheduleCoppPath( ...
        cachedTimedPath, initialState, goalState, options, ...
        departureTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   timedPath = rescheduleCoppPath( ...
%       cachedTimedPath, initialState, goalState, options, departureTime_s)
%**************************************************************************
% PURPOSE
%   - Reuse one densely verified COPP profile for a two-point path while
%     changing only its permitted departure time and initial hold.
%**************************************************************************
% INPUTS
%   - cachedTimedPath (scalar successful COPP timed-path struct)
%       Profile solved at initialState.time_s for the same fixed line.
%   - initialState, goalState, options (scalar structs)
%   - departureTime_s (finite scalar)
%**************************************************************************
% OUTPUTS
%   - timedPath (stable scalar struct)
%       Rescheduled history or explicit timing failure. Geometry,
%       derivatives, limits, and numerical verification are unchanged.
%**************************************************************************
% UNITS
%   - Time is seconds and scientific histories retain degree-based units.
%**************************************************************************
timedPath = cachedTimedPath;
if ~cachedTimedPath.Success
    return;
end

minimumMotionDuration_s = cachedTimedPath.MinimumMotionDuration_s;
minimumWaitDuration_s = max(0, departureTime_s - initialState.time_s);
availableDuration_s = goalState.time_s - initialState.time_s;
goalTimeMode = lower(string(options.GoalTimeMode));
if goalTimeMode == "fixedarrival"
    waitDuration_s = availableDuration_s - minimumMotionDuration_s;
    arrivalTime_s = goalState.time_s;
elseif goalTimeMode == "earliestarrival"
    waitDuration_s = minimumWaitDuration_s;
    arrivalTime_s = initialState.time_s + waitDuration_s + ...
        minimumMotionDuration_s;
else
    error("planAzElMotion:InvalidGoalTimeMode", ...
        "GoalTimeMode must be earliestArrival or fixedArrival.");
end

timeTolerance_s = 1e-10 * max(1, abs(goalState.time_s));
if waitDuration_s + timeTolerance_s < minimumWaitDuration_s
    timedPath = failedCoppSchedule(timedPath, ...
        "The cached COPP motion cannot satisfy the requested departure.", ...
        "goalTimeInfeasible", departureTime_s);
    return;
end
if arrivalTime_s > goalState.time_s + timeTolerance_s
    timedPath = failedCoppSchedule(timedPath, ...
        "The cached COPP arrival exceeds the goal horizon.", ...
        "goalTimeInfeasible", departureTime_s);
    return;
end
initialStateIsMoving = norm(initialState.velocity_deg_s) > 1e-10 || ...
    norm(initialState.acceleration_deg_s2) > 1e-10;
if waitDuration_s > timeTolerance_s && initialStateIsMoving
    timedPath = failedCoppSchedule(timedPath, ...
        "An initial hold is incompatible with the moving initial state.", ...
        "movingHoldUnsupported", departureTime_s);
    return;
end

motionStartTime_s = initialState.time_s + waitDuration_s;
cachedMotionStartTime_s = cachedTimedPath.MotionStartTime_s;
motionMask = cachedTimedPath.time_s >= ...
    cachedMotionStartTime_s - timeTolerance_s;
relativeMotionTime_s = cachedTimedPath.time_s(motionMask) - ...
    cachedMotionStartTime_s;
if waitDuration_s > timeTolerance_s
    holdTime_s = unique([ ...
        (initialState.time_s:options.SampleTime_s:motionStartTime_s).'; ...
        motionStartTime_s]);
    holdTime_s(end) = [];
else
    holdTime_s = zeros(0, 1);
end
holdCount = numel(holdTime_s);
timedPath.time_s = [ ...
    holdTime_s; motionStartTime_s + relativeMotionTime_s];
timedPath.position_deg = [ ...
    repmat(initialState.position_deg, holdCount, 1); ...
    cachedTimedPath.position_deg(motionMask, :)];
timedPath.velocity_deg_s = [ ...
    zeros(holdCount, 2); cachedTimedPath.velocity_deg_s(motionMask, :)];
timedPath.acceleration_deg_s2 = [ ...
    zeros(holdCount, 2); ...
    cachedTimedPath.acceleration_deg_s2(motionMask, :)];
timedPath.jerk_deg_s3 = [ ...
    zeros(holdCount, 2); cachedTimedPath.jerk_deg_s3(motionMask, :)];

sampleFields = [ ...
    "SampleArcLength_deg", ...
    "SampleSpeed_deg_s", ...
    "SampleTangentialAcceleration_deg_s2", ...
    "SampleTangentialJerk_deg_s3"];
for sampleFieldIndex = 1:numel(sampleFields)
    fieldName = sampleFields(sampleFieldIndex);
    values = cachedTimedPath.(fieldName);
    if numel(values) == numel(motionMask)
        timedPath.(fieldName) = [ ...
            zeros(holdCount, 1); values(motionMask)];
    end
end

timeShift_s = motionStartTime_s - cachedMotionStartTime_s;
timedPath.WaypointTime_s = ...
    cachedTimedPath.WaypointTime_s + timeShift_s;
timedPath.CurveNodeTime_s = ...
    cachedTimedPath.CurveNodeTime_s + timeShift_s;
timedPath.DepartureCandidateTime_s = departureTime_s;
timedPath.MotionStartTime_s = motionStartTime_s;
timedPath.WaitDuration_s = waitDuration_s;
timedPath.ArrivalTime_s = arrivalTime_s;
timedPath.GoalLineInterceptTime_s = arrivalTime_s;
timedPath.Message = ...
    "Reused densely verified COPP timing for departure-only rescheduling.";
timedPath.TerminationReason = "goalReached";
end

function timedPath = failedCoppSchedule( ...
        timedPath, message, terminationReason, departureTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   timedPath = failedCoppSchedule( ...
%       timedPath, message, terminationReason, departureTime_s)
%**************************************************************************
% PURPOSE
%   - Convert a cached successful path into the stable timing-failure schema.
%**************************************************************************
% INPUTS
%   - timedPath (scalar struct), message, terminationReason (scalar text)
%   - departureTime_s (finite scalar)
%**************************************************************************
% OUTPUTS
%   - timedPath (scalar unsuccessful timed-path struct)
%**************************************************************************
% UNITS
%   - Time is seconds and empty histories retain degree-based columns.
%**************************************************************************
timedPath.Success = false;
timedPath.Message = string(message);
timedPath.TerminationReason = string(terminationReason);
timedPath.time_s = zeros(0, 1);
timedPath.position_deg = zeros(0, 2);
timedPath.velocity_deg_s = zeros(0, 2);
timedPath.acceleration_deg_s2 = zeros(0, 2);
timedPath.jerk_deg_s3 = zeros(0, 2);
timedPath.WaypointTime_s = zeros(0, 1);
timedPath.DepartureCandidateTime_s = departureTime_s;
timedPath.MotionStartTime_s = NaN;
timedPath.WaitDuration_s = NaN;
timedPath.ArrivalTime_s = NaN;
timedPath.GoalLineInterceptTime_s = NaN;
timedPath.CurveNodeTime_s = zeros(0, 1);
timedPath.SampleArcLength_deg = zeros(0, 1);
timedPath.SampleSpeed_deg_s = zeros(0, 1);
timedPath.SampleTangentialAcceleration_deg_s2 = zeros(0, 1);
timedPath.SampleTangentialJerk_deg_s3 = zeros(0, 1);
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
        clearBlocked, clearCollisionCheck, arrivalValidationBound_s, ...
        cachedSmoothPath, cachedTimedPath)
%% Section 0: Header & Readme
% SYNTAX
%   [clearSmoothPath, clearTimedPath, clearBlocked, ...
%       clearCollisionCheck] = ...
%       refineClearDeparture(route_deg, obstacleField, initialState, ...
%       goalState, limits, options, blockedDepartureTime_s, ...
%       clearDepartureTime_s, clearSmoothPath, clearTimedPath, ...
%       clearBlocked, clearCollisionCheck, arrivalValidationBound_s, ...
%       cachedSmoothPath, cachedTimedPath)
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
%   - cachedSmoothPath, cachedTimedPath (scalar structs)
%       Optional reusable direct-path COPP timing.
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
        arrivalValidationBound_s, cachedSmoothPath, cachedTimedPath);
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

function validation = validatePlan(planningSucceeded, endpointBlocked, ...
        timedPath, blocked, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   validation = validatePlan(planningSucceeded, endpointBlocked, ...
%       timedPath, blocked, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Independently check the returned state and numerical motion bounds.
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
hasNumericalVerification = hasMotion && ...
    isfield(timedPath.ConstraintDiagnostics, ...
    "FiniteJerkNumericallyVerified") && ...
    timedPath.ConstraintDiagnostics.FiniteJerkNumericallyVerified;
numericalVerificationSatisfied = hasMotion && ...
    timedPath.ConstraintDiagnostics.Satisfied && ...
    hasNumericalVerification;
collisionFree = hasMotion && ~any(blocked);
passed = planningSucceeded && ~endpointBlocked && finiteState && ...
    strictTime && endpointMatched && goalTimeSatisfied && ...
    sampleVelocitySatisfied && sampleAccelerationSatisfied && ...
    sampleJerkSatisfied && numericalVerificationSatisfied && collisionFree;
failedChecks = strings(0, 1);
checks = [planningSucceeded, ~endpointBlocked, finiteState, strictTime, ...
    endpointMatched, goalTimeSatisfied, sampleVelocitySatisfied, ...
    sampleAccelerationSatisfied, sampleJerkSatisfied, ...
    numericalVerificationSatisfied, collisionFree];
names = ["planningSucceeded" "endpointsClear" "finiteState" ...
    "strictTime" "endpointMatched" "goalTimeSatisfied" ...
    "sampleVelocitySatisfied" "sampleAccelerationSatisfied" ...
    "sampleJerkSatisfied" "numericalVerificationSatisfied" ...
    "collisionFree"];
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
    "JerkWithinLimits", sampleJerkSatisfied, ...
    "NumericalVerificationSatisfied", numericalVerificationSatisfied, ...
    "CertificateSatisfied", false, ...
    "CollisionFree", collisionFree);
end
