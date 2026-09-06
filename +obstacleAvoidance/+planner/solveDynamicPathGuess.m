function [candidate, checkResult, solverDiagnostics, candidateWasPrechecked, precheckElapsedTime_s, stageTiming, context] = solveDynamicPathGuess(obstacles, initialState, goalState, limits, options, seed, stageTiming, context)
%% Section 0: Header & Readme
% SYNTAX
%   [candidate, checkResult, solverDiagnostics, candidateWasPrechecked, ...
%       precheckElapsedTime_s, stageTiming, context] = solveDynamicPathGuess( ...
%       obstacles, initialState, goalState, limits, options, seed, stageTiming, context)
% PURPOSE
%   Try stationary enclosures, timed BMTP, wait-then-move, and explicit fallback.
% INPUTS
%   Normalized obstacles, states, limits, options, and indexed path guess.
%   stageTiming contains accumulated times; context holds the summary template
%   and reusable enclosure geometry for this request.
% OUTPUTS
%   candidate and checkResult retain paired motion and validation evidence.
%   solverDiagnostics records attempts and fallback outcomes.
%   candidateWasPrechecked says whether the returned motion was fully checked;
%   precheckElapsedTime_s accounts for that nested validation time.
%   Updated stageTiming and context are reused by later guesses.
% UNITS
%   Degrees, seconds, and derivatives in deg/s, deg/s^2, and deg/s^3.

%% Section 1: Try A Conservative Static Projection

% Try static BMTP against geometry covering the complete moving history.
% Validate the motion against the original histories.

preparedObstacles      = obstacles;
candidateWasPrechecked = false;
precheckElapsedTime_s  = 0;
checkResult            = obstacleAvoidance.validation.validatePreparedTrajectory();
trySweptProjection     = string(seed.Source) ~= "directWait" && size(seed.position_deg, 1) > 2;
sweptAttempt           = struct();
timedBmtpAttempt       = struct();
rejectedCandidates     = {};
rejectedChecks         = {};
if trySweptProjection
    if isempty(fieldnames(context.EnclosureGeometry))
        [planningObstacles, context.Enclosure] = obstacleAvoidance.obstacles.createStationaryObstacleEnclosures(preparedObstacles, initialState.time_s, goalState.time_s);
        context.EnclosureGeometry = obstacleAvoidance.planner.prepareStaticSolverGeometry(planningObstacles, initialState.time_s, goalState.time_s);
    end
    projection = context.Enclosure;
    [sweptCandidate, sweptDiagnostics]                          = obstacleAvoidance.planner.solveStaticBmtpTrajectory(seed, context.EnclosureGeometry, initialState, goalState, limits, options);
    [sweptCandidate, sweptCheck, sweptCheckTime_s, stageTiming] = obstacleAvoidance.planner.checkCandidateMotion(sweptCandidate, preparedObstacles, initialState, goalState, limits, options, stageTiming, "The swept-projection BMTP kernel returned no trajectory.");
    precheckElapsedTime_s = precheckElapsedTime_s + sweptCheckTime_s;
    sweptAttempt          = createSweptProjectionRecord(sweptDiagnostics, sweptCheck, projection);
    % Retain the rejected motion as diagnostic evidence while excluding it from final selection.
    if ~sweptCheck.Passed && ~isempty(sweptCandidate.time_s)
        rejectedCandidates{end + 1} = sweptCandidate;
        rejectedChecks{end + 1} = sweptCheck;
    end
    if sweptCheck.Passed
        candidate         = sweptCandidate;
        checkResult       = sweptCheck;
        solverDiagnostics = sweptDiagnostics;
        solverDiagnostics.SweptProjection               = sweptAttempt;
        solverDiagnostics.DynamicObstacleRepresentation = "conservativeStaticProtectedHistoryConvexHull";
        candidate.SolverDiagnostics = solverDiagnostics;
        candidateWasPrechecked = true;
    end
end

%% Section 2: Try Timed-Cell BMTP

% If the static projection fails, try BMTP with the route's timed cells.
% Validate against the original moving obstacles.

tryTimedBmtp = trySweptProjection && ~candidateWasPrechecked && string(seed.Source) == "timeExpandedVisibilityGraph";
if tryTimedBmtp
    [timedCandidate, timedCheck, timedBmtpDiagnostics, ...
        timedCheckTime_s, stageTiming] = obstacleAvoidance.planner.solveTimedBmtpTrajectory(seed, preparedObstacles, initialState, goalState, limits, options, stageTiming);
    precheckElapsedTime_s = precheckElapsedTime_s + timedCheckTime_s;
    timedBmtpAttempt      = struct("Attempted", true, ...
        "SolverDiagnostics", timedBmtpDiagnostics, ...
        "FullObstacleValidation", timedCheck, ...
        "Outcome", "rejectedByFullValidation");
    % Retain the rejected motion as diagnostic evidence while excluding it from final selection.
    if ~timedCheck.Passed && ~isempty(timedCandidate.time_s)
        rejectedCandidates{end + 1} = timedCandidate;
        rejectedChecks{end + 1} = timedCheck;
    end
    if timedCheck.Passed
        timedBmtpAttempt.Outcome = "acceptedAfterFullValidation";
        candidate         = timedCandidate;
        checkResult       = timedCheck;
        solverDiagnostics = timedBmtpDiagnostics;
        solverDiagnostics.SweptProjection = sweptAttempt;
        solverDiagnostics.TimedBmtp       = timedBmtpAttempt;
        candidate.SolverDiagnostics = solverDiagnostics;
        candidateWasPrechecked = true;
    end
end

%% Section 3: Create A Direct-Wait Motion When Applicable

% Try an initial wait and direct move; handle unsupported routes explicitly.

if ~candidateWasPrechecked
    % Only direct-wait seeds use this constructor; other seed types continue through their matching motion method.
    if string(seed.Source) == "directWait"
        [candidate, solverDiagnostics] = obstacleAvoidance.planner.createWaitThenMoveMotion(seed, initialState, goalState, limits, options, [], []);
    else
        [candidate, solverDiagnostics] = unsupportedPathGuess(seed, initialState, options);
    end
    if trySweptProjection
        solverDiagnostics.SweptProjection = sweptAttempt;
        solverDiagnostics.TimedBmtp       = timedBmtpAttempt;
        candidate.SolverDiagnostics = solverDiagnostics;
    end
end

%% Section 4: Apply The Explicit Waypoint Backup Policy

% Use the fallback only when enabled, and record both attempts.

timedTerminationReason     = string(candidate.TerminationReason);
timedTopologyIsUnsupported = any(timedTerminationReason == ["unsupportedTimedMultiWaypointRoute", "invalidDirectWaitSeed", "unsupportedDynamicDirectGuess"]);
if timedTopologyIsUnsupported
    timedDiagnostics = solverDiagnostics;
    if options.UnsupportedTimedTopologyPolicy == "ruckigStopAtWaypoints"
        [candidate, fallbackDiagnostics] = obstacleAvoidance.planner.createRuckigWaypointMotion(seed, initialState, goalState, limits, options);
        solverDiagnostics = combineFallbackDiagnostics(timedDiagnostics, fallbackDiagnostics, timedTerminationReason, true);
        % Adopt the fallback motion only after its own checks accept it; otherwise preserve the original failure evidence.
        if fallbackDiagnostics.Accepted
            candidate.Message = candidate.Message + " Every interior waypoint was constrained to rest " + "by the explicitly enabled Ruckig fallback.";
        else
            candidate.Message           = "The explicitly enabled Ruckig stop-at-waypoints " + "fallback failed. " + candidate.Message;
            candidate.TerminationReason = "ruckigWaypointFallbackFailed";
            solverDiagnostics.FallbackOutcome = candidate.TerminationReason;
        end
        candidate.SolverDiagnostics = solverDiagnostics;
    else
        solverDiagnostics = combineFallbackDiagnostics(timedDiagnostics, struct(), timedTerminationReason, false);
        candidate.SolverDiagnostics = solverDiagnostics;
    end
end
% Keep a constructed rejected motion instead of replacing it with an empty
% eligibility failure. Preserve all attempted-method diagnostics separately.
if ~candidate.Success && ~isempty(rejectedCandidates)
    summaries = repmat(context.SummaryTemplate, numel(rejectedCandidates), 1);
    % Process each item needed to find dynamic path guess.
    for index = 1:numel(rejectedCandidates)
        summaries(index) = obstacleAvoidance.planner.createCandidateSummary(rejectedCandidates{index}, rejectedChecks{index}, struct(), 0, context.SummaryTemplate, limits);
    end
    selection   = obstacleAvoidance.planner.selectValidatedCandidate(summaries, options);
    index       = selection.BestPartialSeedIndex;
    candidate   = rejectedCandidates{index};
    checkResult = rejectedChecks{index};
    candidate.SolverDiagnostics = solverDiagnostics;
    candidateWasPrechecked = true;
end

end

%% Section 5: Local Functions

function diagnostics = combineFallbackDiagnostics(timedDiagnostics, fallbackDiagnostics, originalReason, attempted)
    % Keep the original timed-kernel failure when recovery is attempted.
    diagnostics = timedDiagnostics;
    diagnostics.OriginalTerminationReason = originalReason;
    diagnostics.FallbackAttempted         = attempted;
    diagnostics.FallbackMethod            = "ruckigStopAtWaypoints";
    if ~attempted
        diagnostics.FallbackOutcome = "fallbackDisabledByPolicy";
        return;
    end
    diagnostics.FallbackOutcome     = string(fallbackDiagnostics.EngineTerminationReason);
    diagnostics.FallbackDiagnostics = fallbackDiagnostics;
    % Apply the required validation or transfer to each field name.
    for fieldName = ["InteriorWaypointTime_s", ...
            "InteriorWaypointPosition_deg", ...
            "InteriorWaypointVelocity_deg_s", ...
            "InteriorWaypointAcceleration_deg_s2", ...
            "AllInteriorWaypointsConstrainedToRest"]
        if isfield(fallbackDiagnostics, fieldName)
            diagnostics.(fieldName) = fallbackDiagnostics.(fieldName);
        end
    end
end

function record = createSweptProjectionRecord(diagnostics, checkResult, projection)
    % Record the static projection and validation against moving obstacles.
    record = struct("Attempted", true, ...
        "Projection", projection, ...
        "SolverDiagnostics", diagnostics, ...
        "FullObstacleValidation", checkResult, ...
        "Outcome", "rejectedByFullValidation");
    if checkResult.Passed
        record.Outcome = "acceptedAfterFullValidation";
    end
end

function [candidate, diagnostics] = unsupportedPathGuess(seed, initialState, options)
    % Record an ineligible guess without invoking the wait-motion constructor.
    candidate = bmtpEngine.createMotionRecord(struct(), initialState, [], [], options.SampleTime_s, seed.Source);
    candidate.SeedIndex = seed.Index;
    reason  = "unsupportedTimedMultiWaypointRoute";
    feature = "multiWaypointTimedRoute";
    % Classify two-point seeds as unsupported direct guesses; longer seeds remain unsupported timed multi-waypoint routes.
    if size(seed.position_deg, 1) <= 2
        reason  = "unsupportedDynamicDirectGuess";
        feature = "directGuessWithoutWaitSchedule";
    end
    candidate.TerminationReason = reason;
    candidate.Message           = "This path guess has no supported timed-motion construction.";
    diagnostics = struct("Accepted", false, "TerminationReason", reason, ...
        "OriginalTerminationReason", reason, "FirstUnsupportedFeature", feature, ...
        "FirstUnsupportedTransitionIndex", 1, "FallbackPolicy", options.UnsupportedTimedTopologyPolicy, ...
        "WaypointPosition_deg", seed.position_deg, "Tau", seed.tau);
    candidate.SolverDiagnostics = diagnostics;
end
