function [candidate, checkResult, solverDiagnostics, ...
        candidateWasPrechecked, precheckElapsedTime_s, stageTiming] = ...
        solveDynamicSeed( ...
        obstacles, initialState, goalState, limits, options, seed, stageTiming)
%% Section 0: Header & Readme
% SYNTAX
%   [candidate, checkResult, solverDiagnostics, ...
%       candidateWasPrechecked, precheckElapsedTime_s, stageTiming] = ...
%       obstacleAvoidance.planner.solveDynamicSeed( ...
%       obstacles, initialState, goalState, limits, options, seed, stageTiming)
%
% PURPOSE
%   - Coordinate dynamic-obstacle motion methods and explicit backups.
%   - Retain every attempted representation, check, and fallback outcome.
%
% INPUTS
%   - obstacles, initialState, goalState, limits, options
%       Normalized planning inputs in public planner order.
%   - seed (scalar route-seed struct)
%       Indexed timed or spatial proposal.
%   - stageTiming (scalar struct)
%       Accumulated planner timing before this seed.
%
% OUTPUTS
%   - candidate (scalar motion struct)
%       Selected dynamic-seed attempt or stable failure record.
%   - checkResult (scalar validation struct)
%       Authoritative check when an attempt already passed in this stage.
%   - solverDiagnostics (scalar struct)
%       Static projection, timed BMTP, direct-wait, and backup details.
%   - candidateWasPrechecked (logical scalar)
%       True only when candidate already passed validateTrajectory here.
%   - precheckElapsedTime_s (nonnegative scalar)
%       Full-validation time nested inside this motion stage.
%   - stageTiming (scalar struct)
%       Timing updated by nested authoritative checks.
%
% UNITS
%   - Position is degrees and time is seconds.
%

%% Section 1: Try A Conservative Static Projection

% Try static BMTP against geometry covering the complete moving history.
% Validate the motion against the original histories.

preparedObstacles = obstacles;
candidateWasPrechecked = false;
precheckElapsedTime_s = 0;
checkResult = obstacleAvoidance.validation.validatePreparedTrajectory();
trySweptProjection = string(seed.Source) ~= "directWait" && ...
    size(seed.position_deg, 1) > 2;
sweptAttempt = struct();
timedBmtpAttempt = struct();
if trySweptProjection
    solverGoalState = ...
        obstacleAvoidance.planner.createSolverGoalState( ...
        goalState, options);
    [planningObstacles, projection] = ...
        obstacleAvoidance.obstacles.createStaticPlanningProjection( ...
        preparedObstacles, initialState.time_s, goalState.time_s);
    [sweptCandidate, sweptDiagnostics] = ...
        obstacleAvoidance.planner.solveBmtpTrajectory( ...
        seed, planningObstacles, initialState, solverGoalState, ...
        limits, options);
    [sweptCandidate, sweptCheck, sweptCheckTime_s, stageTiming] = ...
        obstacleAvoidance.planner.checkCandidateMotion( ...
        sweptCandidate, preparedObstacles, initialState, goalState, ...
        limits, options, stageTiming, ...
        "The swept-projection BMTP kernel returned no trajectory.");
    precheckElapsedTime_s = ...
        precheckElapsedTime_s + sweptCheckTime_s;
    sweptAttempt = createSweptProjectionRecord( ...
        sweptDiagnostics, sweptCheck, projection);
    if sweptCheck.Passed
        candidate = sweptCandidate;
        checkResult = sweptCheck;
        solverDiagnostics = sweptDiagnostics;
        solverDiagnostics.SweptProjection = sweptAttempt;
        solverDiagnostics.DynamicObstacleRepresentation = ...
            "conservativeStaticProtectedHistoryConvexHull";
        candidate.SolverDiagnostics = solverDiagnostics;
        candidateWasPrechecked = true;
    end
end

%% Section 2: Try Timed-Cell BMTP

% If the static projection fails, try BMTP with the route's timed cells.
% Validate against the original moving obstacles.

tryTimedBmtp = trySweptProjection && ~candidateWasPrechecked && ...
    string(seed.Source) == "timeExpandedVisibilityGraph";
if tryTimedBmtp
    [timedCandidate, timedCheck, timedBmtpDiagnostics, ...
        timedCheckTime_s, stageTiming] = ...
        obstacleAvoidance.planner.solveTimedBmtpTrajectory( ...
        seed, preparedObstacles, initialState, goalState, limits, options, ...
        stageTiming);
    precheckElapsedTime_s = precheckElapsedTime_s + timedCheckTime_s;
    timedBmtpAttempt = struct( ...
        "Attempted", true, ...
        "SolverDiagnostics", timedBmtpDiagnostics, ...
        "FullObstacleValidation", timedCheck, ...
        "Outcome", "rejectedByFullValidation");
    if timedCheck.Passed
        timedBmtpAttempt.Outcome = "acceptedAfterFullValidation";
        candidate = timedCandidate;
        checkResult = timedCheck;
        solverDiagnostics = timedBmtpDiagnostics;
        solverDiagnostics.SweptProjection = sweptAttempt;
        solverDiagnostics.TimedBmtp = timedBmtpAttempt;
        candidate.SolverDiagnostics = solverDiagnostics;
        candidateWasPrechecked = true;
    end
end

%% Section 3: Create A Direct-Wait Motion When Applicable

% Try an initial wait and direct move; handle unsupported routes explicitly.

if ~candidateWasPrechecked
    [candidate, solverDiagnostics] = ...
        obstacleAvoidance.planner.createDirectWaitMotion( ...
        seed, initialState, goalState, limits, options, [], []);
    if trySweptProjection
        solverDiagnostics.SweptProjection = sweptAttempt;
        solverDiagnostics.TimedBmtp = timedBmtpAttempt;
        candidate.SolverDiagnostics = solverDiagnostics;
    end
end

%% Section 4: Apply The Explicit Waypoint Backup Policy

% Use the fallback only when enabled, and record both attempts.

timedTerminationReason = string(candidate.TerminationReason);
timedTopologyIsUnsupported = any(timedTerminationReason == ...
    ["unsupportedTimedMultiWaypointRoute", "invalidDirectWaitSeed"]);
if timedTopologyIsUnsupported
    timedDiagnostics = solverDiagnostics;
    if options.UnsupportedTimedTopologyPolicy == ...
            "ruckigStopAtWaypoints"
        [candidate, fallbackDiagnostics] = ...
            obstacleAvoidance.planner.createRuckigWaypointMotion( ...
            seed, initialState, goalState, limits, options);
        solverDiagnostics = combineFallbackDiagnostics( ...
            timedDiagnostics, fallbackDiagnostics, ...
            timedTerminationReason, true);
        if fallbackDiagnostics.Accepted
            candidate.Message = candidate.Message + ...
                " Every interior waypoint was constrained to rest " + ...
                "by the explicitly enabled Ruckig fallback.";
        else
            candidate.Message = ...
                "The explicitly enabled Ruckig stop-at-waypoints " + ...
                "fallback failed. " + candidate.Message;
            candidate.TerminationReason = ...
                "ruckigWaypointFallbackFailed";
            solverDiagnostics.FallbackOutcome = ...
                candidate.TerminationReason;
        end
        candidate.SolverDiagnostics = solverDiagnostics;
    else
        solverDiagnostics = combineFallbackDiagnostics( ...
            timedDiagnostics, struct(), timedTerminationReason, false);
        candidate.SolverDiagnostics = solverDiagnostics;
    end
end
end

%% Section 5: Local Functions

function diagnostics = combineFallbackDiagnostics( ...
        timedDiagnostics, fallbackDiagnostics, originalReason, attempted)
% Keep the original timed-kernel failure when recovery is attempted.
diagnostics = timedDiagnostics;
diagnostics.OriginalTerminationReason = originalReason;
diagnostics.FallbackAttempted = attempted;
diagnostics.FallbackMethod = "ruckigStopAtWaypoints";
if ~attempted
    diagnostics.FallbackOutcome = "fallbackDisabledByPolicy";
    return;
end
diagnostics.FallbackOutcome = ...
    string(fallbackDiagnostics.EngineTerminationReason);
diagnostics.FallbackDiagnostics = fallbackDiagnostics;
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

function record = createSweptProjectionRecord( ...
        diagnostics, checkResult, projection)
% Record the static projection and validation against moving obstacles.
record = struct( ...
    "Attempted", true, ...
    "Projection", projection, ...
    "SolverDiagnostics", diagnostics, ...
    "FullObstacleValidation", checkResult, ...
    "Outcome", "rejectedByFullValidation");
if checkResult.Passed
    record.Outcome = "acceptedAfterFullValidation";
end
end
