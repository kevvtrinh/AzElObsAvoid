function [candidate, checkResult, solverDiagnostics, ...
        candidateWasPrechecked, precheckElapsedTime_s, stageTiming] = ...
        solveDynamicSeed(seed, context, stageTiming)
%% Section 0: Header & Readme
% SYNTAX
%   [candidate, checkResult, solverDiagnostics, ...
%       candidateWasPrechecked, precheckElapsedTime_s, stageTiming] = ...
%       obstacleAvoidance.planner.solveDynamicSeed( ...
%       seed, context, stageTiming)
%**************************************************************************
% PURPOSE
%   - Coordinate dynamic-obstacle motion methods and explicit backups.
%   - Retain every attempted representation, check, and fallback outcome.
%**************************************************************************
% INPUTS
%   - seed (scalar route-seed struct)
%       Indexed timed or spatial proposal.
%   - context (scalar struct)
%       PreparedObstacles, InitialState, GoalState, Limits, and Options.
%   - stageTiming (scalar struct)
%       Accumulated planner timing before this seed.
%**************************************************************************
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
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds.
%**************************************************************************

%% Section 1: Try A Conservative Static Projection

% A multi-waypoint seed can first use the static BMTP engine if the complete
% moving history is projected into conservative static geometry. The resulting
% motion is still checked against the original histories before it can pass.

initialState = context.InitialState;
goalState = context.GoalState;
limits = context.Limits;
options = context.Options;
preparedObstacles = context.PreparedObstacles;
candidateWasPrechecked = false;
precheckElapsedTime_s = 0;
checkResult = obstacleAvoidance.validateTrajectory();
trySweptProjection = string(seed.Source) ~= "directWait" && ...
    size(seed.position_deg, 1) > 2;
sweptAttempt = struct();
timedBmtpAttempt = struct();
if trySweptProjection
    kernelGoalState = ...
        obstacleAvoidance.planner.createFixedKernelGoalState( ...
        goalState, options);
    [planningObstacles, projection] = ...
        obstacleAvoidance.obstacles.createStaticPlanningProjection( ...
        preparedObstacles, initialState.time_s, goalState.time_s);
    [sweptCandidate, sweptDiagnostics] = ...
        obstacleAvoidance.planner.solveBmtpTrajectory( ...
        seed, planningObstacles, initialState, kernelGoalState, ...
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

% If the conservative projection rejects a timed visibility route, preserve
% its actual time cells in the timed BMTP method. Again, only a full check
% against the prepared obstacle histories can retain the attempt.

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

% Remaining dynamic seeds go through the compact direct-wait construction.
% Unsupported multi-waypoint topology stays explicit so only the requested
% stop-at-waypoint policy can trigger its backup method.

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

% The backup method is never implicit. Retain the failed primary diagnostics,
% whether policy enabled the attempt, and the complete backup result.

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
% Preserve the earliest timed-kernel failure across an explicit recovery.
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
% Record conservative static mover geometry and authoritative validation.
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
