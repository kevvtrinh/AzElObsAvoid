function [candidate, summary, stageTiming, context] = solvePathGuess(obstacles, initialState, goalState, limits, options, seed, context, stageTiming)
%% Section 0: Header & Readme
% SYNTAX
%   [candidate, summary, stageTiming, context] = solvePathGuess( ...
%       obstacles, initialState, goalState, limits, options, seed, context, stageTiming)
% PURPOSE
%   Construct and independently validate motion for one path guess.
% INPUTS
%   Normalized obstacles, states, limits, options, and indexed path guess.
%   context holds solver choice, summary template, and reusable geometry.
%   stageTiming contains accumulated exclusive times.
% OUTPUTS
%   candidate and summary retain the attempted motion, validation, and diagnostics.
%   Updated stageTiming and context are reused by later guesses in this request.
% UNITS
%   Degrees, seconds, and derivatives in deg/s, deg/s^2, and deg/s^3.

%% Section 1: Choose And Solve The Primary Motion Method

% Use the stationary-obstacle solver only when the whole scene stays stationary.
motionTimer            = tic;
candidateWasPrechecked = false;
precheckElapsedTime_s  = 0;
checkResult            = obstacleAvoidance.validation.validatePreparedTrajectory();
preparedObstacles      = obstacles;
% Dispatch non-rest endpoint requests to the state-to-state engine; rest-to-rest requests use the specialized planner paths.
if context.UseStateToStateSolver
    [candidate, solverDiagnostics] = obstacleAvoidance.planner.createRuckigWaypointMotion(seed, initialState, goalState, limits, options);
% Use exact static-region BMTP when geometry is stationary; moving geometry proceeds through the dynamic solver.
elseif context.UseStaticSolver
    if isempty(fieldnames(context.StaticGeometry))
        context.StaticGeometry = obstacleAvoidance.planner.prepareStaticSolverGeometry(preparedObstacles, initialState.time_s, goalState.time_s);
    end
    [candidate, solverDiagnostics] = obstacleAvoidance.planner.solveStaticBmtpTrajectory(seed, context.StaticGeometry, initialState, goalState, limits, options);
else
    [candidate, checkResult, solverDiagnostics, ...
        candidateWasPrechecked, precheckElapsedTime_s, stageTiming, context] = obstacleAvoidance.planner.solveDynamicPathGuess(preparedObstacles, initialState, goalState, limits, options, seed, stageTiming, context);
end

%% Section 2: Run The Full Motion Check

% Skip validation only if the dynamic solver already ran the same full check.

elapsedTime_s = toc(motionTimer) - precheckElapsedTime_s;
stageTiming.MotionSolvingElapsedTime_s = stageTiming.MotionSolvingElapsedTime_s + elapsedTime_s;
% Validate newly constructed candidates here; prechecked candidates reuse the earlier independent verdict.
if ~candidateWasPrechecked
    [candidate, checkResult, ~, stageTiming] = obstacleAvoidance.planner.checkCandidateMotion(candidate, preparedObstacles, initialState, goalState, limits, options, stageTiming, "The motion kernel returned no trajectory.");

    % Refine the wait for earliest arrival.

    waitRefinementAffectsObjective = options.GoalTimeMode == "earliestArrival";
    % Refine a successful direct-wait motion only when waiting can change the selected objective.
    if candidate.Success && string(seed.Source) == "directWait" && waitRefinementAffectsObjective
        [candidate, checkResult, solverDiagnostics, ...
            refinementElapsedTime_s, stageTiming] = refineDirectWait(seed, candidate, checkResult, solverDiagnostics, preparedObstacles, initialState, goalState, limits, options, stageTiming);
        elapsedTime_s = elapsedTime_s + refinementElapsedTime_s;
        stageTiming.MotionSolvingElapsedTime_s = stageTiming.MotionSolvingElapsedTime_s + refinementElapsedTime_s;
    end
end

%% Section 3: Create The Candidate Summary

% Record the solve, fallback, and validation results for candidate selection.

summary = obstacleAvoidance.planner.createCandidateSummary(candidate, checkResult, solverDiagnostics, elapsedTime_s, context.SummaryTemplate, limits);
end

%% Section 4: Local Functions

function [candidate, checkResult, diagnostics, motionElapsedTime_s, stageTiming] = refineDirectWait(seed, candidate, checkResult, diagnostics, obstacles, initialState, goalState, limits, options, stageTiming)
    % Keep only validated improvements to the move and initial wait.
    initialWaitTime_s = diagnostics.WaitTime_s;
    diagnostics.InitialWaitTime_s = initialWaitTime_s;
    diagnostics.FinalWaitTime_s   = initialWaitTime_s;
    motionElapsedTime_s = 0;
    % Keep the initial direct-wait candidate when refinement is disabled or there is no positive wait to shorten.
    if options.MaximumWaitRefinementIterations == 0 || initialWaitTime_s <= 0
        candidate.SolverDiagnostics = diagnostics;
        return;
    end
    directMotionDuration_s = candidate.TrajectoryDuration_s - initialWaitTime_s;
    diagnostics.InitialDirectMotionDuration_s = directMotionDuration_s;
    diagnostics.FinalDirectMotionDuration_s   = directMotionDuration_s;
    % Time compression preserves the path but changes obstacle encounters.
    % Use derivative bounds to propose it, then validate the full motion.
    durationScale = max([checkResult.PeakVelocity_deg_s ./ limits.maxVelocity_deg_s, sqrt(checkResult.PeakAcceleration_deg_s2 ./ limits.maxAcceleration_deg_s2), nthroot(checkResult.PeakJerk_deg_s3 ./ limits.maxJerk_deg_s3, 3)]);
    % Leave roundoff slack when converting derivative ratios back to a duration.
    shorterDuration_s = directMotionDuration_s * durationScale * (1 + 64 * eps);
    lowerWaitTime_s   = 0;
    upperWaitTime_s   = initialWaitTime_s;
    bestCandidate     = candidate;
    bestCheckResult   = checkResult;

    % Validate every trial; bisection alone does not prove feasibility.
    for refinementIndex = 0:options.MaximumWaitRefinementIterations
        trialDuration_s = directMotionDuration_s;
        if refinementIndex == 0
            if ~isfinite(shorterDuration_s) || shorterDuration_s >= directMotionDuration_s - options.ArrivalTimeTolerance_s
                continue;
            end
            diagnostics.DirectRetimingAttempted = true;
            trialDuration_s = shorterDuration_s;
            trialWaitTime_s = initialWaitTime_s;
        % If no candidate validates, return the best failed check so the caller receives actionable diagnostics.
        elseif ~bestCheckResult.Passed
            break;
        elseif refinementIndex == 1
            trialWaitTime_s = lowerWaitTime_s;
        else
            trialWaitTime_s = 0.5 * (lowerWaitTime_s + upperWaitTime_s);
        end
        motionTimer = tic;
        [trialCandidate, ~] = obstacleAvoidance.planner.createWaitThenMoveMotion(seed, initialState, goalState, limits, options, trialWaitTime_s, trialDuration_s);
        motionElapsedTime_s = motionElapsedTime_s + toc(motionTimer);
        [trialCandidate, trialCheckResult, ~, stageTiming] = obstacleAvoidance.planner.checkCandidateMotion(trialCandidate, obstacles, initialState, goalState, limits, options, stageTiming, "The refined direct-wait kernel returned no trajectory.");
        diagnostics.RefinementCount = refinementIndex;
        % Promote a shortened wait only after independent validation; invalid trials leave the prior passing wait unchanged.
        if trialCheckResult.Passed
            bestCandidate   = trialCandidate;
            bestCheckResult = trialCheckResult;
            upperWaitTime_s = trialWaitTime_s;
            if refinementIndex == 0
                directMotionDuration_s = trialDuration_s;
                diagnostics.DirectRetimingAccepted      = true;
                diagnostics.FinalDirectMotionDuration_s = trialDuration_s;
            end
            if trialWaitTime_s == 0
                break;
            end
        % Report refinement-budget exhaustion only after at least one refinement trial was attempted.
        elseif refinementIndex > 0
            lowerWaitTime_s = trialWaitTime_s;
            diagnostics.InfeasibleLowerWaitTime_s = lowerWaitTime_s;
        end
    end
    candidate   = bestCandidate;
    checkResult = bestCheckResult;
    diagnostics.WaitTime_s      = upperWaitTime_s;
    diagnostics.FinalWaitTime_s = upperWaitTime_s;
    diagnostics.ElapsedTime_s   = diagnostics.ElapsedTime_s + motionElapsedTime_s;
    candidate.SolverDiagnostics = diagnostics;
end
