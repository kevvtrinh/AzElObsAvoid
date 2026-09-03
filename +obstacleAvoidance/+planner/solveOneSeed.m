function [candidate, summary, checkResult, stageTiming] = ...
        solveOneSeed(seed, context, stageTiming)
%% Section 0: Header & Readme
% SYNTAX
%   [candidate, summary, checkResult, stageTiming] = ...
%       obstacleAvoidance.planner.solveOneSeed( ...
%       seed, context, stageTiming)
%**************************************************************************
% PURPOSE
%   - Select and run the primary motion method for one deterministic seed.
%   - Apply explicit backups and retain only full-check acceptance evidence.
%**************************************************************************
% INPUTS
%   - seed (scalar route-seed struct)
%       Indexed route suggestion and timing estimate.
%   - context (scalar struct)
%       Engine choice, prepared request, options, and summary template.
%   - stageTiming (scalar struct)
%       Accumulated planner timing before this seed.
%**************************************************************************
% OUTPUTS
%   - candidate (scalar motion struct)
%       Final attempted motion, including explicit fallback diagnostics.
%   - summary (scalar candidate-summary struct)
%       Stable solve, check, objective, and timing evidence.
%   - checkResult (scalar validation struct)
%       Authoritative result from obstacleAvoidance.validateTrajectory.
%   - stageTiming (scalar struct)
%       Motion-solving and checking time accumulated through this seed.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds; derivatives use deg/s,
%     deg/s^2, and deg/s^3.
%**************************************************************************

%% Section 1: Choose And Solve The Primary Motion Method

% Engine choice follows only resolved options and whether obstacle histories
% are static. Dynamic obstacle meaning stays outside the dimension-neutral
% BMTP and Ruckig engines in the dedicated dynamic-seed coordinator.

obstacleAvoidance.input.throwIfCancellationRequested(context.Options);
motionTimer = tic;
candidateWasPrechecked = false;
precheckElapsedTime_s = 0;
checkResult = obstacleAvoidance.validateTrajectory();
initialState = context.InitialState;
goalState = context.GoalState;
limits = context.Limits;
options = context.Options;
preparedObstacles = context.PreparedObstacles;
if context.UseRuckigWaypoint
    [candidate, solverDiagnostics] = ...
        obstacleAvoidance.planner.createRuckigWaypointMotion( ...
        seed, initialState, goalState, limits, options);
elseif context.UseStaticKernel
    kernelGoalState = ...
        obstacleAvoidance.planner.createFixedKernelGoalState( ...
        goalState, options);
    [candidate, solverDiagnostics] = ...
        obstacleAvoidance.planner.solveBmtpTrajectory( ...
        seed, preparedObstacles, initialState, kernelGoalState, ...
        limits, options);
else
    [candidate, checkResult, solverDiagnostics, ...
        candidateWasPrechecked, precheckElapsedTime_s, stageTiming] = ...
        obstacleAvoidance.planner.solveDynamicSeed( ...
        seed, context, stageTiming);
end

%% Section 2: Run The Full Motion Check

% Primary methods return candidate motions, not obstacle-avoidance approval.
% Skip this call only when the dynamic stage already ran the identical full
% check against the original prepared obstacle histories.

elapsedTime_s = toc(motionTimer) - precheckElapsedTime_s;
obstacleAvoidance.input.throwIfCancellationRequested(options);
stageTiming.MotionSolvingElapsedTime_s = ...
    stageTiming.MotionSolvingElapsedTime_s + elapsedTime_s;
if ~candidateWasPrechecked
    [candidate, checkResult, ~, stageTiming] = ...
        obstacleAvoidance.planner.checkCandidateMotion( ...
        candidate, preparedObstacles, initialState, goalState, limits, ...
        options, stageTiming, "The motion kernel returned no trajectory.");

    %% Section 3: Refine A Passing Direct Wait When Useful

    % Only policies whose objective changes with arrival time benefit from
    % reducing a wait. Every refinement trial is checked in full and the best
    % passing bracket endpoint remains explicit in solver diagnostics.

    waitRefinementAffectsObjective = ...
        options.GoalTimeMode == "earliestArrival" || ...
        (options.GoalTimeMode == "balancedArrival" && ...
        options.MinimumTravelSavingsRate_deg_s > 0);
    if checkResult.Passed && string(seed.Source) == "directWait" && ...
            waitRefinementAffectsObjective
        [candidate, checkResult, solverDiagnostics, ...
            refinementElapsedTime_s, stageTiming] = ...
            obstacleAvoidance.planner.refineDirectWait( ...
            seed, candidate, checkResult, solverDiagnostics, ...
            preparedObstacles, initialState, goalState, limits, options, ...
            stageTiming);
        elapsedTime_s = elapsedTime_s + refinementElapsedTime_s;
        stageTiming.MotionSolvingElapsedTime_s = ...
            stageTiming.MotionSolvingElapsedTime_s + ...
            refinementElapsedTime_s;
    end
end

%% Section 4: Create The Candidate Summary

% Candidate selection consumes only stable summaries. Preserve all primary,
% backup, and validation evidence here so ranking never needs to reconstruct
% how this motion was produced.

summary = obstacleAvoidance.planner.createCandidateSummary( ...
    candidate, checkResult, solverDiagnostics, elapsedTime_s, ...
    context.SummaryTemplate, limits, options, initialState.time_s);
end
