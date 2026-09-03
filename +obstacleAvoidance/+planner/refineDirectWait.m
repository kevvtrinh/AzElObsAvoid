function [candidate, checkResult, diagnostics, motionElapsedTime_s, ...
        stageTiming] = refineDirectWait( ...
        seed, candidate, checkResult, diagnostics, obstacles, initialState, ...
        goalState, limits, options, stageTiming)
%% Section 0: Header & Readme
% SYNTAX
%   [candidate, checkResult, diagnostics, motionElapsedTime_s, ...
%       stageTiming] = obstacleAvoidance.planner.refineDirectWait( ...
%       seed, candidate, checkResult, diagnostics, obstacles, initialState, ...
%       goalState, limits, options, stageTiming)
%**************************************************************************
% PURPOSE
%   - Reduce a passing direct wait through a checked infeasible/feasible bracket.
%**************************************************************************
% INPUTS
%   - seed, candidate, checkResult, diagnostics (scalar structs)
%       Passing direct-wait attempt and its complete evidence.
%   - obstacles (prepared canonical obstacle struct array)
%       Complete protected histories for every refinement check.
%   - initialState, goalState, limits, options (scalar structs)
%       Normalized request and resolved controls.
%   - stageTiming (scalar struct)
%       Accumulated planner timing before refinement.
%**************************************************************************
% OUTPUTS
%   - candidate, checkResult, diagnostics (scalar structs)
%       Best passing wait and its updated evidence.
%   - motionElapsedTime_s (nonnegative scalar)
%       Motion-construction time excluding full validation.
%   - stageTiming (scalar struct)
%       Timing updated by every authoritative check.
%**************************************************************************
% UNITS
%   - Position is degrees and all times are seconds.
%**************************************************************************

%% Section 1: Create A Wait-Time Bracket

initialWaitTime_s = diagnostics.WaitTime_s;
diagnostics.InitialWaitTime_s = initialWaitTime_s;
diagnostics.FinalWaitTime_s = initialWaitTime_s;
motionElapsedTime_s = 0;
if options.MaximumWaitRefinementIterations == 0 || initialWaitTime_s <= 0
    candidate.SolverDiagnostics = diagnostics;
    return;
end
directMotionDuration_s = candidate.MotionDuration_s - initialWaitTime_s;
lowerWaitTime_s = 0;
upperWaitTime_s = initialWaitTime_s;
bestCandidate = candidate;
bestCheckResult = checkResult;

%% Section 2: Check Successively Smaller Waits

% Every trial is a complete delayed motion and passes through the authoritative
% trajectory check. Bisection never treats a solver result or sampled obstacle
% query as final acceptance.

for refinementIndex = 1:options.MaximumWaitRefinementIterations
    if refinementIndex == 1
        trialWaitTime_s = lowerWaitTime_s;
    else
        trialWaitTime_s = 0.5 * (lowerWaitTime_s + upperWaitTime_s);
    end
    motionTimer = tic;
    [trialCandidate, ~] = ...
        obstacleAvoidance.planner.createDirectWaitMotion( ...
        seed, initialState, goalState, limits, options, ...
        trialWaitTime_s, directMotionDuration_s);
    motionElapsedTime_s = motionElapsedTime_s + toc(motionTimer);
    [trialCandidate, trialCheckResult, ~, stageTiming] = ...
        obstacleAvoidance.planner.checkCandidateMotion( ...
        trialCandidate, obstacles, initialState, goalState, limits, options, ...
        stageTiming, "The refined direct-wait kernel returned no trajectory.");
    diagnostics.RefinementCount = refinementIndex;
    if trialCheckResult.Passed
        bestCandidate = trialCandidate;
        bestCheckResult = trialCheckResult;
        upperWaitTime_s = trialWaitTime_s;
        if trialWaitTime_s == 0
            break;
        end
    else
        lowerWaitTime_s = trialWaitTime_s;
        diagnostics.InfeasibleLowerWaitTime_s = lowerWaitTime_s;
    end
end

%% Section 3: Return The Best Passing Wait

candidate = bestCandidate;
checkResult = bestCheckResult;
diagnostics.WaitTime_s = upperWaitTime_s;
diagnostics.FinalWaitTime_s = upperWaitTime_s;
diagnostics.ElapsedTime_s = ...
    diagnostics.ElapsedTime_s + motionElapsedTime_s;
candidate.SolverDiagnostics = diagnostics;
end
