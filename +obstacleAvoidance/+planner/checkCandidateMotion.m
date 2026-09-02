function [candidate, checkResult, elapsedTime_s, stageTiming] = ...
        checkCandidateMotion(candidate, obstacles, initialState, goalState, ...
        limits, options, stageTiming, emptyMessage)
%% Section 0: Header & Readme
% SYNTAX
%   [candidate, checkResult, elapsedTime_s, stageTiming] = ...
%       obstacleAvoidance.planner.checkCandidateMotion( ...
%       candidate, obstacles, initialState, goalState, limits, options, ...
%       stageTiming, emptyMessage)
%**************************************************************************
% PURPOSE
%   - Run and time the sole authoritative check for one candidate motion.
%   - Attach that unchanged check result and update planner stage timing.
%**************************************************************************
% INPUTS
%   - candidate (scalar motion struct)
%       Candidate motion with a sampled time history or documented empties.
%   - obstacles (prepared canonical obstacle struct array)
%       Complete protected histories used for authoritative validation.
%   - initialState, goalState (scalar structs)
%       Normalized endpoint states and goal-time policy.
%   - limits (scalar struct)
%       Workspace and physical motion limits.
%   - options (resolved scalar struct)
%       Planner and validation controls.
%   - stageTiming (scalar struct)
%       Accumulated planner stage durations.
%   - emptyMessage (scalar text)
%       Failure explanation used when no motion history exists.
%**************************************************************************
% OUTPUTS
%   - candidate (scalar motion struct)
%       Input motion with Validation set to checkResult.
%   - checkResult (scalar validation struct)
%       Result returned only by obstacleAvoidance.validateTrajectory.
%   - elapsedTime_s (nonnegative scalar)
%       Wall-clock duration of the full validation call.
%   - stageTiming (scalar struct)
%       Updated collision-checking and remaining validation durations.
%**************************************************************************
% UNITS
%   - elapsedTime_s and stage timing are seconds; motion units follow the
%     public planner contract.
%**************************************************************************

%% Section 1: Run The Authoritative Motion Check

% Empty kernel outputs cannot be approved, but they still need the same stable
% validation record. Nonempty motions are checked only by validateTrajectory;
% no solver-owned flag is promoted to obstacle-avoidance acceptance here.

checkResult = obstacleAvoidance.validateTrajectory();
elapsedTime_s = 0;
if isempty(candidate.time_s)
    if strlength(emptyMessage) > 0
        checkResult.Message = emptyMessage;
    end
else
    validationTimer = tic;
    checkResult = obstacleAvoidance.validateTrajectory( ...
        candidate, obstacles, initialState, goalState, limits, options);
    elapsedTime_s = toc(validationTimer);
    stageTiming.CollisionCheckingElapsedTime_s = ...
        stageTiming.CollisionCheckingElapsedTime_s + ...
        checkResult.CollisionCheckingElapsedTime_s;
    stageTiming.FinalValidationElapsedTime_s = ...
        stageTiming.FinalValidationElapsedTime_s + max( ...
        0, elapsedTime_s - checkResult.CollisionCheckingElapsedTime_s);
end
candidate.Validation = checkResult;
end
