function solution = solveFreeTime( ...
        initialState, terminalState, limits, options, pathConstraints, ...
        minimumFinalTime, maximumFinalTime)
%% Section 0: Header & Readme
% SYNTAX
%   solution = hs3Internal.solver.solveFreeTime(initialState, terminalState, limits, ...
%       options, pathConstraints, minimumFinalTime, maximumFinalTime)
%**************************************************************************
% PURPOSE
%   - Solve the variable-final-time HS3 trajectory problem with fmincon.
%**************************************************************************
% INPUTS
%   - initialState, terminalState, limits, options (resolved scalar structs)
%   - pathConstraints (resolved scalar struct), optional affine path rows.
%   - minimumFinalTime, maximumFinalTime (finite scalar absolute times)
%**************************************************************************
% OUTPUTS
%   - solution (scalar struct)
%       Stable solver status, decision, objective, violations, and diagnostics.
%**************************************************************************
% UNITS
%   - Time and coordinate units are caller-defined and must be consistent.
%**************************************************************************

%% Section 1: Build A Deterministic Feasible-Horizon Start

% Earliest arrival adds final time after the coordinate-major jerk controls.
% Because duration powers multiply jerk, the resulting constraints are
% nonlinear. First solve the fixed problem at the largest allowed time: this
% gives fmincon a repeatable starting jerk profile. More time usually makes
% the physical limits easier to satisfy.
% If this starting solve is poor, the free-time solve also starts poorly.
% Inspect initialFixed diagnostics before changing the nonlinear solver.

solverTimer = tic;
segmentCount = options.SegmentCount;
dimensionCount = numel(initialState.position);
controlCount = 2 * segmentCount + 1;
jerkDecisionCount = dimensionCount * controlCount;
initialFixed = hs3Internal.solver.solveFixedTime( ...
    initialState, terminalState, limits, options, ...
    pathConstraints, maximumFinalTime);
decision0 = [initialFixed.Decision; maximumFinalTime];
lowerBound = [reshape(repmat( ...
    limits.jerkLower, controlCount, 1), [], 1); minimumFinalTime];
% Control bounds apply to each coordinate. The last two bounds limit absolute
% final time. minimumFinalTime is a velocity-based lower bound. The public
% function computes this bound. maximumFinalTime is the caller's search limit.
upperBound = [reshape(repmat( ...
    limits.jerkUpper, controlCount, 1), [], 1); maximumFinalTime];
typicalDecision = max(1, abs(decision0));
constraintFunction = @(decision) hs3Internal.constraints.evaluateConstraints( ...
    decision, true, maximumFinalTime, minimumFinalTime, ...
    maximumFinalTime, segmentCount, initialState, terminalState, ...
    limits, pathConstraints);
problem = struct( ...
    "Decision0", decision0, ...
    "LowerBound", lowerBound, ...
    "UpperBound", upperBound, ...
    "TypicalDecision", typicalDecision, ...
    "IsFreeTime", true, ...
    "ConstraintFunction", constraintFunction, ...
    "ObjectiveFunction", @arrivalTimeObjective, ...
    "ProgressFunction", @(decision, solverValues, state) ...
    progress(decision, solverValues, state, solverTimer, ...
    options.MaximumSolveTime, options.Verbose), ...
    "SubproblemAlgorithm", "cg", ...
    "MaximumFinalTime", maximumFinalTime, ...
    "SolverTimer", solverTimer);

%% Section 2: Minimize Arrival Time And Recover Feasibility

% The primary objective is only final time. optimize may run a second,
% feasibility solve near the same final time if the first stage has residual
% violations. This second solve repairs numerical feasibility.

core = hs3Internal.solver.optimize(problem, options);

%% Section 3: Assemble The Solver Record

% Report a feasible point at the maximum horizon separately. This point
% proves motion at the boundary but does not establish an earlier arrival
% inside the requested interval, so the public Success flag remains false.

finalTime = core.Decision(end);
if strlength(core.StageOneErrorIdentifier) > 0
    terminationReason = "solverError";
    message = "Free-time HS3 solve failed: " + ...
        core.StageOneErrorMessage;
elseif core.ArrivalAtHorizon
    terminationReason = "arrivalAtHorizon";
    message = "HS3 returned a feasible motion pinned to the time horizon.";
elseif core.OptimizerFeasible
    terminationReason = "optimizerFeasible";
    message = "Free-time HS3 returned a constraint-feasible local solution.";
elseif core.TerminationReason == "solverTimeLimit"
    terminationReason = "solverTimeLimit";
    message = "HS3 reached its solve-time limit before feasibility.";
else
    terminationReason = "optimizerInfeasible";
    message = "Free-time HS3 did not satisfy all constraints.";
end
solution = struct( ...
    "Success", core.Success, ...
    "Message", message, ...
    "TerminationReason", terminationReason, ...
    "Decision", core.Decision(1:jerkDecisionCount), ...
    "FinalTime", finalTime, ...
    "Objective", core.Decision(end), ...
    "ExitFlag", core.StageOneExitFlag, ...
    "Output", core.StageOneOutput, ...
    "MaximumInequalityViolation", core.MaximumInequalityViolation, ...
    "MaximumEqualityViolation", core.MaximumEqualityViolation, ...
    "MaximumConstraintViolation", core.MaximumConstraintViolation, ...
    "ErrorIdentifier", core.StageOneErrorIdentifier, ...
    "ErrorMessage", core.StageOneErrorMessage, ...
    "ElapsedTime", core.ElapsedTime, ...
    "StageOneObjective", core.StageOneObjective, ...
    "StageTwoObjective", core.StageTwoObjective, ...
    "StageTwoExitFlag", core.StageTwoExitFlag, ...
    "StageTwoOutput", core.StageTwoOutput, ...
    "StageTwoErrorIdentifier", core.StageTwoErrorIdentifier, ...
    "StageTwoErrorMessage", core.StageTwoErrorMessage);
end

%% Section 5: Local Functions

function [value, gradient] = arrivalTimeObjective(decision)
% Return the appended final-time decision and its exact gradient.
% All jerk components have zero direct objective derivative. They affect the
% result only through the feasibility constraints.
value = decision(end);
gradient = zeros(size(decision));
gradient(end) = 1;
end

function stop = progress( ...
        ~, solverValues, state, solverTimer, maximumTime, verbose)
% Enforce the solver time budget and optionally report iteration state.
% A true value stops fmincon at the next callback. Nonverbose use prints no
% progress information. Verbose output does not change solver decisions.
stop = toc(solverTimer) >= maximumTime;
if verbose && any(string(state) == ["iter", "done"])
    iteration = NaN;
    objective = NaN;
    if isfield(solverValues, "iteration")
        iteration = solverValues.iteration;
    end
    if isfield(solverValues, "fval")
        objective = solverValues.fval;
    end
    fprintf("[HS3] free-time state=%s, iteration=%g, objective=%.9g.\n", ...
        state, iteration, objective);
end
end
