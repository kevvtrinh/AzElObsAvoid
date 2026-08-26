function solution = solveFreeTime( ...
        initialState, terminalState, limits, options, pathConstraints, ...
        minimumFinalTime, maximumFinalTime)
%% Section 0: Header & Readme
% SYNTAX
%   solution = hs3.solveFreeTime(initialState, terminalState, limits, ...
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

solverTimer = tic;
segmentCount = options.SegmentCount;
dimensionCount = numel(initialState.position);
controlCount = 2 * segmentCount + 1;
jerkDecisionCount = dimensionCount * controlCount;
initialFixed = hs3.solveFixedTime( ...
    initialState, terminalState, limits, options, ...
    pathConstraints, maximumFinalTime);
decision0 = [initialFixed.Decision; maximumFinalTime];
lowerBound = [reshape(repmat( ...
    limits.jerkLower, controlCount, 1), [], 1); minimumFinalTime];
upperBound = [reshape(repmat( ...
    limits.jerkUpper, controlCount, 1), [], 1); maximumFinalTime];
typicalDecision = max(1, abs(decision0));
constraintFunction = @(decision) hs3.evaluateConstraints( ...
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

core = hs3.optimize(problem, options);

%% Section 3: Assemble The Solver Record

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
%% Section 0: Header & Readme
% SYNTAX
%   [value, gradient] = arrivalTimeObjective(decision)
%**************************************************************************
% PURPOSE
%   - Return the appended final-time decision and its exact gradient.
%**************************************************************************
% INPUTS
%   - decision (numeric column), jerk controls followed by final time.
%**************************************************************************
% OUTPUTS
%   - value (scalar), final time.
%   - gradient (numeric column), zero except for its last value.
%**************************************************************************
% UNITS
%   - Value uses caller-defined time units.
%**************************************************************************
value = decision(end);
gradient = zeros(size(decision));
gradient(end) = 1;
end

function stop = progress( ...
        ~, solverValues, state, solverTimer, maximumTime, verbose)
%% Section 0: Header & Readme
% SYNTAX
%   stop = progress(decision, solverValues, state, solverTimer, ...
%       maximumTime, verbose)
%**************************************************************************
% PURPOSE
%   - Enforce the solver time budget and optionally report iteration state.
%**************************************************************************
% INPUTS
%   - decision (unused numeric column), current solver decision.
%   - solverValues (scalar struct), fmincon iteration values.
%   - state (text scalar), fmincon callback state.
%   - solverTimer (timer handle), solve start time.
%   - maximumTime (positive scalar), solve-time budget.
%   - verbose (logical scalar), controls progress printing.
%**************************************************************************
% OUTPUTS
%   - stop (logical scalar), true after the time budget is reached.
%**************************************************************************
% UNITS
%   - maximumTime and elapsed time use seconds.
%**************************************************************************
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
