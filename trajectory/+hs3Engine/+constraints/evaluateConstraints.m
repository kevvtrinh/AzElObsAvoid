function [inequality, equality, inequalityGradient, equalityGradient] = ...
        evaluateConstraints(decision, isFreeTime, fixedFinalTime, ...
        minimumFinalTime, maximumFinalTime, segmentCount, initialState, ...
        terminalState, limits, pathConstraints, segmentBreakTau)
%% Section 0: Header & Readme
% SYNTAX
%   [inequality, equality] = hs3Engine.constraints.evaluateConstraints(decision, ...
%       isFreeTime, fixedFinalTime, minimumFinalTime, maximumFinalTime, ...
%       segmentCount, initialState, terminalState, limits, pathConstraints)
%   [inequality, equality, inequalityGradient, equalityGradient] = ...
%       hs3Engine.constraints.evaluateConstraints(decision, isFreeTime, fixedFinalTime, ...
%       minimumFinalTime, maximumFinalTime, segmentCount, initialState, ...
%       terminalState, limits, pathConstraints)
%   [inequality, equality] = hs3Engine.constraints.evaluateConstraints( ...
%       decision, isFreeTime, fixedFinalTime, minimumFinalTime, ...
%       maximumFinalTime, segmentCount, initialState, terminalState, limits, ...
%       pathConstraints, segmentBreakTau)
%**************************************************************************
% PURPOSE
%   - Evaluate dimension-neutral continuous kinematic and affine path bounds.
%**************************************************************************
% INPUTS
%   - decision (numeric column), coordinate-major jerk and optional time.
%   - isFreeTime (logical scalar), selects the final-time decision.
%   - fixedFinalTime, minimumFinalTime, maximumFinalTime (finite scalars)
%   - segmentCount (positive integer scalar), equal-duration segments.
%   - initialState, terminalState, limits (resolved scalar structs)
%   - pathConstraints (scalar struct)
%       Tau/TauEnd are M-by-1, Normal is M-by-D, and LowerBound is M-by-1.
%       Nonzero intervals constrain their complete Bernstein hull.
%   - segmentBreakTau ((N+1)-element vector, optional)
%       Strictly increasing normalized boundaries; [] selects a uniform mesh.
%**************************************************************************
% OUTPUTS
%   - inequality, equality (numeric columns), feasible at c<=0 and ceq=0.
%   - inequalityGradient, equalityGradient (numeric matrices)
%       fmincon orientation: decision count by constraint count.
%**************************************************************************
% UNITS
%   - Constraint rows retain caller-defined consistent coordinate units.
%**************************************************************************

%% Section 1: Evaluate Values And Exact Jerk Columns

% Constraint convention follows fmincon: every inequality is feasible when
% c <= 0, while terminal-state equalities must be exactly zero. The jerk
% columns are analytic because duration is held fixed while differentiating
% them. This avoids finite-difference noise in the large control portion.
% When the optimizer reports a large residual, compare the largest entries in
% inequality and equality. Inequality rows point to continuous bounds or path
% planes. Equality rows point to terminal position, velocity, or acceleration.

if nargin < 11
    segmentBreakTau = [];
end
if isFreeTime && ~isfinite(decision(end))
    % A solver can request a nonfinite time during a line search. Replace this
    % value with a finite value in the permitted range. The decision bounds
    % prevent the solver from accepting the replacement as a solution.
    if isnan(decision(end))
        decision(end) = mean([minimumFinalTime, maximumFinalTime]);
    else
        timeBounds = [minimumFinalTime, maximumFinalTime];
        decision(end) = timeBounds(1 + (decision(end) > 0));
    end
end
[inequality, equality, finalTime] = constraintValues( ...
    decision, isFreeTime, fixedFinalTime, segmentCount, initialState, ...
    terminalState, limits, pathConstraints, segmentBreakTau);
if nargout < 3
    return;
end
dimensionCount = numel(initialState.position);
duration = finalTime - initialState.time;
[inequalityMatrix, equalityMatrix] = hs3Engine.constraints.createFixedConstraintMatrices( ...
    segmentCount, duration, dimensionCount, limits, pathConstraints, ...
    segmentBreakTau);
inequalityGradient = inequalityMatrix.';
equalityGradient = equalityMatrix.';
if ~isFreeTime
    return;
end

%% Section 2: Append One Safeguarded Final-Time Column

% Duration appears through powers up to five, so the time derivative is more
% complex than the affine jerk derivatives. Use a one-sided finite difference.
% Keep the difference point inside the permitted time range. eps^(1/3) balances
% truncation error and round-off error for a first derivative.
% If free-time iterations are unstable, inspect this time column and the time
% bounds before changing solver tolerances.

scale = max(1, abs(finalTime));
baseStep = eps^(1 / 3) * scale;
forwardRoom = maximumFinalTime - finalTime;
backwardRoom = finalTime - minimumFinalTime;
if forwardRoom >= baseStep
    direction = 1;
    differenceStep = baseStep;
elseif backwardRoom >= baseStep
    direction = -1;
    differenceStep = baseStep;
elseif forwardRoom >= backwardRoom && forwardRoom > 0
    direction = 1;
    differenceStep = 0.5 * forwardRoom;
elseif backwardRoom > 0
    direction = -1;
    differenceStep = 0.5 * backwardRoom;
else
    error("evaluateConstraints:NoTimeDifferenceRoom", ...
        "No nonzero final-time perturbation fits inside the time bounds.");
end
trialDecision = decision;
trialDecision(end) = finalTime + direction * differenceStep;
[trialInequality, trialEquality] = constraintValues( ...
    trialDecision, true, fixedFinalTime, segmentCount, initialState, ...
    terminalState, limits, pathConstraints, segmentBreakTau);
if direction > 0
    timeInequalityGradient = ...
        (trialInequality - inequality) / differenceStep;
    timeEqualityGradient = (trialEquality - equality) / differenceStep;
else
    timeInequalityGradient = ...
        (inequality - trialInequality) / differenceStep;
    timeEqualityGradient = (equality - trialEquality) / differenceStep;
end
inequalityGradient(end + 1, :) = timeInequalityGradient.';
equalityGradient(end + 1, :) = timeEqualityGradient.';
end

%% Section 3: Local Functions

function [inequality, equality, finalTime] = constraintValues( ...
        decision, isFreeTime, fixedFinalTime, segmentCount, initialState, ...
        terminalState, limits, pathConstraints, segmentBreakTau)
% Reconstruct one decision and evaluate raw constraint values.
% The final-time decision is appended after all coordinate-major jerk values.
% Rebuilding the polynomial here keeps every constraint tied to the same exact
% integration equations used later for returned trajectory reconstruction.
dimensionCount = numel(initialState.position);
controlCount = 2 * segmentCount + 1;
jerkValueCount = dimensionCount * controlCount;
controlJerk = reshape( ...
    decision(1:jerkValueCount), controlCount, dimensionCount);
finalTime = fixedFinalTime;
if isFreeTime
    finalTime = decision(end);
end
polynomial = hs3Engine.polynomial.createTrajectoryPolynomial( ...
    controlJerk, initialState, finalTime, segmentCount, segmentBreakTau);
[inequality, equality] = ...
    hs3Engine.constraints.evaluatePolynomialConstraints( ...
    polynomial, terminalState, limits, pathConstraints);
end
