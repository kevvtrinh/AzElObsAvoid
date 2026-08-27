function [inequality, equality, inequalityGradient, equalityGradient] = ...
        evaluateConstraints(decision, isFreeTime, fixedFinalTime, ...
        minimumFinalTime, maximumFinalTime, segmentCount, initialState, ...
        terminalState, limits, pathConstraints)
%% Section 0: Header & Readme
% SYNTAX
%   [inequality, equality] = hs3Internal.constraints.evaluateConstraints(decision, ...
%       isFreeTime, fixedFinalTime, minimumFinalTime, maximumFinalTime, ...
%       segmentCount, initialState, terminalState, limits, pathConstraints)
%   [inequality, equality, inequalityGradient, equalityGradient] = ...
%       hs3Internal.constraints.evaluateConstraints(decision, isFreeTime, fixedFinalTime, ...
%       minimumFinalTime, maximumFinalTime, segmentCount, initialState, ...
%       terminalState, limits, pathConstraints)
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
    terminalState, limits, pathConstraints);
if nargout < 3
    return;
end
dimensionCount = numel(initialState.position);
duration = finalTime - initialState.time;
[inequalityMatrix, equalityMatrix] = hs3Internal.constraints.createFixedConstraintMatrices( ...
    segmentCount, duration, dimensionCount, limits, pathConstraints);
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
    terminalState, limits, pathConstraints);
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
        terminalState, limits, pathConstraints)
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
polynomial = hs3Internal.polynomial.createTrajectoryPolynomial( ...
    controlJerk, initialState, finalTime, segmentCount);
inequality = continuousBoundConstraints(polynomial, limits);
if ~isempty(pathConstraints.Tau)
    inequality = [inequality; ...
        affinePathConstraints(polynomial, pathConstraints)];
end
terminal = polynomial.TerminalState;
equality = [ ...
    terminal.position - terminalState.position, ...
    terminal.velocity - terminalState.velocity, ...
    terminal.acceleration - terminalState.acceleration].';
end

function inequality = affinePathConstraints(polynomial, pathConstraints)
% Evaluate point or complete-interval coordinate half-space bounds.
% Projecting a vector polynomial onto a normal produces a scalar polynomial.
% Bounding that scalar's Bernstein values proves the requested half-space over
% the complete normalized interval, including points between output samples.
coefficientCount = size(polynomial.positionPower, 3);
[segmentIndex, hullMap] = hs3Internal.polynomial.createSubintervalBernsteinMap( ...
    pathConstraints.Tau, pathConstraints.TauEnd, ...
    polynomial.SegmentCount, coefficientCount);
isInterval = pathConstraints.TauEnd > pathConstraints.Tau;
rowCounts = 1 + (coefficientCount - 1) * isInterval;
inequality = zeros(sum(rowCounts), 1);
nextRow = 1;
for constraintIndex = 1:numel(pathConstraints.Tau)
    rowCount = rowCounts(constraintIndex);
    rows = nextRow:nextRow + rowCount - 1;
    nextRow = rows(end) + 1;
    segmentPower = reshape(polynomial.positionPower( ...
        segmentIndex(constraintIndex), :, :), [], coefficientCount);
    projectionHull = hullMap(:, :, constraintIndex) * ...
        (pathConstraints.Normal(constraintIndex, :) * segmentPower).';
    inequality(rows) = pathConstraints.LowerBound(constraintIndex) - ...
        projectionHull(1:rowCount);
end
end

function inequality = continuousBoundConstraints(polynomial, limits)
% Convert complete-segment derivative bounds into Bernstein violations.
% Bounds are appended in the same dimension/quantity/upper/lower order used by
% createFixedConstraintMatrices. Infinite bounds add no row because they place
% no physical restriction on that coordinate and derivative.
coefficientFields = [ ...
    "positionPower", "velocityPower", ...
    "accelerationPower", "jerkPower"];
lowerFields = [ ...
    "positionLower", "velocityLower", ...
    "accelerationLower", "jerkLower"];
upperFields = [ ...
    "positionUpper", "velocityUpper", ...
    "accelerationUpper", "jerkUpper"];
dimensionCount = size(polynomial.positionPower, 2);
inequality = zeros(0, 1);
for dimensionIndex = 1:dimensionCount
    for quantityIndex = 1:numel(coefficientFields)
        coefficientArray = polynomial.(coefficientFields(quantityIndex));
        bernstein = coordinateBernstein( ...
            coefficientArray, dimensionIndex);
        upperBounds = limits.(upperFields(quantityIndex));
        lowerBounds = limits.(lowerFields(quantityIndex));
        upperBound = upperBounds(dimensionIndex);
        lowerBound = lowerBounds(dimensionIndex);
        if isfinite(upperBound)
            inequality = [inequality; bernstein - upperBound]; %#ok<AGROW>
        end
        if isfinite(lowerBound)
            inequality = [inequality; lowerBound - bernstein]; %#ok<AGROW>
        end
    end
end
end

function bernstein = coordinateBernstein(coefficientArray, dimensionIndex)
% Convert every segment of one coordinate while preserving row order.
% Coefficient arrays are segment-by-coordinate-by-ascending-power. Conversion
% expects one polynomial per column, and linear indexing then stacks segments.
segmentCount = size(coefficientArray, 1);
coefficientCount = size(coefficientArray, 3);
powerMatrix = reshape(permute( ...
    coefficientArray(:, dimensionIndex, :), [3 1 2]), ...
    coefficientCount, segmentCount);
bernstein = hs3Internal.polynomial.convertPowerToBernstein(powerMatrix);
bernstein = bernstein(:);
end
