function [inequalityMatrix, equalityMatrix, equalityValues, lowerBounds, upperBounds, ...
    jerkControlMap] = createTrajectoryConstraints(segmentCount, degree, ...
    endpointControlPoint_units, limits, decisionVariableCount, ...
    segmentTimeRatios, fixedSegmentTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   [inequalityMatrix, equalityMatrix, equalityValues, lowerBounds, upperBounds, ...
%       jerkControlMap] = bmtpEngine.optimization.createTrajectoryConstraints( ...
%       segmentCount, degree, endpointControlPoint_units, limits, ...
%       decisionVariableCount, segmentTimeRatios, fixedSegmentTime_s)
%**************************************************************************
% PURPOSE
%   - Keep control points within the workspace, set endpoint states, match
%     position through jerk at segment joins, and enforce motion-rate limits.
%     Return the linear equations and bounds used by the trajectory solver.
%**************************************************************************
% INPUTS
%   - segmentCount (positive integer scalar)
%       Number of polynomial motion segments.
%   - degree (positive integer scalar)
%       Polynomial degree.
%   - endpointControlPoint_units (S-by-(D+1)-by-2 numeric array)
%       Controls with the requested initial and goal states already imposed.
%   - limits (scalar struct)
%       Workspace bounds and per-axis speed, acceleration, and jerk limits.
%   - decisionVariableCount (positive integer scalar)
%       Number of unknown values in the solver vector.
%   - segmentTimeRatios (S-by-1 positive numeric vector)
%       Segment durations relative to a shared time scale T; for example,
%       ratios [1 2] give durations [T 2 x T].
%   - fixedSegmentTime_s (empty or S-by-1 positive numeric vector)
%       Known segment durations when the objective uses physical jerk values.
%**************************************************************************
% OUTPUTS
%   - inequalityMatrix (sparse matrix)
%       Motion-limit rows satisfying inequalityMatrix x solverValues <= 0.
%   - equalityMatrix, equalityValues (sparse matrix and numeric column)
%       Endpoint and join equations: equalityMatrix x solverValues = equalityValues.
%   - lowerBounds, upperBounds (numeric columns)
%       Lower and upper limits on each solver variable.
%   - jerkControlMap (sparse matrix)
%       Maps solver values to jerk controls for the fixed-duration objective;
%       left empty of nonzero entries when no fixed durations are supplied.
%**************************************************************************
% UNITS
%   - Controls are coordinate units, fixedSegmentTime_s is seconds, and
%     segmentTimeRatios is dimensionless.
%**************************************************************************

%% Section 1: Set Workspace Bounds And Endpoint Controls

% Store x/y for every control, followed by four variables for [1 T T^2 T^3].
% Each motion limit needs positive/negative rows for both axes.
% Endpoint rows = 2 endpoints x 2 axes x 3 states (position through acceleration).
% Join rows = 2 axes x 4 states (position through jerk). One row fixes T^0 = 1.
segmentTimeRatios    = segmentTimeRatios(:);
controlVariableCount = segmentCount * (degree + 1) * 2;
timePowerIndices     = controlVariableCount + (1:4);
motionLimitRowCount  = 4 * segmentCount * (3 * degree - 3);
equalityRowCount     = 13 + 8 * (segmentCount - 1);
inequalityMatrix     = spalloc(motionLimitRowCount, decisionVariableCount, 6 * motionLimitRowCount);
equalityMatrix       = spalloc(equalityRowCount, decisionVariableCount, 8 * equalityRowCount);
equalityValues       = zeros(equalityRowCount, 1);
lowerBounds          = -Inf(decisionVariableCount, 1);
upperBounds          = Inf(decisionVariableCount, 1);

% A Bezier curve stays within its controls' coordinate ranges, so these
% control bounds also keep the curve within the rectangular workspace.
workspaceIntervals_units = [limits.xInterval_units; limits.yInterval_units];
lowerBounds(1:controlVariableCount) = ...
    repmat(workspaceIntervals_units(:, 1), segmentCount * (degree + 1), 1);
upperBounds(1:controlVariableCount) = ...
    repmat(workspaceIntervals_units(:, 2), segmentCount * (degree + 1), 1);
lowerBounds(timePowerIndices)    = 0;
lowerBounds(timePowerIndices(2)) = eps;

% Set endpoint positions, then differences to the next two controls. Those
% supplied control differences encode endpoint velocity and acceleration.
equalityRowIndex = 0;
for axisIndex = 1:2
    initialCoordinateIndex = axisIndex;
    goalCoordinateIndex    = controlVariableCount - 2 + axisIndex;
    equalityRowIndex       = equalityRowIndex + 1;
    equalityMatrix(equalityRowIndex, initialCoordinateIndex) = 1;
    equalityValues(equalityRowIndex) = endpointControlPoint_units(1, 1, axisIndex);
    equalityRowIndex = equalityRowIndex + 1;
    equalityMatrix(equalityRowIndex, goalCoordinateIndex) = 1;
    equalityValues(equalityRowIndex) = endpointControlPoint_units(end, end, axisIndex);
    for endpointControlOffset = 1:2
        equalityRowIndex = equalityRowIndex + 1;
        equalityMatrix(equalityRowIndex, ...
            [initialCoordinateIndex + 2 * endpointControlOffset, initialCoordinateIndex]) = [1, -1];
        equalityValues(equalityRowIndex) = endpointControlPoint_units(1, endpointControlOffset + 1, axisIndex) - ...
            endpointControlPoint_units(1, 1, axisIndex);
        equalityRowIndex = equalityRowIndex + 1;
        equalityMatrix(equalityRowIndex, ...
            [goalCoordinateIndex - 2 * endpointControlOffset, goalCoordinateIndex]) = [1, -1];
        equalityValues(equalityRowIndex) = endpointControlPoint_units(end, degree - endpointControlOffset + 1, axisIndex) - ...
            endpointControlPoint_units(end, end, axisIndex);
    end
end

%% Section 2: Match Position, Velocity, Acceleration, And Jerk At Joins

controlDifferenceWeights = {1, [-1, 1], [1, -2, 1], [-1, 3, -3, 1]};
joinSegmentIndices       = (1:segmentCount - 1).';
% For derivative order n, match left difference / left ratio^n with
% right difference / right ratio^n. Cross-multiply, then scale each row
% by the larger ratio^n to keep the equation coefficients manageable.
for derivativeOrder = 0:3
    if segmentCount == 1
        break;
    end
    derivativeWeights  = controlDifferenceWeights{derivativeOrder + 1};
    joinRowScale       = max(segmentTimeRatios(1:end - 1), segmentTimeRatios(2:end)) .^ derivativeOrder;
    leftJoinWeights    = derivativeWeights .* segmentTimeRatios(2:end) .^ derivativeOrder ./ joinRowScale;
    rightJoinWeights   = -derivativeWeights .* segmentTimeRatios(1:end - 1) .^ derivativeOrder ./ joinRowScale;
    leftControlColumns = (joinSegmentIndices - 1) * (degree + 1) + degree - derivativeOrder + ...
        (1:derivativeOrder + 1);
    rightControlColumns = joinSegmentIndices * (degree + 1) + (1:derivativeOrder + 1);
    joinRowIndices      = repmat((joinSegmentIndices - 1) * 4 + derivativeOrder + 1, 1, derivativeOrder + 1);
    singleAxisJoinRows   = sparse([joinRowIndices(:); joinRowIndices(:)], ...
        [leftControlColumns(:); rightControlColumns(:)], ...
        [leftJoinWeights(:); rightJoinWeights(:)], 4 * (segmentCount - 1), segmentCount * (degree + 1));
    % Duplicate the single-axis equations for interleaved x/y variables.
    equalityMatrix(13:end - 1, 1:controlVariableCount) = ...
        equalityMatrix(13:end - 1, 1:controlVariableCount) + kron(singleAxisJoinRows, speye(2));
end
% The first time-power variable represents T^0 = 1.
equalityMatrix(end, timePowerIndices(1)) = 1;
equalityValues(end) = 1;

%% Section 3: Enforce Both Signs Of Each Motion-Rate Limit

% A derivative-control value divided by duration^n must lie between
% -limit and +limit. Move limit x ratio^n x T^n to the left of each
% inequality, giving the zero right-hand side documented above.
motionRateLimits = [limits.maxVelocity_units_s; limits.maxAcceleration_units_s2; ...
    limits.maxJerk_units_s3];
jerkControlMap = sparse(6 * segmentCount, decisionVariableCount);
for derivativeOrder = 1:3
    derivativeControlCount   = degree - derivativeOrder + 1;
    derivativeFactor         = factorial(degree) / factorial(degree - derivativeOrder);
    singleAxisDerivativeRows = spdiags( ...
        repmat(derivativeFactor * controlDifferenceWeights{derivativeOrder + 1}, derivativeControlCount, 1), ...
        0:derivativeOrder, derivativeControlCount, degree + 1);
    % For every control difference, create +x, -x, +y, -y limit rows.
    signedAxisRows = kron(kron(singleAxisDerivativeRows, speye(2)), [1; -1]);
    precedingDerivativeRowCount = 4 * ((derivativeOrder - 1) * (degree + 1) - ...
        (derivativeOrder - 1) * derivativeOrder / 2);
    motionLimitRowIndices = reshape(precedingDerivativeRowCount + (1:size(signedAxisRows, 1)).' + ...
        (0:segmentCount - 1) * (motionLimitRowCount / segmentCount), [], 1);
    inequalityMatrix(motionLimitRowIndices, 1:controlVariableCount) = kron(speye(segmentCount), signedAxisRows);
    axisRateLimits   = repmat(motionRateLimits(derivativeOrder, :), derivativeControlCount, 1);
    timePowerWeights = -repelem(reshape(axisRateLimits.', [], 1), 2) * ...
        segmentTimeRatios.' .^ derivativeOrder;
    inequalityMatrix(motionLimitRowIndices, timePowerIndices(derivativeOrder + 1)) = timePowerWeights(:);
    % With known durations, divide jerk controls by duration^3 to supply
    % physical jerk values to the smoothness objective.
    if ~isempty(fixedSegmentTime_s) && derivativeOrder == 3
        jerkControlMap(:, 1:controlVariableCount) = kron( ...
            spdiags(1 ./ fixedSegmentTime_s(:) .^ 3, 0, segmentCount, segmentCount), ...
            kron(singleAxisDerivativeRows, speye(2)));
    end
end
end
