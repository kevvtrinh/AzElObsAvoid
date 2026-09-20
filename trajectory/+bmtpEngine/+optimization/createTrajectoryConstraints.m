function [A, Aeq, beq, lb, ub, jerkMap] = createTrajectoryConstraints( ...
        segmentCount, degree, boundaryControls, limits, variableCount, ...
        segmentRatio, physicalTimes_s)
%% Section 0: Header & Readme
% SYNTAX
%   [A, Aeq, beq, lb, ub, jerkMap] = ...
%       bmtpEngine.optimization.createTrajectoryConstraints(segmentCount, degree, ...
%       boundaryControls, limits, variableCount, segmentRatio, physicalTimes_s)
%**************************************************************************
% PURPOSE
%   - Assemble workspace, endpoint, C3, and derivative constraints.
%**************************************************************************
% INPUTS
%   - segmentCount (positive integer scalar)
%       Number of polynomial spans.
%   - degree (positive integer scalar)
%       Polynomial degree.
%   - boundaryControls (numeric array)
%       Imposed endpoint controls.
%   - limits (scalar struct)
%       Workspace and per-axis derivative limits.
%   - variableCount (positive integer scalar)
%       Decision-vector size.
%   - segmentRatio (S-by-1 positive numeric vector)
%       Relative span durations.
%   - physicalTimes_s (empty or S-by-1 positive numeric vector)
%       Physical span durations for the fixed-clock jerk objective.
%**************************************************************************
% OUTPUTS
%   - A (sparse matrix)
%       Shared linear inequality coefficients.
%   - Aeq (sparse matrix)
%       Endpoint and continuity equality coefficients.
%   - beq (numeric column)
%       Equality right-hand side.
%   - lb (numeric column)
%       Decision-variable lower bounds.
%   - ub (numeric column)
%       Decision-variable upper bounds.
%   - jerkMap (sparse matrix)
%       Physical quadratic-jerk map for fixed-clock objectives.
%**************************************************************************
% UNITS
%   - Controls are coordinate units and physicalTimes_s is seconds.
%**************************************************************************

%% Section 1: Allocate Bounds And Given Endpoint Rows
segmentRatio       = segmentRatio(:);
controlCount        = segmentCount * (degree + 1) * 2;
powerIndex          = controlCount + (1:4);
baseInequalityCount = 4 * segmentCount * (3 * degree - 3);
equalityCount       = 13 + 8 * (segmentCount - 1);
A   = spalloc(baseInequalityCount, variableCount, 6 * baseInequalityCount);
Aeq = spalloc(equalityCount, variableCount, 8 * equalityCount);
beq = zeros(equalityCount, 1);
lb  = -Inf(variableCount, 1);
ub  = Inf(variableCount, 1);
workspaceDomain_units = [limits.xInterval_units; limits.yInterval_units];
lb(1:controlCount) = repmat(workspaceDomain_units(:, 1), segmentCount * (degree + 1), 1);
ub(1:controlCount) = repmat(workspaceDomain_units(:, 2), segmentCount * (degree + 1), 1);
lb(powerIndex)    = 0;
lb(powerIndex(2)) = eps;
equalityIndex = 0;
for axisIndex = 1:2
    firstColumnIndex = axisIndex;
    lastColumnIndex  = controlCount - 2 + axisIndex;
    equalityIndex = equalityIndex + 1;
    Aeq(equalityIndex, firstColumnIndex) = 1;
    beq(equalityIndex) = boundaryControls(1, 1, axisIndex);
    equalityIndex = equalityIndex + 1;
    Aeq(equalityIndex, lastColumnIndex) = 1;
    beq(equalityIndex) = boundaryControls(end, end, axisIndex);
    for endpointOrder = 1:2
        equalityIndex = equalityIndex + 1;
        Aeq(equalityIndex, [firstColumnIndex + 2 * endpointOrder, firstColumnIndex]) = [1, -1];
        beq(equalityIndex) = boundaryControls(1, endpointOrder + 1, axisIndex) - ...
            boundaryControls(1, 1, axisIndex);
        equalityIndex = equalityIndex + 1;
        Aeq(equalityIndex, [lastColumnIndex - 2 * endpointOrder, lastColumnIndex]) = [1, -1];
        beq(equalityIndex) = boundaryControls(end, degree - endpointOrder + 1, axisIndex) - ...
            boundaryControls(end, end, axisIndex);
    end
end

%% Section 2: Preserve Span/Derivative/Axis Continuity Row Order
differenceCoefficients = {1, [-1, 1], [1, -2, 1], [-1, 3, -3, 1]};
joinIndices            = (1:segmentCount - 1).';
% Multiply before dividing, as in the scalar equations, to preserve roundoff.
for derivativeOrder = 0:3
    if segmentCount == 1
        break;
    end
    derivativeCoefficients = differenceCoefficients{derivativeOrder + 1};
    rowScale               = max(segmentRatio(1:end - 1), segmentRatio(2:end)) .^ derivativeOrder;
    leftValues             = derivativeCoefficients .* segmentRatio(2:end) .^ derivativeOrder ./ rowScale;
    rightValues            = -derivativeCoefficients .* segmentRatio(1:end - 1) .^ derivativeOrder ./ rowScale;
    leftColumns            = (joinIndices - 1) * (degree + 1) + degree - derivativeOrder + ...
        (1:derivativeOrder + 1);
    rightColumns = joinIndices * (degree + 1) + (1:derivativeOrder + 1);
    rowIndices   = repmat((joinIndices - 1) * 4 + derivativeOrder + 1, 1, derivativeOrder + 1);
    rows = sparse([rowIndices(:); rowIndices(:)], [leftColumns(:); rightColumns(:)], ...
        [leftValues(:); rightValues(:)], 4 * (segmentCount - 1), segmentCount * (degree + 1));
    Aeq(13:end - 1, 1:controlCount) = Aeq(13:end - 1, 1:controlCount) + kron(rows, speye(2));
end
Aeq(end, powerIndex(1)) = 1;
beq(end) = 1;

%% Section 3: Batch The Same Signed Derivative Rows Across Spans
derivativeLimits = [limits.maxVelocity_units_s; limits.maxAcceleration_units_s2; ...
    limits.maxJerk_units_s3];
jerkMap = sparse(6 * segmentCount, variableCount);
for derivativeOrder = 1:3
    derivativeCount = degree - derivativeOrder + 1;
    scale           = factorial(degree) / factorial(degree - derivativeOrder);
    derivativeRows  = spdiags( ...
        repmat(scale * differenceCoefficients{derivativeOrder + 1}, derivativeCount, 1), ...
        0:derivativeOrder, derivativeCount, degree + 1);
    signedRows      = kron(kron(derivativeRows, speye(2)), [1; -1]);
    precedingRows   = 4 * ((derivativeOrder - 1) * (degree + 1) - ...
        (derivativeOrder - 1) * derivativeOrder / 2);
    targetRows = reshape(precedingRows + (1:size(signedRows, 1)).' + ...
        (0:segmentCount - 1) * (baseInequalityCount / segmentCount), [], 1);
    A(targetRows, 1:controlCount) = kron(speye(segmentCount), signedRows);
    axisLimits = repmat(derivativeLimits(derivativeOrder, :), derivativeCount, 1);
    timePowerCoefficients = -repelem(reshape(axisLimits.', [], 1), 2) * ...
        segmentRatio.' .^ derivativeOrder;
    A(targetRows, powerIndex(derivativeOrder + 1)) = timePowerCoefficients(:);
    if ~isempty(physicalTimes_s) && derivativeOrder == 3
        jerkMap(:, 1:controlCount) = kron( ...
            spdiags(1 ./ physicalTimes_s(:) .^ 3, 0, segmentCount, segmentCount), ...
            kron(derivativeRows, speye(2)));
    end
end
end
