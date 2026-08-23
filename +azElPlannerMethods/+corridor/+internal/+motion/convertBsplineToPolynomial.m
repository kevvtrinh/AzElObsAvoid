function polynomial = convertBsplineToPolynomial(controlPoint_deg, degree, initialTime_s, spanDuration_s)
%% Section 0: Header & Readme
% SYNTAX
%   polynomial = azElPlannerMethods.corridor.internal.motion.convertBsplineToPolynomial( ...
%       controlPoint_deg, degree, initialTime_s, spanDuration_s)
%**************************************************************************
% PURPOSE
%   - Convert an open B-spline into the exact piecewise-power schema shared by
%     motion construction and validation. This is the representation boundary:
%     downstream code no longer needs B-spline basis evaluation.
%**************************************************************************
% INPUTS
%   - controlPoint_deg (N-by-D array), B-spline control positions.
%   - degree (nonnegative integer scalar), spline degree.
%   - initialTime_s (finite scalar), first segment start time.
%   - spanDuration_s (positive vector), physical knot-span durations.
%**************************************************************************
% OUTPUTS
%   - polynomial (scalar struct), exact position-through-jerk coefficients.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s powers.
%**************************************************************************

%% Section 1: Convert Every Knot Span

% Evaluate degree+1 basis samples on each span, solve the small Vandermonde
% system for ascending-power coefficients, then differentiate analytically.
% The continuity projection removes accumulated floating-point mismatch while
% preserving the evaluated endpoints.
spanDuration_s = double(spanDuration_s(:));
spanCount = numel(spanDuration_s);
coordinateCount = size(controlPoint_deg, 2);
segmentStartTime_s = initialTime_s + [0; cumsum(spanDuration_s(1:end - 1))];
knots_s = knotTime( initialTime_s, spanDuration_s, degree, size(controlPoint_deg, 1));
localSamples = linspace(0, 1, degree + 1).';
vandermonde = localSamples.^(0:degree);
positionPower_deg = zeros(spanCount, coordinateCount, degree + 1);
velocityPower_deg_s = zeros(spanCount, coordinateCount, degree);
accelerationPower_deg_s2 = zeros( spanCount, coordinateCount, degree - 1);
jerkPower_deg_s3 = zeros(spanCount, coordinateCount, degree - 2);

% Recover one local power polynomial for each nonzero knot span.
for segmentIndex = 1:spanCount
    sampleTime_s = segmentStartTime_s(segmentIndex) + localSamples * spanDuration_s(segmentIndex);
    basisValues = zeros(degree + 1, size(controlPoint_deg, 1));

    % Evaluate every B-spline basis at enough local points to determine the degree-five polynomial.
    for sampleIndex = 1:degree + 1
        basisValues(sampleIndex, :) = bsplineBasis( ...
            knots_s, degree, sampleTime_s(sampleIndex), size(controlPoint_deg, 1));
    end
    positionPower = vandermonde \ (basisValues * controlPoint_deg);
    positionPower = positionPower.';
    velocityPower = positionPower(:, 2:end) .* (1:degree) / spanDuration_s(segmentIndex);
    accelerationPower = velocityPower(:, 2:end) .* (1:degree - 1) / spanDuration_s(segmentIndex);
    jerkPower = accelerationPower(:, 2:end) .* (1:degree - 2) / spanDuration_s(segmentIndex);
    positionPower_deg(segmentIndex, :, :) = reshape(positionPower, 1, coordinateCount, degree + 1);
    velocityPower_deg_s(segmentIndex, :, :) = reshape(velocityPower, 1, coordinateCount, degree);
    accelerationPower_deg_s2(segmentIndex, :, :) = reshape(accelerationPower, 1, coordinateCount, degree - 1);
    jerkPower_deg_s3(segmentIndex, :, :) = reshape(jerkPower, 1, coordinateCount, degree - 2);
end
positionPower_deg = projectPolynomialContinuity( positionPower_deg, spanDuration_s, degree);

% Differentiate the continuity-corrected position coefficients span by span.
for segmentIndex = 1:spanCount
    positionPower = reshape( positionPower_deg(segmentIndex, :, :), coordinateCount, []);
    velocityPower = positionPower(:, 2:end) .* (1:degree) / spanDuration_s(segmentIndex);
    accelerationPower = velocityPower(:, 2:end) .* (1:degree - 1) / spanDuration_s(segmentIndex);
    jerkPower = accelerationPower(:, 2:end) .* (1:degree - 2) / spanDuration_s(segmentIndex);
    velocityPower_deg_s(segmentIndex, :, :) = reshape(velocityPower, 1, coordinateCount, degree);
    accelerationPower_deg_s2(segmentIndex, :, :) = reshape(accelerationPower, 1, coordinateCount, degree - 1);
    jerkPower_deg_s3(segmentIndex, :, :) = reshape(jerkPower, 1, coordinateCount, degree - 2);
end
terminalPosition_deg = sum(reshape( positionPower_deg(end, :, :), coordinateCount, []), 2).';
terminalVelocity_deg_s = sum(reshape( velocityPower_deg_s(end, :, :), coordinateCount, []), 2).';
terminalAcceleration_deg_s2 = sum(reshape( accelerationPower_deg_s2(end, :, :), coordinateCount, []), 2).';
polynomial = struct( ...
    "SegmentCount", spanCount, ...
    "SegmentStartTime_s", segmentStartTime_s, ...
    "SegmentDuration_s", spanDuration_s, ...
    "FinalTime_s", initialTime_s + sum(spanDuration_s), ...
    "positionPower_deg", positionPower_deg, ...
    "velocityPower_deg_s", velocityPower_deg_s, ...
    "accelerationPower_deg_s2", accelerationPower_deg_s2, ...
    "jerkPower_deg_s3", jerkPower_deg_s3, ...
    "TerminalState", struct( ...
    "position_deg", terminalPosition_deg, ...
    "velocity_deg_s", terminalVelocity_deg_s, "acceleration_deg_s2", terminalAcceleration_deg_s2));
end


function positionPower = projectPolynomialContinuity(positionPower, spanDuration_s, degree)
% Apply the minimum coefficient correction satisfying C3 and endpoints.
spanCount = size(positionPower, 1);
coefficientCount = degree + 1;
constraintCount = 4 * (spanCount - 1) + 6;
constraintMatrix = zeros(constraintCount, spanCount * coefficientCount);
rowIndex = 0;

% Add continuity equations at every boundary between neighboring spans.
for segmentIndex = 1:spanCount - 1

    % Constrain position through jerk so the reconstructed motion is C3.
    for derivativeOrder = 0:3
        rowIndex = rowIndex + 1;
        leftIndex = (segmentIndex - 1) * coefficientCount + (derivativeOrder + 1:coefficientCount);
        powerIndex = derivativeOrder:degree;
        leftFactor = factorial(powerIndex) ./ ...
            factorial(powerIndex - derivativeOrder) / spanDuration_s(segmentIndex) ^ derivativeOrder;
        constraintMatrix(rowIndex, leftIndex) = leftFactor;
        rightIndex = segmentIndex * coefficientCount + derivativeOrder + 1;
        constraintMatrix(rowIndex, rightIndex) = -factorial(derivativeOrder) / ...
            spanDuration_s(segmentIndex + 1) ^ derivativeOrder;
    end
end

% Preserve initial position, velocity, and acceleration during the correction.
for derivativeOrder = 0:2
    rowIndex = rowIndex + 1;
    constraintMatrix(rowIndex, derivativeOrder + 1) = factorial(derivativeOrder) / spanDuration_s(1) ^ derivativeOrder;
end

% Preserve the same three quantities at the final trajectory endpoint.
for derivativeOrder = 0:2
    rowIndex = rowIndex + 1;
    finalIndex = (spanCount - 1) * coefficientCount + (derivativeOrder + 1:coefficientCount);
    powerIndex = derivativeOrder:degree;
    constraintMatrix(rowIndex, finalIndex) = factorial(powerIndex) ./ ...
        factorial(powerIndex - derivativeOrder) / spanDuration_s(end) ^ derivativeOrder;
end

% Apply the minimum-norm continuity correction independently to each coordinate.
for coordinateIndex = 1:size(positionPower, 2)
    coefficient = reshape(permute( positionPower(:, coordinateIndex, :), [3 1 2]), [], 1);
    endpointValue = constraintMatrix(end - 5:end, :) * coefficient;
    constraintValue = [zeros(constraintCount - 6, 1); endpointValue];
    coefficient = coefficient + lsqminnorm( constraintMatrix, constraintValue - constraintMatrix * coefficient);
    positionPower(:, coordinateIndex, :) = permute(reshape( coefficient, coefficientCount, spanCount), [2 3 1]);
end
end

function knots_s = knotTime(initialTime_s, spanDuration_s, degree, controlCount)
% Build one open knot vector with uniform interior multiplicity.
spanBoundary_s = initialTime_s + [0; cumsum(spanDuration_s(:))];
interiorBoundaryCount = numel(spanDuration_s) - 1;
if interiorBoundaryCount == 0
    interiorMultiplicity = 0;
else
    interiorMultiplicity = (controlCount - degree - 1) / interiorBoundaryCount;
end
if interiorMultiplicity ~= floor(interiorMultiplicity) || interiorMultiplicity < 0 || interiorMultiplicity > degree
    error("convertBsplineToPolynomial:InconsistentControlCount", ...
        "Control count does not define a valid uniform knot multiplicity.");
end
knots_s = [ ...
    repmat(spanBoundary_s(1), 1, degree + 1), ...
    repelem(spanBoundary_s(2:end - 1).', interiorMultiplicity), repmat(spanBoundary_s(end), 1, degree + 1)];
end

function basis = bsplineBasis(knots_s, degree, time_s, controlCount)
% Evaluate all nonzero-safe Cox-de Boor basis values at one time.
endpointTolerance_s = 32 * eps(max(1, abs(knots_s(end))));
if time_s >= knots_s(end) - endpointTolerance_s
    basis = zeros(1, controlCount);
    basis(end) = 1;
    return;
end
baseCount = numel(knots_s) - 1;
basis = zeros(1, baseCount);

% Initialize the degree-zero indicator basis over every knot interval.
for basisIndex = 1:baseCount
    basis(basisIndex) = time_s >= knots_s(basisIndex) && time_s < knots_s(basisIndex + 1);
end

% Raise the basis degree recursively using the Cox-de Boor relation.
for recursionDegree = 1:degree
    nextCount = baseCount - recursionDegree;
    nextBasis = zeros(1, nextCount);

    % Combine the left and right contributions for every surviving basis function.
    for basisIndex = 1:nextCount
        leftDenominator_s = knots_s(basisIndex + recursionDegree) - knots_s(basisIndex);
        rightDenominator_s = knots_s(basisIndex + recursionDegree + 1) - knots_s(basisIndex + 1);
        leftValue = 0;
        rightValue = 0;
        if leftDenominator_s > 0
            leftValue = (time_s - knots_s(basisIndex)) / leftDenominator_s * basis(basisIndex);
        end
        if rightDenominator_s > 0
            rightValue = (knots_s(basisIndex + recursionDegree + 1) - time_s) / ...
                rightDenominator_s * basis(basisIndex + 1);
        end
        nextBasis(basisIndex) = leftValue + rightValue;
    end
    basis = nextBasis;
end
basis = basis(1:controlCount);
end
