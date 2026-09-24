function polynomial = createPowerPolynomial(controlPoint_units, segmentTime_s, initialTime_s, ...
    suppliedPowerCoefficients_units, finalTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   polynomial = bmtpEngine.motion.createPowerPolynomial( ...
%       controlPoint_units, segmentTime_s, initialTime_s)
%   polynomial = bmtpEngine.motion.createPowerPolynomial( ...
%       controlPoint_units, segmentTime_s, initialTime_s, suppliedPowerCoefficients_units)
%   polynomial = bmtpEngine.motion.createPowerPolynomial( ...
%       controlPoint_units, segmentTime_s, initialTime_s, suppliedPowerCoefficients_units, finalTime_s)
%**************************************************************************
% PURPOSE
%   - Express each Bezier segment as p(u) = c0 + c1 x u + c2 x u^2 + ...
%     where segment fraction u runs from 0 to 1. Also calculate velocity,
%     acceleration, jerk, and segment times for motion checks and sampling.
%   - Match segment joins where required and report degree-five control
%     changes so the caller can reject adjustments beyond its tolerance.
%**************************************************************************
% INPUTS
%   - controlPoint_units (S-by-(D+1)-by-2 numeric array)
%       Composite Bezier control points.
%   - segmentTime_s (positive numeric scalar or S-by-1 vector)
%       Physical segment durations.
%   - initialTime_s (finite numeric scalar)
%       Absolute motion start time.
%   - suppliedPowerCoefficients_units (S-by-2-by-(D+1) numeric array, optional)
%       Supplied position coefficients override converted controls for each
%       segment/axis whose coefficients are all finite. An axis containing
%       NaN keeps the coefficients calculated from its controls.
%   - finalTime_s (finite numeric scalar, optional)
%       Absolute motion end time recorded by the prepared motion. Defaults
%       to the start time plus the summed durations.
%**************************************************************************
% OUTPUTS
%   - polynomial (scalar struct)
%       Position and derivative coefficients, segment times, final state,
%       and largest coordinate change from degree-five join matching.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Convert Bezier Controls To Position Coefficients

segmentCount = size(controlPoint_units, 1);
if isscalar(segmentTime_s)
    segmentTime_s = repmat(segmentTime_s, segmentCount, 1);
end
segmentTime_s = segmentTime_s(:);
degree        = size(controlPoint_units, 2) - 1;

% Each map row gives one power coefficient as a weighted sum of controls.
% These degree/control indices start at 0; MATLAB array indices start at 1.
[powerDegrees, controlPointIndices] = ndgrid(0:degree);
conversionEntryIsValid = controlPointIndices <= powerDegrees;
bernsteinToPowerMap    = zeros(degree + 1);
bernsteinToPowerMap(conversionEntryIsValid) = factorial(degree) * (-1) .^ ...
    (powerDegrees(conversionEntryIsValid) - controlPointIndices(conversionEntryIsValid)) ./ ...
    (factorial(controlPointIndices(conversionEntryIsValid)) .* ...
    factorial(powerDegrees(conversionEntryIsValid) - controlPointIndices(conversionEntryIsValid)) .* ...
    factorial(degree - powerDegrees(conversionEntryIsValid)));
allCoefficientsWereSupplied = nargin >= 4 && ~isempty(suppliedPowerCoefficients_units) && ...
    all(isfinite(suppliedPowerCoefficients_units), 'all');

% Fully supplied coefficients replace the converted values, so skip control
% adjustments that would be discarded. Otherwise, match degree-five joins
% and report the largest coordinate change. The caller checks that this
% change is small enough; this function does not accept the motion by itself.
joinAdjustment_units = 0;
if degree == 5 && ~allCoefficientsWereSupplied
    suppliedControlPoint_units = controlPoint_units;
    controlPoint_units         = matchQuinticSegmentJoins(controlPoint_units, segmentTime_s);
    joinAdjustment_units       = max(abs(controlPoint_units - suppliedControlPoint_units), [], 'all');
end

% Convert offsets from a common origin to reduce rounding error with large
% coordinates. Add the origin back only to the constant coefficient c0.
coordinateOrigin_units     = reshape(controlPoint_units(1, 1, :), 1, 1, 2);
offsetControlPages_units   = permute(controlPoint_units - coordinateOrigin_units, [2, 1, 3]);
positionCoefficients_units = permute(pagemtimes(bernsteinToPowerMap, offsetControlPages_units), [2, 3, 1]);
positionCoefficients_units(:, :, 1) = positionCoefficients_units(:, :, 1) + reshape(coordinateOrigin_units, 1, 2);

% Higher-degree segments have extra coefficients for matching endpoint states.
if degree > 5 && ~allCoefficientsWereSupplied
    positionCoefficients_units = matchPolynomialEndpointStates( ...
        positionCoefficients_units, controlPoint_units, segmentTime_s);
end

% A supplied coefficient set must match the segment/axis/degree layout.
% Replace a whole axis only when every coefficient for that axis is finite.
if nargin >= 4 && ~isempty(suppliedPowerCoefficients_units)
    assert(isequal(size(suppliedPowerCoefficients_units), size(positionCoefficients_units)), ...
        'bmtpEngine:InvalidPrescribedPower', ...
        'Analytic coefficients must match the composite basis.');
    coefficientWasSupplied = repmat( ...
        all(isfinite(suppliedPowerCoefficients_units), 3), 1, 1, degree + 1);
    positionCoefficients_units(coefficientWasSupplied) = ...
        suppliedPowerCoefficients_units(coefficientWasSupplied);
end

%% Section 2: Calculate Velocity, Acceleration, Jerk, And Timing

% u = elapsed time / segment duration. Each time derivative therefore
% divides by duration: c2 x u^2 contributes 2 x c2 x u / duration to velocity.
velocityCoefficients_units_s = positionCoefficients_units(:, :, 2:end) .* ...
    reshape(1:degree, 1, 1, []) ./ segmentTime_s;
accelerationCoefficients_units_s2 = velocityCoefficients_units_s(:, :, 2:end) .* ...
    reshape(1:degree - 1, 1, 1, []) ./ segmentTime_s;
jerkCoefficients_units_s3 = accelerationCoefficients_units_s2(:, :, 2:end) .* ...
    reshape(1:degree - 2, 1, 1, []) ./ segmentTime_s;
segmentStartTime_s = initialTime_s + [0; cumsum(segmentTime_s(1:end - 1))];
if nargin < 5 || isempty(finalTime_s)
    finalTime_s = initialTime_s + sum(segmentTime_s);
end

% At the last segment's end, u = 1, so adding its coefficients gives
% each final position, velocity, and acceleration.
terminalState = struct( ...
    "position_units",        sum(positionCoefficients_units(end, :, :), 3), ...
    "velocity_units_s",      sum(velocityCoefficients_units_s(end, :, :), 3), ...
    "acceleration_units_s2", sum(accelerationCoefficients_units_s2(end, :, :), 3));
polynomial = struct( ...
    "Degree",                     degree, ...
    "SegmentCount",               segmentCount, ...
    "SegmentStartTime_s",         segmentStartTime_s, ...
    "SegmentDuration_s",          segmentTime_s, ...
    "FinalTime_s",                finalTime_s, ...
    "positionPower_units",        positionCoefficients_units, ...
    "velocityPower_units_s",      velocityCoefficients_units_s, ...
    "accelerationPower_units_s2", accelerationCoefficients_units_s2, ...
    "jerkPower_units_s3",         jerkCoefficients_units_s3, ...
    "TerminalState",              terminalState, ...
    "ContinuityProjectionDisplacement_units", joinAdjustment_units);
end

%% Section 3: Local Functions

function positionCoefficients_units = matchPolynomialEndpointStates( ...
        positionCoefficients_units, controlPoint_units, segmentTime_s)
    % Match position, velocity, acceleration, and jerk at segment endpoints.
    % The first four coefficients set the start state; the final four are
    % adjusted to meet the chosen end state. Later checks verify the curve.
    degree = size(controlPoint_units, 2) - 1;
    segmentCount = size(controlPoint_units, 1);
    endCorrectionDegrees = degree - 3:degree;
    endpointDerivativeOrders = (0:3).';
    endCorrectionMap = factorial(endCorrectionDegrees) ./ ...
        factorial(endCorrectionDegrees - endpointDerivativeOrders);
    endStateTargets = zeros(segmentCount, 2, 4);
    positionCoefficients_units(:, :, 1) = reshape(controlPoint_units(:, 1, :), segmentCount, 2);
    endStateTargets(:, :, 1) = reshape(controlPoint_units(:, end, :), segmentCount, 2);

    % Successive control differences give endpoint derivatives. For example,
    % the start velocity before dividing by duration is D x (P1 - P0).
    for derivativeOrder = 1:3
        controlDifferences_units = diff(controlPoint_units, derivativeOrder, 2);
        derivativeFactor         = factorial(degree) / factorial(degree - derivativeOrder);
        positionCoefficients_units(:, :, derivativeOrder + 1) = ...
            reshape(controlDifferences_units(:, 1, :), segmentCount, 2) * ...
            derivativeFactor / factorial(derivativeOrder);
        endStateTargets(:, :, derivativeOrder + 1) = ...
            reshape(controlDifferences_units(:, end, :), segmentCount, 2) * derivativeFactor;
    end

    % Average each join's two states in physical time units, then convert
    % back to segment-fraction coefficients. Unequal durations must not
    % change what matching velocity or acceleration means. These adjustments
    % change the curve; motion limits and obstacle separation are checked later.
    for derivativeOrder = 0:3
        leftEndState = endStateTargets(1:end - 1, :, derivativeOrder + 1) ./ ...
            segmentTime_s(1:end - 1) .^ derivativeOrder;
        rightStartState = positionCoefficients_units(2:end, :, derivativeOrder + 1) * ...
            factorial(derivativeOrder) ./ ...
            segmentTime_s(2:end) .^ derivativeOrder;
        sharedJoinState = (leftEndState + rightStartState) / 2;
        endStateTargets(1:end - 1, :, derivativeOrder + 1) = sharedJoinState .* ...
            segmentTime_s(1:end - 1) .^ derivativeOrder;
        positionCoefficients_units(2:end, :, derivativeOrder + 1) = sharedJoinState .* ...
            segmentTime_s(2:end) .^ derivativeOrder / factorial(derivativeOrder);
    end

    % Solve for changes to the final four coefficients. Repeat once to
    % reduce rounding error left by the first adjustment.
    for adjustmentPass = 1:2
        currentEndStates = zeros(segmentCount, 2, 4);
        for derivativeOrder = 0:3
            coefficientDegrees    = derivativeOrder:degree;
            derivativeMultipliers = reshape(factorial(coefficientDegrees) ./ ...
                factorial(coefficientDegrees - derivativeOrder), 1, 1, []);
            currentEndStates(:, :, derivativeOrder + 1) = ...
                sum(positionCoefficients_units(:, :, coefficientDegrees + 1) .* derivativeMultipliers, 3);
        end
        endStateDifference = reshape( ...
            permute(endStateTargets - currentEndStates, [3 1 2]), 4, []);
        endCoefficientAdjustment = permute( ...
            reshape(endCorrectionMap \ endStateDifference, 4, segmentCount, 2), [2, 3, 1]);
        positionCoefficients_units(:, :, endCorrectionDegrees + 1) = ...
            positionCoefficients_units(:, :, endCorrectionDegrees + 1) + endCoefficientAdjustment;
    end
end

function controlPoint_units = matchQuinticSegmentJoins(controlPoint_units, segmentTime_s)
    % Adjust the degree-five controls together so adjacent segments share
    % position, velocity, acceleration, and jerk. Keep the trip's initial
    % and final position, velocity, and acceleration unchanged.
    % The caller checks the adjustment size and validates the changed curve.
    segmentCount = size(controlPoint_units, 1);
    controlDifferenceWeights = {1, [-1, 1], [1, -2, 1], [-1, 3, -3, 1]};
    constraintCount = 6 + 4 * (segmentCount - 1);
    endpointAndJoinMap = spalloc(constraintCount, 6 * segmentCount, 8 * constraintCount);
    constraintIndex = 0;

    % Six rows preserve the two endpoint states (position, velocity,
    % acceleration). Each join adds four rows requiring left - right = 0.
    for derivativeOrder = 0:2
        derivativeFactor = factorial(5) / factorial(5 - derivativeOrder);
        constraintIndex  = constraintIndex + 1;
        endpointAndJoinMap(constraintIndex, 1:derivativeOrder + 1) = ...
            derivativeFactor * controlDifferenceWeights{derivativeOrder + 1} / ...
            segmentTime_s(1) ^ derivativeOrder;
        constraintIndex = constraintIndex + 1;
        endpointAndJoinMap(constraintIndex, 6 * segmentCount - derivativeOrder:6 * segmentCount) = ...
            derivativeFactor * controlDifferenceWeights{derivativeOrder + 1} / ...
            segmentTime_s(end) ^ derivativeOrder;
    end
    for segmentIndex = 1:segmentCount - 1
        for derivativeOrder = 0:3
            derivativeFactor = factorial(5) / factorial(5 - derivativeOrder);
            constraintIndex  = constraintIndex + 1;
            endpointAndJoinMap(constraintIndex, 6 * segmentIndex - derivativeOrder:6 * segmentIndex) = ...
                derivativeFactor * controlDifferenceWeights{derivativeOrder + 1} / ...
                segmentTime_s(segmentIndex) ^ derivativeOrder;
            endpointAndJoinMap(constraintIndex, 6 * segmentIndex + (1:derivativeOrder + 1)) = ...
                -derivativeFactor * controlDifferenceWeights{derivativeOrder + 1} / ...
                segmentTime_s(segmentIndex + 1) ^ derivativeOrder;
        end
    end

    % Scale each equation before solving so short durations do not create
    % very large derivative rows. Use offsets to reduce coordinate rounding.
    constraintRowScale          = full(max(abs(endpointAndJoinMap), [], 2));
    endpointAndJoinMap          = spdiags(1 ./ constraintRowScale, 0, constraintCount, constraintCount) * endpointAndJoinMap;
    coordinateOrigin_units      = reshape(controlPoint_units(1, 1, :), 1, 2);
    centeredControlValues_units = reshape(permute(controlPoint_units, [2, 1, 3]), [], 2) - coordinateOrigin_units;
    requiredConstraintValues    = zeros(constraintCount, 2);
    requiredConstraintValues(1:6, :) = endpointAndJoinMap(1:6, :) * centeredControlValues_units;

    % Only endpoint rows may have nonzero targets; all join differences
    % should be zero. Reuse one factorization for both adjustment passes.
    constraintSystemFactor = decomposition(endpointAndJoinMap * endpointAndJoinMap.', 'chol');
    for adjustmentPass = 1:2
        centeredControlValues_units = centeredControlValues_units + endpointAndJoinMap.' * ...
            (constraintSystemFactor \ (requiredConstraintValues - endpointAndJoinMap * centeredControlValues_units));
    end
    controlPoint_units = permute( ...
        reshape(centeredControlValues_units + coordinateOrigin_units, 6, segmentCount, 2), [2, 1, 3]);
end
