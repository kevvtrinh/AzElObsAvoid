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
%   - Describe each Bezier curve with powers of segment fraction u, where
%     u runs from 0 at the start to 1 at the end. This conversion alone
%     keeps the same path and makes motion-rate and endpoint checks easier.
%   - Where needed, make neighboring segments meet smoothly. Calculate
%     velocity, acceleration, jerk, and timing, then report any degree-five
%     control-point movement for the caller to judge.
%**************************************************************************
% INPUTS
%   - controlPoint_units (S-by-(D+1)-by-2 numeric array)
%       S is the segment count; each degree-D curve has D+1 control points.
%       The last dimension holds the two position axes.
%   - segmentTime_s (positive numeric scalar or S-by-1 vector)
%       Duration of each segment. A scalar gives every segment the same time.
%   - initialTime_s (finite numeric scalar)
%       Absolute motion start time.
%   - suppliedPowerCoefficients_units (S-by-2-by-(D+1) numeric array, optional)
%       Optional position coefficients in ascending powers of u. A supplied
%       axis replaces converted controls only if all its coefficients are
%       finite. An axis containing NaN keeps the result from its controls.
%   - finalTime_s (finite numeric scalar, optional)
%       End time to record. Defaults to initialTime_s plus all durations;
%       supplying it does not change any segment duration.
%**************************************************************************
% OUTPUTS
%   - polynomial (scalar struct)
%       Position and motion-rate coefficients, segment times, final state,
%       and largest coordinate change to degree-five controls during joins.
%       This function reports that change; the caller decides whether the
%       adjusted curve is acceptable.
%**************************************************************************
% UNITS
%   - Position and its power coefficients use coordinate units. Velocity,
%     acceleration, and jerk use units/s, units/s^2, and units/s^3.
%     Time is seconds, and segment fraction u is unitless.
%**************************************************************************

%% Section 1: Match Joins And Build Position Coefficients

segmentCount = size(controlPoint_units, 1);
if isscalar(segmentTime_s)
    segmentTime_s = repmat(segmentTime_s, segmentCount, 1);
end
segmentTime_s = segmentTime_s(:);
degree        = size(controlPoint_units, 2) - 1;

% The map rewrites the same curve in powers of u. For one quadratic axis,
% Bezier controls [0, 1, 0] become p(u) = 2 x u - 2 x u^2. Curve degree and
% control indices start at 0 here; MATLAB array indices start at 1.
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

% If every coefficient was supplied, those values will replace the controls'
% conversion, so moving controls would have no effect. Otherwise, adjust
% degree-five controls so neighboring segments meet through jerk. Report the
% largest movement; the caller decides whether it is small enough.
joinAdjustment_units = 0;
if degree == 5 && ~allCoefficientsWereSupplied
    suppliedControlPoint_units = controlPoint_units;
    controlPoint_units         = matchQuinticSegmentJoins(controlPoint_units, segmentTime_s);
    joinAdjustment_units       = max(abs(controlPoint_units - suppliedControlPoint_units), [], 'all');
end

% Subtract the first control point before conversion. If coordinates are
% near 1e9 but the curve moves by 1 unit, this keeps the calculation near 1
% instead of 1e9. Add the origin back to the constant coefficient only.
coordinateOrigin_units     = reshape(controlPoint_units(1, 1, :), 1, 1, 2);
offsetControlPages_units   = permute(controlPoint_units - coordinateOrigin_units, [2, 1, 3]);
positionCoefficients_units = permute(pagemtimes(bernsteinToPowerMap, offsetControlPages_units), [2, 3, 1]);
positionCoefficients_units(:, :, 1) = positionCoefficients_units(:, :, 1) + reshape(coordinateOrigin_units, 1, 2);

% For curves above degree five, adjust polynomial coefficients to align
% endpoint states. This may change the curve, so later motion checks must
% examine the adjusted result.
if degree > 5 && ~allCoefficientsWereSupplied
    positionCoefficients_units = matchPolynomialEndpointStates( ...
        positionCoefficients_units, controlPoint_units, segmentTime_s);
end

% Replace a whole axis only when its supplied coefficients all exist and are
% finite. Mixing supplied and converted terms within one axis would describe
% a curve the caller did not provide.
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

% u = elapsed time / segment duration, so each time derivative divides by
% that duration. If p(u) = 4 x u over 2 seconds, velocity is 4 / 2 = 2
% units/s. The next derivatives divide by duration again.
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
    % Match position, velocity, acceleration, and jerk at each join. Set
    % each segment's start values from its controls, then adjust the highest
    % four powers toward the chosen end state. Later checks examine the curve.
    degree = size(controlPoint_units, 2) - 1;
    segmentCount = size(controlPoint_units, 1);
    endCorrectionDegrees = degree - 3:degree;
    endpointDerivativeOrders = (0:3).';
    % This map tells how changing the last four powers changes end position
    % and its first three derivatives with respect to u.
    endCorrectionMap = factorial(endCorrectionDegrees) ./ ...
        factorial(endCorrectionDegrees - endpointDerivativeOrders);
    endStateTargets = zeros(segmentCount, 2, 4);
    positionCoefficients_units(:, :, 1) = reshape(controlPoint_units(:, 1, :), segmentCount, 2);
    endStateTargets(:, :, 1) = reshape(controlPoint_units(:, end, :), segmentCount, 2);

    % Control-point differences give endpoint derivatives with respect to u.
    % For degree D, the start derivative is D x (P1 - P0). Divide by the
    % segment duration below to compare physical velocity.
    for derivativeOrder = 1:3
        controlDifferences_units = diff(controlPoint_units, derivativeOrder, 2);
        derivativeFactor         = factorial(degree) / factorial(degree - derivativeOrder);
        positionCoefficients_units(:, :, derivativeOrder + 1) = ...
            reshape(controlDifferences_units(:, 1, :), segmentCount, 2) * ...
            derivativeFactor / factorial(derivativeOrder);
        endStateTargets(:, :, derivativeOrder + 1) = ...
            reshape(controlDifferences_units(:, end, :), segmentCount, 2) * derivativeFactor;
    end

    % Compare the two sides of a join in physical time. For the same speed,
    % a 2-second segment needs twice the derivative with respect to u of a
    % 1-second segment. Divide by duration^order, average the two sides,
    % then convert back to each segment's u-based coefficients.
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

    % Change the last four coefficients to reach the chosen end position
    % and three derivatives. Repeat once to reduce rounding error from the
    % first adjustment.
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
    % Move degree-five controls so neighboring segments meet in position,
    % velocity, acceleration, and jerk. Keep the motion's initial and final
    % position, velocity, and acceleration fixed. The caller checks how far
    % controls moved and validates the changed curve.
    segmentCount = size(controlPoint_units, 1);
    controlDifferenceWeights = {1, [-1, 1], [1, -2, 1], [-1, 3, -3, 1]};
    constraintCount = 6 + 4 * (segmentCount - 1);
    endpointAndJoinMap = spalloc(constraintCount, 6 * segmentCount, 8 * constraintCount);
    constraintIndex = 0;

    % Preserve position, velocity, and acceleration at both ends: three
    % equations per end. Each join adds four equations requiring its left
    % and right position, velocity, acceleration, and jerk to agree.
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

    % A time derivative divides by duration^order, so a short segment can
    % make one equation much larger than another. Scale the equations before
    % solving, and subtract the first position to reduce coordinate rounding.
    constraintRowScale          = full(max(abs(endpointAndJoinMap), [], 2));
    endpointAndJoinMap          = spdiags(1 ./ constraintRowScale, 0, constraintCount, constraintCount) * endpointAndJoinMap;
    coordinateOrigin_units      = reshape(controlPoint_units(1, 1, :), 1, 2);
    centeredControlValues_units = reshape(permute(controlPoint_units, [2, 1, 3]), [], 2) - coordinateOrigin_units;
    requiredConstraintValues    = zeros(constraintCount, 2);
    requiredConstraintValues(1:6, :) = endpointAndJoinMap(1:6, :) * centeredControlValues_units;

    % Keep the original endpoint values; every join targets a zero left-right
    % difference. Solve the same matrix twice to reduce rounding error.
    constraintSystemFactor = decomposition(endpointAndJoinMap * endpointAndJoinMap.', 'chol');
    for adjustmentPass = 1:2
        centeredControlValues_units = centeredControlValues_units + endpointAndJoinMap.' * ...
            (constraintSystemFactor \ (requiredConstraintValues - endpointAndJoinMap * centeredControlValues_units));
    end
    controlPoint_units = permute( ...
        reshape(centeredControlValues_units + coordinateOrigin_units, 6, segmentCount, 2), [2, 1, 3]);
end
