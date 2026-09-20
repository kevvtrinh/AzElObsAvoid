function polynomial = createPowerPolynomial(controlPoint_units, segmentTime_s, initialTime_s, ...
        givenPower_units, finalTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   polynomial = bmtpEngine.motion.createPowerPolynomial( ...
%       controlPoint_units, segmentTime_s, initialTime_s)
%   polynomial = bmtpEngine.motion.createPowerPolynomial( ...
%       controlPoint_units, segmentTime_s, initialTime_s, givenPower_units)
%   polynomial = bmtpEngine.motion.createPowerPolynomial( ...
%       controlPoint_units, segmentTime_s, initialTime_s, givenPower_units, finalTime_s)
%**************************************************************************
% PURPOSE
%   - Convert composite Bernstein controls to ascending-power polynomials.
%   - Share physical derivatives at joins.
%**************************************************************************
% INPUTS
%   - controlPoint_units (S-by-(D+1)-by-2 numeric array)
%       Composite Bezier control points.
%   - segmentTime_s (positive numeric scalar or S-by-1 vector)
%       Physical segment durations.
%   - initialTime_s (finite numeric scalar)
%       Absolute motion start time.
%   - givenPower_units (S-by-2-by-(D+1) numeric array, optional)
%       Exact analytic coefficients; NaN axes remain optimized.
%   - finalTime_s (finite numeric scalar, optional)
%       Absolute motion end time recorded by the prepared motion. Defaults
%       to the start time plus the summed durations.
%**************************************************************************
% OUTPUTS
%   - polynomial (scalar struct)
%       Position and derivative powers, segment timing, terminal state, and
%       the largest control displacement the C3 join projection introduced.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Convert Bernstein Controls To Powers
segmentCount = size(controlPoint_units, 1);
if isscalar(segmentTime_s)
    segmentTime_s = repmat(segmentTime_s, segmentCount, 1);
end
segmentTime_s = segmentTime_s(:);
degree        = size(controlPoint_units, 2) - 1;
[powerIndex, bernsteinIndex] = ndgrid(0:degree);
conversionEntryIsValid      = bernsteinIndex <= powerIndex;
conversion                  = zeros(degree + 1);
conversion(conversionEntryIsValid) = factorial(degree) * (-1) .^ ...
    (powerIndex(conversionEntryIsValid) - bernsteinIndex(conversionEntryIsValid)) ./ ...
    (factorial(bernsteinIndex(conversionEntryIsValid)) .* ...
    factorial(powerIndex(conversionEntryIsValid) - bernsteinIndex(conversionEntryIsValid)) .* ...
    factorial(degree - powerIndex(conversionEntryIsValid)));
fullyGiven = nargin >= 4 && ~isempty(givenPower_units) && ...
    all(isfinite(givenPower_units), 'all');
% Fully given powers replace every converted coefficient below. Avoid
% solving a discarded control projection, especially on very short spans.
% The projection may only absorb roundoff-sized join residuals; the largest
% control displacement it introduces is reported so the producer can reject
% a repair that would change the motion.
projectionDisplacement_units = 0;
if degree == 5 && ~fullyGiven
    suppliedControlPoint_units   = controlPoint_units;
    controlPoint_units           = projectQuinticContinuity(controlPoint_units, segmentTime_s);
    projectionDisplacement_units = max(abs(controlPoint_units - suppliedControlPoint_units), [], 'all');
end
% Subtract the common origin before conversion to avoid cancellation between
% large absolute coordinates; only the constant power carries the origin.
origin_units        = reshape(controlPoint_units(1, 1, :), 1, 1, 2);
bernsteinPages      = permute(controlPoint_units - origin_units, [2, 1, 3]);
positionPower_units = permute(pagemtimes(conversion, bernsteinPages), [2, 3, 1]);
positionPower_units(:, :, 1) = positionPower_units(:, :, 1) + reshape(origin_units, 1, 2);
if degree > 5
    positionPower_units = stabilizePolynomialEndpoints( ...
        positionPower_units, controlPoint_units, segmentTime_s);
end
if nargin >= 4 && ~isempty(givenPower_units)
    assert(isequal(size(givenPower_units), size(positionPower_units)), ...
        'bmtpEngine:InvalidPrescribedPower', ...
        'Analytic coefficients must match the composite basis.');
    coefficientIsGiven = repmat( ...
        all(isfinite(givenPower_units), 3), 1, 1, degree + 1);
    positionPower_units(coefficientIsGiven) = ...
        givenPower_units(coefficientIsGiven);
end

%% Section 2: Create Physical Derivative Powers And Timing
velocityPower_units_s = positionPower_units(:, :, 2:end) .* ...
    reshape(1:degree, 1, 1, []) ./ segmentTime_s;
accelerationPower_units_s2 = velocityPower_units_s(:, :, 2:end) .* ...
    reshape(1:degree - 1, 1, 1, []) ./ segmentTime_s;
jerkPower_units_s3 = accelerationPower_units_s2(:, :, 2:end) .* ...
    reshape(1:degree - 2, 1, 1, []) ./ segmentTime_s;
segmentStartTime_s = initialTime_s + [0; cumsum(segmentTime_s(1:end - 1))];
if nargin < 5 || isempty(finalTime_s)
    finalTime_s = initialTime_s + sum(segmentTime_s);
end
terminalState      = struct( ...
    "position_units",        sum(positionPower_units(end, :, :), 3), ...
    "velocity_units_s",      sum(velocityPower_units_s(end, :, :), 3), ...
    "acceleration_units_s2", sum(accelerationPower_units_s2(end, :, :), 3));
polynomial = struct( ...
    "Degree",                     degree, ...
    "SegmentCount",               segmentCount, ...
    "SegmentStartTime_s",         segmentStartTime_s, ...
    "SegmentDuration_s",          segmentTime_s, ...
    "FinalTime_s",                finalTime_s, ...
    "positionPower_units",        positionPower_units, ...
    "velocityPower_units_s",      velocityPower_units_s, ...
    "accelerationPower_units_s2", accelerationPower_units_s2, ...
    "jerkPower_units_s3",         jerkPower_units_s3, ...
    "TerminalState",              terminalState, ...
    "ContinuityProjectionDisplacement_units", projectionDisplacement_units);
end

%% Section 3: Local Functions
function power_units = stabilizePolynomialEndpoints(power_units, controlPoint_units, segmentTime_s)
    % Correct roundoff so position through jerk match at Bernstein endpoints.
    degree           = size(controlPoint_units, 2) - 1;
    segmentCount     = size(controlPoint_units, 1);
    endPowerIndices  = degree - 3:degree;
    derivativeOrders = (0:3).';
    endpointMap      = factorial(endPowerIndices) ./ ...
        factorial(endPowerIndices - derivativeOrders);
    endpointTargets  = zeros(segmentCount, 2, 4);
    power_units(:, :, 1) = reshape(controlPoint_units(:, 1, :), segmentCount, 2);
    endpointTargets(:, :, 1) = reshape(controlPoint_units(:, end, :), segmentCount, 2);
    for derivativeOrder = 1:3
        controlDifferences_units = diff(controlPoint_units, derivativeOrder, 2);
        derivativeScale          = factorial(degree) / factorial(degree - derivativeOrder);
        power_units(:, :, derivativeOrder + 1) = ...
            reshape(controlDifferences_units(:, 1, :), segmentCount, 2) * ...
            derivativeScale / factorial(derivativeOrder);
        endpointTargets(:, :, derivativeOrder + 1) = ...
            reshape(controlDifferences_units(:, end, :), segmentCount, 2) * derivativeScale;
    end
    % Share position through jerk so the returned degree-eight motion retains
    % the C3 continuity imposed by the trajectory problem.
    % A small normalized solver residual can otherwise be amplified by the
    % inverse square of a short span's duration. This changes the returned curve;
    % all derivative bounds and collision proofs are rebuilt afterward.
    for derivativeOrder = 0:3
        left   = endpointTargets(1:end - 1, :, derivativeOrder + 1) ./ ...
            segmentTime_s(1:end - 1) .^ derivativeOrder;
        right  = power_units(2:end, :, derivativeOrder + 1) * factorial(derivativeOrder) ./ ...
            segmentTime_s(2:end) .^ derivativeOrder;
        common = (left + right) / 2;
        endpointTargets(1:end - 1, :, derivativeOrder + 1) = common .* ...
            segmentTime_s(1:end - 1) .^ derivativeOrder;
        power_units(2:end, :, derivativeOrder + 1) = common .* ...
            segmentTime_s(2:end) .^ derivativeOrder / factorial(derivativeOrder);
    end
    for projectionPass = 1:2
        currentEndpointValues = zeros(segmentCount, 2, 4);
        for derivativeOrder = 0:3
            powerIndices = derivativeOrder:degree;
            multipliers  = reshape(factorial(powerIndices) ./ ...
                factorial(powerIndices - derivativeOrder), 1, 1, []);
            currentEndpointValues(:, :, derivativeOrder + 1) = ...
                sum(power_units(:, :, powerIndices + 1) .* multipliers, 3);
        end
        endpointResidual   = reshape( ...
            permute(endpointTargets - currentEndpointValues, [3 1 2]), 4, []);
        endpointCorrection = permute( ...
            reshape(endpointMap \ endpointResidual, 4, segmentCount, 2), [2, 3, 1]);
        power_units(:, :, endPowerIndices + 1) = ...
            power_units(:, :, endPowerIndices + 1) + endpointCorrection;
    end
end

function controls_units = projectQuinticContinuity(controls_units, durations_s)
    % A global linear projection enforces C3 joins while preserving endpoint
    % p/v/a. Unlike independent span endpoint repair, these equations fit in
    % the quintic spline space. The modified curve is proven afterward.
    segmentCount = size(controls_units, 1);
    differenceCoefficients = {1, [-1, 1], [1, -2, 1], [-1, 3, -3, 1]};
    rowCount               = 6 + 4 * (segmentCount - 1);
    constraintMap          = spalloc(rowCount, 6 * segmentCount, 8 * rowCount);
    rowIndex               = 0;
    for derivativeOrder = 0:2
        scale    = factorial(5) / factorial(5 - derivativeOrder);
        rowIndex = rowIndex + 1;
        constraintMap(rowIndex, 1:derivativeOrder + 1) = ...
            scale * differenceCoefficients{derivativeOrder + 1} / ...
            durations_s(1) ^ derivativeOrder;
        rowIndex = rowIndex + 1;
        constraintMap(rowIndex, 6 * segmentCount - derivativeOrder:6 * segmentCount) = ...
            scale * differenceCoefficients{derivativeOrder + 1} / ...
            durations_s(end) ^ derivativeOrder;
    end
    for segmentIndex = 1:segmentCount - 1
        for derivativeOrder = 0:3
            scale    = factorial(5) / factorial(5 - derivativeOrder);
            rowIndex = rowIndex + 1;
            constraintMap(rowIndex, 6 * segmentIndex - derivativeOrder:6 * segmentIndex) = ...
                scale * differenceCoefficients{derivativeOrder + 1} / ...
                durations_s(segmentIndex) ^ derivativeOrder;
            constraintMap(rowIndex, 6 * segmentIndex + (1:derivativeOrder + 1)) = ...
                -scale * differenceCoefficients{derivativeOrder + 1} / ...
                durations_s(segmentIndex + 1) ^ derivativeOrder;
        end
    end
    rowScale             = full(max(abs(constraintMap), [], 2));
    constraintMap        = spdiags(1 ./ rowScale, 0, rowCount, rowCount) * constraintMap;
    origin_units         = reshape(controls_units(1, 1, :), 1, 2);
    controlValues_units  = reshape(permute(controls_units, [2, 1, 3]), [], 2) - origin_units;
    equalityTarget_units = zeros(rowCount, 2);
    equalityTarget_units(1:6, :) = constraintMap(1:6, :) * controlValues_units;
    gramFactor = decomposition(constraintMap * constraintMap.', 'chol');
    for projectionPass = 1:2
        controlValues_units = controlValues_units + constraintMap.' * ...
            (gramFactor \ (equalityTarget_units - constraintMap * controlValues_units));
    end
    controls_units = permute( ...
        reshape(controlValues_units + origin_units, 6, segmentCount, 2), [2, 1, 3]);
end
