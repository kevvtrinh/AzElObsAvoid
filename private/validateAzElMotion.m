function evidence = validateAzElMotion(command, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   evidence = validateAzElMotion(command, limits, options)
%**************************************************************************
% PURPOSE
%   - Verify continuous piecewise-quintic position, velocity, and
%     acceleration limits using polynomial extrema rather than samples.
%**************************************************************************
% INPUTS
%   - command (scalar struct)
%       Coherent knot histories interpreted as quintic Hermite segments.
%   - limits (scalar struct)
%       Position, velocity, and acceleration bounds.
%   - options (scalar struct)
%       Resolved mission tolerances and azimuth-wrap policy.
%**************************************************************************
% OUTPUTS
%   - evidence (scalar struct)
%       Exact-extrema limit evidence and motion-quality metrics.
%**************************************************************************
% UNITS
%   - Angles are degrees and time is seconds.

%% Section 1: Validate Knot Histories
knotTime_s = double(command.time_s(:));
position_deg = double(command.unwrappedPosition_deg);
velocity_deg_s = double(command.velocity_deg_s);
acceleration_deg_s2 = double(command.acceleration_deg_s2);
knotCount = numel(knotTime_s);

timestampIsValid = knotCount >= 2 && all(isfinite(knotTime_s)) && ...
    all(diff(knotTime_s) > 0);
historyIsFinite = isequal(size(position_deg), [knotCount, 2]) && ...
    isequal(size(velocity_deg_s), [knotCount, 2]) && ...
    isequal(size(acceleration_deg_s2), [knotCount, 2]) && ...
    all(isfinite(position_deg), "all") && ...
    all(isfinite(velocity_deg_s), "all") && ...
    all(isfinite(acceleration_deg_s2), "all");

evidence = motionEvidenceTemplate();
evidence.timestampIsValid = timestampIsValid;
evidence.historyIsFinite = historyIsFinite;
if ~timestampIsValid || ~historyIsFinite
    evidence.message = "Command knot histories are invalid.";
    return;
end

%% Section 2: Evaluate Continuous Extrema
minimumPosition_deg = inf(1, 2);
maximumPosition_deg = -inf(1, 2);
maximumAbsoluteVelocity_deg_s = zeros(1, 2);
maximumAbsoluteAcceleration_deg_s2 = zeros(1, 2);
maximumSpeed_deg_s = 0;
angularDistance_deg = 0;

for segmentIndex = 1:(knotCount - 1)
    duration_s = knotTime_s(segmentIndex + 1) - ...
        knotTime_s(segmentIndex);
    startState = stateAtKnot(position_deg, velocity_deg_s, ...
        acceleration_deg_s2, segmentIndex);
    endState = stateAtKnot(position_deg, velocity_deg_s, ...
        acceleration_deg_s2, segmentIndex + 1);
    coefficients = quinticHermiteCoefficients( ...
        startState, endState, duration_s);

    axisVelocityBounds_deg_s = zeros(1, 2);
    for axisIndex = 1:2
        positionCandidates_s = [0; duration_s; ...
            polynomialRootsInside(derivativeCoefficients( ...
                coefficients(:, axisIndex), 1), duration_s)];
        velocityCandidates_s = [0; duration_s; ...
            polynomialRootsInside(derivativeCoefficients( ...
                coefficients(:, axisIndex), 2), duration_s)];
        accelerationCandidates_s = [0; duration_s; ...
            polynomialRootsInside(derivativeCoefficients( ...
                coefficients(:, axisIndex), 3), duration_s)];

        segmentPosition_deg = evaluateAscendingPolynomial( ...
            coefficients(:, axisIndex), positionCandidates_s);
        velocityCoefficients = derivativeCoefficients( ...
            coefficients(:, axisIndex), 1);
        segmentVelocity_deg_s = evaluateAscendingPolynomial( ...
            velocityCoefficients, velocityCandidates_s);
        accelerationCoefficients = derivativeCoefficients( ...
            coefficients(:, axisIndex), 2);
        segmentAcceleration_deg_s2 = evaluateAscendingPolynomial( ...
            accelerationCoefficients, accelerationCandidates_s);

        minimumPosition_deg(axisIndex) = min( ...
            minimumPosition_deg(axisIndex), min(segmentPosition_deg));
        maximumPosition_deg(axisIndex) = max( ...
            maximumPosition_deg(axisIndex), max(segmentPosition_deg));
        axisVelocityBounds_deg_s(axisIndex) = ...
            max(abs(segmentVelocity_deg_s));
        maximumAbsoluteVelocity_deg_s(axisIndex) = max( ...
            maximumAbsoluteVelocity_deg_s(axisIndex), ...
            axisVelocityBounds_deg_s(axisIndex));
        maximumAbsoluteAcceleration_deg_s2(axisIndex) = max( ...
            maximumAbsoluteAcceleration_deg_s2(axisIndex), ...
            max(abs(segmentAcceleration_deg_s2)));
    end
    maximumSpeed_deg_s = max(maximumSpeed_deg_s, ...
        norm(axisVelocityBounds_deg_s));

    metricTime_s = linspace(0, duration_s, 33).';
    segmentVelocity_deg_s = zeros(numel(metricTime_s), 2);
    for axisIndex = 1:2
        segmentVelocity_deg_s(:, axisIndex) = ...
            evaluateAscendingPolynomial(derivativeCoefficients( ...
                coefficients(:, axisIndex), 1), metricTime_s);
    end
    segmentSpeed_deg_s = vecnorm(segmentVelocity_deg_s, 2, 2);
    angularDistance_deg = angularDistance_deg + ...
        trapz(metricTime_s, segmentSpeed_deg_s);
end

%% Section 3: Compare Physical Limits
positionTolerance_deg = options.positionTolerance_deg;
velocityTolerance_deg_s = options.velocityTolerance_deg_s;
accelerationTolerance_deg_s2 = options.accelerationTolerance_deg_s2;

positionViolation_deg = zeros(1, 2);
if ~options.azimuthWrap
    positionViolation_deg(1) = max([ ...
        limits.azimuth_deg(1) - minimumPosition_deg(1), ...
        maximumPosition_deg(1) - limits.azimuth_deg(2), 0]);
end
positionViolation_deg(2) = max([ ...
    limits.elevation_deg(1) - minimumPosition_deg(2), ...
    maximumPosition_deg(2) - limits.elevation_deg(2), 0]);
velocityViolation_deg_s = max( ...
    maximumAbsoluteVelocity_deg_s - limits.maxVelocity_deg_s, 0);
accelerationViolation_deg_s2 = max( ...
    maximumAbsoluteAcceleration_deg_s2 - ...
        limits.maxAcceleration_deg_s2, 0);

positionIsValid = all(positionViolation_deg <= positionTolerance_deg);
velocityIsValid = all( ...
    velocityViolation_deg_s <= velocityTolerance_deg_s);
accelerationIsValid = all( ...
    accelerationViolation_deg_s2 <= accelerationTolerance_deg_s2);

interiorSpeed_deg_s = vecnorm(velocity_deg_s(2:end-1, :), 2, 2);
if isempty(interiorSpeed_deg_s)
    minimumInteriorSpeed_deg_s = NaN;
else
    minimumInteriorSpeed_deg_s = min(interiorSpeed_deg_s);
end

evidence.positionIsValid = positionIsValid;
evidence.velocityIsValid = velocityIsValid;
evidence.accelerationIsValid = accelerationIsValid;
evidence.minimumPosition_deg = minimumPosition_deg;
evidence.maximumPosition_deg = maximumPosition_deg;
evidence.maximumAbsoluteVelocity_deg_s = ...
    maximumAbsoluteVelocity_deg_s;
evidence.maximumAbsoluteAcceleration_deg_s2 = ...
    maximumAbsoluteAcceleration_deg_s2;
evidence.positionViolation_deg = positionViolation_deg;
evidence.velocityViolation_deg_s = velocityViolation_deg_s;
evidence.accelerationViolation_deg_s2 = accelerationViolation_deg_s2;
evidence.maximumSpeed_deg_s = maximumSpeed_deg_s;
evidence.angularDistance_deg = angularDistance_deg;
evidence.minimumInteriorSpeed_deg_s = minimumInteriorSpeed_deg_s;
evidence.isValid = positionIsValid && velocityIsValid && ...
    accelerationIsValid;
if evidence.isValid
    evidence.message = "Continuous polynomial extrema satisfy all limits.";
else
    evidence.message = "At least one continuous motion limit is violated.";
end
end

function evidence = motionEvidenceTemplate()
%% Section 0: Header & Readme
% SYNTAX
%   evidence = motionEvidenceTemplate()
%**************************************************************************
% PURPOSE
%   - Return a stable motion-evidence schema for every validator exit.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - evidence (scalar struct)
%**************************************************************************
% UNITS
%   - Field names state their units.

evidence = struct( ...
    "isValid", false, ...
    "message", "Motion validation did not complete.", ...
    "timestampIsValid", false, ...
    "historyIsFinite", false, ...
    "positionIsValid", false, ...
    "velocityIsValid", false, ...
    "accelerationIsValid", false, ...
    "minimumPosition_deg", [NaN, NaN], ...
    "maximumPosition_deg", [NaN, NaN], ...
    "maximumAbsoluteVelocity_deg_s", [NaN, NaN], ...
    "maximumAbsoluteAcceleration_deg_s2", [NaN, NaN], ...
    "positionViolation_deg", [Inf, Inf], ...
    "velocityViolation_deg_s", [Inf, Inf], ...
    "accelerationViolation_deg_s2", [Inf, Inf], ...
    "maximumSpeed_deg_s", Inf, ...
    "angularDistance_deg", Inf, ...
    "minimumInteriorSpeed_deg_s", NaN);
end

function state = stateAtKnot(position_deg, velocity_deg_s, ...
        acceleration_deg_s2, knotIndex)
%% Section 0: Header & Readme
% SYNTAX
%   state = stateAtKnot(position_deg, velocity_deg_s, ...
%       acceleration_deg_s2, knotIndex)
%**************************************************************************
% PURPOSE
%   - Assemble one polynomial boundary state.
%**************************************************************************
% INPUTS
%   - position_deg (N-by-2 numeric)
%   - velocity_deg_s (N-by-2 numeric)
%   - acceleration_deg_s2 (N-by-2 numeric)
%   - knotIndex (positive integer)
%**************************************************************************
% OUTPUTS
%   - state (scalar struct)
%**************************************************************************
% UNITS
%   - Field names state their units.

state = struct( ...
    "position_deg", position_deg(knotIndex, :), ...
    "velocity_deg_s", velocity_deg_s(knotIndex, :), ...
    "acceleration_deg_s2", acceleration_deg_s2(knotIndex, :));
end

function derivative = derivativeCoefficients(coefficients, order)
%% Section 0: Header & Readme
% SYNTAX
%   derivative = derivativeCoefficients(coefficients, order)
%**************************************************************************
% PURPOSE
%   - Differentiate an ascending-power coefficient vector.
%**************************************************************************
% INPUTS
%   - coefficients (numeric vector)
%   - order (nonnegative integer)
%**************************************************************************
% OUTPUTS
%   - derivative (numeric column vector)
%**************************************************************************
% UNITS
%   - Units follow the requested derivative order.

derivative = coefficients(:);
for derivativeIndex = 1:order
    if isscalar(derivative)
        derivative = 0;
        return;
    end
    powers = (1:(numel(derivative) - 1)).';
    derivative = derivative(2:end) .* powers;
end
end

function roots_s = polynomialRootsInside(coefficients, duration_s)
%% Section 0: Header & Readme
% SYNTAX
%   roots_s = polynomialRootsInside(coefficients, duration_s)
%**************************************************************************
% PURPOSE
%   - Return numerically real polynomial roots inside one segment.
%**************************************************************************
% INPUTS
%   - coefficients (ascending-power numeric vector)
%   - duration_s (positive scalar)
%**************************************************************************
% OUTPUTS
%   - roots_s (numeric column vector)
%**************************************************************************
% UNITS
%   - Roots are seconds.

coefficients = coefficients(:);
scale = max(1, max(abs(coefficients)));
while numel(coefficients) > 1 && ...
        abs(coefficients(end)) <= 64 * eps(scale)
    coefficients(end) = [];
end
if numel(coefficients) <= 1
    roots_s = zeros(0, 1);
    return;
end
candidateRoots_s = roots(flipud(coefficients));
isReal = abs(imag(candidateRoots_s)) <= ...
    1e-10 .* max(1, abs(real(candidateRoots_s)));
candidateRoots_s = real(candidateRoots_s(isReal));
timeTolerance_s = 64 .* eps(max(1, duration_s));
isInside = candidateRoots_s > timeTolerance_s & ...
    candidateRoots_s < duration_s - timeTolerance_s;
roots_s = unique(candidateRoots_s(isInside));
end

function value = evaluateAscendingPolynomial(coefficients, localTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   value = evaluateAscendingPolynomial(coefficients, localTime_s)
%**************************************************************************
% PURPOSE
%   - Evaluate an ascending-power polynomial at vector query times.
%**************************************************************************
% INPUTS
%   - coefficients (numeric vector)
%   - localTime_s (numeric vector)
%**************************************************************************
% OUTPUTS
%   - value (numeric column vector)
%**************************************************************************
% UNITS
%   - Units follow the supplied coefficients.

coefficients = coefficients(:);
localTime_s = localTime_s(:);
value = zeros(size(localTime_s));
for coefficientIndex = numel(coefficients):-1:1
    value = value .* localTime_s + coefficients(coefficientIndex);
end
end
