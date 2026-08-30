function [warmStart, diagnostics] = convertPolynomialToWarmStart( ...
        polynomial, segmentCount, degree)
%% Section 0: Header & Readme
% SYNTAX
%   [warmStart, diagnostics] = ...
%       bmtpEngine.convertPolynomialToWarmStart( ...
%       polynomial, segmentCount, degree)
%**************************************************************************
% PURPOSE
%   - Approximate a piecewise-polynomial motion with equal-duration
%     Bernstein spans for the BMTP warm-start interface.
%   - Measure position error without treating the approximation as an
%     accepted trajectory.
%**************************************************************************
% INPUTS
%   - polynomial (scalar shared polynomial struct)
%       Segment records use ascending local powers and absolute start times.
%       Position coefficients are S-by-D-by-P in degrees.
%   - segmentCount (positive integer scalar)
%       Number of equal-duration output spans.
%   - degree (positive integer scalar)
%       Bernstein degree fitted independently on every output span.
%**************************************************************************
% OUTPUTS
%   - warmStart (scalar struct)
%       ControlPoint_deg is segmentCount-by-(degree+1)-by-D and
%       SegmentTime_s is one positive scalar shared by every span.
%   - diagnostics (scalar struct)
%       Chebyshev sample counts, source breaks per span, and sampled maximum
%       Euclidean position deviations in degrees.
%**************************************************************************
% UNITS
%   - Position is degrees and absolute time is seconds. Polynomial powers
%     use each source segment's normalized local time.
%**************************************************************************

%% Section 1: Validate The Conversion Request

[initialTime_s, finalTime_s, sourceBreakTime_s, dimensionCount] = ...
    validateRequest(polynomial, segmentCount, degree);
segmentCount = double(segmentCount);
degree = double(degree);
duration_s = finalTime_s - initialTime_s;
segmentTime_s = duration_s / segmentCount;

% An overdetermined fit avoids making interpolation nodes look like an error
% certificate. The denser independent grid reports, but does not bound, error.
fitSampleCount = 2 * (degree + 1) + 1;
errorSampleCount = 8 * (degree + 1) + 1;
fitTau = chebyshevLobattoNodes(fitSampleCount);
errorTau = chebyshevLobattoNodes(errorSampleCount);
fitBasis = bernsteinBasis(fitTau, degree);

%% Section 2: Fit Equal-Duration Bernstein Spans

controlPoint_deg = zeros(segmentCount, degree + 1, dimensionCount);
spanMaximumDeviation_deg = zeros(segmentCount, 1);
interiorSourceBreakCount = zeros(segmentCount, 1);
spanStartTime_s = initialTime_s + (0:segmentCount - 1).' * segmentTime_s;
for segmentIndex = 1:segmentCount
    startTime_s = spanStartTime_s(segmentIndex);
    endTime_s = startTime_s + segmentTime_s;
    fitTime_s = startTime_s + fitTau * segmentTime_s;
    [~, fitPosition_deg] = ...
        bmtpEngine.evaluatePolynomial(polynomial, fitTime_s);
    spanControl_deg = fitBasis \ fitPosition_deg;
    controlPoint_deg(segmentIndex, :, :) = ...
        reshape(spanControl_deg, 1, degree + 1, dimensionCount);

    interiorBreak = sourceBreakTime_s > startTime_s & ...
        sourceBreakTime_s < endTime_s;
    interiorSourceBreakCount(segmentIndex) = nnz(interiorBreak);
    breakTau = (sourceBreakTime_s(interiorBreak) - startTime_s) / ...
        segmentTime_s;
    checkTau = unique([errorTau; breakTau]);
    checkTime_s = startTime_s + checkTau * segmentTime_s;
    [~, truePosition_deg] = ...
        bmtpEngine.evaluatePolynomial(polynomial, checkTime_s);
    fittedPosition_deg = bernsteinBasis(checkTau, degree) * ...
        spanControl_deg;
    deviation_deg = vecnorm(fittedPosition_deg - truePosition_deg, 2, 2);
    spanMaximumDeviation_deg(segmentIndex) = max(deviation_deg);
end

%% Section 3: Assemble The Warm Start And Error Record

warmStart = struct( ...
    "ControlPoint_deg", controlPoint_deg, ...
    "SegmentTime_s", segmentTime_s);
diagnostics = struct( ...
    "SourceDuration_s", duration_s, ...
    "SegmentCount", segmentCount, ...
    "Degree", degree, ...
    "FitSampleCount", fitSampleCount, ...
    "ErrorSampleCount", errorSampleCount, ...
    "InteriorSourceBreakCount", interiorSourceBreakCount, ...
    "SpanMaximumPositionDeviation_deg", spanMaximumDeviation_deg, ...
    "MaximumPositionDeviation_deg", max(spanMaximumDeviation_deg));
end

%% Section 4: Local Functions

function [initialTime_s, finalTime_s, breakTime_s, dimensionCount] = ...
        validateRequest(polynomial, segmentCount, degree)
% Protect evaluator assumptions before fitting or allocating output arrays.
requiredFields = {'SegmentCount', 'SegmentStartTime_s', ...
    'SegmentDuration_s', 'FinalTime_s', 'positionPower_deg', ...
    'velocityPower_deg_s', 'accelerationPower_deg_s2', ...
    'jerkPower_deg_s3'};
if ~isstruct(polynomial) || ~isscalar(polynomial) || ...
        ~all(isfield(polynomial, requiredFields))
    error("bmtpEngine:InvalidWarmStartPolynomial", ...
        "polynomial must be scalar and contain the shared segment fields.");
end
validateattributes(segmentCount, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'positive'});
validateattributes(degree, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'positive', '<=', 32});
validateattributes(polynomial.SegmentCount, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'positive'});
sourceSegmentCount = double(polynomial.SegmentCount);
startTime_s = double(polynomial.SegmentStartTime_s(:));
duration_s = double(polynomial.SegmentDuration_s(:));
if isscalar(duration_s)
    duration_s = repmat(duration_s, sourceSegmentCount, 1);
end
finalTime_s = double(polynomial.FinalTime_s);
timeRecordIsValid = numel(startTime_s) == sourceSegmentCount && ...
    numel(duration_s) == sourceSegmentCount && ...
    all(isfinite(startTime_s)) && all(isfinite(duration_s)) && ...
    all(duration_s > 0) && all(diff(startTime_s) > 0) && ...
    isscalar(finalTime_s) && isfinite(finalTime_s) && ...
    finalTime_s > startTime_s(1);
if ~timeRecordIsValid
    error("bmtpEngine:InvalidWarmStartPolynomialTime", ...
        "Polynomial segment times must be finite, positive, and increasing.");
end
expectedEndTime_s = [startTime_s(2:end); finalTime_s];
timeScale_s = max(1, max(abs([startTime_s; finalTime_s])));
timeTolerance_s = 256 * eps(timeScale_s);
if any(abs(startTime_s + duration_s - expectedEndTime_s) > timeTolerance_s)
    error("bmtpEngine:InvalidWarmStartPolynomialTime", ...
        "Segment durations must meet the next start or final time.");
end
coefficientFields = requiredFields(5:end);
dimensionCount = size(polynomial.positionPower_deg, 2);
for fieldIndex = 1:numel(coefficientFields)
    coefficient = polynomial.(coefficientFields{fieldIndex});
    coefficientIsValid = isnumeric(coefficient) && isreal(coefficient) && ...
        size(coefficient, 1) == sourceSegmentCount && ...
        size(coefficient, 2) == dimensionCount && ...
        all(isfinite(coefficient), "all");
    if ~coefficientIsValid
        error("bmtpEngine:InvalidWarmStartPolynomialCoefficient", ...
            "%s must be finite S-by-D-by-P coefficients.", ...
            coefficientFields{fieldIndex});
    end
end
if dimensionCount < 1
    error("bmtpEngine:InvalidWarmStartPolynomialCoefficient", ...
        "positionPower_deg must contain at least one coordinate.");
end
initialTime_s = startTime_s(1);
breakTime_s = startTime_s(2:end);
end

function tau = chebyshevLobattoNodes(sampleCount)
% Include both span endpoints while clustering samples near them.
angle_rad = pi * (0:sampleCount - 1).' / (sampleCount - 1);
tau = 0.5 * (1 - cos(angle_rad));
end

function basis = bernsteinBasis(tau, degree)
% Evaluate a complete degree-d Bernstein basis on normalized time.
tau = double(tau(:));
basis = zeros(numel(tau), degree + 1);
for bernsteinIndex = 0:degree
    basis(:, bernsteinIndex + 1) = nchoosek(degree, bernsteinIndex) * ...
        tau .^ bernsteinIndex .* (1 - tau) .^ ...
        (degree - bernsteinIndex);
end
end
