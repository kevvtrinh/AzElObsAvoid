function [time_s, position_units, velocity_units_s, acceleration_units_s2, jerk_units_s3] = ...
    evaluatePolynomial(polynomial, time_s, sampleSegmentIndices)
%% Section 0: Header & Readme
% SYNTAX
%   [time_s, position_units, velocity_units_s, acceleration_units_s2, ...
%       jerk_units_s3] = bmtpEngine.motion.evaluatePolynomial(polynomial, time_s)
%   [time_s, position_units, velocity_units_s, acceleration_units_s2, ...
%       jerk_units_s3] = bmtpEngine.motion.evaluatePolynomial( ...
%       polynomial, time_s, sampleSegmentIndices)
%**************************************************************************
% PURPOSE
%   - Sample position, velocity, acceleration, and jerk from the stored
%     motion polynomials at absolute times. Times outside the selected
%     segment use its nearest endpoint state.
%**************************************************************************
% INPUTS
%   - polynomial (scalar struct)
%       Coefficients and timing returned by createPowerPolynomial.
%   - time_s (numeric vector)
%       Absolute evaluation times.
%   - sampleSegmentIndices (numeric scalar or vector, optional; default [])
%       Segment to use for each time. A scalar uses that segment for all times.
%       Empty selects from the stored segment start times.
%**************************************************************************
% OUTPUTS
%   - time_s (numeric column)
%       Supplied times as a double column, in their original order.
%   - position_units (N-by-C numeric array)
%       Evaluated positions for every coordinate.
%   - velocity_units_s (N-by-C numeric array)
%       Evaluated velocities for every coordinate.
%   - acceleration_units_s2 (N-by-C numeric array)
%       Evaluated accelerations for every coordinate.
%   - jerk_units_s3 (N-by-C numeric array)
%       Evaluated jerks for every coordinate. Any nonfinite time makes all
%       returned motion values NaN. Empty times return empty histories.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Prepare Sample Arrays And Select A Segment For Each Time

time_s                = double(time_s(:));
sampleCount           = numel(time_s);
coordinateCount       = size(polynomial.positionPower_units, 2);
position_units        = NaN(sampleCount, coordinateCount);
velocity_units_s      = position_units;
acceleration_units_s2 = position_units;
jerk_units_s3         = position_units;
if nargout < 2 || isempty(time_s) || any(~isfinite(time_s))
    return
end

% By default, a time exactly at a join uses the following segment. The
% caller can select a segment explicitly to compare both sides of a join.
if nargin < 3 || isempty(sampleSegmentIndices)
    segmentStartTime_s   = double(polynomial.SegmentStartTime_s(:));
    sampleSegmentIndices = discretize(time_s, [-Inf; segmentStartTime_s(2:end); Inf]);
else
    sampleSegmentIndices = double(sampleSegmentIndices(:));
    if isscalar(sampleSegmentIndices)
        sampleSegmentIndices = repmat(sampleSegmentIndices, sampleCount, 1);
    end
end

%% Section 2: Convert Time To Segment Fraction And Evaluate The Motion

if isscalar(polynomial.SegmentDuration_s)
    selectedSegmentTime_s = polynomial.SegmentDuration_s;
else
    selectedSegmentTime_s = polynomial.SegmentDuration_s(sampleSegmentIndices);
end

% u = (requested time - segment start) / segment duration.
% For start = 2 s and duration = 4 s, time = 3 s gives u = 0.25.
segmentFractions = (time_s - polynomial.SegmentStartTime_s(sampleSegmentIndices)) ./ selectedSegmentTime_s;
% Clamp the evaluated fraction to [0 1]; retain the supplied output times.
segmentFractions = min(1, max(0, segmentFractions));

position_units = evaluateSegmentCoefficients(polynomial.positionPower_units, sampleSegmentIndices, segmentFractions);
if nargout >= 3
    velocity_units_s = evaluateSegmentCoefficients(polynomial.velocityPower_units_s, sampleSegmentIndices, segmentFractions);
end
if nargout >= 4
    acceleration_units_s2 = evaluateSegmentCoefficients( ...
        polynomial.accelerationPower_units_s2, sampleSegmentIndices, segmentFractions);
end
if nargout >= 5
    jerk_units_s3 = evaluateSegmentCoefficients(polynomial.jerkPower_units_s3, sampleSegmentIndices, segmentFractions);
end
end

%% Section 3: Local Functions

function sampledValues = evaluateSegmentCoefficients(polynomialCoefficients, sampleSegmentIndices, segmentFractions)
    % Evaluate c0 + c1 x u + c2 x u^2 + ... for every requested coordinate.
    % The coefficient fields already include physical time scaling for
    % velocity, acceleration, and jerk, so no further duration division is needed.
    coefficientCount = size(polynomialCoefficients, 3);
    powerTerms       = reshape(segmentFractions .^ (0:coefficientCount - 1), [], 1, coefficientCount);
    sampledValues    = sum(polynomialCoefficients(sampleSegmentIndices, :, :) .* powerTerms, 3);
end
