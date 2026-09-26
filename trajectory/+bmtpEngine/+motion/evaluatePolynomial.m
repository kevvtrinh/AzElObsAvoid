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
%   - Evaluate the stored motion at requested absolute times. Return
%     position and, when requested, its first three time derivatives.
%     A time outside its selected segment uses that segment's nearest end.
%**************************************************************************
% INPUTS
%   - polynomial (scalar struct)
%       Segment start times, durations, and power coefficients returned by
%       createPowerPolynomial. Coefficients are ordered constant, linear,
%       quadratic, and so on in each segment's 0-to-1 fraction.
%   - time_s (numeric vector)
%       Absolute evaluation times in seconds; output follows this order.
%   - sampleSegmentIndices (numeric scalar or vector, optional; default [])
%       One-based segment number for each time. A scalar selects the same
%       segment for all times; a vector must match time_s in length. Empty
%       selects by the stored start times. At a join, this uses the later
%       segment; pass an index to examine the earlier segment instead.
%**************************************************************************
% OUTPUTS
%   - time_s (numeric column)
%       Supplied times as a double column, in their original order.
%   - position_units (N-by-C numeric array)
%       Position at N times on C coordinate axes.
%   - velocity_units_s (N-by-C numeric array)
%       Velocity at those times, when requested.
%   - acceleration_units_s2 (N-by-C numeric array)
%       Acceleration at those times, when requested.
%   - jerk_units_s3 (N-by-C numeric array)
%       Jerk at those times, when requested. If any time is nonfinite, all
%       motion outputs remain NaN. Empty times return empty histories.
%**************************************************************************
% UNITS
%   - Position: coordinate units; velocity: units/s; acceleration: units/s^2;
%     jerk: units/s^3; time: seconds.
%**************************************************************************

%% Section 1: Prepare Sample Arrays And Select A Segment For Each Time

time_s                = double(time_s(:));
sampleCount           = numel(time_s);
coordinateCount       = size(polynomial.positionPower_units, 2);
position_units        = NaN(sampleCount, coordinateCount);
velocity_units_s      = position_units;
acceleration_units_s2 = position_units;
jerk_units_s3         = position_units;
% One bad time stops the entire batch; partially filled histories could be
% mistaken for a complete motion. With only the time output, no evaluation
% is needed.
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
% Clamp only the fraction, not the returned time. For a segment from 2 to
% 6 s, a 7 s request returns time 7 s with the segment's state at 6 s.
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
    % Evaluate c0 + c1 x u + c2 x u^2 + ... on every coordinate axis.
    % Each derivative coefficient already includes its segment duration,
    % so velocity, acceleration, and jerk need no further time scaling.
    coefficientCount = size(polynomialCoefficients, 3);
    powerTerms       = reshape(segmentFractions .^ (0:coefficientCount - 1), [], 1, coefficientCount);
    sampledValues    = sum(polynomialCoefficients(sampleSegmentIndices, :, :) .* powerTerms, 3);
end
