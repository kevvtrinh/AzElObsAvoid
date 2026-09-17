function [time_s, position_units, velocity_units_s, acceleration_units_s2, jerk_units_s3] = ...
        evaluatePolynomial(polynomial, time_s, segmentIndex)
%% Section 0: Header & Readme
% SYNTAX
%   [time_s, position_units, velocity_units_s, acceleration_units_s2, ...
%       jerk_units_s3] = bmtpEngine.motion.evaluatePolynomial(polynomial, time_s)
%   [time_s, position_units, velocity_units_s, acceleration_units_s2, ...
%       jerk_units_s3] = bmtpEngine.motion.evaluatePolynomial( ...
%       polynomial, time_s, segmentIndex)
%**************************************************************************
% PURPOSE
%   - Evaluate ascending-power segment records at absolute times.
%**************************************************************************
% INPUTS
%   - polynomial (scalar struct)
%       Trajectory polynomial with per-segment ascending-power records.
%   - time_s (numeric vector)
%       Absolute evaluation times.
%   - segmentIndex (numeric scalar or vector, optional; default [])
%       Explicit segment selection; empty selects from segment start times.
%**************************************************************************
% OUTPUTS
%   - time_s (numeric column)
%       Normalized requested times.
%   - position_units (N-by-C numeric array)
%       Evaluated positions for every coordinate.
%   - velocity_units_s (N-by-C numeric array)
%       Evaluated velocities for every coordinate.
%   - acceleration_units_s2 (N-by-C numeric array)
%       Evaluated accelerations for every coordinate.
%   - jerk_units_s3 (N-by-C numeric array)
%       Evaluated jerks for every coordinate. Nonfinite or empty requested
%       times return NaN histories rather than throwing.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Select Polynomial Segments
time_s                = double(time_s(:));
sampleCount           = numel(time_s);
dimensionCount        = size(polynomial.positionPower_units, 2);
position_units        = NaN(sampleCount, dimensionCount);
velocity_units_s      = position_units;
acceleration_units_s2 = position_units;
jerk_units_s3         = position_units;
if nargout < 2 || isempty(time_s) || any(~isfinite(time_s))
    return
end
if nargin < 3 || isempty(segmentIndex)
    segmentStarts_s = double(polynomial.SegmentStartTime_s(:));
    segmentIndex    = discretize(time_s, [-Inf; segmentStarts_s(2:end); Inf]);
else
    segmentIndex = double(segmentIndex(:));
    if isscalar(segmentIndex)
        segmentIndex = repmat(segmentIndex, sampleCount, 1);
    end
end

%% Section 2: Evaluate Ascending-Power Records
if isscalar(polynomial.SegmentDuration_s)
    selectedDuration_s = polynomial.SegmentDuration_s;
else
    selectedDuration_s = polynomial.SegmentDuration_s(segmentIndex);
end
localTau = (time_s - polynomial.SegmentStartTime_s(segmentIndex)) ./ selectedDuration_s;
localTau = min(1, max(0, localTau));

position_units = evaluateRecords(polynomial.positionPower_units, segmentIndex, localTau);
if nargout >= 3
    velocity_units_s = evaluateRecords(polynomial.velocityPower_units_s, segmentIndex, localTau);
end
if nargout >= 4
    acceleration_units_s2 = evaluateRecords(polynomial.accelerationPower_units_s2, segmentIndex, localTau);
end
if nargout >= 5
    jerk_units_s3 = evaluateRecords(polynomial.jerkPower_units_s3, segmentIndex, localTau);
end
end

%% Section 3: Local Functions
function values = evaluateRecords(coefficientArray, segmentIndex, localTau)
    % Evaluate the selected polynomial segments in local time.
    coefficientCount = size(coefficientArray, 3);
    powerTerms       = reshape(localTau .^ (0:coefficientCount - 1), [], 1, coefficientCount);
    values           = sum(coefficientArray(segmentIndex, :, :) .* powerTerms, 3);
end
