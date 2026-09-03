function [time_s, position_deg, velocity_deg_s, ...
        acceleration_deg_s2, jerk_deg_s3] = evaluatePolynomial( ...
        polynomial, time_s, segmentIndex)
%% Section 0: Header & Readme
% SYNTAX
%   [time_s, position_deg, velocity_deg_s, acceleration_deg_s2, ...
%       jerk_deg_s3] = bmtpEngine.evaluatePolynomial(polynomial, time_s)
%   [time_s, position_deg, velocity_deg_s, acceleration_deg_s2, ...
%       jerk_deg_s3] = bmtpEngine.evaluatePolynomial( ...
%       polynomial, time_s, segmentIndex)
%**************************************************************************
% PURPOSE
%   - Evaluate shared ascending-power segment records at absolute times.
%**************************************************************************
% INPUTS
%   - polynomial (scalar normalized trajectory polynomial struct)
%       Coefficient arrays use N-by-D-by-P shape and ascending powers.
%   - time_s (numeric vector)
%       Absolute evaluation times. The output uses a numeric column.
%   - segmentIndex (numeric scalar or vector, optional; default [])
%       Select an exact segment for each time. Empty values select segments
%       from polynomial start-time records. A scalar applies to all times.
%**************************************************************************
% OUTPUTS
%   - time_s (N-by-1 numeric column)
%       Normalized requested times.
%   - position_deg through jerk_deg_s3 (N-by-D numeric arrays)
%       Evaluated motion histories for every modeled coordinate.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s powers.
%**************************************************************************

%% Section 1: Select Polynomial Segments

time_s = double(time_s(:));
sampleCount = numel(time_s);
dimensionCount = size(polynomial.positionPower_deg, 2);
position_deg = NaN(sampleCount, dimensionCount);
velocity_deg_s = position_deg;
acceleration_deg_s2 = position_deg;
jerk_deg_s3 = position_deg;
if isempty(time_s) || any(~isfinite(time_s))
    return;
end
if nargin < 3 || isempty(segmentIndex)
    segmentStarts_s = double(polynomial.SegmentStartTime_s(:));
    if isscalar(segmentStarts_s)
        segmentIndex = ones(sampleCount, 1);
    else
        segmentIndex = sum(time_s >= segmentStarts_s(2:end).', 2) + 1;
    end
    segmentIndex = min(polynomial.SegmentCount, max(1, segmentIndex));
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
localTau = (time_s - polynomial.SegmentStartTime_s(segmentIndex)) ./ ...
    selectedDuration_s;
localTau = min(1, max(0, localTau));
position_deg = evaluateRecords( ...
    polynomial.positionPower_deg, segmentIndex, localTau);
velocity_deg_s = evaluateRecords( ...
    polynomial.velocityPower_deg_s, segmentIndex, localTau);
acceleration_deg_s2 = evaluateRecords( ...
    polynomial.accelerationPower_deg_s2, segmentIndex, localTau);
jerk_deg_s3 = evaluateRecords( ...
    polynomial.jerkPower_deg_s3, segmentIndex, localTau);
end

%% Section 3: Local Functions

function value = evaluateRecords(coefficientArray, segmentIndex, localTau)
% Evaluate selected coordinate records with ascending local-time powers.
coefficientCount = size(coefficientArray, 3);
power = reshape(localTau .^ (0:coefficientCount - 1), ...
    [], 1, coefficientCount);
value = sum(coefficientArray(segmentIndex, :, :) .* power, 3);
end
