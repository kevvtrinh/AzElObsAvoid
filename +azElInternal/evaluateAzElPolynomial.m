function [time_s, position_deg, velocity_deg_s, ...
        acceleration_deg_s2, jerk_deg_s3] = ...
        evaluateAzElPolynomial(polynomial, time_s, segmentIndex)
%% Section 0: Header & Readme
% SYNTAX
%   [time_s, position_deg, velocity_deg_s, acceleration_deg_s2, ...
%       jerk_deg_s3] = azElInternal.evaluateAzElPolynomial( ...
%       polynomial, time_s)
%   [time_s, position_deg, velocity_deg_s, acceleration_deg_s2, ...
%       jerk_deg_s3] = azElInternal.evaluateAzElPolynomial( ...
%       polynomial, time_s, segmentIndex)
%**************************************************************************
% PURPOSE
%   - Evaluate one normalized planner polynomial at requested times.
%**************************************************************************
% INPUTS
%   - polynomial (scalar normalized planner polynomial struct)
%       Coefficient arrays use ascending powers and N-by-2-by-P shape.
%   - time_s (numeric vector)
%       Absolute evaluation times. The output uses a numeric column.
%   - segmentIndex (numeric scalar or vector, optional; default [])
%       Select an exact segment for each time. Empty values select segments
%       from polynomial start-time records. A scalar applies to all times.
%**************************************************************************
% OUTPUTS
%   - time_s (N-by-1 numeric column)
%       Normalized requested times.
%   - position_deg, velocity_deg_s, acceleration_deg_s2, jerk_deg_s3
%       N-by-2 evaluated histories in [azimuth elevation] order.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s, deg/s^2,
%     and deg/s^3.
%**************************************************************************
%% Section 1: Select Polynomial Segments
time_s = double(time_s(:));
sampleCount = numel(time_s);
position_deg = NaN(sampleCount, 2);
velocity_deg_s = NaN(sampleCount, 2);
acceleration_deg_s2 = NaN(sampleCount, 2);
jerk_deg_s3 = NaN(sampleCount, 2);
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
for sampleIndex = 1:sampleCount
    selectedSegmentIndex = segmentIndex(sampleIndex);
    if isscalar(polynomial.SegmentDuration_s)
        selectedDuration_s = polynomial.SegmentDuration_s;
    else
        selectedDuration_s = polynomial.SegmentDuration_s(selectedSegmentIndex);
    end
    localTau = (time_s(sampleIndex) - polynomial.SegmentStartTime_s(selectedSegmentIndex)) / ...
        selectedDuration_s;
    localTau = min(1, max(0, localTau));
    position_deg(sampleIndex, :) = evaluateRecord( ...
        polynomial.positionPower_deg, selectedSegmentIndex, localTau);
    velocity_deg_s(sampleIndex, :) = evaluateRecord( ...
        polynomial.velocityPower_deg_s, selectedSegmentIndex, localTau);
    acceleration_deg_s2(sampleIndex, :) = evaluateRecord( ...
        polynomial.accelerationPower_deg_s2, ...
        selectedSegmentIndex, localTau);
    jerk_deg_s3(sampleIndex, :) = evaluateRecord( ...
        polynomial.jerkPower_deg_s3, selectedSegmentIndex, localTau);
end
end
%% Section 3: Local Functions
function value = evaluateRecord(coefficientArray, segmentIndex, localTau)
% PURPOSE
%   - Evaluate one two-axis ascending-power coefficient record.
coefficient = reshape(coefficientArray(segmentIndex, :, :), 2, []);
power = localTau .^ (0:size(coefficient, 2) - 1);
value = (coefficient * power.').';
end
