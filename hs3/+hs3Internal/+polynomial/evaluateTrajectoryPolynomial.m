function [time, position, velocity, acceleration, jerk] = ...
        evaluateTrajectoryPolynomial(polynomial, time, segmentIndex)
%% Section 0: Header & Readme
% SYNTAX
%   [time, position, velocity, acceleration, jerk] = ...
%       hs3Internal.polynomial.evaluateTrajectoryPolynomial(polynomial, time)
%   [time, position, velocity, acceleration, jerk] = ...
%       hs3Internal.polynomial.evaluateTrajectoryPolynomial(polynomial, time, segmentIndex)
%**************************************************************************
% PURPOSE
%   - Evaluate dimension-neutral ascending-power segment records.
%**************************************************************************
% INPUTS
%   - polynomial (scalar hs3 polynomial struct)
%       Coefficient arrays use segment-by-coordinate-by-power ordering.
%   - time (numeric vector), absolute evaluation times.
%   - segmentIndex (numeric scalar or vector, optional; default [])
%       This input selects exact segments. An empty input uses segment times.
%**************************************************************************
% OUTPUTS
%   - time (N-by-1 numeric column), normalized requested times.
%   - position, velocity, acceleration, jerk (N-by-D numeric arrays)
%**************************************************************************
% UNITS
%   - Time and coordinate units are caller-defined and must be consistent.
%**************************************************************************

%% Section 1: Select Polynomial Segments

% Returned rows always follow the requested time order. When the caller does
% not specify segments, each time is assigned by the latest segment start not
% exceeding it. Clamping assigns an out-of-range value to an endpoint segment.
% Nonfinite input intentionally returns the preallocated NaNs for diagnostics.

time = double(time(:));
sampleCount = numel(time);
dimensionCount = size(polynomial.positionPower, 2);
position = NaN(sampleCount, dimensionCount);
velocity = NaN(sampleCount, dimensionCount);
acceleration = NaN(sampleCount, dimensionCount);
jerk = NaN(sampleCount, dimensionCount);
if isempty(time) || any(~isfinite(time))
    return;
end
if nargin < 3 || isempty(segmentIndex)
    segmentStarts = double(polynomial.SegmentStartTime(:));
    if isscalar(segmentStarts)
        segmentIndex = ones(sampleCount, 1);
    else
        segmentIndex = sum(time >= segmentStarts(2:end).', 2) + 1;
    end
    segmentIndex = min(polynomial.SegmentCount, max(1, segmentIndex));
else
    segmentIndex = double(segmentIndex(:));
    if isscalar(segmentIndex)
        segmentIndex = repmat(segmentIndex, sampleCount, 1);
    end
end

%% Section 2: Evaluate Ascending-Power Records

% Convert absolute time to local tau. Clamping protects against tiny roundoff
% outside [0,1] at shared boundaries. Each derivative has its own coefficient
% count but uses the same segment and local coordinate.

if isscalar(polynomial.SegmentDuration)
    selectedDuration = polynomial.SegmentDuration;
else
    selectedDuration = polynomial.SegmentDuration(segmentIndex);
end
localTau = (time - polynomial.SegmentStartTime(segmentIndex)) ./ ...
    selectedDuration;
localTau = min(1, max(0, localTau));
if nargout < 2
    return;
end
position = evaluateRecords(polynomial.positionPower, segmentIndex, localTau);
if nargout < 3
    return;
end
velocity = evaluateRecords(polynomial.velocityPower, segmentIndex, localTau);
if nargout < 4
    return;
end
acceleration = evaluateRecords( ...
    polynomial.accelerationPower, segmentIndex, localTau);
if nargout < 5
    return;
end
jerk = evaluateRecords(polynomial.jerkPower, segmentIndex, localTau);
end

%% Section 3: Local Functions

function value = evaluateRecords(coefficientArray, segmentIndex, localTau)
% Evaluate selected coordinate records without per-sample dispatch.
% Broadcasting multiplies each selected coefficient by tau^power, and the sum
% over the third dimension evaluates all samples and coordinates together.
coefficientCount = size(coefficientArray, 3);
power = reshape(localTau .^ (0:coefficientCount - 1), ...
    [], 1, coefficientCount);
value = sum(coefficientArray(segmentIndex, :, :) .* power, 3);
end
