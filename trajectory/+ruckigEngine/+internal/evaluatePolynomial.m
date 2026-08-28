function [time, position, velocity, acceleration, jerk] = ...
        evaluatePolynomial(polynomial, time, segmentIndex)
%% Section 0: Header & Readme
% SYNTAX
%   [time, position, velocity, acceleration, jerk] = ...
%       ruckigEngine.internal.evaluatePolynomial(polynomial, time)
%   [time, position, velocity, acceleration, jerk] = ...
%       ruckigEngine.internal.evaluatePolynomial( ...
%       polynomial, time, segmentIndex)
%**************************************************************************
% PURPOSE
%   - Evaluate the engine-neutral ascending-power trajectory format.
%**************************************************************************
% INPUTS
%   - polynomial (scalar struct)
%       Coefficients use segment-by-coordinate-by-ascending-power ordering.
%   - time (numeric vector)
%       Absolute evaluation times.
%   - segmentIndex (numeric scalar or vector, optional; default [])
%       Empty input selects segments from their absolute start times.
%**************************************************************************
% OUTPUTS
%   - time (N-by-1 numeric column)
%   - position, velocity, acceleration, jerk (N-by-D numeric arrays)
%**************************************************************************
% UNITS
%   - Values retain the caller's consistent coordinate and time units.
%**************************************************************************

%% Section 1: Select Segments

time = double(time(:));
sampleCount = numel(time);
dimensionCount = size(polynomial.positionPower, 2);
position = NaN(sampleCount, dimensionCount);
velocity = position;
acceleration = position;
jerk = position;
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

%% Section 2: Evaluate Ascending Powers

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
position = evaluateRecords( ...
    polynomial.positionPower, segmentIndex, localTau);
if nargout < 3
    return;
end
velocity = evaluateRecords( ...
    polynomial.velocityPower, segmentIndex, localTau);
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
% Evaluate selected coordinate records together without sample dispatch.
coefficientCount = size(coefficientArray, 3);
power = reshape(localTau .^ (0:coefficientCount - 1), ...
    [], 1, coefficientCount);
value = sum(coefficientArray(segmentIndex, :, :) .* power, 3);
end
