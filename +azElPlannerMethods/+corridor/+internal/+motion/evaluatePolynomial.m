function [time_s, position_deg, velocity_deg_s, ...
        acceleration_deg_s2, jerk_deg_s3] = evaluatePolynomial(polynomial, time_s, segmentIndex)
%% Section 0: Header & Readme
% SYNTAX
%   [time_s, position_deg, velocity_deg_s, acceleration_deg_s2, ...
%       jerk_deg_s3] = azElPlannerMethods.corridor.internal.motion.evaluatePolynomial( ...
%       polynomial, time_s, segmentIndex)
%**************************************************************************
% PURPOSE
%   - Evaluate exact segment polynomials at absolute trajectory times.
%**************************************************************************
% INPUTS
%   - polynomial (scalar struct), planner polynomial coefficient schema.
%   - time_s (numeric vector), absolute query times.
%   - segmentIndex (numeric vector, optional), preselected segment indices.
%**************************************************************************
% OUTPUTS
%   - time_s (column vector), normalized query orientation.
%   - position_deg through jerk_deg_s3 (N-by-2 arrays), motion histories.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s powers.
%**************************************************************************

%% Section 1: Select Polynomial Segments

% Absolute query times first map to a segment. A caller may supply segmentIndex
% when it already performed that lookup during a dense validation loop.
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

% Convert absolute time to normalized local tau. Clamping protects endpoint
% roundoff; it does not extend the motion beyond its first or final state.
if isscalar(polynomial.SegmentDuration_s)
    selectedDuration_s = polynomial.SegmentDuration_s;
else
    selectedDuration_s = polynomial.SegmentDuration_s(segmentIndex);
end
localTau = (time_s - polynomial.SegmentStartTime_s(segmentIndex)) ./ selectedDuration_s;
localTau = min(1, max(0, localTau));
if nargout < 2
    return;
end
position_deg = evaluateRecords( polynomial.positionPower_deg, segmentIndex, localTau);
if nargout < 3
    return;
end
velocity_deg_s = evaluateRecords( polynomial.velocityPower_deg_s, segmentIndex, localTau);
if nargout < 4
    return;
end
acceleration_deg_s2 = evaluateRecords( polynomial.accelerationPower_deg_s2, segmentIndex, localTau);
if nargout < 5
    return;
end
jerk_deg_s3 = evaluateRecords( polynomial.jerkPower_deg_s3, segmentIndex, localTau);
end


function value = evaluateRecords(coefficientArray, segmentIndex, localTau)
% Evaluate selected two-axis records without per-sample helper calls.
coefficientCount = size(coefficientArray, 3);
power = reshape(localTau .^ (0:coefficientCount - 1), [], 1, coefficientCount);
value = sum(coefficientArray(segmentIndex, :, :) .* power, 3);
end
