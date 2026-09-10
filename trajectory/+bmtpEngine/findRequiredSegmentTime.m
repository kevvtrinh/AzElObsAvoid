function segmentTime_s = findRequiredSegmentTime(controlPoint_units, limits)
%% Section 0: Header & Readme
% SYNTAX: segmentTime_s = bmtpEngine.findRequiredSegmentTime( controlPoint_units, limits)
% PURPOSE: Find per-segment times that satisfies derivative control bounds.
% INPUTS: controlPoint_units (S-by-(D+1)-by-2 finite numeric array) Bezier controls for S degree-D
%   curve segments. limits (scalar struct) Per-axis maximum velocity, acceleration, and jerk.
% OUTPUTS: segmentTime_s (positive finite scalar) Smallest per-segment durations implied by exact
%   derivative controls.
% UNITS: Controls are coordinate units and segmentTime_s is seconds.

%% Section 1: Bound Every Derivative Order
degree      = size(controlPoint_units, 2) - 1;
limitValues = [limits.maxVelocity_units_s; ...
    limits.maxAcceleration_units_s2; limits.maxJerk_units_s3];
segmentCount = size(controlPoint_units, 1);
segmentTime_s = zeros(segmentCount, 1);
for derivativeOrder = 1:3
    scale         = factorial(degree) / factorial(degree - derivativeOrder);
    peak = reshape(max(abs(scale * diff(controlPoint_units, derivativeOrder, 2)), [], 2), segmentCount, 2);
    segmentTime_s = max(segmentTime_s, max((peak ./ limitValues(derivativeOrder, :)) .^ (1 / derivativeOrder), [], 2));
end
segmentTime_s = max(segmentTime_s, eps);
end
