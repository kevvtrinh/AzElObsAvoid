function segmentTime_s = findRequiredSegmentTime(controlPoint_deg, limits)
%% Section 0: Header & Readme
% SYNTAX
%   segmentTime_s = bmtpEngine.findRequiredSegmentTime( ...
%       controlPoint_deg, limits)
%**************************************************************************
% PURPOSE
%   - Find one common segment time that satisfies derivative control bounds.
%**************************************************************************
% INPUTS
%   - controlPoint_deg (S-by-(D+1)-by-2 finite numeric array)
%       Bezier controls for S equal-duration degree-D curve segments.
%   - limits (scalar struct)
%       Per-axis maximum velocity, acceleration, and jerk.
%**************************************************************************
% OUTPUTS
%   - segmentTime_s (positive finite scalar)
%       Smallest common segment duration implied by exact derivative controls.
%**************************************************************************
% UNITS
%   - Controls are degrees and segmentTime_s is seconds.
%**************************************************************************

%% Section 1: Bound Every Derivative Order

degree = size(controlPoint_deg, 2) - 1;
limitValues = [limits.maxVelocity_deg_s; ...
    limits.maxAcceleration_deg_s2; limits.maxJerk_deg_s3];
segmentTime_s = 0;
for derivativeOrder = 1:3
    scale = factorial(degree) / factorial(degree - derivativeOrder);
    peak = squeeze(max(abs(scale * diff( ...
        controlPoint_deg, derivativeOrder, 2)), [], [1 2]));
    segmentTime_s = max(segmentTime_s, max( ...
        (peak(:).' ./ limitValues(derivativeOrder, :)) .^ ...
        (1 / derivativeOrder)));
end
segmentTime_s = max(segmentTime_s, eps);
end
