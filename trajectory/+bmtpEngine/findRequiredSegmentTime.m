function segmentTime_s = findRequiredSegmentTime(controlPoint_units, limits)
%% Section 0: Header & Readme
% SYNTAX
%   segmentTime_s = bmtpEngine.findRequiredSegmentTime(controlPoint_units, limits)
%**************************************************************************
% PURPOSE
%   - Find per-segment durations that satisfy derivative control bounds.
%**************************************************************************
% INPUTS
%   - controlPoint_units (S-by-(D+1)-by-2 numeric array)
%       Bezier controls for S degree-D curve segments.
%   - limits (scalar struct)
%       Per-axis maximum velocity, acceleration, and jerk.
%**************************************************************************
% OUTPUTS
%   - segmentTime_s (positive numeric column)
%       Smallest segment durations implied by exact derivative controls.
%**************************************************************************
% UNITS
%   - Controls are coordinate units and segmentTime_s is seconds.
%**************************************************************************

%% Section 1: Bound Every Derivative Order
degree           = size(controlPoint_units, 2) - 1;
segmentCount     = size(controlPoint_units, 1);
derivativeLimits = [limits.maxVelocity_units_s; ...
    limits.maxAcceleration_units_s2; ...
    limits.maxJerk_units_s3];

segmentTime_s = zeros(segmentCount, 1);
for derivativeOrder = 1:3
    % Exact Bezier hodograph controls bound the derivative on the whole span.
    derivativeScale         = factorial(degree) / factorial(degree - derivativeOrder);
    derivativeControl_units = derivativeScale * diff(controlPoint_units, derivativeOrder, 2);
    peakDerivative_units    = reshape(max(abs(derivativeControl_units), [], 2), segmentCount, 2);
    requiredTime_s          = (peakDerivative_units ./ derivativeLimits(derivativeOrder, :)) .^ ...
        (1 / derivativeOrder);
    segmentTime_s = max(segmentTime_s, max(requiredTime_s, [], 2));
end
segmentTime_s = max(segmentTime_s, eps);
end
