function segmentTime_s = findRequiredSegmentTime(controlPoint_units, limits)
%% Section 0: Header & Readme
% SYNTAX
%   segmentTime_s = bmtpEngine.motion.findRequiredSegmentTime(controlPoint_units, limits)
%**************************************************************************
% PURPOSE
%   - Give each Bezier segment enough time for its per-axis velocity,
%     acceleration, and jerk limits. Use bounds from the control points;
%     these are safe but can be higher than the curve's actual peaks.
%**************************************************************************
% INPUTS
%   - controlPoint_units (S-by-(D+1)-by-2 numeric array)
%       S is the number of segments. Each degree-D segment has D+1 control
%       points on two position axes; D must be at least 3.
%   - limits (scalar struct)
%       maxVelocity_units_s, maxAcceleration_units_s2, and
%       maxJerk_units_s3, each a positive 1-by-2 row in the same axis order.
%**************************************************************************
% OUTPUTS
%   - segmentTime_s (S-by-1 positive numeric column)
%       Duration that satisfies all control-based bounds. It may exceed
%       the time needed by the exact curve; stationary segments get eps seconds.
%**************************************************************************
% UNITS
%   - Controls use coordinate units, and segmentTime_s is seconds. Velocity,
%     acceleration, and jerk limits use units/s, units/s^2, and units/s^3.
%**************************************************************************

%% Section 1: Bound Motion Rates And Size Each Segment

degree           = size(controlPoint_units, 2) - 1;
segmentCount     = size(controlPoint_units, 1);
motionRateLimits = [limits.maxVelocity_units_s; ...
    limits.maxAcceleration_units_s2; ...
    limits.maxJerk_units_s3];

segmentTime_s = zeros(segmentCount, 1);
for derivativeOrder = 1:3
    % Repeated control-point differences describe each derivative curve.
    % A Bezier curve stays between its controls on each axis, so their
    % largest absolute value is a safe bound. For quadratic derivative
    % controls [0, 2, 0], that bound is 2 although the curve peaks at 1.
    derivativeFactor = factorial(degree) / factorial(degree - derivativeOrder);
    derivativeControlValues_units = derivativeFactor * diff(controlPoint_units, derivativeOrder, 2);
    maximumDerivativeControl_units = reshape( ...
        max(abs(derivativeControlValues_units), [], 2), segmentCount, 2);
    % A time derivative of order k divides by duration T^k. Choose
    % T >= (control bound / limit)^(1/k). If the second derivative with
    % respect to u is bounded by 9 units and the acceleration limit is
    % 1 unit/s^2, T must be at least 3 seconds.
    requiredTime_s = (maximumDerivativeControl_units ./ motionRateLimits(derivativeOrder, :)) .^ ...
        (1 / derivativeOrder);
    % Keep the largest requirement because every axis and rate must pass.
    segmentTime_s = max(segmentTime_s, max(requiredTime_s, [], 2));
end

% A stationary segment needs zero time under these limits, but later
% calculations divide by duration. Keep an eps-second positive floor.
segmentTime_s = max(segmentTime_s, eps);
end
