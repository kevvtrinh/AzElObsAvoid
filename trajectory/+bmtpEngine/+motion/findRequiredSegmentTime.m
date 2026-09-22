function segmentTime_s = findRequiredSegmentTime(controlPoint_units, limits)
%% Section 0: Header & Readme
% SYNTAX
%   segmentTime_s = bmtpEngine.motion.findRequiredSegmentTime(controlPoint_units, limits)
%**************************************************************************
% PURPOSE
%   - Choose a duration for each segment that keeps the Bezier control
%     bounds for speed, acceleration, and jerk within their axis limits.
%**************************************************************************
% INPUTS
%   - controlPoint_units (S-by-(D+1)-by-2 numeric array)
%       Bezier controls for S degree-D curve segments.
%   - limits (scalar struct)
%       Per-axis maximum velocity, acceleration, and jerk.
%**************************************************************************
% OUTPUTS
%   - segmentTime_s (positive numeric column)
%       Duration required by the control bounds. This can exceed the
%       shortest duration the curve itself needs, because its peak rate
%       can be below the largest control value.
%**************************************************************************
% UNITS
%   - Controls are coordinate units and segmentTime_s is seconds.
%**************************************************************************

%% Section 1: Size Each Segment For Speed, Acceleration, And Jerk

degree           = size(controlPoint_units, 2) - 1;
segmentCount     = size(controlPoint_units, 1);
motionRateLimits = [limits.maxVelocity_units_s; ...
    limits.maxAcceleration_units_s2; ...
    limits.maxJerk_units_s3];

segmentTime_s = zeros(segmentCount, 1);
for derivativeOrder = 1:3
    % Differencing Bezier controls gives another Bezier curve for each
    % derivative. That curve stays between its smallest and largest controls,
    % so the largest absolute control bounds every value within the segment.
    derivativeFactor = factorial(degree) / factorial(degree - derivativeOrder);
    derivativeControlValues_units = derivativeFactor * diff(controlPoint_units, derivativeOrder, 2);
    maximumDerivativeControl_units = reshape( ...
        max(abs(derivativeControlValues_units), [], 2), segmentCount, 2);
    % Duration T scales velocity by 1/T, acceleration by 1/T^2, and jerk
    % by 1/T^3. Solve T >= (control bound / limit)^(1 / derivative order).
    requiredTime_s = (maximumDerivativeControl_units ./ motionRateLimits(derivativeOrder, :)) .^ ...
        (1 / derivativeOrder);
    segmentTime_s = max(segmentTime_s, max(requiredTime_s, [], 2));
end

% Keep stationary segments positive in duration for later time divisions.
segmentTime_s = max(segmentTime_s, eps);
end
