function requiredTime_s = findRequiredPolynomialTime(positionPower_units, segmentTime_s, limits)
%% Section 0: Header & Readme
% SYNTAX
%   requiredTime_s = bmtpEngine.motion.findRequiredPolynomialTime( ...
%       positionPower_units, segmentTime_s, limits)
%**************************************************************************
% PURPOSE
%   - Size each segment from the exact speed, acceleration, and jerk peaks
%     of its position polynomial, rather than its Bezier control bounds.
%**************************************************************************
% INPUTS
%   - positionPower_units (S-by-2-by-(D+1) finite real numeric array)
%       Position coefficients in powers of segment fraction u from 0 to 1,
%       with the constant coefficient first.
%   - segmentTime_s (S-by-1 positive finite numeric vector)
%       Current physical duration of each polynomial segment.
%   - limits (scalar struct)
%       Positive per-axis maximum velocity, acceleration, and jerk.
%**************************************************************************
% OUTPUTS
%   - requiredTime_s (S-by-1 positive numeric vector)
%       Duration needed for the polynomial's exact motion-rate peaks.
%       A stationary segment receives an eps-second floor.
%**************************************************************************
% UNITS
%   - Position is coordinate units and durations are seconds. Duration T
%     scales velocity by 1/T, acceleration by 1/T^2, and jerk by 1/T^3.
%     A jerk peak 1 percent over its limit needs T x 1.01^(1/3).
%**************************************************************************

%% Section 1: Check Polynomial, Durations, And Limits

validateattributes(positionPower_units, {'numeric'}, {'nonempty', 'real', 'finite'}, ...
    mfilename, 'positionPower_units');
segmentCount = size(positionPower_units, 1);
assert(size(positionPower_units, 2) == 2, 'findRequiredPolynomialTime:InvalidPositionPower', ...
    'Position coefficients must have two axes in the second dimension.');
validateattributes(segmentTime_s, {'numeric'}, ...
    {'column', 'numel', segmentCount, 'real', 'finite', 'positive'}, mfilename, 'segmentTime_s');
requiredLimitFields = {'maxVelocity_units_s', 'maxAcceleration_units_s2', 'maxJerk_units_s3'};
assert(isstruct(limits) && isscalar(limits) && all(isfield(limits, requiredLimitFields)), ...
    'findRequiredPolynomialTime:InvalidLimits', ...
    'Limits must provide per-axis maximum velocity, acceleration, and jerk.');
motionRateLimits = [limits.maxVelocity_units_s; ...
    limits.maxAcceleration_units_s2; limits.maxJerk_units_s3];
validateattributes(motionRateLimits, {'numeric'}, ...
    {'size', [3, 2], 'real', 'finite', 'positive'}, mfilename, 'limits');

%% Section 2: Size Each Segment From Its Exact Polynomial Peaks

requiredTime_s = zeros(segmentCount, 1);
for segmentIndex = 1:segmentCount
    segmentDuration_s = double(segmentTime_s(segmentIndex));
    for axisIndex = 1:2
        derivativeCoefficients = reshape(double(positionPower_units(segmentIndex, axisIndex, :)), [], 1);
        for derivativeOrder = 1:3
            if numel(derivativeCoefficients) > 1
                derivativeCoefficients = (1:numel(derivativeCoefficients) - 1).' .* ...
                    derivativeCoefficients(2:end);
            else
                derivativeCoefficients = 0;
            end
            physicalRateCoefficients = derivativeCoefficients / segmentDuration_s ^ derivativeOrder;
            [minimumRate, maximumRate] = bmtpEngine.validation.boundPolynomialRange(physicalRateCoefficients);
            peakRate = max(abs(minimumRate), abs(maximumRate));
            durationScale = (peakRate / motionRateLimits(derivativeOrder, axisIndex)) ^ (1 / derivativeOrder);
            requiredTime_s(segmentIndex) = max(requiredTime_s(segmentIndex), segmentDuration_s * durationScale);
        end
    end
end

% A stationary curve needs no travel time, but later calculations divide
% by duration. Keep the same positive floor as control-based sizing.
requiredTime_s = max(requiredTime_s, eps);
end
