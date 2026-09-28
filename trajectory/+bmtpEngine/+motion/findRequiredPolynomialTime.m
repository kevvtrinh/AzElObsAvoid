function requiredTime_s = findRequiredPolynomialTime(positionPower_units, segmentTime_s, limits)
%% Section 0: Header & Readme
% SYNTAX
%   requiredTime_s = bmtpEngine.motion.findRequiredPolynomialTime( ...
%       positionPower_units, segmentTime_s, limits)
%**************************************************************************
% PURPOSE
%   - Find how long each fixed curve segment needs to stay within per-axis
%     velocity, acceleration, and jerk limits. Use the highest rates on the
%     polynomial itself, not estimates from its Bezier control points.
%   - Return required times without changing the curves or supplied times.
%**************************************************************************
% INPUTS
%   - positionPower_units (S-by-2-by-(D+1) finite real numeric array)
%       S is the number of segments; D is the curve degree. Each axis uses
%       coefficients for constant, u, u^2, and so on, where segment fraction
%       u runs from 0 at the start to 1 at the end.
%   - segmentTime_s (S-by-1 positive finite numeric vector)
%       Current duration of each segment.
%   - limits (scalar struct)
%       maxVelocity_units_s, maxAcceleration_units_s2, and
%       maxJerk_units_s3, each a positive 1-by-2 row in the same axis order.
%**************************************************************************
% OUTPUTS
%   - requiredTime_s (S-by-1 positive numeric vector)
%       Shortest duration allowed by all three rate limits on both axes for
%       the same curve. A stationary segment receives an eps-second floor.
%**************************************************************************
% UNITS
%   - Position uses coordinate units; time is seconds. Velocity, acceleration,
%     and jerk limits use units/s, units/s^2, and units/s^3.
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

% Differentiate the position curve once for velocity, twice for acceleration,
% and three times for jerk. Since u = elapsed time / segment duration T,
% the physical rate of order k divides the u-derivative by T^k.
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
            % A large negative rate can violate a limit just as a large
            % positive one can, so use the greater absolute peak.
            peakRate = max(abs(minimumRate), abs(maximumRate));
            % Stretching time by q divides an order-k rate by q^k. If jerk
            % is 8 times its limit, q = 2 because 2^3 = 8. The largest
            % required time across both axes and all rates wins.
            durationScale = (peakRate / motionRateLimits(derivativeOrder, axisIndex)) ^ (1 / derivativeOrder);
            requiredTime_s(segmentIndex) = max(requiredTime_s(segmentIndex), segmentDuration_s * durationScale);
        end
    end
end

% A stationary curve needs zero time under these rate limits, but later
% calculations divide by duration. Use eps seconds as a positive floor.
requiredTime_s = max(requiredTime_s, eps);
end
