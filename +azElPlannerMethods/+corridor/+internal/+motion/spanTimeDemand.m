function timeDemand = spanTimeDemand(polynomial, limits)
%% Section 0: Header & Readme
% SYNTAX
%   timeDemand = azElPlannerMethods.corridor.internal.motion.spanTimeDemand( ...
%       polynomial, limits)
%**************************************************************************
% PURPOSE
%   - Return the derivative-driven local time dilation for every span.
%**************************************************************************
% INPUTS
%   - polynomial (scalar struct), physical-time derivative coefficients.
%   - limits (scalar struct), two-axis velocity, acceleration, and jerk limits.
%**************************************************************************
% OUTPUTS
%   - timeDemand (N-by-1 vector), required local duration scale per span.
%**************************************************************************
% UNITS
%   - Derivatives use degrees and seconds; timeDemand is dimensionless.
%**************************************************************************
normalizedTime = linspace(0, 1, 33).';
derivativeArrays = {polynomial.velocityPower_deg_s, ...
    polynomial.accelerationPower_deg_s2, polynomial.jerkPower_deg_s3};
derivativeLimits = {limits.maxVelocity_deg_s, ...
    limits.maxAcceleration_deg_s2, limits.maxJerk_deg_s3};
spanCount = polynomial.SegmentCount;
timeDemand = zeros(spanCount, 1);
for derivativeOrder = 1:3
    powerArray = derivativeArrays{derivativeOrder};
    basis = normalizedTime.^(0:size(powerArray, 3) - 1);
    for spanIndex = 1:spanCount
        coefficient = reshape(powerArray(spanIndex, :, :), 2, []).';
        derivativeValue = basis * coefficient;
        utilization = max(abs(derivativeValue) ./ ...
            derivativeLimits{derivativeOrder}, [], "all");
        timeDemand(spanIndex) = max(timeDemand(spanIndex), ...
            utilization^(1 / derivativeOrder));
    end
end
end
