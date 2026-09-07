function polynomial = createPowerPolynomial(controlPoint_units, segmentTime_s, initialTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   polynomial = bmtpEngine.createPowerPolynomial( ...
%       controlPoint_units, segmentTime_s, initialTime_s)
%
% PURPOSE
%   - Convert composite Bernstein control points to the stable ascending-power
%     polynomial representation and stabilize exact endpoint derivatives.
%
% INPUTS
%   - controlPoint_units (S-by-(D+1)-by-2 numeric array)
%       Composite Bezier control points.
%   - segmentTime_s (positive numeric scalar)
%       Common physical segment duration.
%   - initialTime_s (finite numeric scalar)
%       Absolute motion start time.
%
% OUTPUTS
%   - polynomial (scalar struct)
%       Position, derivative powers, segment times, and terminal state.
%
% UNITS
%   - Position is coordinate units; time is seconds; derivatives use units/s,
%     units/s^2, and units/s^3.
%

%% Section 1: Convert Bernstein Controls To Powers

segmentCount = size(controlPoint_units, 1);
degree       = size(controlPoint_units, 2) - 1;
[powerIndex, bernsteinIndex] = ndgrid(0:degree);
valid      = bernsteinIndex <= powerIndex;
conversion = zeros(degree + 1);
conversion(valid) = factorial(degree) * (-1) .^ (powerIndex(valid) - bernsteinIndex(valid)) ./ (factorial(bernsteinIndex(valid)) .* factorial(powerIndex(valid) - bernsteinIndex(valid)) .* factorial(degree - powerIndex(valid)));
bernsteinPages    = permute(controlPoint_units, [2 1 3]);
positionPower_units = permute(pagemtimes(conversion, bernsteinPages), [2 3 1]);
positionPower_units = stabilizePolynomialEndpoints(positionPower_units, controlPoint_units);

%% Section 2: Create Physical Derivative Powers And Timing

velocityPower_units_s      = positionPower_units(:, :, 2:end) .* reshape(1:degree, 1, 1, []) / segmentTime_s;
accelerationPower_units_s2 = velocityPower_units_s(:, :, 2:end) .* reshape(1:degree - 1, 1, 1, []) / segmentTime_s;
jerkPower_units_s3         = accelerationPower_units_s2(:, :, 2:end) .* reshape(1:degree - 2, 1, 1, []) / segmentTime_s;
segmentStartTime_s       = initialTime_s + (0:segmentCount - 1).' * segmentTime_s;
polynomial               = struct("Degree", degree, "SegmentCount", segmentCount, ...
    "SegmentStartTime_s", segmentStartTime_s, ...
    "SegmentDuration_s", repmat(segmentTime_s, segmentCount, 1), ...
    "SegmentBreakTau", (0:segmentCount).' / segmentCount, ...
    "FinalTime_s", initialTime_s + segmentCount * segmentTime_s, ...
    "positionPower_units", positionPower_units, ...
    "velocityPower_units_s", velocityPower_units_s, ...
    "accelerationPower_units_s2", accelerationPower_units_s2, ...
    "jerkPower_units_s3", jerkPower_units_s3, ...
    "TerminalState", struct("position_units", ...
    squeeze(controlPoint_units(end, end, :)).', "velocity_units_s", [0 0], "acceleration_units_s2", [0 0]));
end

%% Section 3: Local Functions

function power_units = stabilizePolynomialEndpoints(power_units, controlPoint_units)
    % Correct roundoff so position through jerk match at Bernstein endpoints.
    degree       = size(controlPoint_units, 2) - 1;
    segmentCount = size(controlPoint_units, 1);
    endPower     = degree - 3:degree;
    orders       = (0:3).';
    endMap       = factorial(endPower) ./ factorial(endPower - orders);
    target       = zeros(segmentCount, 2, 4);
    power_units(:, :, 1) = reshape(controlPoint_units(:, 1, :), segmentCount, 2);
    target(:, :, 1) = reshape(controlPoint_units(:, end, :), segmentCount, 2);
    % Process each order needed to complete stabilize polynomial endpoints.
    for order = 1:3
        difference = diff(controlPoint_units, order, 2);
        scale      = factorial(degree) / factorial(degree - order);
        power_units(:, :, order + 1) = reshape(difference(:, 1, :), segmentCount, 2) * scale / factorial(order);
        target(:, :, order + 1) = reshape(difference(:, end, :), segmentCount, 2) * scale;
    end
    % Process each projection pass needed to complete stabilize polynomial endpoints.
    for projectionPass = 1:2
        current = zeros(segmentCount, 2, 4);
        % Process each order needed to complete stabilize polynomial endpoints.
        for order = 0:3
            indices     = order:degree;
            multipliers = reshape(factorial(indices) ./ factorial(indices - order), 1, 1, []);
            current(:, :, order + 1) = sum(power_units(:, :, indices + 1) .* multipliers, 3);
        end
        residual   = reshape(permute(target - current, [3 1 2]), 4, []);
        correction = permute(reshape(endMap \ residual, 4, segmentCount, 2), [2 3 1]);
        power_units(:, :, endPower + 1) = power_units(:, :, endPower + 1) + correction;
    end
end
