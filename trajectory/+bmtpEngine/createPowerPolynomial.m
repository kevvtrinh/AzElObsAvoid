function polynomial = createPowerPolynomial( ...
        controlPoint_deg, segmentTime_s, initialTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   polynomial = bmtpEngine.createPowerPolynomial( ...
%       controlPoint_deg, segmentTime_s, initialTime_s)
%**************************************************************************
% PURPOSE
%   - Convert composite Bernstein control points to the stable ascending-power
%     polynomial representation and stabilize exact endpoint derivatives.
%**************************************************************************
% INPUTS
%   - controlPoint_deg (S-by-(D+1)-by-2 numeric array)
%       Composite Bezier control points.
%   - segmentTime_s (positive numeric scalar)
%       Common physical segment duration.
%   - initialTime_s (finite numeric scalar)
%       Absolute motion start time.
%**************************************************************************
% OUTPUTS
%   - polynomial (scalar struct)
%       Position, derivative powers, segment times, and terminal state.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s,
%     deg/s^2, and deg/s^3.
%**************************************************************************

%% Section 1: Convert Bernstein Controls To Powers

segmentCount = size(controlPoint_deg, 1);
degree = size(controlPoint_deg, 2) - 1;
[powerIndex, bernsteinIndex] = ndgrid(0:degree);
valid = bernsteinIndex <= powerIndex;
conversion = zeros(degree + 1);
conversion(valid) = factorial(degree) * (-1) .^ ...
    (powerIndex(valid) - bernsteinIndex(valid)) ./ ...
    (factorial(bernsteinIndex(valid)) .* ...
    factorial(powerIndex(valid) - bernsteinIndex(valid)) .* ...
    factorial(degree - powerIndex(valid)));
bernsteinPages = permute(controlPoint_deg, [2 1 3]);
positionPower_deg = permute( ...
    pagemtimes(conversion, bernsteinPages), [2 3 1]);
positionPower_deg = stabilizePolynomialEndpoints( ...
    positionPower_deg, controlPoint_deg);

%% Section 2: Create Physical Derivative Powers And Timing

velocityPower_deg_s = positionPower_deg(:, :, 2:end) .* ...
    reshape(1:degree, 1, 1, []) / segmentTime_s;
accelerationPower_deg_s2 = velocityPower_deg_s(:, :, 2:end) .* ...
    reshape(1:degree - 1, 1, 1, []) / segmentTime_s;
jerkPower_deg_s3 = accelerationPower_deg_s2(:, :, 2:end) .* ...
    reshape(1:degree - 2, 1, 1, []) / segmentTime_s;
segmentStartTime_s = initialTime_s + ...
    (0:segmentCount - 1).' * segmentTime_s;
polynomial = struct("Degree", degree, "SegmentCount", segmentCount, ...
    "SegmentStartTime_s", segmentStartTime_s, ...
    "SegmentDuration_s", repmat(segmentTime_s, segmentCount, 1), ...
    "SegmentBreakTau", (0:segmentCount).' / segmentCount, ...
    "FinalTime_s", initialTime_s + segmentCount * segmentTime_s, ...
    "positionPower_deg", positionPower_deg, ...
    "velocityPower_deg_s", velocityPower_deg_s, ...
    "accelerationPower_deg_s2", accelerationPower_deg_s2, ...
    "jerkPower_deg_s3", jerkPower_deg_s3, ...
    "TerminalState", struct("position_deg", ...
    squeeze(controlPoint_deg(end, end, :)).', ...
    "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]));
end

%% Section 3: Local Functions

function power_deg = stabilizePolynomialEndpoints( ...
        power_deg, controlPoint_deg)
% Project power roundoff onto exact Bernstein C0-C3 endpoints.
degree = size(controlPoint_deg, 2) - 1;
segmentCount = size(controlPoint_deg, 1);
endPower = degree - 3:degree;
orders = (0:3).';
endMap = factorial(endPower) ./ factorial(endPower - orders);
target = zeros(segmentCount, 2, 4);
power_deg(:, :, 1) = reshape( ...
    controlPoint_deg(:, 1, :), segmentCount, 2);
target(:, :, 1) = reshape( ...
    controlPoint_deg(:, end, :), segmentCount, 2);
for order = 1:3
    difference = diff(controlPoint_deg, order, 2);
    scale = factorial(degree) / factorial(degree - order);
    power_deg(:, :, order + 1) = reshape( ...
        difference(:, 1, :), segmentCount, 2) * ...
        scale / factorial(order);
    target(:, :, order + 1) = reshape( ...
        difference(:, end, :), segmentCount, 2) * scale;
end
for projectionPass = 1:2
    current = zeros(segmentCount, 2, 4);
    for order = 0:3
        indices = order:degree;
        multipliers = reshape(factorial(indices) ./ ...
            factorial(indices - order), 1, 1, []);
        current(:, :, order + 1) = sum( ...
            power_deg(:, :, indices + 1) .* multipliers, 3);
    end
    residual = reshape(permute(target - current, [3 1 2]), 4, []);
    correction = permute(reshape( ...
        endMap \ residual, 4, segmentCount, 2), [2 3 1]);
    power_deg(:, :, endPower + 1) = ...
        power_deg(:, :, endPower + 1) + correction;
end
end
