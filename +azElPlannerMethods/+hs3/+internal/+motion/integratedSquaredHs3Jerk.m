function [value, gradient, hessian] = integratedSquaredHs3Jerk( ...
        decision, isEarliestArrival, fixedFinalTime_s, ...
        segmentCount, startTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   value = azElPlannerMethods.hs3.internal.motion.integratedSquaredHs3Jerk( ...
%       decision, isEarliestArrival, fixedFinalTime_s, ...
%       segmentCount, startTime_s)
%   [value, gradient] = azElPlannerMethods.hs3.internal.motion.integratedSquaredHs3Jerk( ...
%       decision, isEarliestArrival, fixedFinalTime_s, ...
%       segmentCount, startTime_s)
%   [value, gradient, hessian] = ...
%       azElPlannerMethods.hs3.internal.motion.integratedSquaredHs3Jerk( ...
%       decision, false, fixedFinalTime_s, segmentCount, startTime_s)
%**************************************************************************
% PURPOSE
%   - Integrate squared quadratic HS3 jerk and its exact decision gradient.
%**************************************************************************
% INPUTS
%   - decision (numeric column)
%       Axis-major jerk controls and optional final time.
%   - isEarliestArrival (logical scalar)
%   - fixedFinalTime_s, startTime_s (finite scalar seconds)
%   - segmentCount (positive integer scalar)
%**************************************************************************
% OUTPUTS
%   - value (nonnegative scalar)
%   - gradient (numeric column)
%       Exact gradient with respect to every supplied decision value.
%   - hessian (numeric square matrix)
%       Constant fixed-time jerk Hessian. Requesting it for an earliest-
%       arrival decision is unsupported because duration then varies.
%**************************************************************************
% UNITS
%   - Value is deg^2/s^5; gradients are deg/s^2 or deg^2/s^6 for time.
%**************************************************************************
%% Section 1: Integrate The Quadratic Jerk Records
% Separate the interleaved two-axis jerk controls from the optional arrival time.
controlCount = 2 * segmentCount + 1;
jerkValueCount = 2 * controlCount;
jerk_deg_s3 = reshape(decision(1:jerkValueCount), controlCount, 2);

finalTime_s = fixedFinalTime_s;
if isEarliestArrival
    finalTime_s = decision(end);
end

% Every HS3 segment has equal duration and three shared jerk samples.
segmentDuration_s = (finalTime_s - startTime_s) / segmentCount;
startJerk_deg_s3 = jerk_deg_s3(1:2:end - 2, :);
midpointJerk_deg_s3 = jerk_deg_s3(2:2:end - 1, :);
endJerk_deg_s3 = jerk_deg_s3(3:2:end, :);

% Integrate the squared quadratic interpolant analytically on normalized time.
normalizedIntegral = (4 * startJerk_deg_s3.^2 + ...
    16 * midpointJerk_deg_s3.^2 + 4 * endJerk_deg_s3.^2 + ...
    4 * startJerk_deg_s3 .* midpointJerk_deg_s3 - ...
    2 * startJerk_deg_s3 .* endJerk_deg_s3 + ...
    4 * midpointJerk_deg_s3 .* endJerk_deg_s3) / 30;
value = segmentDuration_s * sum(normalizedIntegral, "all");

if nargout < 2
    return;
end

% Differentiate the same closed-form integral with respect to each jerk sample.
gradientJerk = zeros(controlCount, 2);
gradientJerk(1:2:end - 2, :) = segmentDuration_s * ...
    (8 * startJerk_deg_s3 + 4 * midpointJerk_deg_s3 - ...
    2 * endJerk_deg_s3) / 30;
gradientJerk(2:2:end - 1, :) = segmentDuration_s * ...
    (32 * midpointJerk_deg_s3 + 4 * startJerk_deg_s3 + ...
    4 * endJerk_deg_s3) / 30;
gradientJerk(3:2:end, :) = gradientJerk(3:2:end, :) + ...
    segmentDuration_s * (8 * endJerk_deg_s3 - ...
    2 * startJerk_deg_s3 + 4 * midpointJerk_deg_s3) / 30;
gradient = gradientJerk(:);

% Earliest-arrival mode appends final time to the optimization decision vector.
if isEarliestArrival
    gradient(end + 1, 1) = value / (finalTime_s - startTime_s);
end

if nargout < 3
    return;
end
if isEarliestArrival
    error("integratedSquaredHs3Jerk:VariableTimeHessian", ...
        "The constant Hessian is available only for fixed arrival time.");
end

% Assemble one exact positive-definite quadratic block per axis. Adjacent
% segments share endpoint jerk ordinates, so their local blocks accumulate.
axisHessian = zeros(controlCount);
localHessian = segmentDuration_s / 30 * [ ...
    8 4 -2; 4 32 4; -2 4 8];
for segmentIndex = 1:segmentCount
    controlIndex = 2 * segmentIndex - 1:2 * segmentIndex + 1;
    axisHessian(controlIndex, controlIndex) = ...
        axisHessian(controlIndex, controlIndex) + localHessian;
end
hessian = blkdiag(axisHessian, axisHessian);
end
