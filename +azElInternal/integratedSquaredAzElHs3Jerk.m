function [value, gradient] = integratedSquaredAzElHs3Jerk( ...
        decision, isEarliestArrival, fixedFinalTime_s, ...
        segmentCount, startTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   value = azElInternal.integratedSquaredAzElHs3Jerk( ...
%       decision, isEarliestArrival, fixedFinalTime_s, ...
%       segmentCount, startTime_s)
%   [value, gradient] = azElInternal.integratedSquaredAzElHs3Jerk( ...
%       decision, isEarliestArrival, fixedFinalTime_s, ...
%       segmentCount, startTime_s)
%**************************************************************************
% PURPOSE
%   - Integrate squared quadratic HS3 jerk and its exact decision gradient.
%**************************************************************************
% INPUTS
%   - decision (numeric column)
%       Interleaved two-axis jerk controls and optional final time.
%   - isEarliestArrival (logical scalar)
%   - fixedFinalTime_s, startTime_s (finite scalar seconds)
%   - segmentCount (positive integer scalar)
%**************************************************************************
% OUTPUTS
%   - value (nonnegative scalar)
%   - gradient (numeric column)
%       Exact gradient with respect to every supplied decision value.
%**************************************************************************
% UNITS
%   - Value is deg^2/s^5; gradients are deg/s^2 or deg^2/s^6 for time.
%**************************************************************************
%% Section 1: Integrate The Quadratic Jerk Records
controlCount = 2 * segmentCount + 1;
jerkValueCount = 2 * controlCount;
jerk_deg_s3 = reshape(decision(1:jerkValueCount), controlCount, 2);
finalTime_s = fixedFinalTime_s;
if isEarliestArrival
    finalTime_s = decision(end);
end
segmentDuration_s = (finalTime_s - startTime_s) / segmentCount;
startJerk_deg_s3 = jerk_deg_s3(1:2:end - 2, :);
midpointJerk_deg_s3 = jerk_deg_s3(2:2:end - 1, :);
endJerk_deg_s3 = jerk_deg_s3(3:2:end, :);
normalizedIntegral = (4 * startJerk_deg_s3.^2 + ...
    16 * midpointJerk_deg_s3.^2 + 4 * endJerk_deg_s3.^2 + ...
    4 * startJerk_deg_s3 .* midpointJerk_deg_s3 - ...
    2 * startJerk_deg_s3 .* endJerk_deg_s3 + ...
    4 * midpointJerk_deg_s3 .* endJerk_deg_s3) / 30;
value = segmentDuration_s * sum(normalizedIntegral, "all");
if nargout < 2
    return;
end
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
if isEarliestArrival
    gradient(end + 1, 1) = value / (finalTime_s - startTime_s);
end
end
