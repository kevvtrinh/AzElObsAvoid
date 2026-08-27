function [value, gradient, hessian] = evaluateIntegratedSquaredJerk( ...
        decision, isFreeTime, fixedFinalTime, segmentCount, ...
        startTime, dimensionCount)
%% Section 0: Header & Readme
% SYNTAX
%   value = hs3Internal.polynomial.evaluateIntegratedSquaredJerk(decision, isFreeTime, ...
%       fixedFinalTime, segmentCount, startTime, dimensionCount)
%   [value, gradient] = hs3Internal.polynomial.evaluateIntegratedSquaredJerk(decision, isFreeTime, ...
%       fixedFinalTime, segmentCount, startTime, dimensionCount)
%   [value, gradient, hessian] = hs3Internal.polynomial.evaluateIntegratedSquaredJerk( ...
%       decision, false, fixedFinalTime, segmentCount, startTime, ...
%       dimensionCount)
%**************************************************************************
% PURPOSE
%   - Integrate squared quadratic HS3 jerk and its exact decision gradient.
%**************************************************************************
% INPUTS
%   - decision (numeric column), coordinate-major jerk and optional time.
%   - isFreeTime (logical scalar), true when decision ends in final time.
%   - fixedFinalTime, startTime (finite scalar), caller-defined time unit.
%   - segmentCount (positive integer scalar), equal-duration segments.
%   - dimensionCount (positive integer scalar), modeled coordinates.
%**************************************************************************
% OUTPUTS
%   - value (nonnegative scalar), integrated squared jerk.
%   - gradient (numeric column), exact decision gradient.
%   - hessian (numeric square matrix), fixed-time jerk Hessian.
%**************************************************************************
% UNITS
%   - Value uses coordinate^2/time^5 for consistent caller units.
%**************************************************************************

%% Section 1: Integrate The Quadratic Jerk Records

controlCount = 2 * segmentCount + 1;
jerkValueCount = dimensionCount * controlCount;
controlJerk = reshape( ...
    decision(1:jerkValueCount), controlCount, dimensionCount);
finalTime = fixedFinalTime;
if isFreeTime
    finalTime = decision(end);
end
segmentDuration = (finalTime - startTime) / segmentCount;
startJerk = controlJerk(1:2:end - 2, :);
midpointJerk = controlJerk(2:2:end - 1, :);
endJerk = controlJerk(3:2:end, :);
normalizedIntegral = (4 * startJerk.^2 + ...
    16 * midpointJerk.^2 + 4 * endJerk.^2 + ...
    4 * startJerk .* midpointJerk - 2 * startJerk .* endJerk + ...
    4 * midpointJerk .* endJerk) / 30;
value = segmentDuration * sum(normalizedIntegral, "all");
if nargout < 2
    return;
end
gradientJerk = zeros(controlCount, dimensionCount);
gradientJerk(1:2:end - 2, :) = segmentDuration * ...
    (8 * startJerk + 4 * midpointJerk - 2 * endJerk) / 30;
gradientJerk(2:2:end - 1, :) = segmentDuration * ...
    (32 * midpointJerk + 4 * startJerk + 4 * endJerk) / 30;
gradientJerk(3:2:end, :) = gradientJerk(3:2:end, :) + ...
    segmentDuration * ...
    (8 * endJerk - 2 * startJerk + 4 * midpointJerk) / 30;
gradient = gradientJerk(:);
if isFreeTime
    gradient(end + 1, 1) = value / (finalTime - startTime);
end
if nargout < 3
    return;
end
if isFreeTime
    error("evaluateIntegratedSquaredJerk:VariableTimeHessian", ...
        "The constant Hessian is available only for fixed final time.");
end

%% Section 2: Assemble The Fixed-Time Hessian

coordinateHessian = zeros(controlCount);
localHessian = segmentDuration / 30 * [8 4 -2; 4 32 4; -2 4 8];
for segmentIndex = 1:segmentCount
    controlIndex = 2 * segmentIndex - 1:2 * segmentIndex + 1;
    coordinateHessian(controlIndex, controlIndex) = ...
        coordinateHessian(controlIndex, controlIndex) + localHessian;
end
hessian = zeros(dimensionCount * controlCount);
for dimensionIndex = 1:dimensionCount
    blockIndex = (dimensionIndex - 1) * controlCount + ...
        (1:controlCount);
    hessian(blockIndex, blockIndex) = coordinateHessian;
end
end
