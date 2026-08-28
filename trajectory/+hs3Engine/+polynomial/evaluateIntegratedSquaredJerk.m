function [value, gradient, hessian] = evaluateIntegratedSquaredJerk( ...
        decision, isFreeTime, fixedFinalTime, segmentCount, ...
        startTime, dimensionCount, segmentBreakTau)
%% Section 0: Header & Readme
% SYNTAX
%   value = hs3Engine.polynomial.evaluateIntegratedSquaredJerk(decision, isFreeTime, ...
%       fixedFinalTime, segmentCount, startTime, dimensionCount)
%   [value, gradient] = hs3Engine.polynomial.evaluateIntegratedSquaredJerk(decision, isFreeTime, ...
%       fixedFinalTime, segmentCount, startTime, dimensionCount)
%   [value, gradient, hessian] = hs3Engine.polynomial.evaluateIntegratedSquaredJerk( ...
%       decision, false, fixedFinalTime, segmentCount, startTime, ...
%       dimensionCount)
%   [value, gradient, hessian] = hs3Engine.polynomial.evaluateIntegratedSquaredJerk( ...
%       decision, false, fixedFinalTime, segmentCount, startTime, ...
%       dimensionCount, segmentBreakTau)
%**************************************************************************
% PURPOSE
%   - Integrate squared quadratic HS3 jerk and its exact decision gradient.
%**************************************************************************
% INPUTS
%   - decision (numeric column), coordinate-major jerk and optional time.
%   - isFreeTime (logical scalar), true when decision ends in final time.
%   - fixedFinalTime, startTime (finite scalar), caller-defined time unit.
%   - segmentCount (positive integer scalar), polynomial segment count.
%   - dimensionCount (positive integer scalar), modeled coordinates.
%   - segmentBreakTau ((N+1)-element vector, optional)
%       Strictly increasing normalized boundaries; [] selects uniform timing.
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

% Squared quadratic jerk has degree four. The closed-form expression below is
% its exact integral over local tau, multiplied by segment duration to convert
% back to physical time. Adjacent segments both contribute to a shared boundary
% jerk control, which is why gradient entries at end controls are accumulated.

controlCount = 2 * segmentCount + 1;
jerkValueCount = dimensionCount * controlCount;
controlJerk = reshape( ...
    decision(1:jerkValueCount), controlCount, dimensionCount);
finalTime = fixedFinalTime;
if isFreeTime
    finalTime = decision(end);
end
if nargin < 7
    segmentBreakTau = [];
end
[~, segmentDuration, isUniformMesh] = ...
    hs3Engine.polynomial.resolveSegmentMesh( ...
    segmentCount, finalTime - startTime, segmentBreakTau);
startJerk = controlJerk(1:2:end - 2, :);
midpointJerk = controlJerk(2:2:end - 1, :);
endJerk = controlJerk(3:2:end, :);
normalizedIntegral = (4 * startJerk.^2 + ...
    16 * midpointJerk.^2 + 4 * endJerk.^2 + ...
    4 * startJerk .* midpointJerk - 2 * startJerk .* endJerk + ...
    4 * midpointJerk .* endJerk) / 30;
if isUniformMesh
    value = segmentDuration * sum(normalizedIntegral, "all");
else
    value = sum(segmentDuration .* sum(normalizedIntegral, 2));
end
if nargout < 2
    return;
end
gradientJerk = zeros(controlCount, dimensionCount);
gradientJerk(1:2:end - 2, :) = segmentDuration .* ...
    (8 * startJerk + 4 * midpointJerk - 2 * endJerk) / 30;
gradientJerk(2:2:end - 1, :) = segmentDuration .* ...
    (32 * midpointJerk + 4 * startJerk + 4 * endJerk) / 30;
gradientJerk(3:2:end, :) = gradientJerk(3:2:end, :) + ...
    segmentDuration .* ...
    (8 * endJerk - 2 * startJerk + 4 * midpointJerk) / 30;
gradient = gradientJerk(:);
if isFreeTime
    % Holding jerk controls fixed makes the total cost proportional to total
    % duration, so d(cost)/d(finalTime) is cost/duration.
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

% Each segment contributes a 3-by-3 positive-semidefinite quadratic block for
% its start/mid/end controls. Overlapping blocks couple neighboring segments at
% their shared boundary. Coordinates remain independent, producing identical
% diagonal blocks in the full Hessian.

coordinateHessian = zeros(controlCount);
for segmentIndex = 1:segmentCount
    localDuration = segmentDuration;
    if ~isUniformMesh
        localDuration = segmentDuration(segmentIndex);
    end
    localHessian = localDuration / 30 * ...
        [8 4 -2; 4 32 4; -2 4 8];
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
