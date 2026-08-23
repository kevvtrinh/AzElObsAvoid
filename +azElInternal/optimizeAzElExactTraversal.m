function [bestDecision_deg, diagnostics] = optimizeAzElExactTraversal( ...
        baseMotion, affineBasisPolynomial, initialDecision_deg, ...
        corridorMatrix, corridorBound, maximumOffset_deg, limits)
% Minimize continuous derivative scale by bounded exact-extrema exchange.

%% Section 1: Build The Affine Derivative Maps

derivativeFields = ["velocityPower_deg_s", ...
    "accelerationPower_deg_s2", "jerkPower_deg_s3"];
derivativeLimits = {limits.maxVelocity_deg_s, ...
    limits.maxAcceleration_deg_s2, limits.maxJerk_deg_s3};
decisionCount = numel(initialDecision_deg);
interiorCount = decisionCount / 2;
segmentCount = baseMotion.Polynomial.SegmentCount;
basisPower = cell(3, 1);
basePower = cell(3, 1);
for derivativeOrder = 1:3
    basePower{derivativeOrder} = ...
        baseMotion.Polynomial.(derivativeFields(derivativeOrder));
    coefficientCount = size(basePower{derivativeOrder}, 3);
    basisPower{derivativeOrder} = zeros( ...
        segmentCount, 2, coefficientCount, decisionCount);
    affinePower = ...
        affineBasisPolynomial.(derivativeFields(derivativeOrder));
    for decisionIndex = 1:decisionCount
        interiorIndex = mod(decisionIndex - 1, interiorCount) + 1;
        axisIndex = floor((decisionIndex - 1) / interiorCount) + 1;
        controlPointIndex = interiorIndex + 3;
        basisPower{derivativeOrder}(:, axisIndex, :, decisionIndex) = ...
            affinePower(:, controlPointIndex, :);
    end
end

%% Section 2: Exchange Exact Extrema At Bounded Scales

[activeMatrix, activeBase, activeOrder, initialScale] = ...
    exactDerivativeRows( ...
    basePower, basisPower, initialDecision_deg, derivativeLimits, Inf, true);
bestDecision_deg = initialDecision_deg;
lowerScale = 0;
upperScale = initialScale;
linearSolveCount = 0;
maximumBisectionCount = 10;
[jerkHessian, jerkGradient] = minimumJerkObjective( ...
    basePower{3}, basisPower{3}, derivativeLimits{3});
quadraticOptions = optimoptions("quadprog", "Display", "off");
for bisectionIndex = 1:maximumBisectionCount
    midpointScale = 0.5 * (lowerScale + upperScale);
    scaleIsFeasible = false;
    maximumExchangeCount = 6;
    for exchangeIndex = 1:maximumExchangeCount
        activeBound = midpointScale .^ activeOrder - activeBase;
        [trialDecision_deg, ~, linearExitFlag] = quadprog( ...
            jerkHessian, jerkGradient, ...
            [corridorMatrix; activeMatrix], ...
            [corridorBound; activeBound], [], [], ...
            -maximumOffset_deg * ones(decisionCount, 1), ...
            maximumOffset_deg * ones(decisionCount, 1), ...
            [], quadraticOptions);
        linearSolveCount = linearSolveCount + 1;
        if linearExitFlag <= 0
            break;
        end
        [newMatrix, newBase, newOrder] = exactDerivativeRows( ...
            basePower, basisPower, trialDecision_deg, derivativeLimits, ...
            midpointScale, false);
        if isempty(newOrder)
            scaleIsFeasible = true;
            bestDecision_deg = trialDecision_deg;
            break;
        end
        activeMatrix = [activeMatrix; newMatrix]; %#ok<AGROW>
        activeBase = [activeBase; newBase]; %#ok<AGROW>
        activeOrder = [activeOrder; newOrder]; %#ok<AGROW>
    end
    if scaleIsFeasible
        upperScale = midpointScale;
    else
        lowerScale = midpointScale;
    end
end

%% Section 3: Return Only A Verified Scale Improvement

accepted = upperScale < initialScale - 1e-4 * max(1, initialScale);
if ~accepted
    bestDecision_deg = initialDecision_deg;
end
diagnostics = struct( ...
    "Attempted", true, "Accepted", accepted, ...
    "InitialScale", initialScale, "FinalScale", upperScale, ...
    "LinearSolveCount", linearSolveCount, ...
    "ActiveConstraintCount", numel(activeOrder));
end

%% Section 4: Local Functions

function [hessian, gradient] = minimumJerkObjective( ...
        basePower, basisPower, jerkLimit_deg_s3)
% SYNTAX
%   [hessian, gradient] = minimumJerkObjective( ...
%       basePower, basisPower, jerkLimit_deg_s3)
% PURPOSE
%   - Form a sampled limit-normalized integrated-jerk quadratic objective.
% INPUTS
%   - basePower, basisPower (base and affine jerk coefficient arrays)
%   - jerkLimit_deg_s3 (1-by-2 positive jerk limits)
% OUTPUTS
%   - hessian, gradient (quadratic-program objective terms)
% UNITS
%   - Decision offsets are degrees; the objective is limit-normalized.
sampleTau = linspace(0, 1, 7).';
powerBasis = sampleTau .^ (0:size(basePower, 3) - 1);
decisionCount = size(basisPower, 4);
rowCount = size(basePower, 1) * 2 * numel(sampleTau);
jerkMap = zeros(rowCount, decisionCount);
baseJerk = zeros(rowCount, 1);
rowIndex = 0;
for segmentIndex = 1:size(basePower, 1)
    for axisIndex = 1:2
        rows = rowIndex + (1:numel(sampleTau));
        coefficient = reshape(basePower(segmentIndex, axisIndex, :), [], 1);
        affineCoefficient = reshape( ...
            basisPower(segmentIndex, axisIndex, :, :), ...
            numel(coefficient), decisionCount);
        baseJerk(rows) = powerBasis * coefficient / ...
            jerkLimit_deg_s3(axisIndex);
        jerkMap(rows, :) = powerBasis * affineCoefficient / ...
            jerkLimit_deg_s3(axisIndex);
        rowIndex = rowIndex + numel(sampleTau);
    end
end
hessian = jerkMap.' * jerkMap + 1e-10 * eye(decisionCount);
gradient = jerkMap.' * baseJerk;
end

function [constraintMatrix, constraintBase, derivativeOrder, ...
        maximumScale] = exactDerivativeRows( ...
        basePower, basisPower, decision_deg, limits, requestedScale, ...
        collectAll)
%% Section 0: Header & Readme
% PURPOSE
%   - Return signed affine constraints at exact extrema of one trial spline.
decisionCount = numel(decision_deg);
constraintMatrix = zeros(0, decisionCount);
constraintBase = zeros(0, 1);
derivativeOrder = zeros(0, 1);
maximumScale = 0;
violationTolerance = 1e-8;
for orderIndex = 1:3
    coefficientArray = basePower{orderIndex};
    coefficientCount = size(coefficientArray, 3);
    for segmentIndex = 1:size(coefficientArray, 1)
        for axisIndex = 1:2
            baseCoefficient = reshape( ...
                coefficientArray(segmentIndex, axisIndex, :), [], 1);
            basisCoefficient = reshape( ...
                basisPower{orderIndex}(segmentIndex, axisIndex, :, :), ...
                coefficientCount, decisionCount);
            trialCoefficient = baseCoefficient + ...
                basisCoefficient * decision_deg;
            slopeCoefficient = (1:coefficientCount - 1).' .* ...
                trialCoefficient(2:end);
            lastSlopeIndex = find(slopeCoefficient ~= 0, 1, "last");
            candidateTau = [0; 1];
            if ~isempty(lastSlopeIndex)
                stationaryRoot = roots(flip( ...
                    slopeCoefficient(1:lastSlopeIndex)));
                rootTolerance = 1e-9;
                stationaryTau = real(stationaryRoot);
                stationaryTau = stationaryTau( ...
                    stationaryTau >= -rootTolerance & ...
                    stationaryTau <= 1 + rootTolerance);
                candidateTau = unique([candidateTau; ...
                    min(max(stationaryTau, 0), 1)]);
            end
            powerBasis = candidateTau.^(0:coefficientCount - 1);
            trialValue = powerBasis * trialCoefficient;
            normalizedMagnitude = ...
                abs(trialValue) / limits{orderIndex}(axisIndex);
            maximumScale = max(maximumScale, ...
                max(normalizedMagnitude) ^ (1 / orderIndex));
            for candidateIndex = 1:numel(candidateTau)
                requiredScale = normalizedMagnitude(candidateIndex) ^ ...
                    (1 / orderIndex);
                if ~collectAll && requiredScale <= ...
                        requestedScale * (1 + violationTolerance)
                    continue;
                end
                valueSign = sign(trialValue(candidateIndex));
                if valueSign == 0
                    continue;
                end
                normalizedBasis = ...
                    powerBasis(candidateIndex, :) * basisCoefficient / ...
                    limits{orderIndex}(axisIndex);
                normalizedBase = ...
                    powerBasis(candidateIndex, :) * baseCoefficient / ...
                    limits{orderIndex}(axisIndex);
                constraintMatrix(end + 1, :) = ...
                    valueSign * normalizedBasis; %#ok<AGROW>
                constraintBase(end + 1, 1) = ...
                    valueSign * normalizedBase; %#ok<AGROW>
                derivativeOrder(end + 1, 1) = orderIndex; %#ok<AGROW>
            end
        end
    end
end
end
