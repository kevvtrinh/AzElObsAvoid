function [bestMotion, bestValidation, diagnostics] = solveAzElCompactC3( ...
        seed, upperDuration_s, obstacles, initialState, goalState, ...
        limits, plannerOptions)
%SOLVEAZELCOMPACTC3 Bounded minimum-time search over an eight-span C3 spline.
spanCount = 8;
degree = 5;
controlCount = 2 * spanCount + 4;
interiorIndex = 4:controlCount - 3;
fixedIndex = [1:3, controlCount - 2:controlCount];
controlPoint_deg = [repmat(initialState.position_deg, 3, 1); ...
    zeros(numel(interiorIndex), 2); repmat(goalState.position_deg, 3, 1)];
basisPolynomial = azElInternal.convertAzElBsplineToPolynomial( ...
    eye(controlCount), degree, 0, ones(spanCount, 1) / spanCount);
fitTime = linspace(0, 1, 257).';
[~, fitBasis] = azElInternal.evaluateAzElPolynomial( ...
    basisPolynomial, fitTime);
desiredPosition_deg = interp1(seed.tau, seed.position_deg, fitTime, "linear");
fixedContribution_deg = fitBasis(:, fixedIndex) * ...
    controlPoint_deg(fixedIndex, :);
controlPoint_deg(interiorIndex, :) = fitBasis(:, interiorIndex) \ ...
    (desiredPosition_deg - fixedContribution_deg);
straight = azElInternal.buildAzElQuinticSpline( ...
    [initialState.position_deg; goalState.position_deg], ...
    initialState, goalState, limits, struct( ...
    "SampleTime_s", plannerOptions.SampleTime_s, ...
    "GoalTimeMode", "earliestArrival", ...
    "AllowAzimuthWrapping", plannerOptions.AllowAzimuthWrapping));
physicalLowerDuration_s = straight.MotionDuration_s;
bestMotion = [];
bestValidation = validateAzElTrajectory();
initialDuration_s = upperDuration_s;
duration_s = physicalLowerDuration_s + ...
    0.1 * (upperDuration_s - physicalLowerDuration_s);
failedDuration_s = physicalLowerDuration_s;
qpCount = 0;
trialCount = 0;
lastExitFlag = NaN;
bestExitFlag = NaN;
for trialIndex = 1:8
    [trialMotion, trialControlPoint_deg, lastExitFlag, trialQpCount] = ...
        solveDuration(controlPoint_deg, duration_s, obstacles, ...
        initialState, limits, plannerOptions, interiorIndex, fixedIndex);
    qpCount = qpCount + trialQpCount;
    trialCount = trialIndex;
    validation = validateAzElTrajectory( ...
        trialMotion, obstacles, initialState, goalState, limits, ...
        plannerOptions);
    if validation.Passed
        upperDuration_s = duration_s;
        bestMotion = trialMotion;
        bestValidation = validation;
        bestExitFlag = lastExitFlag;
        controlPoint_deg = trialControlPoint_deg;
        if isnan(failedDuration_s)
            nextDuration_s = max(physicalLowerDuration_s, 0.85 * duration_s);
        else
            nextDuration_s = 0.5 * (failedDuration_s + duration_s);
        end
    else
        if trialIndex == 1
            failedDuration_s = NaN;
            nextDuration_s = initialDuration_s;
        else
            failedDuration_s = duration_s;
            nextDuration_s = 0.25 * duration_s + 0.75 * upperDuration_s;
        end
    end
    if abs(nextDuration_s - duration_s) <= 1e-6
        break;
    end
    duration_s = nextDuration_s;
end
diagnostics = struct("Attempted", true, "Accepted", ~isempty(bestMotion), ...
    "TrialCount", trialCount, "QpCount", qpCount, ...
    "ExitFlag", bestExitFlag, "LastExitFlag", lastExitFlag, ...
    "InitialDuration_s", initialDuration_s, ...
    "BestDuration_s", upperDuration_s);
end
function [motion, controlPoint_deg, exitFlag, qpCount] = solveDuration( ...
        controlPoint_deg, duration_s, obstacles, initialState, limits, ...
        plannerOptions, interiorIndex, fixedIndex)
spanCount = 8;
spanDuration_s = duration_s / spanCount * ones(spanCount, 1);
affinePolynomial = azElInternal.convertAzElBsplineToPolynomial( ...
    eye(size(controlPoint_deg, 1)), 5, initialState.time_s, spanDuration_s);
sampleTime_s = linspace(initialState.time_s, ...
    initialState.time_s + duration_s, 257).';
[~, positionBasis, velocityBasis, accelerationBasis, jerkBasis] = ...
    azElInternal.evaluateAzElPolynomial(affinePolynomial, sampleTime_s);
decision = [controlPoint_deg(interiorIndex, 1); ...
    controlPoint_deg(interiorIndex, 2)];
quadraticOptions = optimoptions("quadprog", "Display", "off", ...
    "Algorithm", "active-set");
exitFlag = NaN;
qpCount = 0;
for iterationIndex = 1:6
    fixedPosition_deg = positionBasis(:, fixedIndex) * ...
        controlPoint_deg(fixedIndex, :);
    [barrierMatrix, barrierBound] = obstacleRows( ...
        positionBasis(:, interiorIndex), fixedPosition_deg, decision, ...
        sampleTime_s, obstacles);
    [kinematicMatrix, kinematicBound] = kinematicRows( ...
        {positionBasis, velocityBasis, accelerationBasis, jerkBasis}, ...
        controlPoint_deg, interiorIndex, fixedIndex, limits);
    fixedJerk = jerkBasis(:, fixedIndex) * controlPoint_deg(fixedIndex, :);
    interiorJerk = jerkBasis(:, interiorIndex);
    jerkMap = blkdiag(interiorJerk, interiorJerk);
    baseJerk = [fixedJerk(:, 1); fixedJerk(:, 2)];
    hessian = 2 * (jerkMap.' * jerkMap + 1e-9 * eye(numel(decision)));
    gradient = 2 * jerkMap.' * baseJerk;
    [trialDecision, ~, exitFlag] = quadprog( ...
        hessian, gradient, [kinematicMatrix; barrierMatrix], ...
        [kinematicBound; barrierBound], [], [], decision - 6, ...
        decision + 6, decision, quadraticOptions);
    qpCount = qpCount + 1;
    if exitFlag <= 0
        break;
    end
    step_deg = max(abs(trialDecision - decision));
    decision = trialDecision;
    controlPoint_deg(interiorIndex, :) = [ ...
        decision(1:numel(interiorIndex)), ...
        decision(numel(interiorIndex) + 1:end)];
    if step_deg <= 1e-4
        break;
    end
end
polynomial = azElInternal.convertAzElBsplineToPolynomial( ...
    controlPoint_deg, 5, initialState.time_s, spanDuration_s);
outputTime_s = (initialState.time_s:plannerOptions.SampleTime_s: ...
    polynomial.FinalTime_s).';
outputTime_s = unique([outputTime_s; polynomial.FinalTime_s]);
[outputTime_s, position_deg, velocity_deg_s, acceleration_deg_s2, ...
    jerk_deg_s3] = azElInternal.evaluateAzElPolynomial( ...
    polynomial, outputTime_s);
motion = struct("time_s", outputTime_s, "position_deg", position_deg, ...
    "velocity_deg_s", velocity_deg_s, ...
    "acceleration_deg_s2", acceleration_deg_s2, ...
    "jerk_deg_s3", jerk_deg_s3, "Polynomial", polynomial, ...
    "FinalTime_s", polynomial.FinalTime_s, ...
    "MotionDuration_s", polynomial.FinalTime_s - initialState.time_s, ...
    "IntegratedSquaredJerk_deg2_s5", ...
    trapz(outputTime_s, sum(jerk_deg_s3 .^ 2, 2)), ...
    "ControlPoint_deg", controlPoint_deg, ...
    "SpanDuration_s", spanDuration_s);
end
function [matrix, bound] = obstacleRows( ...
        interiorBasis, fixedPosition_deg, decision, time_s, obstacles)
interiorCount = size(interiorBasis, 2);
position_deg = fixedPosition_deg + [ ...
    interiorBasis * decision(1:interiorCount), ...
    interiorBasis * decision(interiorCount + 1:end)];
matrix = zeros(numel(time_s) * max(1, numel(obstacles)), 2 * interiorCount);
bound = zeros(size(matrix, 1), 1);
includedRow = false(size(bound));
obstacleCount = numel(obstacles);
for obstacleIndex = 1:obstacleCount
    preparation = obstacles(obstacleIndex).InternalPreparation;
    historyBounds_deg = preparation.HistoryBounds_deg;
    axisDistance_deg = max(cat(3, historyBounds_deg([1 3]) - position_deg, ...
        zeros(size(position_deg)), position_deg - historyBounds_deg([2 4])), ...
        [], 3);
    nearTimeIndex = find(vecnorm(axisDistance_deg, 2, 2) < 10);
    if isempty(nearTimeIndex)
        continue;
    end
    stationaryHistory = all(preparation.IntervalSpeedBound_deg_s == 0);
    if stationaryHistory
        shape = preparation.SampleShapes{1};
        [clearance_deg, nearestPoint_deg] = ...
            azElInternal.pointPolygonClearance( ...
            shape, position_deg(nearTimeIndex, :));
    else
        clearance_deg = zeros(numel(nearTimeIndex), 1);
        nearestPoint_deg = zeros(numel(nearTimeIndex), 2);
        for nearIndex = 1:numel(nearTimeIndex)
            timeIndex = nearTimeIndex(nearIndex);
            shape = azElInternal.obstacleShapeAtTime( ...
                obstacles(obstacleIndex), time_s(timeIndex));
            [clearance_deg(nearIndex), nearestPoint_deg(nearIndex, :)] = ...
                azElInternal.pointPolygonClearance( ...
                shape, position_deg(timeIndex, :));
        end
    end
    direction_deg = position_deg(nearTimeIndex, :) - nearestPoint_deg;
    direction_deg(clearance_deg < 0, :) = -direction_deg(clearance_deg < 0, :);
    directionNorm_deg = vecnorm(direction_deg, 2, 2);
    selectedQuery = clearance_deg < 10 & directionNorm_deg > eps;
    timeIndex = nearTimeIndex(selectedQuery);
    outward = direction_deg(selectedQuery, :) ./ directionNorm_deg(selectedQuery);
    rowIndex = (timeIndex - 1) * obstacleCount + obstacleIndex;
    matrix(rowIndex, 1:interiorCount) = ...
        -outward(:, 1) .* interiorBasis(timeIndex, :);
    matrix(rowIndex, interiorCount + 1:end) = ...
        -outward(:, 2) .* interiorBasis(timeIndex, :);
    bound(rowIndex) = sum(outward .* (fixedPosition_deg(timeIndex, :) - ...
        nearestPoint_deg(selectedQuery, :)), 2) - 0.1;
    includedRow(rowIndex) = true;
end
matrix = matrix(includedRow, :);
bound = bound(includedRow);
end
function [matrix, bound] = kinematicRows( ...
        basisByOrder, controlPoint_deg, interiorIndex, fixedIndex, limits)
limitByOrder = {[Inf Inf], limits.maxVelocity_deg_s, ...
    limits.maxAcceleration_deg_s2, limits.maxJerk_deg_s3};
workspaceByAxis = {limits.azimuthInterval_deg, limits.elevationInterval_deg};
decisionCount = 2 * numel(interiorIndex);
matrix = zeros(0, decisionCount);
bound = zeros(0, 1);
for orderIndex = 1:4
    basis = basisByOrder{orderIndex};
    for axisIndex = 1:2
        if orderIndex == 1
            lowerLimit = workspaceByAxis{axisIndex}(1);
            upperLimit = workspaceByAxis{axisIndex}(2);
        else
            lowerLimit = -0.999 * limitByOrder{orderIndex}(axisIndex);
            upperLimit = 0.999 * limitByOrder{orderIndex}(axisIndex);
        end
        fixedValue = basis(:, fixedIndex) * ...
            controlPoint_deg(fixedIndex, axisIndex);
        block = zeros(size(basis, 1), decisionCount);
        selectedDecision = (axisIndex - 1) * numel(interiorIndex) + ...
            (1:numel(interiorIndex));
        block(:, selectedDecision) = basis(:, interiorIndex);
        matrix = [matrix; block; -block]; %#ok<AGROW>
        bound = [bound; upperLimit - fixedValue; ...
            -lowerLimit + fixedValue]; %#ok<AGROW>
    end
end
end
