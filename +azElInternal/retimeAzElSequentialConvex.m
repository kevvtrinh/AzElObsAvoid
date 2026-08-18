function schedule = retimeAzElSequentialConvex(gridS_deg, pathEvaluator, ...
        derivativeBounds, stopNode, endpointSpeed_deg_s, limits)
%% Section 0: Header & Readme
% SYNTAX
%   schedule = azElInternal.retimeAzElSequentialConvex( ...
%       gridS_deg, pathEvaluator, derivativeBounds, stopNode, ...
%       endpointSpeed_deg_s, limits)
%**************************************************************************
% PURPOSE
%   - Reduce fixed-path travel time using the b(s)=sDot^2
%     sequential-convex formulation of Debrouwere et al. (2013).
%   - Retain a continuous conservative certificate between transcription
%     nodes instead of treating the spatial grid as final validation.
%**************************************************************************
% INPUTS
%   - gridS_deg (N-by-1 increasing numeric)
%       Spatial transcription nodes including every primitive boundary.
%   - pathEvaluator (scalar function handle)
%       Returns position and the first three arc derivatives at query s.
%   - derivativeBounds (N-1 structure array)
%       Certified absolute path-derivative bounds for every spatial cell.
%   - stopNode (N-by-1 logical)
%       Nodes that require zero path speed and tangential acceleration.
%   - endpointSpeed_deg_s (2-by-1 nonnegative numeric)
%       Requested initial and terminal scalar path speeds.
%   - limits (scalar struct)
%       Positive two-axis maxVelocity_deg_s, maxAcceleration_deg_s2, and
%       finite maxJerk_deg_s3 fields.
%**************************************************************************
% OUTPUTS
%   - schedule (scalar struct)
%       Stable success/failure scalar motion, continuous certificate, and
%       sequential-convex diagnostics. Expected infeasibility is returned.
%**************************************************************************
% UNITS
%   - Path position is degrees. Speed, acceleration, and jerk use deg/s,
%     deg/s^2, and deg/s^3. b has units deg^2/s^2.
%**************************************************************************

% A bounded exact-input cache avoids resolving the same relative schedule
% when adaptive temporal passes revisit an unchanged geometric route. The
% key contains every value used by the solve and continuous certificate.
persistent cachedKeys cachedSchedules

%% Section 1: Build The Direct Transcription

% Debrouwere et al., IEEE Transactions on Robotics 29(6), 2013,
% DOI 10.1109/TRO.2013.2277565, write joint jerk as a linear expression
% in b, b', and b'' multiplied by sqrt(b). The concave -J/sqrt(b) term is
% linearized at each feasible iterate to form an inner convex problem.
schedule = emptySchedule();
if exist("fmincon", "file") ~= 2 || exist("quadprog", "file") ~= 2
    schedule.Message = "Finite-jerk retiming requires fmincon and " + ...
        "quadprog from " + ...
        "Optimization Toolbox.";
    schedule.TerminationReason = "unsupportedConfiguration";
    return;
end
gridS_deg = double(gridS_deg(:));
nodeCount = numel(gridS_deg);
cellCount = nodeCount - 1;
decisionCount = 2 * nodeCount;
% SQP solves the exact convex objective efficiently on small grids, while
% its dense linear-constraint handling dominates beyond this size. Larger
% grids use a sparse quadratic model; both branches enforce the
% same linearized constraints and final continuous certificate.
exactObjectiveDecisionLimit = 59;
useQuadraticSubproblem = decisionCount > exactObjectiveDecisionLimit;
stopNode = logical(stopNode(:));
endpointSpeed_deg_s = double(endpointSpeed_deg_s(:));
if nodeCount < 3 || numel(stopNode) ~= nodeCount || ...
        numel(derivativeBounds) ~= cellCount
    error("retimeAzElSequentialConvex:InvalidGrid", ...
        "The grid, stop mask, and derivative bounds have incompatible sizes.");
end
cacheKey = struct( ...
    "gridS_deg", gridS_deg, ...
    "derivativeBounds", derivativeBounds, ...
    "stopNode", stopNode, ...
    "endpointSpeed_deg_s", endpointSpeed_deg_s, ...
    "limits", limits);
if isempty(cachedKeys)
    cachedKeys = cell(0, 1);
    cachedSchedules = cell(0, 1);
end
for cacheIndex = 1:numel(cachedKeys)
    if isequaln(cacheKey, cachedKeys{cacheIndex})
        schedule = cachedSchedules{cacheIndex};
        schedule.CacheHit = true;
        return;
    end
end

quadratureFraction = [0.112701665379258; 0.5; ...
    0.887298334620742];
quadratureWeight = [5; 8; 5] / 18;
if useQuadraticSubproblem
    % Three interior probes keep sparse quadratic subproblems bounded. The
    % continuous cell certificate, not these probes, decides final success.
    constraintFraction = [0.211324865405187; 0.5; ...
        0.788675134594813];
else
    % Endpoint probes guide the exact-objective solve through stop layers;
    % three Gauss points cover the interior of every certified cell.
    endpointProbe = 1e-6;
    constraintFraction = [endpointProbe; 0.112701665379258; 0.5; ...
        0.887298334620742; 1 - endpointProbe];
end
quadratureCountPerCell = numel(quadratureFraction);
constraintCountPerCell = numel(constraintFraction);
quadratureS_deg = zeros(cellCount * quadratureCountPerCell, 1);
constraintS_deg = zeros(cellCount * constraintCountPerCell, 1);
quadratureWeight_deg = zeros(size(quadratureS_deg));
for cellIndex = 1:cellCount
    quadratureRow = (cellIndex - 1) * quadratureCountPerCell + ...
        (1:quadratureCountPerCell);
    constraintRow = (cellIndex - 1) * constraintCountPerCell + ...
        (1:constraintCountPerCell);
    cellLength_deg = gridS_deg(cellIndex + 1) - gridS_deg(cellIndex);
    quadratureS_deg(quadratureRow) = gridS_deg(cellIndex) + ...
        cellLength_deg * quadratureFraction;
    constraintS_deg(constraintRow) = gridS_deg(cellIndex) + ...
        cellLength_deg * constraintFraction;
    quadratureWeight_deg(quadratureRow) = ...
        cellLength_deg * quadratureWeight;
end

[objectiveValueMap, ~, ~] = hermiteMaps( ...
    gridS_deg, quadratureS_deg, stopNode);
[valueMap, firstMap, secondMap] = hermiteMaps( ...
    gridS_deg, constraintS_deg, stopNode);
geometry = pathEvaluator(constraintS_deg);
certifiedGeometry = struct( ...
    "tangent", zeros(size(geometry.tangent)), ...
    "secondDerivative_deg_inv", zeros(size( ...
        geometry.secondDerivative_deg_inv)), ...
    "thirdDerivative_deg_inv2", zeros(size( ...
        geometry.thirdDerivative_deg_inv2)));
for cellIndex = 1:cellCount
    row = (cellIndex - 1) * constraintCountPerCell + ...
        (1:constraintCountPerCell);
    certifiedGeometry.tangent(row, :) = repmat( ...
        derivativeBounds(cellIndex).CertifiedTangentByAxis, ...
        constraintCountPerCell, 1);
    certifiedGeometry.secondDerivative_deg_inv(row, :) = repmat( ...
        derivativeBounds(cellIndex). ...
        CertifiedSecondDerivativeByAxis_deg_inv, ...
        constraintCountPerCell, 1);
    certifiedGeometry.thirdDerivative_deg_inv2(row, :) = repmat( ...
        derivativeBounds(cellIndex). ...
        CertifiedThirdDerivativeByAxis_deg_inv2, ...
        constraintCountPerCell, 1);
end
lowerBound = [-Inf(nodeCount, 1); -Inf(nodeCount, 1)];
upperBound = [Inf(nodeCount, 1); Inf(nodeCount, 1)];
lowerBound(1:nodeCount) = 0;
fixedNode = stopNode;
fixedNode([1 end]) = true;
fixedSpeedSquared = zeros(nodeCount, 1);
fixedSpeedSquared(1) = endpointSpeed_deg_s(1)^2;
fixedSpeedSquared(end) = endpointSpeed_deg_s(2)^2;
lowerBound(find(fixedNode)) = fixedSpeedSquared(fixedNode); %#ok<FNDSB>
upperBound(find(fixedNode)) = fixedSpeedSquared(fixedNode); %#ok<FNDSB>
lowerBound(nodeCount + find(stopNode)) = 0;
upperBound(nodeCount + find(stopNode)) = 0;

% The maintained endpoint contract currently requires zero tangential
% acceleration, hence b'=2*sDoubleDot is zero at both endpoints.
lowerBound(nodeCount + [1 nodeCount]) = 0;
upperBound(nodeCount + [1 nodeCount]) = 0;
equalityMatrix = zeros(0, decisionCount);
equalityRightSide = zeros(0, 1);
nonnegativeMatrix = zeros(0, decisionCount);
for cellIndex = 1:cellCount
    cellLength_deg = gridS_deg(cellIndex + 1) - gridS_deg(cellIndex);
    row = zeros(1, decisionCount);
    if stopNode(cellIndex) && ~stopNode(cellIndex + 1)
        row(nodeCount + cellIndex + 1) = 1;
        row(cellIndex + 1) = -4 / (3 * cellLength_deg);
    elseif ~stopNode(cellIndex) && stopNode(cellIndex + 1)
        row(nodeCount + cellIndex) = 1;
        row(cellIndex) = 4 / (3 * cellLength_deg);
    else
        % Nonnegative cubic Bezier controls are a sufficient continuous
        % condition for b(s) >= 0 between the two non-stop nodes.
        firstRow = zeros(1, decisionCount);
        firstRow(cellIndex) = -1;
        firstRow(nodeCount + cellIndex) = -cellLength_deg / 3;
        secondRow = zeros(1, decisionCount);
        secondRow(cellIndex + 1) = -1;
        secondRow(nodeCount + cellIndex + 1) = cellLength_deg / 3;
        nonnegativeMatrix = [nonnegativeMatrix; ...
            firstRow; secondRow]; %#ok<AGROW>
        continue;
    end
    equalityMatrix(end + 1, :) = row; %#ok<AGROW>
    equalityRightSide(end + 1, 1) = 0; %#ok<AGROW>
end
equalityMatrix = sparse(equalityMatrix);
nonnegativeMatrix = sparse(nonnegativeMatrix);

%% Section 2: Find A Strictly Feasible Initial Profile

finiteVelocity_deg_s = limits.maxVelocity_deg_s(isfinite( ...
    limits.maxVelocity_deg_s));
if isempty(finiteVelocity_deg_s)
    finiteVelocity_deg_s = 1;
end
% Begin near useful path speed, then repeatedly halve the complete profile
% until the original nonlinear collocation constraints accept it.
seedVelocityFraction = 0.8;
seedAmplitude_deg2_s2 = max(1e-6, ...
    (seedVelocityFraction * min(finiteVelocity_deg_s))^2);
cellCap_deg2_s2 = repmat(seedAmplitude_deg2_s2, cellCount, 1);
for cellIndex = 1:cellCount
    pathBound = derivativeBounds(cellIndex);
    for axisIndex = 1:2
        tangent = pathBound.CertifiedTangentByAxis(axisIndex);
        second = pathBound. ...
            CertifiedSecondDerivativeByAxis_deg_inv(axisIndex);
        third = pathBound. ...
            CertifiedThirdDerivativeByAxis_deg_inv2(axisIndex);
        if tangent > 0 && isfinite(limits.maxVelocity_deg_s(axisIndex))
            cellCap_deg2_s2(cellIndex) = min( ...
                cellCap_deg2_s2(cellIndex), ...
                (limits.maxVelocity_deg_s(axisIndex) / tangent)^2);
        end
        if second > 0 && ...
                isfinite(limits.maxAcceleration_deg_s2(axisIndex))
            cellCap_deg2_s2(cellIndex) = min( ...
                cellCap_deg2_s2(cellIndex), ...
                limits.maxAcceleration_deg_s2(axisIndex) / second);
        end
        if third > 0 && isfinite(limits.maxJerk_deg_s3(axisIndex))
            cellCap_deg2_s2(cellIndex) = min( ...
                cellCap_deg2_s2(cellIndex), ...
                (limits.maxJerk_deg_s3(axisIndex) / third)^(2 / 3));
        end
    end
end
nodeCap_deg2_s2 = [cellCap_deg2_s2(1); ...
    min(cellCap_deg2_s2(1:end - 1), cellCap_deg2_s2(2:end)); ...
    cellCap_deg2_s2(end)];
seedB_deg2_s2 = zeros(nodeCount, 1);
seedFirst_deg_s2 = zeros(nodeCount, 1);
blockBoundary = unique([1; find(stopNode); nodeCount]);
for blockIndex = 1:numel(blockBoundary) - 1
    firstIndex = blockBoundary(blockIndex);
    lastIndex = blockBoundary(blockIndex + 1);
    blockLength_deg = gridS_deg(lastIndex) - gridS_deg(firstIndex);
    u = (gridS_deg(firstIndex:lastIndex) - gridS_deg(firstIndex)) / ...
        blockLength_deg;
    startB_deg2_s2 = 0;
    endB_deg2_s2 = 0;
    if firstIndex == 1
        startB_deg2_s2 = endpointSpeed_deg_s(1)^2;
    end
    if lastIndex == nodeCount
        endB_deg2_s2 = endpointSpeed_deg_s(2)^2;
    end
    hump_deg2_s2 = nodeCap_deg2_s2(firstIndex:lastIndex) .* ...
        max(0, 4 * u .* (1 - u)).^(4 / 3);
    seedB_deg2_s2(firstIndex:lastIndex) = ...
        (1 - u) * startB_deg2_s2 + u * endB_deg2_s2 + ...
        hump_deg2_s2;
end
seedB_deg2_s2(stopNode) = 0;
seedB_deg2_s2([1 end]) = endpointSpeed_deg_s.^2;
finiteAcceleration_deg_s2 = limits.maxAcceleration_deg_s2( ...
    isfinite(limits.maxAcceleration_deg_s2));
if ~isempty(finiteAcceleration_deg_s2)
    scalarAcceleration_deg_s2 = min(finiteAcceleration_deg_s2);
    for nodeIndex = 1:nodeCount - 1
        distance_deg = gridS_deg(nodeIndex + 1) - gridS_deg(nodeIndex);
        seedB_deg2_s2(nodeIndex + 1) = min( ...
            seedB_deg2_s2(nodeIndex + 1), ...
            seedB_deg2_s2(nodeIndex) + ...
            2 * scalarAcceleration_deg_s2 * distance_deg);
    end
    for nodeIndex = nodeCount:-1:2
        distance_deg = gridS_deg(nodeIndex) - gridS_deg(nodeIndex - 1);
        seedB_deg2_s2(nodeIndex - 1) = min( ...
            seedB_deg2_s2(nodeIndex - 1), ...
            seedB_deg2_s2(nodeIndex) + ...
            2 * scalarAcceleration_deg_s2 * distance_deg);
    end
end
seedB_deg2_s2([1 end]) = endpointSpeed_deg_s.^2;
for blockIndex = 1:numel(blockBoundary) - 1
    firstIndex = blockBoundary(blockIndex);
    lastIndex = blockBoundary(blockIndex + 1);
    seedFirst_deg_s2(firstIndex:lastIndex) = gradient( ...
        seedB_deg2_s2(firstIndex:lastIndex), ...
        gridS_deg(firstIndex:lastIndex));
end
seedFirst_deg_s2([find(stopNode); 1; nodeCount]) = 0;
for cellIndex = 1:cellCount
    if stopNode(cellIndex) || stopNode(cellIndex + 1)
        continue;
    end
    cellLength_deg = gridS_deg(cellIndex + 1) - gridS_deg(cellIndex);
    seedFirst_deg_s2(cellIndex) = max( ...
        seedFirst_deg_s2(cellIndex), ...
        -3 * seedB_deg2_s2(cellIndex) / cellLength_deg);
    seedFirst_deg_s2(cellIndex + 1) = min( ...
        seedFirst_deg_s2(cellIndex + 1), ...
        3 * seedB_deg2_s2(cellIndex + 1) / cellLength_deg);
end
decision = [seedB_deg2_s2; seedFirst_deg_s2];
if ~isempty(equalityMatrix)
    decision = decision + equalityMatrix' * ...
        ((equalityMatrix * equalityMatrix') \ ...
        (equalityRightSide - equalityMatrix * decision));
end
decision = min(upperBound, max(lowerBound, decision));
[isFeasible, seedViolation] = originalConstraintsSatisfied( ...
    decision, valueMap, firstMap, secondMap, geometry, limits);
isFeasible = isFeasible && ...
    max([0; nonnegativeMatrix * decision]) <= 1e-10;
scalePassCount = 0;
while ~isFeasible && all(endpointSpeed_deg_s == 0) && scalePassCount < 40
    decision = 0.5 * decision;
    scalePassCount = scalePassCount + 1;
    [isFeasible, seedViolation] = originalConstraintsSatisfied( ...
        decision, valueMap, firstMap, secondMap, geometry, limits);
    isFeasible = isFeasible && ...
        max([0; nonnegativeMatrix * decision]) <= 1e-10;
end
if ~isFeasible
    schedule.Message = sprintf( ...
        "No feasible sequential-convex seed; maximum normalized " + ...
        "constraint violation was %.6g.", seedViolation);
    schedule.TerminationReason = "infeasibleInitialization";
    schedule.SeedAmplitude_deg2_s2 = seedAmplitude_deg2_s2;
    return;
end

%% Section 3: Solve The Debrouwere Inner Approximations

maximumIterationCount = 12;
if useQuadraticSubproblem
    maximumIterationCount = 80;
end
timeTolerance_s = 1e-5;
constraintTolerance = 1e-8;
regularization = 1e-8 / max(1, nodeCount);
if useQuadraticSubproblem
    regularization = 1e-6 / max(1, nodeCount);
end
iterationTime_s = NaN(maximumIterationCount, 1);
solverExitFlag = NaN(maximumIterationCount, 1);
solverIterationCount = zeros(maximumIterationCount, 1);
previousTime_s = Inf;
converged = false;
lastOutput = struct("iterations", 0, "message", "Not solved.");

if useQuadraticSubproblem
    solverOptions = optimoptions("quadprog", ...
        "Algorithm", "interior-point-convex", ...
        "Display", "none", ...
        "MaxIterations", 300, ...
        "ConstraintTolerance", constraintTolerance, ...
        "OptimalityTolerance", 1e-7, ...
        "StepTolerance", 1e-10);
else
    solverOptions = optimoptions("fmincon", ...
        "Algorithm", "sqp", ...
        "Display", "none", ...
        "SpecifyObjectiveGradient", true, ...
        "MaxIterations", 300, ...
        "MaxFunctionEvaluations", max(5000, 80 * decisionCount), ...
        "ConstraintTolerance", constraintTolerance, ...
        "OptimalityTolerance", 1e-7, ...
        "StepTolerance", 1e-10, ...
        "ScaleProblem", true);
end
constraintGeometry = certifiedGeometry;
useSignedGeometry = false;
% Tight intermediate-grid intercepts benefit from envelope-guided steps.
% At 100 decisions, signed paper equations avoid excessive conservatism;
% the continuous envelope certificate still controls returned success.
signedGeometryDecisionThreshold = 100;
if decisionCount >= signedGeometryDecisionThreshold
    constraintGeometry = geometry;
    useSignedGeometry = true;
end
for iterationIndex = 1:maximumIterationCount
    current = decision;
    currentB = max(valueMap * current, realmin("double")^(1 / 3));
    inverseRoot = currentB .^ (-0.5);
    inverseRootSlope = -0.5 * currentB .^ (-1.5);
    [inequalityMatrix, inequalityRightSide] = linearizedConstraints( ...
        valueMap, firstMap, secondMap, constraintGeometry, limits, ...
        current, currentB, inverseRoot, inverseRootSlope, ...
        useSignedGeometry);
    subproblemMatrix = [nonnegativeMatrix; inequalityMatrix];
    subproblemRightSide = [zeros(size(nonnegativeMatrix, 1), 1); ...
        inequalityRightSide];
    try
        if useQuadraticSubproblem
            [hessian, linearTerm] = travelTimeQuadraticModel( ...
                current, objectiveValueMap, quadratureWeight_deg, ...
                regularization);
            [decision, ~, exitFlag, output] = quadprog( ...
                hessian, linearTerm, subproblemMatrix, ...
                subproblemRightSide, equalityMatrix, equalityRightSide, ...
                lowerBound, upperBound, current, solverOptions);
        else
            objective = @(candidate) travelTimeObjective( ...
                candidate, objectiveValueMap, quadratureWeight_deg, ...
                current, regularization);
            [decision, ~, exitFlag, output] = fmincon( ...
                objective, current, full(subproblemMatrix), ...
                subproblemRightSide, full(equalityMatrix), ...
                equalityRightSide, lowerBound, upperBound, [], ...
                solverOptions);
        end
    catch exception
        schedule.Message = "The convex subproblem failed: " + ...
            string(exception.message);
        schedule.TerminationReason = "solverFailed";
        return;
    end
    lastOutput = output;
    solverExitFlag(iterationIndex) = exitFlag;
    solverIterationCount(iterationIndex) = output.iterations;
    [isFeasible, ~] = originalConstraintsSatisfied( ...
        decision, valueMap, firstMap, secondMap, geometry, limits);
    isFeasible = isFeasible && ...
        max([0; nonnegativeMatrix * decision]) <= 1e-10;
    if ~isFeasible && exitFlag > 0
        fullStep = decision - current;
        stepFraction = 1;
        for backtrackIndex = 1:24
            stepFraction = 0.5 * stepFraction;
            decision = current + stepFraction * fullStep;
            [isFeasible, ~] = ...
                originalConstraintsSatisfied( ...
                decision, valueMap, firstMap, secondMap, geometry, limits);
            isFeasible = isFeasible && ...
                max([0; nonnegativeMatrix * decision]) <= 1e-10;
            if isFeasible
                break;
            end
        end
    end
    if exitFlag <= 0 || ~isFeasible
        % Every accepted paper iterate is feasible. A numerical failure in
        % the next convex subproblem must not discard that valid schedule.
        % The continuous certificate below still decides returned success.
        decision = current;
        iterationTime_s(iterationIndex) = sum( ...
            quadratureWeight_deg ./ sqrt( ...
            objectiveValueMap * decision));
        break;
    end
    iterationTime_s(iterationIndex) = sum( ...
        quadratureWeight_deg ./ sqrt(objectiveValueMap * decision));
    if abs(previousTime_s - iterationTime_s(iterationIndex)) <= ...
            timeTolerance_s
        converged = true;
        break;
    end
    previousTime_s = iterationTime_s(iterationIndex);
end
iterationCount = find(isfinite(iterationTime_s), 1, "last");
if isempty(iterationCount)
    schedule.Message = "The sequential-convex retimer did not iterate.";
    schedule.TerminationReason = "solverFailed";
    return;
end

%% Section 4: Certify And Sample The Continuous Profile

[peakVelocity_deg_s, peakAcceleration_deg_s2, peakJerk_deg_s3, ...
    timeScale, certificatePassed] = certifyProfile( ...
    decision, gridS_deg, stopNode, derivativeBounds, limits);
if timeScale > 1 + 1e-10
    if any(endpointSpeed_deg_s > 1e-10)
        schedule.Message = "Continuous certification requires time " + ...
            "dilation, but an endpoint speed is fixed and nonzero.";
        schedule.TerminationReason = "continuousCertificateFailed";
        return;
    end
    decision = decision / timeScale^2;
    [peakVelocity_deg_s, peakAcceleration_deg_s2, peakJerk_deg_s3, ...
        secondScale, certificatePassed] = certifyProfile( ...
        decision, gridS_deg, stopNode, derivativeBounds, limits);
    timeScale = timeScale * secondScale;
end
if ~certificatePassed
    schedule.Message = "The continuous derivative certificate failed.";
    schedule.TerminationReason = "continuousCertificateFailed";
    return;
end

[time_s, arcLength_deg, speed_deg_s, acceleration_deg_s2, ...
    jerk_deg_s3, nodeTime_s] = sampleProfile( ...
    decision, gridS_deg, stopNode);
schedule.Success = true;
schedule.Message = "Debrouwere sequential-convex retiming succeeded.";
schedule.TerminationReason = "goalReached";
schedule.time_s = time_s;
schedule.arcLength_deg = arcLength_deg;
schedule.speed_deg_s = speed_deg_s;
schedule.acceleration_deg_s2 = acceleration_deg_s2;
schedule.jerk_deg_s3 = jerk_deg_s3;
schedule.NodeArcLength_deg = gridS_deg;
schedule.NodeTime_s = nodeTime_s;
schedule.NodeSpeed_deg_s = sqrt(max(0, decision(1:nodeCount)));
schedule.NodeAcceleration_deg_s2 = 0.5 * ...
    decision(nodeCount + 1:end);
schedule.MinimumMotionDuration_s = time_s(end);
schedule.PeakVelocity_deg_s = peakVelocity_deg_s;
schedule.PeakAcceleration_deg_s2 = peakAcceleration_deg_s2;
schedule.PeakJerk_deg_s3 = peakJerk_deg_s3;
schedule.TimeScaleFactor = timeScale;
schedule.IterationCount = iterationCount;
schedule.Converged = converged;
schedule.IterationTime_s = iterationTime_s(1:iterationCount);
schedule.SolverExitFlag = solverExitFlag(1:iterationCount);
schedule.SolverIterationCount = solverIterationCount(1:iterationCount);
schedule.SolverMessage = string(lastOutput.message);
schedule.SeedAmplitude_deg2_s2 = seedAmplitude_deg2_s2;
schedule.ReferenceDOI = "10.1109/TRO.2013.2277565";
schedule.CacheHit = false;
maximumCacheEntryCount = 32;
cachedKeys{end + 1, 1} = cacheKey;
cachedSchedules{end + 1, 1} = schedule;
if numel(cachedKeys) > maximumCacheEntryCount
    cachedKeys(1) = [];
    cachedSchedules(1) = [];
end
end

%% Section 5: Local Functions

function [valueMap, firstMap, secondMap] = hermiteMaps( ...
        gridS_deg, queryS_deg, stopNode)
% PURPOSE
%   - Map node b and b' values to a continuous spatial profile.
nodeCount = numel(gridS_deg);
queryCount = numel(queryS_deg);
valueMap = zeros(queryCount, 2 * nodeCount);
firstMap = zeros(queryCount, 2 * nodeCount);
secondMap = zeros(queryCount, 2 * nodeCount);
for queryIndex = 1:queryCount
    cellIndex = find(gridS_deg <= queryS_deg(queryIndex), 1, "last");
    cellIndex = min(cellIndex, nodeCount - 1);
    cellLength_deg = gridS_deg(cellIndex + 1) - gridS_deg(cellIndex);
    u = (queryS_deg(queryIndex) - gridS_deg(cellIndex)) / ...
        cellLength_deg;
    u = min(1, max(0, u));
    if stopNode(cellIndex) && ~stopNode(cellIndex + 1)
        valueMap(queryIndex, cellIndex + 1) = u^(4 / 3);
        firstMap(queryIndex, cellIndex + 1) = ...
            4 * u^(1 / 3) / (3 * cellLength_deg);
        secondMap(queryIndex, cellIndex + 1) = ...
            4 * u^(-2 / 3) / (9 * cellLength_deg^2);
    elseif ~stopNode(cellIndex) && stopNode(cellIndex + 1)
        valueMap(queryIndex, cellIndex) = (1 - u)^(4 / 3);
        firstMap(queryIndex, cellIndex) = ...
            -4 * (1 - u)^(1 / 3) / (3 * cellLength_deg);
        secondMap(queryIndex, cellIndex) = ...
            4 * (1 - u)^(-2 / 3) / (9 * cellLength_deg^2);
    else
        valueBasis = [2*u^3 - 3*u^2 + 1, -2*u^3 + 3*u^2];
        slopeBasis = [u^3 - 2*u^2 + u, u^3 - u^2];
        firstValueBasis = [6*u^2 - 6*u, -6*u^2 + 6*u] / ...
            cellLength_deg;
        firstSlopeBasis = [3*u^2 - 4*u + 1, 3*u^2 - 2*u];
        secondValueBasis = [12*u - 6, -12*u + 6] / ...
            cellLength_deg^2;
        secondSlopeBasis = [6*u - 4, 6*u - 2] / cellLength_deg;
        nodeIndex = [cellIndex, cellIndex + 1];
        valueMap(queryIndex, nodeIndex) = valueBasis;
        valueMap(queryIndex, nodeCount + nodeIndex) = ...
            cellLength_deg * slopeBasis;
        firstMap(queryIndex, nodeIndex) = firstValueBasis;
        firstMap(queryIndex, nodeCount + nodeIndex) = firstSlopeBasis;
        secondMap(queryIndex, nodeIndex) = secondValueBasis;
        secondMap(queryIndex, nodeCount + nodeIndex) = secondSlopeBasis;
    end
end
valueMap = sparse(valueMap);
firstMap = sparse(firstMap);
secondMap = sparse(secondMap);
end

function [matrix, rightSide] = linearizedConstraints(valueMap, ...
        firstMap, secondMap, geometry, limits, current, currentB, ...
        inverseRoot, inverseRootSlope, useSignedGeometry)
% PURPOSE
%   - Assemble certified velocity, acceleration, and paper DC jerk
%     inequalities without relying on cancellation between path terms.
matrix = -valueMap;
rightSide = zeros(size(valueMap, 1), 1);
for axisIndex = 1:2
    tangent = geometry.tangent(:, axisIndex);
    second = geometry.secondDerivative_deg_inv(:, axisIndex);
    third = geometry.thirdDerivative_deg_inv2(:, axisIndex);
    if isfinite(limits.maxVelocity_deg_s(axisIndex))
        matrix = [matrix; tangent.^2 .* valueMap]; %#ok<AGROW>
        rightSide = [rightSide; repmat( ...
            limits.maxVelocity_deg_s(axisIndex)^2, ...
            size(valueMap, 1), 1)]; %#ok<AGROW>
    end
    if isfinite(limits.maxAcceleration_deg_s2(axisIndex))
        if useSignedGeometry
            accelerationMap = second .* valueMap + ...
                0.5 * tangent .* firstMap;
            matrix = [matrix; accelerationMap; -accelerationMap]; ...
                %#ok<AGROW>
        else
            curvatureMap = second .* valueMap;
            tangentialMap = 0.5 * tangent .* firstMap;
            matrix = [matrix; curvatureMap + tangentialMap; ...
                curvatureMap - tangentialMap]; %#ok<AGROW>
        end
        rightSide = [rightSide; repmat( ...
            limits.maxAcceleration_deg_s2(axisIndex), ...
            2 * size(valueMap, 1), 1)]; %#ok<AGROW>
    end
    if isfinite(limits.maxJerk_deg_s3(axisIndex))
        jerkLimit = limits.maxJerk_deg_s3(axisIndex);
        tangentRightSide = jerkLimit * (inverseRoot - ...
            inverseRootSlope .* currentB);
        if useSignedGeometry
            jerkLinearMap = third .* valueMap + ...
                1.5 * second .* firstMap + ...
                0.5 * tangent .* secondMap;
            inverseRootMap = jerkLimit * inverseRootSlope .* valueMap;
            matrix = [matrix; jerkLinearMap - inverseRootMap; ...
                -jerkLinearMap - inverseRootMap]; %#ok<AGROW>
            rightSide = [rightSide; tangentRightSide; ...
                tangentRightSide]; %#ok<AGROW>
        else
            firstSign = sign(firstMap * current);
            secondSign = sign(secondMap * current);
            firstSign(firstSign == 0) = 1;
            secondSign(secondSign == 0) = 1;
            jerkLinearMap = third .* valueMap + ...
                1.5 * second .* firstSign .* firstMap + ...
                0.5 * tangent .* secondSign .* secondMap;
            adjustedMap = jerkLinearMap - ...
                jerkLimit * inverseRootSlope .* valueMap;
            matrix = [matrix; adjustedMap]; %#ok<AGROW>
            rightSide = [rightSide; tangentRightSide]; %#ok<AGROW>
        end
    end
end
end

function [hessian, linearTerm] = travelTimeQuadraticModel( ...
        center, valueMap, quadratureWeight_deg, regularization)
% PURPOSE
%   - Form a convex second-order model of travel time at the SCP center.
b = max(valueMap * center, realmin("double")^(1 / 3));
gradient = -0.5 * valueMap' * ...
    (quadratureWeight_deg ./ b.^(3 / 2));
curvature = 0.75 * quadratureWeight_deg ./ b.^(5 / 2);
hessian = valueMap' * spdiags(curvature, 0, numel(curvature), ...
    numel(curvature)) * valueMap;
diagonalScale = max(1, max(abs(diag(hessian))));
hessian = hessian + regularization * diagonalScale * ...
    speye(numel(center));
hessian = 0.5 * (hessian + hessian');
linearTerm = gradient - hessian * center;
end

function [objective, gradient] = travelTimeObjective(decision, valueMap, ...
        quadratureWeight_deg, center, regularization)
% PURPOSE
%   - Evaluate the exact convex travel-time objective and its gradient.
b = max(valueMap * decision, realmin("double")^(1 / 3));
difference = decision - center;
objective = sum(quadratureWeight_deg ./ sqrt(b)) + ...
    0.5 * regularization * sum(difference.^2);
gradient = -0.5 * valueMap' * ...
    (quadratureWeight_deg ./ b.^(3 / 2)) + ...
    regularization * difference;
end

function [satisfied, maximumViolation] = originalConstraintsSatisfied( ...
        decision, valueMap, firstMap, secondMap, geometry, limits)
% PURPOSE
%   - Check the nonlinear constraints before accepting an SCP iterate.
b = valueMap * decision;
first = firstMap * decision;
second = secondMap * decision;
maximumViolation = max(0, -min(b));
for axisIndex = 1:2
    velocity = abs(geometry.tangent(:, axisIndex)) .* sqrt(max(0, b));
    acceleration = abs(geometry.secondDerivative_deg_inv(:, axisIndex) .* ...
        b + 0.5 * geometry.tangent(:, axisIndex) .* first);
    jerk = abs(sqrt(max(0, b)) .* ( ...
        geometry.thirdDerivative_deg_inv2(:, axisIndex) .* b + ...
        1.5 * geometry.secondDerivative_deg_inv(:, axisIndex) .* first + ...
        0.5 * geometry.tangent(:, axisIndex) .* second));
    maximumViolation = max([maximumViolation; ...
        max(velocity / limits.maxVelocity_deg_s(axisIndex) - 1); ...
        max(acceleration / limits.maxAcceleration_deg_s2(axisIndex) - 1); ...
        max(jerk / limits.maxJerk_deg_s3(axisIndex) - 1)]);
end
satisfied = maximumViolation <= 1e-7;
end

function [peakVelocity_deg_s, peakAcceleration_deg_s2, ...
        peakJerk_deg_s3, scaleFactor, passed] = certifyProfile( ...
        decision, gridS_deg, stopNode, derivativeBounds, limits)
% PURPOSE
%   - Bound every derivative continuously over every spatial cell.
nodeCount = numel(gridS_deg);
b = decision(1:nodeCount);
first = decision(nodeCount + 1:end);
peakVelocity_deg_s = [0 0];
peakAcceleration_deg_s2 = [0 0];
peakJerk_deg_s3 = [0 0];
profileIsNonnegative = true;
for cellIndex = 1:nodeCount - 1
    cellLength_deg = gridS_deg(cellIndex + 1) - gridS_deg(cellIndex);
    if stopNode(cellIndex) && ~stopNode(cellIndex + 1)
        maximumB = b(cellIndex + 1);
        maximumFirst = 4 * maximumB / (3 * cellLength_deg);
        maximumScalarJerk = 2 * maximumB^(3 / 2) / ...
            (9 * cellLength_deg^2);
    elseif ~stopNode(cellIndex) && stopNode(cellIndex + 1)
        maximumB = b(cellIndex);
        maximumFirst = 4 * maximumB / (3 * cellLength_deg);
        maximumScalarJerk = 2 * maximumB^(3 / 2) / ...
            (9 * cellLength_deg^2);
    else
        c0 = b(cellIndex);
        c1 = cellLength_deg * first(cellIndex);
        c2 = -3*b(cellIndex) - 2*c1 + 3*b(cellIndex + 1) - ...
            cellLength_deg * first(cellIndex + 1);
        c3 = 2*b(cellIndex) + c1 - 2*b(cellIndex + 1) + ...
            cellLength_deg * first(cellIndex + 1);
        extrema = roots([3*c3, 2*c2, c1]);
        extrema = real(extrema(abs(imag(extrema)) <= 1e-12 & ...
            real(extrema) >= 0 & real(extrema) <= 1));
        evaluation = [0; 1; extrema(:)];
        value = c0 + c1*evaluation + c2*evaluation.^2 + ...
            c3*evaluation.^3;
        profileIsNonnegative = profileIsNonnegative && ...
            min(value) >= -1e-10;
        maximumB = max(value);
        secondRoot = zeros(0, 1);
        if abs(c3) > eps(max(1, abs(c2)))
            candidate = -c2 / (3*c3);
            if candidate >= 0 && candidate <= 1
                secondRoot = candidate;
            end
        end
        derivativeEvaluation = [0; 1; secondRoot];
        firstValue = (c1 + 2*c2*derivativeEvaluation + ...
            3*c3*derivativeEvaluation.^2) / cellLength_deg;
        maximumFirst = max(abs(firstValue));
        secondPolynomial = [6*c3, 2*c2];
        if any(secondPolynomial ~= 0)
            bPolynomial = [c3, c2, c1, c0];
            jerkSquaredPolynomial = conv(bPolynomial, ...
                conv(secondPolynomial, secondPolynomial)) / ...
                (4 * cellLength_deg^4);
            jerkStationary = roots(polyder(jerkSquaredPolynomial));
            jerkStationary = real(jerkStationary( ...
                abs(imag(jerkStationary)) <= 1e-10 & ...
                real(jerkStationary) >= 0 & real(jerkStationary) <= 1));
            jerkEvaluation = [0; 1; jerkStationary(:)];
            maximumScalarJerk = sqrt(max(0, max( ...
                polyval(jerkSquaredPolynomial, jerkEvaluation))));
        else
            maximumScalarJerk = 0;
        end
    end
    pathBound = derivativeBounds(cellIndex);
    tangent = pathBound.CertifiedTangentByAxis;
    second = pathBound.CertifiedSecondDerivativeByAxis_deg_inv;
    third = pathBound.CertifiedThirdDerivativeByAxis_deg_inv2;
    speed = sqrt(maximumB);
    scalarAcceleration = 0.5 * maximumFirst;
    velocityBound = tangent * speed;
    accelerationBound = tangent * scalarAcceleration + second * maximumB;
    jerkBound = tangent * maximumScalarJerk + ...
        1.5 * second * speed * maximumFirst + third * speed^3;
    peakVelocity_deg_s = max(peakVelocity_deg_s, velocityBound);
    peakAcceleration_deg_s2 = max( ...
        peakAcceleration_deg_s2, accelerationBound);
    peakJerk_deg_s3 = max(peakJerk_deg_s3, jerkBound);
end
velocityRatio = max(peakVelocity_deg_s ./ limits.maxVelocity_deg_s);
accelerationRatio = max( ...
    peakAcceleration_deg_s2 ./ limits.maxAcceleration_deg_s2);
jerkRatio = max(peakJerk_deg_s3 ./ limits.maxJerk_deg_s3);
scaleFactor = max([1, velocityRatio, sqrt(accelerationRatio), ...
    nthroot(jerkRatio, 3)]) * (1 + 64 * eps);
passed = profileIsNonnegative && isfinite(scaleFactor) && ...
    scaleFactor <= 1 + 1e-8;
end

function [time_s, arcLength_deg, speed_deg_s, acceleration_deg_s2, ...
        jerk_deg_s3, nodeTime_s] = sampleProfile( ...
        decision, gridS_deg, stopNode)
% PURPOSE
%   - Invert the certified spatial speed law into a sampled time history.
nodeCount = numel(gridS_deg);
samplesPerCell = 65;
denseS_deg = zeros(0, 1);
denseTime_s = zeros(0, 1);
currentTime_s = 0;
nodeTime_s = zeros(nodeCount, 1);
for cellIndex = 1:nodeCount - 1
    cellLength_deg = gridS_deg(cellIndex + 1) - gridS_deg(cellIndex);
    u = linspace(0, 1, samplesPerCell).';
    localS_deg = gridS_deg(cellIndex) + cellLength_deg * u;
    [valueMap, ~, ~] = hermiteMaps( ...
        gridS_deg, localS_deg, stopNode);
    localB = max(0, valueMap * decision);
    if stopNode(cellIndex) && ~stopNode(cellIndex + 1)
        duration_s = 3 * cellLength_deg / sqrt(localB(end));
        localTime_s = duration_s * u.^(1 / 3);
    elseif ~stopNode(cellIndex) && stopNode(cellIndex + 1)
        duration_s = 3 * cellLength_deg / sqrt(localB(1));
        localTime_s = duration_s * (1 - (1 - u).^(1 / 3));
    else
        localSpeed_deg_s = sqrt(localB);
        localTime_s = zeros(size(u));
        for sampleIndex = 2:samplesPerCell
            averageSpeed_deg_s = 0.5 * ( ...
                localSpeed_deg_s(sampleIndex - 1) + ...
                localSpeed_deg_s(sampleIndex));
            localTime_s(sampleIndex) = localTime_s(sampleIndex - 1) + ...
                (localS_deg(sampleIndex) - ...
                localS_deg(sampleIndex - 1)) / averageSpeed_deg_s;
        end
    end
    localTime_s = currentTime_s + localTime_s;
    keep = 1:samplesPerCell;
    if cellIndex > 1
        keep = 2:samplesPerCell;
    end
    denseS_deg = [denseS_deg; localS_deg(keep)]; %#ok<AGROW>
    denseTime_s = [denseTime_s; localTime_s(keep)]; %#ok<AGROW>
    currentTime_s = localTime_s(end);
    nodeTime_s(cellIndex + 1) = currentTime_s;
end
sampleTime_s = min(0.02, max(1e-4, denseTime_s(end) / 2000));
time_s = unique([0; (0:sampleTime_s:denseTime_s(end)).'; ...
    nodeTime_s; denseTime_s(end)]);
arcLength_deg = interp1(denseTime_s, denseS_deg, time_s, "pchip");
[valueMap, firstMap, secondMap] = hermiteMaps( ...
    gridS_deg, arcLength_deg, stopNode);
b = max(0, valueMap * decision);
first = firstMap * decision;
second = secondMap * decision;
speed_deg_s = sqrt(b);
acceleration_deg_s2 = 0.5 * first;
second(~isfinite(second)) = 0;
jerk_deg_s3 = 0.5 * speed_deg_s .* second;
stopArcLength_deg = gridS_deg(stopNode);
for stopIndex = 1:numel(stopArcLength_deg)
    belongs = abs(arcLength_deg - stopArcLength_deg(stopIndex)) <= ...
        16 * eps(max(1, abs(stopArcLength_deg(stopIndex))));
    nodeIndex = find(gridS_deg == stopArcLength_deg(stopIndex), 1);
    boundaryJerk_deg_s3 = 0;
    if nodeIndex > 1 && ~stopNode(nodeIndex - 1)
        cellLength_deg = gridS_deg(nodeIndex) - gridS_deg(nodeIndex - 1);
        boundaryJerk_deg_s3 = max(boundaryJerk_deg_s3, ...
            2 * decision(nodeIndex - 1)^(3 / 2) / ...
            (9 * cellLength_deg^2));
    end
    if nodeIndex < nodeCount && ~stopNode(nodeIndex + 1)
        cellLength_deg = gridS_deg(nodeIndex + 1) - gridS_deg(nodeIndex);
        boundaryJerk_deg_s3 = max(boundaryJerk_deg_s3, ...
            2 * decision(nodeIndex + 1)^(3 / 2) / ...
            (9 * cellLength_deg^2));
    end
    speed_deg_s(belongs) = 0;
    acceleration_deg_s2(belongs) = 0;
    jerk_deg_s3(belongs) = boundaryJerk_deg_s3;
end
if any(~isfinite([speed_deg_s; acceleration_deg_s2; jerk_deg_s3]))
    error("retimeAzElSequentialConvex:NonfiniteSample", ...
        "The sampled spatial schedule contains a nonfinite derivative.");
end
end

function schedule = emptySchedule()
% PURPOSE
%   - Define the stable scalar schedule schema for every outcome.
schedule = struct( ...
    "Success", false, ...
    "Message", "Sequential-convex retiming was not evaluated.", ...
    "TerminationReason", "notEvaluated", ...
    "time_s", zeros(0, 1), ...
    "arcLength_deg", zeros(0, 1), ...
    "speed_deg_s", zeros(0, 1), ...
    "acceleration_deg_s2", zeros(0, 1), ...
    "jerk_deg_s3", zeros(0, 1), ...
    "NodeArcLength_deg", zeros(0, 1), ...
    "NodeTime_s", zeros(0, 1), ...
    "NodeSpeed_deg_s", zeros(0, 1), ...
    "NodeAcceleration_deg_s2", zeros(0, 1), ...
    "MinimumMotionDuration_s", NaN, ...
    "PeakVelocity_deg_s", [NaN NaN], ...
    "PeakAcceleration_deg_s2", [NaN NaN], ...
    "PeakJerk_deg_s3", [NaN NaN], ...
    "TimeScaleFactor", NaN, ...
    "IterationCount", 0, ...
    "Converged", false, ...
    "IterationTime_s", zeros(0, 1), ...
    "SolverExitFlag", zeros(0, 1), ...
    "SolverIterationCount", zeros(0, 1), ...
    "SolverMessage", "", ...
    "SeedAmplitude_deg2_s2", NaN, ...
    "ReferenceDOI", "10.1109/TRO.2013.2277565", ...
    "CacheHit", false);
end
