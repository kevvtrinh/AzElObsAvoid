function [bestMotion, bestValidation, diagnostics, preparation] = solveCompactC3( ...
        seed, upperDuration_s, obstacles, initialState, goalState, limits, ...
        plannerOptions, preparation)
%% Section 0: Header & Readme
% SYNTAX
%   [bestMotion, bestValidation, diagnostics, preparation] = ...
%       azElPlannerMethods.corridor.internal.motion.solveCompactC3( ...
%       seed, upperDuration_s, obstacles, initialState, goalState, limits, ...
%       plannerOptions, preparation)
%**************************************************************************
% PURPOSE
%   - Improve one already feasible short topology with a compact quintic spline.
%     The routine fits an initial control polygon, solves bounded linearized
%     quadratic programs at trial durations, validates every trial, and keeps
%     only an earlier independently valid motion.
%**************************************************************************
% INPUTS
%   - seed (scalar struct), route positions and normalized route time.
%   - upperDuration_s (positive scalar), retained candidate duration bound.
%   - obstacles (prepared struct array), protected geometry histories.
%   - initialState, goalState, limits, plannerOptions (scalar structs).
%   - preparation (scalar struct or [], optional), reusable affine basis.
%**************************************************************************
% OUTPUTS
%   - bestMotion (scalar struct), earliest retained motion candidate.
%   - bestValidation (scalar struct), independent validation record.
%   - diagnostics (scalar struct), solve evidence and five-stage timing.
%   - preparation (scalar struct), reusable duration-independent setup.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s powers.
%**************************************************************************

%% Section 1: Fit A Fixed-Size Spline To The Seed Route

compactTimer = tic;
stageTiming = azElPlannerMethods.internal.stageTiming();
basisBuilt = nargin < 8 || isempty(preparation);
if basisBuilt
corridorTimer = tic;
geometryIsStatic = isempty(obstacles) || ...
    all(arrayfun(@(obstacle) all( ...
    obstacle.InternalPreparation.IntervalSpeedBound_deg_s == 0), obstacles));
geometryDeforms = any(arrayfun(@obstacleDeforms, obstacles));
hasExplicitHold = any(vecnorm(diff(seed.position_deg), 2, 2) <= 1e-12);
endpointDerivativesZero = max(abs([ ...
    initialState.velocity_deg_s, initialState.acceleration_deg_s2, ...
    goalState.velocity_deg_s, goalState.acceleration_deg_s2])) <= 1e-12;
useSeedAnchoredBarrier = ~geometryIsStatic && geometryDeforms && ...
    ~hasExplicitHold && size(seed.position_deg, 1) > 2;
useExactCorridor = geometryIsStatic && ~hasExplicitHold && ...
    endpointDerivativesZero && size(seed.position_deg, 1) > 10;
if ~isempty(obstacles) && useExactCorridor
    seed.position_deg = ...
        azElPlannerMethods.corridor.internal.motion.expandRouteClearance( ...
        seed.position_deg, obstacles, initialState.time_s, 0.02, 0.02);
end
stageTiming.CorridorConstructionElapsedTime_s = toc(corridorTimer);
motionTimer = tic;
% Short or dynamic routes use a fixed C3 basis; larger static routes align
% one compact C4 span and one exact corridor support with each route edge.
if useExactCorridor || useSeedAnchoredBarrier
    spanCount = size(seed.position_deg, 1) - 1;
    controlCount = spanCount + 5;
    if useExactCorridor
        representation = "C4ExactCorridor";
    else
        representation = "C4SeedAnchoredBarrier";
    end
else
    spanCount = 8;
    controlCount = 2 * spanCount + 4;
    representation = "C3SampledBarrier";
end
normalizedSampleTime = linspace(0, 1, 257).';
if useExactCorridor || useSeedAnchoredBarrier
    edgeLength_deg = vecnorm(diff(seed.position_deg), 2, 2);
    normalizedSpanDuration = edgeLength_deg .^ 1.05;
    normalizedSpanDuration = normalizedSpanDuration / ...
        sum(normalizedSpanDuration);
    routeTau = [0; cumsum(normalizedSpanDuration)];
    routeTau(end) = 1;
else
    normalizedSpanDuration = ones(spanCount, 1) / spanCount;
    routeTau = seed.tau;
end
affineModel = ...
    azElPlannerMethods.corridor.internal.motion.buildFixedDurationAffineModel( ...
    controlCount, 0, normalizedSpanDuration, normalizedSampleTime);
interiorIndex = affineModel.VariableControlIndex;
fixedIndex = affineModel.FixedControlIndex;
controlPoint_deg = [repmat(initialState.position_deg, 3, 1); ...
    zeros(affineModel.VariableControlCount, 2); ...
    repmat(goalState.position_deg, 3, 1)];
fitBasis = affineModel.PositionBasis;
desiredPosition_deg = interp1( ...
    routeTau, seed.position_deg, affineModel.SampleTime_s, "linear");
fixedContribution_deg = fitBasis(:, fixedIndex) * controlPoint_deg(fixedIndex, :);
if numel(interiorIndex) == size(seed.position_deg, 1) - 2
    controlPoint_deg(interiorIndex, :) = seed.position_deg(2:end - 1, :);
else
    controlPoint_deg(interiorIndex, :) = ...
        fitBasis(:, interiorIndex) \ ...
        (desiredPosition_deg - fixedContribution_deg);
end
straight = azElPlannerMethods.corridor.internal.motion.buildQuinticSpline( ...
    [initialState.position_deg; goalState.position_deg], ...
    initialState, goalState, limits, struct( ...
    "SampleTime_s", plannerOptions.SampleTime_s, ...
    "GoalTimeMode", "earliestArrival", "AllowAzimuthWrapping", plannerOptions.AllowAzimuthWrapping));
physicalLowerDuration_s = straight.MotionDuration_s;
stageTiming.MotionSolvingElapsedTime_s = toc(motionTimer);
corridorTimer = tic;
if useExactCorridor
    [staticBarrierMatrix, staticBarrierBound, useStaticBarrier, ...
        staticCorridorBoundary_deg, staticCorridor] = ...
        staticCorridorRows(seed, affineModel, controlPoint_deg, obstacles);
    derivativeLimitScale = 0.99;
else
    [staticBarrierMatrix, staticBarrierBound, useStaticBarrier, ...
        staticCorridorBoundary_deg, staticCorridor] = ...
        staticCorridorRows(seed, affineModel, controlPoint_deg, obstacles([]));
    derivativeLimitScale = 0.999;
end
stageTiming.CorridorConstructionElapsedTime_s = ...
    stageTiming.CorridorConstructionElapsedTime_s + toc(corridorTimer);
preparation = struct( ...
    "SpanCount", spanCount, "Representation", representation, ...
    "UseExactCorridor", useExactCorridor, ...
    "AffineModel", affineModel, "ControlPoint_deg", controlPoint_deg, ...
    "DesiredPosition_deg", desiredPosition_deg, ...
    "PhysicalLowerDuration_s", physicalLowerDuration_s, ...
    "StaticBarrierMatrix", staticBarrierMatrix, ...
    "StaticBarrierBound", staticBarrierBound, ...
    "UseStaticBarrier", useStaticBarrier, ...
    "StaticCorridorBoundary_deg", staticCorridorBoundary_deg, ...
    "StaticCorridor", staticCorridor, ...
    "DerivativeLimitScale", derivativeLimitScale, ...
    "UseSeedAnchoredBarrier", useSeedAnchoredBarrier);
else
    spanCount = preparation.SpanCount;
    representation = preparation.Representation;
    useExactCorridor = preparation.UseExactCorridor;
    affineModel = preparation.AffineModel;
    controlPoint_deg = preparation.ControlPoint_deg;
    desiredPosition_deg = preparation.DesiredPosition_deg;
    physicalLowerDuration_s = preparation.PhysicalLowerDuration_s;
    staticBarrierMatrix = preparation.StaticBarrierMatrix;
    staticBarrierBound = preparation.StaticBarrierBound;
    useStaticBarrier = preparation.UseStaticBarrier;
    staticCorridorBoundary_deg = preparation.StaticCorridorBoundary_deg;
    staticCorridor = preparation.StaticCorridor;
    derivativeLimitScale = preparation.DerivativeLimitScale;
    useSeedAnchoredBarrier = preparation.UseSeedAnchoredBarrier;
end

%% Section 2: Search A Bounded Duration Bracket

% A valid existing candidate supplies the upper duration. The straight motion
% supplies a physical lower reference but may cross obstacles. Each trial is
% therefore accepted only after the public continuous validator passes.
bestMotion = [];
bestValidation = azElPlannerMethods.corridor.validateTrajectory();
initialDuration_s = upperDuration_s;
if plannerOptions.GoalTimeMode == "fixedArrival"
    duration_s = upperDuration_s;
else
    duration_s = physicalLowerDuration_s + ...
        0.1 * (upperDuration_s - physicalLowerDuration_s);
end
failedDuration_s = physicalLowerDuration_s;
qpCount = 0;
linearProgramCount = 0;
trialCount = 0;
lastExitFlag = NaN;
bestExitFlag = NaN;
if plannerOptions.GoalTimeMode == "fixedArrival"
    maximumTrialCount = 1;
elseif useExactCorridor
    maximumTrialCount = 6;
else
    maximumTrialCount = 14;
end
trialDuration_s = NaN(maximumTrialCount, 1);
trialValidationPassed = false(maximumTrialCount, 1);
trialExitFlag = NaN(maximumTrialCount, 1);
trialQpCounts = zeros(maximumTrialCount, 1);
trialElapsedTimes_s = NaN(maximumTrialCount, 1);

% Retain only duration trials that pass independent validation.
for trialIndex = 1:maximumTrialCount
    trialTimer = tic;
    [trialMotion, trialControlPoint_deg, lastExitFlag, trialQpCount, ...
        trialLinearProgramCount, corridorConstructionElapsedTime_s] = solveDuration( ...
        controlPoint_deg, duration_s, obstacles, initialState, goalState, limits, ...
        plannerOptions, affineModel, desiredPosition_deg, ...
        staticBarrierMatrix, staticBarrierBound, useStaticBarrier, ...
        staticCorridorBoundary_deg, staticCorridor, ...
        derivativeLimitScale, useSeedAnchoredBarrier);
    trialElapsedTime_s = toc(trialTimer);
    stageTiming.CorridorConstructionElapsedTime_s = ...
        stageTiming.CorridorConstructionElapsedTime_s + ...
        corridorConstructionElapsedTime_s;
    stageTiming.MotionSolvingElapsedTime_s = ...
        stageTiming.MotionSolvingElapsedTime_s + max(0, ...
        trialElapsedTime_s - corridorConstructionElapsedTime_s);
    qpCount = qpCount + trialQpCount;
    linearProgramCount = linearProgramCount + trialLinearProgramCount;
    trialCount = trialIndex;
    trialDuration_s(trialIndex) = duration_s;
    trialExitFlag(trialIndex) = lastExitFlag;
    trialQpCounts(trialIndex) = trialQpCount;
    trialElapsedTimes_s(trialIndex) = trialElapsedTime_s;
    validation = azElPlannerMethods.corridor.validateTrajectory( ...
        trialMotion, obstacles, initialState, goalState, limits, ...
        plannerOptions);
    stageTiming.CollisionCheckingElapsedTime_s = ...
        stageTiming.CollisionCheckingElapsedTime_s + ...
        validation.CollisionCheckingElapsedTime_s;
    stageTiming.FinalValidationElapsedTime_s = ...
        stageTiming.FinalValidationElapsedTime_s + ...
        max(0, validation.ElapsedTime_s - ...
        validation.CollisionCheckingElapsedTime_s);
    trialValidationPassed(trialIndex) = validation.Passed;
    motionTimer = tic;
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
    stageTiming.MotionSolvingElapsedTime_s = ...
        stageTiming.MotionSolvingElapsedTime_s + toc(motionTimer);
    if abs(nextDuration_s - duration_s) <= 1e-6
        break;
    end
    duration_s = nextDuration_s;
end

%% Section 3: Return Improvement Diagnostics

% Accepted means this helper found at least one valid compact candidate. The
% caller still compares its duration against the retained production result.
diagnostics = struct("Attempted", true, "Accepted", ~isempty(bestMotion), ...
    "SpanCount", spanCount, ...
    "DecisionCount", 2 * affineModel.VariableControlCount, ...
    "Representation", representation, ...
    "UsedStaticBarrier", useStaticBarrier, ...
    "StaticBarrierRowCount", size(staticBarrierMatrix, 1), ...
    "TrialCount", trialCount, "QpCount", qpCount, ...
    "LinearProgramCount", linearProgramCount, ...
    "ExitFlag", bestExitFlag, "LastExitFlag", lastExitFlag, ...
    "InitialDuration_s", initialDuration_s, ...
    "BestDuration_s", upperDuration_s, ...
    "TrialDuration_s", trialDuration_s(1:trialCount), ...
    "TrialValidationPassed", trialValidationPassed(1:trialCount), ...
    "TrialExitFlag", trialExitFlag(1:trialCount), ...
    "TrialQpCount", trialQpCounts(1:trialCount), ...
    "TrialElapsedTime_s", trialElapsedTimes_s(1:trialCount), ...
    "LastValidation", validation, ...
    "AffineBasisBuildCount", double(basisBuilt), ...
    "AffineBasisReuseCount", trialCount, ...
    "StageTiming", azElPlannerMethods.internal.stageTiming());
totalElapsedTime_s = toc(compactTimer);
diagnostics.StageTiming = ...
    azElPlannerMethods.internal.stageTiming( ...
    stageTiming, totalElapsedTime_s);
end

%% Section 4: Local Functions

function [motion, controlPoint_deg, exitFlag, qpCount, ...
        linearProgramCount, corridorConstructionElapsedTime_s] = ...
        solveDuration( ...
        controlPoint_deg, duration_s, obstacles, initialState, goalState, limits, ...
        plannerOptions, affineModel, referencePosition_deg, ...
        staticBarrierMatrix, staticBarrierBound, useStaticBarrier, ...
        staticCorridorBoundary_deg, staticCorridor, ...
        derivativeLimitScale, useSeedAnchoredBarrier)
% Alternate local obstacle half-planes with one minimum-jerk QP at a fixed duration, then assemble the exact polynomial for validation.
spanDuration_s = duration_s / affineModel.ReferenceDuration_s * ...
    affineModel.UnitControlPolynomial.SegmentDuration_s;

% Map the cached unit-duration model onto this physical duration exactly.
% Position basis is unchanged; each time derivative follows the chain rule.
referenceInitialTime_s = ...
    affineModel.UnitControlPolynomial.SegmentStartTime_s(1);
sampleTime_s = initialState.time_s + duration_s / ...
    affineModel.ReferenceDuration_s * ...
    (affineModel.SampleTime_s - referenceInitialTime_s);
durationScale = affineModel.ReferenceDuration_s / duration_s;
positionBasis = affineModel.PositionBasis;
velocityBasis = affineModel.VelocityBasis * durationScale;
accelerationBasis = affineModel.AccelerationBasis * durationScale^2;
jerkBasis = affineModel.JerkBasis * durationScale^3;
interiorIndex = affineModel.VariableControlIndex;
fixedIndex = affineModel.FixedControlIndex;
leftIndex = fixedIndex(1:3);
rightIndex = fixedIndex(end - 2:end);
controlPoint_deg(leftIndex, :) = [ ...
    positionBasis(1, leftIndex); ...
    velocityBasis(1, leftIndex); ...
    accelerationBasis(1, leftIndex)] \ [ ...
    initialState.position_deg; initialState.velocity_deg_s; ...
    initialState.acceleration_deg_s2];
controlPoint_deg(rightIndex, :) = [ ...
    positionBasis(end, rightIndex); ...
    velocityBasis(end, rightIndex); ...
    accelerationBasis(end, rightIndex)] \ [ ...
    goalState.position_deg; goalState.velocity_deg_s; ...
    goalState.acceleration_deg_s2];
decision = [ ...
    controlPoint_deg(interiorIndex, 1); ...
    controlPoint_deg(interiorIndex, 2)];
quadraticOptions = optimoptions("quadprog", "Display", "off", "Algorithm", "active-set");
linearOptions = optimoptions("linprog", "Display", "off");
if useStaticBarrier || useSeedAnchoredBarrier
    workspaceSpan_deg = [diff(limits.azimuthInterval_deg), ...
        diff(limits.elevationInterval_deg)];
    maximumControlStep_deg = max(workspaceSpan_deg);
    maximumIterationCount = 1;
else
    maximumControlStep_deg = 6;
    maximumIterationCount = 6;
end
qpCount = 0;
linearProgramCount = 0;
corridorConstructionElapsedTime_s = 0;

[kinematicMatrix, kinematicBound] = kinematicRows( ...
    {positionBasis, velocityBasis, accelerationBasis, jerkBasis}, ...
    controlPoint_deg, interiorIndex, fixedIndex, limits, ...
    derivativeLimitScale);
fixedJerk = jerkBasis(:, fixedIndex) * controlPoint_deg(fixedIndex, :);
interiorJerk = jerkBasis(:, interiorIndex);
jerkMap = blkdiag(interiorJerk, interiorJerk);
baseJerk = [fixedJerk(:, 1); fixedJerk(:, 2)];
rawHessian = jerkMap.' * jerkMap;
rawGradient = jerkMap.' * baseJerk;
objectiveScale = max([eps; abs(diag(rawHessian))]);
hessian = 2 * (rawHessian / objectiveScale + ...
    1e-9 * eye(numel(decision)));
gradient = 2 * rawGradient / objectiveScale;
fixedPosition_deg = positionBasis(:, fixedIndex) * ...
    controlPoint_deg(fixedIndex, :);

% Exact supports need one convex solve; sampled C3 barriers are relinearized.
for iterationIndex = 1:maximumIterationCount
    if useStaticBarrier
        barrierMatrix = staticBarrierMatrix;
        barrierBound = staticBarrierBound;
    else
        corridorTimer = tic;
        [barrierMatrix, barrierBound] = obstacleRows( ...
            positionBasis(:, interiorIndex), fixedPosition_deg, ...
            decision, sampleTime_s, obstacles, ...
            referencePosition_deg, useSeedAnchoredBarrier);
        corridorConstructionElapsedTime_s = ...
            corridorConstructionElapsedTime_s + toc(corridorTimer);
    end
    inequalityMatrix = [kinematicMatrix; barrierMatrix];
    inequalityBound = [kinematicBound; barrierBound];
    lowerBound = decision - maximumControlStep_deg;
    upperBound = decision + maximumControlStep_deg;
    linearExitFlag = 1;
    if (useStaticBarrier || useSeedAnchoredBarrier) && ...
            any(inequalityMatrix * decision > inequalityBound)
        [feasibleDecision, ~, linearExitFlag] = linprog( ...
            zeros(size(decision)), inequalityMatrix, inequalityBound, ...
            [], [], lowerBound, upperBound, linearOptions);
        linearProgramCount = linearProgramCount + 1;
        if linearExitFlag > 0
            decision = feasibleDecision;
        end
    end
    if linearExitFlag <= 0
        exitFlag = linearExitFlag;
        break;
    end
    [trialDecision, ~, exitFlag] = quadprog( ...
        hessian, gradient, inequalityMatrix, inequalityBound, [], [], ...
        lowerBound, upperBound, decision, quadraticOptions);
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
polynomial = azElPlannerMethods.corridor.internal.motion.convertBsplineToPolynomial( ...
    controlPoint_deg, affineModel.Degree, ...
    initialState.time_s, spanDuration_s);
outputTime_s = (initialState.time_s:plannerOptions.SampleTime_s: polynomial.FinalTime_s).';
outputTime_s = unique([outputTime_s; polynomial.FinalTime_s]);
[outputTime_s, position_deg, velocity_deg_s, acceleration_deg_s2, ...
    jerk_deg_s3] = azElInternal.evaluatePolynomial( ...
        polynomial, outputTime_s);
motion = struct("time_s", outputTime_s, "position_deg", position_deg, ...
    "velocity_deg_s", velocity_deg_s, ...
    "acceleration_deg_s2", acceleration_deg_s2, ...
    "jerk_deg_s3", jerk_deg_s3, "Polynomial", polynomial, ...
    "FinalTime_s", polynomial.FinalTime_s, ...
    "MotionDuration_s", polynomial.FinalTime_s - initialState.time_s, ...
    "IntegratedSquaredJerk_deg2_s5", integratedSquaredJerk(polynomial), ...
    "ControlPoint_deg", controlPoint_deg, ...
    "SpanDuration_s", spanDuration_s, ...
    "SeedCorridorBoundary_deg", staticCorridorBoundary_deg, ...
    "SeedCorridor", staticCorridor);
end

function cost_deg2_s5 = integratedSquaredJerk(polynomial)
% Integrate squared two-axis jerk exactly over every polynomial span.
cost_deg2_s5 = 0;
for segmentIndex = 1:polynomial.SegmentCount
    for axisIndex = 1:2
        jerkPower = reshape( ...
            polynomial.jerkPower_deg_s3(segmentIndex, axisIndex, :), ...
            1, []);
        squaredPower = conv(jerkPower, jerkPower);
        cost_deg2_s5 = cost_deg2_s5 + ...
            polynomial.SegmentDuration_s(segmentIndex) * ...
            sum(squaredPower ./ (1:numel(squaredPower)));
    end
end
end

function [matrix, bound, used, boundary_deg, corridor] = ...
        staticCorridorRows( ...
        seed, affineModel, controlPoint_deg, obstacles)
% Assemble continuous Bernstein half-space constraints once for static geometry.
interiorIndex = affineModel.VariableControlIndex;
decisionCount = 2 * numel(interiorIndex);
matrix = zeros(0, decisionCount);
bound = zeros(0, 1);
used = false;
boundary_deg = zeros(0, 2);
corridorTemplate = struct( ...
    "SegmentIndex", 0, "RegionIndex", 0, "Normal", [0 0], ...
    "BoundaryOffset_deg", 0, "Clearance_deg", 0);
corridor = repmat(corridorTemplate, 0, 1);
if isempty(obstacles) || ~all(arrayfun(@(obstacle) all( ...
        obstacle.InternalPreparation.IntervalSpeedBound_deg_s == 0), obstacles))
    return;
end
corridorSeed = seed;
corridorSeed.tau = linspace(0, 1, size(seed.position_deg, 1)).';
corridorSeed.CorridorBoundary_deg = ...
    azElPlannerMethods.corridor.internal.obstacles.buildEnvelopeBoundary( ...
    obstacles, 1e-6);
boundary_deg = corridorSeed.CorridorBoundary_deg;
corridor = azElPlannerMethods.corridor.internal.validation.buildSeedCorridor( ...
    corridorSeed, affineModel.SpanCount);
if isempty(corridor)
    return;
end
for corridorIndex = 1:numel(corridor)
    corridor(corridorIndex).Clearance_deg = 0.02;
end
fixedControlPoint_deg = zeros(size(controlPoint_deg));
fixedControlPoint_deg(affineModel.FixedControlIndex, :) = ...
    controlPoint_deg(affineModel.FixedControlIndex, :);
fixedPolynomial = ...
    azElPlannerMethods.corridor.internal.motion.convertBsplineToPolynomial( ...
    fixedControlPoint_deg, affineModel.Degree, 0, ...
    affineModel.UnitControlPolynomial.SegmentDuration_s);
baseInequality_deg = azElInternal.seedCorridorInequality( ...
    fixedPolynomial, corridor);
matrix = zeros(numel(baseInequality_deg), decisionCount);
segmentIndex = [corridor.SegmentIndex].';
normal = vertcat(corridor.Normal);
coefficientCount = size( ...
    affineModel.UnitControlPolynomial.positionPower_deg, 3);

% Map each free spline control directly to every continuous corridor bound.
for axisIndex = 1:2
    for interiorOffset = 1:numel(interiorIndex)
        controlIndex = interiorIndex(interiorOffset);
        selectedPower_deg = ...
            affineModel.UnitControlPolynomial.positionPower_deg( ...
            segmentIndex, controlIndex, :);
        basisPower_deg = reshape( ...
            selectedPower_deg, numel(corridor), coefficientCount);
        projectionPower_deg = normal(:, axisIndex) .* basisPower_deg;
        projectionBernstein_deg = azElInternal.powerToBernstein( ...
            projectionPower_deg.');
        decisionIndex = (axisIndex - 1) * numel(interiorIndex) + ...
            interiorOffset;
        matrix(:, decisionIndex) = -projectionBernstein_deg(:);
    end
end
bound = -baseInequality_deg;
used = true;
end

function [matrix, bound] = obstacleRows( ...
        interiorBasis, fixedPosition_deg, decision, time_s, obstacles, ...
        seedPosition_deg, useSeedReference)
% Linearize outward obstacle-clearance constraints near the current path.
interiorCount = size(interiorBasis, 2);
position_deg = fixedPosition_deg + [ ...
    interiorBasis * decision(1:interiorCount), interiorBasis * decision(interiorCount + 1:end)];
if useSeedReference
    referencePosition_deg = seedPosition_deg;
else
    referencePosition_deg = position_deg;
end
matrix = zeros(numel(time_s) * max(1, numel(obstacles)), 2 * interiorCount);
bound = zeros(size(matrix, 1), 1);
includedRow = false(size(bound));
obstacleCount = numel(obstacles);

% Add constraints only for obstacle histories near the sampled candidate path.
for obstacleIndex = 1:obstacleCount
    preparation = obstacles(obstacleIndex).InternalPreparation;
    historyBounds_deg = preparation.HistoryBounds_deg;
    axisDistance_deg = max(cat(3, historyBounds_deg([1 3]) - position_deg, ...
        zeros(size(position_deg)), position_deg - historyBounds_deg([2 4])), [], 3);
    referenceAxisDistance_deg = max(cat(3, ...
        historyBounds_deg([1 3]) - referencePosition_deg, ...
        zeros(size(referencePosition_deg)), ...
        referencePosition_deg - historyBounds_deg([2 4])), [], 3);
    nearTimeIndex = find(min(vecnorm(axisDistance_deg, 2, 2), ...
        vecnorm(referenceAxisDistance_deg, 2, 2)) < 10);
    if isempty(nearTimeIndex)
        continue;
    end
    stationaryHistory = all(preparation.IntervalSpeedBound_deg_s == 0);
    if stationaryHistory
        shape = preparation.SampleShapes{1};
        [clearance_deg, nearestPoint_deg] = azElInternal.geometry.pointPolygonClearance( ...
            shape, referencePosition_deg(nearTimeIndex, :));
    else
        clearance_deg = zeros(numel(nearTimeIndex), 1);
        nearestPoint_deg = zeros(numel(nearTimeIndex), 2);

        % Query moving geometry separately at every nearby trajectory time.
        for nearIndex = 1:numel(nearTimeIndex)
            timeIndex = nearTimeIndex(nearIndex);
            shape = azElInternal.obstacles.shapeAtTime( ...
                obstacles(obstacleIndex), time_s(timeIndex));
            [clearance_deg(nearIndex), nearestPoint_deg(nearIndex, :)] = azElInternal.geometry.pointPolygonClearance( ...
                shape, referencePosition_deg(timeIndex, :));
        end
    end
    clearance_deg = clearance_deg(:);
    direction_deg = referencePosition_deg(nearTimeIndex, :) - nearestPoint_deg;
    direction_deg(clearance_deg < 0, :) = -direction_deg(clearance_deg < 0, :);
    directionNorm_deg = vecnorm(direction_deg, 2, 2);
    selectedQuery = clearance_deg < 10 & directionNorm_deg > eps;
    selectedQuery = selectedQuery(:);
    timeIndex = nearTimeIndex(selectedQuery);
    selectedNorm_deg = reshape(directionNorm_deg(selectedQuery), [], 1);
    outward = direction_deg(selectedQuery, :) ./ selectedNorm_deg;
    rowIndex = (timeIndex - 1) * obstacleCount + obstacleIndex;
    matrix(rowIndex, 1:interiorCount) = -outward(:, 1) .* interiorBasis(timeIndex, :);
    matrix(rowIndex, interiorCount + 1:end) = -outward(:, 2) .* interiorBasis(timeIndex, :);
    bound(rowIndex) = sum(outward .* (fixedPosition_deg(timeIndex, :) - nearestPoint_deg(selectedQuery, :)), 2) - 0.1;
    includedRow(rowIndex) = true;
end
matrix = matrix(includedRow, :);
bound = bound(includedRow);
end

function deforms = obstacleDeforms(obstacle)
% Distinguish rigid translation from input histories whose shape changes.
deforms = false;
reference_deg = [obstacle.az_deg{1}(:), obstacle.el_deg{1}(:)];
for sliceIndex = 2:numel(obstacle.az_deg)
    slice_deg = [obstacle.az_deg{sliceIndex}(:), ...
        obstacle.el_deg{sliceIndex}(:)];
    if ~isequal(size(slice_deg), size(reference_deg))
        deforms = true;
        return;
    end
    finiteRow = all(isfinite(reference_deg), 2) & ...
        all(isfinite(slice_deg), 2);
    displacement_deg = slice_deg(finiteRow, :) - ...
        reference_deg(finiteRow, :);
    if ~isempty(displacement_deg) && max(vecnorm( ...
            displacement_deg - displacement_deg(1, :), 2, 2)) > 1e-10
        deforms = true;
        return;
    end
end
end

function [matrix, bound] = kinematicRows( ...
        basisByOrder, controlPoint_deg, interiorIndex, fixedIndex, ...
        limits, derivativeLimitScale)
% Express sampled workspace and derivative limits as affine QP rows.
limitByOrder = {[Inf Inf], limits.maxVelocity_deg_s, limits.maxAcceleration_deg_s2, limits.maxJerk_deg_s3};
workspaceByAxis = {limits.azimuthInterval_deg, limits.elevationInterval_deg};
decisionCount = 2 * numel(interiorIndex);
matrix = zeros(0, decisionCount);
bound = zeros(0, 1);

% Create affine rows for position, velocity, acceleration, and jerk.
for orderIndex = 1:4
    basis = basisByOrder{orderIndex};

    % Apply the appropriate lower and upper limits to both physical axes.
    for axisIndex = 1:2
        if orderIndex == 1
            lowerLimit = workspaceByAxis{axisIndex}(1);
            upperLimit = workspaceByAxis{axisIndex}(2);
        else
            lowerLimit = -derivativeLimitScale * ...
                limitByOrder{orderIndex}(axisIndex);
            upperLimit = derivativeLimitScale * ...
                limitByOrder{orderIndex}(axisIndex);
        end
        fixedValue = basis(:, fixedIndex) * controlPoint_deg(fixedIndex, axisIndex);
        block = zeros(size(basis, 1), decisionCount);
        selectedDecision = (axisIndex - 1) * numel(interiorIndex) + (1:numel(interiorIndex));
        block(:, selectedDecision) = basis(:, interiorIndex);
        matrix = [matrix; block; -block]; %#ok<AGROW>
        bound = [bound; upperLimit - fixedValue; -lowerLimit + fixedValue]; %#ok<AGROW>
    end
end
end
