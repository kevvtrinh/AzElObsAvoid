function [candidate, stageTiming] = retimeDynamicRoute( ...
        candidate, obstacles, initialState, goalState, limits, ...
        plannerOptions, route_deg, stageTiming)
%% Section 0: Header & Readme
% SYNTAX
%   [candidate, stageTiming] = ...
%       azElPlannerMethods.corridor.internal.motion.retimeDynamicRoute( ...
%       candidate, obstacles, initialState, goalState, limits, ...
%       plannerOptions, route_deg, stageTiming)
%**************************************************************************
% PURPOSE
%   - Retime and, when needed, repair one moving-obstacle route.
%**************************************************************************
% INPUTS
%   - candidate (scalar struct), original failed corridor candidate.
%   - obstacles, initialState, goalState, limits, plannerOptions
%       Resolved inputs for motion construction and independent validation.
%   - route_deg (N-by-2 array), fixed route geometry.
%   - stageTiming (scalar struct), shared value-based stage totals.
%**************************************************************************
% OUTPUTS
%   - candidate (scalar struct), original failure or validated retimed motion.
%   - stageTiming (scalar struct), updated exclusive planner-stage totals.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivative units follow limits.
%**************************************************************************

edgeLength_deg = vecnorm(diff(route_deg), 2, 2);
if any(edgeLength_deg <= 0)
    return;
end
motionTimer = tic;
spanWeights = edgeLength_deg .^ 1.1;
bestMotion = [];
bestDuration_s = Inf;

% Perform a fixed number of path-timing feedback updates.
for iterationIndex = 0:8
    trialMotion = ...
        azElPlannerMethods.corridor.internal.motion.buildQuinticSpline( ...
        route_deg, initialState, goalState, limits, struct( ...
        "SpanWeights", spanWeights, ...
        "SampleTime_s", plannerOptions.SampleTime_s, ...
        "GoalTimeMode", plannerOptions.GoalTimeMode, ...
        "AllowAzimuthWrapping", plannerOptions.AllowAzimuthWrapping));
    if trialMotion.Success && ...
            trialMotion.MotionDuration_s < bestDuration_s
        bestMotion = trialMotion;
        bestDuration_s = trialMotion.MotionDuration_s;
    end
    demandError = log(max(0.1, ...
        azElPlannerMethods.corridor.internal.motion.spanTimeDemand( ...
        trialMotion.Polynomial, limits)));
    demandError = demandError - mean(demandError);
    spanWeights = exp(log(spanWeights) + demandError);
    spanWeights = spanWeights / mean(spanWeights);
end
stageTiming.MotionSolvingElapsedTime_s = ...
    stageTiming.MotionSolvingElapsedTime_s + toc(motionTimer);
if isempty(bestMotion)
    return;
end
validation = azElPlannerMethods.corridor.validateTrajectory( ...
    bestMotion, obstacles, initialState, goalState, limits, plannerOptions);
stageTiming.CollisionCheckingElapsedTime_s = ...
    stageTiming.CollisionCheckingElapsedTime_s + ...
    validation.CollisionCheckingElapsedTime_s;
stageTiming.FinalValidationElapsedTime_s = ...
    stageTiming.FinalValidationElapsedTime_s + ...
    max(0, validation.ElapsedTime_s - ...
    validation.CollisionCheckingElapsedTime_s);
recoveryResidualLimit_deg = 0.005;
if size(route_deg, 1) >= 12 && ...
        (validation.Passed || ...
        validation.MinimumClearance_deg >= -recoveryResidualLimit_deg)
    [bestMotion, validation, stageTiming] = improveDynamicGeometry( ...
        bestMotion, validation, obstacles, initialState, goalState, ...
        limits, plannerOptions, spanWeights, stageTiming);
end
if ~validation.Passed
    return;
end
motionFields = fieldnames(bestMotion);

for fieldIndex = 1:numel(motionFields)
    candidate.(motionFields{fieldIndex}) = ...
        bestMotion.(motionFields{fieldIndex});
end
candidate.Validation = validation;
candidate.Success = true;
candidate.Message = ...
    "The path-fixed-point dynamic retime passed validation.";
candidate.TerminationReason = "quinticValidated";
candidate.OptimizerDiagnostics.ValidationPassed = true;
candidate.OptimizerDiagnostics.ContinuousClearance_deg = ...
    validation.MinimumClearance_deg;
candidate.OptimizerDiagnostics.MotionDuration_s = ...
    bestMotion.MotionDuration_s;
end

function [bestMotion, bestValidation, stageTiming] = improveDynamicGeometry( ...
        bestMotion, bestValidation, obstacles, initialState, goalState, ...
        limits, plannerOptions, spanWeights, stageTiming)
%% Section 0: Header & Readme
% SYNTAX
%   [bestMotion, bestValidation, stageTiming] = improveDynamicGeometry( ...
%       bestMotion, bestValidation, obstacles, initialState, goalState, ...
%       limits, plannerOptions, spanWeights, stageTiming)
%**************************************************************************
% PURPOSE
%   - Apply bounded clearance or minimum-jerk feedback at local barriers.
%**************************************************************************
% INPUTS
%   - bestMotion, bestValidation (scalar structs), retained candidate evidence.
%   - obstacles, initialState, goalState, limits, plannerOptions
%       Resolved motion-construction and validation inputs.
%   - spanWeights (numeric vector), positive fixed route timing weights.
%   - stageTiming (scalar struct), shared value-based stage totals.
%**************************************************************************
% OUTPUTS
%   - bestMotion, bestValidation (scalar structs), last accepted evidence.
%   - stageTiming (scalar struct), updated exclusive planner-stage totals.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; spanWeights are relative.
%**************************************************************************
route_deg = [initialState.position_deg; ...
    bestMotion.ControlPoint_deg(4:end - 3, :); goalState.position_deg];
interiorCount = size(route_deg, 1) - 2;
decisionCount = 2 * interiorCount;
recoveringCollision = ~bestValidation.Passed;
if recoveringCollision
    spanWeights = bestMotion.SpanDuration_s;
end
maximumIterationCount = 24;

for iterationIndex = 1:maximumIterationCount
    motionTimer = tic;
    affineModel = ...
        azElPlannerMethods.corridor.internal.motion.buildFixedDurationAffineModel( ...
        size(bestMotion.ControlPoint_deg, 1), initialState.time_s, ...
        bestMotion.SpanDuration_s, zeros(0, 1));
    stageTiming.MotionSolvingElapsedTime_s = ...
        stageTiming.MotionSolvingElapsedTime_s + toc(motionTimer);
    accepted = false;
    trustRadius_deg = 0.5;

    for backtrackIndex = 1:3
        corridorTimer = tic;
        [barrierMatrix, barrierBound] = dynamicBarrierRows( ...
            bestMotion, affineModel, obstacles, 0.05, 0.5);
        stageTiming.CorridorConstructionElapsedTime_s = ...
            stageTiming.CorridorConstructionElapsedTime_s + ...
            toc(corridorTimer);
        if recoveringCollision
            sensitivity = vecnorm(barrierMatrix, 2, 2);
            violated = barrierBound < 0 & sensitivity > 1e-12;
            if ~any(violated)
                break;
            end
            gain = min(1, trustRadius_deg / max( ...
                -barrierBound(violated) ./ sensitivity(violated)));
            barrierBound(violated) = gain * barrierBound(violated);
            motionTimer = tic;
            [decision_deg, ~, exitFlag] = quadprog( ...
                eye(decisionCount), zeros(decisionCount, 1), ...
                barrierMatrix, barrierBound, [], [], ...
                -trustRadius_deg * ones(decisionCount, 1), ...
                trustRadius_deg * ones(decisionCount, 1), [], ...
                optimoptions("quadprog", "Display", "off"));
            optimizerAccepted = exitFlag > 0;
            if ~optimizerAccepted
                decision_deg = zeros(decisionCount, 1);
            end
            stageTiming.MotionSolvingElapsedTime_s = ...
                stageTiming.MotionSolvingElapsedTime_s + toc(motionTimer);
        else
            motionTimer = tic;
            [decision_deg, diagnostics] = ...
                azElPlannerMethods.corridor.internal.motion.optimizeExactTraversal( ...
                bestMotion, affineModel, zeros(decisionCount, 1), ...
                barrierMatrix, barrierBound, trustRadius_deg, limits);
            optimizerAccepted = diagnostics.Accepted;
            stageTiming.MotionSolvingElapsedTime_s = ...
                stageTiming.MotionSolvingElapsedTime_s + toc(motionTimer);
        end
        motionTimer = tic;
        trialMotion = ...
            azElPlannerMethods.corridor.internal.motion.buildQuinticSpline( ...
            route_deg, initialState, goalState, limits, struct( ...
            "SpanWeights", spanWeights, ...
            "ControlPointOffsets_deg", ...
            [decision_deg(1:interiorCount), ...
            decision_deg(interiorCount + 1:end)], ...
            "SampleTime_s", plannerOptions.SampleTime_s, ...
            "GoalTimeMode", plannerOptions.GoalTimeMode, ...
            "AllowAzimuthWrapping", ...
            plannerOptions.AllowAzimuthWrapping));
        stageTiming.MotionSolvingElapsedTime_s = ...
            stageTiming.MotionSolvingElapsedTime_s + toc(motionTimer);
        trialValidation = azElPlannerMethods.corridor.validateTrajectory( ...
            trialMotion, obstacles, initialState, goalState, ...
            limits, plannerOptions);
        stageTiming.CollisionCheckingElapsedTime_s = ...
            stageTiming.CollisionCheckingElapsedTime_s + ...
            trialValidation.CollisionCheckingElapsedTime_s;
        stageTiming.FinalValidationElapsedTime_s = ...
            stageTiming.FinalValidationElapsedTime_s + ...
            max(0, trialValidation.ElapsedTime_s - ...
            trialValidation.CollisionCheckingElapsedTime_s);
        clearanceImproved = trialMotion.Success && ...
            trialValidation.MinimumClearance_deg > ...
            bestValidation.MinimumClearance_deg + 1e-6;
        durationImproved = trialValidation.Passed && ...
            trialMotion.MotionDuration_s < ...
            bestMotion.MotionDuration_s - 1e-9;
        accepted = optimizerAccepted && ...
            ((recoveringCollision && clearanceImproved) || ...
            (~recoveringCollision && durationImproved));
        if accepted
            break;
        end
        trustRadius_deg = trustRadius_deg / 2;
    end
    if ~accepted
        break;
    end
    bestMotion = trialMotion;
    bestValidation = trialValidation;
    route_deg = [initialState.position_deg; ...
        bestMotion.ControlPoint_deg(4:end - 3, :); goalState.position_deg];
    if recoveringCollision && bestValidation.Passed
        break;
    end
end
end

function [matrix, bound] = dynamicBarrierRows( ...
        motion, affineModel, obstacles, clearanceTarget_deg, ...
        activationRadius_deg)
%% Section 0: Header & Readme
% SYNTAX
%   [matrix, bound] = dynamicBarrierRows( ...
%       motion, affineModel, obstacles, clearanceTarget_deg, ...
%       activationRadius_deg)
%**************************************************************************
% PURPOSE
%   - Linearize protected exterior half-planes at local closest points.
%**************************************************************************
% INPUTS
%   - motion, affineModel (scalar structs), retained motion and control map.
%   - obstacles (struct array), prepared time-varying protected geometry.
%   - clearanceTarget_deg, activationRadius_deg (nonnegative scalars)
%       Clearance target and neighborhood used to activate barriers.
%**************************************************************************
% OUTPUTS
%   - matrix, bound (numeric arrays), affine decision inequalities.
%**************************************************************************
% UNITS
%   - Matrix/bound operate on degree-valued control-point offsets.
%**************************************************************************
interiorCount = affineModel.VariableControlCount;
affineBasis = affineModel.UnitControlPolynomial;
time_s = unique([ ...
    linspace(motion.time_s(1), motion.time_s(end), 161).'; ...
    motion.Polynomial.SegmentStartTime_s; motion.time_s(end)]);
minimumTimes_s = zeros(motion.Polynomial.SegmentCount, 1);

for segmentIndex = 1:motion.Polynomial.SegmentCount
    intervalStart_s = ...
        motion.Polynomial.SegmentStartTime_s(segmentIndex);
    intervalEnd_s = intervalStart_s + ...
        motion.Polynomial.SegmentDuration_s(segmentIndex);
    minimumTimes_s(segmentIndex) = fminbnd( ...
        @(queryTime_s) clearanceAtTime( ...
        motion.Polynomial, obstacles, queryTime_s), ...
        intervalStart_s, intervalEnd_s, ...
        optimset("Display", "off", "TolX", 1e-5));
end
time_s = unique([time_s; minimumTimes_s]);
[~, position_deg] = ...
    azElInternal.evaluatePolynomial( ...
    motion.Polynomial, time_s);
matrix = zeros(numel(time_s), 2 * interiorCount);
bound = zeros(numel(time_s), 1);
barrierCount = 0;
activationDistance_deg = ...
    sqrt(2) * activationRadius_deg + clearanceTarget_deg;

for timeIndex = 1:numel(time_s)
    nearestClearance_deg = Inf;
    nearestPoint_deg = [NaN NaN];

    for obstacleIndex = 1:numel(obstacles)
        shape = ...
            azElInternal.obstacles.shapeAtTime( ...
            obstacles(obstacleIndex), time_s(timeIndex));
        [clearance_deg, obstaclePoint_deg] = ...
            azElInternal.geometry.pointPolygonClearance( ...
            shape, position_deg(timeIndex, :));
        if clearance_deg < nearestClearance_deg
            nearestClearance_deg = clearance_deg;
            nearestPoint_deg = obstaclePoint_deg;
        end
    end
    if nearestClearance_deg >= activationDistance_deg
        continue;
    end
    outward = position_deg(timeIndex, :) - nearestPoint_deg;
    if nearestClearance_deg < 0
        outward = -outward;
    end
    outward = outward / norm(outward);
    segmentIndex = min(motion.Polynomial.SegmentCount, ...
        sum(time_s(timeIndex) >= ...
        motion.Polynomial.SegmentStartTime_s(2:end)) + 1);
    tau = (time_s(timeIndex) - ...
        motion.Polynomial.SegmentStartTime_s(segmentIndex)) / ...
        motion.Polynomial.SegmentDuration_s(segmentIndex);
    basisValue = zeros(1, interiorCount);

    for interiorIndex = 1:interiorCount
        controlPointIndex = ...
            affineModel.VariableControlIndex(interiorIndex);
        coefficient = reshape( ...
            affineBasis.positionPower_deg( ...
            segmentIndex, controlPointIndex, :), 1, []);
        basisValue(interiorIndex) = ...
            (tau .^ (0:numel(coefficient) - 1)) * coefficient.';
    end
    barrierCount = barrierCount + 1;
    matrix(barrierCount, :) = ...
        -[outward(1) * basisValue, outward(2) * basisValue];
    bound(barrierCount) = outward * ...
        (position_deg(timeIndex, :) - nearestPoint_deg).' - ...
        clearanceTarget_deg;
end
matrix = matrix(1:barrierCount, :);
bound = bound(1:barrierCount);
end

function clearance_deg = clearanceAtTime(polynomial, obstacles, time_s)
%% Section 0: Header & Readme
% SYNTAX
%   clearance_deg = clearanceAtTime(polynomial, obstacles, time_s)
%**************************************************************************
% PURPOSE
%   - Evaluate nearest protected-obstacle clearance at one physical time.
%**************************************************************************
% INPUTS
%   - polynomial (scalar struct), complete time-parameterized motion.
%   - obstacles (struct array), prepared protected geometry.
%   - time_s (finite scalar), query time.
%**************************************************************************
% OUTPUTS
%   - clearance_deg (scalar), signed nearest-obstacle clearance.
%**************************************************************************
% UNITS
%   - Time is seconds and clearance is degrees.
%**************************************************************************
[~, position_deg] = ...
    azElInternal.evaluatePolynomial( ...
    polynomial, time_s);
clearance_deg = Inf;

for obstacleIndex = 1:numel(obstacles)
    shape = azElInternal.obstacles.shapeAtTime( ...
        obstacles(obstacleIndex), time_s);
    clearance_deg = min(clearance_deg, ...
        azElInternal.geometry.pointPolygonClearance( ...
        shape, position_deg));
end
end
