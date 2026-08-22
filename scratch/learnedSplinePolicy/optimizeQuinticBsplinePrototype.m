function motion = optimizeQuinticBsplinePrototype( ...
        obstacles, initialState, goalState, limits, route_deg, ...
        optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = optimizeQuinticBsplinePrototype()
%   motion = optimizeQuinticBsplinePrototype( ...
%       obstacles, initialState, goalState, limits, route_deg)
%   motion = optimizeQuinticBsplinePrototype( ...
%       obstacles, initialState, goalState, limits, route_deg, ...
%       optionOverrides)
%**************************************************************************
% PURPOSE
%   - Search a bounded low-dimensional quintic B-spline parameterization and
%     accept only motions that pass maintained continuous validation.
%**************************************************************************
% INPUTS
%   - obstacles (canonical static or time-varying obstacle array, or [])
%       Safety margins must already be applied to protected geometry.
%   - initialState (scalar struct)
%       Initial time, position, velocity, and acceleration request.
%   - goalState (scalar struct)
%       Latest arrival, position, velocity, and acceleration request.
%   - limits (scalar struct)
%       Workspace, velocity, acceleration, and jerk limits.
%   - route_deg (N-by-2 numeric array, N >= 2)
%       Input-driven geometric seed ordered [azimuth elevation].
%   - optionOverrides (scalar struct, optional; default struct())
%       Search-budget, route-reduction, clearance, and objective controls.
%       Call this function with no inputs for fully populated defaults.
%**************************************************************************
% OUTPUTS
%   - motion (scalar quintic trajectory result)
%       On success, Validation is the maintained continuous validator result.
%       Expected search failure returns Success=false with the best partial
%       motion and OptimizerDiagnostics; invalid input contracts throw.
%**************************************************************************
% UNITS
%   - Position and clearance are degrees; time is seconds; derivatives use
%     degrees and seconds. Histories are N-by-2 [azimuth elevation].
%**************************************************************************

%% Section 1: Validate Inputs And Resolve Search Controls

defaults = struct( ...
    "MaximumRouteVertexCount", 24, ...
    "TimingReserveFraction", 0.75, ...
    "MaximumSweeps", 6, ...
    "MaximumFunctionEvaluations", 240, ...
    "MaximumNormalOffset_deg", 3, ...
    "InitialStepFraction", 0.25, ...
    "StepReduction", 0.5, ...
    "MinimumStep_deg", 0.05, ...
    "OptimizationSampleTime_s", 0.2, ...
    "ValidationSampleTime_s", 0.05, ...
    "ClearanceTarget_deg", 0.02, ...
    "CollisionPenaltyWeight", 500, ...
    "DurationWeight", 1, ...
    "JerkWeight", 0.01, ...
    "OffsetPenaltyWeight", 0.01, ...
    "ImprovementTolerance", 1e-8, ...
    "StopAtFirstValidated", true, ...
    "PrintProgress", false);
if nargin == 0
    motion = defaults;
    return;
end
if nargin < 5
    error("optimizeQuinticBsplinePrototype:MissingInputs", ...
        "obstacles, endpoint states, limits, and route_deg are required.");
end
if nargin < 6 || isempty(optionOverrides)
    optionOverrides = struct();
end
[options, unknownNames] = azElInternal.resolveOptions( ...
    defaults, optionOverrides);
if ~isempty(unknownNames)
    warning("optimizeQuinticBsplinePrototype:UnknownOptions", ...
        "Ignoring unknown fields: %s. No behavior changed.", ...
        strjoin(unknownNames, ", "));
end
options = validateOptions(options);
% The constructor provides the maintained input-contract checks before the
% optimizer changes any representation parameters.
buildQuinticBsplinePrototype( ...
    route_deg, initialState, goalState, limits);
obstacles = combineAzElObstacles(obstacles);
[reducedRoute_deg, timingReductionCount] = reduceRouteForTiming( ...
    route_deg, initialState, goalState, limits, options);
normalDirection = interiorNormalDirections(reducedRoute_deg);
interiorCount = size(reducedRoute_deg, 1) - 2;
decisionCount = interiorCount;
spanWeights = vecnorm(diff(reducedRoute_deg), 2, 2);
spanWeights = spanWeights / mean(spanWeights);
validatorOptions = planAzElMotion();
validatorOptions.GoalTimeMode = "earliestArrival";
validatorOptions.SampleTime_s = options.ValidationSampleTime_s;

%% Section 2: Evaluate The Zero-Offset Baseline

searchTimer = tic;
maximumEvaluationCount = options.MaximumFunctionEvaluations;
evaluationRecords = repmat( ...
    emptyEvaluationRecord(), maximumEvaluationCount, 1);
evaluationCount = 0;
baselineDecision_deg = zeros(decisionCount, 1);
[~, baselineMotion] = evaluateDecision( ...
    baselineDecision_deg, reducedRoute_deg, normalDirection, spanWeights, ...
    obstacles, initialState, goalState, limits, validatorOptions, options, ...
    NaN, NaN);
durationReference_s = max(1e-6, baselineMotion.MotionDuration_s);
jerkReference_deg2_s5 = max( ...
    1e-6, baselineMotion.IntegratedSquaredJerk_deg2_s5);
[baselineEvaluation, baselineMotion] = evaluateDecision( ...
    baselineDecision_deg, reducedRoute_deg, normalDirection, spanWeights, ...
    obstacles, initialState, goalState, limits, validatorOptions, options, ...
    durationReference_s, jerkReference_deg2_s5);
evaluationCount = evaluationCount + 1;
baselineEvaluation.EvaluationIndex = evaluationCount;
evaluationRecords(evaluationCount) = baselineEvaluation;
currentDecision_deg = baselineDecision_deg;
currentEvaluation = baselineEvaluation;
currentMotion = baselineMotion;
bestEvaluation = baselineEvaluation;
bestMotion = baselineMotion;
bestValidatedEvaluation = emptyEvaluationRecord();
bestValidatedMotion = baselineMotion;
hasValidatedMotion = baselineEvaluation.ExactValidationPassed;
if hasValidatedMotion
    bestValidatedEvaluation = baselineEvaluation;
    bestValidatedMotion = baselineMotion;
end

%% Section 3: Run Bounded Deterministic Coordinate Search

terminationReason = "baselineValidated";
completedSweepCount = 0;
step_deg = options.InitialStepFraction * ...
    options.MaximumNormalOffset_deg;
shouldSearch = decisionCount > 0 && ...
    ~(hasValidatedMotion && options.StopAtFirstValidated);
while shouldSearch && completedSweepCount < options.MaximumSweeps && ...
        evaluationCount < maximumEvaluationCount && ...
        step_deg >= options.MinimumStep_deg
    completedSweepCount = completedSweepCount + 1;
    sweepImproved = false;
    for decisionIndex = 1:decisionCount
        if evaluationCount >= maximumEvaluationCount
            terminationReason = "evaluationBudget";
            break;
        end
        candidateDecisions_deg = repmat(currentDecision_deg, 1, 2);
        candidateDecisions_deg(decisionIndex, 1) = min( ...
            options.MaximumNormalOffset_deg, ...
            currentDecision_deg(decisionIndex) + step_deg);
        candidateDecisions_deg(decisionIndex, 2) = max( ...
            -options.MaximumNormalOffset_deg, ...
            currentDecision_deg(decisionIndex) - step_deg);
        bestLocalEvaluation = currentEvaluation;
        bestLocalMotion = currentMotion;
        bestLocalDecision_deg = currentDecision_deg;
        for directionIndex = 1:2
            if evaluationCount >= maximumEvaluationCount
                terminationReason = "evaluationBudget";
                break;
            end
            candidateDecision_deg = ...
                candidateDecisions_deg(:, directionIndex);
            if isequal(candidateDecision_deg, currentDecision_deg)
                continue;
            end
            [candidateEvaluation, candidateMotion] = evaluateDecision( ...
                candidateDecision_deg, reducedRoute_deg, normalDirection, ...
                spanWeights, obstacles, initialState, goalState, limits, ...
                validatorOptions, options, durationReference_s, ...
                jerkReference_deg2_s5);
            evaluationCount = evaluationCount + 1;
            candidateEvaluation.EvaluationIndex = evaluationCount;
            evaluationRecords(evaluationCount) = candidateEvaluation;
            if options.PrintProgress
                printEvaluation(candidateEvaluation, completedSweepCount, ...
                    decisionIndex);
            end
            if candidateEvaluation.Objective < bestEvaluation.Objective
                bestEvaluation = candidateEvaluation;
                bestMotion = candidateMotion;
            end
            if candidateEvaluation.ExactValidationPassed && ...
                    (~hasValidatedMotion || ...
                    candidateEvaluation.Objective < ...
                    bestValidatedEvaluation.Objective)
                hasValidatedMotion = true;
                bestValidatedEvaluation = candidateEvaluation;
                bestValidatedMotion = candidateMotion;
            end
            sufficientImprovement = candidateEvaluation.Objective < ...
                bestLocalEvaluation.Objective - ...
                options.ImprovementTolerance;
            if sufficientImprovement
                bestLocalEvaluation = candidateEvaluation;
                bestLocalMotion = candidateMotion;
                bestLocalDecision_deg = candidateDecision_deg;
            end
            if hasValidatedMotion && options.StopAtFirstValidated
                terminationReason = "firstValidated";
                break;
            end
        end
        if hasValidatedMotion && options.StopAtFirstValidated
            break;
        end
        if bestLocalEvaluation.Objective < currentEvaluation.Objective - ...
                options.ImprovementTolerance
            currentEvaluation = bestLocalEvaluation;
            currentMotion = bestLocalMotion;
            currentDecision_deg = bestLocalDecision_deg;
            sweepImproved = true;
        end
    end
    if hasValidatedMotion && options.StopAtFirstValidated
        break;
    end
    if ~sweepImproved
        step_deg = step_deg * options.StepReduction;
    end
end
if ~hasValidatedMotion
    if evaluationCount >= maximumEvaluationCount
        terminationReason = "evaluationBudget";
    elseif step_deg < options.MinimumStep_deg
        terminationReason = "stepTolerance";
    elseif completedSweepCount >= options.MaximumSweeps
        terminationReason = "maximumSweeps";
    else
        terminationReason = "noValidatedSpline";
    end
end

%% Section 4: Assemble The Accepted Or Best-Partial Motion

if hasValidatedMotion
    motion = bestValidatedMotion;
    selectedEvaluation = bestValidatedEvaluation;
    motion.Success = true;
    motion.Message = "The bounded quintic B-spline search passed " + ...
        "maintained continuous validation.";
    motion.TerminationReason = "prototypeValidated";
else
    motion = bestMotion;
    selectedEvaluation = bestEvaluation;
    motion.Success = false;
    motion.Message = "No bounded low-dimensional spline candidate passed " + ...
        "maintained continuous validation.";
    motion.TerminationReason = terminationReason;
end
motion.MotionSource = "optimizedQuinticBsplinePrototype";
motion.OriginalRoute_deg = route_deg;
motion.ReducedRoute_deg = reducedRoute_deg;
motion.OptimizerOptions = options;
motion.OptimizerDiagnostics = struct( ...
    "DecisionVariableCount", decisionCount, ...
    "EvaluationCount", evaluationCount, ...
    "CompletedSweepCount", completedSweepCount, ...
    "FinalStep_deg", step_deg, ...
    "OriginalRouteVertexCount", size(route_deg, 1), ...
    "ReducedRouteVertexCount", size(reducedRoute_deg, 1), ...
    "TimingReductionCount", timingReductionCount, ...
    "NormalDirection", normalDirection, ...
    "SpanWeights", spanWeights, ...
    "SelectedDecision_deg", selectedEvaluation.Decision_deg, ...
    "SelectedObjective", selectedEvaluation.Objective, ...
    "SelectedMinimumSampledClearance_deg", ...
    selectedEvaluation.MinimumSampledClearance_deg, ...
    "HasValidatedMotion", hasValidatedMotion, ...
    "SearchTerminationReason", terminationReason, ...
    "ElapsedTime_s", toc(searchTimer), ...
    "EvaluationTrace", evaluationRecords(1:evaluationCount));
end

%% Section 5: Local Functions

function options = validateOptions(options)
%% Section 0: Header & Readme
% SYNTAX
%   options = validateOptions(options)
%**************************************************************************
% PURPOSE
%   - Validate and normalize all bounded-search controls once.
%**************************************************************************
% INPUTS
%   - options (resolved scalar option struct)
%**************************************************************************
% OUTPUTS
%   - options (validated and normalized scalar option struct)
%**************************************************************************
% UNITS
%   - Offset, step, and clearance fields are degrees; sampling is seconds.
%**************************************************************************
integerPositiveNames = ["MaximumRouteVertexCount", "MaximumSweeps", ...
    "MaximumFunctionEvaluations"];
for optionName = integerPositiveNames
    validateattributes(options.(optionName), {'numeric'}, ...
        {'real', 'finite', 'scalar', 'integer', 'positive'});
end
positiveNames = ["MaximumNormalOffset_deg", "InitialStepFraction", ...
    "StepReduction", "MinimumStep_deg", ...
    "OptimizationSampleTime_s", "ValidationSampleTime_s", ...
    "CollisionPenaltyWeight", ...
    "DurationWeight", "TimingReserveFraction"];
for optionName = positiveNames
    validateattributes(options.(optionName), {'numeric'}, ...
        {'real', 'finite', 'scalar', 'positive'});
end
nonnegativeNames = ["ClearanceTarget_deg", "JerkWeight", ...
    "OffsetPenaltyWeight", "ImprovementTolerance"];
for optionName = nonnegativeNames
    validateattributes(options.(optionName), {'numeric'}, ...
        {'real', 'finite', 'scalar', 'nonnegative'});
end
if options.MaximumRouteVertexCount < 2
    error("optimizeQuinticBsplinePrototype:InvalidRouteVertexLimit", ...
        "MaximumRouteVertexCount must be at least 2.");
end
if options.InitialStepFraction > 1 || options.StepReduction >= 1
    error("optimizeQuinticBsplinePrototype:InvalidStepControl", ...
        "InitialStepFraction must be at most 1 and StepReduction below 1.");
end
if options.TimingReserveFraction > 1
    error("optimizeQuinticBsplinePrototype:InvalidTimingReserve", ...
        "TimingReserveFraction must be in the interval (0, 1].");
end
options.StopAtFirstValidated = azElInternal.normalizeLogicalScalar( ...
    options.StopAtFirstValidated, "StopAtFirstValidated", ...
    "optimizeQuinticBsplinePrototype:InvalidStopControl");
options.PrintProgress = azElInternal.normalizeLogicalScalar( ...
    options.PrintProgress, "PrintProgress", ...
    "optimizeQuinticBsplinePrototype:InvalidPrintControl");
end

function [reducedRoute_deg, reductionCount] = reduceRouteForTiming( ...
        route_deg, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   [reducedRoute_deg, reductionCount] = reduceRouteForTiming( ...
%       route_deg, initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Reduce representation size until its zero-offset motion leaves an
%     explicit fraction of the supplied goal horizon for collision shaping.
%**************************************************************************
% INPUTS
%   - route_deg (N-by-2 input route)
%   - initialState, goalState, limits (physical request structures)
%   - options (validated optimizer options)
%**************************************************************************
% OUTPUTS
%   - reducedRoute_deg (M-by-2 input-driven route)
%   - reductionCount (nonnegative integer)
%**************************************************************************
% UNITS
%   - Route is degrees; goal horizon and motion duration are seconds.
%**************************************************************************
maximumCount = min(size(route_deg, 1), options.MaximumRouteVertexCount);
goalHorizon_s = goalState.time_s - initialState.time_s;
targetDuration_s = options.TimingReserveFraction * goalHorizon_s;
bestDuration_s = Inf;
bestVertexCount = maximumCount;
bestRoute_deg = reduceRouteByArcLength(route_deg, maximumCount);
for candidateCount = maximumCount:-1:2
    candidateRoute_deg = reduceRouteByArcLength(route_deg, candidateCount);
    spanWeights = vecnorm(diff(candidateRoute_deg), 2, 2);
    spanWeights = spanWeights / mean(spanWeights);
    prototypeOptions = struct( ...
        "SpanWeights", spanWeights, ...
        "SampleTime_s", options.OptimizationSampleTime_s);
    baselineMotion = buildQuinticBsplinePrototype( ...
        candidateRoute_deg, initialState, goalState, limits, ...
        prototypeOptions);
    if baselineMotion.MotionDuration_s < bestDuration_s
        bestDuration_s = baselineMotion.MotionDuration_s;
        bestVertexCount = candidateCount;
        bestRoute_deg = candidateRoute_deg;
    end
    if baselineMotion.MotionDuration_s <= targetDuration_s
        reducedRoute_deg = candidateRoute_deg;
        reductionCount = maximumCount - candidateCount;
        return;
    end
end
reducedRoute_deg = bestRoute_deg;
reductionCount = maximumCount - bestVertexCount;
end

function reducedRoute_deg = reduceRouteByArcLength( ...
        route_deg, maximumVertexCount)
%% Section 0: Header & Readme
% SYNTAX
%   reducedRoute_deg = reduceRouteByArcLength(route_deg, maximumVertexCount)
%**************************************************************************
% PURPOSE
%   - Bound representation size through input-driven arc-length resampling.
%**************************************************************************
% INPUTS
%   - route_deg (N-by-2 route)
%   - maximumVertexCount (integer >= 2)
%**************************************************************************
% OUTPUTS
%   - reducedRoute_deg (M-by-2 route, M <= maximumVertexCount)
%**************************************************************************
% UNITS
%   - Route coordinates and arc length are degrees.
%**************************************************************************
route_deg = double(route_deg);
if size(route_deg, 1) <= maximumVertexCount
    reducedRoute_deg = route_deg;
    return;
end
cumulativeLength_deg = [0; cumsum(vecnorm(diff(route_deg), 2, 2))];
sampleLength_deg = linspace( ...
    0, cumulativeLength_deg(end), maximumVertexCount).';
reducedRoute_deg = interp1( ...
    cumulativeLength_deg, route_deg, sampleLength_deg, "linear");
reducedRoute_deg(1, :) = route_deg(1, :);
reducedRoute_deg(end, :) = route_deg(end, :);
end

function normalDirection = interiorNormalDirections(route_deg)
%% Section 0: Header & Readme
% SYNTAX
%   normalDirection = interiorNormalDirections(route_deg)
%**************************************************************************
% PURPOSE
%   - Define one deterministic unit-normal decision axis per interior point.
%**************************************************************************
% INPUTS
%   - route_deg (N-by-2 route)
%**************************************************************************
% OUTPUTS
%   - normalDirection ((N-2)-by-2 unit rows)
%**************************************************************************
% UNITS
%   - Directions are dimensionless.
%**************************************************************************
interiorCount = size(route_deg, 1) - 2;
normalDirection = zeros(interiorCount, 2);
for interiorIndex = 1:interiorCount
    routeIndex = interiorIndex + 1;
    tangent = route_deg(routeIndex + 1, :) - ...
        route_deg(routeIndex - 1, :);
    tangentNorm = norm(tangent);
    if tangentNorm <= eps
        tangent = route_deg(routeIndex, :) - ...
            route_deg(routeIndex - 1, :);
        tangentNorm = norm(tangent);
    end
    unitTangent = tangent / tangentNorm;
    normalDirection(interiorIndex, :) = ...
        [-unitTangent(2), unitTangent(1)];
end
end

function [evaluation, motion] = evaluateDecision( ...
        decision_deg, reducedRoute_deg, normalDirection, spanWeights, ...
        obstacles, initialState, goalState, limits, validatorOptions, ...
        options, durationReference_s, jerkReference_deg2_s5)
%% Section 0: Header & Readme
% SYNTAX
%   [evaluation, motion] = evaluateDecision( ...
%       decision_deg, reducedRoute_deg, normalDirection, spanWeights, ...
%       obstacles, initialState, goalState, limits, validatorOptions, ...
%       options, durationReference_s, jerkReference_deg2_s5)
%**************************************************************************
% PURPOSE
%   - Build, score, and conditionally certify one bounded spline decision.
%**************************************************************************
% INPUTS
%   - decision_deg (normal-offset vector)
%   - reducedRoute_deg, normalDirection, spanWeights (representation data)
%   - obstacles, initialState, goalState, limits (physical request)
%   - validatorOptions, options (resolved controls)
%   - durationReference_s, jerkReference_deg2_s5 (normalizers or NaN)
%**************************************************************************
% OUTPUTS
%   - evaluation (scalar objective and feasibility record)
%   - motion (scalar quintic prototype motion)
%**************************************************************************
% UNITS
%   - Decision and clearance are degrees; duration is seconds.
%**************************************************************************
offset_deg = normalDirection .* decision_deg;
prototypeOptions = struct( ...
    "ControlPointOffsets_deg", offset_deg, ...
    "SpanWeights", spanWeights, ...
    "SampleTime_s", options.OptimizationSampleTime_s);
motion = buildQuinticBsplinePrototype( ...
    reducedRoute_deg, initialState, goalState, limits, prototypeOptions);
[sampledCollision, minimumClearance_deg, clearancePenalty] = ...
    sampledClearancePenalty(motion, obstacles, options.ClearanceTarget_deg);
if isnan(durationReference_s) || isnan(jerkReference_deg2_s5)
    objective = NaN;
else
    durationTerm = motion.MotionDuration_s / durationReference_s;
    jerkTerm = motion.IntegratedSquaredJerk_deg2_s5 / ...
        jerkReference_deg2_s5;
    offsetTerm = mean((decision_deg / ...
        options.MaximumNormalOffset_deg).^2);
    if isempty(offsetTerm) || isnan(offsetTerm)
        offsetTerm = 0;
    end
    objective = options.DurationWeight * durationTerm + ...
        options.JerkWeight * jerkTerm + ...
        options.OffsetPenaltyWeight * offsetTerm + ...
        options.CollisionPenaltyWeight * clearancePenalty;
end
exactValidation = validateAzElTrajectory();
exactValidationPassed = false;
if ~sampledCollision && minimumClearance_deg > 0 && motion.Success
    exactValidation = validateAzElTrajectory( ...
        motion, obstacles, initialState, goalState, limits, validatorOptions);
    exactValidationPassed = exactValidation.Passed;
end
if ~isnan(objective) && ~exactValidationPassed && ...
        ~sampledCollision && minimumClearance_deg > 0
    objective = objective + options.CollisionPenaltyWeight;
end
motion.Validation = exactValidation;
evaluation = emptyEvaluationRecord();
evaluation.Decision_deg = decision_deg;
evaluation.Objective = objective;
evaluation.MinimumSampledClearance_deg = minimumClearance_deg;
evaluation.SampledCollision = sampledCollision;
evaluation.ExactValidationPassed = exactValidationPassed;
evaluation.MotionDuration_s = motion.MotionDuration_s;
evaluation.IntegratedSquaredJerk_deg2_s5 = ...
    motion.IntegratedSquaredJerk_deg2_s5;
end

function [sampledCollision, minimumClearance_deg, penalty] = ...
        sampledClearancePenalty(motion, obstacles, clearanceTarget_deg)
%% Section 0: Header & Readme
% SYNTAX
%   [sampledCollision, minimumClearance_deg, penalty] = ...
%       sampledClearancePenalty(motion, obstacles, clearanceTarget_deg)
%**************************************************************************
% PURPOSE
%   - Supply a smooth sampled search signal without certifying acceptance.
%**************************************************************************
% INPUTS
%   - motion (scalar sampled motion)
%   - obstacles (canonical obstacle array)
%   - clearanceTarget_deg (nonnegative scalar)
%**************************************************************************
% OUTPUTS
%   - sampledCollision (logical scalar)
%   - minimumClearance_deg (signed scalar)
%   - penalty (nonnegative dimensionless search value)
%**************************************************************************
% UNITS
%   - Clearance is degrees; penalty is dimensionless.
%**************************************************************************
if isempty(obstacles)
    sampledCollision = false;
    minimumClearance_deg = Inf;
    penalty = 0;
    return;
end
[isOccupied, ~, details] = queryAzElTimeObstacle( ...
    obstacles, motion.position_deg(:, 1), motion.position_deg(:, 2), ...
    motion.time_s);
clearance_deg = details.MinimumClearance_deg(:);
sampledCollision = any(isOccupied(:));
minimumClearance_deg = min(clearance_deg);
violation_deg = max(0, clearanceTarget_deg - clearance_deg);
penalty = mean(violation_deg.^2) + mean(isOccupied(:));
if ~isfinite(penalty) || ~isfinite(minimumClearance_deg)
    penalty = realmax("double") / 1e300;
end
end

function printEvaluation(evaluation, sweepIndex, decisionIndex)
%% Section 0: Header & Readme
% SYNTAX
%   printEvaluation(evaluation, sweepIndex, decisionIndex)
%**************************************************************************
% PURPOSE
%   - Print one concise optional research progress row.
%**************************************************************************
% INPUTS
%   - evaluation (scalar evaluation record)
%   - sweepIndex, decisionIndex (positive integers)
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Clearance is degrees; duration is seconds.
%**************************************************************************
fprintf( ...
    "sweep=%d decision=%d evaluation=%d objective=%.9g " + ...
    "clearance_deg=%.9g duration_s=%.9g valid=%d\n", ...
    sweepIndex, decisionIndex, evaluation.EvaluationIndex, ...
    evaluation.Objective, evaluation.MinimumSampledClearance_deg, ...
    evaluation.MotionDuration_s, evaluation.ExactValidationPassed);
end

function record = emptyEvaluationRecord()
%% Section 0: Header & Readme
% SYNTAX
%   record = emptyEvaluationRecord()
%**************************************************************************
% PURPOSE
%   - Define the stable per-candidate optimization trace schema.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - record (scalar empty evaluation record)
%**************************************************************************
% UNITS
%   - Decision and clearance are degrees; duration is seconds.
%**************************************************************************
record = struct( ...
    "EvaluationIndex", 0, ...
    "Decision_deg", zeros(0, 1), ...
    "Objective", Inf, ...
    "MinimumSampledClearance_deg", NaN, ...
    "SampledCollision", false, ...
    "ExactValidationPassed", false, ...
    "MotionDuration_s", NaN, ...
    "IntegratedSquaredJerk_deg2_s5", NaN);
end
