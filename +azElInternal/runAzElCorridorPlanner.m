function result = runAzElCorridorPlanner( ...
        result, summaryTemplate, seeds, gridDiagnostics, obstacles, ...
        initialState, goalState, limits, options, planningTimer)
% Construct, validate, rank, and assemble bounded corridor candidates.
%% Section 1: Construct Every Bounded Experimental Candidate
seedSummaries = repmat(summaryTemplate, numel(seeds), 1);
candidates = cell(numel(seeds), 1);
timingRoutes_deg = cell(numel(seeds), 1);
timingSolverOptions = cell(numel(seeds), 1);
canImproveTiming = false(numel(seeds), 1);
firstValidatedMotionTime_s = NaN;
candidateElapsedTime_s = 0;
queryOptions = queryAzElTimeObstacle();
envelopePadding_deg = max(1e-6, ...
    1000 * queryOptions.ClearanceTolerance_deg);
geometryIsStatic = true;
for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacles(obstacleIndex);
    for sliceIndex = 2:numel(obstacle.az_deg)
        geometryIsStatic = geometryIsStatic && ...
            isequal(obstacle.az_deg{sliceIndex}, obstacle.az_deg{1}) && ...
            isequal(obstacle.el_deg{sliceIndex}, obstacle.el_deg{1});
        if ~geometryIsStatic
            break;
        end
    end
end
if geometryIsStatic
    obstacleEnvelopeBoundary_deg = ...
        azElInternal.buildAzElObstacleEnvelopeBoundary( ...
        obstacles, envelopePadding_deg);
else
    obstacleEnvelopeBoundary_deg = zeros(0, 2);
end
for seedIndex = 1:numel(seeds)
    candidateTimer = tic;
    route_deg = seeds(seedIndex).position_deg;
    zeroLengthSpan = vecnorm(diff(route_deg), 2, 2) <= 1e-12;
    dynamicRouteExpansion_deg = 0;
    if ~geometryIsStatic && ~any(zeroLengthSpan)
        [route_deg, dynamicRouteExpansion_deg] = ...
            azElInternal.expandAzElDynamicSeedRoute( ...
            seeds(seedIndex), obstacles, initialState.time_s, goalState.time_s);
    end
    routeVertexTarget = min(22, size(route_deg, 1));
    solverOptions = struct( ...
        "RouteVertexCount", routeVertexTarget, ...
        "RouteTau", seeds(seedIndex).tau, ...
        "SampleTime_s", options.SampleTime_s, ...
        "ObstacleEnvelopeBoundary_deg", obstacleEnvelopeBoundary_deg, ...
        "RequireStaticCorridorCertificate", geometryIsStatic, ...
        "GoalTimeMode", options.GoalTimeMode, ...
        "AllowAzimuthWrapping", options.AllowAzimuthWrapping);
    if geometryIsStatic
        % Protected geometry already contains the public safety margin. This
        % small numerical reserve keeps the exact convex certificate strict
        % without forcing the conservative full-route densification fallback.
        solverOptions.ClearanceTarget_deg = 1e-4;
    end
    if options.Verbose
        fprintf("[AzEl][seed %d/%d][corridorQuintic] start, " + ...
            "source=%s, route=%d.\n", seedIndex, numel(seeds), ...
            seeds(seedIndex).Source, ...
            size(route_deg, 1));
    end
    candidate = azElInternal.solveAzElCorridorQuintic( ...
        obstacles, initialState, goalState, limits, ...
        route_deg, solverOptions);
    canImproveStaticTiming = geometryIsStatic && candidate.Success && ...
        options.GoalTimeMode == "earliestArrival" && ...
        routeVertexTarget == size(route_deg, 1) && ...
        size(route_deg, 1) > 2;
    canImproveTiming(seedIndex) = canImproveStaticTiming;
    timingRoutes_deg{seedIndex} = route_deg;
    timingSolverOptions{seedIndex} = solverOptions;
    candidate.OptimizerDiagnostics.TimingSearchUsed = false;
    candidate.OptimizerDiagnostics.TimingTrialCount = 0;
    candidate.OptimizerDiagnostics.TimingInitialDuration_s = ...
        candidate.MotionDuration_s;
    candidate.OptimizerDiagnostics.TimingBestDuration_s = ...
        candidate.MotionDuration_s;
    candidate.OptimizerDiagnostics.HoldRecoveryUsed = false;
    candidate.OptimizerDiagnostics.HoldMultiplier = 1;
    candidate.OptimizerDiagnostics.HoldTrialCount = 0;
    candidate.OptimizerDiagnostics.DynamicRouteExpansion_deg = ...
        dynamicRouteExpansion_deg;
    candidate.OptimizerDiagnostics.StaticExactFallbackUsed = false;
    candidate.OptimizerDiagnostics.StaticExactDensificationFactor = 1;
    candidate.OptimizerDiagnostics.StaticExactTrialCount = 0;
    candidate.OptimizerDiagnostics.DynamicExactDensificationFactor = 1;
    candidate.OptimizerDiagnostics.DynamicExactTrialCount = 0;
    canTryStaticExact = geometryIsStatic && ~candidate.Success && ...
        size(route_deg, 1) > 2 && ...
        seeds(seedIndex).Source == "visibilityGraph" && ...
        3 * (size(route_deg, 1) - 1) + 1 <= 150;
    if canTryStaticExact
        [expandedRoute_deg, staticExpansion_deg] = ...
            azElInternal.expandAzElDynamicSeedRoute( ...
            seeds(seedIndex), obstacles, initialState.time_s, goalState.time_s);
        exactTrialCount = 0;
        for densificationFactor = 1:3
            [trialRoute_deg, trialTau] = densifyRoute( ...
                expandedRoute_deg, seeds(seedIndex).tau, densificationFactor);
            exactOptions = solverOptions;
            exactOptions.RouteVertexCount = size(trialRoute_deg, 1);
            exactOptions.RouteTau = trialTau;
            exactOptions.ObstacleEnvelopeBoundary_deg = zeros(0, 2);
            exactOptions.RequireStaticCorridorCertificate = false;
            exactTrial = azElInternal.solveAzElCorridorQuintic( ...
                obstacles, initialState, goalState, limits, ...
                trialRoute_deg, exactOptions);
            exactTrialCount = exactTrialCount + 1;
            if exactTrial.Success
                candidate = exactTrial;
                candidate.OptimizerDiagnostics.HoldRecoveryUsed = false;
                candidate.OptimizerDiagnostics.HoldMultiplier = 1;
                candidate.OptimizerDiagnostics.HoldTrialCount = 0;
                candidate.OptimizerDiagnostics.DynamicRouteExpansion_deg = ...
                    staticExpansion_deg;
                candidate.OptimizerDiagnostics.StaticExactFallbackUsed = true;
                candidate.OptimizerDiagnostics. ...
                    StaticExactDensificationFactor = densificationFactor;
                break;
            end
        end
        candidate.OptimizerDiagnostics.StaticExactTrialCount = exactTrialCount;
        if ~isfield(candidate.OptimizerDiagnostics, "StaticExactFallbackUsed")
            candidate.OptimizerDiagnostics.StaticExactFallbackUsed = false;
            candidate.OptimizerDiagnostics. ...
                StaticExactDensificationFactor = 1;
        end
    end
    canTryDynamicExact = ~geometryIsStatic && ~candidate.Success && ...
        ~any(zeroLengthSpan) && size(route_deg, 1) > 2 && ...
        seeds(seedIndex).Source == "visibilityGraph" && ...
        3 * (size(route_deg, 1) - 1) + 1 <= 150;
    if canTryDynamicExact
        dynamicExactTrialCount = 0;
        for densificationFactor = 2:3
            [trialRoute_deg, trialTau] = densifyRoute( ...
                route_deg, seeds(seedIndex).tau, densificationFactor);
            exactOptions = solverOptions;
            exactOptions.RouteVertexCount = size(trialRoute_deg, 1);
            exactOptions.RouteTau = trialTau;
            exactOptions.ObstacleEnvelopeBoundary_deg = zeros(0, 2);
            exactOptions.RequireStaticCorridorCertificate = false;
            exactTrial = azElInternal.solveAzElCorridorQuintic( ...
                obstacles, initialState, goalState, limits, ...
                trialRoute_deg, exactOptions);
            dynamicExactTrialCount = dynamicExactTrialCount + 1;
            if ~exactTrial.Success && densificationFactor == 2 && ...
                    options.GoalTimeMode == "earliestArrival"
                exactTrial = retimeDynamicRoute( ...
                    exactTrial, obstacles, initialState, goalState, ...
                    limits, options, trialRoute_deg);
            end
            if exactTrial.Success
                candidate = exactTrial;
                candidate.OptimizerDiagnostics.HoldRecoveryUsed = false;
                candidate.OptimizerDiagnostics.HoldMultiplier = 1;
                candidate.OptimizerDiagnostics.HoldTrialCount = 0;
                candidate.OptimizerDiagnostics.DynamicRouteExpansion_deg = ...
                    dynamicRouteExpansion_deg;
                candidate.OptimizerDiagnostics.StaticExactFallbackUsed = false;
                candidate.OptimizerDiagnostics. ...
                    StaticExactDensificationFactor = 1;
                candidate.OptimizerDiagnostics.StaticExactTrialCount = 0;
                candidate.OptimizerDiagnostics. ...
                    DynamicExactDensificationFactor = densificationFactor;
                break;
            end
        end
        candidate.OptimizerDiagnostics.DynamicExactTrialCount = ...
            dynamicExactTrialCount;
        if ~isfield(candidate.OptimizerDiagnostics, ...
                "DynamicExactDensificationFactor")
            candidate.OptimizerDiagnostics. ...
                DynamicExactDensificationFactor = 1;
        end
    end
    canRecoverTimedHold = ~geometryIsStatic && any(zeroLengthSpan) && ...
        ~candidate.Success;
    if canRecoverTimedHold
        [candidate, holdMultiplier, holdTrialCount] = recoverTimedHold( ...
            candidate, obstacles, initialState, goalState, limits, ...
            route_deg, solverOptions, zeroLengthSpan);
        candidate.OptimizerDiagnostics.HoldRecoveryUsed = ...
            candidate.Success;
        candidate.OptimizerDiagnostics.HoldMultiplier = holdMultiplier;
        candidate.OptimizerDiagnostics.HoldTrialCount = holdTrialCount;
    end
    if ~isfield(candidate.OptimizerDiagnostics, "TimingSearchUsed")
        candidate.OptimizerDiagnostics.TimingSearchUsed = false;
        candidate.OptimizerDiagnostics.TimingTrialCount = 0;
        candidate.OptimizerDiagnostics.TimingInitialDuration_s = ...
            candidate.MotionDuration_s;
        candidate.OptimizerDiagnostics.TimingBestDuration_s = ...
            candidate.MotionDuration_s;
    end
    candidate.SeedIndex = seeds(seedIndex).Index;
    candidate.SeedSource = seeds(seedIndex).Source;
    candidate.MotionSource = "corridorQuintic";
    candidate.OptimizerFeasible = ...
        candidate.OptimizerDiagnostics.ExitFlag > 0;
    maximumInequality_deg = ...
        candidate.OptimizerDiagnostics.CandidateMaximumInequality_deg;
    if isfinite(maximumInequality_deg)
        candidate.MaximumConstraintViolation = ...
            max(0, maximumInequality_deg);
    else
        candidate.MaximumConstraintViolation = Inf;
    end
    candidate.SolverDiagnostics = candidate.OptimizerDiagnostics;
    candidate.AnalyticDiagnostics = struct();
    candidates{seedIndex} = candidate;
    seedSummaries(seedIndex) = candidateRecord( ...
        candidate, seeds(seedIndex), summaryTemplate);
    candidateElapsedTime_s = candidateElapsedTime_s + toc(candidateTimer);
    if candidate.Success && isnan(firstValidatedMotionTime_s)
        firstValidatedMotionTime_s = toc(planningTimer);
    end
    if options.Verbose
        fprintf("[AzEl][seed %d/%d][corridorQuintic] " + ...
            "success=%d, validation=%d, arrival=%.6g s, reason=%s.\n", ...
            seedIndex, numel(seeds), candidate.Success, ...
            candidate.Validation.Passed, ...
            candidate.FinalTime_s, candidate.TerminationReason);
    end
end
%% Section 2: Improve One Compact Topology
compactEligible = false(numel(seeds), 1);
compactObstacleWork = true;
for obstacleIndex = 1:numel(obstacles)
    preparation = obstacles(obstacleIndex).InternalPreparation;
    stationaryHistory = all(preparation.IntervalSpeedBound_deg_s == 0);
    compactObstacleWork = compactObstacleWork && (stationaryHistory || ...
        max(cellfun(@numel, obstacles(obstacleIndex).az_deg)) <= 256);
end
if compactObstacleWork && options.GoalTimeMode == "earliestArrival"
    for seedIndex = 1:numel(seeds)
        route_deg = seeds(seedIndex).position_deg;
        recoveredHold = seedSummaries(seedIndex).ValidationPassed && ...
            seedSummaries(seedIndex).SolverDiagnostics.HoldRecoveryUsed;
        compactEligible(seedIndex) = seedSummaries(seedIndex).ValidationPassed && ...
            size(route_deg, 1) >= 3 && ...
            size(route_deg, 1) <= 10 && ...
            (all(vecnorm(diff(route_deg), 2, 2) > 1e-12) || recoveredHold);
    end
end
validatedDuration_s = [seedSummaries.ValidationPassed].' .* ...
    [seedSummaries.MotionDuration_s].';
validatedDuration_s(~[seedSummaries.ValidationPassed].') = Inf;
if any(compactEligible) && any(isfinite(validatedDuration_s))
    eligibleIndex = find(compactEligible);
    routeLength_deg = arrayfun(@(index) sum(vecnorm( ...
        diff(seeds(index).position_deg), 2, 2)), eligibleIndex);
    [~, shortestIndex] = min(routeLength_deg);
    compactSeedIndex = eligibleIndex(shortestIndex);
    compactTimer = tic;
    [compactMotion, compactValidation, compactDiagnostics] = ...
        azElInternal.solveAzElCompactC3( ...
        seeds(compactSeedIndex), min(validatedDuration_s), obstacles, ...
        initialState, goalState, limits, options);
    candidateElapsedTime_s = candidateElapsedTime_s + toc(compactTimer);
    if compactDiagnostics.Accepted && ...
            compactMotion.MotionDuration_s < min(validatedDuration_s) - 1e-9
        candidate = candidates{compactSeedIndex};
        motionFields = fieldnames(compactMotion);
        for fieldIndex = 1:numel(motionFields)
            candidate.(motionFields{fieldIndex}) = ...
                compactMotion.(motionFields{fieldIndex});
        end
        candidate.Validation = compactValidation;
        candidate.Success = true;
        candidate.Message = ...
            "The compact C3 duration controller passed validation.";
        candidate.TerminationReason = "compactC3Validated";
        candidate.MotionSource = "corridorQuintic";
        candidate.SeedCorridorBoundary_deg = zeros(0, 2);
        candidate.SeedCorridor = candidate.SeedCorridor([]);
        candidate.OptimizerDiagnostics.CompactC3 = compactDiagnostics;
        candidate.OptimizerDiagnostics.ExitFlag = compactDiagnostics.ExitFlag;
        candidate.OptimizerDiagnostics.ValidationPassed = true;
        candidate.OptimizerDiagnostics.ContinuousClearance_deg = ...
            compactValidation.MinimumClearance_deg;
        candidate.OptimizerDiagnostics.MotionDuration_s = ...
            compactMotion.MotionDuration_s;
        candidate.OptimizerDiagnostics.CandidateMaximumInequality_deg = 0;
        candidate.OptimizerFeasible = true;
        candidate.MaximumConstraintViolation = 0;
        candidate.SolverDiagnostics = candidate.OptimizerDiagnostics;
        candidates{compactSeedIndex} = candidate;
        seedSummaries(compactSeedIndex) = candidateRecord( ...
            candidate, seeds(compactSeedIndex), summaryTemplate);
    end
end
%% Section 2: Select The Earliest Validated Quintic
result.SeedSummaries = seedSummaries;
result.SearchDiagnostics.SeedSummaries = seedSummaries;
result.FirstValidatedMotionTime_s = firstValidatedMotionTime_s;
result.SearchDiagnostics.FirstValidatedMotionTime_s = ...
    firstValidatedMotionTime_s;
result.SearchDiagnostics.ValidatedCandidateCount = ...
    nnz([seedSummaries.ValidationPassed]);
result.SearchDiagnostics.SeedGenerationElapsedTime_s = ...
    gridDiagnostics.ElapsedTime_s;
result.SearchDiagnostics.FirstMotionElapsedTime_s = candidateElapsedTime_s;
result.SearchDiagnostics.AttemptedSeedCount = numel(seeds);
validatedIndices = find([seedSummaries.ValidationPassed]).';
if isempty(validatedIndices)
    result.Message = "No corridor-quintic seed passed independent validation.";
    result.TerminationReason = "noValidatedSeed";
    result.ElapsedPlanningTime_s = toc(planningTimer);
    result.SearchDiagnostics.TerminationReason = result.TerminationReason;
    partialViolation = [seedSummaries.MaximumConstraintViolation];
    partialViolation(~isfinite(partialViolation)) = Inf;
    [~, bestPartialIndex] = min([partialViolation Inf]);
    if bestPartialIndex > numel(seedSummaries)
        bestPartialIndex = 0;
    end
    result.SearchDiagnostics.BestPartialSeedIndex = bestPartialIndex;
    return;
end
arrivalTimes_s = [seedSummaries(validatedIndices).ArrivalTime_s].';
minimumArrival_s = min(arrivalTimes_s);
timeEquivalent = validatedIndices( ...
    arrivalTimes_s <= minimumArrival_s + options.ArrivalTimeTolerance_s);
jerkValues = ...
    [seedSummaries(timeEquivalent).IntegratedSquaredJerk_deg2_s5].';
minimumJerk = min(jerkValues);
selectedIndices = timeEquivalent(jerkValues <= minimumJerk + 1e-12);
selectedSeedIndex = min(selectedIndices);
selectedCandidate = candidates{selectedSeedIndex};
if canImproveTiming(selectedSeedIndex)
    timingTimer = tic;
    selectedCandidate = improveStaticCorridorTiming( ...
        selectedCandidate, obstacles, initialState, goalState, limits, ...
        timingRoutes_deg{selectedSeedIndex}, ...
        timingSolverOptions{selectedSeedIndex});
    candidateElapsedTime_s = candidateElapsedTime_s + toc(timingTimer);
    selectedCandidate.SeedIndex = seeds(selectedSeedIndex).Index;
    selectedCandidate.SeedSource = seeds(selectedSeedIndex).Source;
    selectedCandidate.MotionSource = "corridorQuintic";
    selectedCandidate.OptimizerFeasible = ...
        selectedCandidate.OptimizerDiagnostics.ExitFlag > 0;
    maximumInequality_deg = selectedCandidate.OptimizerDiagnostics. ...
        CandidateMaximumInequality_deg;
    if isfinite(maximumInequality_deg)
        selectedCandidate.MaximumConstraintViolation = ...
            max(0, maximumInequality_deg);
    else
        selectedCandidate.MaximumConstraintViolation = Inf;
    end
    selectedCandidate.SolverDiagnostics = ...
        selectedCandidate.OptimizerDiagnostics;
    selectedCandidate.AnalyticDiagnostics = struct();
    seedSummaries(selectedSeedIndex) = candidateRecord( ...
        selectedCandidate, seeds(selectedSeedIndex), summaryTemplate);
end
result.SeedSummaries = seedSummaries;
result.SearchDiagnostics.SeedSummaries = seedSummaries;
result.SearchDiagnostics.FirstMotionElapsedTime_s = candidateElapsedTime_s;
%% Section 3: Assemble The Stable Public Result
result.Success = true;
result.Message = "An independently validated corridor quintic was selected.";
result.TerminationReason = "goalReached";
result.SelectedSeedIndex = selectedSeedIndex;
result.SelectedMotionSource = "corridorQuintic";
result.SelectedSeed_deg = seeds(selectedSeedIndex).position_deg;
result.time_s = selectedCandidate.time_s;
result.position_deg = selectedCandidate.position_deg;
result.velocity_deg_s = selectedCandidate.velocity_deg_s;
result.acceleration_deg_s2 = selectedCandidate.acceleration_deg_s2;
result.jerk_deg_s3 = selectedCandidate.jerk_deg_s3;
result.Polynomial = selectedCandidate.Polynomial;
result.SeedCorridorBoundary_deg = ...
    selectedCandidate.SeedCorridorBoundary_deg;
result.SeedCorridor = selectedCandidate.SeedCorridor;
result.Validation = selectedCandidate.Validation;
result.ElapsedPlanningTime_s = toc(planningTimer);
result.ArrivalTime_s = selectedCandidate.FinalTime_s;
result.TrajectoryDuration_s = selectedCandidate.MotionDuration_s;
result.SearchDiagnostics.TerminationReason = result.TerminationReason;
result.SearchDiagnostics.BestPartialSeedIndex = selectedSeedIndex;
result.OptimalityStatement = ...
    "Earliest validated corridor quintic from the finite deterministic " + ...
    "seed set; no global certificate.";
end
%% Section 4: Local Functions
function [route_deg, tau] = densifyRoute(route_deg, tau, factor)
% PURPOSE
%   - Insert uniformly parameterized fly-through controls without route stops.
if factor == 1
    return;
end
sourceRoute_deg = route_deg;
sourceTau = tau(:);
edgeCount = size(sourceRoute_deg, 1) - 1;
route_deg = zeros(edgeCount * factor + 1, 2);
tau = zeros(edgeCount * factor + 1, 1);
writeIndex = 1;
for edgeIndex = 1:edgeCount
    fraction = (0:factor - 1).' / factor;
    selectedIndex = writeIndex:writeIndex + factor - 1;
    route_deg(selectedIndex, :) = sourceRoute_deg(edgeIndex, :) + ...
        fraction .* (sourceRoute_deg(edgeIndex + 1, :) - ...
        sourceRoute_deg(edgeIndex, :));
    tau(selectedIndex) = sourceTau(edgeIndex) + ...
        fraction .* (sourceTau(edgeIndex + 1) - sourceTau(edgeIndex));
    writeIndex = writeIndex + factor;
end
route_deg(end, :) = sourceRoute_deg(end, :);
tau(end) = sourceTau(end);
end
function candidate = improveStaticCorridorTiming( ...
        candidate, obstacles, initialState, goalState, limits, ...
        route_deg, solverOptions)
% Reduce static earliest arrival with bounded derivative-demand feedback.
expandedRoute_deg = candidate.ExpandedRoute_deg;
edgeLength_deg = vecnorm(diff(expandedRoute_deg), 2, 2);
if any(edgeLength_deg <= 0)
    candidate.OptimizerDiagnostics.TimingSearchUsed = false;
    candidate.OptimizerDiagnostics.TimingTrialCount = 0;
    candidate.OptimizerDiagnostics.TimingInitialDuration_s = ...
        candidate.MotionDuration_s;
    candidate.OptimizerDiagnostics.TimingBestDuration_s = ...
        candidate.MotionDuration_s;
    return;
end
logSpanWeight = 1.1 * log(edgeLength_deg);
initialDuration_s = candidate.MotionDuration_s;
trialCount = 0;
solverOptions.EnableExactTraversal = true;
[trial, trialCount] = solveTimingTrial( ...
    logSpanWeight, trialCount, obstacles, initialState, goalState, limits, ...
    route_deg, solverOptions);
if ~trial.OptimizerDiagnostics.ExactTraversalAccepted
    solverOptions.EnableExactTraversal = false;
end
initialTrialImproved = trial.Success && trial.MotionDuration_s < ...
    candidate.MotionDuration_s - 1e-9 * max(1, candidate.MotionDuration_s);
if initialTrialImproved
    candidate = trial;
end
maximumTrialCount = 16;
controllerGain = 0.7;
minimumDemand = 0.1;
previousLogSpanWeight = zeros(0, 1);
previousDemandError = zeros(0, 1);
while initialTrialImproved && trial.Success && ...
        trialCount < maximumTrialCount
    timeDemand = spanTimeDemand(trial.Polynomial, limits);
    demandError = log(max(minimumDemand, timeDemand));
    demandError = demandError - mean(demandError);
    if max(abs(demandError)) <= log(1.005)
        break;
    end
    canUseSecantGain = trial.OptimizerDiagnostics.ExactTraversalAccepted;
    if canUseSecantGain && ~isempty(previousDemandError)
        weightStep = logSpanWeight - previousLogSpanWeight;
        demandStep = demandError - previousDemandError;
        demandCurvature = demandStep.' * demandStep;
        if demandCurvature > eps
            secantGain = -(weightStep.' * demandStep) / demandCurvature;
            if isfinite(secantGain) && secantGain > 0
                controllerGain = min(1, max(0.1, secantGain));
            end
        end
    end
    compareUnitGain = canUseSecantGain && isempty(previousDemandError);
    currentLogSpanWeight = logSpanWeight;
    previousLogSpanWeight = logSpanWeight;
    previousDemandError = demandError;
    logSpanWeight = logSpanWeight + controllerGain * demandError;
    logSpanWeight = logSpanWeight - mean(logSpanWeight);
    previousDuration_s = trial.MotionDuration_s;
    [trial, trialCount] = solveTimingTrial( ...
        logSpanWeight, trialCount, obstacles, initialState, goalState, ...
        limits, route_deg, solverOptions);
    if compareUnitGain && trialCount < maximumTrialCount
        unitLogSpanWeight = currentLogSpanWeight + demandError;
        unitLogSpanWeight = unitLogSpanWeight - mean(unitLogSpanWeight);
        [unitTrial, trialCount] = solveTimingTrial( ...
            unitLogSpanWeight, trialCount, obstacles, initialState, ...
            goalState, limits, route_deg, solverOptions);
        if unitTrial.Success && (~trial.Success || ...
                unitTrial.MotionDuration_s < trial.MotionDuration_s)
            trial = unitTrial;
            logSpanWeight = unitLogSpanWeight;
        end
    end
    durationImproved = trial.Success && trial.MotionDuration_s < ...
        previousDuration_s - 1e-9 * max(1, previousDuration_s);
    if solverOptions.EnableExactTraversal && ~durationImproved
        break;
    end
    if trial.Success && trial.MotionDuration_s < candidate.MotionDuration_s
        candidate = trial;
    end
end
candidate.OptimizerDiagnostics.TimingSearchUsed = true;
candidate.OptimizerDiagnostics.TimingTrialCount = trialCount;
candidate.OptimizerDiagnostics.TimingInitialDuration_s = initialDuration_s;
candidate.OptimizerDiagnostics.TimingBestDuration_s = ...
    candidate.MotionDuration_s;
end
function timeDemand = spanTimeDemand(polynomial, limits)
% Return the local time dilation implied by each span's derivative usage.
normalizedTime = linspace(0, 1, 33).';
derivativeArrays = {polynomial.velocityPower_deg_s, ...
    polynomial.accelerationPower_deg_s2, ...
    polynomial.jerkPower_deg_s3};
derivativeLimits = {limits.maxVelocity_deg_s, ...
    limits.maxAcceleration_deg_s2, limits.maxJerk_deg_s3};
spanCount = polynomial.SegmentCount;
timeDemand = zeros(spanCount, 1);
for derivativeOrder = 1:3
    powerArray = derivativeArrays{derivativeOrder};
    basis = normalizedTime.^(0:size(powerArray, 3) - 1);
    for spanIndex = 1:spanCount
        coefficient = reshape(powerArray(spanIndex, :, :), 2, []).';
        derivativeValue = basis * coefficient;
        utilization = max(abs(derivativeValue) ./ ...
            derivativeLimits{derivativeOrder}, [], "all");
        timeDemand(spanIndex) = max(timeDemand(spanIndex), ...
            utilization^(1 / derivativeOrder));
    end
end
end
function candidate = retimeDynamicRoute( ...
        candidate, obstacles, initialState, goalState, limits, ...
        plannerOptions, route_deg)
% Retime one dynamic route cheaply before authoritative validation.
edgeLength_deg = vecnorm(diff(route_deg), 2, 2);
if any(edgeLength_deg <= 0)
    return;
end
spanWeights = edgeLength_deg .^ 1.1;
bestMotion = [];
bestDuration_s = Inf;
for iterationIndex = 0:8
    trialMotion = azElInternal.buildAzElQuinticSpline( ...
        route_deg, initialState, goalState, limits, struct( ...
        "SpanWeights", spanWeights, ...
        "SampleTime_s", plannerOptions.SampleTime_s, ...
        "GoalTimeMode", plannerOptions.GoalTimeMode, ...
        "AllowAzimuthWrapping", plannerOptions.AllowAzimuthWrapping));
    if trialMotion.Success && trialMotion.MotionDuration_s < bestDuration_s
        bestMotion = trialMotion;
        bestDuration_s = trialMotion.MotionDuration_s;
    end
    demandError = log(max(0.1, ...
        spanTimeDemand(trialMotion.Polynomial, limits)));
    demandError = demandError - mean(demandError);
    spanWeights = exp(log(spanWeights) + demandError);
    spanWeights = spanWeights / mean(spanWeights);
end
if isempty(bestMotion)
    return;
end
validation = validateAzElTrajectory( ...
    bestMotion, obstacles, initialState, goalState, limits, plannerOptions);
if ~validation.Passed
    return;
end
if size(route_deg, 1) >= 12
    [bestMotion, validation] = improveDynamicGeometry( ...
        bestMotion, validation, obstacles, initialState, goalState, ...
        limits, plannerOptions, spanWeights);
end
motionFields = fieldnames(bestMotion);
for fieldIndex = 1:numel(motionFields)
    candidate.(motionFields{fieldIndex}) = bestMotion.(motionFields{fieldIndex});
end
candidate.Validation = validation;
candidate.Success = true;
candidate.Message = "The path-fixed-point dynamic retime passed validation.";
candidate.TerminationReason = "quinticValidated";
candidate.OptimizerDiagnostics.ValidationPassed = true;
candidate.OptimizerDiagnostics.ContinuousClearance_deg = ...
    validation.MinimumClearance_deg;
candidate.OptimizerDiagnostics.MotionDuration_s = bestMotion.MotionDuration_s;
end
function [bestMotion, bestValidation] = improveDynamicGeometry( ...
        bestMotion, bestValidation, obstacles, initialState, goalState, ...
        limits, plannerOptions, spanWeights)
% Apply bounded minimum-jerk feedback inside time-local safe-side barriers.
route_deg = [initialState.position_deg; ...
    bestMotion.ControlPoint_deg(4:end - 3, :); goalState.position_deg];
interiorCount = size(route_deg, 1) - 2;
decisionCount = 2 * interiorCount;
maximumIterationCount = 24;
for iterationIndex = 1:maximumIterationCount
    affineBasis = azElInternal.convertAzElBsplineToPolynomial( ...
        eye(size(bestMotion.ControlPoint_deg, 1)), 5, ...
        initialState.time_s, bestMotion.SpanDuration_s);
    accepted = false;
    trustRadius_deg = 0.5;
    for backtrackIndex = 1:3
        [barrierMatrix, barrierBound] = dynamicBarrierRows( ...
            bestMotion, affineBasis, obstacles, interiorCount, 0.05, 0.5);
        [decision_deg, diagnostics] = ...
            azElInternal.optimizeAzElExactTraversal( ...
            bestMotion, affineBasis, zeros(decisionCount, 1), ...
            barrierMatrix, barrierBound, trustRadius_deg, limits);
        trialMotion = azElInternal.buildAzElQuinticSpline( ...
            route_deg, initialState, goalState, limits, struct( ...
            "SpanWeights", spanWeights, ...
            "ControlPointOffsets_deg", ...
            [decision_deg(1:interiorCount), ...
            decision_deg(interiorCount + 1:end)], ...
            "SampleTime_s", plannerOptions.SampleTime_s, ...
            "GoalTimeMode", plannerOptions.GoalTimeMode, ...
            "AllowAzimuthWrapping", plannerOptions.AllowAzimuthWrapping));
        trialValidation = validateAzElTrajectory( ...
            trialMotion, obstacles, initialState, goalState, ...
            limits, plannerOptions);
        accepted = diagnostics.Accepted && trialValidation.Passed && ...
            trialMotion.MotionDuration_s < bestMotion.MotionDuration_s - 1e-9;
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
end
end
function [matrix, bound] = dynamicBarrierRows( ...
        motion, affineBasis, obstacles, interiorCount, ...
        clearanceTarget_deg, activationRadius_deg)
% Linearize protected exterior half-planes at time-local closest points.
time_s = unique([linspace(motion.time_s(1), motion.time_s(end), 161).'; ...
    motion.Polynomial.SegmentStartTime_s; motion.time_s(end)]);
minimumTimes_s = zeros(motion.Polynomial.SegmentCount, 1);
for segmentIndex = 1:motion.Polynomial.SegmentCount
    intervalStart_s = motion.Polynomial.SegmentStartTime_s(segmentIndex);
    intervalEnd_s = intervalStart_s + ...
        motion.Polynomial.SegmentDuration_s(segmentIndex);
    minimumTimes_s(segmentIndex) = fminbnd( ...
        @(queryTime_s) clearanceAtTime( ...
        motion.Polynomial, obstacles, queryTime_s), ...
        intervalStart_s, intervalEnd_s, ...
        optimset("Display", "off", "TolX", 1e-5));
end
time_s = unique([time_s; minimumTimes_s]);
[~, position_deg] = azElInternal.evaluateAzElPolynomial( ...
    motion.Polynomial, time_s);
matrix = zeros(numel(time_s), 2 * interiorCount);
bound = zeros(numel(time_s), 1);
barrierCount = 0;
activationDistance_deg = sqrt(2) * activationRadius_deg + ...
    clearanceTarget_deg;
for timeIndex = 1:numel(time_s)
    nearestClearance_deg = Inf;
    nearestPoint_deg = [NaN NaN];
    for obstacleIndex = 1:numel(obstacles)
        shape = azElInternal.obstacleShapeAtTime( ...
            obstacles(obstacleIndex), time_s(timeIndex));
        [clearance_deg, obstaclePoint_deg] = ...
            azElInternal.pointPolygonClearance( ...
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
    outward = outward / norm(outward);
    segmentIndex = min(motion.Polynomial.SegmentCount, ...
        sum(time_s(timeIndex) >= ...
        motion.Polynomial.SegmentStartTime_s(2:end)) + 1);
    tau = (time_s(timeIndex) - ...
        motion.Polynomial.SegmentStartTime_s(segmentIndex)) / ...
        motion.Polynomial.SegmentDuration_s(segmentIndex);
    basisValue = zeros(1, interiorCount);
    for interiorIndex = 1:interiorCount
        coefficient = reshape(affineBasis.positionPower_deg( ...
            segmentIndex, interiorIndex + 3, :), 1, []);
        basisValue(interiorIndex) = ...
            (tau .^ (0:numel(coefficient) - 1)) * coefficient.';
    end
    barrierCount = barrierCount + 1;
    matrix(barrierCount, :) = -[ ...
        outward(1) * basisValue, outward(2) * basisValue];
    bound(barrierCount) = outward * ...
        (position_deg(timeIndex, :) - nearestPoint_deg).' - ...
        clearanceTarget_deg;
end
matrix = matrix(1:barrierCount, :);
bound = bound(1:barrierCount);
end
function clearance_deg = clearanceAtTime(polynomial, obstacles, time_s)
% Evaluate nearest protected-obstacle clearance at one physical time.
[~, position_deg] = azElInternal.evaluateAzElPolynomial(polynomial, time_s);
clearance_deg = Inf;
for obstacleIndex = 1:numel(obstacles)
    shape = azElInternal.obstacleShapeAtTime(obstacles(obstacleIndex), time_s);
    clearance_deg = min(clearance_deg, ...
        azElInternal.pointPolygonClearance(shape, position_deg));
end
end
function [trial, trialCount] = solveTimingTrial( ...
        logSpanWeight, trialCount, obstacles, initialState, goalState, ...
        limits, route_deg, solverOptions)
% Solve and certify one positive relative knot-span allocation.
spanWeight = exp(logSpanWeight(:));
routeTau = [0; cumsum(spanWeight)];
trialOptions = solverOptions;
trialOptions.RouteTau = routeTau / routeTau(end);
trial = azElInternal.solveAzElCorridorQuintic( ...
    obstacles, initialState, goalState, limits, route_deg, trialOptions);
trialCount = trialCount + 1;
end
function summary = candidateRecord(candidate, seed, template)
% Convert one candidate into the stable seed-summary schema.
summary = template;
summary.SeedIndex = seed.Index;
summary.SeedSource = seed.Source;
summary.OptimizerFeasible = candidate.OptimizerFeasible;
summary.ValidationPassed = candidate.Success;
summary.CollisionFree = candidate.Validation.CollisionFree;
summary.CollisionResolved = candidate.Validation.CollisionResolved;
summary.MinimumClearance_deg = candidate.Validation.MinimumClearance_deg;
summary.UnresolvedIntervalCount = ...
    candidate.Validation.UnresolvedIntervalCount;
summary.ArrivalTime_s = candidate.FinalTime_s;
summary.MotionDuration_s = candidate.MotionDuration_s;
summary.IntegratedSquaredJerk_deg2_s5 = ...
    candidate.IntegratedSquaredJerk_deg2_s5;
summary.MaximumConstraintViolation = candidate.MaximumConstraintViolation;
summary.FirstMotionAttempted = true;
summary.FirstMotionValidationPassed = candidate.Success;
summary.FirstMotionTerminationReason = candidate.TerminationReason;
summary.FirstMotionDiagnostics = candidate.OptimizerDiagnostics;
summary.TerminationReason = candidate.TerminationReason;
summary.Message = candidate.Message + " " + candidate.Validation.Message;
summary.SolverDiagnostics = candidate.OptimizerDiagnostics;
if candidate.Success
    summary.SelectedMotionSource = "corridorQuintic";
end
end
function [candidate, retainedMultiplier, trialCount] = recoverTimedHold( ...
        candidate, obstacles, initialState, goalState, limits, route_deg, ...
        solverOptions, zeroLengthSpan)
% Bracket and bisect the timing of a collision-blocked explicit hold.
baseSpanWeight = diff(solverOptions.RouteTau);
% Cover timing ratios geometrically without increasing the 13-trial budget.
multiplierGrid = 2 .^ (-6:6);
lowerMultiplier = NaN;
upperMultiplier = NaN;
retainedMultiplier = 1;
trialCount = 0;
validatedCandidate = [];
for multiplierIndex = 1:numel(multiplierGrid)
    multiplier = multiplierGrid(multiplierIndex);
    if multiplier == 1
        trial = candidate;
    else
        trialCount = trialCount + 1;
        trialOptions = holdOptions( ...
            solverOptions, baseSpanWeight, zeroLengthSpan, multiplier);
        trial = azElInternal.solveAzElCorridorQuintic( ...
            obstacles, initialState, goalState, limits, route_deg, ...
            trialOptions);
    end
    if trial.Success
        validatedCandidate = trial;
        retainedMultiplier = multiplier;
        upperMultiplier = multiplier;
        if multiplierIndex > 1
            lowerMultiplier = multiplierGrid(multiplierIndex - 1);
        end
        break;
    end
end
if isempty(validatedCandidate) || isnan(lowerMultiplier)
    if ~isempty(validatedCandidate)
        candidate = validatedCandidate;
    end
    return;
end
maximumBisectionTrialCount = 6;
for bisectionIndex = 1:maximumBisectionTrialCount
    midpointMultiplier = 0.5 * (lowerMultiplier + upperMultiplier);
    trialCount = trialCount + 1;
    trialOptions = holdOptions( ...
        solverOptions, baseSpanWeight, zeroLengthSpan, midpointMultiplier);
    trial = azElInternal.solveAzElCorridorQuintic( ...
        obstacles, initialState, goalState, limits, route_deg, trialOptions);
    if trial.Success
        validatedCandidate = trial;
        retainedMultiplier = midpointMultiplier;
        upperMultiplier = midpointMultiplier;
    else
        lowerMultiplier = midpointMultiplier;
    end
end
candidate = validatedCandidate;
end

function options = holdOptions( ...
        options, baseSpanWeight, zeroLengthSpan, holdMultiplier)
% Scale only stationary seed spans and renormalize route time.
spanWeight = baseSpanWeight;
spanWeight(zeroLengthSpan) = ...
    holdMultiplier * spanWeight(zeroLengthSpan);
routeTau = [0; cumsum(spanWeight)];
options.RouteTau = routeTau / routeTau(end);
end
