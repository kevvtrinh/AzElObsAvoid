function result = runCorridorPlanner( ...
        result, summaryTemplate, seeds, gridDiagnostics, obstacles, ...
        initialState, goalState, limits, options, planningTimer)
%% Section 0: Header & Readme
% SYNTAX
%   result = azElPlannerMethods.corridor.internal.motion.runCorridorPlanner(result, ...
%       summaryTemplate, seeds, gridDiagnostics, obstacles, ...
%       initialState, goalState, limits, options, planningTimer)
%**************************************************************************
% PURPOSE
%   - Turn topology seeds into complete timed trajectories. For each seed,
%     this function builds a corridor-constrained quintic, tries only the
%     bounded recovery paths that apply to that geometry, records validation
%     evidence, and deterministically selects the best accepted candidate.
%**************************************************************************
% INPUTS
%   - result, summaryTemplate (structs), stable output templates.
%   - seeds (struct array), bounded geometric or timed proposals.
%   - gridDiagnostics (scalar struct), returned search evidence.
%   - obstacles (prepared struct array), protected obstacle histories.
%   - initialState, goalState, limits, options (scalar structs).
%   - planningTimer (tic identifier), whole-planner elapsed-time origin.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct), stable success or expected-failure record.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s powers.
%**************************************************************************

%% Section 1: Prepare Candidate Evaluation

% A seed is only a route suggestion. The parallel arrays below keep each
% suggestion aligned with its candidate, solver inputs, and public summary so
% a failed attempt remains diagnosable and cannot be mistaken for success.
seedSummaries = repmat(summaryTemplate, numel(seeds), 1);
candidates = cell(numel(seeds), 1);
stageTiming = azElPlannerMethods.internal.stageTiming();
timingRoutes_deg = cell(numel(seeds), 1);
timingSolverOptions = cell(numel(seeds), 1);
canImproveTiming = false(numel(seeds), 1);
firstValidatedMotionTime_s = NaN;
candidateElapsedTime_s = 0;
corridorPreparationTimer = tic;
queryOptions = queryAzElTimeObstacle();
envelopePadding_deg = max(1e-6, 1000 * queryOptions.ClearanceTolerance_deg);
geometryIsStatic = true;

% Static obstacle histories can use one exact convex corridor certificate.
% Moving histories need time-local queries instead, even if the seed route
% looks clear in a single snapshot.
for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacles(obstacleIndex);

    % Compare every later slice with the first to prove this obstacle is stationary.
    for sliceIndex = 2:numel(obstacle.az_deg)
        geometryIsStatic = geometryIsStatic && ...
            isequal(obstacle.az_deg{sliceIndex}, obstacle.az_deg{1}) && ...
            isequal(obstacle.el_deg{sliceIndex}, obstacle.el_deg{1});
        if ~geometryIsStatic
            break;
        end
    end
end
% Static geometry can share one conservative envelope across every seed and trial time.
if geometryIsStatic
    obstacleEnvelopeBoundary_deg = azElPlannerMethods.corridor.internal.obstacles.buildEnvelopeBoundary( obstacles, envelopePadding_deg);
else
    obstacleEnvelopeBoundary_deg = zeros(0, 2);
end
stageTiming.CorridorConstructionElapsedTime_s = ...
    stageTiming.CorridorConstructionElapsedTime_s + ...
    toc(corridorPreparationTimer);

%% Section 2: Construct And Validate Every Seed Candidate

% Preserve seed order so diagnostics and final tie-breaking remain deterministic.
for seedIndex = 1:numel(seeds)
    % -- Translate this topology seed into a motion-solver request. --
    % Repeated neighboring points encode an intentional wait. They must not
    % be expanded like an ordinary moving edge or the wait semantics vanish.
    candidateTimer = tic;
    corridorPreparationTimer = tic;
    route_deg = seeds(seedIndex).position_deg;
    zeroLengthSpan = vecnorm(diff(route_deg), 2, 2) <= 1e-12;
    dynamicRouteExpansion_deg = 0;
    % Moving routes without explicit waits receive a small time-local clearance expansion before fitting.
    if ~geometryIsStatic && ~any(zeroLengthSpan)
        [route_deg, dynamicRouteExpansion_deg] = azElPlannerMethods.corridor.internal.search.expandDynamicRoute( ...
            seeds(seedIndex), obstacles, initialState.time_s, goalState.time_s);
    end
    routeVertexTarget = min(22, size(route_deg, 1));
    solverOptions = struct( ...
        "RouteVertexCount", routeVertexTarget, ...
        "RouteTau", seeds(seedIndex).tau, ...
        "SampleTime_s", options.SampleTime_s, ...
        "ObstacleEnvelopeBoundary_deg", obstacleEnvelopeBoundary_deg, ...
        "RequireStaticCorridorCertificate", geometryIsStatic, ...
        "GoalTimeMode", options.GoalTimeMode, "AllowAzimuthWrapping", options.AllowAzimuthWrapping);
    if geometryIsStatic
        % Protected geometry already contains the public safety margin. This
        % small numerical reserve keeps the exact convex certificate strict
        % without forcing the conservative full-route densification fallback.
        solverOptions.ClearanceTarget_deg = 1e-4;
    end
    stageTiming.CorridorConstructionElapsedTime_s = ...
        stageTiming.CorridorConstructionElapsedTime_s + ...
        toc(corridorPreparationTimer);
    if options.Verbose
        fprintf("[AzEl][seed %d/%d][corridorQuintic] start, " + ...
            "source=%s, route=%d.\n", seedIndex, numel(seeds), seeds(seedIndex).Source, size(route_deg, 1));
    end
    candidate = azElPlannerMethods.corridor.internal.motion.solveCorridorQuintic( ...
        obstacles, initialState, goalState, limits, route_deg, solverOptions);
    stageTiming = addStageTiming( ...
        stageTiming, candidate.OptimizerDiagnostics.StageTiming);
    canImproveStaticTiming = geometryIsStatic && candidate.Success && ...
        options.GoalTimeMode == "earliestArrival" && routeVertexTarget == size(route_deg, 1) && size(route_deg, 1) > 2;
    canImproveTiming(seedIndex) = canImproveStaticTiming;
    timingRoutes_deg{seedIndex} = route_deg;
    timingSolverOptions{seedIndex} = solverOptions;
    candidate.OptimizerDiagnostics.TimingSearchUsed = false;
    candidate.OptimizerDiagnostics.TimingTrialCount = 0;
    candidate.OptimizerDiagnostics.TimingInitialDuration_s = candidate.MotionDuration_s;
    candidate.OptimizerDiagnostics.TimingBestDuration_s = candidate.MotionDuration_s;
    candidate.OptimizerDiagnostics.HoldRecoveryUsed = false;
    candidate.OptimizerDiagnostics.HoldMultiplier = 1;
    candidate.OptimizerDiagnostics.HoldTrialCount = 0;
    candidate.OptimizerDiagnostics.DynamicRouteExpansion_deg = dynamicRouteExpansion_deg;
    candidate.OptimizerDiagnostics.StaticExactFallbackUsed = false;
    candidate.OptimizerDiagnostics.StaticExactDensificationFactor = 1;
    candidate.OptimizerDiagnostics.StaticExactTrialCount = 0;
    candidate.OptimizerDiagnostics.DynamicExactDensificationFactor = 1;
    candidate.OptimizerDiagnostics.DynamicExactTrialCount = 0;
    % -- Try bounded recovery families after the primary solve. --
    % These branches change only how the same seed is represented. None may
    % bypass validateAzElTrajectory or replace a previously valid result with
    % an invalid one.
    canTryStaticExact = geometryIsStatic && ~candidate.Success && ...
        size(route_deg, 1) > 2 && ...
        seeds(seedIndex).Source == "visibilityGraph" && 3 * (size(route_deg, 1) - 1) + 1 <= 150;
    % A failed approximate static candidate may still succeed with the bounded exact corridor formulation.
    if canTryStaticExact
        corridorPreparationTimer = tic;
        [expandedRoute_deg, staticExpansion_deg] = azElPlannerMethods.corridor.internal.search.expandDynamicRoute( ...
            seeds(seedIndex), obstacles, initialState.time_s, goalState.time_s);
        stageTiming.CorridorConstructionElapsedTime_s = ...
            stageTiming.CorridorConstructionElapsedTime_s + ...
            toc(corridorPreparationTimer);
        exactTrialCount = 0;

        % Increase spatial resolution gradually and stop at the first certified static result.
        for densificationFactor = 1:3
            corridorPreparationTimer = tic;
            [trialRoute_deg, trialTau] = densifyRoute( expandedRoute_deg, seeds(seedIndex).tau, densificationFactor);
            exactOptions = solverOptions;
            exactOptions.RouteVertexCount = size(trialRoute_deg, 1);
            exactOptions.RouteTau = trialTau;
            exactOptions.ObstacleEnvelopeBoundary_deg = zeros(0, 2);
            exactOptions.RequireStaticCorridorCertificate = false;
            stageTiming.CorridorConstructionElapsedTime_s = ...
                stageTiming.CorridorConstructionElapsedTime_s + ...
                toc(corridorPreparationTimer);
            exactTrial = azElPlannerMethods.corridor.internal.motion.solveCorridorQuintic( ...
                obstacles, initialState, goalState, limits, trialRoute_deg, exactOptions);
            stageTiming = addStageTiming( ...
                stageTiming, exactTrial.OptimizerDiagnostics.StageTiming);
            exactTrialCount = exactTrialCount + 1;
            % Prefer a validated exact trial immediately; otherwise preserve the original failure diagnostics.
            if exactTrial.Success
                candidate = exactTrial;
                candidate.OptimizerDiagnostics.HoldRecoveryUsed = false;
                candidate.OptimizerDiagnostics.HoldMultiplier = 1;
                candidate.OptimizerDiagnostics.HoldTrialCount = 0;
                candidate.OptimizerDiagnostics.DynamicRouteExpansion_deg = staticExpansion_deg;
                candidate.OptimizerDiagnostics.StaticExactFallbackUsed = true;
                candidate.OptimizerDiagnostics. StaticExactDensificationFactor = densificationFactor;
                break;
            end
        end
        candidate.OptimizerDiagnostics.StaticExactTrialCount = exactTrialCount;
        if ~isfield(candidate.OptimizerDiagnostics, "StaticExactFallbackUsed")
            candidate.OptimizerDiagnostics.StaticExactFallbackUsed = false;
            candidate.OptimizerDiagnostics. StaticExactDensificationFactor = 1;
        end
    end
    canTryDynamicExact = ~geometryIsStatic && ~candidate.Success && ...
        ~any(zeroLengthSpan) && size(route_deg, 1) > 2 && ...
        seeds(seedIndex).Source == "visibilityGraph" && 3 * (size(route_deg, 1) - 1) + 1 <= 150;
    % Dynamic exact refinement is reserved for bounded routes where its additional work can be certified.
    if canTryDynamicExact
        dynamicExactTrialCount = 0;

        % Try only the two bounded denser representations allowed for moving geometry.
        for densificationFactor = 2:3
            corridorPreparationTimer = tic;
            [trialRoute_deg, trialTau] = densifyRoute( route_deg, seeds(seedIndex).tau, densificationFactor);
            exactOptions = solverOptions;
            exactOptions.RouteVertexCount = size(trialRoute_deg, 1);
            exactOptions.RouteTau = trialTau;
            exactOptions.ObstacleEnvelopeBoundary_deg = zeros(0, 2);
            exactOptions.RequireStaticCorridorCertificate = false;
            stageTiming.CorridorConstructionElapsedTime_s = ...
                stageTiming.CorridorConstructionElapsedTime_s + ...
                toc(corridorPreparationTimer);
            exactTrial = azElPlannerMethods.corridor.internal.motion.solveCorridorQuintic( ...
                obstacles, initialState, goalState, limits, trialRoute_deg, exactOptions);
            stageTiming = addStageTiming( ...
                stageTiming, exactTrial.OptimizerDiagnostics.StageTiming);
            dynamicExactTrialCount = dynamicExactTrialCount + 1;
            if ~exactTrial.Success && densificationFactor == 2 && options.GoalTimeMode == "earliestArrival"
                [exactTrial, stageTiming] = ...
                    azElPlannerMethods.corridor.internal.motion.retimeDynamicRoute( ...
                    exactTrial, obstacles, initialState, goalState, ...
                    limits, options, trialRoute_deg, stageTiming);
            end
            if exactTrial.Success
                candidate = exactTrial;
                candidate.OptimizerDiagnostics.HoldRecoveryUsed = false;
                candidate.OptimizerDiagnostics.HoldMultiplier = 1;
                candidate.OptimizerDiagnostics.HoldTrialCount = 0;
                candidate.OptimizerDiagnostics.DynamicRouteExpansion_deg = dynamicRouteExpansion_deg;
                candidate.OptimizerDiagnostics.StaticExactFallbackUsed = false;
                candidate.OptimizerDiagnostics. StaticExactDensificationFactor = 1;
                candidate.OptimizerDiagnostics.StaticExactTrialCount = 0;
                candidate.OptimizerDiagnostics. DynamicExactDensificationFactor = densificationFactor;
                break;
            end
        end
        candidate.OptimizerDiagnostics.DynamicExactTrialCount = dynamicExactTrialCount;
        if ~isfield(candidate.OptimizerDiagnostics, "DynamicExactDensificationFactor")
            candidate.OptimizerDiagnostics. DynamicExactDensificationFactor = 1;
        end
    end
    canRecoverTimedHold = ~geometryIsStatic && any(zeroLengthSpan) && ~candidate.Success;
    % Explicit wait spans may need their duration shifted until the moving obstacle clears.
    if canRecoverTimedHold
        [candidate, holdMultiplier, holdTrialCount, stageTiming] = ...
            recoverTimedHold( ...
            candidate, obstacles, initialState, goalState, limits, ...
            route_deg, solverOptions, zeroLengthSpan, stageTiming);
        candidate.OptimizerDiagnostics.HoldRecoveryUsed = candidate.Success;
        candidate.OptimizerDiagnostics.HoldMultiplier = holdMultiplier;
        candidate.OptimizerDiagnostics.HoldTrialCount = holdTrialCount;
    end
    if ~isfield(candidate.OptimizerDiagnostics, "TimingSearchUsed")
        candidate.OptimizerDiagnostics.TimingSearchUsed = false;
        candidate.OptimizerDiagnostics.TimingTrialCount = 0;
        candidate.OptimizerDiagnostics.TimingInitialDuration_s = candidate.MotionDuration_s;
        candidate.OptimizerDiagnostics.TimingBestDuration_s = candidate.MotionDuration_s;
    end
    candidate.SeedIndex = seeds(seedIndex).Index;
    % -- Convert internal solver output into the stable candidate schema. --
    % Selection below reads only these normalized fields, which keeps solver
    % implementation details out of the public result contract.
    candidate.SeedSource = seeds(seedIndex).Source;
    candidate.MotionSource = "corridorQuintic";
    candidate.OptimizerFeasible = candidate.OptimizerDiagnostics.ExitFlag > 0;
    maximumInequality_deg = candidate.OptimizerDiagnostics.CandidateMaximumInequality_deg;
    if isfinite(maximumInequality_deg)
        candidate.MaximumConstraintViolation = max(0, maximumInequality_deg);
    else
        candidate.MaximumConstraintViolation = Inf;
    end
    candidate.SolverDiagnostics = candidate.OptimizerDiagnostics;
    candidate.AnalyticDiagnostics = struct();
    candidates{seedIndex} = candidate;
    seedSummaries(seedIndex) = candidateRecord( candidate, seeds(seedIndex), summaryTemplate);
    candidateElapsedTime_s = candidateElapsedTime_s + toc(candidateTimer);
    % Record time-to-first-solution once; later improvements must not overwrite this diagnostic.
    if candidate.Success && isnan(firstValidatedMotionTime_s)
        firstValidatedMotionTime_s = toc(planningTimer);
    end
    if options.Verbose
        fprintf("[AzEl][seed %d/%d][corridorQuintic] " + ...
            "success=%d, validation=%d, arrival=%.6g s, reason=%s.\n", ...
            seedIndex, numel(seeds), candidate.Success, ...
            candidate.Validation.Passed, candidate.FinalTime_s, candidate.TerminationReason);
    end
end

%% Section 3: Improve One Compact Topology

% The compact C3 solve is an optional improvement of an already validated,
% short route. Its result is accepted only when it also validates and arrives
% strictly earlier than the best retained corridor candidate.
compactEligible = false(numel(seeds), 1);
compactObstacleWork = true;

% Reject compact improvement when any dynamic obstacle history is too large for bounded work.
for obstacleIndex = 1:numel(obstacles)
    preparation = obstacles(obstacleIndex).InternalPreparation;
    stationaryHistory = all(preparation.IntervalSpeedBound_deg_s == 0);
    compactObstacleWork = compactObstacleWork && (stationaryHistory || ...
        max(cellfun(@numel, obstacles(obstacleIndex).az_deg)) <= 256);
end
if compactObstacleWork && options.GoalTimeMode == "earliestArrival"

    % Mark only short validated routes that the fixed eight-span representation can express.
    for seedIndex = 1:numel(seeds)
        route_deg = seeds(seedIndex).position_deg;
        recoveredHold = seedSummaries(seedIndex).ValidationPassed && ...
            seedSummaries(seedIndex).SolverDiagnostics.HoldRecoveryUsed;
        compactEligible(seedIndex) = seedSummaries(seedIndex).ValidationPassed && ...
            size(route_deg, 1) >= 3 && ...
            size(route_deg, 1) <= 10 && (all(vecnorm(diff(route_deg), 2, 2) > 1e-12) || recoveredHold);
    end
end
validatedDuration_s = [seedSummaries.ValidationPassed].' .* [seedSummaries.MotionDuration_s].';
validatedDuration_s(~[seedSummaries.ValidationPassed].') = Inf;
if any(compactEligible) && any(isfinite(validatedDuration_s))
    eligibleIndex = find(compactEligible);
    routeLength_deg = arrayfun(@(index) sum(vecnorm( diff(seeds(index).position_deg), 2, 2)), eligibleIndex);
    [~, shortestIndex] = min(routeLength_deg);
    compactSeedIndex = eligibleIndex(shortestIndex);
    compactTimer = tic;
    [compactMotion, compactValidation, compactDiagnostics] = azElPlannerMethods.corridor.internal.motion.solveCompactC3( ...
        seeds(compactSeedIndex), min(validatedDuration_s), obstacles, initialState, goalState, limits, options);
    candidateElapsedTime_s = candidateElapsedTime_s + toc(compactTimer);
    stageTiming = addStageTiming( ...
        stageTiming, compactDiagnostics.StageTiming);
    result.SearchDiagnostics.CompactC3 = compactDiagnostics;
    if compactDiagnostics.Accepted && compactMotion.MotionDuration_s < min(validatedDuration_s) - 1e-9
        candidate = candidates{compactSeedIndex};
        motionFields = fieldnames(compactMotion);

        % Replace the retained candidate's motion payload while preserving its seed identity.
        for fieldIndex = 1:numel(motionFields)
            candidate.(motionFields{fieldIndex}) = compactMotion.(motionFields{fieldIndex});
        end
        candidate.Validation = compactValidation;
        candidate.Success = true;
        candidate.Message = "The compact C3 duration controller passed validation.";
        candidate.TerminationReason = "compactC3Validated";
        candidate.MotionSource = "corridorQuintic";
        candidate.SeedCorridorBoundary_deg = zeros(0, 2);
        candidate.SeedCorridor = candidate.SeedCorridor([]);
        candidate.OptimizerDiagnostics.CompactC3 = compactDiagnostics;
        candidate.OptimizerDiagnostics.ExitFlag = compactDiagnostics.ExitFlag;
        candidate.OptimizerDiagnostics.ValidationPassed = true;
        candidate.OptimizerDiagnostics.ContinuousClearance_deg = compactValidation.MinimumClearance_deg;
        candidate.OptimizerDiagnostics.MotionDuration_s = compactMotion.MotionDuration_s;
        candidate.OptimizerDiagnostics.CandidateMaximumInequality_deg = 0;
        candidate.OptimizerFeasible = true;
        candidate.MaximumConstraintViolation = 0;
        candidate.SolverDiagnostics = candidate.OptimizerDiagnostics;
        candidates{compactSeedIndex} = candidate;
        seedSummaries(compactSeedIndex) = candidateRecord( candidate, seeds(compactSeedIndex), summaryTemplate);
    end
end

%% Section 4: Select The Earliest Validated Quintic

% Expected failure returns the same result schema and the complete attempt
% summaries. On success, arrival time dominates; near-equal arrivals use exact
% integrated jerk, then seed index, for deterministic repeatability.
result.SeedSummaries = seedSummaries;
result.SearchDiagnostics.SeedSummaries = seedSummaries;
result.FirstValidatedMotionTime_s = firstValidatedMotionTime_s;
result.SearchDiagnostics.FirstValidatedMotionTime_s = firstValidatedMotionTime_s;
result.SearchDiagnostics.ValidatedCandidateCount = nnz([seedSummaries.ValidationPassed]);
result.SearchDiagnostics.SeedGenerationElapsedTime_s = gridDiagnostics.ElapsedTime_s;
result.SearchDiagnostics.FirstMotionElapsedTime_s = candidateElapsedTime_s;
result.SearchDiagnostics.AttemptedSeedCount = numel(seeds);
validatedIndices = find([seedSummaries.ValidationPassed]).';
% No candidate passed the independent validator, so return the best diagnostic failure rather than a motion.
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
    result.SearchDiagnostics.StageTiming = stageTiming;
    return;
end
arrivalTimes_s = [seedSummaries(validatedIndices).ArrivalTime_s].';
minimumArrival_s = min(arrivalTimes_s);
timeEquivalent = validatedIndices( arrivalTimes_s <= minimumArrival_s + options.ArrivalTimeTolerance_s);
jerkValues = [seedSummaries(timeEquivalent).IntegratedSquaredJerk_deg2_s5].';
minimumJerk = min(jerkValues);
selectedIndices = timeEquivalent(jerkValues <= minimumJerk + 1e-12);
selectedSeedIndex = min(selectedIndices);
selectedCandidate = candidates{selectedSeedIndex};
% Improve timing only after topology selection so every seed receives the same initial work budget.
if canImproveTiming(selectedSeedIndex)
    timingTimer = tic;
    [selectedCandidate, stageTiming] = improveStaticCorridorTiming( ...
        selectedCandidate, obstacles, initialState, goalState, limits, ...
        timingRoutes_deg{selectedSeedIndex}, ...
        timingSolverOptions{selectedSeedIndex}, stageTiming);
    candidateElapsedTime_s = candidateElapsedTime_s + toc(timingTimer);
    selectedCandidate.SeedIndex = seeds(selectedSeedIndex).Index;
    selectedCandidate.SeedSource = seeds(selectedSeedIndex).Source;
    selectedCandidate.MotionSource = "corridorQuintic";
    selectedCandidate.OptimizerFeasible = selectedCandidate.OptimizerDiagnostics.ExitFlag > 0;
    maximumInequality_deg = selectedCandidate.OptimizerDiagnostics. CandidateMaximumInequality_deg;
    if isfinite(maximumInequality_deg)
        selectedCandidate.MaximumConstraintViolation = max(0, maximumInequality_deg);
    else
        selectedCandidate.MaximumConstraintViolation = Inf;
    end
    selectedCandidate.SolverDiagnostics = selectedCandidate.OptimizerDiagnostics;
    selectedCandidate.AnalyticDiagnostics = struct();
    seedSummaries(selectedSeedIndex) = candidateRecord( selectedCandidate, seeds(selectedSeedIndex), summaryTemplate);
end
result.SeedSummaries = seedSummaries;
result.SearchDiagnostics.SeedSummaries = seedSummaries;
result.SearchDiagnostics.FirstMotionElapsedTime_s = candidateElapsedTime_s;

%% Section 5: Assemble The Stable Public Result

% Copy only the selected candidate's maintained trajectory fields. Search and
% rejected-candidate evidence already live in SearchDiagnostics/SeedSummaries.
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
result.SeedCorridorBoundary_deg = selectedCandidate.SeedCorridorBoundary_deg;
result.SeedCorridor = selectedCandidate.SeedCorridor;
result.Validation = selectedCandidate.Validation;
result.ElapsedPlanningTime_s = toc(planningTimer);
result.ArrivalTime_s = selectedCandidate.FinalTime_s;
result.TrajectoryDuration_s = selectedCandidate.MotionDuration_s;
result.SearchDiagnostics.TerminationReason = result.TerminationReason;
result.SearchDiagnostics.BestPartialSeedIndex = selectedSeedIndex;
result.OptimalityStatement = "Earliest validated corridor quintic from the finite deterministic " + ...
    "seed set; no global certificate.";
result.SearchDiagnostics.StageTiming = stageTiming;
end


function [route_deg, tau] = densifyRoute(route_deg, tau, factor)
% Insert uniformly parameterized fly-through controls without route stops.
if factor == 1
    return;
end
sourceRoute_deg = route_deg;
sourceTau = tau(:);
edgeCount = size(sourceRoute_deg, 1) - 1;
route_deg = zeros(edgeCount * factor + 1, 2);
tau = zeros(edgeCount * factor + 1, 1);
writeIndex = 1;

% Subdivide each source edge without changing its original normalized timing law.
for edgeIndex = 1:edgeCount
    fraction = (0:factor - 1).' / factor;
    selectedIndex = writeIndex:writeIndex + factor - 1;
    route_deg(selectedIndex, :) = sourceRoute_deg(edgeIndex, :) + ...
        fraction .* (sourceRoute_deg(edgeIndex + 1, :) - sourceRoute_deg(edgeIndex, :));
    tau(selectedIndex) = sourceTau(edgeIndex) + fraction .* (sourceTau(edgeIndex + 1) - sourceTau(edgeIndex));
    writeIndex = writeIndex + factor;
end
route_deg(end, :) = sourceRoute_deg(end, :);
tau(end) = sourceTau(end);
end

function [candidate, stageTiming] = improveStaticCorridorTiming( ...
        candidate, obstacles, initialState, goalState, limits, ...
        route_deg, solverOptions, stageTiming)
% Reduce static earliest arrival with bounded derivative-demand feedback.
expandedRoute_deg = candidate.ExpandedRoute_deg;
edgeLength_deg = vecnorm(diff(expandedRoute_deg), 2, 2);
% Timing redistribution assumes every retained span represents actual motion.
if any(edgeLength_deg <= 0)
    candidate.OptimizerDiagnostics.TimingSearchUsed = false;
    candidate.OptimizerDiagnostics.TimingTrialCount = 0;
    candidate.OptimizerDiagnostics.TimingInitialDuration_s = candidate.MotionDuration_s;
    candidate.OptimizerDiagnostics.TimingBestDuration_s = candidate.MotionDuration_s;
    return;
end
logSpanWeight = 1.1 * log(edgeLength_deg);
initialDuration_s = candidate.MotionDuration_s;
trialCount = 0;
solverOptions.EnableExactTraversal = true;
[trial, trialCount, stageTiming] = solveTimingTrial( ...
    logSpanWeight, trialCount, obstacles, initialState, goalState, ...
    limits, route_deg, solverOptions, stageTiming);
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

% Redistribute span time until derivative demand is balanced or improvement stalls.
while initialTrialImproved && trial.Success && trialCount < maximumTrialCount
    timeDemand = ...
        azElPlannerMethods.corridor.internal.motion.spanTimeDemand( ...
        trial.Polynomial, limits);
    demandError = log(max(minimumDemand, timeDemand));
    demandError = demandError - mean(demandError);
    % Stop when every span's derivative demand is balanced to within roughly half a percent.
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
    [trial, trialCount, stageTiming] = solveTimingTrial( ...
        logSpanWeight, trialCount, obstacles, initialState, goalState, ...
        limits, route_deg, solverOptions, stageTiming);
    if compareUnitGain && trialCount < maximumTrialCount
        unitLogSpanWeight = currentLogSpanWeight + demandError;
        unitLogSpanWeight = unitLogSpanWeight - mean(unitLogSpanWeight);
        [unitTrial, trialCount, stageTiming] = solveTimingTrial( ...
            unitLogSpanWeight, trialCount, obstacles, initialState, ...
            goalState, limits, route_deg, solverOptions, stageTiming);
        if unitTrial.Success && (~trial.Success || unitTrial.MotionDuration_s < trial.MotionDuration_s)
            trial = unitTrial;
            logSpanWeight = unitLogSpanWeight;
        end
    end
    durationImproved = trial.Success && trial.MotionDuration_s < previousDuration_s - 1e-9 * max(1, previousDuration_s);
    % Exact traversal is expensive; stop repeating it once a trial no longer shortens the motion.
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
candidate.OptimizerDiagnostics.TimingBestDuration_s = candidate.MotionDuration_s;
end

function [trial, trialCount, stageTiming] = solveTimingTrial( ...
        logSpanWeight, trialCount, obstacles, initialState, goalState, ...
        limits, route_deg, solverOptions, stageTiming)
% Solve and certify one positive relative knot-span allocation.
spanWeight = exp(logSpanWeight(:));
routeTau = [0; cumsum(spanWeight)];
trialOptions = solverOptions;
trialOptions.RouteTau = routeTau / routeTau(end);
trial = azElPlannerMethods.corridor.internal.motion.solveCorridorQuintic( ...
    obstacles, initialState, goalState, limits, route_deg, trialOptions);
stageTiming = addStageTiming( ...
    stageTiming, trial.OptimizerDiagnostics.StageTiming);
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
summary.UnresolvedIntervalCount = candidate.Validation.UnresolvedIntervalCount;
summary.ArrivalTime_s = candidate.FinalTime_s;
summary.MotionDuration_s = candidate.MotionDuration_s;
summary.IntegratedSquaredJerk_deg2_s5 = candidate.IntegratedSquaredJerk_deg2_s5;
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

function [candidate, retainedMultiplier, trialCount, stageTiming] = ...
        recoverTimedHold( ...
        candidate, obstacles, initialState, goalState, limits, route_deg, ...
        solverOptions, zeroLengthSpan, stageTiming)
% Bracket and bisect the timing of a collision-blocked explicit hold.
baseSpanWeight = diff(solverOptions.RouteTau);
% Cover timing ratios geometrically without increasing the 13-trial budget.
multiplierGrid = 2 .^ (-6:6);
lowerMultiplier = NaN;
upperMultiplier = NaN;
retainedMultiplier = 1;
trialCount = 0;
validatedCandidate = [];

% Scan a geometric hold-duration grid until a validated upper bracket is found.
for multiplierIndex = 1:numel(multiplierGrid)
    multiplier = multiplierGrid(multiplierIndex);
    if multiplier == 1
        trial = candidate;
    else
        trialCount = trialCount + 1;
        trialOptions = holdOptions( solverOptions, baseSpanWeight, zeroLengthSpan, multiplier);
        trial = azElPlannerMethods.corridor.internal.motion.solveCorridorQuintic( ...
            obstacles, initialState, goalState, limits, route_deg, trialOptions);
        stageTiming = addStageTiming( ...
            stageTiming, trial.OptimizerDiagnostics.StageTiming);
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

% Refine the first failed/valid hold-duration bracket without exceeding the trial budget.
for bisectionIndex = 1:maximumBisectionTrialCount
    midpointMultiplier = 0.5 * (lowerMultiplier + upperMultiplier);
    trialCount = trialCount + 1;
    trialOptions = holdOptions( solverOptions, baseSpanWeight, zeroLengthSpan, midpointMultiplier);
    trial = azElPlannerMethods.corridor.internal.motion.solveCorridorQuintic( ...
        obstacles, initialState, goalState, limits, route_deg, trialOptions);
    stageTiming = addStageTiming( ...
        stageTiming, trial.OptimizerDiagnostics.StageTiming);
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

function options = holdOptions(options, baseSpanWeight, zeroLengthSpan, holdMultiplier)
% Scale only stationary seed spans and renormalize route time.
spanWeight = baseSpanWeight;
spanWeight(zeroLengthSpan) = holdMultiplier * spanWeight(zeroLengthSpan);
routeTau = [0; cumsum(spanWeight)];
options.RouteTau = routeTau / routeTau(end);
end

function stageTiming = addStageTiming(stageTiming, contribution)
%% Section 0: Header & Readme
% SYNTAX
%   stageTiming = addStageTiming(stageTiming, contribution)
%**************************************************************************
% PURPOSE
%   - Add one component's five exclusive stages to the planner total.
%**************************************************************************
% INPUTS
%   - stageTiming (scalar struct), shared planner-stage totals.
%   - contribution (scalar struct), component stage contribution.
%**************************************************************************
% OUTPUTS
%   - stageTiming (scalar struct), value-updated exclusive stage totals.
%**************************************************************************
% UNITS
%   - All timing fields are seconds.
%**************************************************************************
stageNames = [ ...
    "TopologyElapsedTime_s", ...
    "CorridorConstructionElapsedTime_s", ...
    "MotionSolvingElapsedTime_s", ...
    "CollisionCheckingElapsedTime_s", ...
    "FinalValidationElapsedTime_s"];

for name = stageNames
    stageTiming.(name) = stageTiming.(name) + contribution.(name);
end
end
