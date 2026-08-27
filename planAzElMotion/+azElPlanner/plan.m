function result = plan(obstacles, initialState, goalState, limits, optionOverrides)
%% Section 0: Standalone Hermite-Simpson Planner
% Generate neutral topology proposals, solve each selected proposal with the
% HS3 transcription, and accept only canonical independently valid motion.

if nargin == 0
    result = azElInput.resolveHs3Options();
    return;
end
if nargin < 4
    error("planAzElMotion:MissingInputs", ...
        "obstacles, initialState, goalState, and limits are required.");
end
if nargin < 5 || isempty(optionOverrides)
    optionOverrides = struct();
end
planningTimer = tic;
options = azElInput.resolveHs3Options(optionOverrides);
[obstacles, initialState, goalState, limits] = ...
    azElInput.normalizePlannerRequest( ...
    obstacles, initialState, goalState, limits, options);
[result, summaryTemplate] = azElPlanner.emptyPlannerResult( ...
    obstacles, initialState, goalState, limits, options, ...
    validateAzElTrajectory());
obstacles = azElObstacles.prepareDynamic(obstacles);
stageTiming = result.SearchDiagnostics.StageTiming;

%% Section 1: Reject Physically Invalid Endpoints

[endpointFeasible, endpointMessage, endpointReason] = ...
    azElInput.validatePlannerEndpoints( ...
    obstacles, initialState, goalState, limits, options);
if ~endpointFeasible
    result.Message = endpointMessage;
    result.TerminationReason = endpointReason;
    result.SearchDiagnostics.TerminationReason = endpointReason;
    result = azElPlanner.stageTiming( ...
        result, planningTimer, stageTiming);
    return;
end

%% Section 2: Generate Input-Driven Topology Proposals

topologyTimer = tic;
[seeds, gridDiagnostics] = azElSearch.generateTopologySeeds( ...
    obstacles, initialState, goalState, limits, options);
gridDiagnostics.ElapsedTime_s = toc(topologyTimer);
stageTiming.TopologyElapsedTime_s = gridDiagnostics.ElapsedTime_s;
result.Seeds = seeds;
result.SearchDiagnostics.Grid = gridDiagnostics;
result.SearchDiagnostics.SeedGenerationElapsedTime_s = gridDiagnostics.ElapsedTime_s;
seedSummaries = repmat(summaryTemplate, numel(seeds), 1);
candidates = cell(numel(seeds), 1);

% Prefer a direct route only when the conservative swept-geometry graph
% certified its endpoint edge. Otherwise try input-derived detours first and
% retain direct proposals for dynamic cases where timing may make them viable.
routePointCount = reshape(arrayfun(@(seed) size(seed.position_deg, 1), seeds), [], 1);
isTimedTopology = any(string({seeds.Source}).' == ["directWait", "timeExpandedVisibilityGraph"], 2);
timedIndices = find(isTimedTopology);
detourIndices = find(routePointCount > 2 & ~isTimedTopology);
directIndices = find(routePointCount <= 2 & ~isTimedTopology);
if numel(detourIndices) > 1
    % Earliest-arrival work starts with the shortest geometric proposal.
    % Per-seed work budgets already account for route complexity.
    topologyEffort = [seeds(detourIndices).Length_deg];
    [~, effortOrder] = sort(topologyEffort, "ascend");
    detourIndices = detourIndices(effortOrder);
end
if conservativeDirectEdgeAccepted(gridDiagnostics)
    spatialOrder = [directIndices(:); detourIndices(:)];
else
    spatialOrder = [detourIndices(:); directIndices(:)];
end
hasChangingObstacles = obstacleHistoryChanges(obstacles, initialState.time_s, goalState.time_s);
exactSpatialGraphUsed = gridDiagnostics.Coverage.ExactSpatialProposalUsed && ...
    gridDiagnostics.ExhaustiveVisibilityUsed;
staticNoPathCertified = ~hasChangingObstacles && exactSpatialGraphUsed && ...
    ~gridDiagnostics.HomologySearchTruncated && isempty(detourIndices) && ...
    ~conservativeDirectEdgeAccepted(gridDiagnostics);
gridDiagnostics.StaticNoPathCertified = staticNoPathCertified;
result.SearchDiagnostics.Grid = gridDiagnostics;
if hasChangingObstacles
    % Temporal search proposals already carry a causal obstacle-time law.
    % Exercise that information before free-time spatial local minima.
    seedOrder = [timedIndices(:); spatialOrder(:)];
else
    seedOrder = spatialOrder;
end
if hasChangingObstacles && options.GoalTimeMode == "fixedArrival" && ...
        ~isempty(seedOrder)
    % Fixed arrival ranks valid motions by spatial length. Trying the
    % shortest input-derived proposals first can prove the geometric lower
    % bound before spending work on longer topologies.
    orderedLength_deg = reshape( ...
        [seeds(seedOrder).Length_deg], [], 1);
    [~, lengthOrder] = sort(orderedLength_deg, "ascend");
    seedOrder = seedOrder(lengthOrder);
end
if staticNoPathCertified
    % A topology-preserving motion solve cannot repair an exact static graph
    % that exhausted every route and rejected its only direct edge.
    seedOrder = zeros(0, 1);
end
firstValidatedMotionTime_s = NaN;
bestValidatedFinalTime_s = Inf;
straightLineLowerBound_deg = norm( ...
    goalState.position_deg - initialState.position_deg);
lengthLowerBoundTolerance_deg = max( ...
    1e-10, 1e-10 * max(1, straightLineLowerBound_deg));

%% Section 3: Solve, Validate, And Boundedly Repair HS3 Motions

for orderIndex = 1:numel(seedOrder)
    if remainingPlanningTime(options, planningTimer) <= 0.1
        break;
    end
    seedIndex = seedOrder(orderIndex);
    originalSeed = seeds(seedIndex);
    axisTravel_deg = sum(abs(diff( ...
        originalSeed.position_deg, 1, 1)), 1);
    velocityDurationBound_s = max([1e-3, ...
        axisTravel_deg ./ limits.maxVelocity_deg_s]);
    spatialArrivalLowerBound_s = initialState.time_s + ...
        velocityDurationBound_s;
    isSpatialBoundDominated = ~isTimedTopology(seedIndex) && ...
        spatialArrivalLowerBound_s >= bestValidatedFinalTime_s - ...
        options.ArrivalTimeTolerance_s;
    if isSpatialBoundDominated
        seedSummaries(seedIndex).SeedIndex = seedIndex;
        seedSummaries(seedIndex).SeedSource = originalSeed.Source;
        seedSummaries(seedIndex).TerminationReason = ...
            "arrivalBoundDominated";
        seedSummaries(seedIndex).Message = ...
            "A validated topology already meets this route's arrival bound.";
        continue;
    end
    originalSeed.CollinearityDirection = directCollinearityDirection( ...
        originalSeed, obstacles, gridDiagnostics, initialState, ...
        goalState, limits);
    trialSeed = originalSeed;
    solverGoalState = goalState;
    if options.GoalTimeMode == "earliestArrival" && ...
            ~isTimedTopology(seedIndex) && isfinite(bestValidatedFinalTime_s)
        % A later solution cannot win candidate ranking. Tightening only the
        % free-time horizon preserves every potentially earlier motion.
        solverGoalState.time_s = min( ...
            solverGoalState.time_s, bestValidatedFinalTime_s);
    end
    seedGoalTimeMode = options.GoalTimeMode;
    if options.GoalTimeMode == "earliestArrival" && (isempty(obstacles) || any( ...
            originalSeed.Source == ["directWait", ...
            "timeExpandedVisibilityGraph"]))
        % Timed topologies own an arrival proposal, while obstacle-free
        % motion has a convex fixed-time representation. Both avoid the
        % corresponding free-time local minimum through bounded bisection.
        solverGoalState.time_s = goalState.time_s;
        if ~isempty(obstacles)
            solverGoalState.time_s = min(goalState.time_s, ...
                initialState.time_s + originalSeed.EstimatedDuration_s);
        end
        seedGoalTimeMode = "fixedArrival";
    end
    segmentCount = min(options.MaximumCollocationSegmentCount, max( ...
        (1 + (isempty(obstacles) && options.GoalTimeMode == "earliestArrival")) * ...
        options.CollocationSegmentCount, max(2, size(originalSeed.position_deg, 1) - 1)));
    % Skip the midpoint solve when base segments span acceleration cycles.
    meshGrowthFactor = 2 + 2 * (size(originalSeed.position_deg, 1) > 3 && ...
        originalSeed.EstimatedDuration_s > 2 * options.CollocationSegmentCount * ...
        max(limits.maxVelocity_deg_s ./ limits.maxAcceleration_deg_s2) && ...
        ~isTimedTopology(seedIndex));
    totalRelinearizationCount = 0;
    meshRelinearizationCount = 0;
    meshRefinementCount = 0;
    validRelinearizationCount = 0;
    spatialTimingTarget_s = NaN;
    timedArrivalSearchActive = false;
    timedInfeasibleTime_s = NaN;
    timedFeasibleTime_s = NaN;
    timedArrivalTrialCount = 0;
    maximumTimedArrivalTrialCount = 14;
    validatedCandidate = [];
    finalCandidate = [];
    previousSolve = struct("FinalTime_s", NaN, "MinimumClearance_deg", NaN);
    % Reserve a bounded share for every retained candidate. Counting only
    % multi-point detours lets one expensive solve starve a later timed or
    % direct proposal whose distinct motion law may be the feasible answer.
    seedWorkBudget_s = remainingPlanningTime(options, planningTimer) / ...
        (numel(seedOrder) - orderIndex + 1);
    seedTimer = tic;
    while remainingPlanningTime(options, planningTimer) > 0.1 && ...
            seedWorkBudget_s - toc(seedTimer) > 0.1
        remaining_s = min(remainingPlanningTime(options, planningTimer), ...
            seedWorkBudget_s - toc(seedTimer));
        solverOptions = options;
        solverOptions.GoalTimeMode = seedGoalTimeMode;
        solverOptions.CollocationSegmentCount = segmentCount;
        solverOptions.MaximumSolverTime_s = max(0.05, 0.70 * remaining_s);
        if isfinite(bestValidatedFinalTime_s) && ...
                ~options.DeterministicWorkBudget && ...
                solverOptions.GoalTimeMode == "earliestArrival"
            % Once feasibility exists, a later candidate is optional
            % improvement work. Bound its nonlinear long tail while still
            % giving measured fast improvements a full opportunity.
            solverOptions.MaximumSolverTime_s = min( ...
                solverOptions.MaximumSolverTime_s, 3);
        end
        finalCandidate = ...
            azElPlanner.solveSeed( ...
            obstacles, initialState, solverGoalState, limits, ...
            solverOptions, trialSeed);
        finalCandidate.MotionSource = "hs3";
        finalCandidate.Validation = validateCandidate( ...
            finalCandidate, obstacles, initialState, goalState, ...
            limits, options);
        stageTiming = addCandidateTiming(stageTiming, finalCandidate);
        if finalCandidate.Validation.Passed
            if isnan(firstValidatedMotionTime_s)
                firstValidatedMotionTime_s = toc(planningTimer);
            end
            if isempty(validatedCandidate) || candidateImproves( ...
                    finalCandidate, validatedCandidate, ...
                    options.GoalTimeMode)
                validatedCandidate = finalCandidate;
            end
            refinementTime_s = min( ...
                remainingPlanningTime(options, planningTimer), ...
                seedWorkBudget_s - toc(seedTimer));
            relativeLengthExcess = coarseMotionLengthExcess(finalCandidate, originalSeed);
            % A 25% derivative reserve distinguishes velocity-dominated
            % coarse timing from motions already pressing higher derivatives.
            hasDerivativeSlack = all([ ...
                finalCandidate.Validation.PeakAcceleration_deg_s2 ./ ...
                limits.maxAcceleration_deg_s2, ...
                finalCandidate.Validation.PeakJerk_deg_s3 ./ ...
                limits.maxJerk_deg_s3] < 0.75);
            useFixedQualitySearch = meshRefinementCount < 1 && ~hasChangingObstacles && ...
                relativeLengthExcess > 2.5 / segmentCount;
            if seedGoalTimeMode == "fixedArrival" && ...
                    options.GoalTimeMode == "earliestArrival"
                if ~timedArrivalSearchActive
                    timedInfeasibleTime_s = initialState.time_s + velocityDurationBound_s;
                    timedArrivalSearchActive = true;
                end
                timedFeasibleTime_s = min( ...
                    timedFeasibleTime_s, finalCandidate.FinalTime_s, ...
                    "omitmissing");
                proposedSpatialTimingTarget_s = initialState.time_s + ...
                    2 * velocityDurationBound_s;
                hasSpatialTimingTrial = ~isfinite(spatialTimingTarget_s) && ...
                    originalSeed.Source == "timeExpandedVisibilityGraph" && ...
                    proposedSpatialTimingTarget_s < ...
                    validatedCandidate.FinalTime_s - ...
                    options.ArrivalTimeTolerance_s && ...
                    refinementTime_s > 0.1;
                if hasSpatialTimingTrial
                    spatialTimingTarget_s = proposedSpatialTimingTarget_s;
                    totalRelinearizationCount = totalRelinearizationCount + 1;
                    solverGoalState.time_s = spatialTimingTarget_s;
                    trialSeed = spatialTimingSeed(originalSeed);
                    continue;
                end
                hasTimedTrial = timedArrivalTrialCount < ...
                    maximumTimedArrivalTrialCount && refinementTime_s > 0.1 && ...
                    timedFeasibleTime_s - timedInfeasibleTime_s > ...
                    options.ArrivalTimeTolerance_s;
                if hasTimedTrial
                    timedArrivalTrialCount = timedArrivalTrialCount + 1;
                    totalRelinearizationCount = totalRelinearizationCount + 1;
                    solverGoalState.time_s = 0.5 * ...
                        (timedInfeasibleTime_s + timedFeasibleTime_s);
                    trialSeed = seedFromCandidate( ...
                        finalCandidate, originalSeed, ...
                        solverGoalState.time_s);
                    continue;
                end
            elseif options.GoalTimeMode == "earliestArrival" && ...
                    ~isTimedTopology(seedIndex) && ...
                    meshRefinementCount < options.MaximumMeshRefinementPasses && ...
                    ((validRelinearizationCount < 1 && ...
                    relativeLengthExcess > 1 / segmentCount) || ...
                    (validRelinearizationCount == 1 && hasDerivativeSlack) || ...
                    useFixedQualitySearch) && ...
                    refinementTime_s > 0.1
                nextSegmentCount = min(options.MaximumCollocationSegmentCount, ...
                    meshGrowthFactor * segmentCount + useFixedQualitySearch * ...
                    options.MaximumCollocationSegmentCount);
                if nextSegmentCount > segmentCount
                    meshRefinementCount = meshRefinementCount + 1;
                    totalRelinearizationCount = totalRelinearizationCount + 1;
                    segmentCount = nextSegmentCount;
                    trialSeed = originalSeed;
                    if validRelinearizationCount == 1
                        trialSeed = seedFromCandidate(finalCandidate, originalSeed);
                    end
                    if useFixedQualitySearch
                        seedGoalTimeMode = "fixedArrival";
                        solverGoalState.time_s = finalCandidate.FinalTime_s;
                    end
                    validRelinearizationCount = 2;
                    continue;
                end
            elseif validRelinearizationCount < 1 && refinementTime_s > 0.1
                validRelinearizationCount = validRelinearizationCount + 1;
                totalRelinearizationCount = totalRelinearizationCount + 1;
                trialSeed = seedFromCandidate(finalCandidate, originalSeed);
                continue;
            end
            finalCandidate = validatedCandidate;
            break;
        end

        if timedArrivalSearchActive && ~isempty(validatedCandidate)
            isSpatialTimingFailure = isfinite(spatialTimingTarget_s) && ...
                matchesWithin(solverGoalState.time_s, ...
                spatialTimingTarget_s, options.ArrivalTimeTolerance_s);
            if ~isSpatialTimingFailure
                timedInfeasibleTime_s = max( ...
                    timedInfeasibleTime_s, solverGoalState.time_s);
            end
            remaining_s = min(remainingPlanningTime(options, planningTimer), ...
                seedWorkBudget_s - toc(seedTimer));
            hasTimedTrial = timedArrivalTrialCount < ...
                maximumTimedArrivalTrialCount && remaining_s > 0.1 && ...
                timedFeasibleTime_s - timedInfeasibleTime_s > ...
                options.ArrivalTimeTolerance_s;
            if hasTimedTrial
                timedArrivalTrialCount = timedArrivalTrialCount + 1;
                totalRelinearizationCount = totalRelinearizationCount + 1;
                solverGoalState.time_s = 0.5 * ...
                    (timedInfeasibleTime_s + timedFeasibleTime_s);
                trialSeed = seedFromCandidate( ...
                    validatedCandidate, originalSeed, ...
                    solverGoalState.time_s);
                continue;
            end
        end
        if ~isempty(validatedCandidate)
            finalCandidate = validatedCandidate;
            break;
        end

        % Relinearizing around a motion that reproduced its predecessor only
        % re-derives the same rejected answer, so refine the mesh instead.
        repeatsPreviousSolve = reproducesSolve( ...
            finalCandidate, previousSolve, options);
        previousSolve.FinalTime_s = finalCandidate.FinalTime_s;
        previousSolve.MinimumClearance_deg = ...
            finalCandidate.Validation.MinimumClearance_deg;
        collisionOnly = ~isempty(finalCandidate.time_s) && ...
            ~finalCandidate.Validation.CollisionFree && ...
            finalCandidate.OptimizerFeasible;
        if collisionOnly && ~repeatsPreviousSolve && meshRelinearizationCount < 2
            meshRelinearizationCount = meshRelinearizationCount + 1;
            totalRelinearizationCount = totalRelinearizationCount + 1;
            trialSeed = seedFromCandidate(finalCandidate, originalSeed);
            continue;
        end
        nextSegmentCount = min(options.MaximumCollocationSegmentCount, ...
            max(segmentCount + 1, 2 * segmentCount));
        if meshRefinementCount >= options.MaximumMeshRefinementPasses || ...
                nextSegmentCount <= segmentCount
            break;
        end
        meshRefinementCount = meshRefinementCount + 1;
        segmentCount = nextSegmentCount;
        meshRelinearizationCount = 0;
        if ~isempty(finalCandidate.time_s)
            trialSeed = seedFromCandidate(finalCandidate, originalSeed);
        end
    end
    if isempty(finalCandidate)
        continue;
    end
    candidates{seedIndex} = finalCandidate;
    seedSummaries(seedIndex) = candidateSummary( ...
        finalCandidate, totalRelinearizationCount, meshRefinementCount, ...
        summaryTemplate);
    if finalCandidate.Validation.Passed
        bestValidatedFinalTime_s = min( ...
            bestValidatedFinalTime_s, finalCandidate.FinalTime_s);
    end
    fixedLengthLowerBoundReached = ...
        options.GoalTimeMode == "fixedArrival" && ...
        finalCandidate.Validation.Passed && ...
        finalCandidate.MotionLength_deg <= ...
        straightLineLowerBound_deg + lengthLowerBoundTolerance_deg;
    if fixedLengthLowerBoundReached
        % Euclidean displacement is a global spatial lower bound. Once an
        % independently valid motion attains it, no later seed can be shorter.
        break;
    end
    % A motion pinned to the goal horizon is a fallback, not an earliest
    % arrival, so keep proposing topologies while budget remains.
    if finalCandidate.Validation.Passed && ~hasChangingObstacles && ...
            ~finalCandidate.ArrivalAtHorizon && ...
            options.GoalTimeMode == "earliestArrival"
        break;
    end
end

%% Section 4: Select Or Return Diagnosable Failure

result.SeedSummaries = seedSummaries;
result.SearchDiagnostics.SeedSummaries = seedSummaries;
result.SearchDiagnostics.AttemptedSeedCount = nnz([seedSummaries.Hs3Attempted]);
result.SearchDiagnostics.FirstValidatedMotionTime_s = firstValidatedMotionTime_s;
result.FirstValidatedMotionTime_s = firstValidatedMotionTime_s;
validatedIndices = find([seedSummaries.ValidationPassed]).';
result.SearchDiagnostics.ValidatedCandidateCount = numel(validatedIndices);
result.SearchDiagnostics.Hs3ElapsedTime_s = toc(planningTimer) - stageTiming.TopologyElapsedTime_s;
if isempty(validatedIndices)
    if gridDiagnostics.StaticNoPathCertified
        result.Message = "The exact static visibility graph exhausted all " + ...
            "routes, so no topology-preserving HS3 solve was attempted.";
    else
        result.Message = ...
            "No attempted HS3 motion passed independent validation.";
    end
    result.TerminationReason = "noValidatedSeed";
    result.SearchDiagnostics.TerminationReason = result.TerminationReason;
    result.SearchDiagnostics.BestPartialSeedIndex = ...
        bestPartialSeed(seedSummaries);
    result.SearchDiagnostics.StageTiming = stageTiming;
    result = azElPlanner.stageTiming( ...
        result, planningTimer, stageTiming);
    return;
end

selectedSeedIndex = selectValidatedCandidate( ...
    seedSummaries, validatedIndices, options.GoalTimeMode, ...
    options.ArrivalTimeTolerance_s);
selectedCandidate = candidates{selectedSeedIndex};
result.Success = true;
result.Message = "An independently validated Hermite-Simpson motion was selected.";
result.TerminationReason = "goalReached";
result.SelectedSeedIndex = selectedSeedIndex;
result.SelectedMotionSource = "hs3";
result.SelectedSeed_deg = seeds(selectedSeedIndex).position_deg;
for fieldName = ["time_s", "position_deg", "velocity_deg_s", ...
        "acceleration_deg_s2", "jerk_deg_s3", "Polynomial", ...
        "SeedCorridorBoundary_deg", "SeedCorridor", "Validation"]
    result.(fieldName) = selectedCandidate.(fieldName);
end
result.ArrivalTime_s = selectedCandidate.FinalTime_s;
result.TrajectoryDuration_s = selectedCandidate.MotionDuration_s;
result.SearchDiagnostics.TerminationReason = result.TerminationReason;
result.SearchDiagnostics.BestPartialSeedIndex = selectedSeedIndex;
result.SearchDiagnostics.MeshRefinementPassCount = ...
    seedSummaries(selectedSeedIndex).MeshRefinementPassCount;
if options.GoalTimeMode == "fixedArrival"
    result.OptimalityStatement = "Shortest independently validated sampled " + ...
        "motion among the fixed-arrival candidates completed within the " + ...
        "global work budget; no global certificate.";
elseif selectedCandidate.ArrivalAtHorizon
    result.Message = "An independently validated Hermite-Simpson motion " + ...
        "was selected, but its arrival is pinned to the goal horizon.";
    result.OptimalityStatement = "Arrival equals the goal horizon, so the " + ...
        "free-time solve never descended; this motion is feasible but " + ...
        "carries no earliest-arrival claim.";
else
    result.OptimalityStatement = "Earliest independently validated HS3 " + ...
        "motion among the candidates completed within the global work " + ...
        "budget; no global certificate.";
end
result.SearchDiagnostics.StageTiming = stageTiming;
result = azElPlanner.stageTiming( ...
    result, planningTimer, stageTiming);
end

function remaining_s = remainingPlanningTime(options, planningTimer)
% Apply one cooperative wall-time budget across topology, solve, and validation.
% Releasing that budget leaves only machine-independent caps -- seed count,
% relinearizations, mesh passes, and NLP iterations -- so the same request
% returns the same motion on a slower or busier machine.
if options.DeterministicWorkBudget
    remaining_s = Inf;
    return;
end
remaining_s = options.MaximumPlanningTime_s - toc(planningTimer);
end

function accepted = conservativeDirectEdgeAccepted(gridDiagnostics)
% Read the search-owned swept-envelope certificate without querying a method.
accepted = false;
edges_deg = gridDiagnostics.AcceptedEdges_deg;
if isempty(edges_deg)
    return;
end
start_deg = gridDiagnostics.Start_deg;
goal_deg = gridDiagnostics.Goal_deg;
tolerance_deg = 1e-9;
forward = max(abs(edges_deg(:, 1:2) - start_deg), [], 2) <= tolerance_deg & ...
    max(abs(edges_deg(:, 3:4) - goal_deg), [], 2) <= tolerance_deg;
reverse = max(abs(edges_deg(:, 1:2) - goal_deg), [], 2) <= tolerance_deg & ...
    max(abs(edges_deg(:, 3:4) - start_deg), [], 2) <= tolerance_deg;
accepted = any(forward | reverse);
end

function changing = obstacleHistoryChanges(obstacles, startTime_s, endTime_s)
% Distinguish static scenes from geometry where timing can change topology.
changing = false;
for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacles(obstacleIndex);
    if numel(obstacle.time_s) > 1 && ...
            (obstacle.time_s(1) > startTime_s || ...
            obstacle.time_s(end) < endTime_s)
        changing = true;
        return;
    end
    for sampleIndex = 2:numel(obstacle.time_s)
        if ~isequaln(obstacle.az_deg{sampleIndex}, obstacle.az_deg{1}) || ...
                ~isequaln(obstacle.el_deg{sampleIndex}, obstacle.el_deg{1})
            changing = true;
            return;
        end
    end
end
end

function validation = validateCandidate( ...
        candidate, obstacles, initialState, goalState, limits, options)
% Canonical validation is authoritative even when the optimizer is feasible.
if isempty(candidate.time_s)
    validation = validateAzElTrajectory();
    validation.Message = "The HS3 solver returned no trajectory.";
else
    validation = validateAzElTrajectory( ...
        candidate, obstacles, initialState, goalState, limits, options);
end
end

function stageTiming = addCandidateTiming(stageTiming, candidate)
% Add exclusive solver/corridor and independent-validation timings.
diagnostics = candidate.SolverDiagnostics;
stageTiming.CorridorConstructionElapsedTime_s = ...
    stageTiming.CorridorConstructionElapsedTime_s + ...
    diagnostics.CorridorConstructionElapsedTime_s;
stageTiming.MotionSolvingElapsedTime_s = ...
    stageTiming.MotionSolvingElapsedTime_s + ...
    diagnostics.MotionSolvingElapsedTime_s;
validation = candidate.Validation;
stageTiming.CollisionCheckingElapsedTime_s = ...
    stageTiming.CollisionCheckingElapsedTime_s + ...
    validation.CollisionCheckingElapsedTime_s;
stageTiming.FinalValidationElapsedTime_s = ...
    stageTiming.FinalValidationElapsedTime_s + max(0, ...
    validation.ElapsedTime_s - validation.CollisionCheckingElapsedTime_s);
end

function repeats = reproducesSolve(candidate, previousSolve, options)
% Compare arrival and clearance so a stalled relinearization is not repaid.
repeats = ~isempty(candidate.time_s) && ...
    matchesWithin(candidate.FinalTime_s, previousSolve.FinalTime_s, ...
    options.ArrivalTimeTolerance_s) && ...
    matchesWithin(candidate.Validation.MinimumClearance_deg, ...
    previousSolve.MinimumClearance_deg, ...
    max(options.CollisionClearanceTolerance_deg, 1e-9));
end

function relativeLengthExcess = coarseMotionLengthExcess(candidate, seed)
% Measure route inflation so the caller can scale one bounded quality pass.
if isempty(candidate.position_deg) || ~isfinite(seed.Length_deg) || ...
        seed.Length_deg <= 0
    relativeLengthExcess = 0;
    return;
end
motionLength_deg = sum(vecnorm(diff( ...
    candidate.position_deg, 1, 1), 2, 2));
relativeLengthExcess = motionLength_deg / seed.Length_deg - 1;
end

function seed = spatialTimingSeed(seed)
% Remove waits and distribute a timed topology by geometric arc length.
edgeLength_deg = vecnorm(diff(seed.position_deg, 1, 1), 2, 2);
retainedPoint = [true; edgeLength_deg > 1e-12];
seed.position_deg = seed.position_deg(retainedPoint, :);
cumulativeLength_deg = [0; cumsum(vecnorm( ...
    diff(seed.position_deg, 1, 1), 2, 2))];
seed.tau = cumulativeLength_deg / cumulativeLength_deg(end);
end

function matches = matchesWithin(current, previous, tolerance)
% Treat equal nonfinite evidence as unchanged without comparing NaN to NaN.
matches = (isinf(current) && isequal(current, previous)) || abs(current - previous) <= tolerance;
end

function seed = seedFromCandidate( ...
        candidate, originalSeed, targetFinalTime_s)
% Reassociate frozen corridors around HS3 motion without changing topology.
seed = originalSeed;
isTimedSeed = any(originalSeed.Source == ["directWait", "timeExpandedVisibilityGraph"]);
if nargin >= 3 && targetFinalTime_s < candidate.FinalTime_s && ~isTimedSeed
    return;
end
if nargin < 3 || targetFinalTime_s >= candidate.FinalTime_s
    sampleTau = (candidate.time_s - candidate.time_s(1)) / ...
        candidate.MotionDuration_s;
    [sampleTau, retainedIndex] = unique(sampleTau, "stable");
    seed.tau = candidate.ControlTau;
    seed.position_deg = interp1(sampleTau, ...
        candidate.position_deg(retainedIndex, :), seed.tau, "linear");
    seed.EstimatedDuration_s = candidate.MotionDuration_s;
    return;
end

% Shortening a timed motion must preserve absolute event timing. Rescaling
% its normalized wait would move an already validated crossing earlier.
controlTime_s = candidate.time_s(1) + ...
    candidate.ControlTau * candidate.MotionDuration_s;
retainedControl = controlTime_s < targetFinalTime_s;
retainedTime_s = controlTime_s(retainedControl);
retainedPosition_deg = interp1(candidate.time_s, ...
    candidate.position_deg, retainedTime_s, "linear");
trialDuration_s = targetFinalTime_s - candidate.time_s(1);
seed.tau = [(retainedTime_s - candidate.time_s(1)) / ...
    trialDuration_s; 1];
seed.position_deg = [retainedPosition_deg; originalSeed.position_deg(end, :)];
seed.EstimatedDuration_s = trialDuration_s;
end

function summary = candidateSummary(candidate, relinearizationCount, ...
        meshRefinementCount, template)
% Retain full failure evidence without duplicating trajectory arrays.
summary = template;
summary.SeedIndex = candidate.SeedIndex;
summary.SeedSource = candidate.SeedSource;
summary.OptimizerFeasible = candidate.OptimizerFeasible;
summary.ValidationPassed = candidate.Validation.Passed;
summary.CollisionFree = candidate.Validation.CollisionFree;
summary.CollisionResolved = candidate.Validation.CollisionResolved;
summary.MinimumClearance_deg = candidate.Validation.MinimumClearance_deg;
summary.UnresolvedIntervalCount = candidate.Validation.UnresolvedIntervalCount;
summary.ArrivalTime_s = candidate.FinalTime_s;
summary.MotionDuration_s = candidate.MotionDuration_s;
summary.MotionLength_deg = candidate.MotionLength_deg;
summary.IntegratedSquaredJerk_deg2_s5 = ...
    candidate.IntegratedSquaredJerk_deg2_s5;
summary.MaximumConstraintViolation = candidate.MaximumConstraintViolation;
summary.RelinearizationCount = relinearizationCount;
summary.MeshRefinementPassCount = meshRefinementCount;
summary.Hs3Attempted = true;
summary.Hs3OptimizerFeasible = candidate.OptimizerFeasible;
summary.Hs3ValidationPassed = candidate.Validation.Passed;
summary.Hs3TerminationReason = candidate.TerminationReason;
summary.Hs3SolverDiagnostics = candidate.SolverDiagnostics;
summary.TerminationReason = candidate.TerminationReason;
summary.Message = candidate.Message + " " + candidate.Validation.Message;
summary.SolverDiagnostics = candidate.SolverDiagnostics;
if candidate.Validation.Passed
    summary.SelectedMotionSource = "hs3";
end
end

function index = bestPartialSeed(summaries)
% Prefer resolved collision evidence, then NLP residual and clearance.
violation = [summaries.MaximumConstraintViolation].';
violation(~isfinite(violation)) = Inf;
collisionRank = 2 * ~[summaries.CollisionResolved].' + ...
    ~[summaries.CollisionFree].';
clearance = [summaries.MinimumClearance_deg].';
clearance(~isfinite(clearance)) = -Inf;
[~, order] = sortrows([collisionRank, violation, -clearance], [1 2 3]);
if isempty(order) || isinf(violation(order(1)))
    index = 0;
else
    index = order(1);
end
end

function selectedIndex = selectValidatedCandidate( ...
        summaries, validatedIndices, goalTimeMode, arrivalTolerance_s)
% Rank fixed arrivals by motion length and free arrivals by time.
if goalTimeMode == "fixedArrival"
    motionLength_deg = ...
        [summaries(validatedIndices).MotionLength_deg].';
    lengthTolerance_deg = max(1e-10, ...
        1e-10 * max(1, min(motionLength_deg)));
    lengthEquivalent = validatedIndices( ...
        motionLength_deg <= min(motionLength_deg) + lengthTolerance_deg);
    jerk = ...
        [summaries(lengthEquivalent).IntegratedSquaredJerk_deg2_s5].';
    selectedIndex = min( ...
        lengthEquivalent(jerk <= min(jerk) + 1e-12));
    return;
end
arrival = [summaries(validatedIndices).ArrivalTime_s].';
timeEquivalent = validatedIndices( ...
    arrival <= min(arrival) + arrivalTolerance_s);
jerk = [summaries(timeEquivalent).IntegratedSquaredJerk_deg2_s5].';
selectedIndex = min(timeEquivalent(jerk <= min(jerk) + 1e-12));
end

function improves = candidateImproves( ...
        candidate, incumbent, goalTimeMode)
%% Section 0: Header & Readme
% SYNTAX
%   improves = candidateImproves( ...
%       candidate, incumbent, goalTimeMode)
%**************************************************************************
% PURPOSE
%   - Compare independently valid candidates using the public time policy.
%**************************************************************************
% INPUTS
%   - candidate, incumbent (scalar candidate structs)
%       Both records contain final time, sampled length, and jerk cost.
%   - goalTimeMode (scalar string)
%       fixedArrival prioritizes length; earliestArrival prioritizes time.
%**************************************************************************
% OUTPUTS
%   - improves (logical scalar)
%       True only when candidate strictly improves the active quality order.
%**************************************************************************
% UNITS
%   - Length is degrees and arrival tolerance is seconds.
%**************************************************************************
if goalTimeMode == "fixedArrival"
    lengthTolerance_deg = max(1e-10, ...
        1e-10 * max(1, incumbent.MotionLength_deg));
    lengthImproves = candidate.MotionLength_deg < ...
        incumbent.MotionLength_deg - lengthTolerance_deg;
    lengthEquivalent = abs(candidate.MotionLength_deg - ...
        incumbent.MotionLength_deg) <= lengthTolerance_deg;
    jerkImproves = candidate.IntegratedSquaredJerk_deg2_s5 < ...
        incumbent.IntegratedSquaredJerk_deg2_s5 - 1e-12;
    improves = lengthImproves || (lengthEquivalent && jerkImproves);
    return;
end
% Preserve the established earliest-arrival refinement rule exactly.
improves = candidate.FinalTime_s < incumbent.FinalTime_s;
end

function direction = directCollinearityDirection( ...
        seed, obstacles, gridDiagnostics, initialState, goalState, limits)
%% Section 0: Header & Readme
% SYNTAX
%   direction = directCollinearityDirection( ...
%       seed, obstacles, gridDiagnostics, initialState, goalState, limits)
%**************************************************************************
% PURPOSE
%   - Identify direct motions whose shortest spatial path can be imposed
%     without tightening their axis-limited earliest-arrival bound.
%**************************************************************************
% INPUTS
%   - seed (scalar topology-seed struct)
%       position_deg is the proposed N-by-2 [azimuth elevation] route.
%   - obstacles (canonical protected obstacle struct array)
%       Empty scenes certify direct geometry without a visibility graph.
%   - gridDiagnostics (scalar struct)
%       Contains accepted spatial edges and timed-search provenance.
%   - initialState, goalState (normalized scalar state structs)
%       Endpoint velocity and acceleration must be parallel to the route;
%       sampled moving goals are excluded.
%   - limits (normalized scalar limits struct)
%       Per-axis velocity, acceleration, and jerk limits determine whether
%       one axis governs every finite normalized derivative bound.
%**************************************************************************
% OUTPUTS
%   - direction (1-by-2 numeric row or explicit 0-by-2 empty)
%       Unit direct-path direction when collinearity is safe to impose.
%**************************************************************************
% UNITS
%   - Position is degrees; derivative limits use degrees and seconds.
%**************************************************************************

direction = zeros(0, 2);
hasMovingGoal = isfield(goalState, "targetTime_s") && ...
    ~isempty(goalState.targetTime_s);
if hasMovingGoal || size(seed.position_deg, 1) < 2
    return;
end
displacement_deg = goalState.position_deg - initialState.position_deg;
distance_deg = norm(displacement_deg);
if distance_deg <= 1e-12
    return;
end
candidateDirection = displacement_deg / distance_deg;
normal = [-candidateDirection(2), candidateDirection(1)];
routeOffset_deg = (seed.position_deg - initialState.position_deg) * normal.';
routeProgress_deg = (seed.position_deg - initialState.position_deg) * ...
    candidateDirection.';
lineTolerance_deg = max(1e-10, 1e-10 * distance_deg);
routeIsDirect = max(abs(routeOffset_deg)) <= lineTolerance_deg && ...
    min(routeProgress_deg) >= -lineTolerance_deg && ...
    max(routeProgress_deg) <= distance_deg + lineTolerance_deg;
sourceIsTimedDirect = any(seed.Source == [ ...
    "directWait", "timeExpandedVisibilityGraph"]);
geometryIsCertified = isempty(obstacles) || ...
    conservativeDirectEdgeAccepted(gridDiagnostics) || sourceIsTimedDirect;
if ~routeIsDirect || ~geometryIsCertified
    return;
end
derivativeTolerance = 1e-10;
endpointDerivative = [ ...
    initialState.velocity_deg_s; goalState.velocity_deg_s; ...
    initialState.acceleration_deg_s2; goalState.acceleration_deg_s2];
derivativesAreParallel = all(abs(endpointDerivative * normal.') <= ...
    derivativeTolerance * max(1, vecnorm(endpointDerivative, 2, 2)));
if ~derivativesAreParallel
    return;
end

axisTravel_deg = abs(displacement_deg);
activeAxis = axisTravel_deg > lineTolerance_deg;
commonBottleneck = activeAxis;
derivativeLimits = [ ...
    limits.maxVelocity_deg_s; limits.maxAcceleration_deg_s2; ...
    limits.maxJerk_deg_s3];
for derivativeIndex = 1:size(derivativeLimits, 1)
    normalizedLimit = inf(1, 2);
    normalizedLimit(activeAxis) = ...
        derivativeLimits(derivativeIndex, activeAxis) ./ ...
        axisTravel_deg(activeAxis);
    minimumLimit = min(normalizedLimit);
    if ~isfinite(minimumLimit)
        continue;
    end
    comparisonTolerance = max(1e-12, 1e-12 * minimumLimit);
    commonBottleneck = commonBottleneck & ...
        normalizedLimit <= minimumLimit + comparisonTolerance;
end
if any(commonBottleneck)
    direction = candidateDirection;
end
end
