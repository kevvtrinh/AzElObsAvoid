function result = plan(obstacles, initialState, goalState, limits, optionOverrides)
%% Section 0: Standalone Hermite-Simpson Planner
% Generate neutral topology proposals, solve each selected proposal with the
% HS3 transcription, and accept only canonical independently valid motion.

if nargin == 0
    result = azElPlannerMethods.hs3.resolvePlannerOptions();
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
options = azElPlannerMethods.hs3.resolvePlannerOptions(optionOverrides);
[obstacles, initialState, goalState, limits] = ...
    azElInternal.normalizePlannerRequest( ...
    obstacles, initialState, goalState, limits, options);
[result, summaryTemplate] = azElInternal.emptyPlannerResult( ...
    obstacles, initialState, goalState, limits, options, ...
    validateAzElTrajectory(), "hs3");
obstacles = azElInternal.obstacles.prepareDynamic(obstacles);
stageTiming = result.SearchDiagnostics.StageTiming;

%% Section 1: Reject Physically Invalid Endpoints

[endpointFeasible, endpointMessage, endpointReason] = ...
    azElInternal.validatePlannerEndpoints( ...
    obstacles, initialState, goalState, limits, options);
if ~endpointFeasible
    result.Message = endpointMessage;
    result.TerminationReason = endpointReason;
    result.SearchDiagnostics.TerminationReason = endpointReason;
    result = azElPlannerMethods.internal.stageTiming( ...
        result, planningTimer, stageTiming);
    return;
end

%% Section 2: Generate Input-Driven Topology Proposals

topologyTimer = tic;
[seeds, gridDiagnostics] = azElInternal.generateTopologySeeds( ...
    obstacles, initialState, goalState, limits, options);
gridDiagnostics.ElapsedTime_s = toc(topologyTimer);
stageTiming.TopologyElapsedTime_s = gridDiagnostics.ElapsedTime_s;
result.Seeds = seeds;
result.SearchDiagnostics.Grid = gridDiagnostics;
result.SearchDiagnostics.SeedGenerationElapsedTime_s = ...
    gridDiagnostics.ElapsedTime_s;
seedSummaries = repmat(summaryTemplate, numel(seeds), 1);
candidates = cell(numel(seeds), 1);

% Prefer a direct route only when the conservative swept-geometry graph
% certified its endpoint edge. Otherwise try input-derived detours first and
% retain direct proposals for dynamic cases where timing may make them viable.
routePointCount = arrayfun(@(seed) size(seed.position_deg, 1), seeds).';
detourIndices = find(routePointCount > 2);
directIndices = find(routePointCount <= 2);
if conservativeDirectEdgeAccepted(gridDiagnostics)
    seedOrder = [directIndices(:); detourIndices(:)];
else
    seedOrder = [detourIndices(:); directIndices(:)];
end
firstValidatedMotionTime_s = NaN;
stopAtFirstValidatedMotion = ~obstacleHistoryChanges( ...
    obstacles, initialState.time_s, goalState.time_s);

%% Section 3: Solve, Validate, And Boundedly Repair HS3 Motions

for orderIndex = 1:numel(seedOrder)
    if remainingPlanningTime(options, planningTimer) <= 0.1
        break;
    end
    seedIndex = seedOrder(orderIndex);
    originalSeed = seeds(seedIndex);
    trialSeed = originalSeed;
    solverGoalState = goalState;
    seedGoalTimeMode = options.GoalTimeMode;
    if options.GoalTimeMode == "earliestArrival" && any( ...
            originalSeed.Source == ["directWait", ...
            "timeExpandedVisibilityGraph"])
        % A timed topology already owns a physical arrival proposal. Solve
        % that HS3 transcription at its input-derived time instead of
        % discarding the wait law inside a free-time local minimum.
        solverGoalState.time_s = min(goalState.time_s, ...
            initialState.time_s + originalSeed.EstimatedDuration_s);
        seedGoalTimeMode = "fixedArrival";
    end
    segmentCount = topologyAlignedSegmentCount(originalSeed, options);
    totalRelinearizationCount = 0;
    meshRelinearizationCount = 0;
    meshRefinementCount = 0;
    finalCandidate = [];
    while remainingPlanningTime(options, planningTimer) > 0.1
        remaining_s = remainingPlanningTime(options, planningTimer);
        solverOptions = options;
        solverOptions.GoalTimeMode = seedGoalTimeMode;
        solverOptions.CollocationSegmentCount = segmentCount;
        solverOptions.MaximumSolverTime_s = max(0.05, 0.55 * remaining_s);
        finalCandidate = ...
            azElPlannerMethods.hs3.internal.motion.solveHs3( ...
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
            break;
        end

        collisionOnly = ~isempty(finalCandidate.time_s) && ...
            ~finalCandidate.Validation.CollisionFree && ...
            finalCandidate.OptimizerFeasible;
        if collisionOnly && meshRelinearizationCount < 2
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
    if finalCandidate.Validation.Passed && stopAtFirstValidatedMotion
        break;
    end
end

%% Section 4: Select Or Return Diagnosable Failure

result.SeedSummaries = seedSummaries;
result.SearchDiagnostics.SeedSummaries = seedSummaries;
result.SearchDiagnostics.AttemptedSeedCount = nnz( ...
    [seedSummaries.Hs3Attempted]);
result.SearchDiagnostics.FirstValidatedMotionTime_s = ...
    firstValidatedMotionTime_s;
result.FirstValidatedMotionTime_s = firstValidatedMotionTime_s;
validatedIndices = find([seedSummaries.ValidationPassed]).';
result.SearchDiagnostics.ValidatedCandidateCount = numel(validatedIndices);
result.SearchDiagnostics.Hs3ElapsedTime_s = toc(planningTimer) - ...
    stageTiming.TopologyElapsedTime_s;
if isempty(validatedIndices)
    result.Message = "No attempted HS3 motion passed independent validation.";
    result.TerminationReason = "noValidatedSeed";
    result.SearchDiagnostics.TerminationReason = result.TerminationReason;
    result.SearchDiagnostics.BestPartialSeedIndex = ...
        bestPartialSeed(seedSummaries);
    result.SearchDiagnostics.StageTiming = stageTiming;
    result = azElPlannerMethods.internal.stageTiming( ...
        result, planningTimer, stageTiming);
    return;
end

selectedSeedIndex = selectValidatedCandidate( ...
    seedSummaries, validatedIndices, options.ArrivalTimeTolerance_s);
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
result.OptimalityStatement = "Earliest independently validated HS3 motion " + ...
    "among the candidates completed within the global work budget; no " + ...
    "global certificate.";
result.SearchDiagnostics.StageTiming = stageTiming;
result = azElPlannerMethods.internal.stageTiming( ...
    result, planningTimer, stageTiming);
end

function remaining_s = remainingPlanningTime(options, planningTimer)
% Apply one cooperative wall-time budget across topology, solve, and validation.
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

function segmentCount = topologyAlignedSegmentCount(seed, options)
% Give every geometric route leg at least one HS3 segment when permitted.
routeSegmentCount = max(2, size(seed.position_deg, 1) - 1);
segmentCount = min(options.MaximumCollocationSegmentCount, ...
    max(options.CollocationSegmentCount, routeSegmentCount));
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

function seed = seedFromCandidate(candidate, originalSeed)
% Reassociate frozen corridors around HS3 motion without changing topology.
seed = originalSeed;
sampleTau = (candidate.time_s - candidate.time_s(1)) / ...
    candidate.MotionDuration_s;
[sampleTau, retainedIndex] = unique(sampleTau, "stable");
seed.tau = candidate.ControlTau;
seed.position_deg = interp1(sampleTau, ...
    candidate.position_deg(retainedIndex, :), seed.tau, "linear");
seed.EstimatedDuration_s = candidate.MotionDuration_s;
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
        summaries, validatedIndices, arrivalTolerance_s)
% Rank by arrival, then integrated jerk, then stable seed index.
arrival = [summaries(validatedIndices).ArrivalTime_s].';
timeEquivalent = validatedIndices( ...
    arrival <= min(arrival) + arrivalTolerance_s);
jerk = [summaries(timeEquivalent).IntegratedSquaredJerk_deg2_s5].';
selectedIndex = min(timeEquivalent(jerk <= min(jerk) + 1e-12));
end
