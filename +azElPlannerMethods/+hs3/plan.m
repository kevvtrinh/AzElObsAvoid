function result = plan(obstacles, initialState, goalState, ...
        limits, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = planAzElMotion()
%   result = planAzElMotion( ...
%       obstacles, initialState, goalState, limits)
%   result = planAzElMotion( ...
%       obstacles, initialState, goalState, limits, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Build and independently validate finite-jerk motions from bounded seeds.
%**************************************************************************
% INPUTS
%   - obstacles (canonical protected obstacle array, nested cells, or [])
%       Construct safety margins with makeAzElObstacleData exactly once.
%       Azimuth wrapping requires an empty obstacle array.
%   - initialState (scalar struct)
%       time_s and 1-by-2 position_deg are required. Velocity and
%       acceleration default to [0 0].
%   - goalState (scalar struct)
%       time_s is the fixed time or latest arrival. position_deg is 1-by-2.
%       Optional targetTime_s and targetPosition_deg define a moving goal.
%       Moving goals do not support azimuth wrapping.
%   - limits (scalar struct)
%       Positive 1-by-2 maxVelocity_deg_s, maxAcceleration_deg_s2, and
%       maxJerk_deg_s3 fields are required. Optional azimuthInterval_deg
%       and elevationInterval_deg define the workspace and default to
%       [-180 180] and [-90 90] degrees.
%   - optionOverrides (scalar struct, optional; default struct())
%       Partial options are accepted. Empty fields retain defaults.
%       EnableHs3Improvement defaults true. False skips optional HS3 only
%       when a valid first motion exists. MaximumHs3ImprovementTime_s
%       defaults to 15 seconds.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Stable success-or-failure motion, seed, validation, and diagnostics,
%       including selected source, arrival, duration, and first-valid time.
%       Invalid contracts throw. Expected planning failure returns
%       Success=false and preserves every attempted seed summary.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s, deg/s^2,
%     and deg/s^3. Histories are N-by-2 [azimuth elevation].
%**************************************************************************
%% Section 1: Validate Inputs And Apply Defaults
defaults = plannerDefaults();
if nargin == 0
    result = defaults;
    return;
end
if nargin < 4
    error("planAzElMotion:MissingInputs", ...
        "obstacles, initialState, goalState, and limits are required.");
end
planningTimer = tic;
stageTiming = azElPlannerMethods.internal.stageTiming();
if nargin < 5 || isempty(optionOverrides)
    optionOverrides = struct();
end
options = resolvePlannerOptions(defaults, optionOverrides);
obstacles = combineAzElObstacles(obstacles);
initialState = normalizeState(initialState, "initialState");
goalState = normalizeGoalState(goalState);
limits = normalizeLimits(limits);
if goalState.time_s <= initialState.time_s
    error("planAzElMotion:InvalidTimeWindow", ...
        "goalState.time_s must be greater than initialState.time_s.");
end
hasMovingGoal = isfield(goalState, "targetTime_s") && ~isempty(goalState.targetTime_s);
if options.AllowAzimuthWrapping && ...
        (~isempty(obstacles) || hasMovingGoal)
    error("planAzElMotion:UnsupportedWrappedGeometry", ...
        "AllowAzimuthWrapping is supported only for obstacle-free " + ...
        "fixed-position goals. Disable wrapping for this request.");
end
if options.AllowAzimuthWrapping
    turnCount = round((initialState.position_deg(1) - ...
        goalState.position_deg(1)) / 360);
    goalState.position_deg(1) = goalState.position_deg(1) + 360 * turnCount;
    if isfield(goalState, "targetPosition_deg") && ~isempty(goalState.targetPosition_deg)
        goalState.targetPosition_deg(:, 1) = goalState.targetPosition_deg(:, 1) + 360 * turnCount;
    end
end
[result, summaryTemplate] = azElPlannerMethods.hs3.internal.emptyAzElPlannerResult( ...
    obstacles, initialState, goalState, limits, options);
obstacles = azElPlannerMethods.hs3.internal.obstacles.prepareDynamic(obstacles);
if options.Verbose
    fprintf("[AzEl] Planning started.\n");
    fprintf("[AzEl][setup] workspace az=[%.6g %.6g] deg, " + ...
        "el=[%.6g %.6g] deg.\n", limits.azimuthInterval_deg, ...
        limits.elevationInterval_deg);
    fprintf("[AzEl][setup] obstacles=%d, seeds<=%d, goalMode=%s.\n", ...
        numel(obstacles), options.MaximumSeedCount, options.GoalTimeMode);
end
%% Section 2: Validate Endpoint Feasibility
[endpointFeasible, endpointMessage, endpointReason] = validateEndpoints( ...
    obstacles, initialState, goalState, limits, options);
if ~endpointFeasible
    result.Message = endpointMessage;
    result.TerminationReason = endpointReason;
    result.SearchDiagnostics.TerminationReason = endpointReason;
    result = azElPlannerMethods.internal.stageTiming( ...
        result, planningTimer, stageTiming);
    if options.Verbose
        fprintf("[AzEl] Complete: success=0, reason=%s, elapsed=%.3f s. %s\n", ...
            endpointReason, result.ElapsedPlanningTime_s, endpointMessage);
    end
    return;
end
%% Section 3: Generate The Bounded Deterministic Seed Set
seedTimer = tic;
if options.Verbose
    fprintf("[AzEl][seeds] generating topology proposals.\n");
end
[seeds, gridDiagnostics] = azElPlannerMethods.hs3.internal.search.generateTopologySeeds( ...
    obstacles, initialState, goalState, limits, options);
gridDiagnostics.ElapsedTime_s = toc(seedTimer);
stageTiming.TopologyElapsedTime_s = gridDiagnostics.ElapsedTime_s;
result.Seeds = seeds;
result.SearchDiagnostics.Grid = gridDiagnostics;
if options.Verbose
    fprintf("[AzEl][seeds] nodes=%d, visibleEdges=%d, rejectedEdges=%d, " + ...
        "expanded=%d, seeds=%d, elapsed=%.3f s.\n", ...
        gridDiagnostics.NodeCount, ...
        gridDiagnostics.VisibilityEdgeCount, ...
        gridDiagnostics.RejectedTransitionCount, ...
        gridDiagnostics.ExpandedCount, numel(seeds), ...
        gridDiagnostics.ElapsedTime_s);
end
seedSummaries = repmat(summaryTemplate, numel(seeds), 1);
candidates = cell(numel(seeds), 1);
internalOptions = options;
firstValidatedMotionTime_s = NaN;
firstMotionElapsedTime_s = 0;
hs3ElapsedTime_s = 0;
%% Section 4: Construct And Validate Deterministic First Motions

% Try the deterministic analytic construction for every topology seed before
% spending optional nonlinear-solver work.
for seedIndex = 1:numel(seeds)
    firstMotionTimer = tic;
    if options.Verbose
        fprintf("[AzEl][seed %d/%d][first] source=%s, waypoints=%d, " + ...
            "length=%.6g deg.\n", seedIndex, numel(seeds), ...
            seeds(seedIndex).Source, size(seeds(seedIndex).position_deg, 1), ...
            seeds(seedIndex).Length_deg);
    end
    firstMotionOptions = internalOptions;
    firstMotionOptions.AttemptEarlyHs3 = true;
    [firstCandidate, stageTiming] = ...
        azElPlannerMethods.hs3.internal.motion.buildStopWaypointMotion( ...
        obstacles, initialState, goalState, limits, ...
        firstMotionOptions, seeds(seedIndex), stageTiming);
    candidates{seedIndex} = firstCandidate;
    seedSummaries(seedIndex) = candidateSummary( ...
        firstCandidate, 0, summaryTemplate);
    seedSummaries(seedIndex).FirstMotionAttempted = true;
    seedSummaries(seedIndex).FirstMotionValidationPassed = firstCandidate.Validation.Passed;
    seedSummaries(seedIndex).FirstMotionTerminationReason = firstCandidate.TerminationReason;
    seedSummaries(seedIndex).FirstMotionDiagnostics = firstCandidate.AnalyticDiagnostics;
    if firstCandidate.Validation.Passed
        seedSummaries(seedIndex).SelectedMotionSource = firstCandidate.MotionSource;
        if isnan(firstValidatedMotionTime_s)
            firstValidatedMotionTime_s = toc(planningTimer);
        end
    end
    firstMotionStageTime_s = toc(firstMotionTimer);
    if firstCandidate.MotionSource == "hs3"
        solverElapsedTime_s = firstCandidate.SolverDiagnostics.ElapsedTime_s;
        hs3ElapsedTime_s = hs3ElapsedTime_s + solverElapsedTime_s;
        firstMotionStageTime_s = max( ...
            0, firstMotionStageTime_s - solverElapsedTime_s);
    end
    firstMotionElapsedTime_s = firstMotionElapsedTime_s + firstMotionStageTime_s;
    if options.Verbose
        fprintf("[AzEl][seed %d/%d][first] validation=%d, " + ...
            "arrival=%.6g s, reason=%s, elapsed=%.3f s.\n", ...
            seedIndex, numel(seeds), firstCandidate.Validation.Passed, ...
            firstCandidate.FinalTime_s, firstCandidate.TerminationReason, ...
            firstMotionStageTime_s);
    end
    if firstCandidate.Validation.Passed
        break;
    end
end
%% Section 5: Apply Optional HS3 Improvement
firstMotionIndices = find([seedSummaries.ValidationPassed]).';
continueExactMultiObstacleSearch = numel(obstacles) > 1 && any( ...
    [seedSummaries.Hs3ValidationPassed] & ...
    ~[seeds.UsesReducedGeometry]);
if isempty(firstMotionIndices)
    % HS3 remains the general path for endpoint states, moving targets, and
    % timed waits that the conservative first-motion family does not cover.
    improvementOrder = (1:numel(seeds)).';
    improvementDeadline_s = Inf;
    requiredSolveCount = max(1, 3 * numel(improvementOrder));
    requiredNlpIterations = max(10, ...
        floor(options.MaximumNlpIterations / requiredSolveCount));
    requiredNlpEvaluations = max(1000, floor( ...
        options.MaximumNlpFunctionEvaluations / requiredSolveCount));
elseif options.EnableHs3Improvement && ( ...
        ~any([seedSummaries.Hs3ValidationPassed]) || ...
        continueExactMultiObstacleSearch)
    bestFirstIndex = selectValidatedCandidate( ...
        seedSummaries, firstMotionIndices, ...
        options.ArrivalTimeTolerance_s);
    allSeedIndices = (1:numel(seeds)).';
    improvementOrder = allSeedIndices( ...
        ~[seedSummaries.Hs3Attempted].');
    if any(improvementOrder == bestFirstIndex)
        improvementOrder = [bestFirstIndex; improvementOrder( ...
            improvementOrder ~= bestFirstIndex)];
    end
    remainingHs3Time_s = max(0, ...
        options.MaximumHs3ImprovementTime_s - hs3ElapsedTime_s);
    improvementDeadline_s = toc(planningTimer) + remainingHs3Time_s;
else
    improvementOrder = zeros(0, 1);
    improvementDeadline_s = toc(planningTimer);
end

% Revisit seeds in the established improvement order so validated analytic
% motions and unresolved candidates retain their original priority policy.
for orderIndex = 1:numel(improvementOrder)
    hs3AttemptTimer = tic;
    seedIndex = improvementOrder(orderIndex);
    elapsedTime_s = toc(planningTimer);
    remainingTime_s = improvementDeadline_s - elapsedTime_s;
    minimumSolverTime_s = 0.05;
    if isfinite(remainingTime_s) && remainingTime_s <= minimumSolverTime_s
        break;
    end
    if isfinite(remainingTime_s)
        remainingSeedCount = numel(improvementOrder) - orderIndex + 1;
        seedBudget_s = remainingTime_s / remainingSeedCount;
        validationReserve_s = min(2, max(0.05, 0.2 * seedBudget_s));
        solverBudget_s = seedBudget_s - validationReserve_s;
        if solverBudget_s <= minimumSolverTime_s
            break;
        end
        seedDeadline_s = elapsedTime_s + seedBudget_s;
    else
        solverBudget_s = Inf;
        seedDeadline_s = Inf;
    end
    solverOptions = internalOptions;
    solverOptions.MaximumSolverTime_s = solverBudget_s;
    if ~isfinite(improvementDeadline_s)
        solverOptions.MaximumNlpIterations = requiredNlpIterations;
        solverOptions.MaximumNlpFunctionEvaluations = requiredNlpEvaluations;
    end
    validationOptions = internalOptions;
    trialSeed = seeds(seedIndex);
    if options.Verbose
        fprintf("[AzEl][seed %d/%d][HS3] start, source=%s, " + ...
            "segments=%d.\n", seedIndex, numel(seeds), ...
            trialSeed.Source, solverOptions.CollocationSegmentCount);
    end
    finalCandidate = azElPlannerMethods.hs3.internal.motion.solveHs3( ...
        obstacles, initialState, goalState, limits, ...
        solverOptions, trialSeed);
    finalCandidate.MotionSource = "hs3";
    if isempty(finalCandidate.time_s)
        validation = azElPlannerMethods.hs3.validateTrajectory();
        validation.Message = "The solver returned no trajectory.";
    else
        validation = azElPlannerMethods.hs3.validateTrajectory( ...
            finalCandidate, obstacles, initialState, goalState, ...
            limits, validationOptions);
    end
    finalCandidate.Validation = validation;
    stageTiming = azElPlannerMethods.hs3.internal.addHs3CandidateTiming( ...
        stageTiming, finalCandidate);
    relinearizationCount = 0;

    % Rebuild a frozen collision corridor only while collision is the remaining
    % validation failure and the bounded relinearization budget remains.
    while ~validation.Passed && ...
            ~validation.CollisionFree && ~isempty(finalCandidate.time_s) && ...
            relinearizationCount < 2 && ...
            (~isfinite(seedDeadline_s) || ...
            seedDeadline_s - toc(planningTimer) > minimumSolverTime_s)
        relinearizationCount = relinearizationCount + 1;
        trialSeed = azElPlannerMethods.hs3.internal.candidateSeed( ...
            finalCandidate, seeds(seedIndex));
        remainingSeedTime_s = seedDeadline_s - toc(planningTimer);
        if isfinite(remainingSeedTime_s)
            validationReserve_s = min(1, 0.2 * remainingSeedTime_s);
            solverOptions.MaximumSolverTime_s = remainingSeedTime_s - validationReserve_s;
            if solverOptions.MaximumSolverTime_s <= minimumSolverTime_s
                break;
            end
        else
            solverOptions.MaximumSolverTime_s = Inf;
        end
        finalCandidate = azElPlannerMethods.hs3.internal.motion.solveHs3( ...
            obstacles, initialState, goalState, limits, ...
            solverOptions, trialSeed);
        finalCandidate.MotionSource = "hs3";
        if isempty(finalCandidate.time_s)
            validation = azElPlannerMethods.hs3.validateTrajectory();
            validation.Message = "The solver returned no trajectory.";
        else
            validation = azElPlannerMethods.hs3.validateTrajectory( ...
                finalCandidate, obstacles, initialState, goalState, ...
                limits, validationOptions);
        end
        finalCandidate.Validation = validation;
        stageTiming = ...
            azElPlannerMethods.hs3.internal.addHs3CandidateTiming( ...
            stageTiming, finalCandidate);
    end
    finalCandidate.Validation = validation;
    previousSummary = seedSummaries(seedIndex);
    previousCandidate = candidates{seedIndex};
    if validation.Passed
        if isnan(firstValidatedMotionTime_s)
            firstValidatedMotionTime_s = toc(planningTimer);
        end
        improvementDeadline_s = min(improvementDeadline_s, ...
            toc(planningTimer) + options.MaximumHs3ImprovementTime_s);
    end
    if candidateIsBetter(finalCandidate, previousCandidate, ...
            options.ArrivalTimeTolerance_s)
        selectedCandidate = finalCandidate;
    elseif ~candidatePassed(previousCandidate) && ...
            ~validation.Passed
        selectedCandidate = finalCandidate;
    else
        selectedCandidate = previousCandidate;
    end
    candidates{seedIndex} = selectedCandidate;
    seedSummaries(seedIndex) = candidateSummary( ...
        selectedCandidate, relinearizationCount, previousSummary);
    seedSummaries(seedIndex).Hs3Attempted = true;
    seedSummaries(seedIndex).Hs3OptimizerFeasible = finalCandidate.OptimizerFeasible;
    seedSummaries(seedIndex).Hs3ValidationPassed = validation.Passed;
    seedSummaries(seedIndex).Hs3TerminationReason = finalCandidate.TerminationReason;
    seedSummaries(seedIndex).Hs3SolverDiagnostics = finalCandidate.SolverDiagnostics;
    if candidatePassed(selectedCandidate)
        seedSummaries(seedIndex).SelectedMotionSource = selectedCandidate.MotionSource;
    end
    if options.Verbose
        fprintf("[AzEl][seed %d/%d][HS3] optimizer=%d, validation=%d, " + ...
            "arrival=%.6g s, violation=%.3g, reason=%s.\n", ...
            seedIndex, numel(seeds), ...
            seedSummaries(seedIndex).OptimizerFeasible, ...
            seedSummaries(seedIndex).ValidationPassed, ...
            seedSummaries(seedIndex).ArrivalTime_s, ...
            seedSummaries(seedIndex).MaximumConstraintViolation, ...
            seedSummaries(seedIndex).TerminationReason);
    end
    hs3ElapsedTime_s = hs3ElapsedTime_s + toc(hs3AttemptTimer);
end
%% Section 6: Select And Assemble The Result
% Preserve every seed summary and aggregate timing before selecting a winner.
result.SeedSummaries = seedSummaries;
result.SearchDiagnostics.SeedSummaries = seedSummaries;
result.FirstValidatedMotionTime_s = firstValidatedMotionTime_s;
result.SearchDiagnostics.FirstValidatedMotionTime_s = firstValidatedMotionTime_s;
validatedIndices = find([seedSummaries.ValidationPassed]).';
result.SearchDiagnostics.ValidatedCandidateCount = numel(validatedIndices);
result.SearchDiagnostics.SeedGenerationElapsedTime_s = gridDiagnostics.ElapsedTime_s;
result.SearchDiagnostics.FirstMotionElapsedTime_s = firstMotionElapsedTime_s;
result.SearchDiagnostics.Hs3ElapsedTime_s = hs3ElapsedTime_s;
result.SearchDiagnostics.AttemptedSeedCount = nnz( ...
    [seedSummaries.FirstMotionAttempted] | [seedSummaries.Hs3Attempted]);

% Return a stable, diagnosable failure when no candidate passed validation.
if isempty(validatedIndices)
    result.Message = "No attempted seed motion passed independent validation.";
    result.TerminationReason = "noValidatedSeed";
    result.SearchDiagnostics.TerminationReason = result.TerminationReason;
    result.SearchDiagnostics.BestPartialSeedIndex = bestPartialSeed(seedSummaries);
    result = azElPlannerMethods.internal.stageTiming( ...
        result, planningTimer, stageTiming);
    if options.Verbose
        fprintf("[AzEl] Complete: success=0, reason=%s, seeds=%d, " + ...
            "attempted=%d, validated=0, elapsed=%.3f s.\n", ...
            result.TerminationReason, numel(seeds), ...
            result.SearchDiagnostics.AttemptedSeedCount, ...
            result.ElapsedPlanningTime_s);
    end
    return;
end

% Choose the best independently validated candidate under the configured tie tolerance.
selectedSeedIndex = selectValidatedCandidate( ...
    seedSummaries, validatedIndices, options.ArrivalTimeTolerance_s);
selectedCandidate = candidates{selectedSeedIndex};

if options.Verbose
    fprintf("[AzEl][select] seed=%d, source=%s, arrival=%.6g s.\n", ...
        selectedSeedIndex, selectedCandidate.MotionSource, ...
        selectedCandidate.FinalTime_s);
end
selectedAttemptSummary = result.SeedSummaries(selectedSeedIndex);
meshRefinementPassCount = 0;
% Refine only an HS3-selected motion; deterministic stop motions have no HS3 mesh.
if selectedCandidate.MotionSource == "hs3"
    refinementOptions = internalOptions;
    segmentCount = selectedCandidate.Polynomial.SegmentCount;
    for passIndex = 1:refinementOptions.MaximumMeshRefinementPasses
        nextSegmentCount = min(2 * segmentCount, ...
            refinementOptions.MaximumCollocationSegmentCount);
        remainingTime_s = improvementDeadline_s - toc(planningTimer);
        if nextSegmentCount <= segmentCount || ...
                (isfinite(remainingTime_s) && remainingTime_s <= 0.1)
            break;
        end
        refinedOptions = refinementOptions;
        refinedOptions.CollocationSegmentCount = nextSegmentCount;
        if refinementOptions.Verbose
            fprintf("[AzEl][refine] pass=%d, segments=%d->%d.\n", ...
                passIndex, segmentCount, nextSegmentCount);
        end
        if isfinite(remainingTime_s)
            validationReserve_s = min(2, max(0.05, 0.2 * remainingTime_s));
            refinedOptions.MaximumSolverTime_s = remainingTime_s - ...
                validationReserve_s;
        else
            refinedOptions.MaximumSolverTime_s = Inf;
        end
        refinedSeed = azElPlannerMethods.hs3.internal.candidateSeed( ...
            selectedCandidate, seeds(selectedSeedIndex));
        trial = azElPlannerMethods.hs3.internal.motion.solveHs3( ...
            obstacles, initialState, goalState, limits, ...
            refinedOptions, refinedSeed);
        trial.MotionSource = "hs3";
        if isempty(trial.time_s)
            trial.Validation = azElPlannerMethods.hs3.validateTrajectory();
            trial.Validation.Message = "The solver returned no trajectory.";
        else
            trial.Validation = azElPlannerMethods.hs3.validateTrajectory( ...
                trial, obstacles, initialState, goalState, limits, ...
                refinementOptions);
        end
        stageTiming = azElPlannerMethods.hs3.internal.addHs3CandidateTiming( ...
            stageTiming, trial);
        arrivalIsNoWorse = trial.FinalTime_s <= selectedCandidate.FinalTime_s + ...
            refinementOptions.ArrivalTimeTolerance_s;
        arrivalIsBetter = trial.FinalTime_s < selectedCandidate.FinalTime_s - ...
            refinementOptions.ArrivalTimeTolerance_s;
        jerkTolerance = max(1e-12, ...
            refinementOptions.OptimalityTolerance * max(1, ...
            selectedCandidate.IntegratedSquaredJerk_deg2_s5));
        jerkIsNoWorse = trial.IntegratedSquaredJerk_deg2_s5 <= ...
            selectedCandidate.IntegratedSquaredJerk_deg2_s5 + jerkTolerance;
        qualityIsNoWorse = arrivalIsNoWorse && ...
            (arrivalIsBetter || jerkIsNoWorse);
        if ~trial.Validation.Passed || ~qualityIsNoWorse
            if refinementOptions.Verbose
                fprintf("[AzEl][refine] pass=%d retained=0, " + ...
                    "validation=%d.\n", passIndex, trial.Validation.Passed);
            end
            break;
        end
        selectedCandidate = trial;
        segmentCount = nextSegmentCount;
        meshRefinementPassCount = passIndex;
        if refinementOptions.Verbose
            fprintf("[AzEl][refine] pass=%d retained=1, " + ...
                "validation=1.\n", passIndex);
        end
    end
end
% Replace the selected seed's earlier summary with its final refined evidence.
selectedRelinearizationCount = selectedAttemptSummary.RelinearizationCount;
result.SeedSummaries(selectedSeedIndex) = candidateSummary( ...
    selectedCandidate, selectedRelinearizationCount, ...
    selectedAttemptSummary);
result.SeedSummaries(selectedSeedIndex).MeshRefinementPassCount = meshRefinementPassCount;
result.SearchDiagnostics.SeedSummaries = result.SeedSummaries;
result.SearchDiagnostics.MeshRefinementPassCount = meshRefinementPassCount;

% Describe success according to the source of the selected validated motion.
result.Success = true;
if selectedCandidate.MotionSource == "hs3"
    result.Message = "An independently validated local HS3 improvement was selected.";
    result.OptimalityStatement = "Best validated motion from the finite deterministic seed set and " + ...
        "bounded local HS3 work; no global certificate.";
else
    result.Message = "A deterministic finite-jerk seed motion passed independent validation.";
    result.OptimalityStatement = "Validated stop-at-waypoint motion on one finite deterministic " + ...
        "seed; no minimum-time or global certificate.";
end
result.TerminationReason = "goalReached";
result.SelectedSeedIndex = selectedSeedIndex;
result.SelectedMotionSource = selectedCandidate.MotionSource;
result.SelectedSeed_deg = seeds(selectedSeedIndex).position_deg;

% Copy the complete time-parameterized trajectory and its independent certificate.
result.time_s = selectedCandidate.time_s;
result.position_deg = selectedCandidate.position_deg;
result.velocity_deg_s = selectedCandidate.velocity_deg_s;
result.acceleration_deg_s2 = selectedCandidate.acceleration_deg_s2;
result.jerk_deg_s3 = selectedCandidate.jerk_deg_s3;
result.Polynomial = selectedCandidate.Polynomial;
result.SeedCorridorBoundary_deg = selectedCandidate.SeedCorridorBoundary_deg;
result.SeedCorridor = selectedCandidate.SeedCorridor;
result.Validation = selectedCandidate.Validation;

% Finish the public timing and search records without rerunning any planning stage.
result.ArrivalTime_s = selectedCandidate.FinalTime_s;
result.TrajectoryDuration_s = selectedCandidate.MotionDuration_s;
result.GoalHorizon_s = goalState.time_s - initialState.time_s;
result.SearchDiagnostics.TerminationReason = result.TerminationReason;
result.SearchDiagnostics.BestPartialSeedIndex = selectedSeedIndex;
result = azElPlannerMethods.internal.stageTiming( ...
    result, planningTimer, stageTiming);

if options.Verbose
    fprintf("[AzEl] Complete: success=1, reason=%s, seeds=%d, " + ...
        "attempted=%d, validated=%d, selected=%d, source=%s, " + ...
        "arrival=%.6g s, elapsed=%.3f s.\n", ...
        result.TerminationReason, numel(seeds), ...
        result.SearchDiagnostics.AttemptedSeedCount, ...
        numel(validatedIndices), selectedSeedIndex, ...
        result.SelectedMotionSource, result.ArrivalTime_s, ...
        result.ElapsedPlanningTime_s);
end
end

function options = plannerDefaults()
% Define the complete public planner option source of truth.
options = struct( ...
    "GoalTimeMode", "earliestArrival", ...
    "SampleTime_s", 0.05, ...
    "AllowAzimuthWrapping", false, ...
    "MaximumSeedCount", 5, ...
    "DirectSeedOnly", false, ...
    "SeedClusterDistance_deg", 0, ...
    "CollocationSegmentCount", 10, ...
    "MaximumCollocationSegmentCount", 24, ...
    "MaximumMeshRefinementPasses", 0, ...
    "MaximumNlpIterations", 300, ...
    "MaximumNlpFunctionEvaluations", 30000, ...
    "EnableHs3Improvement", true, ...
    "MaximumHs3ImprovementTime_s", 15, ...
    "ArrivalTimeTolerance_s", 1e-3, ...
    "ConstraintTolerance", 1e-7, ...
    "OptimalityTolerance", 1e-7, ...
    "StepTolerance", 1e-10, ...
    "CollisionClearanceTolerance_deg", 1e-7, ...
    "CollisionMinimumTimeStep_s", 0.00025, ...
    "Verbose", false, ...
    "RandomSeed", 0);
end

function options = resolvePlannerOptions(defaults, overrides)
% Merge, normalize, and validate all public options once.
if ~isstruct(overrides) || ~isscalar(overrides)
    error("planAzElMotion:InvalidOptions", ...
        "optionOverrides must be a scalar struct.");
end
removedNames = ["MaximumPlanningTime_s", "AzimuthInterval_deg", ...
    "ElevationInterval_deg"];

% Reject every retired option explicitly so an old request cannot silently
% change meaning on this planner snapshot.
for removedName = removedNames
    if isfield(overrides, removedName)
        if removedName == "MaximumPlanningTime_s"
            error("planAzElMotion:RemovedMaximumPlanningTime", ...
                "MaximumPlanningTime_s has been removed. Use finite " + ...
                "algorithmic work limits and Verbose progress instead.");
        end
        replacementName = lower(extractBefore(removedName, "Interval")) + ...
            "Interval_deg";
        error("planAzElMotion:WorkspaceLimitMoved", ...
            "%s has moved from options to limits.%s.", ...
            removedName, replacementName);
    end
end
[options, unknownNames] = azElPlannerMethods.hs3.internal.resolveOptions(defaults, overrides);
if ~isempty(unknownNames)
    warning("planAzElMotion:UnknownOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", ...
        strjoin(unknownNames, ", "));
end
options.GoalTimeMode = string(options.GoalTimeMode);
if ~isscalar(options.GoalTimeMode) || ...
        ~any(options.GoalTimeMode == ["earliestArrival", "fixedArrival"])
    error("planAzElMotion:InvalidGoalTimeMode", ...
        "GoalTimeMode must be 'earliestArrival' or 'fixedArrival'.");
end
logicalNames = ["AllowAzimuthWrapping", "DirectSeedOnly", ...
    "EnableHs3Improvement", "Verbose"];

% Normalize each public logical control with the same scalar contract.
for name = logicalNames
    options.(name) = azElPlannerMethods.hs3.internal.normalizeLogicalScalar( ...
        options.(name), name, "planAzElMotion:InvalidLogicalOption");
end
validateattributes(options.SampleTime_s, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'positive'});
validateattributes(options.MaximumSeedCount, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', '>=', 1, '<=', 9});
validateattributes(options.SeedClusterDistance_deg, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'});
validateattributes(options.CollocationSegmentCount, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', '>=', 2, '<=', 40});
validateattributes(options.MaximumCollocationSegmentCount, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', ...
    '>=', options.CollocationSegmentCount, '<=', 40});
validateattributes(options.MaximumMeshRefinementPasses, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', '>=', 0, '<=', 2});
integerNames = ["MaximumNlpIterations", ...
    "MaximumNlpFunctionEvaluations"];

% Validate both finite NLP work budgets as positive integer counts.
for name = integerNames
    validateattributes(options.(name), {'numeric'}, ...
        {'real', 'finite', 'scalar', 'integer', 'positive'});
end
positiveNames = [ ...
    "ArrivalTimeTolerance_s", ...
    "ConstraintTolerance", "OptimalityTolerance", "StepTolerance", ...
    "CollisionMinimumTimeStep_s"];

% Validate every strictly positive tolerance and time-step control uniformly.
for name = positiveNames
    validateattributes(options.(name), {'numeric'}, ...
        {'real', 'finite', 'scalar', 'positive'});
end
validateattributes(options.CollisionClearanceTolerance_deg, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'});
validateattributes(options.MaximumHs3ImprovementTime_s, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'});
validateattributes(options.RandomSeed, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'nonnegative'});
end

function state = normalizeState(state, label)
% Normalize one fixed endpoint state and derivative defaults.
if ~isstruct(state) || ~isscalar(state) || ...
        ~all(isfield(state, {'time_s', 'position_deg'}))
    error("planAzElMotion:InvalidState", ...
        "%s must be a scalar struct with time_s and position_deg.", label);
end
validateattributes(state.time_s, {'numeric'}, ...
    {'real', 'finite', 'scalar'});
validateattributes(state.position_deg, {'numeric'}, ...
    {'real', 'finite', 'vector', 'numel', 2});
state.time_s = double(state.time_s);
state.position_deg = double(state.position_deg(:).');
state = defaultDerivative(state, "velocity_deg_s");
state = defaultDerivative(state, "acceleration_deg_s2");
end

function goalState = normalizeGoalState(goalState)
% Normalize a fixed goal or sampled moving-goal history.
goalState = normalizeState(goalState, "goalState");
if isfield(goalState, "targetTime_s") || ...
        isfield(goalState, "targetPosition_deg")
    if ~all(isfield(goalState, ...
            {'targetTime_s', 'targetPosition_deg'}))
        error("planAzElMotion:IncompleteMovingGoal", ...
            "targetTime_s and targetPosition_deg must be supplied together.");
    end
    validateattributes(goalState.targetTime_s, {'numeric'}, ...
        {'real', 'finite', 'vector', 'increasing'});
    goalState.targetTime_s = double(goalState.targetTime_s(:));
    if numel(goalState.targetTime_s) < 2
        error("planAzElMotion:MovingGoalHistoryTooShort", ...
            "targetTime_s must contain at least two increasing samples.");
    end
    validateattributes(goalState.targetPosition_deg, {'numeric'}, ...
        {'real', 'finite', '2d', 'ncols', 2, ...
        'nrows', numel(goalState.targetTime_s)});
    goalState.targetPosition_deg = double(goalState.targetPosition_deg);
    if goalState.time_s < goalState.targetTime_s(1) || ...
            goalState.time_s > goalState.targetTime_s(end)
        error("planAzElMotion:MovingGoalHorizonOutsideHistory", ...
            "goalState.time_s must be inside targetTime_s.");
    end
    if ~isfield(goalState, "InterpolationMethod") || ...
            isempty(goalState.InterpolationMethod)
        goalState.InterpolationMethod = "linear";
    end
    goalState.InterpolationMethod = string(goalState.InterpolationMethod);
    if ~isscalar(goalState.InterpolationMethod) || ...
            ~any(goalState.InterpolationMethod == ["linear", "pchip"])
        error("planAzElMotion:InvalidGoalInterpolation", ...
            "InterpolationMethod must be 'linear' or 'pchip'.");
    end
end
end

function state = defaultDerivative(state, fieldName)
% Apply one two-axis zero derivative default at the public boundary.
if ~isfield(state, fieldName) || isempty(state.(fieldName))
    state.(fieldName) = [0 0];
else
    validateattributes(state.(fieldName), {'numeric'}, ...
        {'real', 'finite', 'vector', 'numel', 2});
    state.(fieldName) = double(state.(fieldName)(:).');
end
end

function limits = normalizeLimits(limits)
% Validate physical limits and own the workspace intervals.
requiredFields = ["maxVelocity_deg_s", "maxAcceleration_deg_s2", ...
    "maxJerk_deg_s3"];
if ~isstruct(limits) || ~isscalar(limits) || ...
        ~all(isfield(limits, cellstr(requiredFields)))
    error("planAzElMotion:InvalidLimits", ...
        "limits must contain velocity, acceleration, and jerk limits.");
end

% Normalize every required per-axis derivative limit into a row pair.
for name = requiredFields
    validateattributes(limits.(name), {'numeric'}, ...
        {'real', 'finite', 'positive', 'vector', 'numel', 2});
    limits.(name) = double(limits.(name)(:).');
end
intervalDefaults = struct( ...
    "azimuthInterval_deg", [-180 180], ...
    "elevationInterval_deg", [-90 90]);
intervalNames = string(fieldnames(intervalDefaults));

% Apply and validate each workspace interval independently so its axis and
% units remain explicit.
for name = reshape(intervalNames, 1, [])
    if ~isfield(limits, name) || isempty(limits.(name))
        limits.(name) = intervalDefaults.(name);
    end
    validateattributes(limits.(name), {'numeric'}, ...
        {'real', 'finite', 'vector', 'numel', 2, 'increasing'}, ...
        "planAzElMotion", name);
    limits.(name) = double(limits.(name)(:).');
end
end

function [feasible, message, reason] = validateEndpoints( ...
        obstacles, initialState, goalState, limits, options)
% Return expected endpoint infeasibility without invoking the optimizer.
goalPosition_deg = azElPlannerMethods.hs3.internal.goalPositionAtTime( ...
    goalState, goalState.time_s);
startIsBlocked = queryAzElTimeObstacle( ...
    obstacles, initialState.position_deg(1), ...
    initialState.position_deg(2), initialState.time_s);
fixedTerminalIsBlocked = false;
if options.GoalTimeMode == "fixedArrival"
    fixedTerminalIsBlocked = queryAzElTimeObstacle( ...
        obstacles, goalPosition_deg(1), goalPosition_deg(2), ...
        goalState.time_s);
end
if startIsBlocked || fixedTerminalIsBlocked
    feasible = false;
    message = "The protected geometry contains the start or fixed terminal point.";
    reason = "endpointBlocked";
    return;
end
derivativesWithinLimits = all(abs(initialState.velocity_deg_s) <= ...
    limits.maxVelocity_deg_s) && ...
    all(abs(goalState.velocity_deg_s) <= limits.maxVelocity_deg_s) && ...
    all(abs(initialState.acceleration_deg_s2) <= ...
    limits.maxAcceleration_deg_s2) && ...
    all(abs(goalState.acceleration_deg_s2) <= ...
    limits.maxAcceleration_deg_s2);
if ~derivativesWithinLimits
    feasible = false;
    message = "An endpoint derivative exceeds its physical limit.";
    reason = "dynamicEndpointInfeasible";
    return;
end
hasMovingGoal = isfield(goalState, "targetTime_s") && ...
    ~isempty(goalState.targetTime_s);
availableDuration_s = goalState.time_s - initialState.time_s;
if options.GoalTimeMode == "fixedArrival" || ~hasMovingGoal
    endpointDisplacement_deg = abs(goalPosition_deg - ...
        initialState.position_deg);
    minimumVelocityDuration_s = max(endpointDisplacement_deg ./ ...
        limits.maxVelocity_deg_s);
    if minimumVelocityDuration_s > availableDuration_s + ...
            options.ArrivalTimeTolerance_s
        feasible = false;
        message = sprintf( ...
            "The time window is too short for the endpoint displacement " + ...
            "at the configured velocity limits (minimum %.6g s, " + ...
            "available %.6g s). Increase goalState.time_s or the " + ...
            "velocity limits.", minimumVelocityDuration_s, ...
            availableDuration_s);
        reason = "timeWindowInfeasible";
        return;
    end
end
endpointPosition_deg = initialState.position_deg;
if options.GoalTimeMode == "fixedArrival" || ~hasMovingGoal
    endpointPosition_deg(end + 1, :) = goalPosition_deg;
end
positionWithinBounds = all(endpointPosition_deg(:, 2) >= ...
    limits.elevationInterval_deg(1)) && ...
    all(endpointPosition_deg(:, 2) <= limits.elevationInterval_deg(2));
if ~options.AllowAzimuthWrapping
    positionWithinBounds = positionWithinBounds && ...
        all(endpointPosition_deg(:, 1) >= limits.azimuthInterval_deg(1)) && ...
        all(endpointPosition_deg(:, 1) <= limits.azimuthInterval_deg(2));
end
if ~positionWithinBounds
    feasible = false;
    message = "An endpoint is outside the configured workspace.";
    reason = "endpointOutsideWorkspace";
    return;
end
feasible = true;
message = "";
reason = "";
end

function summary = candidateSummary( ...
        candidate, relinearizationCount, previousSummary)
% Retain concise outcomes without duplicating full trajectories.
summary = previousSummary;
summary.SeedIndex = candidate.SeedIndex;
summary.SeedSource = candidate.SeedSource;
if candidate.Validation.Passed
    summary.SelectedMotionSource = candidate.MotionSource;
end
summary.OptimizerFeasible = candidate.OptimizerFeasible;
summary.ValidationPassed = candidate.Validation.Passed;
summary.CollisionFree = candidate.Validation.CollisionFree;
summary.CollisionResolved = candidate.Validation.CollisionResolved;
summary.MinimumClearance_deg = candidate.Validation.MinimumClearance_deg;
summary.UnresolvedIntervalCount = candidate.Validation.UnresolvedIntervalCount;
summary.ArrivalTime_s = candidate.FinalTime_s;
summary.MotionDuration_s = candidate.MotionDuration_s;
summary.IntegratedSquaredJerk_deg2_s5 = candidate.IntegratedSquaredJerk_deg2_s5;
summary.MaximumConstraintViolation = candidate.MaximumConstraintViolation;
summary.RelinearizationCount = relinearizationCount;
summary.TerminationReason = candidate.TerminationReason;
summary.Message = candidate.Message + " " + candidate.Validation.Message;
summary.SolverDiagnostics = candidate.SolverDiagnostics;
if candidate.MotionSource == "hs3"
    summary.Hs3Attempted = true;
    summary.Hs3OptimizerFeasible = candidate.OptimizerFeasible;
    summary.Hs3ValidationPassed = candidate.Validation.Passed;
    summary.Hs3TerminationReason = candidate.TerminationReason;
    summary.Hs3SolverDiagnostics = candidate.SolverDiagnostics;
end
end

function passed = candidatePassed(candidate)
% Read one candidate validation state without assuming a nonempty cell.
passed = ~isempty(candidate) && isstruct(candidate) && ...
    isfield(candidate, "Validation") && candidate.Validation.Passed;
end

function better = candidateIsBetter(trial, current, arrivalTolerance_s)
% Compare two validated motions by arrival, jerk, and stable seed order.
if ~candidatePassed(trial)
    better = false;
    return;
end
if ~candidatePassed(current)
    better = true;
    return;
end
arrivalDifference_s = trial.FinalTime_s - current.FinalTime_s;
better = arrivalDifference_s < -arrivalTolerance_s || ...
    (abs(arrivalDifference_s) <= arrivalTolerance_s && ...
    trial.IntegratedSquaredJerk_deg2_s5 < ...
    current.IntegratedSquaredJerk_deg2_s5 - 1e-12);
end

function selectedIndex = selectValidatedCandidate( ...
        summaries, validatedIndices, arrivalTolerance_s)
% Apply time, jerk, and deterministic seed-index lexicographic order.
arrivalTimes_s = [summaries(validatedIndices).ArrivalTime_s].';
minimumArrival_s = min(arrivalTimes_s);
timeEquivalent = validatedIndices( ...
    arrivalTimes_s <= minimumArrival_s + arrivalTolerance_s);
jerkValues = [summaries(timeEquivalent).IntegratedSquaredJerk_deg2_s5].';
minimumJerk = min(jerkValues);
jerkEquivalent = timeEquivalent(jerkValues <= minimumJerk + 1e-12);
selectedIndex = min(jerkEquivalent);
end

function index = bestPartialSeed(summaries)
% Prefer independently resolved collision state before NLP residual.
violation = [summaries.MaximumConstraintViolation].';
violation(~isfinite(violation)) = Inf;
collisionRank = 2 * ~[summaries.CollisionResolved].' + ...
    ~[summaries.CollisionFree].';
minimumClearance_deg = [summaries.MinimumClearance_deg].';
minimumClearance_deg(~isfinite(minimumClearance_deg)) = -Inf;
[~, order] = sortrows( ...
    [collisionRank, violation, -minimumClearance_deg], [1 2 3]);
if isempty(order) || isinf(violation(order(1)))
    index = 0;
else
    index = order(1);
end
end
