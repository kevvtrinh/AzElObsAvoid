function result = runCorridorPlanner( ...
        result, summaryTemplate, seeds, gridDiagnostics, obstacles, ...
        initialState, goalState, limits, options, planningTimer)
%% Section 0: Header & Readme
% SYNTAX
%   result = azElInternal.motion.runCorridorPlanner(result, ...
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
stageTiming = azElInternal.stageTiming();
firstValidatedMotionTime_s = NaN;
candidateElapsedTime_s = 0;
geometryIsStatic = isempty(obstacles) || all(arrayfun(@(obstacle) all( ...
    obstacle.InternalPreparation.IntervalSpeedBound_deg_s == 0), obstacles));

% Prefer one bounded compact solve on the input-eligible topology whose
% timing semantics must be preserved, then fall back to spatial topology.
compactPrimaryIndex = 0;
compactPrimaryLength_deg = Inf;
hasExplicitHoldSeed = any(arrayfun(@(seed) ...
    seed.Source == "directWait" && any( ...
    vecnorm(diff(seed.position_deg), 2, 2) <= 1e-12), seeds));
for seedIndex = 1:numel(seeds)
    seedRoute_deg = seeds(seedIndex).position_deg;
    compactEligible = size(seedRoute_deg, 1) >= 3 && ...
        seeds(seedIndex).Source == "visibilityGraph";
    seedLength_deg = sum(vecnorm(diff(seedRoute_deg), 2, 2));
    if compactEligible && seedLength_deg < compactPrimaryLength_deg
        compactPrimaryIndex = seedIndex;
        compactPrimaryLength_deg = seedLength_deg;
    end
end
compactPrimaryCandidate = [];
compactPrimaryElapsedTime_s = 0;
if compactPrimaryIndex > 0
    [compactPrimaryCandidate, compactPrimaryDiagnostics, ...
        compactPrimaryElapsedTime_s] = ...
        azElInternal.motion.solveCompactC3Candidate( ...
        seeds(compactPrimaryIndex), obstacles, initialState, goalState, ...
        limits, options);
    stageTiming = addStageTiming( ...
        stageTiming, compactPrimaryDiagnostics.StageTiming);
    result.SearchDiagnostics.CompactC3 = compactPrimaryDiagnostics;
end
compactPrimaryAccepted = ~isempty(compactPrimaryCandidate) && ...
    compactPrimaryCandidate.Success;
if compactPrimaryAccepted && ~hasExplicitHoldSeed && ...
        options.GoalTimeMode == "earliestArrival"
    attemptedSeedIndices = compactPrimaryIndex;
elseif hasExplicitHoldSeed && options.GoalTimeMode == "earliestArrival"
    attemptedSeedIndices = find( ...
        [seeds.Source] ~= "directVisibilityEdge");
else
    attemptedSeedIndices = 1:numel(seeds);
end
candidateElapsedTime_s = candidateElapsedTime_s + ...
    compactPrimaryElapsedTime_s;

%% Section 2: Construct And Validate Every Seed Candidate

% Preserve seed order so diagnostics and final tie-breaking remain deterministic.
for seedIndex = attemptedSeedIndices
    % -- Translate this topology seed into a motion-solver request. --
    % Repeated neighboring points encode an intentional wait. They must not
    % be expanded like an ordinary moving edge or the wait semantics vanish.
    candidateTimer = tic;
    route_deg = seeds(seedIndex).position_deg;
    if options.Verbose
        fprintf("[AzEl][seed %d/%d][corridorQuintic] start, " + ...
            "source=%s, route=%d.\n", seedIndex, numel(seeds), seeds(seedIndex).Source, size(route_deg, 1));
    end
    usedCompactPrimary = compactPrimaryIndex > 0 && ...
        seedIndex == compactPrimaryIndex;
    if usedCompactPrimary
        candidate = compactPrimaryCandidate;
    elseif geometryIsStatic && size(route_deg, 1) == 2
        candidate = buildDirectCandidate( ...
            route_deg, obstacles, initialState, goalState, limits, options);
        stageTiming = addStageTiming( ...
            stageTiming, candidate.OptimizerDiagnostics.StageTiming);
    else
        [candidate, compactDiagnostics] = ...
            azElInternal.motion.solveCompactC3Candidate( ...
            seeds(seedIndex), obstacles, initialState, goalState, ...
            limits, options);
        stageTiming = addStageTiming( ...
            stageTiming, compactDiagnostics.StageTiming);
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

%% Section 3: Select The Earliest Validated Quintic

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
result.SearchDiagnostics.AttemptedSeedCount = numel(attemptedSeedIndices);
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
result.SeedSummaries = seedSummaries;
result.SearchDiagnostics.SeedSummaries = seedSummaries;
result.SearchDiagnostics.FirstMotionElapsedTime_s = candidateElapsedTime_s;

%% Section 4: Assemble The Stable Public Result

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
result.OptimalityStatement = "Earliest validated compact duration among the " + ...
    "attempted deterministic topologies; no global certificate.";
result.SearchDiagnostics.StageTiming = stageTiming;
end


function candidate = buildDirectCandidate( ...
        route_deg, obstacles, initialState, goalState, limits, options)
% Build and independently validate the exact two-point analytic quintic.
directTimer = tic;
stageTiming = azElInternal.stageTiming();
motionTimer = tic;
motionOptions = struct( ...
    "SampleTime_s", options.SampleTime_s, ...
    "GoalTimeMode", options.GoalTimeMode, ...
    "AllowAzimuthWrapping", options.AllowAzimuthWrapping);
candidate = azElInternal.motion.buildQuinticSpline( ...
    route_deg, initialState, goalState, limits, motionOptions);
stageTiming.MotionSolvingElapsedTime_s = toc(motionTimer);
corridorTimer = tic;
if ~isempty(obstacles)
    boundary_deg = ...
        azElInternal.obstacles.buildEnvelopeBoundary( ...
        obstacles, 1e-6);
    seed = struct("position_deg", route_deg, ...
        "tau", [0; 1], "CorridorBoundary_deg", boundary_deg);
    candidate.SeedCorridorBoundary_deg = boundary_deg;
    candidate.SeedCorridor = ...
        azElInternal.buildSeedCorridor( ...
        seed, candidate.Polynomial.SegmentCount);
end
stageTiming.CorridorConstructionElapsedTime_s = toc(corridorTimer);
validation = validateAzElTrajectory( ...
    candidate, obstacles, initialState, goalState, limits, options);
stageTiming.CollisionCheckingElapsedTime_s = ...
    validation.CollisionCheckingElapsedTime_s;
stageTiming.FinalValidationElapsedTime_s = max(0, ...
    validation.ElapsedTime_s - validation.CollisionCheckingElapsedTime_s);
candidate.Validation = validation;
candidate.Success = validation.Passed;
if validation.Passed
    candidate.Message = "The direct analytic quintic passed validation.";
    candidate.TerminationReason = "directQuinticValidated";
else
    candidate.Message = "The direct analytic quintic failed validation.";
    candidate.TerminationReason = "directQuinticNotValidated";
end
candidate.OriginalRoute_deg = route_deg;
candidate.ExpandedRoute_deg = route_deg;
candidate.ReducedRoute_deg = route_deg;
candidate.OptimizerOptions = motionOptions;
if validation.Passed
    exitFlag = 1;
    maximumInequality_deg = 0;
else
    exitFlag = -1;
    maximumInequality_deg = Inf;
end
candidate.OptimizerDiagnostics = struct( ...
    "ExitFlag", exitFlag, ...
    "CandidateMaximumInequality_deg", maximumInequality_deg, ...
    "ValidationPassed", validation.Passed, ...
    "ContinuousClearance_deg", validation.MinimumClearance_deg, ...
    "MotionDuration_s", candidate.MotionDuration_s, ...
    "DecisionCount", 0, ...
    "ActualCorridorRecordCount", numel(candidate.SeedCorridor), ...
    "CorridorCertified", validation.SeedCorridorCertified, ...
    "ExactTraversalAttempted", false, ...
    "ExactTraversalAccepted", false, ...
    "StageTiming", azElInternal.stageTiming());
candidate.OptimizerDiagnostics.StageTiming = ...
    azElInternal.stageTiming(stageTiming, toc(directTimer));
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
