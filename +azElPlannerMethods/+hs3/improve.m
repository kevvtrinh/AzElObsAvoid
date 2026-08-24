function result = improve(immutableCompactBaseline, resolvedHs3Options)
%% Section 0: Header & Readme
% SYNTAX
%   result = azElPlannerMethods.hs3.improve( ...
%       immutableCompactBaseline, resolvedHs3Options)
%**************************************************************************
% PURPOSE
%   - Try bounded HS3 work without risking a validated compact baseline or
%     concealing rejected solver and validation evidence.
%**************************************************************************

%% Section 1: Preserve And Describe The Immutable Baseline

baselineFields = {'Success', 'Inputs', 'Seeds', 'SeedSummaries', ...
    'SelectedSeedIndex', 'Polynomial', 'Validation', 'ArrivalTime_s', ...
    'SearchDiagnostics', 'ElapsedPlanningTime_s'};
optionFields = {'EnableHs3Improvement', 'MaximumHs3ImprovementTime_s', ...
    'MaximumMeshRefinementPasses', 'CollocationSegmentCount'};
if ~isstruct(immutableCompactBaseline) || ...
        ~isscalar(immutableCompactBaseline) || ...
        ~all(isfield(immutableCompactBaseline, baselineFields)) || ...
        ~isstruct(resolvedHs3Options) || ~isscalar(resolvedHs3Options) || ...
        ~all(isfield(resolvedHs3Options, optionFields))
    error("azElPlannerMethods:hs3:improve:InvalidInputs", ...
        "Use a compact result and a resolved HS3 options record.");
end
result = immutableCompactBaseline;
result.Options = resolvedHs3Options;
result.SeedSummaries = normalizeSummaries(result.SeedSummaries);
if isfield(result.SearchDiagnostics, "SeedSummaries")
    result.SearchDiagnostics.SeedSummaries = result.SeedSummaries;
end
result.SearchDiagnostics.Hs3ElapsedTime_s = 0;
baselinePassed = immutableCompactBaseline.Success && immutableCompactBaseline.Validation.Passed;
baselineJerk = azElInternal.integratedSquaredPolynomialJerk(immutableCompactBaseline.Polynomial);
attemptTemplate = struct( ...
    "SeedIndex", 0, "SeedSource", "", ...
    "OptimizerFeasible", false, "ValidationPassed", false, ...
    "Accepted", false, "TerminationReason", "notRun", ...
    "ArrivalTime_s", NaN, "IntegratedSquaredJerk_deg2_s5", Inf, ...
    "Comparison", struct(), "SolverDiagnostics", struct(), ...
    "Validation", struct());
composition = struct( ...
    "Mode", "immutableCompactBaselineWithOptionalHs3", ...
    "Baseline", struct( ...
    "Success", immutableCompactBaseline.Success, ...
    "ValidationPassed", baselinePassed, ...
    "SelectedSeedIndex", immutableCompactBaseline.SelectedSeedIndex, ...
    "ArrivalTime_s", immutableCompactBaseline.ArrivalTime_s, ...
    "IntegratedSquaredJerk_deg2_s5", baselineJerk), ...
    "Hs3", struct( ...
    "Enabled", logical(resolvedHs3Options.EnableHs3Improvement), ...
    "Budget_s", resolvedHs3Options.MaximumHs3ImprovementTime_s, ...
    "RefinementSupported", false, ...
    "RequestedRefinementPasses", ...
    resolvedHs3Options.MaximumMeshRefinementPasses, ...
    "Attempted", false, "Accepted", false, "SelectedSeedIndex", 0, ...
    "ElapsedTime_s", 0, "Attempts", repmat(attemptTemplate, 0, 1)));
result.CompositionDiagnostics = composition;
if ~composition.Hs3.Enabled || composition.Hs3.Budget_s <= 0 || ...
        isempty(result.Seeds)
    return;
end

%% Section 2: Try Deterministic Seeds And Monotone Refinements

improvementTimer = tic;
if baselinePassed
    seedOrder = immutableCompactBaseline.SelectedSeedIndex;
else
    seedOrder = 1:numel(immutableCompactBaseline.Seeds);
end
seedOrder = seedOrder(seedOrder >= 1 & seedOrder <= numel(result.Seeds));
for orderIndex = 1:numel(seedOrder)
    seedArrayIndex = seedOrder(orderIndex);
    trialSeed = result.Seeds(seedArrayIndex);
    remaining_s = composition.Hs3.Budget_s - toc(improvementTimer);
    remainingSeedCount = numel(seedOrder) - orderIndex + 1;
    if remaining_s <= 0.01
        break;
    end
    solverOptions = resolvedHs3Options;
    solverOptions.MaximumSolverTime_s = 0.8 * remaining_s / remainingSeedCount;
    trial = solveAndValidate(immutableCompactBaseline, solverOptions, trialSeed);
    [accepted, comparison] = azElInternal.acceptsTrajectoryImprovement( ...
        immutableCompactBaseline, trial);
    attempt = struct( ...
        "SeedIndex", trial.SeedIndex, "SeedSource", trial.SeedSource, ...
        "OptimizerFeasible", trial.OptimizerFeasible, ...
        "ValidationPassed", trial.Validation.Passed, "Accepted", accepted, ...
        "TerminationReason", trial.TerminationReason, ...
        "ArrivalTime_s", trial.FinalTime_s, ...
        "IntegratedSquaredJerk_deg2_s5", ...
        trial.IntegratedSquaredJerk_deg2_s5, "Comparison", comparison, ...
        "SolverDiagnostics", trial.SolverDiagnostics, ...
        "Validation", trial.Validation);
    composition.Hs3.Attempts(end + 1, 1) = attempt;
    result.SeedSummaries(seedArrayIndex).Hs3Attempted = true;
    result.SeedSummaries(seedArrayIndex).Hs3OptimizerFeasible = trial.OptimizerFeasible;
    result.SeedSummaries(seedArrayIndex).Hs3ValidationPassed = trial.Validation.Passed;
    result.SeedSummaries(seedArrayIndex).Hs3TerminationReason = trial.TerminationReason;
    result.SeedSummaries(seedArrayIndex).Hs3SolverDiagnostics = trial.SolverDiagnostics;
    result.SearchDiagnostics.SeedSummaries = result.SeedSummaries;
    if accepted
        composition.Hs3.Accepted = true;
        composition.Hs3.SelectedSeedIndex = seedArrayIndex;
        result = selectCandidate(result, trial, seedArrayIndex);
        break;
    end
end
composition.Hs3.ElapsedTime_s = toc(improvementTimer);
composition.Hs3.Attempted = ~isempty(composition.Hs3.Attempts);
result.CompositionDiagnostics = composition;
result.ElapsedPlanningTime_s = immutableCompactBaseline.ElapsedPlanningTime_s + composition.Hs3.ElapsedTime_s;
result.SearchDiagnostics.Hs3ElapsedTime_s = composition.Hs3.ElapsedTime_s;
result.SearchDiagnostics.StageTiming = azElPlannerMethods.internal.stageTiming(result.SearchDiagnostics.StageTiming, result.ElapsedPlanningTime_s);
if result.Success && ~baselinePassed
    result.FirstValidatedMotionTime_s = result.ElapsedPlanningTime_s;
    result.SearchDiagnostics.FirstValidatedMotionTime_s = result.ElapsedPlanningTime_s;
end
end

function trial = solveAndValidate(baseline, options, seed)
inputs = baseline.Inputs;
trial = azElPlannerMethods.hs3.internal.motion.solveHs3( ...
    inputs.obstacles, inputs.initialState, inputs.goalState, ...
    inputs.limits, options, seed);
trial.MotionSource = "hs3";
if isempty(trial.time_s)
    trial.Validation = validateAzElTrajectory();
    trial.Validation.Message = "The HS3 solver returned no trajectory.";
else
    trial.Validation = validateAzElTrajectory( ...
        trial, inputs.obstacles, inputs.initialState, inputs.goalState, ...
        inputs.limits, options);
    trial.IntegratedSquaredJerk_deg2_s5 = ...
        azElInternal.integratedSquaredPolynomialJerk(trial.Polynomial);
end
end

function summaries = normalizeSummaries(summaries)
names = {'Hs3Attempted', 'Hs3OptimizerFeasible', ...
    'Hs3ValidationPassed', 'Hs3TerminationReason', ...
    'Hs3SolverDiagnostics'};
defaults = {false, false, false, "notRun", struct()};
if isempty(summaries)
    template = cell2struct(cell(numel(fieldnames(summaries)), 1), fieldnames(summaries), 1);
    for fieldIndex = 1:numel(names)
        template.(names{fieldIndex}) = defaults{fieldIndex};
    end
    summaries = repmat(template, size(summaries));
    return;
end
for fieldIndex = 1:numel(names)
    [summaries.(names{fieldIndex})] = deal(defaults{fieldIndex});
end
end

function result = selectCandidate(result, candidate, seedArrayIndex)
result.Success = true;
result.Message = "A validated bounded HS3 improvement was selected.";
result.TerminationReason = "goalReached";
result.SelectedSeedIndex = seedArrayIndex;
result.SelectedMotionSource = "hs3";
result.SelectedSeed_deg = result.Seeds(seedArrayIndex).position_deg;
result.SearchDiagnostics.TerminationReason = result.TerminationReason;
result.SearchDiagnostics.ValidatedCandidateCount = ...
    result.SearchDiagnostics.ValidatedCandidateCount + 1;
result.SearchDiagnostics.BestPartialSeedIndex = seedArrayIndex;
fields = {'time_s', 'position_deg', 'velocity_deg_s', ...
    'acceleration_deg_s2', 'jerk_deg_s3', 'Polynomial', ...
    'SeedCorridorBoundary_deg', 'SeedCorridor', 'Validation'};
for fieldIndex = 1:numel(fields)
    result.(fields{fieldIndex}) = candidate.(fields{fieldIndex});
end
result.ArrivalTime_s = candidate.FinalTime_s;
result.TrajectoryDuration_s = candidate.MotionDuration_s;
summaryNames = {'SelectedMotionSource', 'ValidationPassed', ...
    'ArrivalTime_s', 'MotionDuration_s', ...
    'IntegratedSquaredJerk_deg2_s5', 'MaximumConstraintViolation'};
summaryValues = {"hs3", true, candidate.FinalTime_s, ...
    candidate.MotionDuration_s, candidate.IntegratedSquaredJerk_deg2_s5, ...
    candidate.MaximumConstraintViolation};
for fieldIndex = 1:numel(summaryNames)
    result.SeedSummaries(seedArrayIndex).(summaryNames{fieldIndex}) = ...
        summaryValues{fieldIndex};
end
result.SearchDiagnostics.SeedSummaries = result.SeedSummaries;
result.OptimalityStatement = "Validated bounded HS3 improvement over an " + ...
    "immutable compact baseline; no global certificate.";
end
