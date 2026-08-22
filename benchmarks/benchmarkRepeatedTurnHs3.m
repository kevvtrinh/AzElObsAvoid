function report = benchmarkRepeatedTurnHs3( ...
        turnCounts, benchmarkOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   report = benchmarkRepeatedTurnHs3()
%   report = benchmarkRepeatedTurnHs3(turnCounts)
%   report = benchmarkRepeatedTurnHs3( ...
%       turnCounts, benchmarkOverrides)
%**************************************************************************
% PURPOSE
%   - Measure current public-planner and HS3 scaling on one parameterized
%     alternating-barrier family before changing the motion representation.
%**************************************************************************
% INPUTS
%   - turnCounts (numeric vector, optional; default [1 2 5 10 20])
%       Positive integer alternating-barrier counts. Input order is retained.
%   - benchmarkOverrides (scalar struct, optional; default struct())
%       .RepeatCount is a positive integer (default 1).
%       .MaximumHs3ImprovementTime_s is positive (default 60 seconds).
%       .PlannerVerbose is scalar logical or binary numeric (default false).
%       .PrintProgress is scalar logical or binary numeric (default true).
%       .RandomSeed is a finite nonnegative integer (default 325).
%**************************************************************************
% OUTPUTS
%   - report (scalar struct)
%       Frozen environment/configuration, every run, and per-count summaries.
%       Failed or time-limited runs remain present with NaN motion metrics.
%**************************************************************************
% UNITS
%   - Position and path length are degrees; time is seconds; derivative
%     ratios are dimensionless. Histories use [azimuth elevation].
%**************************************************************************

%% Section 1: Resolve Benchmark Controls

if nargin < 1 || isempty(turnCounts)
    turnCounts = [1 2 5 10 20];
end
if nargin < 2 || isempty(benchmarkOverrides)
    benchmarkOverrides = struct();
end
validateattributes(turnCounts, {'numeric'}, ...
    {'real', 'finite', 'vector', 'integer', 'positive'});
turnCounts = double(turnCounts(:));
defaults = struct( ...
    "RepeatCount", 1, ...
    "MaximumHs3ImprovementTime_s", 60, ...
    "PlannerVerbose", false, ...
    "PrintProgress", true, ...
    "RandomSeed", 325);
[controls, unknownNames] = azElInternal.resolveOptions( ...
    defaults, benchmarkOverrides);
if ~isempty(unknownNames)
    warning("benchmarkRepeatedTurnHs3:UnknownOptions", ...
        "Ignoring unknown fields: %s. No behavior changed.", ...
        strjoin(unknownNames, ", "));
end
controls.PlannerVerbose = azElInternal.normalizeLogicalScalar( ...
    controls.PlannerVerbose, "PlannerVerbose", ...
    "benchmarkRepeatedTurnHs3:InvalidLogicalOption");
controls.PrintProgress = azElInternal.normalizeLogicalScalar( ...
    controls.PrintProgress, "PrintProgress", ...
    "benchmarkRepeatedTurnHs3:InvalidLogicalOption");
validateattributes(controls.RepeatCount, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'positive'});
validateattributes(controls.MaximumHs3ImprovementTime_s, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'positive'});
validateattributes(controls.RandomSeed, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'nonnegative'});

%% Section 2: Freeze Environment And Scenario Constants

environment = benchmarkEnvironment();
scenarioConstants = struct( ...
    "barrierSpacing_deg", 4, ...
    "barrierHalfWidth_deg", 0.7, ...
    "barrierCenterMagnitude_deg", 2.5, ...
    "barrierHalfHeight_deg", 2.5, ...
    "safetyMargin_deg", 0.1, ...
    "goalTimePerStage_s", 5.5, ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1], ...
    "maxJerk_deg_s3", [2 2], ...
    "elevationInterval_deg", [-5 5]);
plannerOptions = planAzElMotion();
plannerOptions.GoalTimeMode = "earliestArrival";
plannerOptions.MaximumSeedCount = 3;
plannerOptions.CollocationSegmentCount = 8;
plannerOptions.MaximumCollocationSegmentCount = 40;
plannerOptions.MaximumMeshRefinementPasses = 0;
plannerOptions.MaximumHs3ImprovementTime_s = ...
    controls.MaximumHs3ImprovementTime_s;
plannerOptions.Verbose = controls.PlannerVerbose;
plannerOptions.RandomSeed = controls.RandomSeed;

%% Section 3: Run Every Case Serially

runCount = numel(turnCounts) * controls.RepeatCount;
runs = repmat(emptyRunRecord(), runCount, 1);
plannerResults = cell(runCount, 1);
runIndex = 0;
for turnCountIndex = 1:numel(turnCounts)
    turnCount = turnCounts(turnCountIndex);
    [obstacles, initialState, goalState, limits] = ...
        createRepeatedTurnBenchmarkScenario( ...
        turnCount, scenarioConstants);
    for repeatIndex = 1:controls.RepeatCount
        runIndex = runIndex + 1;
        rng(controls.RandomSeed, "twister");
        plannerTimer = tic;
        result = planAzElMotion( ...
            obstacles, initialState, goalState, limits, plannerOptions);
        plannerResults{runIndex} = result;
        plannerWallTime_s = toc(plannerTimer);
        validationTimer = tic;
        independentValidation = validateAzElTrajectory( ...
            result, obstacles, initialState, goalState, ...
            limits, plannerOptions);
        validationElapsedTime_s = toc(validationTimer);
        runs(runIndex) = collectRunRecord( ...
            turnCount, repeatIndex, controls.RandomSeed, result, ...
            independentValidation, limits, plannerWallTime_s, ...
            validationElapsedTime_s);
        if controls.PrintProgress
            printRunRecord(runs(runIndex));
        end
    end
end

%% Section 4: Summarize Repetitions Without Hiding Extremes

runTable = struct2table(runs);
summaryTable = summarizeRuns(runTable, turnCounts);

%% Section 5: Assemble The Report

report = struct( ...
    "BenchmarkName", "repeatedAlternatingBarrierHs3", ...
    "Environment", environment, ...
    "Controls", controls, ...
    "PlannerOptions", plannerOptions, ...
    "ScenarioConstants", scenarioConstants, ...
    "TurnCounts", turnCounts, ...
    "Runs", runTable, ...
    "Summary", summaryTable, ...
    "PlannerResults", {plannerResults});
end

%% Section 6: Local Functions

function environment = benchmarkEnvironment()
%% Section 0: Header & Readme
% SYNTAX
%   environment = benchmarkEnvironment()
%**************************************************************************
% PURPOSE
%   - Record the source and runtime facts needed to reproduce timing.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - environment (scalar struct)
%       Source revision, MATLAB/toolbox versions, CPU count, and pool state.
%**************************************************************************
% UNITS
%   - CPU count is dimensionless.
%**************************************************************************
[commitStatus, commitText] = system("git rev-parse HEAD");
[branchStatus, branchText] = system("git branch --show-current");
optimizationToolbox = ver("optim");
if isempty(optimizationToolbox)
    optimizationVersion = "unavailable";
else
    optimizationVersion = string(optimizationToolbox.Version);
end
parallelPoolState = "unavailable";
if exist("gcp", "file") == 2
    pool = gcp("nocreate");
    if isempty(pool)
        parallelPoolState = "notRunning";
    else
        parallelPoolState = "running:" + pool.NumWorkers;
    end
end
environment = struct( ...
    "Commit", commandText(commitStatus, commitText), ...
    "Branch", commandText(branchStatus, branchText), ...
    "MATLABVersion", string(version), ...
    "OptimizationToolboxVersion", optimizationVersion, ...
    "Computer", string(computer), ...
    "ReportedCoreCount", feature("numcores"), ...
    "ParallelPoolState", parallelPoolState, ...
    "FigureVisibility", "off", ...
    "AnimationEnabled", false);
end

function value = commandText(status, output)
%% Section 0: Header & Readme
% SYNTAX
%   value = commandText(status, output)
%**************************************************************************
% PURPOSE
%   - Normalize one read-only source-control query for the report.
%**************************************************************************
% INPUTS
%   - status (numeric scalar)
%   - output (text)
%**************************************************************************
% OUTPUTS
%   - value (scalar string)
%**************************************************************************
% UNITS
%   - Dimensionless text.
%**************************************************************************
if status == 0
    value = strip(string(output));
else
    value = "unavailable";
end
end

function record = collectRunRecord(turnCount, repeatIndex, randomSeed, ...
        result, validation, limits, plannerWallTime_s, ...
        validationElapsedTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   record = collectRunRecord(turnCount, repeatIndex, randomSeed, ...
%       result, validation, limits, plannerWallTime_s, ...
%       validationElapsedTime_s)
%**************************************************************************
% PURPOSE
%   - Preserve planner, HS3, validation, and quality evidence for one run.
%**************************************************************************
% INPUTS
%   - turnCount, repeatIndex, randomSeed (integer scalars)
%   - result, validation, limits (planner and independent-check records)
%   - plannerWallTime_s, validationElapsedTime_s (nonnegative scalars)
%**************************************************************************
% OUTPUTS
%   - record (scalar struct)
%**************************************************************************
% UNITS
%   - Length is degrees; time is seconds; limit ratios are dimensionless.
%**************************************************************************
record = emptyRunRecord();
record.TurnCount = turnCount;
record.RepeatIndex = repeatIndex;
record.RandomSeed = randomSeed;
record.ObstacleCount = numel(result.Inputs.obstacles);
record.InitialAzimuth_deg = result.Inputs.initialState.position_deg(1);
record.InitialElevation_deg = result.Inputs.initialState.position_deg(2);
record.GoalAzimuth_deg = result.Inputs.goalState.position_deg(1);
record.GoalElevation_deg = result.Inputs.goalState.position_deg(2);
record.GoalHorizon_s = result.GoalHorizon_s;
record.PlannerSuccess = result.Success;
record.IndependentValidationPassed = result.Success && validation.Passed;
record.TerminationReason = string(result.TerminationReason);
record.SelectedMotionSource = string(result.SelectedMotionSource);
record.PlannerReportedTime_s = result.ElapsedPlanningTime_s;
record.TotalWallTime_s = plannerWallTime_s;
record.SeedGenerationTime_s = ...
    result.SearchDiagnostics.SeedGenerationElapsedTime_s;
record.FirstMotionTime_s = result.SearchDiagnostics.FirstMotionElapsedTime_s;
record.Hs3WallTime_s = result.SearchDiagnostics.Hs3ElapsedTime_s;
record.IndependentValidationTime_s = validationElapsedTime_s;
record.FirstValidatedMotionTime_s = result.FirstValidatedMotionTime_s;
[hs3Summary, hs3SeedIndex] = selectedHs3Summary(result);
if hs3SeedIndex > 0
    record.Hs3Called = true;
    record.Hs3SeedIndex = hs3SeedIndex;
    record.Hs3RouteVertexCount = size( ...
        result.Seeds(hs3SeedIndex).position_deg, 1);
    record.Hs3ValidationPassed = hs3Summary.Hs3ValidationPassed;
    diagnostics = hs3Summary.Hs3SolverDiagnostics;
    record.Hs3SegmentCount = fieldOrNaN(diagnostics, "SegmentCount");
    record.Hs3DecisionVariableCount = ...
        fieldOrNaN(diagnostics, "DecisionVariableCount");
    record.Hs3InequalityConstraintCount = ...
        fieldOrNaN(diagnostics, "InequalityConstraintCount");
    record.Hs3EqualityConstraintCount = ...
        fieldOrNaN(diagnostics, "EqualityConstraintCount");
end
attemptedIndices = find([result.SeedSummaries.Hs3Attempted]);
record.Hs3SeedAttemptCount = numel(attemptedIndices);
record.Hs3SolveCount = sum(1 + ...
    [result.SeedSummaries(attemptedIndices).RelinearizationCount]);
[record.RetainedHs3SolverTime_s, record.Hs3IterationCount, ...
    record.Hs3FunctionEvaluationCount, ...
    record.MaximumHs3RouteVertexCount, ...
    record.MaximumHs3SegmentCount, ...
    record.MaximumHs3DecisionVariableCount] = ...
    aggregateHs3Attempts(result, attemptedIndices);
record.Hs3FallbackUsed = result.Success && record.Hs3Called && ...
    record.SelectedMotionSource ~= "hs3";
if ~result.Success
    return;
end
record.SelectedRouteVertexCount = size( ...
    result.Seeds(result.SelectedSeedIndex).position_deg, 1);
record.SelectedPolylineLength_deg = ...
    result.Seeds(result.SelectedSeedIndex).Length_deg;
record.SmoothedPathLength_deg = sum(vecnorm( ...
    diff(result.position_deg, 1, 1), 2, 2));
record.ArrivalTime_s = result.ArrivalTime_s;
record.MotionDuration_s = result.TrajectoryDuration_s;
record.IntegratedSquaredJerk_deg2_s5 = result.SeedSummaries( ...
    result.SelectedSeedIndex).IntegratedSquaredJerk_deg2_s5;
record.MaximumVelocityRatio = max( ...
    validation.PeakVelocity_deg_s ./ limits.maxVelocity_deg_s);
record.MaximumAccelerationRatio = max( ...
    validation.PeakAcceleration_deg_s2 ./ ...
    limits.maxAcceleration_deg_s2);
record.MaximumJerkRatio = max( ...
    validation.PeakJerk_deg_s3 ./ limits.maxJerk_deg_s3);
record.CollisionFree = validation.CollisionFree;
record.KinematicCertificatePassed = ...
    validation.VelocityWithinLimits && ...
    validation.AccelerationWithinLimits && ...
    validation.JerkWithinLimits && validation.DynamicsConsistent;
end

function [solverTime_s, iterationCount, evaluationCount, ...
        maximumRouteVertexCount, maximumSegmentCount, ...
        maximumDecisionVariableCount] = ...
        aggregateHs3Attempts(result, attemptedIndices)
%% Section 0: Header & Readme
% SYNTAX
%   [solverTime_s, iterationCount, evaluationCount, ...
%       maximumRouteVertexCount, maximumSegmentCount, ...
%       maximumDecisionVariableCount] = ...
%       aggregateHs3Attempts(result, attemptedIndices)
%**************************************************************************
% PURPOSE
%   - Sum all HS3 work and retain the largest attempted problem dimensions.
%**************************************************************************
% INPUTS
%   - result (scalar planner result)
%   - attemptedIndices (numeric index vector)
%**************************************************************************
% OUTPUTS
%   - solverTime_s (summed solver-reported seconds)
%   - iterationCount, evaluationCount (summed fmincon counts or NaN)
%   - maximumRouteVertexCount, maximumSegmentCount,
%       maximumDecisionVariableCount (attempted maxima or NaN)
%**************************************************************************
% UNITS
%   - Time is seconds; other outputs are dimensionless counts.
%**************************************************************************
solverTime_s = 0;
iterationCount = 0;
evaluationCount = 0;
maximumRouteVertexCount = NaN;
maximumSegmentCount = NaN;
maximumDecisionVariableCount = NaN;
hasIterationCount = false;
hasEvaluationCount = false;
for attemptIndex = 1:numel(attemptedIndices)
    seedIndex = attemptedIndices(attemptIndex);
    diagnostics = result.SeedSummaries(seedIndex).Hs3SolverDiagnostics;
    elapsedTime_s = fieldOrNaN(diagnostics, "ElapsedTime_s");
    if isfinite(elapsedTime_s)
        solverTime_s = solverTime_s + elapsedTime_s;
    end
    stageIterations = solverOutputTotal(diagnostics, "iterations");
    if isfinite(stageIterations)
        iterationCount = iterationCount + stageIterations;
        hasIterationCount = true;
    end
    stageEvaluations = solverOutputTotal(diagnostics, "funcCount");
    if isfinite(stageEvaluations)
        evaluationCount = evaluationCount + stageEvaluations;
        hasEvaluationCount = true;
    end
    maximumRouteVertexCount = max( ...
        maximumRouteVertexCount, ...
        size(result.Seeds(seedIndex).position_deg, 1), "omitnan");
    maximumSegmentCount = max(maximumSegmentCount, ...
        fieldOrNaN(diagnostics, "SegmentCount"), "omitnan");
    maximumDecisionVariableCount = max( ...
        maximumDecisionVariableCount, ...
        fieldOrNaN(diagnostics, "DecisionVariableCount"), "omitnan");
end
if ~hasIterationCount
    iterationCount = NaN;
end
if ~hasEvaluationCount
    evaluationCount = NaN;
end
if isempty(attemptedIndices)
    solverTime_s = NaN;
end
end

function [summary, seedIndex] = selectedHs3Summary(result)
%% Section 0: Header & Readme
% SYNTAX
%   [summary, seedIndex] = selectedHs3Summary(result)
%**************************************************************************
% PURPOSE
%   - Select the final-route HS3 attempt or the longest attempted fallback.
%**************************************************************************
% INPUTS
%   - result (scalar planner result)
%**************************************************************************
% OUTPUTS
%   - summary (scalar seed-summary struct or struct())
%   - seedIndex (nonnegative integer scalar)
%**************************************************************************
% UNITS
%   - Dimensionless indices.
%**************************************************************************
summary = struct();
seedIndex = 0;
if isempty(result.SeedSummaries)
    return;
end
attemptedIndices = find([result.SeedSummaries.Hs3Attempted]);
if isempty(attemptedIndices)
    return;
end
if result.SelectedSeedIndex > 0 && ...
        any(attemptedIndices == result.SelectedSeedIndex)
    seedIndex = result.SelectedSeedIndex;
else
    elapsedTimes_s = zeros(numel(attemptedIndices), 1);
    for attemptIndex = 1:numel(attemptedIndices)
        diagnostics = result.SeedSummaries( ...
            attemptedIndices(attemptIndex)).Hs3SolverDiagnostics;
        elapsedTimes_s(attemptIndex) = ...
            fieldOrNaN(diagnostics, "ElapsedTime_s");
    end
    elapsedTimes_s(~isfinite(elapsedTimes_s)) = -Inf;
    [~, maximumIndex] = max(elapsedTimes_s);
    seedIndex = attemptedIndices(maximumIndex);
end
summary = result.SeedSummaries(seedIndex);
end

function total = solverOutputTotal(diagnostics, fieldName)
%% Section 0: Header & Readme
% SYNTAX
%   total = solverOutputTotal(diagnostics, fieldName)
%**************************************************************************
% PURPOSE
%   - Sum one available fmincon output count across both solver stages.
%**************************************************************************
% INPUTS
%   - diagnostics (scalar struct)
%   - fieldName (scalar text)
%**************************************************************************
% OUTPUTS
%   - total (numeric scalar, NaN when unavailable)
%**************************************************************************
% UNITS
%   - Counts are dimensionless.
%**************************************************************************
values = NaN(2, 1);
stageNames = ["StageOneOutput", "StageTwoOutput"];
for stageIndex = 1:numel(stageNames)
    stageName = stageNames(stageIndex);
    if isfield(diagnostics, stageName) && ...
            isstruct(diagnostics.(stageName)) && ...
            isfield(diagnostics.(stageName), fieldName)
        values(stageIndex) = diagnostics.(stageName).(fieldName);
    end
end
if all(isnan(values))
    total = NaN;
else
    total = sum(values, "omitnan");
end
end

function value = fieldOrNaN(record, fieldName)
%% Section 0: Header & Readme
% SYNTAX
%   value = fieldOrNaN(record, fieldName)
%**************************************************************************
% PURPOSE
%   - Read one numeric diagnostic without inventing unavailable evidence.
%**************************************************************************
% INPUTS
%   - record (scalar struct)
%   - fieldName (scalar text)
%**************************************************************************
% OUTPUTS
%   - value (numeric scalar or NaN)
%**************************************************************************
% UNITS
%   - Defined by fieldName.
%**************************************************************************
value = NaN;
if isstruct(record) && isfield(record, fieldName)
    value = record.(fieldName);
end
end

function summaryTable = summarizeRuns(runTable, turnCounts)
%% Section 0: Header & Readme
% SYNTAX
%   summaryTable = summarizeRuns(runTable, turnCounts)
%**************************************************************************
% PURPOSE
%   - Report first, median, minimum, and maximum timing without omissions.
%**************************************************************************
% INPUTS
%   - runTable (table)
%   - turnCounts (numeric column vector)
%**************************************************************************
% OUTPUTS
%   - summaryTable (table)
%**************************************************************************
% UNITS
%   - Timing fields are seconds; counts are dimensionless.
%**************************************************************************
countTotal = numel(turnCounts);
summary = repmat(emptySummaryRecord(), countTotal, 1);
for countIndex = 1:countTotal
    rows = runTable(runTable.TurnCount == turnCounts(countIndex), :);
    summary(countIndex).TurnCount = turnCounts(countIndex);
    summary(countIndex).RepeatCount = height(rows);
    summary(countIndex).PlannerSuccessCount = nnz(rows.PlannerSuccess);
    summary(countIndex).ValidationPassCount = ...
        nnz(rows.IndependentValidationPassed);
    summary(countIndex).Hs3CalledCount = nnz(rows.Hs3Called);
    summary(countIndex).Hs3ValidationPassCount = ...
        nnz(rows.Hs3ValidationPassed);
    summary(countIndex).FallbackCount = nnz(rows.Hs3FallbackUsed);
    summary(countIndex).Hs3WallFirst_s = rows.Hs3WallTime_s(1);
    summary(countIndex).Hs3WallMedian_s = ...
        median(rows.Hs3WallTime_s, "omitnan");
    summary(countIndex).Hs3WallMinimum_s = ...
        min(rows.Hs3WallTime_s, [], "omitnan");
    summary(countIndex).Hs3WallMaximum_s = ...
        max(rows.Hs3WallTime_s, [], "omitnan");
    summary(countIndex).TotalWallFirst_s = rows.TotalWallTime_s(1);
    summary(countIndex).TotalWallMedian_s = ...
        median(rows.TotalWallTime_s, "omitnan");
    summary(countIndex).TotalWallMinimum_s = ...
        min(rows.TotalWallTime_s, [], "omitnan");
    summary(countIndex).TotalWallMaximum_s = ...
        max(rows.TotalWallTime_s, [], "omitnan");
    summary(countIndex).ArrivalMedian_s = ...
        median(rows.ArrivalTime_s, "omitnan");
end
summaryTable = struct2table(summary);
end

function printRunRecord(record)
%% Section 0: Header & Readme
% SYNTAX
%   printRunRecord(record)
%**************************************************************************
% PURPOSE
%   - Emit one complete scaling row immediately after its serial run.
%**************************************************************************
% INPUTS
%   - record (scalar run struct)
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Timing is seconds.
%**************************************************************************
fprintf("[RepeatedTurnHS3] turns=%d repeat=%d success=%d " + ...
    "validation=%d hs3=%d seeds=%d solves=%d hs3Valid=%d fallback=%d " + ...
    "vertices=%g " + ...
    "segments=%g variables=%g iterations=%g evaluations=%g " + ...
    "hs3Wall=%.6f totalWall=%.6f arrival=%.6f reason=%s\n", ...
    record.TurnCount, record.RepeatIndex, record.PlannerSuccess, ...
    record.IndependentValidationPassed, record.Hs3Called, ...
    record.Hs3SeedAttemptCount, record.Hs3SolveCount, ...
    record.Hs3ValidationPassed, record.Hs3FallbackUsed, ...
    record.Hs3RouteVertexCount, record.Hs3SegmentCount, ...
    record.Hs3DecisionVariableCount, record.Hs3IterationCount, ...
    record.Hs3FunctionEvaluationCount, record.Hs3WallTime_s, ...
    record.TotalWallTime_s, record.ArrivalTime_s, ...
    record.TerminationReason);
end

function record = emptyRunRecord()
%% Section 0: Header & Readme
% SYNTAX
%   record = emptyRunRecord()
%**************************************************************************
% PURPOSE
%   - Define one stable run schema for success, fallback, and failure.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - record (scalar struct)
%**************************************************************************
% UNITS
%   - Length is degrees; time is seconds; ratios are dimensionless.
%**************************************************************************
record = struct( ...
    "TurnCount", 0, "RepeatIndex", 0, "RandomSeed", 0, ...
    "ObstacleCount", 0, "InitialAzimuth_deg", NaN, ...
    "InitialElevation_deg", NaN, "GoalAzimuth_deg", NaN, ...
    "GoalElevation_deg", NaN, "GoalHorizon_s", NaN, ...
    "PlannerSuccess", false, "IndependentValidationPassed", false, ...
    "TerminationReason", "notRun", "SelectedMotionSource", "", ...
    "Hs3Called", false, "Hs3ValidationPassed", false, ...
    "Hs3FallbackUsed", false, "Hs3SeedAttemptCount", 0, ...
    "Hs3SolveCount", 0, "Hs3SeedIndex", 0, ...
    "SelectedRouteVertexCount", NaN, "Hs3RouteVertexCount", NaN, ...
    "MaximumHs3RouteVertexCount", NaN, ...
    "Hs3SegmentCount", NaN, "Hs3DecisionVariableCount", NaN, ...
    "MaximumHs3SegmentCount", NaN, ...
    "MaximumHs3DecisionVariableCount", NaN, ...
    "Hs3InequalityConstraintCount", NaN, ...
    "Hs3EqualityConstraintCount", NaN, ...
    "Hs3IterationCount", NaN, "Hs3FunctionEvaluationCount", NaN, ...
    "SeedGenerationTime_s", NaN, "FirstMotionTime_s", NaN, ...
    "RetainedHs3SolverTime_s", NaN, "Hs3WallTime_s", NaN, ...
    "IndependentValidationTime_s", NaN, ...
    "FirstValidatedMotionTime_s", NaN, ...
    "PlannerReportedTime_s", NaN, "TotalWallTime_s", NaN, ...
    "SelectedPolylineLength_deg", NaN, "SmoothedPathLength_deg", NaN, ...
    "ArrivalTime_s", NaN, "MotionDuration_s", NaN, ...
    "IntegratedSquaredJerk_deg2_s5", NaN, ...
    "MaximumVelocityRatio", NaN, "MaximumAccelerationRatio", NaN, ...
    "MaximumJerkRatio", NaN, "CollisionFree", false, ...
    "KinematicCertificatePassed", false);
end

function record = emptySummaryRecord()
%% Section 0: Header & Readme
% SYNTAX
%   record = emptySummaryRecord()
%**************************************************************************
% PURPOSE
%   - Define one stable per-turn-count summary schema.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - record (scalar struct)
%**************************************************************************
% UNITS
%   - Timing is seconds; other fields are counts.
%**************************************************************************
record = struct( ...
    "TurnCount", 0, "RepeatCount", 0, ...
    "PlannerSuccessCount", 0, "ValidationPassCount", 0, ...
    "Hs3CalledCount", 0, "Hs3ValidationPassCount", 0, ...
    "FallbackCount", 0, "Hs3WallFirst_s", NaN, ...
    "Hs3WallMedian_s", NaN, "Hs3WallMinimum_s", NaN, ...
    "Hs3WallMaximum_s", NaN, "TotalWallFirst_s", NaN, ...
    "TotalWallMedian_s", NaN, "TotalWallMinimum_s", NaN, ...
    "TotalWallMaximum_s", NaN, "ArrivalMedian_s", NaN);
end
