function report = benchmarkCorridorConstrainedQuintic( turnCounts, benchmarkOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   report = benchmarkCorridorConstrainedQuintic()
%   report = benchmarkCorridorConstrainedQuintic(turnCounts)
%   report = benchmarkCorridorConstrainedQuintic( ...
%       turnCounts, benchmarkOverrides)
%**************************************************************************
% PURPOSE
%   - Measure deterministic corridor-constrained quintic scaling on the
%     frozen repeated-turn family using only the corridor planner.
%**************************************************************************
% INPUTS
%   - turnCounts (numeric vector, optional; default [1 2 5 10 20])
%       Positive integer alternating-barrier counts, executed serially.
%   - benchmarkOverrides (scalar struct, optional; default struct())
%       .RepeatCount is a positive integer (default 3).
%       .MaximumRouteVertexCount is an integer >= 2 (default 22).
%       .PrintProgress is logical (default true).
%       .RandomSeed is a nonnegative integer (default 325).
%       .PrototypeOptions is a scalar partial option struct (default
%       struct()). RouteVertexCount is owned by MaximumRouteVertexCount.
%**************************************************************************
% OUTPUTS
%   - report (scalar struct)
%       Environment, frozen controls, every run, summaries, and full motion
%       results. Failed candidates remain present with their diagnostics.
%**************************************************************************
% UNITS
%   - Geometry and clearance are degrees; time is seconds; derivative ratios
%     are dimensionless. Histories use [azimuth elevation].
%**************************************************************************

%% Section 1: Resolve Benchmark Controls

if nargin < 1 || isempty(turnCounts)
    turnCounts = [1 2 5 10 20];
end
if nargin < 2 || isempty(benchmarkOverrides)
    benchmarkOverrides = struct();
end
validateattributes(turnCounts, {'numeric'}, {'real', 'finite', 'vector', 'integer', 'positive'});
turnCounts = double(turnCounts(:));
defaults = struct( ...
    "RepeatCount", 3, ...
    "MaximumRouteVertexCount", 22, "PrintProgress", true, "RandomSeed", 325, "PrototypeOptions", struct());
[controls, unknownNames] = azElInternal.resolveOptions( ...
    defaults, benchmarkOverrides);
if ~isempty(unknownNames)
    warning("benchmarkCorridorConstrainedQuintic:UnknownOptions", ...
        "Ignoring unknown fields: %s. No behavior changed.", strjoin(unknownNames, ", "));
end
validateattributes(controls.RepeatCount, {'numeric'}, {'real', 'finite', 'scalar', 'integer', 'positive'});
validateattributes(controls.MaximumRouteVertexCount, {'numeric'}, {'real', 'finite', 'scalar', 'integer', '>=', 2});
validateattributes(controls.RandomSeed, {'numeric'}, {'real', 'finite', 'scalar', 'integer', 'nonnegative'});
controls.PrintProgress = azElInternal.normalizeLogicalScalar( ...
    controls.PrintProgress, "PrintProgress", "benchmarkCorridorConstrainedQuintic:InvalidPrintControl");
if ~isstruct(controls.PrototypeOptions) || ~isscalar(controls.PrototypeOptions)
    error("benchmarkCorridorConstrainedQuintic:InvalidPrototypeOptions", ...
        "PrototypeOptions must be a scalar partial option struct.");
end
if isfield(controls.PrototypeOptions, "RouteVertexCount")
    error("benchmarkCorridorConstrainedQuintic:ConflictingRouteCount", ...
        "Set MaximumRouteVertexCount on the benchmark rather than " + "RouteVertexCount inside PrototypeOptions.");
end
scenarioConstants = repeatedTurnConstants();
plannerOptions = planAzElMotion();
plannerOptions.GoalTimeMode = "earliestArrival";
plannerOptions.MaximumSeedCount = 3;
plannerOptions.RandomSeed = controls.RandomSeed;

%% Section 2: Run Every Case And Repetition Serially

runCount = numel(turnCounts) * controls.RepeatCount;
runs = repmat(emptyRunRecord(), runCount, 1);
motionResults = cell(runCount, 1);
runIndex = 0;

% Build and seed each requested geometry scale once so every repetition uses the same route.
for turnCountIndex = 1:numel(turnCounts)
    turnCount = turnCounts(turnCountIndex);
    [obstacles, initialState, goalState, limits] = createRepeatedTurnBenchmarkScenario( turnCount, scenarioConstants);
    rng(controls.RandomSeed, "twister");
    seedTimer = tic;
    [seeds, seedDiagnostics] = azElPlannerMethods.corridor.internal.search.generateTopologySeeds( ...
        obstacles, initialState, goalState, limits, plannerOptions);
    seedElapsedTime_s = toc(seedTimer);
    visibilitySeedIndex = find( [seeds.Source] == "visibilityGraph", 1, "first");
    if isempty(visibilitySeedIndex)
        error("benchmarkCorridorConstrainedQuintic:MissingVisibilitySeed", ...
            "Turn count %d produced no visibility-graph seed.", turnCount);
    end
    route_deg = seeds(visibilitySeedIndex).position_deg;
    routeVertexCount = min( controls.MaximumRouteVertexCount, size(route_deg, 1));
    prototypeOptions = controls.PrototypeOptions;
    prototypeOptions.RouteVertexCount = routeVertexCount;

    % Repeat the frozen solve and validation to measure runtime variation without changing topology.
    for repeatIndex = 1:controls.RepeatCount
        runIndex = runIndex + 1;
        candidateTimer = tic;
        motion = azElPlannerMethods.corridor.internal.motion.solveCorridorQuintic( ...
            obstacles, initialState, goalState, limits, route_deg, prototypeOptions);
        candidateWallTime_s = toc(candidateTimer);
        validationTimer = tic;
        validation = azElPlannerMethods.corridor.validateTrajectory( motion, obstacles, initialState, goalState, limits, plannerOptions);
        validationElapsedTime_s = toc(validationTimer);
        motionResults{runIndex} = motion;
        runs(runIndex) = collectRunRecord( ...
            turnCount, repeatIndex, controls.RandomSeed, route_deg, ...
            routeVertexCount, seedDiagnostics, seedElapsedTime_s, ...
            motion, candidateWallTime_s, validation, validationElapsedTime_s, limits);
        if controls.PrintProgress
            printRunRecord(runs(runIndex));
        end
    end
end

%% Section 3: Summarize Repetitions Without Hiding Extremes

runTable = struct2table(runs);
summaryTable = summarizeRuns(runTable, turnCounts);

%% Section 4: Assemble Reproducible Evidence

report = struct( ...
    "BenchmarkName", "corridorConstrainedQuinticScaling", ...
    "Environment", benchmarkEnvironment(), ...
    "Controls", controls, ...
    "PlannerOptions", plannerOptions, ...
    "ScenarioConstants", scenarioConstants, ...
    "TurnCounts", turnCounts, "Runs", runTable, "Summary", summaryTable, "MotionResults", {motionResults});
end

function constants = repeatedTurnConstants()
% Freeze the maintained repeated-turn geometry and physical limits.
% SYNTAX
%   constants = repeatedTurnConstants()
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - constants (scalar scenario-constant structure)
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use seconds.
%**************************************************************************
constants = struct( ...
    "barrierSpacing_deg", 4, ...
    "barrierHalfWidth_deg", 0.7, ...
    "barrierCenterMagnitude_deg", 2.5, ...
    "barrierHalfHeight_deg", 2.5, ...
    "safetyMargin_deg", 0.1, ...
    "goalTimePerStage_s", 5.5, ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1], "maxJerk_deg_s3", [2 2], "elevationInterval_deg", [-5 5]);
end

function environment = benchmarkEnvironment()
% Record source, MATLAB, toolbox, CPU, and parallel-pool facts.
% SYNTAX
%   environment = benchmarkEnvironment()
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - environment (scalar reproducibility record)
%**************************************************************************
% UNITS
%   - CPU and worker counts are dimensionless.
%**************************************************************************
[commitStatus, commitText] = system("git rev-parse HEAD");
[branchStatus, branchText] = system("git branch --show-current");
optimizationToolbox = ver("optim");
if isempty(optimizationToolbox)
    optimizationVersion = "unavailable";
else
    optimizationVersion = string(optimizationToolbox.Version);
end
parallelPoolState = "notRunning";
workerCount = 0;
if exist("gcp", "file") == 2
    pool = gcp("nocreate");
    if ~isempty(pool)
        parallelPoolState = "running";
        workerCount = pool.NumWorkers;
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
    "WorkerCount", workerCount, "FigureVisibility", "off", "AnimationEnabled", false, "Verbose", false);
end

function value = commandText(status, output)
% Normalize one read-only source-control query.
% SYNTAX
%   value = commandText(status, output)
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

function record = collectRunRecord( ...
        turnCount, repeatIndex, randomSeed, route_deg, ...
        requestedRouteVertexCount, seedDiagnostics, seedElapsedTime_s, ...
        motion, candidateWallTime_s, validation, validationElapsedTime_s, limits)
% Preserve timing, feasibility, quality, and certificate evidence.
% SYNTAX
%   record = collectRunRecord(...)
%**************************************************************************
% INPUTS
%   - Inputs are the request, stage timings, motion, and validation records.
%**************************************************************************
% OUTPUTS
%   - record (scalar stable benchmark row)
%**************************************************************************
% UNITS
%   - Geometry is degrees; time is seconds; ratios are dimensionless.
%**************************************************************************
record = emptyRunRecord();
record.TurnCount = turnCount;
record.RepeatIndex = repeatIndex;
record.RandomSeed = randomSeed;
record.ObstacleCount = turnCount;
record.RouteVertexCount = size(route_deg, 1);
record.RequestedRouteVertexCount = requestedRouteVertexCount;
record.ReducedRouteVertexCount = size(motion.ReducedRoute_deg, 1);
record.DecisionVariableCount = motion.OptimizerDiagnostics.DecisionCount;
record.CorridorRecordCount = numel(motion.SeedCorridor);
record.TopologyGeneratedSeedCount = seedDiagnostics.GeneratedSeedCount;
record.SeedGenerationTime_s = seedElapsedTime_s;
record.CandidateWallTime_s = candidateWallTime_s;
record.CandidateSolveTime_s = motion.OptimizerDiagnostics.SolveTime_s;
record.IndependentValidationTime_s = validationElapsedTime_s;
record.TotalWallTime_s = seedElapsedTime_s + candidateWallTime_s + validationElapsedTime_s;
record.Success = motion.Success;
record.IndependentValidationPassed = validation.Passed;
record.CorridorCertified = motion.OptimizerDiagnostics.CorridorCertified;
record.EnvelopeContainsObstacles = motion.OptimizerDiagnostics.EnvelopeContainsObstacles;
record.FallbackInvoked = false;
record.SelectedPolylineLength_deg = sum(vecnorm(diff(route_deg), 2, 2));
if ~isempty(motion.position_deg)
    record.SmoothedPathLength_deg = sum( vecnorm(diff(motion.position_deg), 2, 2));
end
record.MotionDuration_s = motion.MotionDuration_s;
record.MinimumClearance_deg = validation.MinimumClearance_deg;
record.CollisionFree = validation.CollisionFree;
record.MaximumVelocityRatio = max( validation.PeakVelocity_deg_s ./ limits.maxVelocity_deg_s);
record.MaximumAccelerationRatio = max( validation.PeakAcceleration_deg_s2 ./ limits.maxAcceleration_deg_s2);
record.MaximumJerkRatio = max( validation.PeakJerk_deg_s3 ./ limits.maxJerk_deg_s3);
record.TerminationReason = motion.TerminationReason;
end

function printRunRecord(record)
% Print one complete outcome directly in the MATLAB log.
% SYNTAX
%   printRunRecord(record)
%**************************************************************************
% INPUTS
%   - record (scalar benchmark row)
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Geometry is degrees; time is seconds.
%**************************************************************************
fprintf( ...
    "turns=%d repeat=%d success=%d validation=%d certificate=%d " + ...
    "route=%d->%d decisions=%d records=%d duration_s=%.12g " + ...
    "clearance_deg=%.12g candidate_s=%.12g total_s=%.12g reason=%s\n", ...
    record.TurnCount, record.RepeatIndex, record.Success, ...
    record.IndependentValidationPassed, record.CorridorCertified, ...
    record.RouteVertexCount, record.ReducedRouteVertexCount, ...
    record.DecisionVariableCount, record.CorridorRecordCount, ...
    record.MotionDuration_s, record.MinimumClearance_deg, ...
    record.CandidateWallTime_s, record.TotalWallTime_s, record.TerminationReason);
end

function summaryTable = summarizeRuns(runTable, turnCounts)
% Report first, median, minimum, and maximum timing per input scale.
% SYNTAX
%   summaryTable = summarizeRuns(runTable, turnCounts)
%**************************************************************************
% INPUTS
%   - runTable (table of benchmark rows)
%   - turnCounts (numeric column vector)
%**************************************************************************
% OUTPUTS
%   - summaryTable (one row per requested turn count)
%**************************************************************************
% UNITS
%   - Time is seconds; counts are dimensionless.
%**************************************************************************
summaries = repmat(emptySummaryRecord(), numel(turnCounts), 1);

% Summarize each turn-count group separately so the fastest and slowest repetitions remain visible.
for turnIndex = 1:numel(turnCounts)
    rows = runTable(runTable.TurnCount == turnCounts(turnIndex), :);
    candidateTimes_s = rows.CandidateWallTime_s;
    totalTimes_s = rows.TotalWallTime_s;
    summaries(turnIndex).TurnCount = turnCounts(turnIndex);
    summaries(turnIndex).RepeatCount = height(rows);
    summaries(turnIndex).SuccessCount = sum(rows.Success);
    summaries(turnIndex).ValidationPassCount = sum(rows.IndependentValidationPassed);
    summaries(turnIndex).FirstCandidateWallTime_s = candidateTimes_s(1);
    summaries(turnIndex).MedianCandidateWallTime_s = median(candidateTimes_s);
    summaries(turnIndex).MinimumCandidateWallTime_s = min(candidateTimes_s);
    summaries(turnIndex).MaximumCandidateWallTime_s = max(candidateTimes_s);
    summaries(turnIndex).MedianTotalWallTime_s = median(totalTimes_s);
    summaries(turnIndex).MotionDuration_s = rows.MotionDuration_s(1);
    summaries(turnIndex).MinimumClearance_deg = rows.MinimumClearance_deg(1);
    summaries(turnIndex).TerminationReason = rows.TerminationReason(1);
end
summaryTable = struct2table(summaries);
end

function record = emptyRunRecord()
% Define the stable benchmark-row field order and empty values.
% SYNTAX
%   record = emptyRunRecord()
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - record (scalar empty benchmark row)
%**************************************************************************
% UNITS
%   - Numeric unavailable values are NaN.
%**************************************************************************
record = struct( ...
    "TurnCount", NaN, ...
    "RepeatIndex", NaN, ...
    "RandomSeed", NaN, ...
    "ObstacleCount", NaN, ...
    "RouteVertexCount", NaN, ...
    "RequestedRouteVertexCount", NaN, ...
    "ReducedRouteVertexCount", NaN, ...
    "DecisionVariableCount", NaN, ...
    "CorridorRecordCount", NaN, ...
    "TopologyGeneratedSeedCount", NaN, ...
    "SeedGenerationTime_s", NaN, ...
    "CandidateWallTime_s", NaN, ...
    "CandidateSolveTime_s", NaN, ...
    "IndependentValidationTime_s", NaN, ...
    "TotalWallTime_s", NaN, ...
    "Success", false, ...
    "IndependentValidationPassed", false, ...
    "CorridorCertified", false, ...
    "EnvelopeContainsObstacles", false, ...
    "FallbackInvoked", false, ...
    "SelectedPolylineLength_deg", NaN, ...
    "SmoothedPathLength_deg", NaN, ...
    "MotionDuration_s", NaN, ...
    "MinimumClearance_deg", NaN, ...
    "CollisionFree", false, ...
    "MaximumVelocityRatio", NaN, ...
    "MaximumAccelerationRatio", NaN, "MaximumJerkRatio", NaN, "TerminationReason", "notRun");
end

function record = emptySummaryRecord()
% Define the stable per-scale summary field order.
% SYNTAX
%   record = emptySummaryRecord()
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - record (scalar empty summary row)
%**************************************************************************
% UNITS
%   - Numeric unavailable values are NaN.
%**************************************************************************
record = struct( ...
    "TurnCount", NaN, ...
    "RepeatCount", NaN, ...
    "SuccessCount", 0, ...
    "ValidationPassCount", 0, ...
    "FirstCandidateWallTime_s", NaN, ...
    "MedianCandidateWallTime_s", NaN, ...
    "MinimumCandidateWallTime_s", NaN, ...
    "MaximumCandidateWallTime_s", NaN, ...
    "MedianTotalWallTime_s", NaN, "MotionDuration_s", NaN, "MinimumClearance_deg", NaN, "TerminationReason", "notRun");
end
