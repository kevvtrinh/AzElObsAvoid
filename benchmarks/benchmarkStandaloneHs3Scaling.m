function report = benchmarkStandaloneHs3Scaling( ...
        turnCounts, benchmarkOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   report = benchmarkStandaloneHs3Scaling()
%   report = benchmarkStandaloneHs3Scaling(turnCounts)
%   report = benchmarkStandaloneHs3Scaling( ...
%       turnCounts, benchmarkOverrides)
%**************************************************************************
% PURPOSE
%   - Measure the public standalone Hermite-Simpson planner on the shared
%     repeated-turn and hairpin scenarios without compact-planner work.
%**************************************************************************
% INPUTS
%   - turnCounts (numeric vector, optional; default [1 5 10 20])
%       Positive integer repeated-turn scales. Pass [] for hairpin only.
%   - benchmarkOverrides (scalar struct, optional; default struct())
%       .IncludeHairpin is logical (default true).
%       .HairpinCount is a positive integer (default 12).
%       .MaximumPlanningTime_s is positive (default 115).
%       .PrintProgress is logical (default true).
%       .RandomSeed is a nonnegative integer (default 325).
%       .PlannerOverrides is a scalar HS3 option struct (default struct()).
%**************************************************************************
% OUTPUTS
%   - report (scalar struct)
%       Frozen controls and one case record per requested scenario. Each
%       record retains wall time, full planner result, independently rerun
%       validation, selected or best-partial route, and solver diagnostics.
%**************************************************************************
% UNITS
%   - Geometry and clearance are degrees; time is seconds; derivatives use
%     deg/s powers. Histories use [azimuth elevation].
%**************************************************************************

%% Section 1: Resolve Benchmark Controls

if nargin < 1
    turnCounts = [1 5 10 20];
end
if nargin < 2 || isempty(benchmarkOverrides)
    benchmarkOverrides = struct();
end
if ~isempty(turnCounts)
    validateattributes(turnCounts, {'numeric'}, ...
        {'real', 'finite', 'vector', 'integer', 'positive'});
end
turnCounts = double(turnCounts(:));
defaults = struct( ...
    "IncludeHairpin", true, ...
    "HairpinCount", 12, ...
    "MaximumPlanningTime_s", 115, ...
    "PrintProgress", true, ...
    "RandomSeed", 325, ...
    "PlannerOverrides", struct());
[controls, unknownNames] = azElInternal.resolveOptions( ...
    defaults, benchmarkOverrides);
if ~isempty(unknownNames)
    warning("benchmarkStandaloneHs3Scaling:UnknownOptions", ...
        "Ignoring unknown fields: %s. No behavior changed.", ...
        strjoin(unknownNames, ", "));
end
controls.IncludeHairpin = azElInternal.normalizeLogicalScalar( ...
    controls.IncludeHairpin, "IncludeHairpin", ...
    "benchmarkStandaloneHs3Scaling:InvalidHairpinControl");
controls.PrintProgress = azElInternal.normalizeLogicalScalar( ...
    controls.PrintProgress, "PrintProgress", ...
    "benchmarkStandaloneHs3Scaling:InvalidPrintControl");
validateattributes(controls.HairpinCount, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'positive'});
validateattributes(controls.MaximumPlanningTime_s, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'positive'});
validateattributes(controls.RandomSeed, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'nonnegative'});
if ~isstruct(controls.PlannerOverrides) || ...
        ~isscalar(controls.PlannerOverrides)
    error("benchmarkStandaloneHs3Scaling:InvalidPlannerOverrides", ...
        "PlannerOverrides must be a scalar struct.");
end
if isempty(turnCounts) && ~controls.IncludeHairpin
    error("benchmarkStandaloneHs3Scaling:NoCases", ...
        "Request at least one turn count or IncludeHairpin=true.");
end

plannerOptions = controls.PlannerOverrides;
plannerOptions.PlannerMethod = "hs3";
plannerOptions.GoalTimeMode = "earliestArrival";
plannerOptions.MaximumPlanningTime_s = controls.MaximumPlanningTime_s;
plannerOptions.RandomSeed = controls.RandomSeed;

%% Section 2: Run Every Requested Case Serially

caseCount = numel(turnCounts) + double(controls.IncludeHairpin);
cases = repmat(emptyCaseRecord(), caseCount, 1);
scenarioConstants = struct();
caseIndex = 0;

for turnIndex = 1:numel(turnCounts)
    turnCount = turnCounts(turnIndex);
    [obstacles, initialState, goalState, limits, scenarioConstants] = ...
        createRepeatedTurnBenchmarkScenario(turnCount);
    caseIndex = caseIndex + 1;
    cases(caseIndex) = runCase( ...
        "repeatedTurn", turnCount, obstacles, initialState, ...
        goalState, limits, plannerOptions);
    if controls.PrintProgress
        printCase(cases(caseIndex));
    end
end

if controls.IncludeHairpin
    [obstacles, initialState, goalState, limits] = ...
        createHairpinBenchmarkScenario(controls.HairpinCount);
    caseIndex = caseIndex + 1;
    cases(caseIndex) = runCase( ...
        "hairpin", controls.HairpinCount, obstacles, initialState, ...
        goalState, limits, plannerOptions);
    if controls.PrintProgress
        printCase(cases(caseIndex));
    end
end

%% Section 3: Assemble Reproducible Evidence

report = struct( ...
    "BenchmarkName", "standaloneHs3Scaling", ...
    "MotionMethod", "hs3", ...
    "Environment", benchmarkEnvironment(), ...
    "Controls", controls, ...
    "PlannerOptions", plannerOptions, ...
    "ScenarioConstants", scenarioConstants, ...
    "TurnCounts", turnCounts, ...
    "Cases", cases);
end

%% Section 4: Local Functions

function record = runCase( ...
        scenarioFamily, scenarioScale, obstacles, initialState, ...
        goalState, limits, plannerOptions)
% Run only the public HS3 path and preserve unsuccessful evidence.
rng(plannerOptions.RandomSeed, "twister");
plannerTimer = tic;
result = planAzElMotion( ...
    obstacles, initialState, goalState, limits, plannerOptions);
plannerWallTime_s = toc(plannerTimer);
enforceStandaloneHs3(result);

validation = validateAzElTrajectory();
validationAttempted = false;
validationWallTime_s = 0;
if result.Success
    validationTimer = tic;
    validation = validateAzElTrajectory(result);
    validationWallTime_s = toc(validationTimer);
    validationAttempted = true;
end
[route_deg, solverDiagnostics] = routeAndSolverEvidence(result);

record = emptyCaseRecord();
record.ScenarioFamily = scenarioFamily;
record.ScenarioScale = scenarioScale;
record.ObstacleCount = numel(obstacles);
record.Route_deg = route_deg;
record.RouteVertexCount = size(route_deg, 1);
record.PlannerWallTime_s = plannerWallTime_s;
record.ValidationWallTime_s = validationWallTime_s;
record.TotalWallTime_s = plannerWallTime_s + validationWallTime_s;
record.Success = result.Success;
record.IndependentValidationAttempted = validationAttempted;
record.IndependentValidationPassed = ...
    validationAttempted && validation.Passed;
record.TerminationReason = result.TerminationReason;
record.SelectedMotionSource = string(result.SelectedMotionSource);
record.SolverDiagnostics = solverDiagnostics;
record.PlannerResult = result;
record.IndependentValidation = validation;
end

function enforceStandaloneHs3(result)
% Treat evidence of method composition as a benchmark contract failure.
if isfield(result, "CompositionDiagnostics")
    error("benchmarkStandaloneHs3Scaling:CompositionDetected", ...
        "Standalone HS3 results must not contain CompositionDiagnostics.");
end
if result.Options.PlannerMethod ~= "hs3" || ...
        result.SearchDiagnostics.PlannerMethod ~= "hs3"
    error("benchmarkStandaloneHs3Scaling:WrongPlannerMethod", ...
        "The public planner did not echo PlannerMethod='hs3'.");
end
if result.Success && result.SelectedMotionSource ~= "hs3"
    error("benchmarkStandaloneHs3Scaling:WrongMotionSource", ...
        "A successful standalone result must select HS3 motion.");
end
end

function [route_deg, solverDiagnostics] = routeAndSolverEvidence(result)
% Select the accepted route or the best attempted route on failure.
route_deg = zeros(0, 2);
solverDiagnostics = struct();
seedIndex = 0;
if result.Success
    seedIndex = result.SelectedSeedIndex;
elseif isfield(result.SearchDiagnostics, "BestPartialSeedIndex")
    seedIndex = result.SearchDiagnostics.BestPartialSeedIndex;
end
hasSeedIndex = isnumeric(seedIndex) && isscalar(seedIndex) && ...
    isfinite(seedIndex) && seedIndex >= 1 && seedIndex == floor(seedIndex);
if ~hasSeedIndex && isfield(result, "SeedSummaries")
    attemptedIndices = find([result.SeedSummaries.Hs3Attempted], 1);
    if ~isempty(attemptedIndices)
        seedIndex = attemptedIndices;
        hasSeedIndex = true;
    end
end
if ~hasSeedIndex || ~isfield(result, "Seeds") || ...
        seedIndex > numel(result.Seeds)
    return;
end
route_deg = result.Seeds(seedIndex).position_deg;
if isfield(result, "SeedSummaries") && ...
        seedIndex <= numel(result.SeedSummaries)
    solverDiagnostics = ...
        result.SeedSummaries(seedIndex).SolverDiagnostics;
end
end

function environment = benchmarkEnvironment()
% Record runtime facts without starting a parallel pool.
poolState = "notRunning";
workerCount = 0;
if exist("gcp", "file") == 2
    pool = gcp("nocreate");
    if ~isempty(pool)
        poolState = "running";
        workerCount = pool.NumWorkers;
    end
end
environment = struct( ...
    "MATLABVersion", string(version), ...
    "Computer", string(computer), ...
    "ReportedCoreCount", feature("numcores"), ...
    "ParallelPoolState", poolState, ...
    "WorkerCount", workerCount, ...
    "RandomGenerator", "twister", ...
    "FigureVisibility", "off");
end

function record = emptyCaseRecord()
% Define stable fields while retaining full nested evidence.
record = struct( ...
    "ScenarioFamily", "notRun", ...
    "ScenarioScale", NaN, ...
    "ObstacleCount", NaN, ...
    "Route_deg", zeros(0, 2), ...
    "RouteVertexCount", 0, ...
    "PlannerWallTime_s", NaN, ...
    "ValidationWallTime_s", NaN, ...
    "TotalWallTime_s", NaN, ...
    "Success", false, ...
    "IndependentValidationAttempted", false, ...
    "IndependentValidationPassed", false, ...
    "TerminationReason", "notRun", ...
    "SelectedMotionSource", "none", ...
    "SolverDiagnostics", struct(), ...
    "PlannerResult", struct(), ...
    "IndependentValidation", struct());
end

function printCase(record)
% Print one complete status row without hiding failures.
fprintf( ...
    "scenario=%s scale=%d success=%d validation=%d route=%d " + ...
    "planner_s=%.12g validation_s=%.12g total_s=%.12g reason=%s\n", ...
    record.ScenarioFamily, record.ScenarioScale, record.Success, ...
    record.IndependentValidationPassed, record.RouteVertexCount, ...
    record.PlannerWallTime_s, record.ValidationWallTime_s, ...
    record.TotalWallTime_s, record.TerminationReason);
end
