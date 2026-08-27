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
%   - Measure the public Hermite-Simpson planner on repeated-turn and
%     hairpin scenarios.
%**************************************************************************
% INPUTS
%   - turnCounts (numeric vector, optional; default [1 5 10 20])
%       Positive integer repeated-turn scales. Pass [] for hairpin only.
%   - benchmarkOverrides (scalar struct, optional; default struct())
%       .IncludeHairpin is logical (default true).
%       .HairpinCount is a positive integer (default 12).
%       .PrintProgress is logical (default true).
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

% Define the segment counts, repeat count, and solver limits once. These values
% are part of the reported environment. Use them when comparing two runs.

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
    "PrintProgress", true, ...
    "PlannerOverrides", struct());
[controls, unknownNames] = obstacleAvoidance.input.resolveOptions( ...
    defaults, benchmarkOverrides);
if ~isempty(unknownNames)
    warning("benchmarkStandaloneHs3Scaling:UnknownOptions", ...
        "Ignoring unknown fields: %s. No behavior changed.", ...
        strjoin(unknownNames, ", "));
end
controls.IncludeHairpin = obstacleAvoidance.input.normalizeLogicalScalar( ...
    controls.IncludeHairpin, "IncludeHairpin", ...
    "benchmarkStandaloneHs3Scaling:InvalidHairpinControl");
controls.PrintProgress = obstacleAvoidance.input.normalizeLogicalScalar( ...
    controls.PrintProgress, "PrintProgress", ...
    "benchmarkStandaloneHs3Scaling:InvalidPrintControl");
validateattributes(controls.HairpinCount, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'positive'});
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
plannerOptions.GoalTimeMode = "earliestArrival";

%% Section 2: Run Every Requested Case Serially

% Change one requested scale at a time. Run repeats serially so simultaneous
% solver work does not distort elapsed time. Keep failures in the measurements.

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

% Report individual runs and aggregate values. Inspect individual termination
% reasons before using a median because a fast failed run is not an improvement.

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
plannerTimer = tic;
result = obstacleAvoidance.planTrajectory( ...
    obstacles, initialState, goalState, limits, plannerOptions);
plannerWallTime_s = toc(plannerTimer);
enforcePlannerContract(result);

validation = obstacleAvoidance.validateTrajectory();
validationAttempted = false;
validationWallTime_s = 0;
if result.Success
    validationTimer = tic;
    validation = obstacleAvoidance.validateTrajectory(result);
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
record.SolverDiagnostics = solverDiagnostics;
record.PlannerResult = result;
record.IndependentValidation = validation;
end

function enforcePlannerContract(result)
% Reject obsolete dispatch and alternate-motion provenance fields.
if isfield(result, "SelectedMotionSource") || ...
        isfield(result.Options, "PlannerMethod")
    error("benchmarkStandaloneHs3Scaling:ObsoletePlannerField", ...
        "The public result contains an obsolete planner-selection field.");
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
    "SolverDiagnostics", struct(), ...
    "PlannerResult", struct(), ...
    "IndependentValidation", struct());
end

function printCase(record)
% Print one complete status row without hiding failures.
fprintf( ...
    "scenario=%s scale=%d success=%s validation=%s route=%d " + ...
    "planner_s=%.12g validation_s=%.12g total_s=%.12g reason=%s\n", ...
    record.ScenarioFamily, record.ScenarioScale, logicalText(record.Success), ...
    logicalText(record.IndependentValidationPassed), record.RouteVertexCount, ...
    record.PlannerWallTime_s, record.ValidationWallTime_s, ...
    record.TotalWallTime_s, record.TerminationReason);
end

function text = logicalText(value)
% Render scalar logical benchmark status as true or false.
if logical(value)
    text = "true";
else
    text = "false";
end
end

function [obstacles, initialState, goalState, limits] = ...
        createHairpinBenchmarkScenario(hairpinCount)
% Build the benchmark's deterministic alternating-end maze.

wallSpacing_deg = 4;
wallHalfThickness_deg = 0.35;
workspaceHalfWidth_deg = 10;
openingInnerEdgeMagnitude_deg = 6.5;
safetyMargin_deg = 0.15;
bottomElevation_deg = 0;
topElevation_deg = wallSpacing_deg * (hairpinCount + 1);
obstacleCells = cell(hairpinCount, 1);

% Alternating openings force each input-derived route to reverse laterally.
for wallIndex = 1:hairpinCount
    wallElevation_deg = wallSpacing_deg * wallIndex;
    if mod(wallIndex, 2) == 1
        wallAzimuthInterval_deg = [ ...
            -workspaceHalfWidth_deg, openingInnerEdgeMagnitude_deg];
    else
        wallAzimuthInterval_deg = [ ...
            -openingInnerEdgeMagnitude_deg, workspaceHalfWidth_deg];
    end
    wallAzimuth_deg = wallAzimuthInterval_deg([1 2 2 1]).';
    wallElevationBoundary_deg = wallElevation_deg + ...
        wallHalfThickness_deg * [-1 -1 1 1].';
    obstacleCells{wallIndex} = obstacleAvoidance.obstacles.createObstacle( ...
        "alternating wall " + wallIndex, 0, wallAzimuth_deg, ...
        wallElevationBoundary_deg, safetyMargin_deg);
end
obstacles = [obstacleCells{:}].';
obstacles = obstacleAvoidance.obstacles.prepareDynamic(obstacles);

initialState = struct( ...
    "time_s", 0, ...
    "position_deg", [0, 0.5 * wallSpacing_deg], ...
    "velocity_deg_s", [0 0], ...
    "acceleration_deg_s2", [0 0]);
goalState = struct( ...
    "time_s", 120 * (hairpinCount + 1), ...
    "position_deg", [0, topElevation_deg - 0.5 * wallSpacing_deg], ...
    "velocity_deg_s", [0 0], ...
    "acceleration_deg_s2", [0 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1], ...
    "maxJerk_deg_s3", [2 2], ...
    "azimuthInterval_deg", ...
    [-workspaceHalfWidth_deg, workspaceHalfWidth_deg], ...
    "elevationInterval_deg", [bottomElevation_deg, topElevation_deg]);
end
