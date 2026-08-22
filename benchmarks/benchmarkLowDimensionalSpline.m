function report = benchmarkLowDimensionalSpline( ...
        turnCounts, benchmarkOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   report = benchmarkLowDimensionalSpline()
%   report = benchmarkLowDimensionalSpline(turnCounts)
%   report = benchmarkLowDimensionalSpline(turnCounts, benchmarkOverrides)
%**************************************************************************
% PURPOSE
%   - Measure bounded quintic B-spline motion construction on the same
%     parameterized repeated-turn family used by the frozen HS3 baseline.
%**************************************************************************
% INPUTS
%   - turnCounts (numeric vector, optional; default [1 2 5])
%       Positive integer alternating-barrier counts, executed serially.
%   - benchmarkOverrides (scalar struct, optional; default struct())
%       .RandomSeed is a nonnegative integer (default 325).
%       .PrintProgress is logical (default true).
%       .OptimizerOptions is a scalar partial option struct (default struct()).
%**************************************************************************
% OUTPUTS
%   - report (scalar struct)
%       Environment, controls, scenario constants, run table, and full motion
%       results. Expected no-motion outcomes remain ordinary table rows.
%**************************************************************************
% UNITS
%   - Geometry is degrees; time and wall time are seconds; derivative ratios
%     are dimensionless.
%**************************************************************************

%% Section 1: Resolve Benchmark Controls

if nargin < 1 || isempty(turnCounts)
    turnCounts = [1 2 5];
end
if nargin < 2 || isempty(benchmarkOverrides)
    benchmarkOverrides = struct();
end
validateattributes(turnCounts, {'numeric'}, ...
    {'real', 'finite', 'vector', 'integer', 'positive'});
turnCounts = double(turnCounts(:));
defaults = struct( ...
    "RandomSeed", 325, ...
    "PrintProgress", true, ...
    "OptimizerOptions", struct());
[controls, unknownNames] = azElInternal.resolveOptions( ...
    defaults, benchmarkOverrides);
if ~isempty(unknownNames)
    warning("benchmarkLowDimensionalSpline:UnknownOptions", ...
        "Ignoring unknown fields: %s. No behavior changed.", ...
        strjoin(unknownNames, ", "));
end
validateattributes(controls.RandomSeed, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'nonnegative'});
controls.PrintProgress = azElInternal.normalizeLogicalScalar( ...
    controls.PrintProgress, "PrintProgress", ...
    "benchmarkLowDimensionalSpline:InvalidPrintControl");
if ~isstruct(controls.OptimizerOptions) || ...
        ~isscalar(controls.OptimizerOptions)
    error("benchmarkLowDimensionalSpline:InvalidOptimizerOptions", ...
        "OptimizerOptions must be a scalar partial option struct.");
end
constants = scenarioConstants();
plannerOptions = planAzElMotion();
plannerOptions.GoalTimeMode = "earliestArrival";
plannerOptions.MaximumSeedCount = 3;
plannerOptions.RandomSeed = controls.RandomSeed;

%% Section 2: Execute Identical Seed And Motion Requests Serially

runRecords = repmat(emptyRunRecord(), numel(turnCounts), 1);
motionResults = cell(numel(turnCounts), 1);
for turnIndex = 1:numel(turnCounts)
    turnCount = turnCounts(turnIndex);
    [obstacles, initialState, goalState, limits] = ...
        createRepeatedTurnBenchmarkScenario(turnCount, constants);
    rng(controls.RandomSeed, "twister");
    seedTimer = tic;
    [seeds, seedDiagnostics] = ...
        azElInternal.generateAzElTopologySeeds( ...
        obstacles, initialState, goalState, limits, plannerOptions);
    seedElapsedTime_s = toc(seedTimer);
    visibilitySeedIndex = find( ...
        [seeds.Source] == "visibilityGraph", 1, "first");
    if isempty(visibilitySeedIndex)
        error("benchmarkLowDimensionalSpline:MissingVisibilitySeed", ...
            "Turn count %d produced no visibility-graph seed.", turnCount);
    end
    route_deg = seeds(visibilitySeedIndex).position_deg;
    optimizerTimer = tic;
    motion = optimizeQuinticBsplinePrototype( ...
        obstacles, initialState, goalState, limits, route_deg, ...
        controls.OptimizerOptions);
    optimizerWallTime_s = toc(optimizerTimer);
    validationTimer = tic;
    independentValidation = validateAzElTrajectory( ...
        motion, obstacles, initialState, goalState, limits, ...
        plannerOptions);
    validationElapsedTime_s = toc(validationTimer);
    motionResults{turnIndex} = motion;
    runRecords(turnIndex) = collectRunRecord( ...
        turnCount, controls.RandomSeed, route_deg, seedDiagnostics, ...
        motion, independentValidation, limits, ...
        goalState.time_s - initialState.time_s, seedElapsedTime_s, ...
        optimizerWallTime_s, validationElapsedTime_s);
    if controls.PrintProgress
        printRunRecord(runRecords(turnIndex));
    end
end

%% Section 3: Assemble Reproducible Evidence

[commitStatus, commitText] = system("git rev-parse HEAD");
[branchStatus, branchText] = system("git branch --show-current");
if commitStatus ~= 0
    commitText = "unavailable";
end
if branchStatus ~= 0
    branchText = "unavailable";
end
report = struct( ...
    "BenchmarkName", "boundedLowDimensionalQuinticSpline", ...
    "Environment", struct( ...
    "Commit", strip(string(commitText)), ...
    "Branch", strip(string(branchText)), ...
    "MATLABVersion", string(version), ...
    "Computer", string(computer), ...
    "ReportedCoreCount", feature("numcores")), ...
    "Controls", controls, ...
    "ScenarioConstants", constants, ...
    "TurnCounts", turnCounts, ...
    "Runs", struct2table(runRecords), ...
    "MotionResults", {motionResults});
end

%% Section 4: Local Functions

function record = collectRunRecord( ...
        turnCount, randomSeed, route_deg, seedDiagnostics, motion, ...
        validation, limits, goalHorizon_s, seedElapsedTime_s, ...
        optimizerWallTime_s, ...
        validationElapsedTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   record = collectRunRecord( ...
%       turnCount, randomSeed, route_deg, seedDiagnostics, motion, ...
%       validation, limits, goalHorizon_s, seedElapsedTime_s, ...
%       optimizerWallTime_s, ...
%       validationElapsedTime_s)
%**************************************************************************
% PURPOSE
%   - Preserve comparable timing, quality, and exact-validation evidence.
%**************************************************************************
% INPUTS
%   - turnCount, randomSeed (integer scalars)
%   - route_deg, seedDiagnostics, motion, validation, limits (case records)
%   - goalHorizon_s (positive latest-arrival duration)
%   - seedElapsedTime_s, optimizerWallTime_s, validationElapsedTime_s
%       Nonnegative measured wall times.
%**************************************************************************
% OUTPUTS
%   - record (scalar stable run record)
%**************************************************************************
% UNITS
%   - Length is degrees; time is seconds; ratios are dimensionless.
%**************************************************************************
record = emptyRunRecord();
record.TurnCount = turnCount;
record.RandomSeed = randomSeed;
record.ObstacleCount = turnCount;
record.GoalHorizon_s = goalHorizon_s;
record.TopologyGeneratedSeedCount = seedDiagnostics.GeneratedSeedCount;
record.RouteVertexCount = size(route_deg, 1);
record.ReducedRouteVertexCount = ...
    motion.OptimizerDiagnostics.ReducedRouteVertexCount;
record.DecisionVariableCount = ...
    motion.OptimizerDiagnostics.DecisionVariableCount;
record.FunctionEvaluationCount = ...
    motion.OptimizerDiagnostics.EvaluationCount;
record.SeedGenerationTime_s = seedElapsedTime_s;
record.OptimizerWallTime_s = optimizerWallTime_s;
record.IndependentValidationTime_s = validationElapsedTime_s;
record.TotalWallTime_s = seedElapsedTime_s + optimizerWallTime_s + ...
    validationElapsedTime_s;
record.Success = motion.Success;
record.IndependentValidationPassed = validation.Passed;
record.SelectedPolylineLength_deg = sum(vecnorm(diff(route_deg), 2, 2));
if ~isempty(motion.position_deg)
    record.SmoothedPathLength_deg = sum( ...
        vecnorm(diff(motion.position_deg), 2, 2));
end
record.MotionDuration_s = motion.MotionDuration_s;
record.IntegratedSquaredJerk_deg2_s5 = ...
    motion.IntegratedSquaredJerk_deg2_s5;
record.MaximumVelocityRatio = max( ...
    validation.PeakVelocity_deg_s ./ limits.maxVelocity_deg_s);
record.MaximumAccelerationRatio = max( ...
    validation.PeakAcceleration_deg_s2 ./ ...
    limits.maxAcceleration_deg_s2);
record.MaximumJerkRatio = max( ...
    validation.PeakJerk_deg_s3 ./ limits.maxJerk_deg_s3);
record.MinimumClearance_deg = validation.MinimumClearance_deg;
record.CollisionFree = validation.CollisionFree;
record.KinematicCertificatePassed = validation.VelocityWithinLimits && ...
    validation.AccelerationWithinLimits && validation.JerkWithinLimits && ...
    validation.DynamicsConsistent;
record.TerminationReason = motion.TerminationReason;
end

function printRunRecord(record)
%% Section 0: Header & Readme
% SYNTAX
%   printRunRecord(record)
%**************************************************************************
% PURPOSE
%   - Print one complete benchmark outcome directly in the MATLAB log.
%**************************************************************************
% INPUTS
%   - record (scalar benchmark run record)
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Length and clearance are degrees; time is seconds.
%**************************************************************************
fprintf( ...
    "turns=%d success=%d validation=%d route=%d->%d decisions=%d " + ...
    "evaluations=%d duration_s=%.12g clearance_deg=%.12g " + ...
    "optimizer_s=%.12g total_s=%.12g reason=%s\n", ...
    record.TurnCount, record.Success, record.IndependentValidationPassed, ...
    record.RouteVertexCount, record.ReducedRouteVertexCount, ...
    record.DecisionVariableCount, record.FunctionEvaluationCount, ...
    record.MotionDuration_s, record.MinimumClearance_deg, ...
    record.OptimizerWallTime_s, record.TotalWallTime_s, ...
    record.TerminationReason);
end

function record = emptyRunRecord()
%% Section 0: Header & Readme
% SYNTAX
%   record = emptyRunRecord()
%**************************************************************************
% PURPOSE
%   - Define the stable success-or-failure benchmark row schema.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - record (scalar empty run record)
%**************************************************************************
% UNITS
%   - Length and clearance are degrees; time is seconds; ratios dimensionless.
%**************************************************************************
record = struct( ...
    "TurnCount", 0, "RandomSeed", 0, "ObstacleCount", 0, ...
    "GoalHorizon_s", NaN, "TopologyGeneratedSeedCount", 0, ...
    "RouteVertexCount", 0, "ReducedRouteVertexCount", 0, ...
    "DecisionVariableCount", 0, "FunctionEvaluationCount", 0, ...
    "SeedGenerationTime_s", NaN, "OptimizerWallTime_s", NaN, ...
    "IndependentValidationTime_s", NaN, "TotalWallTime_s", NaN, ...
    "Success", false, "IndependentValidationPassed", false, ...
    "SelectedPolylineLength_deg", NaN, "SmoothedPathLength_deg", NaN, ...
    "MotionDuration_s", NaN, "IntegratedSquaredJerk_deg2_s5", NaN, ...
    "MaximumVelocityRatio", NaN, "MaximumAccelerationRatio", NaN, ...
    "MaximumJerkRatio", NaN, "MinimumClearance_deg", NaN, ...
    "CollisionFree", false, "KinematicCertificatePassed", false, ...
    "TerminationReason", "");
end

function constants = scenarioConstants()
%% Section 0: Header & Readme
% SYNTAX
%   constants = scenarioConstants()
%**************************************************************************
% PURPOSE
%   - Define the exact geometry and limits shared with the HS3 baseline.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - constants (scalar repeated-turn scenario record)
%**************************************************************************
% UNITS
%   - Geometry is degrees; time is seconds; derivatives use deg/s powers.
%**************************************************************************
constants = struct( ...
    "barrierSpacing_deg", 4, "barrierHalfWidth_deg", 0.7, ...
    "barrierCenterMagnitude_deg", 2.5, ...
    "barrierHalfHeight_deg", 2.5, "safetyMargin_deg", 0.1, ...
    "goalTimePerStage_s", 5.5, "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1], "maxJerk_deg_s3", [2 2], ...
    "elevationInterval_deg", [-5 5]);
end
