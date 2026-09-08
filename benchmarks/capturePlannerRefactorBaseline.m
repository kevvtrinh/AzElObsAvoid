function baseline = capturePlannerRefactorBaseline(outputPath)
%% Section 0: Header & Readme
% SYNTAX
%   baseline = capturePlannerRefactorBaseline()
%   baseline = capturePlannerRefactorBaseline(outputPath)
%**************************************************************************
% PURPOSE
%   - Run every maintained example with display work disabled.
%   - Save complete results and runtime-free comparison records before a
%     behavior-preserving planner refactor.
%**************************************************************************
% INPUTS
%   - outputPath (scalar text, optional)
%       MAT-file path. Empty uses benchmarks/planner_refactor_baseline.mat.
%**************************************************************************
% OUTPUTS
%   - baseline (scalar struct)
%       Starting revision, MATLAB version, example results, independent
%       checks, runtimes, and physical records used for later comparison.
%**************************************************************************
% UNITS
%   - Position is coordinate units. Time is seconds. Derivatives use units/s, units/s^2,
%     and units/s^3.
%**************************************************************************

%% Section 1: Locate The Repository And Examples

% The examples use public planner and plotting entry points. Add their
% folders once so each run starts from the same production implementation.

repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot);
addpath(fullfile(repositoryRoot, "trajectory"));
addpath(fullfile(repositoryRoot, "examples"));
if nargin < 1 || isempty(outputPath)
    outputPath = fullfile(repositoryRoot, "benchmarks", "planner_refactor_baseline.mat");
end
outputPath = string(outputPath);
if ~isscalar(outputPath) || ismissing(outputPath) || strlength(outputPath) == 0
    error("capturePlannerRefactorBaseline:InvalidOutputPath", "outputPath must be nonempty scalar text.");
end

exampleNames = [ ...
    "exampleAlternatingSlalom", ...
    "exampleVietnamKeepoutSlew", ...
    "exampleDenseConcaveObstacle", ...
    "exampleFourAcceleratingCircles", ...
    "exampleInterceptMovingTargetAtSetTime", ...
    "exampleInterceptMovingTargetEarliest", ...
    "exampleMovingBarrierWait", ...
    "exampleMovingCircleNoWrap", ...
    "exampleMovingDeformingUSOutlineVisibility", ...
    "exampleMovingRotatingObstacleField", ...
    "exampleNoPath", ...
    "exampleObstacleFree", ...
    "exampleOpeningUShapedObstacle", ...
    "exampleStraightTargetAlternatingOcclusion", ...
    "exampleTargetExitsObstacle", ...
    "exampleTwoOpposingUVisibilityGraph", ...
    "exampleStaticUShapedObstacle", ...
    "exampleUSOutlineExtremeVisibility"];

% Expected outcomes belong to the scenario, never to the returned result.
expectedSuccess = exampleNames ~= "exampleNoPath";

%% Section 2: Run And Check Every Example

% Each example must return its unmodified planner result. Run a fresh full
% motion check here so the saved baseline does not merely trust Success.

exampleCount      = numel(exampleNames);
exampleResults    = cell(exampleCount, 1);
exampleDiagnoses  = cell(exampleCount, 1);
independentChecks = cell(exampleCount, 1);
physicalRecords   = cell(exampleCount, 1);
elapsedTime_s     = zeros(exampleCount, 1);
% Process each example included in this benchmark measurement.
for exampleIndex = 1:exampleCount
    exampleName  = exampleNames(exampleIndex);
    exampleTimer = tic;
    [result, diagnosis] = feval(exampleName, struct("PlotOutputs", false, "FigureVisible", "off", "Verbose", false));
    elapsedTime_s(exampleIndex) = toc(exampleTimer);

    checkResult = validateExampleResult(result, exampleName, struct("ExpectedSuccess", expectedSuccess(exampleIndex)), diagnosis);
    exampleResults{exampleIndex} = result;
    exampleDiagnoses{exampleIndex} = diagnosis;
    independentChecks{exampleIndex} = checkResult;
    physicalRecords{exampleIndex} = createPhysicalRecord(result);
    printExampleResult(exampleName, result, checkResult, elapsedTime_s(exampleIndex));
end

% The maintained matrix covers the major scenario families. Retain named
% copies of the focused cases and run the explicit backup-policy request that
% is not enabled by any normal example.
focusedRecords = createFocusedRecords(exampleNames, exampleResults);
focusedRecords.ExplicitRuckigBackup = createRuckigBackupRecord();

%% Section 3: Save The Baseline

% The Git revision identifies the source that produced these values. Runtime
% remains separate because refactoring must preserve physics even when timing
% naturally varies between runs.

startingCommit = readStartingCommit(repositoryRoot);
baseline       = struct("SchemaVersion", 2, ...
    "StartingCommit", startingCommit, ...
    "MatlabVersion", string(version), ...
    "ExampleNames", exampleNames, ...
    "ExpectedSuccess", expectedSuccess, ...
    "ExampleResults", {exampleResults}, ...
    "ExampleDiagnoses", {exampleDiagnoses}, ...
    "IndependentChecks", {independentChecks}, ...
    "PhysicalRecords", {physicalRecords}, ...
    "FocusedRecords", focusedRecords, ...
    "ElapsedTime_s", elapsedTime_s);
save(outputPath, "baseline", "-v7.3");
passedCount = 0;
% Process each example included in this benchmark measurement.
for exampleIndex = 1:exampleCount
    passedCount = passedCount + independentChecks{exampleIndex}.Passed;
end
fprintf("REFACTOR_BASELINE total=%d checked=%d file=%s\n", exampleCount, passedCount, outputPath);
end

%% Section 4: Local Functions

function focusedRecords = createFocusedRecords(exampleNames, exampleResults)
    % Name the scenarios required for quick diagnosis after each refactor phase.
    focusedRecords = struct("StaticDetour", findExampleRecord("exampleStaticUShapedObstacle", exampleNames, exampleResults), ...
        "DirectWait", findExampleRecord("exampleMovingBarrierWait", exampleNames, exampleResults), ...
        "MovingSpatialDetour", findExampleRecord("exampleMovingRotatingObstacleField", exampleNames, exampleResults), ...
        "DenseOutline", findExampleRecord("exampleUSOutlineExtremeVisibility", exampleNames, exampleResults), ...
        "DisconnectedGraph", findExampleRecord("exampleTwoOpposingUVisibilityGraph", exampleNames, exampleResults), ...
        "ExpectedNoPath", findExampleRecord("exampleNoPath", exampleNames, exampleResults));
end

function record = findExampleRecord(exampleName, exampleNames, exampleResults)
    % Return one runtime-free record from the fixed maintained-example inventory.
    exampleIndex = find(exampleNames == exampleName, 1);
    if isempty(exampleIndex)
        error("capturePlannerRefactorBaseline:MissingFocusedExample", "The focused example '%s' is not in the maintained inventory.", exampleName);
    end
    record = createPhysicalRecord(exampleResults{exampleIndex});
end

function record = createRuckigBackupRecord()
    % Run one route whose unsupported timed topology explicitly enables backup.
    missionEndTime_s = 8;
    obstacleTime_s   = [0; missionEndTime_s];
    uPosition_units    = [ ...
        -8 7; -5 7; -5 -4; 5 -4; 5 7; 8 7; 8 -7; -8 -7];
    staticObstacle  = obstacleAvoidance.obstacles.createObstacle("static U-shaped obstacle", obstacleTime_s, uPosition_units(:, 1), uPosition_units(:, 2), 0.20);
    movingStart_units = [30 30; 32 30; 32 32; 30 32];
    movingEnd_units   = movingStart_units + [4 0];
    movingObstacle  = obstacleAvoidance.obstacles.createObstacle("distant moving obstacle", obstacleTime_s, {movingStart_units(:, 1); movingEnd_units(:, 1)}, {movingStart_units(:, 2); movingEnd_units(:, 2)}, 0.10);
    obstacles       = obstacleAvoidance.obstacles.combineObstacles(staticObstacle, movingObstacle);
    initialState    = struct();
    initialState.time_s       = 0;
    initialState.position_units = [0 0];
    goalState = struct("time_s", missionEndTime_s, "position_units", [0 -10]);
    limits = struct();
    limits.maxVelocity_units_s      = [2 2];
    limits.maxAcceleration_units_s2 = [0.75 0.75];
    limits.maxJerk_units_s3         = [2.5 2.5];
    options = struct();
    options.GoalTimeMode                   = "fixedArrival";
    options.MaximumSeedCount               = 3;
    options.SampleTime_s                   = 0.05;
    options.UnsupportedTimedTopologyPolicy = "ruckigStopAtWaypoints";
    result = obstacleAvoidance.planTrajectory(obstacles, initialState, goalState, limits, options);
    record = createPhysicalRecord(result);
end

function record = createPhysicalRecord(result)
    % Retain planning decisions and physical outputs while removing runtime noise.
    record = struct("Success", result.Success, ...
        "TerminationReason", result.TerminationReason, ...
        "Options", result.Options, ...
        "Inputs", result.Inputs, ...
        "Route_units", result.Route_units, ...
        "time_s", result.time_s, ...
        "position_units", result.position_units, ...
        "velocity_units_s", result.velocity_units_s, ...
        "acceleration_units_s2", result.acceleration_units_s2, ...
        "jerk_units_s3", result.jerk_units_s3, ...
        "Polynomial", result.Polynomial, ...
        "SeedCorridorBoundary_units", result.SeedCorridorBoundary_units, ...
        "SeedCorridor", result.SeedCorridor, ...
        "PlaneCertificate", result.PlaneCertificate, ...
        "Validation", result.Validation, ...
        "ArrivalTime_s", result.ArrivalTime_s, ...
        "TrajectoryDuration_s", result.TrajectoryDuration_s);
end

function printExampleResult(exampleName, result, checkResult, elapsedTime_s)
    % Print the complete maintained-example report required by repository policy.
    if result.Success
        motionCheck        = checkResult.TrajectoryValidation;
        motionLength_units   = sum(vecnorm(diff(result.position_units), 2, 2));
        polylineLength_units = sum(vecnorm(diff(result.Route_units, 1, 1), 2, 2));
        kinematicPassed    = motionCheck.VelocityWithinLimits && motionCheck.AccelerationWithinLimits && motionCheck.JerkWithinLimits;
        certificatePassed  = motionCheck.CollisionFree && motionCheck.CollisionResolved && motionCheck.PolynomialFormatValid && motionCheck.DynamicsConsistent && motionCheck.InitialStateMatched && motionCheck.TerminalStateMatched;
        reportFormat       = "REFACTOR_EXAMPLE name=%s jerk=enabled success=%d check=%d validator=%d " + "reason=%s polyline_units=%.15g smooth_units=%.15g " + "duration_s=%.15g collision=%d kinematic=%d certificate=%d " + "runtime_s=%.6f\n";
        fprintf(reportFormat, exampleName, result.Success, checkResult.Passed, motionCheck.Passed, result.TerminationReason, polylineLength_units, motionLength_units, result.TrajectoryDuration_s, motionCheck.CollisionFree, kinematicPassed, certificatePassed, elapsedTime_s);
    else
        reportFormat = "REFACTOR_EXAMPLE name=%s jerk=enabled success=%d check=%d validator=%d " + "reason=%s polyline_units=NaN smooth_units=NaN duration_s=NaN " + "collision=0 kinematic=NaN certificate=NaN runtime_s=%.6f\n";
        fprintf(reportFormat, exampleName, result.Success, checkResult.Passed, false, result.TerminationReason, elapsedTime_s);
    end
end

function startingCommit = readStartingCommit(repositoryRoot)
    % Read the exact revision without changing repository or global Git settings.
    safeRoot = replace(string(repositoryRoot), "\", "/");
    command  = sprintf('git -c safe.directory="%s" rev-parse HEAD', safeRoot);
    [status, output] = system(command);
    if status == 0
        startingCommit = strtrim(string(output));
    else
        startingCommit = "unknown";
    end
end
