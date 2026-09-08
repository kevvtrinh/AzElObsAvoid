function report = benchmarkRandomMovingPolygonStress(randomSeeds, benchmarkOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   report = benchmarkRandomMovingPolygonStress()
%   report = benchmarkRandomMovingPolygonStress(randomSeeds)
%   report = benchmarkRandomMovingPolygonStress( ...
%       randomSeeds, benchmarkOverrides)
%**************************************************************************
% PURPOSE
%   - Stress the public HS3 planner with deterministic large random polygons
%     that
%     translate across most of the X/Y frame while rotating at least 180
%     coordinate units.
%   - Preserve exact inputs and returned diagnostics for every failure.
%   - Keep an analytically clear boundary witness outside planner inputs so
%     search failures are distinguishable from obvious geometric no-paths.
%**************************************************************************
% INPUTS
%   - randomSeeds (numeric vector, optional; default 1001:1012)
%       Nonnegative deterministic scenario seeds.
%   - benchmarkOverrides (scalar struct, optional; default struct())
%       .ObstacleCount is an integer at least two (default 3).
%       .MissionTime_s is positive (default 60).
%       .PrintProgress is logical (default true).
%       .PlannerOverrides is a scalar HS3 option struct (default struct()).
%**************************************************************************
% OUTPUTS
%   - report (scalar struct)
%       Controls plus one record per seed. Each record retains exact
%       inputs, scenario geometry, result, independent validation, wall
%       time, and the analytic witness-clearance lower bound.
%**************************************************************************
% UNITS
%   - Positions, radii, and clearance are coordinate units. Time is seconds and
%     derivatives use units/s, units/s^2, and units/s^3.
%**************************************************************************

%% Section 1: Resolve Reproducible Controls

% Fix the random seed and all run limits before creating cases. A failed case
% can then be created again with the same obstacle motion. Start here when two
% runs produce different results for the same requested seed.

if nargin < 1 || isempty(randomSeeds)
    randomSeeds = 1001:1012;
end
if nargin < 2 || isempty(benchmarkOverrides)
    benchmarkOverrides = struct();
end
validateattributes(randomSeeds, {'numeric'}, {'real', 'finite', 'vector', 'integer', 'nonnegative'});
defaults = struct();
defaults.ObstacleCount    = 3;
defaults.MissionTime_s    = 60;
defaults.PrintProgress    = true;
defaults.PlannerOverrides = struct();
[controls, unknownNames] = obstacleAvoidance.input.resolveOptions(defaults, benchmarkOverrides);
if ~isempty(unknownNames)
    warning("benchmarkRandomMovingPolygonStress:UnknownOptions", "Ignoring unknown fields: %s. No behavior changed.", strjoin(unknownNames, ", "));
end
validateattributes(controls.ObstacleCount, {'numeric'}, {'real', 'finite', 'scalar', 'integer', '>=', 2});
validateattributes(controls.MissionTime_s, {'numeric'}, {'real', 'finite', 'scalar', 'positive'});
controls.PrintProgress = obstacleAvoidance.input.normalizeLogicalScalar(controls.PrintProgress, "PrintProgress", "benchmarkRandomMovingPolygonStress:InvalidPrintProgress");
if ~isstruct(controls.PlannerOverrides) || ~isscalar(controls.PlannerOverrides)
    error("benchmarkRandomMovingPolygonStress:InvalidPlannerOverrides", "PlannerOverrides must be a scalar struct.");
end
randomSeeds = double(randomSeeds(:));

%% Section 2: Run Every Seed With HS3

% Run cases in serial order. Record success, validation, motion length, solver
% time, and the final reason for every seed. A planning failure remains in the
% report. It is not replaced with a new random case.

recordCount = numel(randomSeeds);
records     = repmat(emptyRecord(), recordCount, 1);
% Process each seed included in this benchmark measurement.
for seedIndex = 1:numel(randomSeeds)
    randomSeed = randomSeeds(seedIndex);
    [obstacles, initialState, goalState, limits, scenario] = createStressScenario(randomSeed, controls);
    records(seedIndex) = runStressCase(randomSeed, obstacles, initialState, goalState, limits, scenario, controls);
    if controls.PrintProgress
        printRecord(records(seedIndex));
    end
end

%% Section 3: Assemble Failure-Preserving Evidence

% Keep all case records in output order. Summary counts help find a trend, but
% each record keeps the seed and failure details needed for investigation.

passed = [records.Success].' & [records.IndependentValidationPassed].';
report = struct("BenchmarkName", "randomMovingPolygonStress", ...
    "Controls", controls, ...
    "RandomSeeds", randomSeeds, ...
    "Records", records, ...
    "PassedCount", nnz(passed), ...
    "FailedCount", nnz(~passed));
end

%% Section 4: Local Functions

function [obstacles, initialState, goalState, limits, scenario] = createStressScenario(randomSeed, controls)
    % Create several large rotating polygons with a clear outer witness route.
    rng(randomSeed, "twister");
    missionTime_s           = controls.MissionTime_s;
    obstacleTime_s          = linspace(0, missionTime_s, 13).';
    obstacleCount           = controls.ObstacleCount;
    safetyMargin_units        = 0.25;
    centerXLimit_units  = 17.5;
    centerY_units     = linspace(-4, 4, obstacleCount).';
    obstacleCells           = cell(obstacleCount, 1);
    histories               = cell(obstacleCount, 1);
    maximumSourceRadius_units = zeros(obstacleCount, 1);
    rotationTravel_deg      = zeros(obstacleCount, 1);

    % Each source polygon is star-shaped, simple, and has five to twelve vertices.
    for obstacleIndex = 1:obstacleCount
        vertexCount        = randi([5 12]);
        nominalAngles_rad  = (0:vertexCount - 1).' * (2 * pi / vertexCount);
        angularJitter_rad  = (rand(vertexCount, 1) - 0.5) * (0.55 * 2 * pi / vertexCount);
        sourceAngles_rad   = sort(nominalAngles_rad + angularJitter_rad);
        nominalRadius_units  = 6.5 + 1.5 * rand;
        sourceRadius_units   = nominalRadius_units * (0.68 + 0.32 * rand(vertexCount, 1));
        sourcePosition_units = sourceRadius_units .* [cos(sourceAngles_rad), sin(sourceAngles_rad)];
        maximumSourceRadius_units(obstacleIndex) = max(sourceRadius_units);
        initialRotation_deg = 360 * rand;
        rotationTravel_deg(obstacleIndex) = 180 + 180 * rand;
        if mod(obstacleIndex, 2) == 1
            startX_units = -centerXLimit_units;
            endX_units   = centerXLimit_units;
        else
            startX_units = centerXLimit_units;
            endX_units   = -centerXLimit_units;
        end
        verticalPhase_rad = 2 * pi * rand;
        sliceTransform    = @(source_units, sampleTime_s, sampleIndex) transformStressPolygon(source_units, sampleTime_s, sampleIndex, missionTime_s, startX_units, endX_units, centerY_units(obstacleIndex), verticalPhase_rad, initialRotation_deg, rotationTravel_deg(obstacleIndex));
        [obstacleCells{obstacleIndex}, histories{obstacleIndex}] = obstacleAvoidance.obstacles.createMovingObstacle("Random crossing polygon " + obstacleIndex, obstacleTime_s, sourcePosition_units(:, 1), sourcePosition_units(:, 2), sliceTransform, safetyMargin_units, struct("Verbose", false));
    end
    obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacleCells);

    workspaceX_units           = [-30 30];
    workspaceY_units         = [-20 20];
    startPosition_units              = [-28 0];
    goalPosition_units               = [28 0];
    witnessY_units           = -18;
    maximumRadiusWithMargin_units    = max(maximumSourceRadius_units) + safetyMargin_units;
    sideClearanceLowerBound_units    = abs(startPosition_units(1) - (-centerXLimit_units)) - maximumRadiusWithMargin_units;
    maximumCenterExcursion_units     = max(abs(centerY_units)) + 0.75;
    bottomClearanceLowerBound_units  = abs(witnessY_units) - maximumCenterExcursion_units - maximumRadiusWithMargin_units;
    witnessClearanceLowerBound_units = min(sideClearanceLowerBound_units, bottomClearanceLowerBound_units);
    if witnessClearanceLowerBound_units <= 0
        error("benchmarkRandomMovingPolygonStress:InvalidWitness", "Generated radial bounds do not leave the declared witness route clear.");
    end

    initialState = struct("time_s", 0, "position_units", startPosition_units, ...
        "velocity_units_s", [0 0], "acceleration_units_s2", [0 0]);
    goalState = struct("time_s", missionTime_s, "position_units", goalPosition_units, ...
        "velocity_units_s", [0 0], "acceleration_units_s2", [0 0]);
    limits = struct("maxVelocity_units_s", [4 4], ...
        "maxAcceleration_units_s2", [1.5 1.5], ...
        "maxJerk_units_s3", [3 3], ...
        "xInterval_units", workspaceX_units, ...
        "yInterval_units", workspaceY_units);
    scenario = struct("ObstacleTime_s", obstacleTime_s, ...
        "ObstacleHistories", {histories}, ...
        "MaximumSourceRadius_units", maximumSourceRadius_units, ...
        "RotationTravel_deg", rotationTravel_deg, ...
        "WitnessRoute_units", [ ...
            startPosition_units; ...
            startPosition_units(1) witnessY_units; goalPosition_units(1) witnessY_units; goalPosition_units], "WitnessClearanceLowerBound_units", witnessClearanceLowerBound_units);
end

function transformed_units = transformStressPolygon(sourcePosition_units, sampleTime_s, ~, missionTime_s, startX_units, endX_units, centerY_units, verticalPhase_rad, initialRotation_deg, rotationTravel_deg)
    % Translate across the frame and rotate through at least one half-turn.
    progress       = min(max(sampleTime_s / missionTime_s, 0), 1);
    smoothProgress = 10 * progress^3 - 15 * progress^4 + 6 * progress^5;
    center_units     = [ ...
        startX_units + ...
            smoothProgress * (endX_units - startX_units), ...
        centerY_units + ...
            0.75 * sin(2 * pi * progress + verticalPhase_rad)];
    rotation_rad   = deg2rad(initialRotation_deg + rotationTravel_deg * smoothProgress);
    rotationMatrix = [ ...
        cos(rotation_rad) -sin(rotation_rad); sin(rotation_rad) cos(rotation_rad)];
    transformed_units = sourcePosition_units * rotationMatrix.' + center_units;
end

function record = runStressCase(randomSeed, obstacles, initialState, goalState, limits, scenario, controls)
    % Run HS3 once and preserve unsuccessful evidence without retrying.
    plannerOptions = controls.PlannerOverrides;
    plannerOptions.GoalTimeMode = "earliestArrival";
    plannerTimer      = tic;
    result            = planner(obstacles, initialState, goalState, limits, plannerOptions);
    plannerWallTime_s = toc(plannerTimer);
    validation        = obstacleAvoidance.validateTrajectory();
    if result.Success
        validation = obstacleAvoidance.validateTrajectory(result);
    end
    record = emptyRecord();
    record.RandomSeed                     = randomSeed;
    record.Success                        = result.Success;
    record.IndependentValidationPassed    = result.Success && validation.Passed;
    record.TerminationReason              = result.TerminationReason;
    record.PlannerWallTime_s              = plannerWallTime_s;
    record.ArrivalTime_s                  = result.ArrivalTime_s;
    record.WitnessClearanceLowerBound_units = scenario.WitnessClearanceLowerBound_units;
    record.Inputs                         = struct("obstacles", obstacles, ...
        "initialState", initialState, ...
        "goalState", goalState, ...
        "limits", limits, ...
        "options", result.Options);
    record.Scenario              = scenario;
    record.PlannerResult         = result;
    record.IndependentValidation = validation;
end

function record = emptyRecord()
    % Define stable success/failure evidence for one HS3 seed.
    record = struct("RandomSeed", NaN, ...
        "Success", false, ...
        "IndependentValidationPassed", false, ...
        "TerminationReason", "", ...
        "PlannerWallTime_s", NaN, ...
        "ArrivalTime_s", NaN, ...
        "WitnessClearanceLowerBound_units", NaN, ...
        "Inputs", struct(), ...
        "Scenario", struct(), ...
        "PlannerResult", struct(), ...
        "IndependentValidation", obstacleAvoidance.validateTrajectory());
end

function printRecord(record)
    % Print concise evidence while retaining complete records in the report.
    fprintf("random=%d success=%s validation=%s wall_s=%.6f " + "arrival_s=%.6g witness_clearance_units=%.6g reason=%s\n", record.RandomSeed, string(logical(record.Success)), string(logical(record.IndependentValidationPassed)), record.PlannerWallTime_s, record.ArrivalTime_s, record.WitnessClearanceLowerBound_units, record.TerminationReason);
end
