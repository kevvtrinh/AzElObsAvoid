function tests = testAzElPlannerMigration
%% Section 0: Header & Readme
% SYNTAX
%   results = runtests("tests/testAzElPlannerMigration.m")
%**************************************************************************
% PURPOSE
%   - Guard construction-time safety-margin ownership, migration errors,
%     display conventions, and deterministic planner progress reporting.
%   - Certify analytic jerk limits, fixed-arrival holds, rounded detours,
%     and the standard four-panel kinematic diagnostic schema.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest.FunctionTestCase array)
%       Focused regression tests for the maintained planner API.
%**************************************************************************
% UNITS
%   - Angles are degrees and time is seconds.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
%% Section 0: Header & Readme
% Put the repository API on the path and keep all test figures invisible.
testFilePath = mfilename("fullpath");
repositoryRoot = fileparts(fileparts(testFilePath));
testCase.TestData.OriginalPath = path;
testCase.TestData.OriginalFigureVisibility = ...
    get(groot, "DefaultFigureVisible");
addpath(repositoryRoot);
addpath(fullfile(repositoryRoot, "examples"));
set(groot, "DefaultFigureVisible", "off");
end

function teardownOnce(testCase)
%% Section 0: Header & Readme
% Restore process-wide state changed by this test file.
closeTestFigures();
set(groot, "DefaultFigureVisible", ...
    testCase.TestData.OriginalFigureVisibility);
path(testCase.TestData.OriginalPath);
end

function setup(~)
%% Section 0: Header & Readme
% Start every test with no graphics left by an earlier assertion.
closeTestFigures();
end

function teardown(~)
%% Section 0: Header & Readme
% Close diagnostic and animation figures even after a failed assertion.
closeTestFigures();
end

function testConstructorOwnsAbsoluteIdempotentMargin(testCase)
%% Section 0: Header & Readme
% Verify one stored margin and absolute, non-cumulative reinflation.
[rawAzimuth_deg, rawElevation_deg] = squareBoundary([0 0]);
margin_deg = 0.4;
obstacle = makeAzElObstacleData( ...
    "margin owner", [0; 10], rawAzimuth_deg, ...
    rawElevation_deg, margin_deg);

testCase.verifyEqual(obstacle.safetyMargin_deg, margin_deg);
testCase.verifyEqual(obstacle.originalAz_deg{1}, rawAzimuth_deg);
testCase.verifyEqual(obstacle.originalEl_deg{1}, rawElevation_deg);
testCase.verifyEqual(obstacle.targetName, "margin owner");

largerObstacle = inflateAzElObstacleData(obstacle, 0.8);
reappliedObstacle = inflateAzElObstacleData(largerObstacle, margin_deg);
for sampleIndex = 1:numel(obstacle.time_s)
    testCase.verifyEqual(reappliedObstacle.az_deg{sampleIndex}, ...
        obstacle.az_deg{sampleIndex}, "AbsTol", 1e-12);
    testCase.verifyEqual(reappliedObstacle.el_deg{sampleIndex}, ...
        obstacle.el_deg{sampleIndex}, "AbsTol", 1e-12);
    testCase.verifyEqual(reappliedObstacle.originalAz_deg{sampleIndex}, ...
        rawAzimuth_deg);
    testCase.verifyEqual(reappliedObstacle.originalEl_deg{sampleIndex}, ...
        rawElevation_deg);
end
testCase.verifyEqual(reappliedObstacle.safetyMargin_deg, margin_deg);
testCase.verifyEqual(reappliedObstacle.targetName, obstacle.targetName);
end

function testMixedMarginsRemainPerObstacle(testCase)
%% Section 0: Header & Readme
% Verify packed metadata and collision geometry retain mixed margins.
firstObstacle = makeSquareObstacle("no margin", [0 0], 0, [0; 10]);
secondObstacle = makeSquareObstacle("wide margin", [5 0], 0.5, [0; 10]);
obstacleField = buildAzElTimeObstacleField( ...
    {firstObstacle, secondObstacle});

testCase.verifyEqual(obstacleField.SafetyMarginsDeg, [0; 0.5]);
testCase.verifyEqual( ...
    [obstacleField.Obstacles.SafetyMarginDeg].', [0; 0.5]);
[isOccupied, blockingObstacleIndex, queryDetails] = ...
    queryAzElTimeObstacle(obstacleField, [1.25 6.25], [0 0], [5 5]);
testCase.verifyEqual(logical(isOccupied), [false true]);
testCase.verifyEqual(double(blockingObstacleIndex), [0 2]);
testCase.verifyEqual( ...
    queryDetails.ObstacleSafetyMarginsDeg, [0; 0.5]);
end

function testLegacyMarginOptionsExplainMigration(testCase)
%% Section 0: Header & Readme
% Verify downstream safety controls fail with the migration identifiers.
obstacle = makeSquareObstacle("migration", [0 0], 0.2, [0; 10]);
obstacleField = buildAzElTimeObstacleField(obstacle);
initialState = struct("time_s", 0, "position_deg", [-3 0]);
goalState = struct("time_s", 10, "position_deg", [3 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1]);

testCase.verifyError(@() planAzElMotion( ...
    obstacle, initialState, goalState, limits, ...
    struct("SafetyMarginDeg", 0.2)), ...
    "planAzElMotion:SafetyMarginMoved");
testCase.verifyError(@() planAzElMotion( ...
    obstacle, initialState, goalState, limits, ...
    struct("RoundingClearance_deg", 0.2)), ...
    "planAzElMotion:SafetyMarginMoved");
testCase.verifyError(@() queryAzElTimeObstacle( ...
    obstacleField, 0, 0, 5, struct("SafetyMarginDeg", 0.2)), ...
    "queryAzElTimeObstacle:SafetyMarginMoved");
testCase.verifyError(@() queryAzElTimeObstacle( ...
    obstacleField, 0, 0, 5, struct("BoundsMarginDeg", 0.2)), ...
    "queryAzElTimeObstacle:SafetyMarginMoved");

plannerDefaults = planAzElMotion();
queryDefaults = queryAzElTimeObstacle();
testCase.verifyFalse(isfield(plannerDefaults, "SafetyMarginDeg"));
testCase.verifyFalse(isfield(plannerDefaults, "RoundingClearance_deg"));
testCase.verifyFalse(isfield(queryDefaults, "SafetyMarginDeg"));
testCase.verifyFalse(isfield(queryDefaults, "BoundsMarginDeg"));
end

function testOriginalAndProtectedBoundaryStyles(testCase)
%% Section 0: Header & Readme
% Verify static and animated views use solid original and dashed protected.
obstacle = makeSquareObstacle("styled", [0 0], 0.3, [0; 1]);
obstacleField = buildAzElTimeObstacleField(obstacle);
initialState = struct("time_s", 0, "position_deg", [-2 2]);
goalState = struct("time_s", 1, "position_deg", [2 2]);
viewOptions = struct( ...
    "FigureVisible", "off", ...
    "ShowSweptSurfaces", false, ...
    "ShowBoundaryCandidates", false, ...
    "ShowBoundaryConnections", false, ...
    "ShowVisibilityGraph", false);
spaceView = visualizeAzElTimeSpace( ...
    obstacleField, initialState, goalState, viewOptions);
verifyLineStyles(testCase, spaceView.OriginalBoundaryHandles, "-");
verifyLineStyles(testCase, spaceView.ProtectedBoundaryHandles, "--");

timedPath = struct( ...
    "Success", true, ...
    "time_s", [0; 1], ...
    "position_deg", [-2 2; 2 2], ...
    "velocity_deg_s", [4 0; 4 0]);
animation = animateAzElTimedSlopePath( ...
    timedPath, obstacleField, struct( ...
    "FigureVisible", "off", ...
    "FrameStride", 10, ...
    "Pause_s", 0, ...
    "ShowSweptSurfaces", false));
verifyLineStyles(testCase, animation.OriginalBoundary3D, "-");
verifyLineStyles(testCase, animation.ProtectedBoundary3D, "--");
verifyLineStyles(testCase, animation.OriginalBoundary2D, "-");
verifyLineStyles(testCase, animation.ProtectedBoundary2D, "--");
end

function testVerboseDoesNotChangePlanOrProgressLog(testCase)
%% Section 0: Header & Readme
% Verify Verbose changes console mirroring only, never deterministic data.
obstacle = makeSquareObstacle( ...
    "distant", [10 10], 0.15, [0; 8]); %#ok<NASGU>
[initialState, goalState, limits, quietOptions] = ...
    directPlanInputs(); %#ok<ASGLU>
quietOutput = evalc("quietResult = planAzElMotion(" + ...
    "obstacle, initialState, goalState, limits, quietOptions);");
closeTestFigures();
verboseOptions = quietOptions;
verboseOptions.Verbose = true;
verboseOutput = evalc("verboseResult = planAzElMotion(" + ...
    "obstacle, initialState, goalState, limits, verboseOptions);");

testCase.verifyEqual(strlength(strtrim(string(quietOutput))), 0);
testCase.verifyNotEmpty(strfind(verboseOutput, "[AzEl]"));
testCase.verifyEqual(quietResult.ProgressLog, verboseResult.ProgressLog);
testCase.verifyEqual(quietResult.selectedCandidateIndex, ...
    verboseResult.selectedCandidateIndex);
testCase.verifyEqual(quietResult.timedSlopePath.position_deg, ...
    verboseResult.timedSlopePath.position_deg, "AbsTol", 0);
testCase.verifyEqual(quietResult.timedSlopePath.time_s, ...
    verboseResult.timedSlopePath.time_s, "AbsTol", 0);
testCase.verifyTrue(quietResult.Success);
end

function testExpectedFailureHasTerminalProgressRecord(testCase)
%% Section 0: Header & Readme
% Verify blocked endpoints return stable diagnostics and a terminal event.
obstacle = makeSquareObstacle("blocked start", [0 0], 0.2, [0; 10]);
initialState = struct("time_s", 0, "position_deg", [0 0]);
goalState = struct("time_s", 10, "position_deg", [4 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1]);
options = plannerTestOptions();
result = planAzElMotion( ...
    obstacle, initialState, goalState, limits, options);

testCase.verifyFalse(result.Success);
testCase.verifyFalse(result.Validation.EndpointClear);
testCase.verifyEqual(string(fieldnames(result.ProgressLog)), [ ...
    "Sequence"; "Stage"; "Event"; "Status"; "Message"; ...
    "ObstacleIndex"; "CandidateIndex"; "Details"]);
testCase.verifyEqual([result.ProgressLog.Sequence].', ...
    (1:numel(result.ProgressLog)).');
testCase.verifyEqual(result.ProgressLog(end).Event, "PlanningFailed");
testCase.verifyEqual(result.ProgressLog(end).Status, "failed");
end

function testIncompatibleEndpointVelocityReturnsDiagnostics(testCase)
%% Section 0: Header & Readme
% Verify candidate-level dynamic incompatibility never escapes the planner.
obstacle = makeSquareObstacle("distant", [10 10], 0.2, [0; 12]);
initialState = struct( ...
    "time_s", 0, ...
    "position_deg", [0 0], ...
    "velocity_deg_s", [0 0.5]);
goalState = struct("time_s", 12, "position_deg", [5 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1]);
motionTypes = ["exactStop" "velocityCarrying"];
for motionIndex = 1:numel(motionTypes)
    options = plannerTestOptions();
    options.MotionType = motionTypes(motionIndex);
    result = planAzElMotion( ...
        obstacle, initialState, goalState, limits, options);
    testCase.verifyFalse(result.Success);
    testCase.verifyTrue(result.diagnosticTimedPathAvailable);
    testCase.verifyEqual(result.ProgressLog(end).Event, "PlanningFailed");
    closeTestFigures();
end
end

function testLegacyPackedMarginMetadataFallback(testCase)
%% Section 0: Header & Readme
% Verify geometry-only queries remain compatible with pre-v4 metadata.
firstObstacle = makeSquareObstacle("first", [0 0], 0.1, [0; 2]);
secondObstacle = makeSquareObstacle("second", [5 0], 0.4, [0; 2]);
legacyField = buildAzElTimeObstacleField( ...
    {firstObstacle, secondObstacle});
legacyField = rmfield(legacyField, "SafetyMarginsDeg");
legacyField.Obstacles = rmfield( ...
    legacyField.Obstacles, "SafetyMarginDeg");
[isOccupied, blockingIndex, details] = queryAzElTimeObstacle( ...
    legacyField, [0 5], [0 0], [1 1]);
testCase.verifyEqual(logical(isOccupied), [true true]);
testCase.verifyEqual(double(blockingIndex), [1 2]);
testCase.verifyEqual(details.ObstacleSafetyMarginsDeg, [0; 0]);
end

function testObstacleFreePlannerUsesCanonicalEmptyField(testCase)
%% Section 0: Header & Readme
% Verify [] means a real zero-obstacle planning environment.
canonicalObstacles = combineAzElObstacles([]);
nestedCanonicalObstacles = combineAzElObstacles( ...
    {[], {cell(0, 1), []}});
obstacleField = buildAzElTimeObstacleField([]);
testCase.verifySize(canonicalObstacles, [0 1]);
testCase.verifySize(nestedCanonicalObstacles, [0 1]);
testCase.verifyEqual( ...
    fieldnames(nestedCanonicalObstacles), fieldnames(canonicalObstacles));
testCase.verifyEqual(obstacleField.ObstacleCount, 0);
testCase.verifyEmpty(obstacleField.Obstacles);
testCase.verifyEmpty(obstacleField.SafetyMarginsDeg);
[occupied, blocker, details] = queryAzElTimeObstacle( ...
    obstacleField, [0 1], [0 1], [0 1]);
testCase.verifyEqual(logical(occupied), [false false]);
testCase.verifyEqual(double(blocker), [0 0]);
testCase.verifyEmpty(details.ObstacleSafetyMarginsDeg);

initialState = struct("time_s", 0, "position_deg", [0 0]);
goalState = struct("time_s", 15, "position_deg", [8 -3]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [0.8 0.8]);
motionTypes = ["exactStop" "velocityCarrying"];
for motionIndex = 1:numel(motionTypes)
    options = plannerTestOptions();
    options.MotionType = motionTypes(motionIndex);
    result = planAzElMotion( ...
        [], initialState, goalState, limits, options);
    testCase.verifyTrue(result.Success);
    testCase.verifyEqual(result.obstacleField.ObstacleCount, 0);
    testCase.verifyEqual(height(result.candidateDiagnostics), 1);
    testCase.verifyFalse(any(result.directBlocked));
    testCase.verifyFalse(any( ...
        [result.ProgressLog.Event] == "ObstaclePrepared"));
    closeTestFigures();
end
end

function testMovingTargetInterceptExamples(testCase)
%% Section 0: Header & Readme
% Verify earliest and specified-time velocity-matched rendezvous examples.
options = plannerTestOptions();
options.MotionType = "velocityCarrying";
earliestResult = exampleInterceptMovingTargetEarliest(options);
testCase.verifyTrue(earliestResult.Success);
testCase.verifyTrue(earliestResult.InterceptValidation.Passed);
testCase.verifyTrue(earliestResult.SearchDiagnostics.EarliestCertified);
testCase.verifyLessThanOrEqual( ...
    earliestResult.InterceptValidation.PositionError_deg, 1e-7);
testCase.verifyLessThanOrEqual( ...
    earliestResult.InterceptValidation.VelocityError_deg_s, 1e-7);
testCase.verifyEqual(earliestResult.obstacleField.ObstacleCount, 0);
closeTestFigures();

animationOptions = options;
animationOptions.ShowAnimation = true;
animationOptions.AnimationFrameStride = 100000;
animationOptions.AnimationPause_s = 0;
specifiedResult = exampleInterceptMovingTargetAtSetTime( ...
    12, animationOptions);
testCase.verifyTrue(specifiedResult.Success);
testCase.verifyTrue(specifiedResult.InterceptValidation.Passed);
testCase.verifyEqual(specifiedResult.InterceptTime_s, 12, ...
    "AbsTol", 1e-8);
testCase.verifyEqual(specifiedResult.goalLineInterceptTime_s, 12, ...
    "AbsTol", 1e-7);
testCase.verifyGreaterThan( ...
    specifiedResult.timedSlopePath.WaitDuration_s, 0);
testCase.verifyEqual(specifiedResult.obstacleField.ObstacleCount, 0);
testCase.verifyTrue(isgraphics( ...
    specifiedResult.animation.TargetCurrent3D));
testCase.verifyTrue(isgraphics( ...
    specifiedResult.animation.TargetCurrent2D));
animatedTargetPosition_deg = [ ...
    specifiedResult.animation.TargetCurrent2D.XData, ...
    specifiedResult.animation.TargetCurrent2D.YData];
testCase.verifyEqual(animatedTargetPosition_deg, ...
    specifiedResult.TargetPositionAtIntercept_deg, "AbsTol", 1e-9);
end

function testAnalyticJerkRetimersHaveExactSampleInvariantTiming(testCase)
%% Section 0: Header & Readme
% Verify both motion modes use the same certified analytic S-curve on a
% direct line, independent of the requested output sampling interval.
[initialState, goalState, limits] = jerkLimitedPlanInputs();
motionTypes = ["exactStop" "velocityCarrying"];
sampleTimes_s = [0.05 0.8];
arrivalTimes_s = zeros(numel(motionTypes), numel(sampleTimes_s));
for motionIndex = 1:numel(motionTypes)
    for sampleIndex = 1:numel(sampleTimes_s)
        options = plannerTestOptions();
        options.MotionType = motionTypes(motionIndex);
        options.SampleTime_s = sampleTimes_s(sampleIndex);
        result = planAzElMotion( ...
            [], initialState, goalState, limits, options);
        timedPath = result.timedSlopePath;
        diagnostics = timedPath.ConstraintDiagnostics;
        arrivalTimes_s(motionIndex, sampleIndex) = ...
            result.goalLineInterceptTime_s;

        testCase.verifyTrue(result.Success);
        testCase.verifyTrue(result.Validation.Passed);
        testCase.verifyTrue(result.Validation.JerkWithinLimits);
        testCase.verifyEqual(timedPath.MinimumMotionDuration_s, 7.5, ...
            "AbsTol", 1e-10);
        testCase.verifyEqual(timedPath.WaitDuration_s, 0, ...
            "AbsTol", 1e-12);
        testCase.verifyEqual(timedPath.position_deg(1, :), [0 0], ...
            "AbsTol", 1e-12);
        testCase.verifyEqual(timedPath.position_deg(end, :), [10 0], ...
            "AbsTol", 1e-10);
        testCase.verifyEqual(timedPath.velocity_deg_s([1 end], :), ...
            zeros(2), "AbsTol", 1e-10);
        testCase.verifyEqual(timedPath.acceleration_deg_s2([1 end], :), ...
            zeros(2), "AbsTol", 1e-10);
        testCase.verifySize(timedPath.jerk_deg_s3, ...
            size(timedPath.position_deg));
        testCase.verifyLessThanOrEqual(max(abs( ...
            timedPath.velocity_deg_s), [], 1), ...
            limits.maxVelocity_deg_s + 1e-9);
        testCase.verifyLessThanOrEqual(max(abs( ...
            timedPath.acceleration_deg_s2), [], 1), ...
            limits.maxAcceleration_deg_s2 + 1e-9);
        testCase.verifyLessThanOrEqual(max(abs( ...
            timedPath.jerk_deg_s3), [], 1), ...
            limits.maxJerk_deg_s3 + 1e-9);
        testCase.verifyEqual(diagnostics.PeakVelocity_deg_s, [2 0], ...
            "AbsTol", 1e-10);
        testCase.verifyEqual(diagnostics.PeakAcceleration_deg_s2, [1 0], ...
            "AbsTol", 1e-10);
        testCase.verifyEqual(diagnostics.PeakJerk_deg_s3, [2 0], ...
            "AbsTol", 1e-10);
        testCase.verifyTrue(diagnostics.JerkConstrained);
        testCase.verifyTrue(diagnostics.JerkSatisfied);
        testCase.verifyTrue(diagnostics.FiniteJerkCertified);
        testCase.verifyTrue(diagnostics.Satisfied);
        testCase.verifyEqual( ...
            diagnostics.CurvatureDiscontinuityStopCount, 0);
        if motionTypes(motionIndex) == "exactStop"
            testCase.verifyFalse(diagnostics.RoundedVelocityCarried);
        else
            testCase.verifyTrue(diagnostics.RoundedVelocityCarried);
        end
        testCase.verifyNotEmpty(regexpi( ...
            char(timedPath.RetimerType), "jerk", "once"));
        closeTestFigures();
    end
end
testCase.verifyEqual(arrivalTimes_s, 7.5 * ones(2), ...
    "AbsTol", 1e-10);
end

function testJerkRetimersHonorFixedArrivalWithStartHold(testCase)
%% Section 0: Header & Readme
% Verify fixed arrival adds a stationary hold without changing the
% analytically minimum seven-and-a-half-second motion profile.
[initialState, goalState, limits] = jerkLimitedPlanInputs();
goalState.time_s = 12;
motionTypes = ["exactStop" "velocityCarrying"];
for motionIndex = 1:numel(motionTypes)
    options = plannerTestOptions();
    options.MotionType = motionTypes(motionIndex);
    options.GoalTimeMode = "fixedArrival";
    options.SampleTime_s = 0.3;
    result = planAzElMotion( ...
        [], initialState, goalState, limits, options);
    timedPath = result.timedSlopePath;
    holdSamples = timedPath.time_s <= ...
        timedPath.MotionStartTime_s + 1e-10;

    testCase.verifyTrue(result.Success);
    testCase.verifyTrue(result.Validation.Passed);
    testCase.verifyEqual(result.goalLineInterceptTime_s, 12, ...
        "AbsTol", 1e-10);
    testCase.verifyEqual(timedPath.GoalLineInterceptTime_s, 12, ...
        "AbsTol", 1e-10);
    testCase.verifyEqual(timedPath.MinimumMotionDuration_s, 7.5, ...
        "AbsTol", 1e-10);
    testCase.verifyEqual(timedPath.WaitDuration_s, 4.5, ...
        "AbsTol", 1e-10);
    testCase.verifyEqual(timedPath.MotionStartTime_s, 4.5, ...
        "AbsTol", 1e-10);
    testCase.verifyTrue(any(holdSamples));
    testCase.verifyEqual(timedPath.position_deg(holdSamples, :), ...
        zeros(nnz(holdSamples), 2), "AbsTol", 1e-12);
    testCase.verifyEqual(timedPath.velocity_deg_s(holdSamples, :), ...
        zeros(nnz(holdSamples), 2), "AbsTol", 1e-12);
    testCase.verifyEqual(timedPath.acceleration_deg_s2(holdSamples, :), ...
        zeros(nnz(holdSamples), 2), "AbsTol", 1e-12);
    testCase.verifyEqual(timedPath.jerk_deg_s3(holdSamples, :), ...
        zeros(nnz(holdSamples), 2), "AbsTol", 1e-12);
    testCase.verifyTrue(result.Validation.JerkWithinLimits);
    testCase.verifyTrue( ...
        timedPath.ConstraintDiagnostics.FiniteJerkCertified);
    closeTestFigures();
end
end

function testJerkKinematicPlotUsesFourPanelSchema(testCase)
%% Section 0: Header & Readme
% Verify the standard kinematic output publishes jerk as a fourth panel.
[initialState, goalState, limits] = jerkLimitedPlanInputs();
options = plannerTestOptions();
options.MotionType = "velocityCarrying";
options.SampleTime_s = 0.8;
options.ShowKinematicPlot = true;
result = planAzElMotion([], initialState, goalState, limits, options);
kinematicPlot = result.kinematicPlot;

testCase.verifyTrue(result.Success);
testCase.verifySize(kinematicPlot.Axes, [4 1]);
testCase.verifySize(kinematicPlot.PositionLines, [2 1]);
testCase.verifySize(kinematicPlot.VelocityLines, [2 1]);
testCase.verifySize(kinematicPlot.AccelerationLines, [2 1]);
testCase.verifySize(kinematicPlot.JerkLines, [2 1]);
testCase.verifyTrue(all(isgraphics(kinematicPlot.Axes)));
testCase.verifyTrue(all(isgraphics(kinematicPlot.JerkLines)));
testCase.verifyEqual(kinematicPlot.Diagnostics, ...
    result.timedSlopePath.ConstraintDiagnostics);
testCase.verifyEqual(string(get(kinematicPlot.Axes(4), "YLabel").String), ...
    "Jerk (deg/s^3)");
end

function testJerkLimitValidationRejectsNonpositiveValues(testCase)
%% Section 0: Header & Readme
% Verify a finite jerk constraint must be strictly positive on every axis.
[initialState, goalState, limits] = jerkLimitedPlanInputs();
limits.maxJerk_deg_s3 = [2 0];
testCase.verifyError(@() planAzElMotion( ...
    [], initialState, goalState, limits, plannerTestOptions()), ...
    "MATLAB:expectedPositive");
end

function testMixedFiniteJerkAxisAllowsInactiveHorizontalAxis(testCase)
%% Section 0: Header & Readme
% Verify one finite axis activates jerk retiming without constraining an
% orthogonal straight-line motion whose realized jerk is exactly zero.
[initialState, goalState, limits] = jerkLimitedPlanInputs();
limits.maxJerk_deg_s3 = [Inf 2];
motionTypes = ["exactStop" "velocityCarrying"];
for motionIndex = 1:numel(motionTypes)
    options = plannerTestOptions();
    options.MotionType = motionTypes(motionIndex);
    result = planAzElMotion( ...
        [], initialState, goalState, limits, options);
    timedPath = result.timedSlopePath;

    testCase.verifyTrue(result.Success);
    testCase.verifyTrue(result.Validation.Passed);
    testCase.verifyTrue(result.Validation.JerkWithinLimits);
    testCase.verifyTrue( ...
        timedPath.ConstraintDiagnostics.JerkConstrained);
    testCase.verifyTrue( ...
        timedPath.ConstraintDiagnostics.FiniteJerkCertified);
    testCase.verifyEqual(timedPath.jerk_deg_s3(:, 2), ...
        zeros(size(timedPath.time_s)), "AbsTol", 1e-12);
    testCase.verifyEqual( ...
        timedPath.ConstraintDiagnostics.PeakJerk_deg_s3(2), 0, ...
        "AbsTol", 1e-12);
    closeTestFigures();
end
end

function testJerkLimitedVelocityCarryingRetimeCoversRoundedDetour(testCase)
%% Section 0: Header & Readme
% Exercise finite-jerk retiming on a real obstacle detour. Quintic G3
% blends remove the line/curve curvature jumps, so velocity remains
% positive through rounded joins while Cartesian jerk stays bounded.
uBoundary_deg = [ ...
    -8,  7; -5,  7; -5, -4; 5, -4; ...
     5,  7;  8,  7;  8, -7; -8, -7];
obstacle = makeAzElObstacleData( ...
    "jerk U detour", [0; 120], uBoundary_deg(:, 1), ...
    uBoundary_deg(:, 2), 0.2);
initialState = struct( ...
    "time_s", 0, ...
    "position_deg", [0 0], ...
    "velocity_deg_s", [0 0], ...
    "acceleration_deg_s2", [0 0]);
goalState = struct( ...
    "time_s", 120, ...
    "position_deg", [0 -10], ...
    "velocity_deg_s", [0 0], ...
    "acceleration_deg_s2", [0 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [0.75 0.75], ...
    "maxJerk_deg_s3", [2.5 2.5]);
options = plannerTestOptions();
options.MotionType = "velocityCarrying";
result = planAzElMotion( ...
    obstacle, initialState, goalState, limits, options);
timedPath = result.timedSlopePath;
primitiveTypes = string({result.smoothPath.Primitives.Type});
internalNodes = 2:(numel(timedPath.CurveSpeed_deg_s) - 1);

testCase.verifyTrue(result.Success);
testCase.verifyTrue(result.Validation.Passed);
testCase.verifyTrue(any(result.directBlocked));
testCase.verifyTrue(any(primitiveTypes == "quintic"));
testCase.verifyEqual(timedPath.CurvatureDiscontinuityStopCount, 0);
testCase.verifyNotEmpty(internalNodes);
testCase.verifyGreaterThan(min( ...
    timedPath.CurveSpeed_deg_s(internalNodes)), 1e-6);
testCase.verifyTrue( ...
    timedPath.ConstraintDiagnostics.VelocityCarriedAcrossG3Joins);
testCase.verifyGreaterThan( ...
    timedPath.ConstraintDiagnostics.MinimumG3JoinSpeed_deg_s, 1e-6);
testCase.verifyEqual( ...
    timedPath.ConstraintDiagnostics.JoinContinuityOrder, "G3");
testCase.verifyTrue( ...
    timedPath.ConstraintDiagnostics.FiniteJerkNumericallyVerified);
testCase.verifyTrue(result.Validation.JerkWithinLimits);
testCase.verifyTrue(timedPath.ConstraintDiagnostics.JerkSatisfied);
testCase.verifyFalse( ...
    timedPath.ConstraintDiagnostics.ContinuousJerkCertified);
testCase.verifyLessThanOrEqual(max(abs( ...
    timedPath.jerk_deg_s3), [], 1), ...
    limits.maxJerk_deg_s3 + 1e-8);
end

function testJerkLimitedExactStopRetimeCoversMultiSegmentDetour(testCase)
%% Section 0: Header & Readme
% Verify finite-jerk exact-stop retiming stops with zero acceleration at
% every visibility corner while preserving the analytic jerk certificate.
uBoundary_deg = [ ...
    -8,  7; -5,  7; -5, -4; 5, -4; ...
     5,  7;  8,  7;  8, -7; -8, -7];
obstacle = makeAzElObstacleData( ...
    "exact-stop jerk U detour", [0; 120], uBoundary_deg(:, 1), ...
    uBoundary_deg(:, 2), 0.2);
initialState = struct( ...
    "time_s", 0, "position_deg", [0 0], ...
    "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
goalState = struct( ...
    "time_s", 120, "position_deg", [0 -10], ...
    "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [0.75 0.75], ...
    "maxJerk_deg_s3", [2.5 2.5]);
options = plannerTestOptions();
options.MotionType = "exactStop";
result = planAzElMotion( ...
    obstacle, initialState, goalState, limits, options);
timedPath = result.timedSlopePath;
internalNodes = 2:(numel(timedPath.CurveSpeed_deg_s) - 1);

testCase.verifyTrue(result.Success);
testCase.verifyTrue(result.Validation.Passed);
testCase.verifyTrue(any(result.directBlocked));
testCase.verifyGreaterThan(size(result.selectedRoute_deg, 1), 2);
testCase.verifyNotEmpty(internalNodes);
testCase.verifyEqual(timedPath.CurveSpeed_deg_s(internalNodes), ...
    zeros(numel(internalNodes), 1), "AbsTol", 1e-12);
testCase.verifyEqual( ...
    timedPath.CurveTangentialAcceleration_deg_s2(internalNodes), ...
    zeros(numel(internalNodes), 1), "AbsTol", 1e-12);
testCase.verifyEqual( ...
    timedPath.ConstraintDiagnostics.MandatoryStopCount, ...
    numel(internalNodes));
testCase.verifyEqual( ...
    timedPath.ConstraintDiagnostics.MandatoryStopArcLength_deg, ...
    timedPath.CurveArcLength_deg(internalNodes), "AbsTol", 1e-12);
testCase.verifyTrue( ...
    timedPath.ConstraintDiagnostics.FiniteJerkCertified);
testCase.verifyTrue( ...
    timedPath.ConstraintDiagnostics.FiniteJerkNumericallyVerified);
testCase.verifyFalse( ...
    timedPath.ConstraintDiagnostics.RoundedVelocityCarried);
testCase.verifyTrue(result.Validation.JerkWithinLimits);
end

function testExtremeVisibilityCandidatesStayBoundedForDensePolygon(testCase)
%% Section 0: Header & Readme
% Verify graph-node count depends on the configured silhouette budget, not
% on the number of vertices retained by canonical collision geometry.
verticesPerSide = 3000;
bottomX = linspace(-2, 2, verticesPerSide).';
rightY = linspace(-1, 1, verticesPerSide).';
topX = linspace(2, -2, verticesPerSide).';
leftY = linspace(1, -1, verticesPerSide).';
azimuth_deg = [bottomX; ...
    2 * ones(verticesPerSide - 1, 1); ...
    topX(2:end); ...
    -2 * ones(verticesPerSide - 2, 1)];
elevation_deg = [-ones(verticesPerSide, 1); ...
    rightY(2:end); ...
    ones(verticesPerSide - 1, 1); ...
    leftY(2:end - 1)];
obstacle = makeAzElObstacleData( ...
    "dense polygon", [0; 30], azimuth_deg, elevation_deg, 0);
obstacleField = buildAzElTimeObstacleField(obstacle);
startState = struct("time_s", 0, "position_deg", [0 -4]);
goalState = struct("time_s", 30, "position_deg", [0 4]);
directionCount = 16;
maximumTangencies = 2;
spaceView = visualizeAzElTimeSpace( ...
    obstacleField, startState, goalState, struct( ...
    "FigureVisible", "off", ...
    "ShowSweptSurfaces", false, ...
    "MaximumDisplayedSlicesPerObstacle", 1, ...
    "VisibilitySampleStep_deg", 1.0, ...
    "PolygonCandidateMode", "extreme", ...
    "ExtremeDirectionCount", directionCount, ...
    "MaximumTangenciesPerReference", maximumTangencies));

diagnostic = spaceView.CandidateReductionDiagnostics;
candidateBound = directionCount + 2 * maximumTangencies;
testCase.verifyEqual(numel(diagnostic), 1);
testCase.verifyGreaterThan(diagnostic.InputVertexCount, 11000);
testCase.verifyLessThanOrEqual( ...
    diagnostic.SelectedCandidateCount, candidateBound);
testCase.verifyGreaterThan(diagnostic.ReductionPercent, 99.8);
testCase.verifyEqual(spaceView.Options.PolygonCandidateMode, "extreme");
testCase.verifyLessThanOrEqual( ...
    size(spaceView.VisibilityGraphs(1).NodePosition_deg, 1), ...
    candidateBound + 2);
testCase.verifyLessThanOrEqual( ...
    spaceView.VisibilityGraphs(1).GeneratedVisibilityEdgeCount, ...
    nchoosek(candidateBound + 2, 2));
end

function testExamplesToggleJerkAndKeepVisibilityPlotTemplate(testCase)
%% Section 0: Header & Readme
% Verify the shared example-only jerk switch and the standard az/el/time
% visibility, boundary-edge, selected-route, and direct-line graphics.
baseOptions = struct( ...
    "FigureVisible", "off", ...
    "ShowAnimation", false, ...
    "ShowKinematicPlot", false, ...
    "Verbose", false);
jerkOnOptions = baseOptions;
jerkOnOptions.EnableJerkConstraint = true;
jerkOnOptions.MaxJerk_deg_s3 = [2.5 2.5];
jerkOnResult = exampleUShapedAzElTimeSpace(jerkOnOptions);

jerkOffOptions = baseOptions;
jerkOffOptions.EnableJerkConstraint = false;
jerkOffResult = exampleTwoOpposingUVisibilityGraph(jerkOffOptions);

testCase.verifyTrue( ...
    jerkOnResult.ExampleConfiguration.JerkConstraintEnabled);
testCase.verifyEqual(jerkOnResult.limits.maxJerk_deg_s3, [2.5 2.5]);
testCase.verifyTrue( ...
    jerkOnResult.timedSlopePath.ConstraintDiagnostics.JerkConstrained);
testCase.verifyFalse( ...
    jerkOffResult.ExampleConfiguration.JerkConstraintEnabled);
testCase.verifyEqual(jerkOffResult.limits.maxJerk_deg_s3, [Inf Inf]);
testCase.verifyFalse( ...
    jerkOffResult.timedSlopePath.ConstraintDiagnostics.JerkConstrained);

exampleResults = {jerkOnResult, jerkOffResult};
for resultIndex = 1:numel(exampleResults)
    result = exampleResults{resultIndex};
    viewResult = result.spaceView;
    testCase.verifyTrue(isgraphics(viewResult.Figure));
    testCase.verifyTrue(isgraphics(viewResult.Axes));
    testCase.verifyTrue(isgraphics(viewResult.DirectLine));
    testCase.verifyTrue(viewResult.Options.ShowVisibilityGraph);
    testCase.verifyTrue(viewResult.Options.ShowBoundaryConnections);
    testCase.verifyNotEmpty(viewResult.CandidatePointsAzElTime);
    testCase.verifyNotEmpty(viewResult.VisibilityGraphs);
    testCase.verifyNotEmpty( ...
        viewResult.VisibilityGraphHandles.VisibilityEdges);
    testCase.verifyNotEmpty( ...
        viewResult.VisibilityGraphHandles.BoundaryEdges);
    testCase.verifyNotEmpty( ...
        viewResult.VisibilityGraphHandles.SelectedPaths);
end
end

function testMovingSlicesReduceIndependentlyAndSkipEnclosedRegions(testCase)
%% Section 0: Header & Readme
% Verify every deformed slice gets a fresh bounded candidate set and that
% an enclosed free-space ring contributes no unreachable graph nodes.
sampleTime_s = [0; 5; 10];
outerSamplesAz_deg = cell(3, 1);
outerSamplesEl_deg = cell(3, 1);
verticesPerSide = 300;
for sampleIndex = 1:3
    phase = 0.5 * (sampleIndex - 1);
    bottomX = linspace(-2.5, 2.5, verticesPerSide).';
    rightY = linspace(-1.5, 1.5, verticesPerSide).';
    topX = linspace(2.5, -2.5, verticesPerSide).';
    leftY = linspace(1.5, -1.5, verticesPerSide).';
    outerAz = [bottomX; ...
        2.5 * ones(verticesPerSide - 1, 1); ...
        topX(2:end); ...
        -2.5 * ones(verticesPerSide - 2, 1)];
    outerEl = [-1.5 * ones(verticesPerSide, 1); ...
        rightY(2:end); ...
        1.5 * ones(verticesPerSide - 1, 1); ...
        leftY(2:end - 1)];
    outerAz = (1 + 0.04 * phase) .* outerAz + ...
        0.08 * phase .* outerEl + 0.15 * phase;
    outerEl = (1 - 0.03 * phase) .* outerEl + ...
        0.04 * sin(2 * outerAz + phase);
    holeAngle = linspace(0, 2 * pi, 181).';
    holeAngle(end) = [];
    holeAz = 0.65 * cos(holeAngle) - 0.10 * phase;
    holeEl = 0.45 * sin(holeAngle) + 0.08 * phase;
    outerSamplesAz_deg{sampleIndex} = [outerAz; NaN; holeAz];
    outerSamplesEl_deg{sampleIndex} = [outerEl; NaN; holeEl];
end
obstacle = makeAzElObstacleData( ...
    "moving dense ring", sampleTime_s, ...
    outerSamplesAz_deg, outerSamplesEl_deg, 0);
field = buildAzElTimeObstacleField(obstacle);
startState = struct("time_s", 0, "position_deg", [0 -3]);
goalState = struct("time_s", 10, "position_deg", [0 3]);
viewResult = visualizeAzElTimeSpace(field, startState, goalState, struct( ...
    "FigureVisible", "off", ...
    "ShowSweptSurfaces", false, ...
    "MaximumDisplayedSlicesPerObstacle", 3, ...
    "ExtremeDirectionCount", 12, ...
    "MaximumTangenciesPerReference", 2, ...
    "BoundaryRouteReductionTolerance_deg", 0.15));

diagnostic = viewResult.CandidateReductionDiagnostics;
testCase.verifyEqual(numel(diagnostic), 6);
for sampleIndex = 1:3
    onSlice = [diagnostic.SampleIndex] == sampleIndex;
    sliceDiagnostic = diagnostic(onSlice);
    testCase.verifyEqual(nnz([sliceDiagnostic.Mode] == ...
        "inaccessibleRegion"), 1);
    testCase.verifyEqual(sum( ...
        [sliceDiagnostic([sliceDiagnostic.Mode] == ...
        "inaccessibleRegion").SelectedCandidateCount]), 0);
    exposedDiagnostic = sliceDiagnostic( ...
        [sliceDiagnostic.Mode] ~= "inaccessibleRegion");
    testCase.verifyLessThanOrEqual( ...
        exposedDiagnostic.SelectedCandidateCount, 16);
    testCase.verifyLessThanOrEqual( ...
        exposedDiagnostic.GraphActiveCandidateCount, ...
        exposedDiagnostic.SelectedCandidateCount);
end
testCase.verifyEqual(numel(viewResult.VisibilityGraphs), 3);
testCase.verifyTrue(all([viewResult.VisibilityGraphs.Success]));
testCase.verifyTrue(all( ...
    [viewResult.VisibilityGraphs.BoundaryRouteRetainedVertexCount] < ...
    [viewResult.VisibilityGraphs.BoundaryRouteInputVertexCount]));
firstSliceCandidates = viewResult.CandidatePointsAzElTime( ...
    viewResult.CandidateSampleIndex == 1, 1:2);
finalSliceCandidates = viewResult.CandidatePointsAzElTime( ...
    viewResult.CandidateSampleIndex == 3, 1:2);
testCase.verifyNotEqual(firstSliceCandidates, finalSliceCandidates);
end

function testMovingSnapshotRoutesAreConsolidatedBeforeRetiming(testCase)
%% Section 0: Header & Readme
% Keep every moving-obstacle visibility graph for inspection while bounding
% the number of distinct graph routes sent through kinematic retiming.
obstacleTime_s = (0:5:40).';
circleAngle_rad = linspace(0, 2 * pi, 41).';
circleAngle_rad(end) = [];
circleAzimuth_deg = cell(numel(obstacleTime_s), 1);
circleElevation_deg = cell(numel(obstacleTime_s), 1);
for sampleIndex = 1:numel(obstacleTime_s)
    centerElevation_deg = -0.8 + ...
        1.6 * obstacleTime_s(sampleIndex) / obstacleTime_s(end);
    circleAzimuth_deg{sampleIndex} = 1.5 * cos(circleAngle_rad);
    circleElevation_deg{sampleIndex} = ...
        centerElevation_deg + 1.5 * sin(circleAngle_rad);
end

obstacle = makeAzElObstacleData( ...
    "slowly moving regression circle", obstacleTime_s, ...
    circleAzimuth_deg, circleElevation_deg, 0.10);
initialState = struct("time_s", 0, "position_deg", [-5 0]);
goalState = struct("time_s", 40, "position_deg", [5 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1]);
options = plannerTestOptions();
options.MaximumDisplayedSlicesPerObstacle = numel(obstacleTime_s);
options.MaximumRetimedVisibilityRoutes = 3;
result = planAzElMotion( ...
    obstacle, initialState, goalState, limits, options);

testCase.verifyTrue(result.Success, result.Message);
testCase.verifyTrue(result.Validation.Passed, result.Validation.Message);
testCase.verifyEqual( ...
    numel(result.spaceView.VisibilityGraphs), numel(obstacleTime_s));
testCase.verifyGreaterThan( ...
    result.visibilityRouteConsolidation.DistinctRouteCount, ...
    options.MaximumRetimedVisibilityRoutes);
testCase.verifyLessThanOrEqual( ...
    numel(result.retimedVisibilityGraphIndices), ...
    options.MaximumRetimedVisibilityRoutes);
testCase.verifyEqual( ...
    result.visibilityRouteConsolidation.SelectedRouteCount, ...
    numel(result.retimedVisibilityGraphIndices));
testCase.verifyTrue(all(ismember( ...
    result.retimedVisibilityGraphIndices, ...
    find([result.spaceView.VisibilityGraphs.Success]))));
end

function testExplorerDefaultsToTenMovingSlices(testCase)
%% Section 0: Header & Readme
% Verify long obstacle histories remain intact while the default explorer
% uses ten evenly distributed slices to control plot and graph clutter.
sampleTime_s = (0:20).';
azimuthBySlice_deg = cell(numel(sampleTime_s), 1);
elevationBySlice_deg = cell(numel(sampleTime_s), 1);
for sampleIndex = 1:numel(sampleTime_s)
    centerAzimuth_deg = 0.02 * sampleTime_s(sampleIndex);
    azimuthBySlice_deg{sampleIndex} = ...
        centerAzimuth_deg + [-1; 1; 1; -1];
    elevationBySlice_deg{sampleIndex} = [-1; -1; 1; 1];
end
obstacle = makeAzElObstacleData( ...
    "twenty-one-slice obstacle", sampleTime_s, ...
    azimuthBySlice_deg, elevationBySlice_deg, 0);
obstacleField = buildAzElTimeObstacleField(obstacle);
startState = struct("time_s", 0, "position_deg", [-3 0]);
goalState = struct("time_s", 20, "position_deg", [3 0]);
viewResult = visualizeAzElTimeSpace( ...
    obstacleField, startState, goalState, struct( ...
    "FigureVisible", "off", "ShowSweptSurfaces", false));

testCase.verifyEqual(numel(obstacleField.Obstacles.TimeSeconds), 21);
testCase.verifyEqual( ...
    viewResult.Options.MaximumDisplayedSlicesPerObstacle, 10);
testCase.verifyEqual(numel(viewResult.VisibilityGraphs), 10);
testCase.verifyEqual(viewResult.VisibilityGraphs(1).Time_s, 0);
testCase.verifyEqual(viewResult.VisibilityGraphs(end).Time_s, 20);
end

function testAnimationDefaultsFavorFastPlayback(testCase)
%% Section 0: Header & Readme
% Keep planner-driven and direct animation defaults synchronized and fast.
plannerOptions = planAzElMotion();
animationOptions = animateAzElTimedSlopePath();
testCase.verifyEqual(plannerOptions.AnimationFrameStride, 10);
testCase.verifyEqual(plannerOptions.AnimationPause_s, 0.001);
testCase.verifyEqual(animationOptions.FrameStride, ...
    plannerOptions.AnimationFrameStride);
testCase.verifyEqual(animationOptions.Pause_s, ...
    plannerOptions.AnimationPause_s);
end

function testGenericMovingObstacleConstructorAllowsIndependentShapes(testCase)
%% Section 0: Header & Readme
% Verify the production constructor, rather than an example loop, owns
% arbitrary slice generation and accepts different vertex counts per time.
sourcePosition_deg = [ ...
    -1 -1; 1 -1; 1 1; -1 1];
sampleTime_s = [0; 5; 10];
sliceTransform = @(source_deg, time_s, sampleIndex) ...
    independentlyShapedSlice(source_deg, time_s, sampleIndex);
[obstacle, history] = makeMovingAzElObstacleData( ...
    "independently shaped moving obstacle", sampleTime_s, ...
    sourcePosition_deg(:, 1), sourcePosition_deg(:, 2), ...
    sliceTransform, 0.15, struct( ...
    "UseParallel", false, "Verbose", false));

testCase.verifyEqual(history.vertexCount, [4; 5; 6]);
testCase.verifyEqual(history.time_s, sampleTime_s);
testCase.verifyFalse(history.ParallelExecution.Enabled);
testCase.verifyEqual(numel(obstacle.time_s), 3);
testCase.verifyEqual(obstacle.safetyMargin_deg, 0.15);
originalVertexCount = zeros(3, 1);
for sampleIndex = 1:3
    originalVertexCount(sampleIndex) = nnz(isfinite( ...
        obstacle.originalAz_deg{sampleIndex}));
end
testCase.verifyEqual(originalVertexCount, [4; 5; 6]);
testCase.verifyGreaterThan(range(history.area_deg2), 0);
testCase.verifyGreaterThan(range(history.centroid_deg(:, 1)), 0);
end

function testExampleValidationIndependentlyChecksReturnedMotion(testCase)
%% Section 0: Header & Readme
% Verify example validation checks public trajectory data independently of
% the planner-owned Success and Validation fields.
[initialState, goalState, limits, options] = directPlanInputs();
result = planAzElMotion( ...
    [], initialState, goalState, limits, options);
validation = validateAzElExampleResult(result, "direct regression");
testCase.verifyTrue(validation.Passed, validation.Message);
testCase.verifyTrue(validation.CollisionFree);
testCase.verifyEqual(result.TerminationReason, "goalReached");
testCase.verifyGreaterThanOrEqual(result.ElapsedPlanningTime_s, 0);

tamperedResult = result;
tamperedResult.timedSlopePath.velocity_deg_s(2, 1) = ...
    2 * limits.maxVelocity_deg_s(1);
tamperedValidation = validateAzElExampleResult( ...
    tamperedResult, "tampered direct regression");
testCase.verifyFalse(tamperedValidation.Passed);
testCase.verifyFalse(tamperedValidation.VelocityWithinLimits);
testCase.verifyTrue(any(tamperedValidation.Issues == ...
    "velocityWithinLimits"));
end

function obstacle = makeSquareObstacle( ...
        obstacleName, center_deg, safetyMargin_deg, time_s)
%% Section 0: Header & Readme
% Construct one canonical two-degree square for focused tests.
[azimuth_deg, elevation_deg] = squareBoundary(center_deg);
obstacle = makeAzElObstacleData( ...
    obstacleName, time_s, azimuth_deg, elevation_deg, safetyMargin_deg);
end

function [azimuth_deg, elevation_deg] = squareBoundary(center_deg)
%% Section 0: Header & Readme
% Return a counterclockwise square centered at the supplied az/el point.
vertices_deg = reshape(double(center_deg), 1, 2) + [ ...
    -1 -1; 1 -1; 1 1; -1 1];
azimuth_deg = vertices_deg(:, 1);
elevation_deg = vertices_deg(:, 2);
end

function position_deg = independentlyShapedSlice( ...
        sourcePosition_deg, sampleTime_s, sampleIndex)
%% Section 0: Header & Readme
% Return an ordered polygon whose topology and vertex count vary by slice.
vertexCount = 3 + sampleIndex;
angle_rad = (0:vertexCount - 1).' * 2 * pi / vertexCount;
sourceScale = max(abs(sourcePosition_deg), [], "all");
radius_deg = sourceScale * (1 + 0.05 * sampleIndex);
position_deg = [ ...
    0.05 * sampleTime_s + radius_deg * cos(angle_rad), ...
    radius_deg * sin(angle_rad)];
end

function [initialState, goalState, limits, options] = directPlanInputs()
%% Section 0: Header & Readme
% Return a short unobstructed exact-stop planning request.
initialState = struct("time_s", 0, "position_deg", [-1 0]);
goalState = struct("time_s", 8, "position_deg", [1 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1]);
options = plannerTestOptions();
end

function [initialState, goalState, limits] = jerkLimitedPlanInputs()
%% Section 0: Header & Readme
% Return the analytic 7.5-second direct-line S-curve regression case.
initialState = struct( ...
    "time_s", 0, ...
    "position_deg", [0 0], ...
    "velocity_deg_s", [0 0], ...
    "acceleration_deg_s2", [0 0]);
goalState = struct( ...
    "time_s", 30, ...
    "position_deg", [10 0], ...
    "velocity_deg_s", [0 0], ...
    "acceleration_deg_s2", [0 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1], ...
    "maxJerk_deg_s3", [2 2]);
end

function options = plannerTestOptions()
%% Section 0: Header & Readme
% Disable interactive rendering while retaining the normal planner path.
options = struct( ...
    "MotionType", "exactStop", ...
    "SampleTime_s", 0.2, ...
    "FigureVisible", "off", ...
    "ShowAnimation", false, ...
    "ShowKinematicPlot", false, ...
    "ShowSweptSurfaces", false, ...
    "MaximumDisplayedSlicesPerObstacle", 2, ...
    "Verbose", false, ...
    "Title", "migration regression test");
end

function verifyLineStyles(testCase, handles, expectedStyle)
%% Section 0: Header & Readme
% Require at least one live handle and one exact style on every handle.
testCase.verifyNotEmpty(handles);
for handleIndex = 1:numel(handles)
    testCase.verifyTrue(isgraphics(handles(handleIndex)));
    testCase.verifyEqual( ...
        string(get(handles(handleIndex), "LineStyle")), expectedStyle);
end
end

function closeTestFigures()
%% Section 0: Header & Readme
% Close all figures created by planner diagnostics and display tests.
figureHandles = findall(groot, "Type", "figure");
if ~isempty(figureHandles)
    close(figureHandles);
end
end
