function tests = testAzElPlannerMigration
%% Section 0: Header & Readme
% SYNTAX
%   results = runtests("tests/testAzElPlannerMigration.m")
%**************************************************************************
% PURPOSE
%   - Guard construction-time safety-margin ownership, migration errors,
%     display conventions, and deterministic planner progress reporting.
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
