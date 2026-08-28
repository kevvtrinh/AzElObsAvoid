function tests = testObstacleAvoidanceSandboxDiagnosis
%% Section 0: Header & Readme
% SYNTAX
%   tests = testObstacleAvoidanceSandboxDiagnosis
%**************************************************************************
% PURPOSE
%   - Verify sandbox diagnosis bundles preserve pre-run requests plus
%     successful and failed planner calls without graphics handles.
%   - Verify a reloaded success bundle can reproduce the exact public call.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest function test array)
%**************************************************************************
% UNITS
%   - Test positions are degrees and time is seconds.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
% Create temporary output paths for saved diagnosis data. These tests check
% export, import, replay, request preservation, and result preservation. The
% failed test name identifies which transfer step to inspect.
% Add production, examples, and sandbox folders for public-call tests.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot);
addpath(fullfile(repositoryRoot, "trajectory"));
addpath(fullfile(repositoryRoot, "examples"));
addpath(fullfile(repositoryRoot, "sandbox"));
testCase.TestData.RepositoryRoot = repositoryRoot;
end

function testSuccessfulBundleRoundTripsAndReproduces(testCase)
% Preserve exact success inputs/result and replay the public planner call.
[initialState, goalState, limits, options] = simpleRequest();
result = obstacleAvoidance.planTrajectory( ...
    [], initialState, goalState, limits, options);
verifyTrue(testCase, result.Success, result.Message);
validation = obstacleAvoidance.validateTrajectory(result);
verifyTrue(testCase, validation.Passed, validation.Message);
sandboxState = syntheticSandboxState( ...
    result, validation, initialState, goalState, obstacleAvoidance.obstacles.combineObstacles());
filePath = string(tempname) + ".mat";
testCase.addTeardown(@() deleteIfPresent(filePath));

exportInfo = exportSandboxDiagnosis( ...
    filePath, sandboxState, "goal");
loaded = load(char(filePath), "diagnosisBundle");
bundle = loaded.diagnosisBundle;
verifyEqual(testCase, exportInfo.Format, ...
    "obstacleAvoidanceSandboxDiagnosis-v1");
verifyGreaterThan(testCase, exportInfo.Bytes, 0);
verifyTrue(testCase, exportInfo.HasPlannerResult);
verifyEqual(testCase, bundle.Mode, "goal");
verifyEqual(testCase, bundle.PlannerInputs.initialState, ...
    result.Inputs.initialState);
verifyEqual(testCase, bundle.PlannerInputs.goalState, ...
    result.Inputs.goalState);
verifyEqual(testCase, bundle.Result.TerminationReason, "goalReached");
verifyTrue(testCase, bundle.IndependentValidation.Passed);
verifyFalse(testCase, isfield(bundle.Scene, "GraphicsHandles"));

reproduced = obstacleAvoidance.planTrajectory( ...
    bundle.PlannerInputs.obstacles, ...
    bundle.PlannerInputs.initialState, ...
    bundle.PlannerInputs.goalState, ...
    bundle.PlannerInputs.limits, bundle.PlannerOptions);
reproducedValidation = obstacleAvoidance.validateTrajectory(reproduced);
verifyTrue(testCase, reproduced.Success, reproduced.Message);
verifyTrue(testCase, reproducedValidation.Passed, ...
    reproducedValidation.Message);
verifyEqual(testCase, reproduced.ArrivalTime_s, ...
    result.ArrivalTime_s, "AbsTol", 1e-9);
end

function testPersistentSandboxCreatesExportActions(testCase)
% Verify both tabs expose an export button and the direct export API.
sandboxState = obstacleAvoidanceSandbox( ...
    struct("FigureVisible", "off"));
testCase.addTeardown(@() closeIfPresent(sandboxState.FigureHandle));
currentState = sandboxState.ReadState();
verifyTrue(testCase, isfield( ...
    currentState.GoalMode.GraphicsHandles.Actions, "Export"));
verifyTrue(testCase, isfield( ...
    currentState.FreeMode.GraphicsHandles.Actions, "Export"));
verifyTrue(testCase, isfield( ...
    currentState.GoalMode.GraphicsHandles.Actions, "SetMotion"));
verifyTrue(testCase, isfield( ...
    currentState.FreeMode.GraphicsHandles.Actions, "SetMotion"));
constructorNames = [ ...
    "AddPolygon", "AddCircle", "AddHandDrawn", "AddSquare"];
constructorLabels = ["Polygon", "Circle", "Hand Drawn", "Square"];
verifyTrue(testCase, isgraphics( ...
    currentState.GoalMode.GraphicsHandles.AddPanel));
verifyEqual(testCase, get( ...
    currentState.GoalMode.GraphicsHandles.AddPanel, "Title"), 'Add');
for constructorIndex = 1:numel(constructorNames)
    constructorName = constructorNames(constructorIndex);
    verifyTrue(testCase, isfield( ...
        currentState.GoalMode.GraphicsHandles.Actions, constructorName));
    verifyEqual(testCase, string(get( ...
        currentState.GoalMode.GraphicsHandles.Actions.(constructorName), ...
        "String")), constructorLabels(constructorIndex));
end
verifyTrue(testCase, isgraphics( ...
    currentState.GoalMode.GraphicsHandles.Controls.MotionProfileHandle));
verifyEqual(testCase, ...
    currentState.GoalMode.PolygonMotionVectors_deg, zeros(0, 2));
verifyEqual(testCase, ...
    currentState.GoalMode.PolygonMotionProfiles, strings(0, 1));
verifyEqual(testCase, get( ...
    currentState.GoalMode.GraphicsHandles.Actions.Export, "Enable"), ...
    'off');
verifyEqual(testCase, get( ...
    currentState.FreeMode.GraphicsHandles.Actions.Export, "Enable"), ...
    'off');
verifyTrue(testCase, isa(currentState.ExportBundle, "function_handle"));
end

function testSandboxDefaultsBoundInteractivePlannerWork(testCase)
% Keep the UI-specific work limits separate from production planner defaults.
sandboxState = obstacleAvoidanceSandbox(struct("FigureVisible", "off"));
testCase.addTeardown(@() closeIfPresent(sandboxState.FigureHandle));
plannerOptions = sandboxState.Options.PlannerOptions;
verifyTrue(testCase, sandboxState.Options.AnimateOnRun);
verifyEqual(testCase, sandboxState.Options.AnimationFrameStride, 20);
verifyEqual(testCase, sandboxState.Options.AnimationPause_s, 0.001);
verifyEqual(testCase, plannerOptions.MaximumSeedCount, 3);
verifyEqual(testCase, plannerOptions.CollocationSegmentCount, 8);
verifyEqual(testCase, plannerOptions.MaximumCollocationSegmentCount, 16);
verifyEqual(testCase, plannerOptions.MaximumMeshRefinementPasses, 0);
verifyEqual(testCase, plannerOptions.MaximumNlpIterations, 80);
verifyEqual(testCase, plannerOptions.MaximumNlpFunctionEvaluations, 5000);
verifyEqual(testCase, plannerOptions.ArrivalTimeTolerance_s, 0.05);
productionOptions = obstacleAvoidance.planTrajectory();
verifyEqual(testCase, productionOptions.MaximumSeedCount, 5);
verifyEqual(testCase, productionOptions.MaximumMeshRefinementPasses, 2);
end

function testPreRunGoalBundlePreservesRequest(testCase)
% Export a complete Goal Mode request before any planner call occurs.
sandboxState = obstacleAvoidanceSandbox(struct("FigureVisible", "off"));
testCase.addTeardown(@() closeIfPresent(sandboxState.FigureHandle));
applicationState = guidata(sandboxState.FigureHandle);
applicationState.GoalMode.StartPosition_deg = [-3 1];
applicationState.GoalMode.GoalPosition_deg = [5 -2];
guidata(sandboxState.FigureHandle, applicationState);
currentState = sandboxState.ReadState();
filePath = string(tempname) + ".mat";
testCase.addTeardown(@() deleteIfPresent(filePath));

exportInfo = currentState.ExportBundle(filePath, "goal");

loaded = load(char(filePath), "diagnosisBundle");
bundle = loaded.diagnosisBundle;
verifyFalse(testCase, exportInfo.HasPlannerResult);
verifyEqual(testCase, exportInfo.TerminationReason, "notRun");
verifyEqual(testCase, bundle.PlanningState, "notRun");
verifyFalse(testCase, bundle.HasPlannerResult);
verifyEmpty(testCase, fieldnames(bundle.Result));
verifyEqual(testCase, ...
    bundle.PlannerInputs.initialState.position_deg, [-3 1]);
verifyEqual(testCase, ...
    bundle.PlannerInputs.goalState.position_deg, [5 -2]);
verifyFalse(testCase, isfield(bundle.PlannerOptions, "PlannerMethod"));
verifyTrue(testCase, bundle.ExportRequest.HasCompleteScene);
end

function testPolygonMotionProfilesFollowSelectedTiming(testCase)
% Verify the four sandbox motion choices use distinct, expected profiles.
polygon_deg = [0 0; 1 0; 1 1; 0 1];
time_s = [0; 10];
motionVector_deg = [4 2];
profiles = [ ...
    "nonzeroVelocity", "zeroStart", "trapezoidal", "oscillating"];
expectedMiddle_deg = [2 1; 1 0.5; 2 1; 4 2];
expectedFinal_deg = [4 2; 4 2; 4 2; 0 0];
for profileIndex = 1:numel(profiles)
    [profileTime_s, azimuthBySlice_deg, elevationBySlice_deg] = ...
        createSandboxPolygonMotionHistory( ...
            polygon_deg, time_s, motionVector_deg, ...
            profiles(profileIndex));
    middleIndex = ceil(numel(profileTime_s) / 2);
    firstVertex_deg = [ ...
        azimuthBySlice_deg{1}(1), elevationBySlice_deg{1}(1)];
    middleVertex_deg = [ ...
        azimuthBySlice_deg{middleIndex}(1), ...
        elevationBySlice_deg{middleIndex}(1)];
    finalVertex_deg = [ ...
        azimuthBySlice_deg{end}(1), elevationBySlice_deg{end}(1)];
    verifyEqual(testCase, firstVertex_deg, [0 0], "AbsTol", 1e-12);
    verifyEqual(testCase, middleVertex_deg, ...
        expectedMiddle_deg(profileIndex, :), "AbsTol", 1e-12);
    verifyEqual(testCase, finalVertex_deg, ...
        expectedFinal_deg(profileIndex, :), "AbsTol", 1e-12);
end
end

function testPublicExportBundleWritesCurrentState(testCase)
% Verify the public no-dialog hook writes the live guidata-backed result.
[initialState, goalState, limits, options] = simpleRequest();
result = obstacleAvoidance.planTrajectory([], initialState, goalState, limits, options);
validation = obstacleAvoidance.validateTrajectory(result);
sandboxState = obstacleAvoidanceSandbox(struct("FigureVisible", "off"));
testCase.addTeardown(@() closeIfPresent(sandboxState.FigureHandle));
applicationState = guidata(sandboxState.FigureHandle);
applicationState.GoalMode.LastPlannerResult = result;
applicationState.GoalMode.LastValidation = validation;
guidata(sandboxState.FigureHandle, applicationState);
currentState = sandboxState.ReadState();
filePath = string(tempname) + ".mat";
testCase.addTeardown(@() deleteIfPresent(filePath));

exportInfo = currentState.ExportBundle(filePath, "goal");

verifyTrue(testCase, isfile(filePath));
verifyGreaterThan(testCase, exportInfo.Bytes, 0);
loaded = load(char(filePath), "diagnosisBundle");
verifyEqual(testCase, loaded.diagnosisBundle.Result.TerminationReason, ...
    result.TerminationReason);
savedVariables = whos('-file', char(filePath));
verifyTrue(testCase, any(string({savedVariables.name}) == "diagnosisBundle"));
end

function testPolygonMotionHistoryCreatesQueryableObstacle(testCase)
% Verify a sandbox profile becomes the expected obstacle translation.
polygon_deg = [0 0; 1 0; 1 1; 0 1];
[time_s, azimuthBySlice_deg, elevationBySlice_deg] = ...
    createSandboxPolygonMotionHistory( ...
        polygon_deg, [0; 10], [4 2], "nonzeroVelocity");
obstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "sandbox moving polygon", time_s, ...
    azimuthBySlice_deg, elevationBySlice_deg, 0);
[shape, ~] = obstacleAvoidance.obstacles.shapeAtTime(obstacle, 5, false);
[centroidAzimuth_deg, centroidElevation_deg] = centroid(shape);
verifyEqual(testCase, ...
    [centroidAzimuth_deg, centroidElevation_deg], ...
    [2.5 1.5], "AbsTol", 1e-10);
end

function testFailedBundleRetainsDiagnosisEvidence(testCase)
% Preserve endpoint-blocked failure inputs, diagnostics, and validation.
[initialState, goalState, limits, options] = simpleRequest();
obstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "start enclosure", [0; goalState.time_s], ...
    [-1; 1; 1; -1], [-1; -1; 1; 1], 0);
result = obstacleAvoidance.planTrajectory( ...
    obstacle, initialState, goalState, limits, options);
verifyFalse(testCase, result.Success);
verifyEqual(testCase, result.TerminationReason, "endpointBlocked");
sandboxState = syntheticSandboxState( ...
    result, result.Validation, initialState, goalState, obstacle);
filePath = string(tempname) + ".mat";
testCase.addTeardown(@() deleteIfPresent(filePath));

exportInfo = exportSandboxDiagnosis( ...
    filePath, sandboxState, "goal");
loaded = load(char(filePath), "diagnosisBundle");
bundle = loaded.diagnosisBundle;
verifyFalse(testCase, exportInfo.PlannerSuccess);
verifyEqual(testCase, exportInfo.TerminationReason, "endpointBlocked");
verifyFalse(testCase, bundle.Result.Success);
verifyEqual(testCase, bundle.Result.TerminationReason, ...
    "endpointBlocked");
verifyNotEmpty(testCase, bundle.Result.Message);
verifyEqual(testCase, numel(bundle.PlannerInputs.obstacles), 1);
verifyTrue(testCase, isfield(bundle.Result, "SearchDiagnostics"));
end

function [initialState, goalState, limits, options] = simpleRequest()
% Build one deterministic rest-to-rest fixed-arrival request.
initialState = struct( ...
    "time_s", 0, "position_deg", [0 0], ...
    "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
goalState = struct( ...
    "time_s", 6, "position_deg", [4 0], ...
    "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1], ...
    "maxJerk_deg_s3", [2 2]);
options = obstacleAvoidance.planTrajectory();
options.GoalTimeMode = "fixedArrival";
end

function sandboxState = syntheticSandboxState( ...
        result, validation, initialState, goalState, obstacles)
% Build the stable saved sandbox data without graphics handles.
modeState = struct( ...
    "StartPosition_deg", initialState.position_deg, ...
    "GoalPosition_deg", goalState.position_deg, ...
    "WaypointPositions_deg", zeros(0, 2), ...
    "RawObstacleStrokes_deg", {cell(0, 1)}, ...
    "LineObstaclePositions_deg", {cell(0, 1)}, ...
    "PolygonObstaclePositions_deg", {cell(0, 1)}, ...
    "PolygonMotionVectors_deg", zeros(0, 2), ...
    "PolygonMotionProfiles", strings(0, 1), ...
    "SelectedPolygonIndex", 0, ...
    "CanonicalObstacles", obstacles, ...
    "SegmentResults", repmat(struct(), 0, 1), ...
    "CombinedTrajectory", struct( ...
        "time_s", zeros(0, 1), ...
        "position_deg", zeros(0, 2), ...
        "velocity_deg_s", zeros(0, 2), ...
        "acceleration_deg_s2", zeros(0, 2), ...
        "jerk_deg_s3", zeros(0, 2), ...
        "SegmentStartIndices", zeros(0, 1)), ...
    "LastPlannerResult", result, ...
    "LastValidation", validation, ...
    "Status", result.Message, ...
    "PlannerLog", "synthetic export regression", ...
    "ResolvedControls", struct());
sandboxState = struct( ...
    "Options", struct("PlannerOptions", result.Options), ...
    "GoalMode", modeState, "FreeMode", modeState);
end

function deleteIfPresent(filePath)
% Remove only the exact temporary MAT artifact created by one test.
if isfile(filePath)
    delete(filePath);
end
end


function closeIfPresent(figureHandle)
% Close only the hidden sandbox figure created by one UI smoke test.
if isgraphics(figureHandle)
    close(figureHandle);
end
end
