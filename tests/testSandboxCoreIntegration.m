function tests = testSandboxCoreIntegration
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testSandboxCoreIntegration.m')
% PURPOSE: Verify the port against real core motion, graph, and validation data.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Tests for detours, failure, replay, and native sandbox callbacks.
% UNITS: Coordinate units and seconds.

%% Section 1: Register Tests
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    repositoryRoot = fileparts(fileparts(mfilename('fullpath')));
    addpath(repositoryRoot, fullfile(repositoryRoot, 'trajectory'), ...
        fullfile(repositoryRoot, 'sandbox'), fullfile(repositoryRoot, 'offlinesandbox'));
    folder = tempname; mkdir(folder);
    testCase.TestData.Folder = folder;
    testCase.addTeardown(@() rmdir(folder, 's'));
end

function testDetourGraphC3AndReplay(testCase)
    request = sceneRequest();
    [response, bundle] = runRequest(testCase, request);
    verifyTrue(testCase, response.result.Success, response.result.Message);
    verifyTrue(testCase, response.validation.InterSegmentContinuous);
    verifyEqual(testCase, bundle.Result.Options.ConstraintTolerance, 1e-8);
    graph = bundle.Result.VisibilityGraph;
    verifyEqual(testCase, response.diagnosis.Search.NodeCount, size(graph.NodePosition_units, 1));
    verifyEqual(testCase, response.diagnosis.Search.AcceptedEdgeCount, size(graph.AcceptedNodeIndex, 1));
    verifyEqual(testCase, response.diagnosis.Search.RejectedTransitionCount, size(graph.RejectedNodeIndex, 1));
    verifyTrue(testCase, response.diagnosis.Search.GraphIsFullyEnumerated);
    verifyFalse(testCase, isfield(response.diagnosis, 'Attempts'));
    verifyEqual(testCase, response.result.ElapsedTime_s, bundle.Result.ElapsedTime_s);
    verifyEqual(testCase, response.result.MotionLength_units, bundle.Result.MotionLength_units);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(bundle.Result).Passed);
    tampered = bundle.Result;
    tampered.Polynomial.jerkPower_units_s3(2, 1, 1) = 1e4;
    verifyFalse(testCase, obstacleAvoidance.validateTrajectory(tampered).Passed);
    filePath = fullfile(testCase.TestData.Folder, 'detour.mat');
    diagnosisBundle = bundle; %#ok<NASGU>
    save(filePath, 'diagnosisBundle', '-v7.3');
    [replayed, replayBundle] = offlineSandbox.replayDiagnosisBundle(filePath, ...
        fullfile(testCase.TestData.Folder, 'replayed.json'));
    verifyTrue(testCase, replayed.validation.Passed);
    verifyEqual(testCase, replayBundle.PlannerInputs.goalState.time_s, request.goalState.time_s);
    verifyEqual(testCase, replayBundle.PlannerInputs.obstacles.originalX_units, bundle.PlannerInputs.obstacles.originalX_units);
    verifyEqual(testCase, replayBundle.PlannerInputs.obstacles.x_units, bundle.PlannerInputs.obstacles.x_units);
end

function testNoPathIsAStableBrowserResult(testCase)
    request = sceneRequest();
    request.obstacles.keyframes.vertices_units = [-1 -5; 1 -5; 1 5; -1 5];
    [response, bundle] = runRequest(testCase, request);
    verifyFalse(testCase, response.result.Success);
    verifyEqual(testCase, response.result.TerminationReason, "noVisibilityRoute");
    verifyFalse(testCase, response.validation.Passed);
    verifyEmpty(testCase, response.result.time_s);
    verifyFalse(testCase, obstacleAvoidance.validateTrajectory(bundle.Result).Passed);
end

function testInvalidRequestDoesNotReplaceOutput(testCase)
    request = sceneRequest();
    request.goalState.time_s = -1;
    requestPath = fullfile(testCase.TestData.Folder, 'invalid.json');
    resultPath = fullfile(testCase.TestData.Folder, 'retained.json');
    writeText(requestPath, jsonencode(request));
    writeText(resultPath, 'retained result');
    verifyError(testCase, @() offlineSandbox.runPlanningRequest(requestPath, resultPath), ...
        'planTrajectory:InvalidTimeOrder');
    verifyEqual(testCase, fileread(resultPath), 'retained result');
end

function testNativeDetourAndDiagnosticPlots(testCase)
    ui = obstacleAvoidanceSandbox(struct('FigureVisible', 'off', 'AnimateOnRun', false, ...
        'MissionTime_s', 12, 'MaxAcceleration_units_s2', [2 2], ...
        'MaxJerk_units_s3', [4 4], 'WorkspaceXInterval_units', [-6 6], ...
        'WorkspaceYInterval_units', [-4 4], 'ObstacleSafetyMargin_units', 0.25, ...
        'PlannerOptions', struct('GoalTimeMode', 'fixedArrival')));
    testCase.addTeardown(@() close(ui.FigureHandle));
    state = guidata(ui.FigureHandle);
    state.GoalMode.StartPosition_units = [-4 0];
    state.GoalMode.GoalPosition_units = [4 0];
    state.GoalMode.PolygonObstaclePositions_units = {[-1 -1; 1 -1; 1 1; -1 1]};
    state.GoalMode.PolygonMotionVectors_units = [0 0];
    state.GoalMode.PolygonMotionProfiles = "stationary";
    guidata(ui.FigureHandle, state);
    runHandle = state.GoalMode.GraphicsHandles.Actions.Run;
    callback = get(runHandle, 'Callback'); callback(runHandle, []);
    state = ui.ReadState();
    verifyTrue(testCase, state.GoalMode.LastPlannerResult.Success, state.GoalMode.Status);
    verifyTrue(testCase, state.GoalMode.LastValidation.InterSegmentContinuous);
    diagnosticsHandle = state.GoalMode.GraphicsHandles.Actions.Diagnostics;
    callback = get(diagnosticsHandle, 'Callback'); callback(diagnosticsHandle, []);
    state = ui.ReadState();
    verifyNotEmpty(testCase, fieldnames(state.GoalMode.GraphicsHandles.DiagnosticPlotHandles));
    path = fullfile(testCase.TestData.Folder, 'native.mat');
    state.ExportBundle(path, 'goal');
    loaded = load(path, 'diagnosisBundle');
    verifyEqual(testCase, loaded.diagnosisBundle.PlannerInputs.goalState.time_s, 12);
    verifyEqual(testCase, loaded.diagnosisBundle.PlannerOptions.GoalTimeMode, "fixedArrival");
    verifyTrue(testCase, loaded.diagnosisBundle.IndependentValidation.Passed);
end

function request = sceneRequest()
    request = struct('schemaVersion', 'offlineSandboxRequest/v1', 'requestId', 'core-detour', ...
        'initialState', struct('time_s', 0, 'position_units', [-4 0]), ...
        'goalState', struct('time_s', 12, 'position_units', [4 0]), ...
        'limits', struct('xInterval_units', [-6 6], 'yInterval_units', [-4 4], ...
            'maxVelocity_units_s', [2 2], 'maxAcceleration_units_s2', [2 2], 'maxJerk_units_s3', [4 4]), ...
        'options', struct('GoalTimeMode', 'fixedArrival'), ...
        'obstacles', struct('name', 'center block', 'safetyMargin_units', 0.25, ...
            'keyframes', struct('time_s', 0, 'vertices_units', [-1 -1; 1 -1; 1 1; -1 1])));
end

function [response, bundle] = runRequest(testCase, request)
    requestPath = fullfile(testCase.TestData.Folder, 'request.json');
    resultPath = fullfile(testCase.TestData.Folder, 'result.json');
    writeText(requestPath, jsonencode(request));
    [response, bundle] = offlineSandbox.runPlanningRequest(requestPath, resultPath);
    decoded = jsondecode(fileread(resultPath));
    verifyEqual(testCase, decoded.result.Success, response.result.Success);
    verifyEqual(testCase, decoded.validation.Passed, response.validation.Passed);
end

function writeText(path, value)
    file = fopen(path, 'w');
    cleanup = onCleanup(@() fclose(file));
    fprintf(file, '%s', value);
end
