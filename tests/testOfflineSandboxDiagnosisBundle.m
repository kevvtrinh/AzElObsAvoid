function tests = testOfflineSandboxDiagnosisBundle
%% Section 0: Header & Readme
% SYNTAX
%   tests = testOfflineSandboxDiagnosisBundle
%**************************************************************************
% PURPOSE
%   - Verify that the HTML sandbox creates a replayable, handle-free MATLAB
%     diagnosis bundle and exposes a request-matched download workflow.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest.FunctionTestCase array)
%       Focused tests for bundle construction and HTML/server integration.
%**************************************************************************
% UNITS
%   - Positions and polygon vertices use coordinate units. Time uses seconds.
%**************************************************************************

tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    % Add only the repository and offline-sandbox public-function folders.
    repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
    addpath(repositoryRoot, fullfile(repositoryRoot, "offlinesandbox"));
    testCase.TestData.RepositoryRoot = repositoryRoot;
end

function testBundlePreservesExactResultAndRemovesCallback(testCase)
    % Preserve replay data while excluding the trusted transport callback.
    request       = createSyntheticRequest();
    plannerInputs = struct("obstacles", struct.empty(0, 1), ...
        "initialState", request.initialState, ...
        "goalState", request.goalState, ...
        "limits", request.limits);
    plannerOptions = request.options;
    plannerOptions.TransientHandle = @() false;
    result = struct("Success", false, ...
        "Message", "No feasible path found.", ...
        "TerminationReason", "openSetExhausted", ...
        "Inputs", plannerInputs, ...
        "Options", plannerOptions);
    diagnosis  = struct("Search", struct("ExpandedCount", 17));
    validation = struct();
    validation.Passed  = false;
    validation.Message = "Expected failure.";

    bundle = offlineSandbox.createDiagnosisBundle(request, result, validation, diagnosis);

    verifyEqual(testCase, bundle.Format, "obstacleAvoidanceSandboxDiagnosis-v2");
    verifyEqual(testCase, bundle.ExportRequest.RequestId, request.requestId);
    verifyEqual(testCase, bundle.PlannerInputs, plannerInputs);
    verifyEqual(testCase, bundle.Diagnosis.Search.ExpandedCount, 17);
    verifyEmpty(testCase, bundle.Result.Options.TransientHandle);
    verifyEmpty(testCase, bundle.PlannerOptions.TransientHandle);
    verifyEqual(testCase, bundle.Result.TerminationReason, "openSetExhausted");
    verifyFalse(testCase, bundle.Result.Success);
end

function testPageAndServerExposeMatchedBundleDownload(testCase)
    % Keep bundle download and fresh replay routes wired to the browser controls.
    htmlPath   = fullfile(testCase.TestData.RepositoryRoot, "offlinesandbox", "xy_planner_sandbox.html");
    serverPath = fullfile(testCase.TestData.RepositoryRoot, "offlinesandbox", "+offlineSandbox", "serveSandbox.m");
    htmlText   = string(fileread(htmlPath));
    serverText = string(fileread(serverPath));

    verifyTrue(testCase, contains(htmlText, 'id="downloadBundleButton"'));
    verifyTrue(testCase, contains(htmlText, 'id="bundleFileInput"'));
    verifyTrue(testCase, contains(htmlText, '`${state.serverOrigin}/run-bundle`'));
    verifyTrue(testCase, contains(htmlText, 'id="setMotionButton"'));
    verifyTrue(testCase, contains(htmlText, 'id="copyObstacleButton"'));
    verifyTrue(testCase, contains(htmlText, 'id="rotateObstacleButton"'));
    verifyTrue(testCase, contains(htmlText, 'id="obstacleActionMenu"'));
    verifyTrue(testCase, contains(htmlText, 'function copySelectedObstacle()'));
    verifyTrue(testCase, contains(htmlText, 'function rotateSelectedObstacle()'));
    verifyTrue(testCase, contains(htmlText, 'function positionObstacleActionMenu()'));
    verifyTrue(testCase, contains(htmlText, 'motionVelocity_units_s'));
    verifyTrue(testCase, contains(htmlText, 'element("motionSpeedReadout").textContent = `${speed_units_s.toFixed(3)} units/s`'));
    verifyTrue(testCase, contains(htmlText, 'const MAXIMUM_OBSTACLE_SPEED_UNITS_S = 10;'));
    verifyTrue(testCase, contains(htmlText, 'motionVelocityFromDrag'));
    verifyTrue(testCase, contains(htmlText, 'motionVectorDisplayEnd'));
    verifyTrue(testCase, contains(htmlText, 'id="walkthroughDepthButton"'));
    verifyTrue(testCase, contains(htmlText, 'id="walkthroughReplayMenu"'));
    verifyTrue(testCase, contains(htmlText, 'class="walkthrough-replay-menu"'));
    verifyTrue(testCase, contains(htmlText, '<summary>Technical details</summary>'));
    verifyTrue(testCase, contains(htmlText, 'type="button">End</button>'));
    verifyTrue(testCase, contains(htmlText, 'class="compact-button walkthrough-nav-primary"'));
    verifyTrue(testCase, contains(htmlText, 'id="decisionTreePanel"'));
    verifyTrue(testCase, contains(htmlText, 'id="plannerInputsContent"'));
    verifyTrue(testCase, contains(htmlText, 'function renderDecisionTree(stages)'));
    verifyTrue(testCase, contains(htmlText, 'function walkthroughHasReachedView(view)'));
    verifyTrue(testCase, contains(htmlText, 'walkthroughHasReachedView("search")'));
    verifyTrue(testCase, contains(htmlText, 'tree.scrollTop'));
    verifyFalse(testCase, contains(htmlText, 'scrollIntoView'));
    verifyTrue(testCase, contains(htmlText, 'data-walkthrough-step'));
    verifyTrue(testCase, contains(htmlText, 'function buildDeepDiveStages()'));
    verifyTrue(testCase, contains(htmlText, 'Super deep dive: On'));
    verifyTrue(testCase, contains(htmlText, 'rejectedEdgeIndices'));
    verifyTrue(testCase, contains(htmlText, 'grouped in this single step'));
    verifyFalse(testCase, contains(htmlText, 'const edgeBudget'));
    verifyFalse(testCase, contains(htmlText, 'deepDiveStage(`Pruned edge ${index + 1}`'));
    verifyTrue(testCase, contains(htmlText, 'Decision path:'));
    verifyTrue(testCase, contains(htmlText, 'while fully outside'));
    verifyTrue(testCase, contains(htmlText, 'not obstruct the visible planning workspace'));
    verifyFalse(testCase, contains(htmlText, 'leaves the workspace during its motion.'));
    verifyTrue(testCase, contains(htmlText, 'fetch(`${state.serverOrigin}/bundle`'));
    verifyTrue(testCase, contains(htmlText, 'state.bundleRequestId = payload.requestId'));
    verifyTrue(testCase, contains(serverText, 'path == "/bundle"'));
    verifyTrue(testCase, contains(serverText, 'path == "/run-bundle"'));
    verifyTrue(testCase, contains(serverText, 'offlineSandbox.replayDiagnosisBundle'));
    verifyTrue(testCase, contains(serverText, 'cachedRequestId ~= requestId'));
    verifyTrue(testCase, contains(serverText, '"application/vnd.matlab.mat-file"'));
end

function testReplayDiagnosisBundleRunsStoredCanonicalRequest(testCase)
    % Replay a moving-obstacle canonical request and write a fresh browser result.
    initialState = struct();
    initialState.time_s              = 0;
    initialState.position_units        = [0 0];
    initialState.velocity_units_s      = [0 0];
    initialState.acceleration_units_s2 = [0 0];
    goalState = struct();
    goalState.time_s              = 10;
    goalState.position_units        = [1 0];
    goalState.velocity_units_s      = [0 0];
    goalState.acceleration_units_s2 = [0 0];
    limits = struct();
    limits.maxVelocity_units_s      = [2 2];
    limits.maxAcceleration_units_s2 = [2 2];
    limits.maxJerk_units_s3         = [5 5];
    limits.xInterval_units    = [-10 10];
    limits.yInterval_units  = [-10 10];
    time_s               = [0; 10];
    xBySlice_units   = {[-6; -5; -5; -6]; [-5.5; -4.5; -4.5; -5.5]};
    yBySlice_units = {[5; 5; 6; 6]; [5; 5; 6; 6]};
    movingObstacle       = obstacleAvoidance.obstacles.createObstacle("moving replay sentinel", time_s, xBySlice_units, yBySlice_units, 0.1);
    plannerInputs        = struct("obstacles", obstacleAvoidance.obstacles.combineObstacles({movingObstacle}), ...
        "initialState", initialState, ...
        "goalState", goalState, ...
        "limits", limits);
    plannerOptions = planner();
    plannerOptions.GoalTimeMode     = "earliestArrival";
    plannerOptions.MaximumSeedCount = 1;
    diagnosisBundle = struct("Format", "obstacleAvoidanceSandboxDiagnosis-v2", ...
        "PlannerInputs", plannerInputs, ...
        "PlannerOptions", plannerOptions);

    bundleFilePath = string(tempname) + ".mat";
    resultFilePath = string(tempname) + ".json";
    cleanup        = onCleanup(@() deleteTestFiles(bundleFilePath, resultFilePath));
    save(bundleFilePath, "diagnosisBundle", "-v7");

    [response, reproducedBundle] = offlineSandbox.replayDiagnosisBundle(bundleFilePath, resultFilePath);

    verifyTrue(testCase, response.result.Success, response.result.Message);
    verifyEqual(testCase, response.schemaVersion, "offlineSandboxResult/v1");
    verifyTrue(testCase, startsWith(string(response.requestId), "bundle-replay-"));
    verifyTrue(testCase, isfile(resultFilePath));
    verifyEqual(testCase, numel(response.obstacles), 1);
    verifyEqual(testCase, reproducedBundle.Format, "obstacleAvoidanceSandboxDiagnosis-v2");
    verifyEqual(testCase, reproducedBundle.PlannerInputs.initialState.position_units, [0 0]);
    verifyEqual(testCase, reproducedBundle.PlannerInputs.goalState.position_units, [1 0]);
    verifyEqual(testCase, reproducedBundle.PlannerInputs.obstacles(1).time_s, time_s);
    verifyEqual(testCase, reproducedBundle.PlannerInputs.obstacles(1).originalX_units{2}, xBySlice_units{2});
    clear cleanup;
end

function testSuppliedFailureBundleReplaysUnderEightySeconds(testCase)
    % Exercise the supplied scene with current planner options.
    bundleFilePath = fullfile(testCase.TestData.RepositoryRoot, "Rogue Examples", "failed.mat");
    resultFilePath = string(tempname) + ".json";
    cleanup        = onCleanup(@() deleteTestFiles(resultFilePath));

    verifyTrue(testCase, isfile(bundleFilePath), "The supplied failed.mat diagnosis bundle must remain in the suite.");
    loaded          = load(bundleFilePath, "diagnosisBundle");
    diagnosisBundle = loaded.diagnosisBundle;
    diagnosisBundle.PlannerOptions.GoalTimeMode = "earliestArrival";
    fixturePath    = string(tempname) + ".mat";
    fixtureCleanup = onCleanup(@() deleteTestFiles(fixturePath));
    save(fixturePath, "diagnosisBundle");
    [response, reproducedBundle] = offlineSandbox.replayDiagnosisBundle(fixturePath, resultFilePath);

    verifyTrue(testCase, response.result.Success, response.result.Message);
    verifyEqual(testCase, response.result.TerminationReason, "goalReached");
    verifyTrue(testCase, reproducedBundle.IndependentValidation.Passed, reproducedBundle.IndependentValidation.Message);
    verifyLessThan(testCase, response.result.ArrivalTime_s, 80, "The known feasible motion must arrive in less than 80 seconds.");
    selectedSummary = reproducedBundle.Diagnosis.Attempts(reproducedBundle.Diagnosis.SelectedAttemptIndex);
    verifyLessThan(testCase, selectedSummary.MotionLength_units + selectedSummary.TrajectoryDuration_s, 217, "Preserve the historical travel-plus-duration regression bound.");
    if ~isempty(fieldnames(reproducedBundle.Diagnosis.Search))
        verifyLessThan(testCase, reproducedBundle.Diagnosis.Search. RouteShorteningCandidateCount, 1000, "Same-class route cleanup must not restore the discarded candidate sweep.");
        verifyTrue(testCase, reproducedBundle.Diagnosis.Search. RouteClassSearchTruncated, "The bounded route-class search must report its completeness limit.");
    else
        % A validated exact motion needs no route-class search.
        verifyEqual(testCase, reproducedBundle.Diagnosis. AttemptedCount, 1);
    end
    clear cleanup;
end

function request = createSyntheticRequest()
    % Create a complete browser request without invoking the planner.
    request = struct("schemaVersion", "offlineSandboxRequest/v1", ...
        "requestId", "bundle-test-request", ...
        "obstacles", struct.empty(0, 1), ...
        "initialState", struct("time_s", 0, ...
            "position_units", [0 0], ...
            "velocity_units_s", [0 0], ...
            "acceleration_units_s2", [0 0]), ...
        "goalState", struct("time_s", 5, ...
            "position_units", [1 0], ...
            "velocity_units_s", [0 0], ...
            "acceleration_units_s2", [0 0]), ...
        "limits", struct("maxVelocity_units_s", [2 2], ...
            "maxAcceleration_units_s2", [2 2], ...
            "maxJerk_units_s3", [5 5], ...
            "xInterval_units", [-10 10], ...
            "yInterval_units", [-10 10]), ...
        "options", struct("GoalTimeMode", "earliestArrival"));
end

function deleteTestFiles(varargin)
    % Delete only temporary files created by one focused replay test.
    for fileIndex = 1:numel(varargin)
        if isfile(varargin{fileIndex})
            delete(varargin{fileIndex});
        end
    end
end

function testOptionalBundlePreservesFileAndMotionOutputs(testCase)
    % The documented file-only call and both output counts retain the same motion.
    request = struct('schemaVersion',"offlineSandboxRequest/v1", 'requestId',"optional-bundle", ...
        'obstacles',[], 'initialState',struct('time_s',0,'position_units',[0 0]), ...
        'goalState',struct('time_s',5,'position_units',[1 0]), ...
        'limits',struct('maxVelocity_units_s',[2 2], 'maxAcceleration_units_s2',[1 1], ...
            'maxJerk_units_s3',[2 2]), 'options',struct());
    requestPath = string(tempname) + ".json";
    resultPath  = string(tempname) + ".json";
    testCase.addTeardown(@() deleteTestFiles(requestPath, resultPath));
    fileIdentifier = fopen(requestPath, 'w');
    fprintf(fileIdentifier, '%s', jsonencode(request));
    fclose(fileIdentifier);
    single = offlineSandbox.runPlanningRequest(requestPath, resultPath);
    [paired, bundle] = offlineSandbox.runPlanningRequest(requestPath, resultPath);
    offlineSandbox.runPlanningRequest(requestPath, resultPath);
    fileOnly = jsondecode(fileread(resultPath));
    verifyTrue(testCase, single.validation.Passed);
    verifyTrue(testCase, paired.validation.Passed);
    verifyTrue(testCase, fileOnly.validation.Passed);
    verifyEqual(testCase, rmfield(single.result,'ElapsedPlanningTime_s'), rmfield(paired.result,'ElapsedPlanningTime_s'));
    verifyEqual(testCase, single.result.position_units, fileOnly.result.position_units);
    verifyEqual(testCase, paired.result.position_units, bundle.Result.position_units);
    verifyEqual(testCase, bundle.PlannerInputs.initialState.position_units, [0 0]);
end
