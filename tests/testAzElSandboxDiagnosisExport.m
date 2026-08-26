function tests = testAzElSandboxDiagnosisExport
%% Section 0: Header & Readme
% SYNTAX
%   tests = testAzElSandboxDiagnosisExport
%**************************************************************************
% PURPOSE
%   - Verify sandbox diagnosis bundles preserve successful and failed
%     planner calls without graphics handles.
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
% Add production, examples, and sandbox folders for public-call tests.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot);
addpath(fullfile(repositoryRoot, "examples"));
addpath(fullfile(repositoryRoot, "sandbox"));
testCase.TestData.RepositoryRoot = repositoryRoot;
end

function testSuccessfulBundleRoundTripsAndReproduces(testCase)
% Preserve exact success inputs/result and replay the public planner call.
[initialState, goalState, limits, options] = simpleRequest();
result = planAzElMotion( ...
    [], initialState, goalState, limits, options);
verifyTrue(testCase, result.Success, result.Message);
validation = validateAzElTrajectory(result);
verifyTrue(testCase, validation.Passed, validation.Message);
sandboxState = syntheticSandboxState( ...
    result, validation, initialState, goalState, combineAzElObstacles());
filePath = string(tempname) + ".mat";
testCase.addTeardown(@() deleteIfPresent(filePath));

exportInfo = exportAzElSandboxDiagnosis( ...
    filePath, sandboxState, "goal");
loaded = load(char(filePath), "diagnosisBundle");
bundle = loaded.diagnosisBundle;
verifyEqual(testCase, exportInfo.Schema, "azElSandboxDiagnosis-v1");
verifyGreaterThan(testCase, exportInfo.Bytes, 0);
verifyEqual(testCase, bundle.Mode, "goal");
verifyEqual(testCase, bundle.PlannerInputs.initialState, ...
    result.Inputs.initialState);
verifyEqual(testCase, bundle.PlannerInputs.goalState, ...
    result.Inputs.goalState);
verifyEqual(testCase, bundle.Result.TerminationReason, "goalReached");
verifyTrue(testCase, bundle.IndependentValidation.Passed);
verifyFalse(testCase, isfield(bundle.Scene, "GraphicsHandles"));

reproduced = planAzElMotion( ...
    bundle.PlannerInputs.obstacles, ...
    bundle.PlannerInputs.initialState, ...
    bundle.PlannerInputs.goalState, ...
    bundle.PlannerInputs.limits, bundle.PlannerOptions);
reproducedValidation = validateAzElTrajectory(reproduced);
verifyTrue(testCase, reproduced.Success, reproduced.Message);
verifyTrue(testCase, reproducedValidation.Passed, ...
    reproducedValidation.Message);
verifyEqual(testCase, reproduced.ArrivalTime_s, ...
    result.ArrivalTime_s, "AbsTol", 1e-9);
end

function testPersistentSandboxCreatesExportActions(testCase)
% Verify both tabs expose an export button and the direct export API.
sandboxState = azElInteractiveSandbox( ...
    struct("FigureVisible", "off"));
testCase.addTeardown(@() closeIfPresent(sandboxState.FigureHandle));
currentState = sandboxState.ReadState();
verifyTrue(testCase, isfield( ...
    currentState.GoalMode.GraphicsHandles.Actions, "Export"));
verifyTrue(testCase, isfield( ...
    currentState.FreeMode.GraphicsHandles.Actions, "Export"));
verifyEqual(testCase, get( ...
    currentState.GoalMode.GraphicsHandles.Actions.Export, "Enable"), ...
    'off');
verifyEqual(testCase, get( ...
    currentState.FreeMode.GraphicsHandles.Actions.Export, "Enable"), ...
    'off');
verifyTrue(testCase, isa(currentState.ExportBundle, "function_handle"));
end

function testPublicExportBundleWritesCurrentState(testCase)
% Verify the public no-dialog hook writes the live guidata-backed result.
[initialState, goalState, limits, options] = simpleRequest();
result = planAzElMotion([], initialState, goalState, limits, options);
validation = validateAzElTrajectory(result);
sandboxState = azElInteractiveSandbox(struct("FigureVisible", "off"));
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

function testFailedBundleRetainsDiagnosisEvidence(testCase)
% Preserve endpoint-blocked failure inputs, diagnostics, and validation.
[initialState, goalState, limits, options] = simpleRequest();
obstacle = makeAzElObstacleData( ...
    "start enclosure", [0; goalState.time_s], ...
    [-1; 1; 1; -1], [-1; -1; 1; 1], 0);
result = planAzElMotion( ...
    obstacle, initialState, goalState, limits, options);
verifyFalse(testCase, result.Success);
verifyEqual(testCase, result.TerminationReason, "endpointBlocked");
sandboxState = syntheticSandboxState( ...
    result, result.Validation, initialState, goalState, obstacle);
filePath = string(tempname) + ".mat";
testCase.addTeardown(@() deleteIfPresent(filePath));

exportInfo = exportAzElSandboxDiagnosis( ...
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
options = planAzElMotion();
options.GoalTimeMode = "fixedArrival";
options.RandomSeed = 325;
end

function sandboxState = syntheticSandboxState( ...
        result, validation, initialState, goalState, obstacles)
% Build the handle-free portion of the stable persistent-sandbox schema.
modeState = struct( ...
    "StartPosition_deg", initialState.position_deg, ...
    "GoalPosition_deg", goalState.position_deg, ...
    "WaypointPositions_deg", zeros(0, 2), ...
    "RawObstacleStrokes_deg", {cell(0, 1)}, ...
    "LineObstaclePositions_deg", {cell(0, 1)}, ...
    "PolygonObstaclePositions_deg", {cell(0, 1)}, ...
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
