function tests = testMinimalAzElWorkflow
%% Section 0: Header & Readme
% SYNTAX
%   tests = testMinimalAzElWorkflow
%**************************************************************************
% PURPOSE
%   - Verify the minimal construct, reduce-search, plan, validate, and plot
%     workflow with finite and unlimited jerk.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (MATLAB function-test array)
%       Deterministic tests for the retained public workflow.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds.
%**************************************************************************
root = fileparts(fileparts(mfilename("fullpath")));
addpath(root, fullfile(root, "examples"));
tests = functiontests(localfunctions);
end

function testDenseObstacleIsReducedOnlyForSearch(testCase)
result = exampleAzElPlanning(struct("FigureVisible", "off", ...
    "PlotOutputs", false, "EnableJerkConstraint", true));
testCase.verifyTrue(result.Success, result.Message);
testCase.verifyTrue(result.Validation.Passed, result.Validation.Message);
diagnostics = result.candidateReductionDiagnostics;
inputVertexCount = sum([diagnostics.InputVertexCount]);
selectedCandidateCount = sum([diagnostics.SelectedCandidateCount]);
testCase.verifyGreaterThan(inputVertexCount, 1000);
testCase.verifyLessThan(selectedCandidateCount, 0.01 * inputVertexCount);
blocked = queryAzElTimedPathCollision(result.obstacleField, ...
    result.timedSlopePath.time_s, result.timedSlopePath.position_deg);
testCase.verifyFalse(any(blocked));
testCase.verifyTrue(result.timedSlopePath.ConstraintDiagnostics. ...
    FiniteJerkCertified);
end

function testUnlimitedJerkIsReportedHonestly(testCase)
result = exampleAzElPlanning(struct("FigureVisible", "off", ...
    "PlotOutputs", false, "EnableJerkConstraint", false));
testCase.verifyTrue(result.Success, result.Message);
testCase.verifyTrue(result.Validation.Passed, result.Validation.Message);
testCase.verifyFalse(result.timedSlopePath.ConstraintDiagnostics. ...
    FiniteJerkCertified);
testCase.verifyTrue(all(isinf(result.limits.maxJerk_deg_s3)));
testCase.verifyTrue(all(isnan(result.timedSlopePath. ...
    ConstraintDiagnostics.PeakJerk_deg_s3)));
end

function testUnlimitedJerkUShapeDoesNotInsertAccelerationZeros(testCase)
result = exampleUShapedAzElTimeSpace(struct("FigureVisible", "off", ...
    "PlotOutputs", false, "EnableJerkConstraint", false, ...
    "Verbose", false));
testCase.verifyTrue(result.Success, result.Message);
testCase.verifyTrue(result.ExampleValidation.Passed, ...
    result.ExampleValidation.Message);
timedPath = result.timedSlopePath;
tangentialAcceleration_deg_s2 = ...
    timedPath.SampleTangentialAcceleration_deg_s2;
artificialZero = abs(tangentialAcceleration_deg_s2(2:end - 1)) < 1e-12 & ...
    abs(tangentialAcceleration_deg_s2(1:end - 2)) > 1e-4 & ...
    abs(tangentialAcceleration_deg_s2(3:end)) > 1e-4;
testCase.verifyEqual(nnz(artificialZero), 0, ...
    "Spatial profile joins must not inject one-sample acceleration zeros.");
for profileIndex = 1:numel(timedPath.SegmentProfiles)
    profile = timedPath.SegmentProfiles(profileIndex);
    testCase.verifyEqual(profile.PhaseStartAcceleration_deg_s2(1), ...
        profile.TangentialAcceleration_deg_s2, "AbsTol", 1e-12);
end
testCase.verifyLessThan(timedPath.MinimumMotionDuration_s, 30);
testCase.verifyTrue(all(isnan(timedPath.jerk_deg_s3), "all"));
blocked = queryAzElTimedPathCollision(result.obstacleField, ...
    timedPath.time_s, timedPath.position_deg);
testCase.verifyFalse(any(blocked));
jerkConstrainedResult = exampleUShapedAzElTimeSpace(struct( ...
    "FigureVisible", "off", "PlotOutputs", false, ...
    "EnableJerkConstraint", true, "Verbose", false));
testCase.verifyTrue(jerkConstrainedResult.Success, ...
    jerkConstrainedResult.Message);
testCase.verifyEqual(jerkConstrainedResult.limits.maxJerk_deg_s3, ...
    [2.5 2.5]);
testCase.verifyTrue(jerkConstrainedResult.timedSlopePath. ...
    ConstraintDiagnostics.FiniteJerkCertified);
testCase.verifyGreaterThan( ...
    jerkConstrainedResult.timedSlopePath.MinimumMotionDuration_s, ...
    timedPath.MinimumMotionDuration_s);
end

function testPlotConsumesResultWithoutReplanning(testCase)
result = exampleAzElPlanning(struct("FigureVisible", "off", ...
    "PlotOutputs", false));
handles = plotAzElMotion(result, struct("FigureVisible", "off", ...
    "FrameStride", 1000, "Pause_s", 0));
cleanup = onCleanup(@() close([handles.WorkspaceFigure ...
    handles.KinematicsFigure handles.Animation.Figure]));
testCase.verifyTrue(isgraphics(handles.WorkspaceFigure));
testCase.verifyTrue(isgraphics(handles.KinematicsFigure));
testCase.verifyEqual(numel(handles.KinematicsAxes), 4);
testCase.verifyTrue(isgraphics(handles.Animation.Figure));
testCase.verifyTrue(isgraphics(handles.Animation.Axes3D));
testCase.verifyTrue(isgraphics(handles.Animation.Axes2D));
end

function testMovingPointHistoryUsesSameObstacleConstructor(testCase)
squareAzimuth_deg = {[-2; 0; 0; -2], [1; 3; 3; 1]};
squareElevation_deg = {[-1; -1; 1; 1], [-1; -1; 1; 1]};
obstacle = makeAzElObstacleData("moving obstacle", [0; 20], ...
    squareAzimuth_deg, squareElevation_deg, 0);
field = buildAzElTimeObstacleField(obstacle);
testCase.verifyTrue(queryAzElTimeObstacle(field, -1, 0, 0));
testCase.verifyTrue(queryAzElTimeObstacle(field, 2, 0, 20));
testCase.verifyFalse(queryAzElTimeObstacle(field, -1, 0, 20));
initialState = struct("time_s", 0, "position_deg", [-4 3]);
goalState = struct("time_s", 20, "position_deg", [4 3]);
limits = struct("maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1], "maxJerk_deg_s3", [Inf Inf]);
result = planAzElMotion(obstacle, initialState, goalState, limits);
testCase.verifyTrue(result.Success, result.Message);
testCase.verifyTrue(result.Validation.Passed, result.Validation.Message);
end

function testBlockedEndpointReturnsStableFailure(testCase)
azimuth_deg = [-1; 1; 1; -1];
elevation_deg = [-1; -1; 1; 1];
obstacle = makeAzElObstacleData("obstacle", [0; 20], ...
    azimuth_deg, elevation_deg, 0);
initialState = struct("time_s", 0, "position_deg", [0 0]);
goalState = struct("time_s", 20, "position_deg", [4 0]);
limits = struct("maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1], "maxJerk_deg_s3", [2 2]);
result = planAzElMotion(obstacle, initialState, goalState, limits);
testCase.verifyFalse(result.Success);
testCase.verifyEqual(result.TerminationReason, "endpointBlocked");
testCase.verifyFalse(isempty(result.SearchDiagnostics));
handles = plotAzElMotion(result, struct("FigureVisible", "off"));
cleanup = onCleanup(@() close([handles.WorkspaceFigure ...
    handles.KinematicsFigure]));
testCase.verifyTrue(isgraphics(handles.WorkspaceFigure));
end
