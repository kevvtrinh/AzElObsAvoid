function tests = testAzElHs3Sandbox
%% Section 0: Header & Readme
% SYNTAX
%   tests = testAzElHs3Sandbox
%**************************************************************************
% PURPOSE
%   - Verify segment conversion, continuity, solving, and sandbox graphics.
%**************************************************************************
% INPUTS
%   - This function has no inputs.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest.Test array)
%       Deterministic tests for the HS-3 segment sandbox.
%**************************************************************************
% UNITS
%   - Position is degrees. Time is seconds. Derivative units are in field
%     names. Vectors use [azimuth elevation] order.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
% PURPOSE
%   - Add the sandbox source folder for this test file only.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
sandboxFolder = fullfile(repositoryRoot, "sandbox");
addpath(repositoryRoot, sandboxFolder);
testCase.TestData.SandboxFolder = sandboxFolder;
end

function teardownOnce(testCase)
% PURPOSE
%   - Remove the sandbox source folder after all tests finish.
rmpath(testCase.TestData.SandboxFolder);
end

function testDefaultsAndTemplateAreComplete(testCase)
% PURPOSE
%   - Verify documented option defaults and physical segment fields.
options = solveAzElHs3Segments();
segment = azElHs3SegmentTemplate();
testCase.verifyEqual(options.SampleTime_s, 0.05);
testCase.verifyEqual(options.MaximumPlanningTimePerSegment_s, 30);
testCase.verifyFalse(options.Verbose);
testCase.verifyTrue(all(isfield(segment, [ ...
    "startPosition_deg" "endPosition_deg" ...
    "initialVelocity_deg_s" "finalVelocity_deg_s" ...
    "initialAcceleration_deg_s2" "finalAcceleration_deg_s2" ...
    "limits" "maximumDuration_s" "arrivalMode"])));
testCase.verifyTrue(all(isfield(segment.limits, [ ...
    "maxVelocity_deg_s" "maxAcceleration_deg_s2" ...
    "maxJerk_deg_s3"])));
end

function testDiscontinuousSegmentsAreRejected(testCase)
% PURPOSE
%   - Prevent a combined profile with a hidden state jump.
firstSegment = azElHs3SegmentTemplate();
secondSegment = firstSegment;
secondSegment.startPosition_deg = firstSegment.endPosition_deg + [0.1 0];
testCase.verifyError(@() solveAzElHs3Segments( ...
    [firstSegment; secondSegment]), ...
    "solveAzElHs3Segments:DiscontinuousSegments");
end

function testConnectedFixedArrivalSegmentsBuildCertifiedProfile(testCase)
% PURPOSE
%   - Verify connected fixed-arrival HS-3 profiles through the public planner.
segment = azElHs3SegmentTemplate();
segment.startPosition_deg = [0 0];
segment.endPosition_deg = [1 0];
segment.maximumDuration_s = 4;
segment.arrivalMode = "fixedArrival";
segment.limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [2 2], ...
    "maxJerk_deg_s3", [4 4]);
secondSegment = segment;
secondSegment.startPosition_deg = segment.endPosition_deg;
secondSegment.endPosition_deg = [2 0];
plannerOptions = struct( ...
    "InitialCollocationSegmentCount", 4, ...
    "MaximumMeshRefinementPasses", 0, ...
    "MaximumCollocationSegmentCount", 4, ...
    "MaximumCorridorRelinearizations", 0, ...
    "MaximumNlpIterations", 100, ...
    "MaximumNlpFunctionEvaluations", 20000, ...
    "EnableJerkTieBreak", false);
[result, diagnostics] = solveAzElHs3Segments( ...
    [segment; secondSegment], struct( ...
    "SampleTime_s", 0.1, ...
    "MaximumPlanningTimePerSegment_s", 20, ...
    "PlannerOptions", plannerOptions));

testCase.verifyTrue(result.Success, result.Message);
testCase.verifyTrue(result.Validation.Passed);
testCase.verifyEqual(result.time_s([1 end]), [0; 8], "AbsTol", 1e-8);
testCase.verifyEqual(result.position_deg(1, :), [0 0], ...
    "AbsTol", 1e-8);
testCase.verifyEqual(result.position_deg(end, :), [2 0], ...
    "AbsTol", 1e-8);
testCase.verifyEqual(result.velocity_deg_s([1 end], :), ...
    zeros(2, 2), "AbsTol", 1e-7);
testCase.verifyEqual(result.acceleration_deg_s2([1 end], :), ...
    zeros(2, 2), "AbsTol", 1e-7);
testCase.verifyTrue(diagnostics.SegmentResults{1}.Validation.Passed);
testCase.verifyTrue(diagnostics.SegmentResults{2}.Validation.Passed);
testCase.verifyTrue(isfield(diagnostics.PlannerDiagnostics{1}, ...
    "TimedPath"));
testCase.verifyEqual(unique(result.SegmentIndex), [1; 2]);
testCase.verifySize(diagnostics.JerkJumpByJunction_deg_s3, [1 2]);
end

function testHiddenSandboxConvertsDefaultRow(testCase)
% PURPOSE
%   - Verify the GUI table can produce one semantic segment record.
app = azElHs3Sandbox(struct("FigureVisible", "off"));
cleanup = onCleanup(@() app.Close());
app.AddRow();
segments = app.ReadSegments();
testCase.verifySize(segments, [1 1]);
testCase.verifyEqual(segments.startPosition_deg, [-5 0]);
testCase.verifyEqual(segments.endPosition_deg, [5 0]);
testCase.verifyTrue(isgraphics(app.Figure));
testCase.verifyTrue(isgraphics(app.Axes));
end

function testHiddenProfilePlotUsesReturnedLimits(testCase)
% PURPOSE
%   - Verify the profile plot accepts combined histories without replanning.
time_s = (0:0.5:1).';
result = struct( ...
    "Success", true, ...
    "time_s", time_s, ...
    "position_deg", [time_s zeros(3, 1)], ...
    "velocity_deg_s", [ones(3, 1) zeros(3, 1)], ...
    "acceleration_deg_s2", zeros(3, 2), ...
    "jerk_deg_s3", zeros(3, 2), ...
    "maxVelocity_deg_s", repmat([2 2], 3, 1), ...
    "maxAcceleration_deg_s2", repmat([3 3], 3, 1), ...
    "maxJerk_deg_s3", repmat([4 4], 3, 1), ...
    "SegmentIndex", ones(3, 1));
handles = plotAzElHs3Motion(result, struct("FigureVisible", "off"));
cleanup = onCleanup(@() close(handles.Figure));
testCase.verifyTrue(isgraphics(handles.Figure));
testCase.verifyEqual(numel(handles.Axes), 4);
end
