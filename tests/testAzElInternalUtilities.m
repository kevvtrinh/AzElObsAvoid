function tests = testAzElInternalUtilities
%% Section 0: Header & Readme
% SYNTAX
%   tests = testAzElInternalUtilities()
%**************************************************************************
% PURPOSE
%   - Verify shared option resolution, logical normalization, and packed
%     obstacle-slice decoding independently of planner behavior.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest.FunctionTestCase array)
%       Focused tests for repository-wide internal invariants.
%**************************************************************************
% UNITS
%   - Packed polygon coordinates are degrees; controls are dimensionless.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
% PURPOSE
%   - Put the repository API on the path for package-helper tests.
testFilePath = mfilename("fullpath");
repositoryRoot = fileparts(fileparts(testFilePath));
testCase.TestData.OriginalPath = path;
addpath(repositoryRoot);
end

function teardownOnce(testCase)
% PURPOSE
%   - Restore the caller path after this focused test file completes.
path(testCase.TestData.OriginalPath);
end

function testResolveOptionsPreservesDefaultsAndFieldOrder(testCase)
% PURPOSE
%   - Verify that option resolution keeps default order and values.
defaults = struct( ...
    "First", 1, ...
    "Second", 2, ...
    "Third", 3);
overrides = struct( ...
    "Second", 20, ...
    "Unknown", 99, ...
    "First", []);

[resolved, unknownNames] = azElInternal.resolveOptions( ...
    defaults, overrides);

testCase.verifyEqual(fieldnames(resolved), fieldnames(defaults));
testCase.verifyEqual(resolved.First, 1);
testCase.verifyEqual(resolved.Second, 20);
testCase.verifyEqual(resolved.Third, 3);
testCase.verifyEqual(unknownNames, "Unknown");
testCase.verifyFalse(isfield(resolved, "Unknown"));
end

function testNormalizeLogicalScalarAcceptsDocumentedForms(testCase)
% PURPOSE
%   - Verify all documented scalar logical forms.
identifier = "testAzElInternalUtilities:InvalidControl";
testCase.verifyFalse(azElInternal.normalizeLogicalScalar( ...
    false, "Control", identifier));
testCase.verifyTrue(azElInternal.normalizeLogicalScalar( ...
    true, "Control", identifier));
testCase.verifyFalse(azElInternal.normalizeLogicalScalar( ...
    0, "Control", identifier));
testCase.verifyTrue(azElInternal.normalizeLogicalScalar( ...
    1, "Control", identifier));
end

function testNormalizeLogicalScalarRejectsAmbiguity(testCase)
% PURPOSE
%   - Verify that ambiguous logical controls use the caller error ID.
identifier = "testAzElInternalUtilities:InvalidControl";
testCase.verifyError(@() azElInternal.normalizeLogicalScalar( ...
    2, "Control", identifier), identifier);
testCase.verifyError(@() azElInternal.normalizeLogicalScalar( ...
    NaN, "Control", identifier), identifier);
testCase.verifyError(@() azElInternal.normalizeLogicalScalar( ...
    [0 1], "Control", identifier), identifier);
end

function testNormalizeParallelModeUsesOneAliasPolicy(testCase)
% PURPOSE
%   - Verify one normalized representation for parallel-mode aliases.
identifier = "testAzElInternalUtilities:InvalidParallelMode";
testCase.verifyEqual(azElInternal.normalizeParallelMode( ...
    "AUTO", identifier), "auto");
testCase.verifyEqual(azElInternal.normalizeParallelMode( ...
    true, identifier), "on");
testCase.verifyEqual(azElInternal.normalizeParallelMode( ...
    0, identifier), "off");
testCase.verifyError(@() azElInternal.normalizeParallelMode( ...
    2, identifier), identifier);
testCase.verifyError(@() azElInternal.normalizeParallelMode( ...
    "sometimes", identifier), identifier);
end

function testUnpackObstacleSliceRegionsPreservesRingOrder(testCase)
% PURPOSE
%   - Verify ordered decoding of two NaN-separated polygon rings.
firstRing_deg = [ ...
    -3 -2; ...
    -1 -2; ...
    -1 0; ...
    -3 0];
secondRing_deg = [ ...
    1 1; ...
    3 1; ...
    3 3; ...
    1 3];
separatedRings_deg = [ ...
    firstRing_deg; ...
    NaN NaN; ...
    secondRing_deg];
obstacle = makeAzElObstacleData( ...
    "two rings", [0; 1], ...
    separatedRings_deg(:, 1), separatedRings_deg(:, 2), 0);
obstacleField = buildAzElTimeObstacleField(obstacle);

regions = azElInternal.unpackObstacleSliceRegions( ...
    obstacleField.Obstacles(1), 1);

testCase.verifySize(regions, [2 1]);
testCase.verifyEqual(regions{1}, firstRing_deg);
testCase.verifyEqual(regions{2}, secondRing_deg);
end

function testEmptyDiagnosticsUseStableFailureValues(testCase)
% PURPOSE
%   - Verify empty clearance and validation records remain explicit.
clearance = azElInternal.emptyCollisionClearanceDiagnostics();
validation = azElInternal.emptyTrajectoryValidation();

testCase.verifyFalse(clearance.ProvenClear);
testCase.verifySize(clearance.UnresolvedPathSegmentMask, [0 1]);
testCase.verifyEqual(clearance.MaximumSubdivisionDepth, 18);
testCase.verifyTrue(isnan(clearance.MinimumProvenLowerBound_deg));
testCase.verifyFalse(validation.Passed);
testCase.verifyEqual(validation.Issues, "notValidated");
testCase.verifyEqual(validation.InitialStateError, inf(1, 6));
testCase.verifyTrue(isinf(validation.MaximumDynamicsDefect));
end

function testPolylineLengthHandlesEmptyAndSegmentedRoutes(testCase)
% PURPOSE
%   - Verify zero-point, one-point, and multi-segment polyline lengths.
testCase.verifyEqual(azElInternal.polylineLength(zeros(0, 2)), 0);
testCase.verifyEqual(azElInternal.polylineLength([4 8]), 0);
testCase.verifyEqual(azElInternal.polylineLength( ...
    [0 0; 3 4; 3 6]), 7, "AbsTol", eps);
end
