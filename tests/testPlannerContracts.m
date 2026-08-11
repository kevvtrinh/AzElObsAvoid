function tests = testPlannerContracts
%% Section 0: Header & Readme
% SYNTAX
%   tests = testPlannerContracts
%**************************************************************************
% PURPOSE
%   - Define contract, failure-semantics, and example-purity regressions.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest function-based test suite)
%**************************************************************************
% UNITS
%   - Not applicable.

tests = functiontests(localfunctions);
end

function setupOnce(testCase)
%% Section 0: Header & Readme
% SYNTAX
%   setupOnce(testCase)
%**************************************************************************
% PURPOSE
%   - Add the repository root and examples for this test file.
%**************************************************************************
% INPUTS
%   - testCase (matlab.unittest.FunctionTestCase)
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Not applicable.

repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot);
addpath(fullfile(repositoryRoot, "examples"));
testCase.TestData.repositoryRoot = repositoryRoot;
end

function testCanonicalBuilderPreservesStaticAndDisconnectedGeometry(testCase)
%% Section 0: Header & Readme
% SYNTAX
%   testCanonicalBuilderPreservesStaticAndDisconnectedGeometry(testCase)
%**************************************************************************
% PURPOSE
%   - Protect canonical fields, column cells, static repetition, and paired
%     nonfinite disconnected-region separators.
%**************************************************************************
% INPUTS
%   - testCase (matlab.unittest.FunctionTestCase)
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Angles are degrees and time is seconds.

azimuth_deg = [-4; -1; -1; -4; NaN; 1; 4; 4; 1];
elevation_deg = [-2; -2; 2; 2; NaN; -2; -2; 2; 2];
azElData = makeAzElObstacleData( ...
    "two regions", [0; 3; 9], azimuth_deg, elevation_deg);

verifyEqual(testCase, string(fieldnames(azElData)), ...
    ["targetName"; "time_s"; "az_deg"; "el_deg"; "status"]);
verifySize(testCase, azElData.az_deg, [3, 1]);
verifySize(testCase, azElData.el_deg, [3, 1]);
verifyEqual(testCase, azElData.az_deg{1}, azimuth_deg);
verifyEqual(testCase, azElData.az_deg{3}, azimuth_deg);
verifyEqual(testCase, isfinite(azElData.az_deg{2}), ...
    isfinite(azElData.el_deg{2}));
end

function testEveryPlannerExitUsesTheSamePublicSchema(testCase)
%% Section 0: Header & Readme
% SYNTAX
%   testEveryPlannerExitUsesTheSamePublicSchema(testCase)
%**************************************************************************
% PURPOSE
%   - Ensure success and invalid input return the same inspectable fields.
%**************************************************************************
% INPUTS
%   - testCase (matlab.unittest.FunctionTestCase)
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Not applicable.

request = basicRequest();
successResult = planAzElAvoidance(request);
invalidResult = planAzElAvoidance(struct());

verifyTrue(testCase, successResult.success);
verifyEqual(testCase, invalidResult.status, "invalid");
verifyEqual(testCase, fieldnames(invalidResult), fieldnames(successResult));
verifyEqual(testCase, fieldnames(invalidResult.command), ...
    fieldnames(successResult.command));
end

function testBlockedStartAndImpossibleDeadlineAreProvedFailures(testCase)
%% Section 0: Header & Readme
% SYNTAX
%   testBlockedStartAndImpossibleDeadlineAreProvedFailures(testCase)
%**************************************************************************
% PURPOSE
%   - Distinguish endpoint and obstacle-free deadline proofs from unknown.
%**************************************************************************
% INPUTS
%   - testCase (matlab.unittest.FunctionTestCase)
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Angles are degrees and time is seconds.

blockedRequest = basicRequest();
blockedRequest.obstacles = makeAzElObstacleData( ...
    "start block", [0; 20], [-2; 2; 2; -2], [-2; -2; 2; 2]);
blockedResult = planAzElAvoidance(blockedRequest);
verifyEqual(testCase, blockedResult.status, "infeasible");
verifyEqual(testCase, blockedResult.reasonCode, "BlockedInitialState");

deadlineRequest = basicRequest();
deadlineRequest.options = struct("deadline_s", 0.1);
deadlineResult = planAzElAvoidance(deadlineRequest);
verifyEqual(testCase, deadlineResult.status, "infeasible");
verifyEqual(testCase, deadlineResult.reasonCode, ...
    "ObstacleFreeBoundaryInfeasible");
end

function testResourceExhaustionIsUnknown(testCase)
%% Section 0: Header & Readme
% SYNTAX
%   testResourceExhaustionIsUnknown(testCase)
%**************************************************************************
% PURPOSE
%   - Prevent a time-limited search from being labeled infeasible.
%**************************************************************************
% INPUTS
%   - testCase (matlab.unittest.FunctionTestCase)
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Planning wall time is seconds.

request = basicRequest();
request.obstacles = makeAzElObstacleData( ...
    "interior block", [0; 30], [8; 12; 12; 8], [2; 2; 8; 8]);
request.options = struct( ...
    "deadline_s", 30, ...
    "planningWallTime_s", 1e-6);
result = planAzElAvoidance(request);
verifyFalse(testCase, result.success);
verifyEqual(testCase, result.status, "unknown");
verifyNotEqual(testCase, result.status, "infeasible");
end

function testExamplesContainOnlyMissionInputsAndEvidence(testCase)
%% Section 0: Header & Readme
% SYNTAX
%   testExamplesContainOnlyMissionInputsAndEvidence(testCase)
%**************************************************************************
% PURPOSE
%   - Enforce that examples cannot contain private work controls or hints.
%**************************************************************************
% INPUTS
%   - testCase (matlab.unittest.FunctionTestCase)
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Not applicable.

exampleFiles = dir(fullfile(testCase.TestData.repositoryRoot, ...
    "examples", "*.m"));
forbiddenTerms = [ ...
    "spatialresolution", ...
    "temporalresolution", ...
    "gridspacing", ...
    "refinementlevel", ...
    "iterationcount", ...
    "candidatecount", ...
    "waypoint", ...
    "guidepath", ...
    "corridor", ...
    "storedcommand", ...
    "plannermode", ...
    "planningwalltime_s"];
for fileIndex = 1:numel(exampleFiles)
    sourceText = lower(string(fileread(fullfile( ...
        exampleFiles(fileIndex).folder, exampleFiles(fileIndex).name))));
    for termIndex = 1:numel(forbiddenTerms)
        verifyFalse(testCase, contains(sourceText, ...
            forbiddenTerms(termIndex)), sprintf( ...
            "%s contains forbidden example term '%s'.", ...
            exampleFiles(fileIndex).name, forbiddenTerms(termIndex)));
    end
end
end

function request = basicRequest()
%% Section 0: Header & Readme
% SYNTAX
%   request = basicRequest()
%**************************************************************************
% PURPOSE
%   - Build a small deterministic complete-state mission for contract tests.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - request (scalar planning request)
%**************************************************************************
% UNITS
%   - Angles are degrees and time is seconds.

request = struct();
request.obstacles = [];
request.initialState = struct( ...
    "time_s", 0, ...
    "position_deg", [0, 0], ...
    "velocity_deg_s", [0, 0], ...
    "acceleration_deg_s2", [0, 0]);
request.goal = struct( ...
    "type", "fixed", ...
    "time_s", NaN, ...
    "position_deg", [20, 10], ...
    "velocity_deg_s", [0, 0], ...
    "acceleration_deg_s2", [0, 0]);
request.limits = struct( ...
    "azimuth_deg", [-180, 180], ...
    "elevation_deg", [-90, 90], ...
    "maxVelocity_deg_s", [10, 10], ...
    "maxAcceleration_deg_s2", [5, 5]);
request.options = struct();
end
