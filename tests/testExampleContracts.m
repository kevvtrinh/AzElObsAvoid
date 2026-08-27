function tests = testExampleContracts
%% Section 0: Header & Readme
% SYNTAX
%   tests = testExampleContracts
%**************************************************************************
% PURPOSE
%   - Protect the maintained physical contracts of two large examples.
%   - Verify that examples return the normal planner result fields.
%   - Verify that every maintained example applies the shared jerk override.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest function test array)
%**************************************************************************
% UNITS
%   - Test positions are degrees and test times are seconds.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
% Locate maintained examples and reference text once. These checks do not run
% long plans. A failure points to the example file or required input that no
% longer follows the common example pattern.
% Add the repository and example folders for source and metric checks.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot);
addpath(fullfile(repositoryRoot, "hs3"));
addpath(fullfile(repositoryRoot, "examples"));
testCase.TestData.RepositoryRoot = repositoryRoot;
end

function testPhysicalContractHashes(testCase)
% Protect reviewed inputs and the native U.S. deformation from silent drift.
relativePaths = [ ...
    "examples/exampleStaticUShapedObstacle.m", ...
    "examples/exampleMovingDeformingUSOutlineVisibility.m", "examples/private/createContiguousUSObstacle.m"];
expectedHashes = [ ...
    "23dc6b4547d454caddc04d46f89707296bf7becd1a61745781d0d4003f03020c", ...
    "c1779b98aa853799bf24efcdfb57c4a0256ad97ac952410ad931551d12b17980", ...
    "b20b23767ef8a6310ec9993e35a5c0b893137511a8f0bc8cf658bb28788c2454"];

% Read each fixture. Compare its normalized source hash or required text.
for contractIndex = 1:numel(relativePaths)
    contractPath = fullfile( testCase.TestData.RepositoryRoot, relativePaths(contractIndex));
    sourceText = string(fileread(contractPath));
    sourceText = replace(sourceText, [string(char([13 10])) string(char(13))], newline);
    if contractIndex < numel(relativePaths)
        sourceText = extractBefore( extractAfter(sourceText, "%% Section 1:"), "%% Section 6:");
    end
    hashEngine = java.security.MessageDigest.getInstance("SHA-256");
    hashEngine.update(unicode2native(char(sourceText), "UTF-8"));
    hashBytes = mod(double(hashEngine.digest()), 256);
    actualHash = lower(string( reshape(dec2hex(hashBytes, 2).', 1, [])));
    verifyEqual(testCase, actualHash, expectedHashes(contractIndex), ...
        "The reviewed example contract changed in " + relativePaths(contractIndex) + ".");
end
end

function testNonTargetArrivalPoliciesAndSlalomBounds(testCase)
% Verify every fixed-goal example states earliest arrival and the slalom bound.
exampleNames = [ ...
    "exampleAlternatingSlalom", "exampleObstacleAvoidance", ...
    "exampleDenseConcaveObstacle", ...
    "exampleMovingBarrierWait", "exampleMovingCircleNoAzimuthWrap", ...
    "exampleMovingDeformingUSOutlineVisibility", ...
    "exampleNoPath", "exampleObstacleFree", ...
    "exampleOpeningUShapedObstacle", ...
    "exampleTwoOpposingUVisibilityGraph", "exampleStaticUShapedObstacle", "exampleUSOutlineExtremeVisibility"];

% Verify every maintained example explicitly requests earliest-arrival planning.
for exampleName = exampleNames
    examplePath = fullfile(testCase.TestData.RepositoryRoot, "examples", exampleName + ".m");
    sourceText = fileread(examplePath);
    policyMatch = regexp(sourceText, '"GoalTimeMode"\s*,\s*"earliestArrival"', 'once');
    verifyNotEmpty(testCase, policyMatch, exampleName + " must state GoalTimeMode=earliestArrival.");
end
slalomPath = fullfile(testCase.TestData.RepositoryRoot, "examples", "exampleAlternatingSlalom.m");
slalomText = fileread(slalomPath);
boundMatch = regexp(slalomText, '"elevationInterval_deg"\s*,\s*\[-5\s+5\]', 'once');
verifyNotEmpty(testCase, boundMatch, "The slalom elevation interval must be [-5 5] degrees.");
end

function testBenchmarkUsesNeutralDurationColumn(testCase)
% Keep benchmark duration reporting outside the returned planner structure.
benchmarkPath = fullfile(testCase.TestData.RepositoryRoot, "benchmark.csv");
benchmarkHeader = extractBefore(string(fileread(benchmarkPath)), newline);
verifyTrue(testCase, contains(benchmarkHeader, "MotionDuration_s"));
verifyFalse(testCase, contains(benchmarkHeader, "MinimumMotionDuration_s"));
verifyFalse(testCase, isfile(fullfile( ...
    testCase.TestData.RepositoryRoot, "examples", ...
    "computeExampleMetrics.m")));
end

function testExampleResolverMaterializesPlannerDefaults(testCase)
% Verify examples materialize and override the maintained HS3 defaults.
[defaultOptions, ~] = resolveExampleOptions( ...
    struct("PlotOutputs", false), struct("MaximumSeedCount", 2));
verifyTrue(testCase, defaultOptions.Verbose);
verifyEqual(testCase, defaultOptions.MaximumSeedCount, 2);
verifyFalse(testCase, isfield(defaultOptions, "PlannerMethod"));
verifyTrue(testCase, isfield(defaultOptions, "CollocationSegmentCount"));
verifyFalse(testCase, isfield(defaultOptions, "MotionMethod"));

[hs3Options, ~] = resolveExampleOptions( ...
    struct("Verbose", true), ...
    struct("CollocationSegmentCount", 6));
verifyTrue(testCase, hs3Options.Verbose);
verifyFalse(testCase, isfield(hs3Options, "PlannerMethod"));
verifyEqual(testCase, hs3Options.CollocationSegmentCount, 6);
verifyFalse(testCase, isfield(hs3Options, "MotionMethod"));
end

function testUniformMaximumJerkRouting(testCase)
% Verify that every example routes the shared jerk control into limits.
exampleNames = [ ...
    "exampleAlternatingSlalom", "exampleObstacleAvoidance", ...
    "exampleDenseConcaveObstacle", ...
    "exampleFourAcceleratingCircles", ...
    "exampleInterceptMovingTargetAtSetTime", ...
    "exampleInterceptMovingTargetEarliest", ...
    "exampleMovingBarrierWait", "exampleMovingCircleNoAzimuthWrap", ...
    "exampleMovingDeformingUSOutlineVisibility", ...
    "exampleNoPath", "exampleObstacleFree", ...
    "exampleOpeningUShapedObstacle", ...
    "exampleStraightTargetAlternatingOcclusion", ...
    "exampleTargetExitsObstacle", ...
    "exampleTwoOpposingUVisibilityGraph", "exampleStaticUShapedObstacle", "exampleUSOutlineExtremeVisibility"];

% Verify every maintained example routes its configured jerk limit into planner limits.
for exampleName = exampleNames
    examplePath = fullfile(testCase.TestData.RepositoryRoot, "examples", exampleName + ".m");
    sourceText = fileread(examplePath);
    routedMatch = regexp(sourceText, '"maxJerk_deg_s3"\s*,\s*\w+\.MaxJerk_deg_s3', 'once');
    verifyNotEmpty(testCase, routedMatch, exampleName + " must route MaxJerk_deg_s3 into limits.");
end
end

function testEveryMaintainedExampleReturnsPlannerSchema(testCase)
% Read each maintained example. Confirm that it returns the planner result
% directly. This prevents examples from adding private result fields.
% Prevent examples from appending demo-only fields to planner results.
exampleFolder = fullfile(testCase.TestData.RepositoryRoot, "examples");
exampleFiles = dir(fullfile(exampleFolder, "example*.m"));
for fileIndex = 1:numel(exampleFiles)
    exampleName = erase(string(exampleFiles(fileIndex).name), ".m");
    sourceText = string(fileread( ...
        fullfile(exampleFolder, exampleFiles(fileIndex).name)));
    addedField = regexp(sourceText, ...
        '(?m)^\s*result\.[A-Za-z]\w*\s*=', 'once');
    verifyEmpty(testCase, addedField, ...
        exampleName + " must not append fields to the planner result.");
end
end
