function tests = testExampleContracts
%% Section 0: Header & Readme
% SYNTAX
%   tests = testExampleContracts
%**************************************************************************
% PURPOSE
%   - Protect the maintained physical contracts of three large examples.
%   - Verify that duration metrics state the active arrival policy.
%   - Verify that every maintained example applies the shared jerk override.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest function test array)
%**************************************************************************
% UNITS
%   - Contract positions are degrees and contract times are seconds.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
% Add the repository and example folders for source and metric checks.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot);
addpath(fullfile(repositoryRoot, "planAzElMotion"));
addpath(fullfile(repositoryRoot, "hs3"));
addpath(fullfile(repositoryRoot, "examples"));
testCase.TestData.RepositoryRoot = repositoryRoot;
end

function testPhysicalContractHashes(testCase)
% Protect reviewed inputs and the native U.S. deformation from silent drift.
relativePaths = [ ...
    "examples/exampleUShapedAzElTimeSpace.m", ...
    "examples/exampleFortyMovingCircleGrid.m", ...
    "examples/exampleMovingDeformingUSOutlineVisibility.m", "examples/private/createContiguousUSObstacle.m"];
expectedHashes = [ ...
    "96ca19d633bbc3c4d23c4c5bd94313a0ae00237ec2b76c96f24260dd9000fa81", ...
    "469c568ce255d08a192f7bf291c2f50b352f69728f4644d5a82bb102b197616e", ...
    "a704e4310fb112b2bedecef466b9c1f916b01f0314273be41bd039ef07e80e1a", ...
    "dfd62f710e54d709222ffb5d764aa9c16adc8b13f1c9b997afadeb0a452b9658"];

% Read every contract fixture and compare its normalized source hash or required text.
for contractIndex = 1:numel(relativePaths)
    contractPath = fullfile( testCase.TestData.RepositoryRoot, relativePaths(contractIndex));
    sourceText = string(fileread(contractPath));
    sourceText = replace(sourceText, [string(char([13 10])) string(char(13))], newline);
    if contractIndex < 4
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
    "exampleAlternatingSlalom", "exampleAzElPlanning", ...
    "exampleDenseConcaveAzElMotion", "exampleFortyMovingCircleGrid", ...
    "exampleMovingBarrierWait", "exampleMovingCircleNoAzimuthWrap", ...
    "exampleMovingDeformingUSOutlineVisibility", ...
    "exampleNoPathAzElMotion", "exampleObstacleFreeAzElMotion", ...
    "exampleOpeningUShapedAzElTimeSpace", ...
    "exampleTwoOpposingUVisibilityGraph", "exampleUShapedAzElTimeSpace", "exampleUSOutlineExtremeVisibility"];

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

function testDurationMetricNamesTheArrivalPolicy(testCase)
% Verify that a fixed-arrival duration is not reported as a minimum.
result = struct( ...
    "Success", true, ...
    "Seeds", struct("Length_deg", 5), ...
    "SelectedSeedIndex", 1, ...
    "position_deg", [0 0; 3 4], ...
    "time_s", [0; 12], ...
    "velocity_deg_s", zeros(2, 2), ...
    "acceleration_deg_s2", zeros(2, 2), ...
    "jerk_deg_s3", zeros(2, 2), ...
    "Validation", struct( ...
        "CollisionFree", true, ...
        "VelocityWithinLimits", true, ...
        "AccelerationWithinLimits", true, ...
        "JerkWithinLimits", true, ...
        "DynamicsConsistent", true), ...
    "Options", struct("GoalTimeMode", "fixedArrival"), "TerminationReason", "goalReached");
fixedMetrics = computeAzElExampleMetrics(result);
verifyEqual(testCase, fixedMetrics.MotionDuration_s, 12);
verifyEqual(testCase, fixedMetrics.MotionDurationInterpretation, "fixedArrivalDuration");
verifyFalse(testCase, isfield(fixedMetrics, "MinimumMotionDuration_s"));

result.Options.GoalTimeMode = "earliestArrival";
earliestMetrics = computeAzElExampleMetrics(result);
verifyEqual(testCase, earliestMetrics.MotionDurationInterpretation, "earliestValidatedDuration");

benchmarkPath = fullfile(testCase.TestData.RepositoryRoot, "benchmark.csv");
benchmarkHeader = extractBefore(string(fileread(benchmarkPath)), newline);
verifyTrue(testCase, contains(benchmarkHeader, "MotionDuration_s"));
verifyFalse(testCase, contains(benchmarkHeader, "MinimumMotionDuration_s"));
result.Success = false;
result.TerminationReason = "noValidatedSeed";
failureMetrics = computeAzElExampleMetrics(result);
verifyTrue(testCase, isnan(failureMetrics.CollisionFree));
verifyTrue(testCase, isnan( failureMetrics.KinematicCertificatePassed));
end

function testExampleResolverMaterializesPlannerDefaults(testCase)
% Verify examples materialize and override the maintained HS3 defaults.
[defaultOptions, ~] = resolveAzElExampleOptions( ...
    struct("PlotOutputs", false), struct("MaximumSeedCount", 2));
verifyFalse(testCase, defaultOptions.Verbose);
verifyEqual(testCase, defaultOptions.MaximumSeedCount, 2);
verifyEqual(testCase, defaultOptions.PlannerMethod, "hs3");
verifyTrue(testCase, isfield(defaultOptions, "CollocationSegmentCount"));
verifyFalse(testCase, isfield(defaultOptions, "MotionMethod"));

[hs3Options, ~] = resolveAzElExampleOptions( ...
    struct("Verbose", true), ...
    struct("CollocationSegmentCount", 6));
verifyTrue(testCase, hs3Options.Verbose);
verifyEqual(testCase, hs3Options.PlannerMethod, "hs3");
verifyEqual(testCase, hs3Options.CollocationSegmentCount, 6);
verifyFalse(testCase, isfield(hs3Options, "MotionMethod"));
end

function testUniformMaximumJerkRouting(testCase)
% Verify that every example routes the shared jerk control into limits.
exampleNames = [ ...
    "exampleAlternatingSlalom", "exampleAzElPlanning", ...
    "exampleDenseConcaveAzElMotion", "exampleFortyMovingCircleGrid", ...
    "exampleFourAcceleratingCircles", ...
    "exampleInterceptMovingTargetAtSetTime", ...
    "exampleInterceptMovingTargetEarliest", ...
    "exampleMovingBarrierWait", "exampleMovingCircleNoAzimuthWrap", ...
    "exampleMovingDeformingUSOutlineVisibility", ...
    "exampleNoPathAzElMotion", "exampleObstacleFreeAzElMotion", ...
    "exampleOpeningUShapedAzElTimeSpace", ...
    "exampleStraightTargetAlternatingOcclusion", ...
    "exampleTargetExitsObstacle", ...
    "exampleTwoOpposingUVisibilityGraph", "exampleUShapedAzElTimeSpace", "exampleUSOutlineExtremeVisibility"];

% Verify every maintained example routes its configured jerk limit into planner limits.
for exampleName = exampleNames
    examplePath = fullfile(testCase.TestData.RepositoryRoot, "examples", exampleName + ".m");
    sourceText = fileread(examplePath);
    routedMatch = regexp(sourceText, '"maxJerk_deg_s3"\s*,\s*\w+\.MaxJerk_deg_s3', 'once');
    verifyNotEmpty(testCase, routedMatch, exampleName + " must route MaxJerk_deg_s3 into limits.");
end
end

function testEveryMaintainedExampleReturnsItsName(testCase)
% Keep serial reporting and downstream benchmark identity uniform.
exampleFolder = fullfile(testCase.TestData.RepositoryRoot, "examples");
exampleFiles = dir(fullfile(exampleFolder, "example*.m"));
for fileIndex = 1:numel(exampleFiles)
    exampleName = erase(string(exampleFiles(fileIndex).name), ".m");
    sourceText = string(fileread( ...
        fullfile(exampleFolder, exampleFiles(fileIndex).name)));
    requiredAssignment = "result.ExampleName = """ + ...
        exampleName + """;";
    verifyTrue(testCase, contains(sourceText, requiredAssignment), ...
        exampleName + " must return its exact ExampleName metadata.");
end
end
