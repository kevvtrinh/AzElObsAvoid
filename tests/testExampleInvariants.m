function tests = testExampleInvariants
%% Section 0: Header & Readme
% SYNTAX
%   tests = testExampleInvariants
%**************************************************************************
% PURPOSE
%   - Protect the maintained physical requirements of two large examples.
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
addpath(fullfile(repositoryRoot, "trajectory"));
addpath(fullfile(repositoryRoot, "examples"));
testCase.TestData.RepositoryRoot = repositoryRoot;
end

function testPhysicalRequirementHashes(testCase)
% Protect reviewed inputs and the native U.S. deformation from silent drift.
relativePaths = [ ...
    "examples/exampleStaticUShapedObstacle.m", ...
    "examples/exampleMovingDeformingUSOutlineVisibility.m", "examples/private/createContiguousUSObstacle.m"];
expectedHashes = [ ...
    "23dc6b4547d454caddc04d46f89707296bf7becd1a61745781d0d4003f03020c", ...
    "e3ebbdfac79424bb47763b8c8496d898dfff7df68496d174f5a3f2c84a3786d1", ...
    "b20b23767ef8a6310ec9993e35a5c0b893137511a8f0bc8cf658bb28788c2454"];

% Read each fixture. Compare its normalized source hash or required text.
for requirementIndex = 1:numel(relativePaths)
    requirementPath = fullfile( testCase.TestData.RepositoryRoot, relativePaths(requirementIndex));
    sourceText = string(fileread(requirementPath));
    sourceText = replace(sourceText, [string(char([13 10])) string(char(13))], newline);
    if requirementIndex < numel(relativePaths)
        sourceText = extractBefore( extractAfter(sourceText, "%% Section 1:"), "%% Section 6:");
    end
    hashEngine = java.security.MessageDigest.getInstance("SHA-256");
    hashEngine.update(unicode2native(char(sourceText), "UTF-8"));
    hashBytes = mod(double(hashEngine.digest()), 256);
    actualHash = lower(string( reshape(dec2hex(hashBytes, 2).', 1, [])));
    verifyEqual(testCase, actualHash, expectedHashes(requirementIndex), ...
        "The reviewed example requirement changed in " + relativePaths(requirementIndex) + ".");
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
[defaultOptions, defaultDisplayOptions] = resolveExampleOptions( ...
    struct("PlotOutputs", false), struct("MaximumSeedCount", 2));
verifyFalse(testCase, isfield(defaultOptions, "Verbose"));
verifyTrue(testCase, defaultDisplayOptions.Verbose);
verifyEqual(testCase, defaultOptions.MaximumSeedCount, 2);
verifyFalse(testCase, isfield(defaultOptions, "PlannerMethod"));
verifyFalse(testCase, isfield(defaultOptions, "CollocationSegmentCount"));
verifyFalse(testCase, isfield(defaultOptions, "MotionMethod"));

[hs3Options, displayOptions] = resolveExampleOptions( ...
    struct("Verbose", false), ...
    struct("MaximumSeedCount", 6));
verifyFalse(testCase, isfield(hs3Options, "Verbose"));
verifyFalse(testCase, displayOptions.Verbose);
verifyFalse(testCase, isfield(hs3Options, "PlannerMethod"));
verifyEqual(testCase, hs3Options.MaximumSeedCount, 6);
verifyFalse(testCase, isfield(hs3Options, "MotionMethod"));
end

function testExampleResolverForwardsDeprecatedCollocation(testCase)
% Forward the retired mesh knob to the planner's single warning owner.
legacyOptions = struct( ...
    "CollocationSegmentCount", 6, "PlotOutputs", false);
lastwarn("", "");
[forwardedOptions, ~] = resolveExampleOptions(legacyOptions, struct());
[~, warningIdentifier] = lastwarn;
verifyEmpty(testCase, warningIdentifier);
verifyEqual(testCase, forwardedOptions.CollocationSegmentCount, 6);
verifyWarning(testCase, @() ...
    obstacleAvoidance.input.resolvePlannerOptions(forwardedOptions), ...
    "planTrajectory:DeprecatedCollocationSegmentCount");
end

function testExampleResolverForwardsDeprecatedPlannerOptions(testCase)
% Preserve legacy planner fields only until the planner migration shim runs.
legacyOptions = struct( ...
    "WaypointWarmStartMode", "passThrough", ...
    "RequestedWaypointWarmStartMode", "none", ...
    "IsWaypointWarmStartAvailable", true, ...
    "PlotOutputs", false);
lastwarn("", "");
[forwardedOptions, ~] = resolveExampleOptions(legacyOptions, struct());
[~, warningIdentifier] = lastwarn;
verifyEmpty(testCase, warningIdentifier);
verifyEqual(testCase, forwardedOptions.WaypointWarmStartMode, "passThrough");
verifyEqual(testCase, forwardedOptions.RequestedWaypointWarmStartMode, "none");
verifyTrue(testCase, forwardedOptions.IsWaypointWarmStartAvailable);
verifyWarning(testCase, @() ...
    obstacleAvoidance.input.resolvePlannerOptions(forwardedOptions), ...
    "planTrajectory:DeprecatedWaypointWarmStartOptions");
end

function testExampleResolverForwardsDeprecatedWorkBudget(testCase)
% Forward the retired cutoff to the planner's single migration-warning owner.
legacyOptions = struct( ...
    "PerSeedWorkBudgetMultiplier", 3, "PlotOutputs", false);
lastwarn("", "");
[forwardedOptions, ~] = resolveExampleOptions(legacyOptions, struct());
[~, warningIdentifier] = lastwarn;
verifyEmpty(testCase, warningIdentifier);
verifyEqual(testCase, forwardedOptions.PerSeedWorkBudgetMultiplier, 3);
verifyWarning(testCase, @() ...
    obstacleAvoidance.input.resolvePlannerOptions(forwardedOptions), ...
    "planTrajectory:DeprecatedPerSeedWorkBudgetMultiplier");
end

function testExampleResolverForwardsDeprecatedSeedCluster(testCase)
% Forward the retired clustering option to the planner migration owner.
legacyOptions = struct( ...
    "SeedClusterDistance_deg", 2, "PlotOutputs", false);
lastwarn("", "");
[forwardedOptions, ~] = resolveExampleOptions(legacyOptions, struct());
[~, warningIdentifier] = lastwarn;
verifyEmpty(testCase, warningIdentifier);
verifyEqual(testCase, forwardedOptions.SeedClusterDistance_deg, 2);
verifyWarning(testCase, @() ...
    obstacleAvoidance.input.resolvePlannerOptions(forwardedOptions), ...
    "planTrajectory:DeprecatedSeedClusterDistance");
end

function testExampleResolverForwardsDeprecatedPlaneReuseOptions(testCase)
% Forward old reuse knobs only until the planner migration shim strips them.
legacyOptions = struct( ...
    "EnablePlaneReuse", false, ...
    "PlaneReuseImprovementTolerance_s", 1e-7, ...
    "PlotOutputs", false);
lastwarn("", "");
[forwardedOptions, ~] = resolveExampleOptions(legacyOptions, struct());
[~, warningIdentifier] = lastwarn;
verifyEmpty(testCase, warningIdentifier);
verifyFalse(testCase, forwardedOptions.EnablePlaneReuse);
verifyEqual(testCase, ...
    forwardedOptions.PlaneReuseImprovementTolerance_s, 1e-7);
verifyWarning(testCase, @() ...
    obstacleAvoidance.input.resolvePlannerOptions(forwardedOptions), ...
    "planTrajectory:DeprecatedPlaneReuseOptions");
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

function testEveryMaintainedExampleReturnsPlannerFormat(testCase)
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

function testObstacleAvoidanceRunsHeadlessly(testCase)
% Execute a maintained obstacle case instead of checking only its source text.
result = exampleObstacleAvoidance(struct( ...
    "PlotOutputs", false, "FigureVisible", "off"));

verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
verifyEqual(testCase, result.ArrivalTime_s, 7.574542, "AbsTol", 1e-6);
summary = result.SeedSummaries(result.SelectedSeedIndex);
verifyEqual(testCase, summary.MotionLength_deg, 11.411861, "AbsTol", 1e-6);
verifyEqual(testCase, result.SearchDiagnostics.Grid.SeedCluster.Distance_deg, 0);
verifyGreaterThan(testCase, ...
    result.SearchDiagnostics.Grid.SeedCluster.SourceRegionCount, 0);
verifyEqual(testCase, ...
    result.SearchDiagnostics.Grid.SeedCluster.ClusterGroupCount, 0);
verifyEqual(testCase, ...
    result.SearchDiagnostics.Grid.SeedCluster.ClusteredRegionCount, 0);
verifyEmpty(testCase, ...
    result.SearchDiagnostics.Grid.SeedCluster.ClusterBoundary_deg);
end

function testObstacleFreeRunsHeadlessly(testCase)
% Execute the direct maintained example and check public result evidence.
result = exampleObstacleFree(struct( ...
    "PlotOutputs", false, "FigureVisible", "off"));

verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
verifyGreaterThan(testCase, result.ArrivalTime_s, 0);
summary = result.SeedSummaries(result.SelectedSeedIndex);
verifyEqual(testCase, summary.MotionLength_deg, sqrt(20), "AbsTol", 1e-9);
end
