function tests = testPlannerOptions
%% Section 0: Header & Readme
% Verify that one obstacle-planner option owner resolves every public control.
tests = functiontests(localfunctions);
end

function setupOnce(~)
% Add source folders before option tests. These checks isolate default values,
% overrides, warnings, and invalid input from trajectory generation.
% Add the repository and Az/El product paths for direct test execution.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot);
addpath(fullfile(repositoryRoot, "trajectory"));
end

function testDefaultsMatchPublicPlannerRequirement(testCase)
% Keep one source of truth between the package and public defaults calls.
options = obstacleAvoidance.input.resolvePlannerOptions();
expected = obstacleAvoidance.planTrajectory();

verifyEqual(testCase, options, expected);
verifyFalse(testCase, isfield(options, "MotionMethod"));
verifyFalse(testCase, isfield(options, "RandomSeed"));
verifyFalse(testCase, isfield(options, "MaximumPlanningTime_s"));
end

function testPartialOverridesResolveAndNormalize(testCase)
% Apply known nonempty fields, retain empty defaults, and normalize values.
overrides = struct( ...
    "GoalTimeMode", 'fixedArrival', ...
    "SampleTime_s", [], ...
    "MaximumSeedCount", 3, ...
    "Verbose", 1);
options = obstacleAvoidance.input.resolvePlannerOptions(overrides);

verifyEqual(testCase, options.GoalTimeMode, "fixedArrival");
verifyEqual(testCase, options.SampleTime_s, 0.05);
verifyEqual(testCase, options.MaximumSeedCount, 3);
verifyTrue(testCase, options.Verbose);
end

function testUnknownFieldsWarnOnceAndRemainIgnored(testCase)
% Unknown options must give one warning and must not change output. Multiple
% warnings show repeated parsing. Changed output shows an unsupported field
% reached algorithm code.
% Aggregate all ignored names into the established warning identifier.
overrides = struct("UnknownFirst", 1, "UnknownSecond", 2);
verifyWarning(testCase, @() ...
    obstacleAvoidance.input.resolvePlannerOptions(overrides), ...
    "planTrajectory:UnknownOptions");
options = callWithoutWarning(overrides);
verifyFalse(testCase, isfield(options, "UnknownFirst"));
verifyFalse(testCase, isfield(options, "UnknownSecond"));
end

function testInvalidRequirementsRetainEstablishedErrors(testCase)
% Preserve explicit errors for malformed, moved, and invalid values.
verifyError(testCase, @() ...
    obstacleAvoidance.input.resolvePlannerOptions(1), ...
    "planTrajectory:InvalidOptions");
verifyError(testCase, @() obstacleAvoidance.input.resolvePlannerOptions( ...
    struct("AzimuthInterval_deg", [-1 1])), ...
    "planTrajectory:WorkspaceLimitMoved");
verifyError(testCase, @() obstacleAvoidance.input.resolvePlannerOptions( ...
    struct("GoalTimeMode", "invalid")), ...
    "planTrajectory:InvalidGoalTimeMode");
verifyWarning(testCase, @() obstacleAvoidance.input.resolvePlannerOptions( ...
    struct("DirectSeedOnly", true)), ...
    "planTrajectory:UnknownOptions");
end

function options = callWithoutWarning(overrides)
% Suppress the already-verified aggregate warning for output inspection.
warningState = warning("off", "planTrajectory:UnknownOptions");
warningCleanup = onCleanup(@() warning(warningState));
options = obstacleAvoidance.input.resolvePlannerOptions(overrides);
end
