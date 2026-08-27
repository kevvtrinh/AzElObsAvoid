function tests = testHs3OptionOwner
%% Section 0: Header & Readme
% Verify that the standalone HS3 option owner resolves only HS3 controls.
tests = functiontests(localfunctions);
end

function setupOnce(~)
% Add the repository and Az/El product paths for direct test execution.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot);
addpath(fullfile(repositoryRoot, "hs3"));
end

function testDefaultsMatchPublicHs3Contract(testCase)
% Keep one source of truth between the package and public defaults calls.
options = obstacleAvoidance.input.resolveHs3Options();
expected = obstacleAvoidance.planTrajectory("hs3");
expected = rmfield(expected, "PlannerMethod");

verifyEqual(testCase, options, expected);
verifyEqual(testCase, options.MaximumPlanningTime_s, 115);
verifyFalse(testCase, isfield(options, "MotionMethod"));
end

function testPartialOverridesResolveAndNormalize(testCase)
% Apply known nonempty fields, retain empty defaults, and normalize values.
overrides = struct( ...
    "GoalTimeMode", 'fixedArrival', ...
    "SampleTime_s", [], ...
    "MaximumSeedCount", 3, ...
    "MaximumPlanningTime_s", 9, ...
    "Verbose", 1);
options = obstacleAvoidance.input.resolveHs3Options(overrides);

verifyEqual(testCase, options.GoalTimeMode, "fixedArrival");
verifyEqual(testCase, options.SampleTime_s, 0.05);
verifyEqual(testCase, options.MaximumSeedCount, 3);
verifyEqual(testCase, options.MaximumPlanningTime_s, 9);
verifyTrue(testCase, options.Verbose);
end

function testUnknownFieldsWarnOnceAndRemainIgnored(testCase)
% Aggregate all ignored names into the established warning identifier.
overrides = struct("UnknownFirst", 1, "UnknownSecond", 2);
verifyWarning(testCase, @() ...
    obstacleAvoidance.input.resolveHs3Options(overrides), ...
    "planTrajectory:UnknownOptions");
options = callWithoutWarning(overrides);
verifyFalse(testCase, isfield(options, "UnknownFirst"));
verifyFalse(testCase, isfield(options, "UnknownSecond"));
end

function testInvalidContractsRetainEstablishedErrors(testCase)
% Preserve explicit errors for malformed, moved, and invalid values.
verifyError(testCase, @() ...
    obstacleAvoidance.input.resolveHs3Options(1), ...
    "planTrajectory:InvalidOptions");
verifyError(testCase, @() obstacleAvoidance.input.resolveHs3Options( ...
    struct("AzimuthInterval_deg", [-1 1])), ...
    "planTrajectory:WorkspaceLimitMoved");
verifyError(testCase, @() obstacleAvoidance.input.resolveHs3Options( ...
    struct("GoalTimeMode", "invalid")), ...
    "planTrajectory:InvalidGoalTimeMode");
verifyError(testCase, @() obstacleAvoidance.input.resolveHs3Options( ...
    struct("DirectSeedOnly", 2)), ...
    "planTrajectory:InvalidLogicalOption");
end

function options = callWithoutWarning(overrides)
% Suppress the already-verified aggregate warning for output inspection.
warningState = warning("off", "planTrajectory:UnknownOptions");
warningCleanup = onCleanup(@() warning(warningState));
options = obstacleAvoidance.input.resolveHs3Options(overrides);
end
