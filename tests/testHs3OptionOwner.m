function tests = testHs3OptionOwner
%% Section 0: Header & Readme
% Verify that the standalone HS3 option owner resolves only HS3 controls.
tests = functiontests(localfunctions);
end

function setupOnce(~)
% Add the repository root for path-based test execution.
addpath(fileparts(fileparts(mfilename("fullpath"))));
end

function testDefaultsMatchPublicHs3Contract(testCase)
% Keep one source of truth between the package and public defaults calls.
options = azElPlannerMethods.hs3.resolvePlannerOptions();
expected = planAzElMotion("hs3");
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
options = azElPlannerMethods.hs3.resolvePlannerOptions(overrides);

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
    azElPlannerMethods.hs3.resolvePlannerOptions(overrides), ...
    "planAzElMotion:UnknownOptions");
options = callWithoutWarning(overrides);
verifyFalse(testCase, isfield(options, "UnknownFirst"));
verifyFalse(testCase, isfield(options, "UnknownSecond"));
end

function testInvalidContractsRetainEstablishedErrors(testCase)
% Preserve explicit errors for malformed, moved, and invalid values.
verifyError(testCase, @() ...
    azElPlannerMethods.hs3.resolvePlannerOptions(1), ...
    "planAzElMotion:InvalidOptions");
verifyError(testCase, @() azElPlannerMethods.hs3.resolvePlannerOptions( ...
    struct("AzimuthInterval_deg", [-1 1])), ...
    "planAzElMotion:WorkspaceLimitMoved");
verifyError(testCase, @() azElPlannerMethods.hs3.resolvePlannerOptions( ...
    struct("GoalTimeMode", "invalid")), ...
    "planAzElMotion:InvalidGoalTimeMode");
verifyError(testCase, @() azElPlannerMethods.hs3.resolvePlannerOptions( ...
    struct("DirectSeedOnly", 2)), ...
    "planAzElMotion:InvalidLogicalOption");
end

function options = callWithoutWarning(overrides)
% Suppress the already-verified aggregate warning for output inspection.
warningState = warning("off", "planAzElMotion:UnknownOptions");
warningCleanup = onCleanup(@() warning(warningState));
options = azElPlannerMethods.hs3.resolvePlannerOptions(overrides);
end
