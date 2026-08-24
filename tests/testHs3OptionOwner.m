function tests = testHs3OptionOwner
% Verify the isolated HS3 option owner and compact projection contract.
tests = functiontests(localfunctions);
end

function setupOnce(~)
% Add the repository root for path-based test execution.
addpath(fileparts(fileparts(mfilename("fullpath"))));
end

function testDefaultsMatchExistingHs3Contract(testCase)
% Preserve every current default except the deliberately disabled improvement.
[options, compactOverrides] = ...
    azElPlannerMethods.hs3.resolvePlannerOptions();
expected = planAzElMotion("hs3");
expected = rmfield(expected, "PlannerMethod");
expected.EnableHs3Improvement = false;
verifyEqual(testCase, options, expected);
verifyFalse(testCase, isfield(compactOverrides, ...
    "EnableHs3Improvement"));
end

function testPartialOverridesResolveAndNormalize(testCase)
% Apply known nonempty fields, retain empty defaults, and normalize values.
overrides = struct( ...
    "GoalTimeMode", 'fixedArrival', ...
    "SampleTime_s", [], ...
    "MaximumSeedCount", 3, ...
    "EnableHs3Improvement", 1, ...
    "Verbose", 1);
[options, compactOverrides] = ...
    azElPlannerMethods.hs3.resolvePlannerOptions(overrides);
verifyEqual(testCase, options.GoalTimeMode, "fixedArrival");
verifyEqual(testCase, options.SampleTime_s, 0.05);
verifyEqual(testCase, options.MaximumSeedCount, 3);
verifyTrue(testCase, options.EnableHs3Improvement);
verifyTrue(testCase, options.Verbose);
verifyEqual(testCase, compactOverrides.MaximumSeedCount, 3);
verifyTrue(testCase, compactOverrides.Verbose);
end

function testUnknownFieldsWarnOnceAndRemainIgnored(testCase)
% Aggregate all ignored names into the established warning identifier.
overrides = struct("UnknownFirst", 1, "UnknownSecond", 2);
verifyWarning(testCase, @() ...
    azElPlannerMethods.hs3.resolvePlannerOptions(overrides), ...
    "planAzElMotion:UnknownOptions");
[options, compactOverrides] = callWithoutWarning(overrides);
verifyFalse(testCase, isfield(options, "UnknownFirst"));
verifyFalse(testCase, isfield(compactOverrides, "UnknownSecond"));
end

function testInvalidContractsRetainEstablishedErrors(testCase)
% Preserve explicit errors for malformed, retired, moved, and invalid values.
verifyError(testCase, @() ...
    azElPlannerMethods.hs3.resolvePlannerOptions(1), ...
    "planAzElMotion:InvalidOptions");
verifyError(testCase, @() azElPlannerMethods.hs3.resolvePlannerOptions( ...
    struct("MaximumPlanningTime_s", 1)), ...
    "planAzElMotion:RemovedMaximumPlanningTime");
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

function testProjectionContainsOnlyCommonCompactFields(testCase)
% Exclude selectors, compatibility fields, and all HS3-only solver controls.
[options, compactOverrides] = ...
    azElPlannerMethods.hs3.resolvePlannerOptions();
expectedNames = ["GoalTimeMode"; "SampleTime_s"; ...
    "AllowAzimuthWrapping"; "MaximumSeedCount"; "DirectSeedOnly"; ...
    "SeedClusterDistance_deg"; "ArrivalTimeTolerance_s"; ...
    "ConstraintTolerance"; "CollisionClearanceTolerance_deg"; ...
    "CollisionMinimumTimeStep_s"; "Verbose"; "RandomSeed"];
verifyEqual(testCase, string(fieldnames(compactOverrides)), expectedNames);
for name = expectedNames.'
    verifyEqual(testCase, compactOverrides.(name), options.(name));
end
verifyFalse(testCase, isfield(compactOverrides, "PlannerMethod"));
verifyFalse(testCase, isfield(compactOverrides, "MotionMethod"));
verifyFalse(testCase, isfield(compactOverrides, ...
    "CollocationSegmentCount"));
end

function [options, compactOverrides] = callWithoutWarning(overrides)
% Suppress the already-verified aggregate warning for output inspection.
warningState = warning("off", "planAzElMotion:UnknownOptions");
warningCleanup = onCleanup(@() warning(warningState));
[options, compactOverrides] = ...
    azElPlannerMethods.hs3.resolvePlannerOptions(overrides);
end
