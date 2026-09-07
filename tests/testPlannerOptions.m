function tests = testPlannerOptions
%% Section 0: Header & Readme
% Verify that one obstacle-planner option owner resolves every public control.
tests = functiontests(localfunctions);
end

function setupOnce(~)
    % Add source folders before option tests. These checks isolate default values,
    % overrides, warnings, and invalid input from trajectory generation.
    % Add the repository and X/Y product paths for direct test execution.
    repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
    addpath(repositoryRoot);
    addpath(fullfile(repositoryRoot, "trajectory"));
end

function testDefaultsMatchPublicPlannerRequirement(testCase)
    % Keep one source of truth between the package and public defaults calls.
    options = obstacleAvoidance.input.resolvePlannerOptions();
    verifyFalse(testCase, isfield(options, "CancellationCheckFcn"));
    expected = obstacleAvoidance.planTrajectory();

    verifyEqual(testCase, options, expected);
    requiredFields = {'GoalTimeMode', 'SampleTime_s', 'MaximumSeedCount', ...
        'MaximumWaitRefinementIterations', ...
        'CollisionClearanceTolerance_units', 'WrapX', 'WrapY'};
    verifyTrue(testCase, isstruct(options) && isscalar(options));
    verifyTrue(testCase, all(isfield(options, requiredFields)));
    verifyFalse(testCase, isfield(options, "MotionMethod"));
    verifyFalse(testCase, isfield(options, "RandomSeed"));
    verifyFalse(testCase, isfield(options, "MaximumPlanningTime_s"));
    verifyFalse(testCase, isfield(options, "WaypointWarmStartMode"));
    verifyFalse(testCase, isfield(options, "RequestedWaypointWarmStartMode"));
    verifyFalse(testCase, isfield(options, "IsWaypointWarmStartAvailable"));
    verifyFalse(testCase, isfield(options, "PerSeedWorkBudgetMultiplier"));
    verifyFalse(testCase, isfield(options, "SeedClusterDistance_units"));
    verifyFalse(testCase, isfield(options, "Verbose"));
    verifyFalse(testCase, isfield(options, "EnablePlaneReuse"));
    verifyFalse(testCase, isfield(options, "PlaneReuseImprovementTolerance_s"));
    verifyFalse(testCase, isfield(options, "MaximumNlpIterations"));
    verifyFalse(testCase, isfield(options, "CollocationSegmentCount"));
    verifyEqual(testCase, options.UnsupportedTimedTopologyPolicy, "fail");
    verifyEqual(testCase, options.GoalTimeMode, "earliestArrival");
    verifyFalse(testCase, isfield(options, "MinimumTravelSavingsRate_units_s"));
    verifyEqual(testCase, options.MaximumSeedCount, 2);
    verifyEqual(testCase, options.MaximumWaitRefinementIterations, 16);
end

function testRetiredFieldsUseAggregateUnknownWarningAndAreIgnored(testCase)
    % Keep one unknown-field policy after removing dead compatibility shims.
    retiredOptions = struct();
    retiredOptions.PerSeedWorkBudgetMultiplier      = "invalid";
    retiredOptions.SeedClusterDistance_units          = "invalid";
    retiredOptions.Verbose                          = "invalid";
    retiredOptions.MaximumNlpIterations             = "invalid";
    retiredOptions.CollocationSegmentCount          = "invalid";
    retiredOptions.EnablePlaneReuse                 = false;
    retiredOptions.PlaneReuseImprovementTolerance_s = "invalid";
    retiredOptions.WaypointWarmStartMode            = "invalid";
    retiredOptions.RequestedWaypointWarmStartMode   = "invalid";
    retiredOptions.IsWaypointWarmStartAvailable     = true;
    verifyWarning(testCase, @() obstacleAvoidance.input.resolvePlannerOptions(retiredOptions), "planTrajectory:UnknownOptions");
    resolvedOptions = callWithoutWarning(retiredOptions);
    verifyEqual(testCase, resolvedOptions, obstacleAvoidance.input.resolvePlannerOptions());
end

function testPartialOverridesResolveAndNormalize(testCase)
    % Apply known nonempty fields, retain empty defaults, and normalize values.
    overrides = struct();
    overrides.GoalTimeMode                    = 'fixedArrival';
    overrides.UnsupportedTimedTopologyPolicy  = 'ruckigStopAtWaypoints';
    overrides.SampleTime_s                    = [];
    overrides.MaximumSeedCount                = 3;
    overrides.MaximumWaitRefinementIterations = 8;
    overrides.ArrivalTimeTolerance_s          = 2e-3;
    options = obstacleAvoidance.input.resolvePlannerOptions(overrides);

    verifyEqual(testCase, options.GoalTimeMode, "fixedArrival");
    verifyEqual(testCase, options.UnsupportedTimedTopologyPolicy, "ruckigStopAtWaypoints");
    verifyEqual(testCase, options.SampleTime_s, 0.05);
    verifyEqual(testCase, options.MaximumSeedCount, 3);
    verifyEqual(testCase, options.MaximumWaitRefinementIterations, 8);
end

function testUnknownFieldsWarnOnceAndRemainIgnored(testCase)
    % Unknown options must give one warning and must not change output. Multiple
    % warnings show repeated parsing. Changed output shows an unsupported field
    % reached algorithm code.
    % Aggregate all ignored names into the established warning identifier.
    overrides = struct();
    overrides.UnknownFirst  = 1;
    overrides.UnknownSecond = 2;
    verifyWarning(testCase, @() obstacleAvoidance.input.resolvePlannerOptions(overrides), "planTrajectory:UnknownOptions");
    options = callWithoutWarning(overrides);
    verifyFalse(testCase, isfield(options, "UnknownFirst"));
    verifyFalse(testCase, isfield(options, "UnknownSecond"));
end

function testRetiredTrajectoryMethodIsIgnored(testCase)
    % Old saved requests follow the standard unknown-option policy.
    defaults = obstacleAvoidance.input.resolvePlannerOptions();
    verifyFalse(testCase, isfield(defaults, "TrajectoryMethod"));
    % Exercise each value covered by this regression.
    for value = ["bmtp", "ruckigWaypoint", "invalid"]
        overrides = struct("TrajectoryMethod", value);
        verifyWarning(testCase, @() obstacleAvoidance.input.resolvePlannerOptions(overrides), "planTrajectory:UnknownOptions");
        verifyEqual(testCase, callWithoutWarning(overrides), defaults);
    end
end

function testInvalidRequirementsRetainEstablishedErrors(testCase)
    % Preserve explicit errors for malformed, moved, and invalid values.
    verifyError(testCase, @() obstacleAvoidance.input.resolvePlannerOptions(1), "planTrajectory:InvalidOptions");
    verifyError(testCase, @() obstacleAvoidance.input.resolvePlannerOptions(struct("XInterval_units", [-1 1])), "planTrajectory:WorkspaceLimitMoved");
    verifyError(testCase, @() obstacleAvoidance.input.resolvePlannerOptions(struct("GoalTimeMode", "invalid")), "planTrajectory:InvalidGoalTimeMode");
    verifyError(testCase, @() obstacleAvoidance.input.resolvePlannerOptions(struct("UnsupportedTimedTopologyPolicy", "invalid")), "planTrajectory:InvalidUnsupportedTimedTopologyPolicy");
    verifyError(testCase, @() obstacleAvoidance.input.resolvePlannerOptions(struct("MaximumSeedCount", 6)), "MATLAB:notLessEqual");
    verifyWarning(testCase, @() obstacleAvoidance.input.resolvePlannerOptions(struct("DirectSeedOnly", true)), "planTrajectory:UnknownOptions");
    verifyWarning(testCase, @() obstacleAvoidance.input.resolvePlannerOptions(struct("CollectAllSeedCandidates", true)), "planTrajectory:UnknownOptions");
end

function options = callWithoutWarning(overrides)
    % Suppress the already-verified aggregate warning for output inspection.
    warningState   = warning("off", "planTrajectory:UnknownOptions");
    warningCleanup = onCleanup(@() warning(warningState));
    options        = obstacleAvoidance.input.resolvePlannerOptions(overrides);
end

function testUnsupportedArrivalModeIsRejected(testCase)
    verifyError(testCase, @() obstacleAvoidance.input.resolvePlannerOptions(struct("GoalTimeMode", "balancedArrival")), "planTrajectory:InvalidGoalTimeMode");
end
