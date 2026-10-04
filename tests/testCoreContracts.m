function tests = testCoreContracts
% Check public inputs, stable outcomes, exact clocks, and target derivatives.
% Run with runtests('tests/testCoreContracts.m').
tests = functiontests(localfunctions);
end

function setupOnce(~)
    rootFolder = fileparts(fileparts(mfilename('fullpath')));
    addpath(rootFolder, fullfile(rootFolder, 'trajectory'), fullfile(rootFolder, 'examples'));
end

function testPublicInputContracts(testCase)
    baseLimits = standardLimits();
    audit = auditLimits();
    initialState = struct('time_s', 0, 'position_units', [-4, 0]);
    goalState = struct('time_s', 12, 'position_units', [4, 0]);
    runCases(testCase, { ...
        @(testCase) fixedArrivalCoincidentEndpointsStillThrow(testCase, audit); ...
        @(testCase) matchedDerivativeConflictUsesNormalizedValue(testCase, audit); ...
        @benchmarkDefaultTemplatePathExists; ...
        @(testCase) stateValidationDecisions(testCase, baseLimits); ...
        @(testCase) backwardsTimePrecedesWrappedTargetEvaluation(testCase, baseLimits); ...
        @(testCase) backwardsTimeOnUnwrappedFixedGoal(testCase, baseLimits); ...
        @(testCase) limitValidationDecisions(testCase, baseLimits); ...
        @(testCase) optionValidationDecisions(testCase, baseLimits); ...
        @(testCase) targetDerivativeValidationDecisions(testCase, baseLimits); ...
        @(testCase) targetHistoryValidationDecisions(testCase, baseLimits); ...
        @(testCase) invalidInputs(testCase, initialState, goalState)});
end

function testPublicOutcomeAndClockContracts(testCase)
    baseLimits = standardLimits();
    core = coreInputs();
    fixed = fixedInputs();
    runCases(testCase, { ...
        @(testCase) givenArrivalAndContinuousMotion(testCase, fixed); ...
        @movingTargetEndsAtExactGivenClock; ...
        @(testCase) staticGoalUsesExactGivenClock(testCase, fixed.Limits); ...
        @(testCase) fixedTimedSearchRetainsBoundaryVelocity(testCase, fixed); ...
        @(testCase) initialEndpointBlocked(testCase, baseLimits); ...
        @(testCase) terminalReachabilityBlocked(testCase, baseLimits); ...
        @(testCase) endpointDerivativeLimit(testCase, baseLimits); ...
        @(testCase) endpointOutsideWorkspace(testCase, baseLimits); ...
        @(testCase) workspaceBoundaryDerivativeIsRejectedBeforePlanning(testCase, baseLimits); ...
        @(testCase) workspaceOvershootFallsThroughToBmtp(testCase, baseLimits); ...
        @(testCase) passingAnalyticProbeStillChecksEveryCollisionPair(testCase, baseLimits); ...
        @(testCase) timeWindowInfeasible(testCase, baseLimits); ...
        @(testCase) earliestStaticDirectUsesAnalyticClock(testCase, baseLimits); ...
        @c3ChordDropsRoundoffZeroPhases; ...
        @jerkChordPreservesShortPhysicalRampsBesideLongCruise; ...
        @(testCase) noPath(testCase, core); ...
        @movingCellFringeBlocksTerminalReachability});
end

function testMovingTargetClockAndDerivativeContracts(testCase)
    baseLimits = standardLimits();
    audit = auditLimits();
    runCases(testCase, { ...
        @(testCase) earliestInterceptIsPlannedWhenTargetReachesStartAtDeadline(testCase, audit); ...
        @(testCase) positionOnlyInterceptAtLinearTargetCorner(testCase, audit); ...
        @(testCase) earliestStaticNonrestUsesPhysicalClockTrials(testCase, baseLimits); ...
        @(testCase) fixedMovingTargetMatchesPchipDerivatives(testCase, baseLimits); ...
        @(testCase) earliestMovingTargetUsesArrivalTimeClock(testCase, baseLimits); ...
        @(testCase) earliestMovingTargetMatchesSelectedClockDerivatives(testCase, baseLimits); ...
        @(testCase) movingTargetTrialRetainsTrialClockDerivatives(testCase, baseLimits); ...
        @(testCase) disconnectedMovingTargetClockAdvances(testCase, baseLimits)});
end

function fixedArrivalCoincidentEndpointsStillThrow(testCase, limits)
    initial = struct('time_s', 0, 'position_units', [0 0]);
    fixedGoal = struct('time_s', 10, 'position_units', [0 0]);
    verifyError(testCase, @() planner([], initial, fixedGoal, limits, ...
        struct('GoalTimeMode', 'fixedArrival')), 'planTrajectory:CoincidentEndpoints');
    targetMotion = struct('time_s', [0; 10], 'position_units', [10 0; 0 0]);
    targetGoal = struct('time_s', 10, 'targetMotion', targetMotion);
    verifyError(testCase, @() planner([], initial, targetGoal, limits, ...
        struct('GoalTimeMode', 'fixedArrival')), 'planTrajectory:CoincidentEndpoints');
end

function matchedDerivativeConflictUsesNormalizedValue(testCase, limits)
    targetMotion = struct('time_s', [0; 10], 'position_units', [0 0; 10 0]);
    initial = struct('time_s', 0, 'position_units', [-4 0]);
    options = struct('GoalTimeMode', 'fixedArrival', 'MatchTargetVelocity', true);

    columnGoal = struct('time_s', 10, 'targetMotion', targetMotion, 'velocity_units_s', [1; 0]);
    result = planner([], initial, columnGoal, limits, options);
    verifyTrue(testCase, result.Success, result.Message);
    verifyEqual(testCase, result.Inputs.goalState.velocity_units_s, [1 0], 'AbsTol', 1e-12);

    conflictingGoal = struct('time_s', 10, 'targetMotion', targetMotion, 'velocity_units_s', [0 1]);
    verifyError(testCase, @() planner([], initial, conflictingGoal, limits, options), ...
        'planner:ConflictingTargetDerivative');
end

function benchmarkDefaultTemplatePathExists(testCase)
    % The default template lives in an untracked folder, so only the
    % declared path is checked; its presence depends on the checkout.
    root = fileparts(fileparts(mfilename('fullpath')));
    benchmarkSource = fileread(fullfile(root, 'benchmarks', 'benchmarkRandomGoalVisibilityWindows.m'));
    verifyTrue(testCase, contains(benchmarkSource, '"Rogue Cases"'));
    verifyFalse(testCase, contains(benchmarkSource, 'RogueCasses'));
end

function stateValidationDecisions(testCase, baseLimits)
    goal = state(10, [4, 0]);
    verifyError(testCase, @() planner([], 42, goal, baseLimits, struct()), 'planTrajectory:InvalidState');
    initial = state(0, [0, 0]); initial.unusedField = 1;
    verifyError(testCase, @() planner([], initial, goal, baseLimits, struct()), 'planTrajectory:UnsupportedStateField');
    initial = state(0, [0, 0]); initial.position_units = [0, NaN];
    verifyError(testCase, @() planner([], initial, goal, baseLimits, struct()), 'planTrajectory:InvalidState');
    initial = state(NaN, [0, 0]);
    verifyError(testCase, @() planner([], initial, goal, baseLimits, struct()), 'MATLAB:expectedFinite');
    verifyError(testCase, @() planner([], state(0, [0, 0]), state(0, [4, 0]), ...
        baseLimits, struct()), 'planTrajectory:InvalidTimeOrder');
    verifyError(testCase, @() planner([], state(0, [0, 0]), state(10, [0, 0]), ...
        baseLimits, struct()), 'planTrajectory:CoincidentEndpoints');
end

function backwardsTimePrecedesWrappedTargetEvaluation(testCase, baseLimits)
    initial = state(5, [0, 0]);
    targetMotion = struct('time_s', [5; 10], ...
        'position_units', [1, 0; 2, 0], 'InterpolationMethod', 'linear');
    goal = struct('time_s', 0, 'targetMotion', targetMotion);
    verifyError(testCase, @() planner([], initial, goal, baseLimits, ...
        struct('WrapX', true)), 'planTrajectory:InvalidTimeOrder');
end

function backwardsTimeOnUnwrappedFixedGoal(testCase, baseLimits)
    verifyError(testCase, @() planner([], state(5, [0, 0]), state(0, [2, 0]), ...
        baseLimits, struct()), 'planTrajectory:InvalidTimeOrder');
end

function limitValidationDecisions(testCase, baseLimits)
    initial = state(0, [0, 0]); goal = state(10, [4, 0]);
    verifyError(testCase, @() planner([], initial, goal, 42, struct()), 'planTrajectory:InvalidLimits');
    limits = baseLimits; limits.unusedField = 1;
    verifyError(testCase, @() planner([], initial, goal, limits, struct()), 'planTrajectory:UnsupportedLimitField');
    limits = baseLimits; limits.xInterval_units = [1, 1];
    verifyError(testCase, @() planner([], initial, goal, limits, struct()), 'planTrajectory:InvalidWorkspace');
    limits = baseLimits; limits.maxVelocity_units_s = 2;
    verifyError(testCase, @() planner([], initial, goal, limits, struct()), 'planTrajectory:MixedLimitModes');
    limits = baseLimits; limits.maxJerk_units_s3 = [4, 0];
    verifyError(testCase, @() planner([], initial, goal, limits, struct()), 'planTrajectory:InvalidDerivativeLimit');

    limits = baseLimits;
    limits.maxVelocity_units_s = 2;
    limits.maxAcceleration_units_s2 = 2;
    limits.maxJerk_units_s3 = 4;
    result = planner([], initial, goal, limits, struct('GoalTimeMode', 'fixedArrival'));
    verifyTrue(testCase, result.Success, result.Message);
    verifyEqual(testCase, result.Diagnostics.Limits.maxVelocity_units_s, [sqrt(2), sqrt(2)], 'AbsTol', 1e-12);
end

function optionValidationDecisions(testCase, baseLimits)
    initial = state(0, [0, 0]); goal = state(10, [4, 0]); limits = baseLimits;
    verifyError(testCase, @() planner([], initial, goal, limits, 42), 'planTrajectory:InvalidOptions');
    verifyError(testCase, @() planner([], initial, goal, limits, ...
        struct('GoalTimeMode', 'unsupported')), 'planner:UnsupportedGoalTimeMode');
    verifyError(testCase, @() planner([], initial, goal, limits, struct('WrapX', 2)), 'planner:InvalidWrapOption');
    verifyError(testCase, @() planner([], initial, goal, limits, ...
        struct('WrapY', "sideways")), 'planner:InvalidWrapOption');
    verifyError(testCase, @() planner([], initial, goal, limits, struct('SampleTime_s', 0)), 'MATLAB:expectedPositive');
    verifyError(testCase, @() planner([], initial, goal, limits, ...
        struct('MaxArrivalTrials', 0)), 'MATLAB:expectedPositive');
    verifyError(testCase, @() planner([], initial, goal, limits, ...
        struct('MaxArrivalTrials', 1.5)), 'MATLAB:expectedInteger');
    verifyError(testCase, @() planner([], initial, goal, limits, ...
        struct('MaxArrivalCandidates', 0)), 'MATLAB:expectedPositive');
    verifyError(testCase, @() planner([], initial, goal, limits, ...
        struct('MaxArrivalCandidates', 1.5)), 'MATLAB:expectedInteger');
    verifyError(testCase, @() planner([], initial, goal, limits, ...
        struct('BestSoFarRefinementTrialLimit', -1)), 'MATLAB:expectedNonnegative');
    verifyError(testCase, @() planner([], initial, goal, limits, ...
        struct('BestSoFarRefinementTrialLimit', 1.5)), 'MATLAB:expectedInteger');
    verifyError(testCase, @() planner([], initial, goal, limits, ...
        struct('ArrivalImprovementTolerance_s', -0.01)), 'MATLAB:expectedNonnegative');
    verifyError(testCase, @() planner([], initial, goal, limits, ...
        struct('SpatialProbeIterationLimit', 0)), 'MATLAB:expectedPositive');
    verifyError(testCase, @() planner([], initial, goal, limits, struct('SpatialProbeIterationLimit', 36)), ...
        'planner:InvalidSpatialProbeIterationLimit');
    defaulted = planner([], initial, goal, limits, struct('SampleTime_s', []));
    verifyTrue(testCase, defaulted.Success, defaulted.Message);
    verifyEqual(testCase, defaulted.Options.SampleTime_s, 0.05);
    verifyEqual(testCase, defaulted.Options.BestSoFarRefinementTrialLimit, 0);
    verifyEqual(testCase, defaulted.Options.ArrivalImprovementTolerance_s, 0.01);
    verifyWarning(testCase, @() planner([], initial, goal, limits, ...
        struct('unusedOption', 1)), 'planTrajectory:UnknownOptions');
end

function targetDerivativeValidationDecisions(testCase, baseLimits)
    initial = state(0, [0, 0]); limits = baseLimits;
    verifyError(testCase, @() planner([], initial, state(10, [4, 0]), limits, ...
        struct('MatchTargetVelocity', true)), 'planner:MissingTarget');
    targetMotion = struct('time_s', [0; 10], ...
        'position_units', [3, 0; 4, 0], 'InterpolationMethod', 'linear');
    goal = struct('time_s', 10, 'targetMotion', targetMotion, ...
        'velocity_units_s', [0, 0]);
    verifyError(testCase, @() planner([], initial, goal, limits, struct('MatchTargetVelocity', true)), ...
        'planner:ConflictingTargetDerivative');

    goal.velocity_units_s = [0.1, 0];
    matched = planner([], initial, goal, limits, ...
        struct('GoalTimeMode', 'fixedArrival', 'MatchTargetVelocity', true));
    verifyTrue(testCase, matched.Success, matched.Message);
    verifyEqual(testCase, matched.velocity_units_s(end, :), [0.1, 0], 'AbsTol', 1e-8);

    goal = struct('time_s', 10, 'targetMotion', targetMotion);
    accelerationOnly = planner([], initial, goal, limits, ...
        struct('GoalTimeMode', 'fixedArrival', 'MatchTargetAcceleration', true));
    verifyTrue(testCase, accelerationOnly.Success, accelerationOnly.Message);
    verifyEqual(testCase, accelerationOnly.acceleration_units_s2(end, :), [0, 0], 'AbsTol', 1e-8);
end

function targetHistoryValidationDecisions(testCase, baseLimits)
    initial = state(0, [0, 0]); limits = baseLimits;
    goal = struct('time_s', 10, 'targetMotion', struct('time_s', [0; 10]));
    verifyError(testCase, @() planner([], initial, goal, limits, struct()), 'planner:InvalidTarget');

    motion = struct('time_s', [0; 0], 'position_units', [3, 0; 4, 0]);
    goal = struct('time_s', 0, 'targetMotion', motion);
    verifyError(testCase, @() planner([], initial, goal, limits, struct()), 'planner:InvalidTargetTime');

    motion = struct('time_s', [0; 10], 'position_units', [3, 0; 4, 0]);
    goal = struct('time_s', 11, 'targetMotion', motion);
    verifyError(testCase, @() planner([], initial, goal, limits, struct()), 'planner:TargetTimeOutsideHistory');

    motion.InterpolationMethod = 'unsupported';
    goal = struct('time_s', 5, 'targetMotion', motion);
    verifyError(testCase, @() planner([], initial, goal, limits, struct()), 'planner:InvalidTargetInterpolation');

    motion = struct('time_s', [0; 5; 10], ...
        'position_units', [3, 0; 4, 0; 6, 0], 'InterpolationMethod', 'linear');
    goal = struct('time_s', 5, 'targetMotion', motion);
    verifyError(testCase, @() planner([], initial, goal, limits, struct('MatchTargetVelocity', true)), ...
        'planner:UndefinedTargetDerivative');
end

function invalidInputs(testCase, initialState, goalState)
    initial = initialState; initial.position_units = [NaN 0];
    verifyError(testCase, @() planner([], initial, goalState), 'planTrajectory:InvalidState');
    goal = goalState; goal.time_s = -1;
    verifyError(testCase, @() planner([], initialState, goal), 'planTrajectory:InvalidTimeOrder');
end

function givenArrivalAndContinuousMotion(testCase, data)
    result = planner([], data.Initial, data.Goal, data.Limits, data.Options);
    assertTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase, result.ArrivalTime_s, 10, 'AbsTol', 1e-8);
    verifyEqual(testCase, result.Diagnostics.VisibilityGraph.SearchKind, "initialSpatialSnapshot");
    verifyTrue(testCase, result.Diagnostics.VisibilityGraph.GraphIsFullyEnumerated);
    verifyLessThan(testCase, size(result.Diagnostics.Route_units, 1), 9);
end

function movingTargetEndsAtExactGivenClock(testCase)
    goalTime_s = 13.984378262112314;
    initial = struct('time_s', 0.45999999999999996, 'position_units', [-4, 0]);
    targetMotion = struct( ...
        'time_s', [initial.time_s; goalTime_s], ...
        'position_units', [4, 0; 5, 0]);
    goal = struct('time_s', goalTime_s, 'targetMotion', targetMotion);
    limits = struct( ...
        'xInterval_units', [-20, 20], ...
        'yInterval_units', [-8, 8], ...
        'maxVelocity_units_s', [3, 3], ...
        'maxAcceleration_units_s2', [2, 2], ...
        'maxJerk_units_s3', [4, 4]);
    options = struct('GoalTimeMode', 'fixedArrival', 'WrapX', true);

    result = planner([], initial, goal, limits, options);

    assertTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase, result.ArrivalTime_s, goal.time_s);
    verifyEqual(testCase, result.Diagnostics.Polynomial.FinalTime_s, goal.time_s);
    verifyEqual(testCase, result.time_s(end), goal.time_s);
end

function staticGoalUsesExactGivenClock(testCase, limits)
    % This clock pair reproduces one ulp late when the arrival is rebuilt
    % from the start time plus summed durations.
    initial = struct('time_s', 0.45999999999999996, 'position_units', [0, 0]);
    goal = struct('time_s', 13.984378262112314, 'position_units', [5, 1]);

    result = planner([], initial, goal, limits, struct('GoalTimeMode', 'fixedArrival'));

    assertTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase, result.ArrivalTime_s, goal.time_s);
    verifyEqual(testCase, result.Diagnostics.Polynomial.FinalTime_s, goal.time_s);
    verifyEqual(testCase, result.time_s(end), goal.time_s);
end

function fixedTimedSearchRetainsBoundaryVelocity(testCase, data)
    data.Initial.velocity_units_s = [0.1, 0];
    result = planner([], data.Initial, data.Goal, data.Limits, data.Options);
    assertTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, result.Diagnostics.Validation.Passed);
    verifyEqual(testCase, result.velocity_units_s([1, end], :), [0.1, 0; 0, 0], 'AbsTol', 1e-8);
end

function initialEndpointBlocked(testCase, baseLimits)
    obstacle = struct('Vertices_units', [-5, -1; -3, -1; -3, 1; -5, 1]);
    result = planner(obstacle, state(0, [-4, 0]), state(10, [4, 0]), ...
        baseLimits, struct('GoalTimeMode', 'fixedArrival'));
    verifyFailure(testCase, result, "endpointBlocked");
end

function terminalReachabilityBlocked(testCase, baseLimits)
    box = [-2, -2; 2, -2; 2, 2; -2, 2];
    obstacle = obstacleAvoidance.obstacles.createObstacle('terminal box', [0; 9], ...
        {box(:, 1); box(:, 1)}, {box(:, 2); box(:, 2)}, 0);
    limits = baseLimits;
    limits.xInterval_units = [-12, 12];
    result = planner(obstacle, state(0, [-10, 0]), state(10, [0, 0]), limits, ...
        struct('GoalTimeMode', 'fixedArrival'));
    verifyFailure(testCase, result, "terminalReachabilityBlocked");
end

function endpointDerivativeLimit(testCase, baseLimits)
    initial = state(0, [-4, 0]);
    initial.velocity_units_s = [3, 0];
    result = planner([], initial, state(10, [4, 0]), baseLimits, ...
        struct('GoalTimeMode', 'fixedArrival'));
    verifyFailure(testCase, result, "dynamicEndpointInfeasible");
end

function endpointOutsideWorkspace(testCase, baseLimits)
    result = planner([], state(0, [-7, 0]), state(10, [4, 0]), baseLimits, ...
        struct('GoalTimeMode', 'fixedArrival'));
    verifyFailure(testCase, result, "endpointOutsideWorkspace");
end

function workspaceBoundaryDerivativeIsRejectedBeforePlanning(testCase, baseLimits)
    limits = baseLimits; limits.xInterval_units = [-1, 1];
    initial = state(0, [1, 0]); initial.velocity_units_s = [1, 0];
    result = planner([], initial, state(10, [0, 0]), limits, ...
        struct('GoalTimeMode', 'fixedArrival'));
    verifyFailure(testCase, result, "dynamicEndpointInfeasible");
    verifyEqual(testCase, result.Diagnostics.VisibilityGraph.SearchKind, "notSearched");

    goal = state(10, [-1, 0]); goal.velocity_units_s = [1, 0];
    result = planner([], state(0, [0, 0]), goal, limits, ...
        struct('GoalTimeMode', 'fixedArrival'));
    verifyFailure(testCase, result, "dynamicEndpointInfeasible");
    verifyEqual(testCase, result.Diagnostics.VisibilityGraph.SearchKind, "notSearched");

    initial = state(0, [1, 0]); initial.acceleration_units_s2 = [1, 0];
    result = planner([], initial, state(10, [0, 0]), limits, ...
        struct('GoalTimeMode', 'fixedArrival'));
    verifyFailure(testCase, result, "dynamicEndpointInfeasible");

    goal = state(10, [-1, 0]); goal.acceleration_units_s2 = [-1, 0];
    result = planner([], state(0, [0, 0]), goal, limits, ...
        struct('GoalTimeMode', 'fixedArrival'));
    verifyFailure(testCase, result, "dynamicEndpointInfeasible");
end

function workspaceOvershootFallsThroughToBmtp(testCase, baseLimits)
    limits = baseLimits; limits.xInterval_units = [-1, 1];
    initial = state(0, [0.5, 0]);
    initial.velocity_units_s = [0.3, 0];
    result = planner([], initial, state(10, [0, 0]), limits, ...
        struct('GoalTimeMode', 'fixedArrival'));
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    verifyLessThanOrEqual(testCase, max(result.position_units(:, 1)), 1);
end

function passingAnalyticProbeStillChecksEveryCollisionPair(testCase, baseLimits)
    rectangle = [4, 20; 6, 20; 6, 22; 4, 22];
    obstacle = obstacleAvoidance.obstacles.createObstacle( ...
        'distant translating rectangle', [0; 5; 10], ...
        {rectangle(:, 1); rectangle(:, 1); rectangle(:, 1)}, ...
        {rectangle(:, 2); rectangle(:, 2) + 1; rectangle(:, 2) + 2}, 0.1);
    limits = baseLimits;
    limits.xInterval_units = [-5, 15]; limits.yInterval_units = [-5, 30];
    result = planner(obstacle, state(0, [0, 0]), state(10, [10, 0]), limits, ...
        struct('GoalTimeMode', 'fixedArrival'));
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    verifyGreaterThan(testCase, result.Diagnostics.SeparationProof.AllPairCount, 0);
    verifyEqual(testCase, result.Diagnostics.SeparationProof.VerifiedPairCount, ...
        result.Diagnostics.SeparationProof.AllPairCount);
end

function timeWindowInfeasible(testCase, baseLimits)
    limits = baseLimits;
    limits.maxVelocity_units_s = [1, 1];
    result = planner([], state(0, [-5, 0]), state(1, [5, 0]), limits, ...
        struct('GoalTimeMode', 'fixedArrival'));
    verifyFailure(testCase, result, "timeWindowInfeasible");
end

function earliestStaticDirectUsesAnalyticClock(testCase, baseLimits)
    result = planner([], state(0, [0, 0]), state(10, [4, 1]), baseLimits, ...
        struct('GoalTimeMode', 'earliestArrival'));
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase, result.Diagnostics.VisibilityGraph.SearchKind, "initialSpatialSnapshot");
    verifyTrue(testCase, result.Diagnostics.VisibilityGraph.GraphIsFullyEnumerated);
    verifyEqual(testCase, numel(result.Diagnostics.Attempts), 1);
    verifyEqual(testCase, result.Diagnostics.Attempts.Kind, "spatialVisibility");
    verifyTrue(testCase, result.Diagnostics.Attempts.Selected);
    verifyFalse(testCase, result.Diagnostics.EarliestArrival.GlobalEarliestProven);
    verifyEqual(testCase, result.Diagnostics.EarliestArrival.SelectedAttemptIndex, 1);
end

function c3ChordDropsRoundoffZeroPhases(testCase)
    % A regime-boundary hold can evaluate to a positive 1e-16-second
    % remnant. It must not become the smoothing kernel and erase the chord.
    limits = struct('maxVelocity_units_s', [10, 10], ...
        'maxAcceleration_units_s2', [10, 10], ...
        'maxJerk_units_s3', [10, 10]);
    start_units = [-5, 0];
    goal_units = [0.201, 2.999];
    [controls_units, durations_s, powers_units] = ...
        bmtpEngine.motion.createC3Chord(start_units, goal_units, limits);
    verifyGreaterThan(testCase, min(durations_s), 1e-6);
    verifyEqual(testCase, squeeze(controls_units(1, 1, :)).', start_units, 'AbsTol', 1e-12);
    verifyEqual(testCase, squeeze(controls_units(end, end, :)).', goal_units, 'AbsTol', 1e-10);
    verifyEqual(testCase, sum(squeeze(powers_units(end, :, :)), 2).', goal_units, 'AbsTol', 1e-10);
end

function jerkChordPreservesShortPhysicalRampsBesideLongCruise(testCase)
    limits = struct('maxVelocity_units_s', [1e-9, 1e-9], ...
        'maxAcceleration_units_s2', [1e5, 1e5], ...
        'maxJerk_units_s3', [1e18, 1e18]);
    [controls_units, durations_s] = bmtpEngine.motion.createJerkLimitedChord( ...
        [0, 0], [1, 0], limits, 3);
    verifyGreaterThan(testCase, numel(durations_s), 1);
    verifyGreaterThan(testCase, min(durations_s), 0);
    verifyEqual(testCase, squeeze(controls_units(1, 1, :)).', [0, 0], 'AbsTol', 1e-14);
    verifyEqual(testCase, squeeze(controls_units(end, end, :)).', [1, 0], 'AbsTol', 1e-12);
end

function noPath(testCase, data)
    obstacle = struct('Vertices_units', [-1 -5; 1 -5; 1 5; -1 5]);
    r = planner(obstacle, data.Initial, data.Goal, data.Limits, data.Options);
    verifyFalse(testCase, r.Success);
    verifyEqual(testCase, r.TerminationReason, "noVisibilityRoute");
    verifyEmpty(testCase, r.time_s);
    earliest = planner(obstacle, data.Initial, data.Goal, ...
        data.Limits, struct('GoalTimeMode', 'earliestArrival'));
    verifyFalse(testCase, earliest.Success);
    verifyEqual(testCase, earliest.TerminationReason, "noVisibilityRoute");
    verifyEqual(testCase, earliest.Diagnostics.Attempts.FailureKind, "noSpatialRoute");
end

function movingCellFringeBlocksTerminalReachability(testCase)
    % The moving-cell enclosure hulls carried triangles with margin squares, so it
    % reaches past every protected sample. A fixed goal that is free of the
    % samples but inside that fringe must still be proven unreachable before
    % any planning stage runs.
    lower = [-1, -1; 1, -1; 1, 1; -1, 1];
    upper = [-1, -1; 1, -1; 0.5, 1; -1, 1];
    obstacle = obstacleAvoidance.obstacles.createObstacle('swept fringe', [0; 10], ...
        {lower(:, 1); upper(:, 1)}, {lower(:, 2); upper(:, 2)}, 1, ...
        struct('vertexCorrespondence', 'sourceIndex'));
    prepared = obstacleAvoidance.obstacles.prepareObstacles(obstacle, [0, 10], true);
    goal = struct('time_s', 10, 'position_units', [2.3, 0]);
    verifyTrue(testCase, all(cellfun(@max, prepared.x_units) < goal.position_units(1)));
    verifyTrue(testCase, prepared.InternalPreparation.IntervalUsesMovingCells(1));
    initial = struct('time_s', 0, 'position_units', [-5, 0]);
    limits = struct('xInterval_units', [-8, 8], 'yInterval_units', [-8, 8], ...
        'maxVelocity_units_s', [3, 3], 'maxAcceleration_units_s2', [2, 2], 'maxJerk_units_s3', [4, 4]);
    result = planner(obstacle, initial, goal, limits, struct('GoalTimeMode', 'fixedArrival'));
    verifyFalse(testCase, result.Success);
    verifyEqual(testCase, result.TerminationReason, "terminalReachabilityBlocked");
end

function earliestInterceptIsPlannedWhenTargetReachesStartAtDeadline(testCase, limits)
    % The deadline position of a moving target only bounds the search.
    targetMotion = struct('time_s', [0; 10], 'position_units', [10 0; 0 0]);
    initial = struct('time_s', 0, 'position_units', [0 0]);
    goal = struct('time_s', 10, 'targetMotion', targetMotion);
    result = planner([], initial, goal, limits, ...
        struct('GoalTimeMode', 'earliestArrival'));
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    verifyLessThan(testCase, result.ArrivalTime_s, 10);
end

function positionOnlyInterceptAtLinearTargetCorner(testCase, limits)
    % No derivative matching is requested, so the corner's undefined
    % derivative must not reject the intercept.
    targetMotion = struct('time_s', [0; 10; 20], 'position_units', [0 0; 4 0; 4 4]);
    initial = struct('time_s', 0, 'position_units', [-4 0]);
    goal = struct('time_s', 10, 'targetMotion', targetMotion);
    result = planner([], initial, goal, limits, ...
        struct('GoalTimeMode', 'fixedArrival'));
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase, result.Diagnostics.Intercept.TargetPosition_units, [4 0], 'AbsTol', 1e-9);
    verifyFalse(testCase, isfield(result.Diagnostics.Intercept, 'TargetVelocity_units_s'));
end

function earliestStaticNonrestUsesPhysicalClockTrials(testCase, baseLimits)
    initial = state(0, [0, 0]); initial.velocity_units_s = [0.1, 0];
    result = planner([], initial, state(10, [4, 0]), baseLimits, ...
        struct('GoalTimeMode', 'earliestArrival'));
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    verifyTrue(testCase, isfield(result.Diagnostics, 'TemporalSearch'));
    verifyEqual(testCase, result.Diagnostics.FixedArrivalTrialTime_s, result.ArrivalTime_s, 'AbsTol', 1e-12);
    verifyFalse(testCase, result.Diagnostics.TemporalSearch.GlobalEarliestProven);
    verifyFalse(testCase, result.Diagnostics.EarliestArrival.Capabilities.TimedVariableClockBmtp);
    verifyGreaterThanOrEqual(testCase, numel(result.Diagnostics.Attempts), 1);
    verifyTrue(testCase, all([result.Diagnostics.Attempts.Kind] == "arrivalTimeTrial"));
    verifyEqual(testCase, nnz([result.Diagnostics.Attempts.Selected]), 1);
end

function fixedMovingTargetMatchesPchipDerivatives(testCase, baseLimits)
    targetMotion = struct('time_s', [0; 5; 10], ...
        'position_units', [2, 0; 3, 0; 4, 0], ...
        'InterpolationMethod', 'pchip');
    goal = struct('time_s', 10, 'targetMotion', targetMotion);
    options = struct('GoalTimeMode', 'fixedArrival', ...
        'MatchTargetVelocity', true, 'MatchTargetAcceleration', true);
    result = planner([], state(0, [0, 0]), goal, baseLimits, options);
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase, result.Diagnostics.Intercept.Time_s, 10, 'AbsTol', 1e-10);
    verifyEqual(testCase, result.Diagnostics.Intercept.TargetVelocity_units_s, [0.2, 0], 'AbsTol', 1e-10);
    verifyEqual(testCase, result.Diagnostics.Intercept.TargetAcceleration_units_s2, [0, 0], 'AbsTol', 1e-10);
    verifyEqual(testCase, result.velocity_units_s(end, :), [0.2, 0], 'AbsTol', 1e-8);
    verifyEqual(testCase, result.acceleration_units_s2(end, :), [0, 0], 'AbsTol', 1e-8);
end

function earliestMovingTargetUsesArrivalTimeClock(testCase, baseLimits)
    targetTime_s = (0:2:12).';
    targetMotion = struct('time_s', targetTime_s, ...
        'position_units', [4 + 0.1 * targetTime_s, ones(size(targetTime_s))], ...
        'InterpolationMethod', 'linear');
    goal = struct('time_s', 12, 'targetMotion', targetMotion);
    result = planner([], state(0, [0, 0]), goal, baseLimits, ...
        struct('GoalTimeMode', 'earliestArrival', 'TemporalResolution_s', 0.5));
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    verifyTrue(testCase, isfield(result.Diagnostics, 'TemporalSearch'));
    verifyFalse(testCase, result.Diagnostics.TemporalSearch.GlobalEarliestProven);
    verifyEqual(testCase, result.Diagnostics.FixedArrivalTrialTime_s, result.ArrivalTime_s, 'AbsTol', 1e-10);
    verifyEqual(testCase, result.Diagnostics.Intercept.Time_s, result.ArrivalTime_s, 'AbsTol', 1e-10);
    verifyFalse(testCase, result.Diagnostics.EarliestArrival.Capabilities.TimedVariableClockBmtp);
    verifyGreaterThanOrEqual(testCase, numel(result.Diagnostics.Attempts), 1);
    verifyTrue(testCase, all([result.Diagnostics.Attempts.Kind] == "arrivalTimeTrial"));
    verifyEqual(testCase, nnz([result.Diagnostics.Attempts.Selected]), 1);
end

function earliestMovingTargetMatchesSelectedClockDerivatives(testCase, baseLimits)
    targetMotion = struct('time_s', [0; 5; 10], ...
        'position_units', [2, 0; 3, 0; 4, 0], ...
        'InterpolationMethod', 'linear');
    goal = struct('time_s', 10, 'targetMotion', targetMotion);
    options = struct('GoalTimeMode', 'earliestArrival', ...
        'MatchTargetVelocity', true, 'MatchTargetAcceleration', true, ...
        'TemporalResolution_s', 0.5);
    result = planner([], state(0, [0, 0]), goal, baseLimits, options);
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    verifyLessThan(testCase, result.ArrivalTime_s, goal.time_s);
    [targetPosition_units, targetVelocity_units_s, targetAcceleration_units_s2] = ...
        obstacleAvoidance.input.targetPositionAtTime( ...
        targetMotion, result.ArrivalTime_s);
    verifyEqual(testCase, result.Diagnostics.Intercept.TargetPosition_units, targetPosition_units, 'AbsTol', 1e-10);
    verifyEqual(testCase, result.velocity_units_s(end, :), targetVelocity_units_s, 'AbsTol', 1e-8);
    verifyEqual(testCase, result.acceleration_units_s2(end, :), targetAcceleration_units_s2, 'AbsTol', 1e-8);
end

function movingTargetTrialRetainsTrialClockDerivatives(testCase, baseLimits)
    targetMotion = struct('time_s', [0; 4.3; 10], ...
        'position_units', [2, 0; 2.43, 0; 5.28, 0], ...
        'InterpolationMethod', 'linear');
    initial = state(0, [0, 0]);
    goal = struct('time_s', 10, 'targetMotion', targetMotion);
    options = struct('GoalTimeMode', 'earliestArrival', ...
        'MatchTargetVelocity', true, 'MatchTargetAcceleration', true, ...
        'TemporalResolution_s', 0.5);
    result = planner([], initial, goal, baseLimits, options);

    expectedTrialTime_s = 3;
    firstDuration_s = targetMotion.time_s(2) - targetMotion.time_s(1);
    expectedTrialVelocity_units_s = ...
        (targetMotion.position_units(2, :) - targetMotion.position_units(1, :))/firstDuration_s;
    expectedTrialPosition_units = targetMotion.position_units(1, :)+ ...
        (expectedTrialTime_s - targetMotion.time_s(1)) * expectedTrialVelocity_units_s;
    expectedTrialAcceleration_units_s2 = [0, 0];
    finalDuration_s = targetMotion.time_s(3) - targetMotion.time_s(2);
    expectedHorizonVelocity_units_s = ...
        (targetMotion.position_units(3, :) - targetMotion.position_units(2, :))/finalDuration_s;

    verifyTrue(testCase, result.Success, result.Message);
    verifyEqual(testCase, result.Inputs.goalState.time_s, goal.time_s);
    verifyEqual(testCase, result.Diagnostics.FixedArrivalTrialTime_s, expectedTrialTime_s, 'AbsTol', 1e-12);
    verifyEqual(testCase, result.ArrivalTime_s, expectedTrialTime_s, 'AbsTol', 1e-12);
    verifyEqual(testCase, result.Inputs.goalState.targetMotion, targetMotion);
    verifyEqual(testCase, result.Inputs.goalState.position_units, expectedTrialPosition_units, 'AbsTol', 1e-12);
    verifyEqual(testCase, result.Inputs.goalState.velocity_units_s, expectedTrialVelocity_units_s, 'AbsTol', 1e-12);
    verifyEqual(testCase, result.Inputs.goalState.acceleration_units_s2, ...
        expectedTrialAcceleration_units_s2, 'AbsTol', 1e-12);
    verifyEqual(testCase, result.Diagnostics.Intercept.Time_s, expectedTrialTime_s, 'AbsTol', 1e-12);
    verifyEqual(testCase, result.Diagnostics.Intercept.TargetPosition_units, ...
        expectedTrialPosition_units, 'AbsTol', 1e-12);
    verifyEqual(testCase, result.Diagnostics.Intercept.TargetVelocity_units_s, ...
        expectedTrialVelocity_units_s, 'AbsTol', 1e-12);
    verifyEqual(testCase, result.Diagnostics.Intercept.TargetAcceleration_units_s2, ...
        expectedTrialAcceleration_units_s2, 'AbsTol', 1e-12);
    verifyGreaterThan(testCase, norm(expectedHorizonVelocity_units_s- expectedTrialVelocity_units_s), 0.3);
    verifyGreaterThan(testCase, norm(result.Diagnostics.Intercept.TargetVelocity_units_s- ...
        expectedHorizonVelocity_units_s), 0.3);
end

function disconnectedMovingTargetClockAdvances(testCase, baseLimits)
    wall_units = [-0.4, -5; 0.4, -5; 0.4, 5; -0.4, 5];
    obstacle = obstacleAvoidance.obstacles.createObstacle( ...
        'static separating wall', 0, {wall_units(:, 1)}, {wall_units(:, 2)}, 0);
    targetMotion = struct('time_s', [0; 10], ...
        'position_units', [4, 0; -2, 0], 'InterpolationMethod', 'linear');
    goal = struct('time_s', 10, 'targetMotion', targetMotion);
    limits = baseLimits;
    limits.xInterval_units = [-5, 5];
    limits.yInterval_units = [-5, 5];
    result = planner(obstacle, state(0, [-4, 0]), goal, limits, ...
        struct('GoalTimeMode', 'earliestArrival', 'TemporalResolution_s', 1));
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    verifyGreaterThan(testCase, numel(result.Diagnostics.Attempts), 1);
    verifyEqual(testCase, result.Diagnostics.Attempts(1).FailureStage, "search");
    verifyTrue(testCase, result.Diagnostics.Attempts(1).NextMethodAllowed);
    verifyTrue(testCase, any([result.Diagnostics.Attempts.Selected]));

    fixedInitial = state(0, [-4, 0]);
    fixedInitial.velocity_units_s = [0.1, 0];
    fixedGoal = state(10, [4, 0]);
    fixedOptions = struct('GoalTimeMode', 'earliestArrival', ...
        'TemporalResolution_s', 1, 'WrapX', false, 'WrapY', false);
    fixedResult = planner(obstacle, fixedInitial, fixedGoal, limits, fixedOptions);
    verifyFalse(testCase, fixedResult.Success);
    verifyEqual(testCase, fixedResult.TerminationReason, "noVisibilityRoute");
    verifyEqual(testCase, numel(fixedResult.Diagnostics.Attempts), 1);
    verifyFalse(testCase, fixedResult.Diagnostics.Attempts.NextMethodAllowed);
    verifyTrue(testCase, fixedResult.Diagnostics.TemporalSearch.TerminalFailure);
    verifyEqual(testCase, fixedResult.Options.GoalTimeMode, "fixedArrival");
    verifyEqual(testCase, fixedResult.Options.WrapX, "false");
    verifyEqual(testCase, fixedResult.Options.WrapY, "false");
    verifyEqual(testCase, fixedResult.Diagnostics.SuppliedLimits, limits);
    verifyEqual(testCase, fixedResult.Diagnostics.SuppliedGoalState.position_units, fixedGoal.position_units);
    verifyEqual(testCase, fixedResult.Diagnostics.SuppliedGoalState.time_s, 4);
    % Anchored to the obstacle this test passed in, not to another field of
    % the same result: a record agreeing with itself proves nothing here.
    verifyEqual(testCase, fixedResult.Inputs.obstacles, ...
        obstacleAvoidance.obstacles.prepareObstacles(obstacle, [0, 4], true));
    verifyEqual(testCase, fixedResult.Inputs.goalState.time_s, 4);
    verifyTrue(testCase, isfield(fixedResult.Diagnostics, 'ParentRequest'));
    verifyEqual(testCase, fixedResult.Diagnostics.ParentRequest.Obstacles, obstacle);
    verifyEqual(testCase, fixedResult.Diagnostics.ParentRequest.GoalTime_s, fixedGoal.time_s);
    % The supplied goal provenance is a different field from the outer clock;
    % relocating one does not cover the other.
    verifyEqual(testCase, fixedResult.Diagnostics.ParentRequest.SuppliedGoalState, fixedGoal);
    verifyEqual(testCase, fixedResult.Diagnostics.ParentRequest.GoalTimeMode, string(fixedOptions.GoalTimeMode));
    verifyEqual(testCase, fixedResult.Diagnostics.ParentRequest.FixedArrivalTrialTime_s, 4);
end

function limits = auditLimits()
    limits = struct('xInterval_units', [-20, 20], 'yInterval_units', [-20, 20], ...
        'maxVelocity_units_s', [2, 2], 'maxAcceleration_units_s2', [2, 2], ...
        'maxJerk_units_s3', [4, 4]);
end

function data = coreInputs()
    data.Initial = struct('time_s', 0, 'position_units', [-4, 0]);
    data.Goal = struct('time_s', 12, 'position_units', [4, 0]);
    data.Limits = struct('xInterval_units', [-6, 6], 'yInterval_units', [-4, 4], ...
        'maxVelocity_units_s', [2, 2], 'maxAcceleration_units_s2', [2, 2], ...
        'maxJerk_units_s3', [4, 4]);
    data.Options = struct('GoalTimeMode', 'fixedArrival');
end

function data = fixedInputs()
    data.Initial = struct('time_s', 0, 'position_units', [0, 0]);
    data.Goal = struct('time_s', 10, 'position_units', [5, 0]);
    data.Limits = struct('xInterval_units', [-1, 6], 'yInterval_units', [-2, 2], ...
        'maxVelocity_units_s', [2, 2], 'maxAcceleration_units_s2', [2, 2], ...
        'maxJerk_units_s3', [4, 4]);
    data.Options = struct('GoalTimeMode', 'fixedArrival');
end

function value = state(time_s, position_units)
    value = struct('time_s', time_s, 'position_units', position_units, ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
end

function limits = standardLimits()
    limits = struct('xInterval_units', [-6, 6], 'yInterval_units', [-6, 6], ...
        'maxVelocity_units_s', [2, 2], 'maxAcceleration_units_s2', [2, 2], ...
        'maxJerk_units_s3', [4, 4]);
end

function verifyFailure(testCase, result, reason)
    verifyFalse(testCase, result.Success);
    verifyEqual(testCase, result.TerminationReason, reason);
    verifyEmpty(testCase, result.time_s);
end
