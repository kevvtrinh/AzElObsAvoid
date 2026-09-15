function tests = testAuditPlannerFixes
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testAuditPlannerFixes.m')
% PURPOSE: Regress the planner request-handling defects found by the
%          read-only audit: earliest-target endpoint coincidence,
%          position-only intercepts at linear corners, normalized derivative
%          conflict checks, and the benchmark template path.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Function-based test results.
% UNITS: Coordinate units and seconds.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root, fullfile(root, 'trajectory'));
    testCase.TestData.Root   = root;
    testCase.TestData.Limits = struct( ...
        'xInterval_units',          [-20 20], ...
        'yInterval_units',          [-20 20], ...
        'maxVelocity_units_s',      [2 2], ...
        'maxAcceleration_units_s2', [2 2], ...
        'maxJerk_units_s3',         [4 4]);
end

function testEarliestInterceptIsPlannedWhenTargetReachesStartAtDeadline(testCase)
    % The deadline position of a moving target only bounds the search.
    targetMotion = struct('time_s', [0; 10], 'position_units', [10 0; 0 0]);
    initial      = struct('time_s', 0, 'position_units', [0 0]);
    goal         = struct('time_s', 10, 'targetMotion', targetMotion);
    result = planner([], initial, goal, testCase.TestData.Limits, ...
        struct('GoalTimeMode', 'earliestArrival'));
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    verifyLessThan(testCase, result.ArrivalTime_s, 10);
end

function testFixedArrivalCoincidentEndpointsStillThrow(testCase)
    initial = struct('time_s', 0, 'position_units', [0 0]);
    fixedGoal = struct('time_s', 10, 'position_units', [0 0]);
    verifyError(testCase, @() planner([], initial, fixedGoal, testCase.TestData.Limits, ...
        struct('GoalTimeMode', 'fixedArrival')), 'planTrajectory:CoincidentEndpoints');
    targetMotion = struct('time_s', [0; 10], 'position_units', [10 0; 0 0]);
    targetGoal   = struct('time_s', 10, 'targetMotion', targetMotion);
    verifyError(testCase, @() planner([], initial, targetGoal, testCase.TestData.Limits, ...
        struct('GoalTimeMode', 'fixedArrival')), 'planTrajectory:CoincidentEndpoints');
end

function testPositionOnlyInterceptAtLinearTargetCorner(testCase)
    % No derivative matching is requested, so the corner's undefined
    % derivative must not reject the intercept.
    targetMotion = struct('time_s', [0; 10; 20], 'position_units', [0 0; 4 0; 4 4]);
    initial      = struct('time_s', 0, 'position_units', [-4 0]);
    goal         = struct('time_s', 10, 'targetMotion', targetMotion);
    result = planner([], initial, goal, testCase.TestData.Limits, ...
        struct('GoalTimeMode', 'fixedArrival'));
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase, result.Intercept.TargetPosition_units, [4 0], 'AbsTol', 1e-9);
    verifyFalse(testCase, isfield(result.Intercept, 'TargetVelocity_units_s'));
end

function testMatchedDerivativeConflictUsesNormalizedValue(testCase)
    targetMotion = struct('time_s', [0; 10], 'position_units', [0 0; 10 0]);
    initial      = struct('time_s', 0, 'position_units', [-4 0]);
    options      = struct('GoalTimeMode', 'fixedArrival', 'MatchTargetVelocity', true);

    columnGoal = struct('time_s', 10, 'targetMotion', targetMotion, 'velocity_units_s', [1; 0]);
    result = planner([], initial, columnGoal, testCase.TestData.Limits, options);
    verifyTrue(testCase, result.Success, result.Message);
    verifyEqual(testCase, result.Inputs.goalState.velocity_units_s, [1 0], 'AbsTol', 1e-12);

    conflictingGoal = struct('time_s', 10, 'targetMotion', targetMotion, 'velocity_units_s', [0 1]);
    verifyError(testCase, @() planner([], initial, conflictingGoal, testCase.TestData.Limits, options), ...
        'planner:ConflictingTargetDerivative');
end

function testBenchmarkDefaultTemplatePathExists(testCase)
    root         = testCase.TestData.Root;
    templatePath = fullfile(root, "Rogue Cases", "x-y-request.json");
    verifyTrue(testCase, isfile(templatePath));
    benchmarkSource = fileread(fullfile(root, 'benchmarks', 'benchmarkRandomGoalVisibilityWindows.m'));
    verifyTrue(testCase, contains(benchmarkSource, '"Rogue Cases"'));
    verifyFalse(testCase, contains(benchmarkSource, 'RogueCasses'));
end
