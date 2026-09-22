function tests = testAuditValidationFixes
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testAuditValidationFixes.m')
% PURPOSE: Regress the independent-validator defects found by the read-only
%          audit: coverage metadata that excludes part of the motion, and
%          incomplete successful records that must fail instead of throw.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Function-based test results.
% UNITS: Coordinate units and seconds.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root, fullfile(root, 'trajectory'));
    limits = struct( ...
        'xInterval_units',          [-6 6], ...
        'yInterval_units',          [-4 4], ...
        'maxVelocity_units_s',      [2 2], ...
        'maxAcceleration_units_s2', [2 2], ...
        'maxJerk_units_s3',         [4 4]);
    initial = struct('time_s', 0, 'position_units', [-4 0]);
    goal    = struct('time_s', 10, 'position_units', [4 0]);
    result  = planner([], initial, goal, limits, struct('GoalTimeMode', 'fixedArrival'));
    assert(result.Success, result.Message);
    testCase.TestData.Result = result;
end

function testCoverageMetadataCannotExcludeTheMotion(testCase)
    % A record whose declared coverage ends at the start time carries an
    % empty dynamic proof; the straight motion crosses the square.
    result = testCase.TestData.Result;
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);

    square_units = [-0.5 -0.5; 0.5 -0.5; 0.5 0.5; -0.5 0.5];
    obstacle     = obstacleAvoidance.obstacles.createObstacle('center square', 0, ...
        square_units(:, 1), square_units(:, 2), 0);
    tampered = result;
    tampered.Inputs.obstacles                     = obstacle;
    tampered.TrajectoryCoverageEndTime_s          = result.Inputs.initialState.time_s;
    tampered.SeparationProof.Coverage            = struct( ...
        'Passed',               true, ...
        'ExactRegionCount',     0, ...
        'ActiveTimeInterval_s', zeros(0, 2), ...
        'EndRegions_units',     {cell(0, 1)});
    validation = obstacleAvoidance.validateTrajectory(tampered);
    verifyFalse(testCase, validation.Passed);
    verifyFalse(testCase, validation.SeparationProofValid);
end

function testContinuityProjectionMayOnlyAbsorbRoundoff(testCase)
    % A solver motion converts with a roundoff-sized join repair; a
    % discontinuous input is rejected instead of being silently connected.
    result                = testCase.TestData.Result;
    coordinateScale_units = max(1, max(abs(result.position_units), [], 'all'));
    verifyLessThanOrEqual(testCase, result.Polynomial.ContinuityProjectionDisplacement_units, ...
        1e-6 * coordinateScale_units);

    controls_units             = zeros(2, 6, 2);
    controls_units(2, :, 1)    = 10;
    polynomial = bmtpEngine.motion.createPowerPolynomial(controls_units, [1; 1], 0);
    verifyGreaterThan(testCase, polynomial.ContinuityProjectionDisplacement_units, 1);

    route_units = [result.Inputs.initialState.position_units; result.Inputs.goalState.position_units];
    seed        = struct('position_units', route_units, 'tau', [0; 1]);
    request     = bmtpEngine.pipeline.createSolveRequest(seed, ...
        struct('regions_units', {cell(0, 1)}, ...
        'coverage', struct('Passed', true, 'ExactRegionCount', 0)), ...
        struct('initialState', result.Inputs.initialState, ...
        'goalState', result.Inputs.goalState, ...
        'limits', result.Limits, ...
        'options', result.Options));
    prepared = bmtpEngine.pipeline.prepareFinalMotion(request, controls_units, [5; 5]);
    verifyFalse(testCase, prepared.Success);
    verifyEqual(testCase, prepared.TerminationReason, "continuityProjectionExceedsTolerance");
end

function testIncompleteSuccessfulRecordFailsWithoutThrowing(testCase)
    result = testCase.TestData.Result;

    missingArrival = rmfield(result, 'ArrivalTime_s');
    validation     = obstacleAvoidance.validateTrajectory(missingArrival);
    verifyFalse(testCase, validation.Passed);
    verifyTrue(testCase, contains(validation.Message, "record"));

    missingOption = result;
    missingOption.Options = rmfield(missingOption.Options, 'ArrivalTimeTolerance_s');
    validation = obstacleAvoidance.validateTrajectory(missingOption);
    verifyFalse(testCase, validation.Passed);

    missingIntercept = result;
    missingIntercept.Inputs.goalState.targetMotion = struct( ...
        'time_s', [0; 10], 'position_units', [4 0; 4 0]);
    missingIntercept = rmfield(missingIntercept, 'Intercept');
    validation = obstacleAvoidance.validateTrajectory(missingIntercept);
    verifyFalse(testCase, validation.Passed);
end
