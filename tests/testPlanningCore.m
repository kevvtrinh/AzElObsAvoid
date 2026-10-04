function tests = testPlanningCore
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testPlanningCore.m')
% PURPOSE: Exercise separate clock/constraint tolerances, detour validation,
%          tampered-motion rejection, and single obstacle margins.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Function-based test results.
% UNITS: Coordinate units and seconds.
tests = functiontests(localfunctions);
end

function setupOnce(~)
    rootFolder = fileparts(fileparts(mfilename('fullpath')));
    addpath(rootFolder, fullfile(rootFolder, 'trajectory'), fullfile(rootFolder, 'examples'));
end

function testTimeToleranceIsIndependentOfConstraintTolerance(testCase)
    initial = struct('time_s', 0, 'position_units', [-1, 0]);
    goal    = struct('time_s', 10, 'position_units', [1, 0]);
    limits  = struct( ...
        'xInterval_units',          [-5, 5], ...
        'yInterval_units',          [-5, 5], ...
        'maxVelocity_units_s',      [10, 10], ...
        'maxAcceleration_units_s2', [10, 10], ...
        'maxJerk_units_s3',         [10, 10]);
    looseTolerance      = 1e-3;
    tightTolerance      = 1e-8;
    crossedTolerances  = [looseTolerance, tightTolerance; tightTolerance, looseTolerance];
    expectedAcceptance = [false; true];
    clockOffset_s       = 1e-4;
    controlPoint_units  = zeros(1, 6, 2);
    controlPoint_units(1, :, 1) = [-1, -1, -1, 1, 1, 1];

    for settingIndex = 1:size(crossedTolerances, 1)
        options = struct( ...
            'GoalTimeMode',          'fixedArrival', ...
            'ConstraintTolerance',   crossedTolerances(settingIndex, 1), ...
            'ArrivalTimeTolerance_s', crossedTolerances(settingIndex, 2));
        baseResult = planner([], initial, goal, limits, options);
        assertTrue(testCase, baseResult.Success, baseResult.Message);
        assertTrue(testCase, obstacleAvoidance.validateTrajectory(baseResult).Passed);

        seed = struct( ...
            'position_units', [initial.position_units; goal.position_units], ...
            'tau',            [0; 1]);
        request = bmtpEngine.prepareRequest(seed, ...
            struct('regions_units', {cell(0, 1)}, ...
            'coverage', struct('Passed', true, 'ExactRegionCount', 0)), ...
            struct('initialState', baseResult.Inputs.initialState, ...
            'goalState', baseResult.Inputs.goalState, ...
            'limits', baseResult.Diagnostics.Limits, ...
            'options', baseResult.Options));
        preparedMotion = bmtpEngine.motion.createMotion(request, controlPoint_units, ...
            request.MotionHorizon_s + clockOffset_s);
        output = createProvenOutput(baseResult, request, preparedMotion);
        validation = obstacleAvoidance.validateTrajectory(output);

        verifyTrue(testCase, preparedMotion.Success);
        verifyEqual(testCase, preparedMotion.FitsRequestHorizon, expectedAcceptance(settingIndex));
        verifyEqual(testCase, validation.Passed, expectedAcceptance(settingIndex));
    end

    for constraintTolerance = [tightTolerance, looseTolerance]
        options = struct( ...
            'GoalTimeMode',          'fixedArrival', ...
            'ConstraintTolerance',   constraintTolerance, ...
            'ArrivalTimeTolerance_s', tightTolerance);
        result = planner([], initial, goal, limits, options);
        assertTrue(testCase, result.Success, result.Message);
        result.Inputs.goalState.time_s = result.Diagnostics.Polynomial.FinalTime_s + clockOffset_s;
        validation = obstacleAvoidance.validateTrajectory(result);
        verifyFalse(testCase, validation.Passed);
        verifyFalse(testCase, validation.EndpointStatesMatched);

        if constraintTolerance == looseTolerance
            altered = result;
            altered.Diagnostics.Polynomial.FinalTime_s = altered.Diagnostics.Polynomial.FinalTime_s + clockOffset_s;
            validation = obstacleAvoidance.validateTrajectory(altered);
            verifyFalse(testCase, validation.SegmentTimingConsistent);

            altered = result;
            altered.Diagnostics.Polynomial.SegmentStartTime_s(1) = ...
                altered.Diagnostics.Polynomial.SegmentStartTime_s(1) + clockOffset_s;
            validation = obstacleAvoidance.validateTrajectory(altered);
            verifyFalse(testCase, validation.SegmentTimingConsistent);
            verifyFalse(testCase, validation.EndpointStatesMatched);

            altered           = result;
            altered.time_s(1) = altered.time_s(1) + clockOffset_s;
            validation = obstacleAvoidance.validateTrajectory(altered);
            verifyFalse(testCase, validation.SampledHistoriesMatched);

            altered             = result;
            altered.time_s(end) = altered.time_s(end) - clockOffset_s;
            validation = obstacleAvoidance.validateTrajectory(altered);
            verifyFalse(testCase, validation.SampledHistoriesMatched);

            altered = result;
            altered.Inputs.initialState.time_s = ...
                altered.Inputs.initialState.time_s + clockOffset_s;
            validation = obstacleAvoidance.validateTrajectory(altered);
            verifyFalse(testCase, validation.SeparationProofValid);
        end
    end
end

function testDetourAndTampering(testCase)
    options = planner();
    verifyEqual(testCase, options.GoalTimeMode, "fixedArrival");
    verifyFalse(testCase, isfield(options, "Success"));
    obstacle = struct("Name", "center block", ...
        "Vertices_units", [-1 -1; 1 -1; 1 1; -1 1], ...
        "SafetyMargin_units", 0.25);
    initial = struct("time_s", 0, "position_units", [-4 0]);
    goal = struct("time_s", 12, "position_units", [4 0]);
    limits = struct("xInterval_units", [-180 180], "yInterval_units", [-90 90], ...
        "maxVelocity_units_s", [2 2], "maxAcceleration_units_s2", [2 2], ...
        "maxJerk_units_s3", [4 4]);
    r = planner(obstacle, initial, goal, limits, options);
    verifyTrue(testCase,r.Success,r.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
    verifyEqual(testCase,r.Diagnostics.VisibilityGraph.SearchKind,"initialSpatialSnapshot");
    verifyTrue(testCase,r.Diagnostics.VisibilityGraph.GraphIsFullyEnumerated);
    altered = r; altered.position_units(2,1) = altered.position_units(2,1)+0.1;
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(altered).Passed);
    altered = r; altered.Diagnostics.Polynomial.jerkPower_units_s3(1,1,1) = 1e4;
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(altered).Passed);
    altered = r; altered.Diagnostics.SeparationProof.Regions_units = {};
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(altered).Passed);
end

function output = createProvenOutput(baseResult, request, preparedMotion)
    % Build an adversarial validator fixture without stale planner decisions.
    roundoffReserve_units = baseResult.Diagnostics.SeparationProof.RoundoffReserve_units;
    target_units = baseResult.Diagnostics.SeparationProof.RequiredGap_units - roundoffReserve_units;
    motionOutput = bmtpEngine.createMotionOutput(struct(), request, preparedMotion);
    output = baseResult;
    output.time_s                = motionOutput.time_s;
    output.position_units        = motionOutput.position_units;
    output.velocity_units_s      = motionOutput.velocity_units_s;
    output.acceleration_units_s2 = motionOutput.acceleration_units_s2;
    output.jerk_units_s3         = motionOutput.jerk_units_s3;
    output.ArrivalTime_s         = motionOutput.ArrivalTime_s;
    output.MotionLength_units    = motionOutput.MotionLength_units;
    output.Diagnostics.Polynomial                      = motionOutput.Polynomial;
    output.Diagnostics.TrajectoryDuration_s            = motionOutput.TrajectoryDuration_s;
    output.Diagnostics.IntegratedSquaredJerk_units2_s5 = motionOutput.IntegratedSquaredJerk_units2_s5;
    output.Diagnostics.MaximumConstraintViolation      = motionOutput.MaximumConstraintViolation;
    output.Diagnostics.SeparationProof = bmtpEngine.validation.checkFinalMotion( ...
        request, preparedMotion, roundoffReserve_units, target_units);
    output.Diagnostics.Validation = struct("Passed", false, ...
        "Message", "Synthetic motion has not been validated.");
end
