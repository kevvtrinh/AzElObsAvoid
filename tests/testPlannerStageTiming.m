function tests = testPlannerStageTiming
%% Section 0: Header & Readme
% Verify exclusive stage timing for the maintained obstacle planner.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    % Create common planner inputs for timing checks. Named stages must be
    % nonnegative, exclusive, and consistent with total elapsed time. An over-count
    % points to nested timers. Missing time points to an unrecorded branch.
    % Add production and maintained example entry points.
    repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
    addpath(repositoryRoot);
    addpath(fullfile(repositoryRoot, "trajectory"));
    addpath(fullfile(repositoryRoot, "examples"));
    testCase.TestData.RepositoryRoot = repositoryRoot;
end

function testValidatorSeparatesCollisionActivity(testCase)
    % Keep nested collision work a subset rather than a second additive stage.
    initialState = state(0, [-1 0]);
    initialState.velocity_deg_s = [2 0];
    goalState = state(1, [1 0]);
    goalState.velocity_deg_s = [2 0];
    limits = physicalLimits();
    limits.maxVelocity_deg_s = [3 3];
    trajectory = linearTrajectory(initialState, goalState);
    obstacle   = rectangleObstacle([0 1], [-0.1 0.1 -1 1], 0);
        validation = obstacleAvoidance.validateTrajectory(trajectory, obstacle, initialState, goalState, limits, fixedTimeOptions());

    verifyFalse(testCase, validation.Passed);
    verifyGreaterThanOrEqual(testCase, validation.CollisionCheckingElapsedTime_s, 0);
    verifyLessThanOrEqual(testCase, validation.CollisionCheckingElapsedTime_s, validation.ElapsedTime_s + timingTolerance(validation.ElapsedTime_s));
end

function testSuccessAndEndpointFailureShareTiming(testCase)
    % Require timing on a solved request and critical early exit.
    initialState = state(0, [0 0]);
    goalState    = state(4, [1 0]);
    limits       = physicalLimits();
    options      = fixedTimeOptions();
    [success, successDiagnosis] = obstacleAvoidance.planTrajectory([], initialState, goalState, limits, options);
    blockingObstacle = rectangleObstacle([0 4], [-1 1 -1 1], 0);
    [failure, failureDiagnosis] = obstacleAvoidance.planTrajectory(blockingObstacle, initialState, goalState, limits, options);

    verifyTrue(testCase, success.Success, success.Message);
    verifyTrue(testCase, success.Validation.Passed, success.Validation.Message);
    verifyFalse(testCase, isfield(success, "SelectedMotionSource"));
    verifyFalse(testCase, failure.Success);
    verifyEqual(testCase, failure.TerminationReason, "endpointBlocked");
    verifyEqual(testCase, fieldnames(successDiagnosis.Timing), fieldnames(failureDiagnosis.Timing));
    verifyStageTiming(testCase, success, successDiagnosis);
    verifyStageTiming(testCase, failure, failureDiagnosis);
    verifyEqual(testCase, failureDiagnosis.Timing.RouteSearchElapsedTime_s, 0);
end

function testMotionSolverWorkReconcilesTiming(testCase)
    % Keep selected-engine solver and validation work visible in exclusive stages.
    initialState = state(0, [0 0]);
    goalState    = state(3, [1 0]);
    limits       = physicalLimits();
    options      = fixedTimeOptions();
    farObstacle  = rectangleObstacle([0 3], [-100 -90 70 80], 0);
    [result, resultDiagnosis] = obstacleAvoidance.planTrajectory(farObstacle, initialState, goalState, limits, options);

    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
    verifyFalse(testCase, isfield(result, "SelectedMotionSource"));
    summary = resultDiagnosis.Attempts(resultDiagnosis.SelectedAttemptIndex);
    details = testSupport.solverDetails(resultDiagnosis, resultDiagnosis.SelectedAttemptIndex);
    verifyTrue(testCase, any(details.Field == "ElapsedTime_s"));
    verifyGreaterThan(testCase, summary.SeedPlanningElapsedTime_s, 0);
    verifyGreaterThan(testCase, resultDiagnosis.Timing.MotionSolvingElapsedTime_s, 0);
    verifyStageTiming(testCase, result, resultDiagnosis);
end

function testFinalizerRejectsOverAttribution(testCase)
    % Reject overlapping stage ownership instead of hiding it in a zero residual.
    timing = obstacleAvoidance.planner.createStageTiming();
    timing.MotionSolvingElapsedTime_s = 2;

    verifyError(testCase, @() obstacleAvoidance.planner.reconcileStageTiming(timing, 1), "stageTiming:OverAttributed");
end

function verifyStageTiming(testCase, result, resultDiagnosis)
    % Verify exact shared field ownership and top-level reconciliation.
    requiredNames = [ ...
        "RouteSearchElapsedTime_s"; ...
        "MotionSolvingElapsedTime_s"; ...
        "CollisionCheckingElapsedTime_s"; ...
        "FinalValidationElapsedTime_s"; ...
        "UnattributedElapsedTime_s"; "TotalElapsedTime_s"];
    timing = resultDiagnosis.Timing;
    verifyEqual(testCase, string(fieldnames(timing)), requiredNames);
    verifyAdditiveTiming(testCase, timing);
    verifyEqual(testCase, timing.TotalElapsedTime_s, result.ElapsedPlanningTime_s, "AbsTol", timingTolerance(timing.TotalElapsedTime_s));
end

function verifyAdditiveTiming(testCase, timing)
    % Verify finite nonnegative exclusive fields, residual, and total.
    values = struct2array(timing);

    for value = values
        verifyTrue(testCase, isnumeric(value) && isreal(value) && isscalar(value));
        verifyTrue(testCase, isfinite(value));
        verifyGreaterThanOrEqual(testCase, value, 0);
    end
    exclusiveElapsedTime_s    = sum(values(1:end - 2));
    unattributedElapsedTime_s = values(end - 1);
    totalElapsedTime_s        = values(end);
    tolerance_s               = timingTolerance(totalElapsedTime_s);
    verifyLessThanOrEqual(testCase, exclusiveElapsedTime_s, totalElapsedTime_s + tolerance_s);
    verifyEqual(testCase, unattributedElapsedTime_s, max(0, totalElapsedTime_s - exclusiveElapsedTime_s), "AbsTol", tolerance_s);
    verifyEqual(testCase, exclusiveElapsedTime_s + unattributedElapsedTime_s, totalElapsedTime_s, "AbsTol", tolerance_s);
end

function tolerance_s = timingTolerance(totalElapsedTime_s)
    % Use a clock-accounting tolerance, not a performance threshold.
    tolerance_s = max(1e-6, 64 * eps(max(1, totalElapsedTime_s)));
end

function options = fixedTimeOptions()
    % Return deterministic fixed-time controls for timing tests.
    options = obstacleAvoidance.planTrajectory();
    options.GoalTimeMode     = "fixedArrival";
    options.MaximumSeedCount = 1;
    options.SampleTime_s     = 0.05;
end

function value = state(time_s, position_deg)
    % Construct one stationary-derivative endpoint.
    value = struct("time_s", time_s, "position_deg", position_deg, ...
        "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
end

function limits = physicalLimits()
    % Construct shared two-axis workspace and derivative limits.
    limits = struct();
    limits.maxVelocity_deg_s      = [2 2];
    limits.maxAcceleration_deg_s2 = [1 1];
    limits.maxJerk_deg_s3         = [2 2];
    limits.azimuthInterval_deg    = [-180 180];
    limits.elevationInterval_deg  = [-90 90];
end

function obstacle = rectangleObstacle(time_s, bounds_deg, margin_deg)
    % Construct a static rectangle from [minAz maxAz minEl maxEl].
    azimuth_deg   = bounds_deg([1 2 2 1]).';
    elevation_deg = bounds_deg([3 3 4 4]).';
    obstacle      = obstacleAvoidance.obstacles.createObstacle("rectangle", time_s(:), azimuth_deg, elevation_deg, margin_deg);
end

function trajectory = linearTrajectory(initialState, goalState)
    % Build one exact constant-velocity polynomial sampled at its endpoints.
    duration_s        = goalState.time_s - initialState.time_s;
    positionPower_deg = zeros(1, 2, 6);
    positionPower_deg(1, :, 1) = initialState.position_deg;
    positionPower_deg(1, :, 2) = goalState.position_deg - initialState.position_deg;
    velocityPower_deg_s = zeros(1, 2, 5);
    velocityPower_deg_s(1, :, 1) = initialState.velocity_deg_s;
    polynomial = struct("SegmentCount", 1, ...
        "SegmentStartTime_s", initialState.time_s, ...
        "SegmentDuration_s", duration_s, ...
        "FinalTime_s", goalState.time_s, ...
        "positionPower_deg", positionPower_deg, ...
        "velocityPower_deg_s", velocityPower_deg_s, ...
        "accelerationPower_deg_s2", zeros(1, 2, 4), ...
        "jerkPower_deg_s3", zeros(1, 2, 3), ...
        "TerminalState", struct());
    trajectory = struct("time_s", [initialState.time_s; goalState.time_s], ...
        "position_deg", [initialState.position_deg; goalState.position_deg], ...
        "velocity_deg_s", ...
        [initialState.velocity_deg_s; goalState.velocity_deg_s], ...
        "acceleration_deg_s2", zeros(2, 2), ...
        "jerk_deg_s3", zeros(2, 2), "Polynomial", polynomial);
end

function testFixedClockTimingPreservesFirstAcceptanceAndExclusiveWork(testCase)
    % A detour at the physical time floor can be the sole validated construction.
    for vertices = {[-1 -1;1 -1;1 1;-1 1], ...
            [-1.5 -0.5;0 -1.3;1.5 0;0.3 1.7;-1 0.6]}
        initial = state(0, [-8 0]);
        goal    = state(20, [8 0]);
        limits  = struct();
        limits.maxVelocity_deg_s      = [3 3];
        limits.maxAcceleration_deg_s2 = [2 2];
        limits.maxJerk_deg_s3         = [4 4];
        limits.azimuthInterval_deg    = [-10 10];
        limits.elevationInterval_deg  = [-6 6];
        options = obstacleAvoidance.planTrajectory();
        options.MaximumSeedCount = 1;
        options.GoalTimeMode     = "fixedArrival";
        [~, initial, goal, limits] = obstacleAvoidance.input.normalizePlannerRequest([], initial, goal, limits, options);
        direct = bmtpEngine.createDirectMotion(initial, goal, limits, options);
        goal.time_s = max(direct.MinimumAxisDuration_s);
        polygon  = vertices{1};
        obstacle = obstacleAvoidance.obstacles.createObstacle('timing detour', 0, polygon(:,1), polygon(:,2), 0.1);
        for mode = ["fixedArrival", "earliestArrival"]
            options.GoalTimeMode = mode;
            [result, diagnosis] = obstacleAvoidance.planTrajectory(obstacle, initial, goal, limits, options);
            verifyTrue(testCase, result.Success, result.Message);
            validation = obstacleAvoidance.validateTrajectory(result);
            verifyTrue(testCase, validation.Passed, validation.Message);
            isDetour = [diagnosis.Attempts.SeedSource] == "fixedClockLateralExcursion";
            verifyTrue(testCase, any(isDetour & [diagnosis.Attempts.ValidationPassed]));
            verifyTrue(testCase, isfinite(diagnosis.FirstValidatedMotionTime_s));
            verifyGreaterThan(testCase, diagnosis.FirstValidatedMotionTime_s, 0);
            verifyLessThanOrEqual(testCase, diagnosis.FirstValidatedMotionTime_s, result.ElapsedPlanningTime_s);
            if mode == "fixedArrival"
                verifyFalse(testCase, any([diagnosis.Attempts(~isDetour).ValidationPassed]));
            end
            directTime_s   = testSupport.diagnosisValue(diagnosis.DirectMotion, "ElapsedTime_s");
            recordedTime_s = directTime_s + sum([diagnosis.Attempts.SeedPlanningElapsedTime_s]);
            verifyEqual(testCase, recordedTime_s, diagnosis.Timing.MotionSolvingElapsedTime_s, "AbsTol", timingTolerance(result.ElapsedPlanningTime_s));
            verifyStageTiming(testCase, result, diagnosis);
        end
    end
end
