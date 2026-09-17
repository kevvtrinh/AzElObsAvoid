function tests = testRandomMovingTargetInterceptions
%% Section 0: Header & Readme
% SYNTAX
%   results = runtests('tests/testRandomMovingTargetInterceptions.m')
%**************************************************************************
% PURPOSE
%   - Exercise many reproducible moving-goal interceptions against static
%     target zones and independently translating obstacles.
%   - Independently validate every successful motion and stable expected
%     failure without changing the caller's global random state.
%**************************************************************************
% INPUTS
%   - MATLAB function-based test framework.
%**************************************************************************
% OUTPUTS
%   - Determinism, occupancy, interception, and validation test results.
%**************************************************************************
% UNITS
%   - Positions are coordinate units and time is seconds.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root, fullfile(root, 'examples'));
    testCase.TestData.CaseCount  = 12;
    testCase.TestData.RandomSeed = 941207;
end

function testGeneratorIsDeterministicAndLocallyRandom(testCase)
    previousRandomState = rng;
    caseCount           = 40;
    staticStarts_units  = zeros(caseCount, 2);
    movingStarts_units  = zeros(caseCount, 2);
    for caseIndex = 1:caseCount
        staticScenario = createRandomInterceptionScenario( ...
            caseIndex, 'staticZone', testCase.TestData.RandomSeed);
        movingScenario = createRandomInterceptionScenario( ...
            caseIndex, 'movingObstacles', testCase.TestData.RandomSeed);
        verifyEqual(testCase, staticScenario, createRandomInterceptionScenario( ...
            caseIndex, 'staticZone', testCase.TestData.RandomSeed));
        verifyEqual(testCase, movingScenario, createRandomInterceptionScenario( ...
            caseIndex, 'movingObstacles', testCase.TestData.RandomSeed));
        staticStarts_units(caseIndex, :) = staticScenario.InitialState.position_units;
        movingStarts_units(caseIndex, :) = movingScenario.InitialState.position_units;
    end
    verifyEqual(testCase, rng, previousRandomState);
    verifySize(testCase, unique(staticStarts_units, 'rows'), [caseCount, 2]);
    verifySize(testCase, unique(movingStarts_units, 'rows'), [caseCount, 2]);
    alternateSeedScenario = createRandomInterceptionScenario( ...
        17, 'staticZone', testCase.TestData.RandomSeed + 1);
    verifyNotEqual(testCase, alternateSeedScenario.InitialState.position_units, ...
        staticStarts_units(17, :));
    verifyEqual(testCase, rng, previousRandomState);
end

function testStaticZonesWithMovingGoal(testCase)
    for caseIndex = 1:testCase.TestData.CaseCount
        scenario = createRandomInterceptionScenario( ...
            caseIndex, 'staticZone', testCase.TestData.RandomSeed);
        label = "static-zone case " + caseIndex;
        verifyTargetOccupancyPattern(testCase, scenario, label);
        result = verifyPlannerOutcome(testCase, scenario, label);
        if caseIndex == 8
            verifyGreaterThan(testCase,numel(result.Attempts),1);
            verifyEqual(testCase,result.Attempts(1).FailureStage,"timing");
            verifyEqual(testCase,result.Attempts(1).FailureKind, ...
                "kinematicCertificateUnavailable");
            verifyTrue(testCase,result.Attempts(1).MethodFallbackEligible);
            verifyTrue(testCase,any([result.Attempts.Selected]));
        end
    end
end

function testMovingObstaclesWithMovingGoal(testCase)
    for caseIndex = 1:testCase.TestData.CaseCount
        scenario = createRandomInterceptionScenario( ...
            caseIndex, 'movingObstacles', testCase.TestData.RandomSeed);
        label = "moving-obstacle case " + caseIndex;
        verifyMovingGeometry(testCase, scenario, label);
        verifyTargetOccupancyPattern(testCase, scenario, label);
        if scenario.DirectRouteShouldBeBlocked
            verifyTrue(testCase, directRouteIsBlocked(scenario), ...
                label + " did not block its timed direct route.");
        end
        verifyPlannerOutcome(testCase, scenario, label);
    end
end

function testFixedMovingTargetTimedFallbackMatchesTargetDerivatives(testCase)
    % Pin the physical failure that motivated the core eligibility change.
    % The spatial seed is unavailable; the one exact timed route waits for
    % the crossing obstacle and must retain both matched target derivatives.
    scenario = timedFallbackRegressionScenario();
    label    = "fixed moving-target timed fallback";
    verifyMovingGeometry(testCase, scenario, label);
    verifyTargetOccupancyPattern(testCase, scenario, label);
    verifyTrue(testCase, directRouteIsBlocked(scenario), ...
        label + ": timed direct route is unexpectedly clear.");
    result = verifyPlannerOutcome(testCase, scenario, label);

    verifyEqual(testCase, result.VisibilityGraph.SearchKind, ...
        "timeExpandedVisibilityGraph", ...
        label + ": timed route was not selected.");
    verifyGreaterThanOrEqual(testCase, numel(result.Attempts), 2, ...
        label + ": rejected spatial-guide evidence is missing.");
    verifyTrue(testCase, any([result.Attempts(1:end - 1).FallbackEligible]), ...
        label + ": no rejected guide admitted the timed fallback.");
    verifyEqual(testCase, result.Attempts(end).Kind, "timedVisibility");
    verifyEqual(testCase, result.Attempts(end).Outcome, "accepted");
    [~, targetVelocity_units_s, targetAcceleration_units_s2] = ...
        obstacleAvoidance.input.targetPositionAtTime( ...
        scenario.GoalState.targetMotion, scenario.GoalState.time_s);
    verifyEqual(testCase, result.Intercept.TargetVelocity_units_s, ...
        targetVelocity_units_s, 'AbsTol', 1e-9);
    verifyEqual(testCase, result.Intercept.TargetAcceleration_units_s2, ...
        targetAcceleration_units_s2, 'AbsTol', 1e-9);
    verifyEqual(testCase, result.velocity_units_s(end, :), ...
        targetVelocity_units_s, 'AbsTol', 1e-8);
    verifyEqual(testCase, result.acceleration_units_s2(end, :), ...
        targetAcceleration_units_s2, 'AbsTol', 1e-8);
end

function testRejectsInvalidGeneratorInputs(testCase)
    verifyError(testCase, @()createRandomInterceptionScenario(0, 'staticZone'), ...
        'MATLAB:expectedPositive');
    verifyError(testCase, @()createRandomInterceptionScenario(1, 'unknown'), ...
        'createRandomInterceptionScenario:InvalidFamily');
end

function result = verifyPlannerOutcome(testCase, scenario, label)
    % Check the public stable outcome and independently certify every success.
    result = planner(scenario.Obstacles, scenario.InitialState, ...
        scenario.GoalState, scenario.Limits, scenario.Options);
    verifyEqual(testCase, result.Success, scenario.ExpectedSuccess, ...
        label + ": " + result.Message);
    verifyEqual(testCase, string(result.TerminationReason), ...
        scenario.ExpectedTerminationReason, label + ": unexpected termination.");
    if result.Success ~= scenario.ExpectedSuccess
        return
    end
    if ~scenario.ExpectedSuccess
        verifyEmpty(testCase, result.time_s, label + ": failure returned a motion.");
        return
    end

    validation = obstacleAvoidance.validateTrajectory(result);
    verifyTrue(testCase, validation.Passed, label + ": " + validation.Message);
    expectedTarget_units = obstacleAvoidance.input.targetPositionAtTime( ...
        scenario.GoalState.targetMotion, result.Intercept.Time_s);
    verifyEqual(testCase, result.Intercept.TargetPosition_units, ...
        expectedTarget_units, 'AbsTol', 1e-8, ...
        label + ": intercept did not match the target.");
    targetIsOccupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
        scenario.Obstacles, expectedTarget_units(1), expectedTarget_units(2), ...
        result.Intercept.Time_s);
    verifyFalse(testCase, targetIsOccupied, ...
        label + ": successful intercept remained inside an obstacle.");
    verifyGreaterThan(testCase, norm(diff( ...
        scenario.GoalState.targetMotion.position_units, 1, 1)), 0.5, ...
        label + ": target did not move materially.");

    if string(scenario.Options.GoalTimeMode) == "earliestArrival"
        verifyLessThan(testCase, result.Intercept.Time_s, scenario.GoalState.time_s, ...
            label + ": earliest search only returned the horizon.");
        verifyTrue(testCase, isfield(result, 'TemporalSearch'), ...
            label + ": chronological search diagnostics are missing.");
    else
        verifyEqual(testCase, result.Intercept.Time_s, scenario.GoalState.time_s, ...
            'AbsTol', 1e-8, label + ": fixed intercept time changed.");
    end
end

function verifyTargetOccupancyPattern(testCase, scenario, label)
    % Probe the target independently over its complete declared history.
    sampleCount  = 401;
    sampleTime_s = linspace(0, scenario.GoalState.time_s, sampleCount).';
    target_units = obstacleAvoidance.input.targetPositionAtTime( ...
        scenario.GoalState.targetMotion, sampleTime_s);
    occupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
        scenario.Obstacles, target_units(:, 1), target_units(:, 2), sampleTime_s);
    transitionCount = nnz(diff(occupied) ~= 0);

    switch scenario.TargetOccupancyPattern
        case "outsideInsideOutside"
            verifyFalse(testCase, occupied(1), label + ": target started occupied.");
            verifyTrue(testCase, any(occupied), label + ": target never entered the zone.");
            verifyFalse(testCase, occupied(end), label + ": target did not leave the zone.");
            verifyGreaterThanOrEqual(testCase, transitionCount, 2, ...
                label + ": expected entry and exit transitions.");
        case "insideOutside"
            verifyTrue(testCase, occupied(1), label + ": target did not start occupied.");
            verifyFalse(testCase, occupied(end), label + ": target did not exit.");
            verifyGreaterThanOrEqual(testCase, transitionCount, 1, ...
                label + ": expected one exit transition.");
        case "outsideInside"
            verifyFalse(testCase, occupied(1), label + ": target started occupied.");
            verifyTrue(testCase, occupied(end), label + ": target did not finish occupied.");
            verifyGreaterThanOrEqual(testCase, transitionCount, 1, ...
                label + ": expected one entry transition.");
        case "outsideOutside"
            verifyFalse(testCase, occupied(1), label + ": target started occupied.");
            verifyFalse(testCase, occupied(end), label + ": target ended occupied.");
        otherwise
            verifyFail(testCase, label + ": unknown occupancy expectation.");
    end
end

function verifyMovingGeometry(testCase, scenario, label)
    % Every obstacle in this family must translate between its endpoint slices.
    for obstacleIndex = 1:numel(scenario.Obstacles)
        obstacle = scenario.Obstacles(obstacleIndex);
        verifyGreaterThan(testCase, numel(obstacle.time_s), 1, ...
            label + ": obstacle history is static.");
        start_units = [obstacle.x_units{1}, obstacle.y_units{1}];
        end_units   = [obstacle.x_units{end}, obstacle.y_units{end}];
        verifyGreaterThan(testCase, max(abs(end_units - start_units), [], 'all'), ...
            0.5, label + ": obstacle did not move materially.");
    end
end

function blocked = directRouteIsBlocked(scenario)
    % Sample only to assert fixture construction; planner acceptance remains exact.
    sampleCount  = 401;
    sampleTime_s = linspace(0, scenario.GoalState.time_s, sampleCount).';
    goal_units   = obstacleAvoidance.input.targetPositionAtTime( ...
        scenario.GoalState.targetMotion, scenario.GoalState.time_s);
    fraction     = sampleTime_s / scenario.GoalState.time_s;
    route_units  = scenario.InitialState.position_units + fraction .* ...
        (goal_units - scenario.InitialState.position_units);
    occupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
        scenario.Obstacles, route_units(:, 1), route_units(:, 2), sampleTime_s);
    blocked = any(occupied);
end

function scenario = timedFallbackRegressionScenario()
    % Freeze one general moving-target/moving-obstacle factory regression.
    missionEndTime_s = 12.6824677531027;
    startVertices_units = [ ...
        -2.41212665955762, -3.10756783587549; ...
        -1.26869088471832, -3.85354373008669; ...
        -0.11574128448923, -2.08629651778552; ...
        -1.25917705932853, -1.34032062357432];
    endVertices_units = [ ...
        0.451174172683163, 1.28131474212419; ...
        1.59460994752247,  0.535338847912989; ...
        2.74755954775155,  2.30258606021416; ...
        1.60412377291225,  3.04856195442535];
    obstacle = obstacleAvoidance.obstacles.createObstacle( ...
        'Crossing rectangle', [0; missionEndTime_s], ...
        {startVertices_units(:, 1); endVertices_units(:, 1)}, ...
        {startVertices_units(:, 2); endVertices_units(:, 2)}, ...
        0.06, struct('vertexCorrespondence', 'sourceIndex'));

    initialState = struct( ...
        'time_s',                0, ...
        'position_units',        [-5.67556230729695, 3.70483299936472], ...
        'velocity_units_s',      [0, 0], ...
        'acceleration_units_s2', [0, 0]);
    targetStart_units = [3.56626840907745, -2.18951624292289];
    targetEnd_units   = [5.47468754926656, -3.74372919576700];
    targetDisplacement_units = targetEnd_units - targetStart_units;
    targetMotion = struct( ...
        'time_s', [0; missionEndTime_s / 2; missionEndTime_s], ...
        'position_units', [targetStart_units; ...
            targetStart_units + 0.4 * targetDisplacement_units; ...
            targetEnd_units], ...
        'InterpolationMethod', 'pchip');
    goalState = struct('time_s', missionEndTime_s, 'targetMotion', targetMotion);
    limits = struct( ...
        'xInterval_units',          [-20, 20], ...
        'yInterval_units',          [-20, 20], ...
        'maxVelocity_units_s',      [2.5, 2.5], ...
        'maxAcceleration_units_s2', [1.2, 1.2], ...
        'maxJerk_units_s3',         [2.5, 2.5]);
    options = struct( ...
        'GoalTimeMode',          'fixedArrival', ...
        'SampleTime_s',          0.1, ...
        'TemporalResolution_s',  1, ...
        'MaxArrivalTrials',      16, ...
        'MatchTargetVelocity',   true, ...
        'MatchTargetAcceleration', true);
    scenario = struct( ...
        'Obstacles',                  obstacle, ...
        'InitialState',               initialState, ...
        'GoalState',                  goalState, ...
        'Limits',                     limits, ...
        'Options',                    options, ...
        'ExpectedSuccess',            true, ...
        'ExpectedTerminationReason',  "goalReached", ...
        'TargetOccupancyPattern',     "outsideOutside", ...
        'DirectRouteShouldBeBlocked', true);
end
