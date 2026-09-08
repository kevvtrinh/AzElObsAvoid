function tests = testStaticClockCorridor
%% Section 0: Header & Readme
% SYNTAX: tests = testStaticClockCorridor
% PURPOSE: Check exact source corridors and conditional physical clocks.
% INPUTS: None.
% OUTPUTS: MATLAB regressions with fresh public trajectory validation.
% UNITS: Coordinate units and seconds.
tests = functiontests(localfunctions);
end

function setupOnce(~)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root, fullfile(root, 'trajectory'));
end

function testClockDetourAcrossAxesAndTimeOrigins(testCase)
    for swap = [false true]
        for origin = [0 7]
            [obstacle, geometry, initial, goal, limits, options, seed] = request(swap, origin);
            [motion, details] = bmtpEngine.solveClockCorridor(seed, geometry, initial, goal, limits, options);
            verifyTrue(testCase, motion.Success, motion.Message);
            verifyTrue(testCase, details.Attempted);
            check = obstacleAvoidance.validateTrajectory(motion, obstacle, initial, goal, limits, options);
            verifyTrue(testCase, check.Passed, check.Message);
            verifyEqual(testCase, motion.SeedIndex, 4);
            direct = bmtpEngine.createDirectMotion(initial, goal, limits, options);
            verifyLessThanOrEqual(testCase, motion.TrajectoryDuration_s, direct.TrajectoryDuration_s + options.ArrivalTimeTolerance_s);
            verifyEqual(testCase, motion.PlaneCertificate.VerifiedPairCount, motion.PlaneCertificate.AllPairCount);
        end
    end
end

function testTiesNonzeroStatesAndNonmonotoneGuidesRemainExplicit(testCase)
    [~, geometry, initial, goal, limits, options, seed] = request(false, 0);
    changedInitial = initial; changedInitial.velocity_units_s = [.1 0];
    [motion, details] = bmtpEngine.solveClockCorridor(seed, geometry, changedInitial, goal, limits, options);
    verifyFalse(testCase, motion.Success);
    verifyFalse(testCase, details.Attempted);
    seed.position_units(3, 1) = -2;
    motion = bmtpEngine.solveClockCorridor(seed, geometry, initial, goal, limits, options);
    verifyEqual(testCase, motion.TerminationReason, "nonMonotoneClockGuide");
    initial.position_units = [-5 -5]; goal.position_units = [5 5];
    limits.maxVelocity_units_s = [2 2]; limits.maxAcceleration_units_s2 = [1 1]; limits.maxJerk_units_s3 = [2 2];
    motion = bmtpEngine.solveClockCorridor(seed, geometry, initial, goal, limits, options);
    verifyEqual(testCase, motion.TerminationReason, "tiedPhysicalClocks");
end

function testBlockedGuideIsRejectedBeforeConicOptimization(testCase)
    [~, geometry, initial, goal, limits, options, seed] = request(false, 0);
    seed.position_units = [initial.position_units; goal.position_units]; seed.tau = [0; 1];
    [motion, details] = bmtpEngine.solveClockCorridor(seed, geometry, initial, goal, limits, options);
    verifyFalse(testCase, motion.Success);
    verifyEqual(testCase, motion.TerminationReason, "guideIntersectsSourceInterior");
    verifyEqual(testCase, details.ConicSolver.CallCount, 0);
end

function testPrescribedArrivalKeepsTheGeneralFormulation(testCase)
    [obstacle, geometry, initial, goal, limits, options, seed] = request(true, 7);
    options.GoalTimeMode = "fixedArrival";
    [motion, details] = bmtpEngine.solveClockCorridor(seed, geometry, initial, goal, limits, options);
    verifyFalse(testCase, motion.Success);
    verifyFalse(testCase, details.Attempted);
    verifyEqual(testCase, details.TerminationReason, "prescribedArrivalUsesGeneralSolver");
    [motion, ~] = obstacleAvoidance.planner.solveStaticBmtpTrajectory(seed, geometry, initial, goal, limits, options);
    check = obstacleAvoidance.validateTrajectory(motion, obstacle, initial, goal, limits, options);
    verifyTrue(testCase, motion.Success, motion.Message);
    verifyTrue(testCase, check.Passed, check.Message);
    verifyEqual(testCase, motion.ArrivalTime_s, goal.time_s, 'AbsTol', options.ConstraintTolerance);
end

function testFreeAxisReachabilityRejectsAnEarlyImpossibleTurn(testCase)
    [~, ~, initial, goal, limits, options, seed] = request(false, 7);
    vertices = [-4.9 -.5; -4.8 -.5; -4.8 .5; -4.9 .5];
    obstacle = obstacleAvoidance.obstacles.createObstacle("early turn", 7, vertices(:, 1), vertices(:, 2), 0);
    prepared = obstacleAvoidance.obstacles.prepareObstacles(obstacle);
    geometry = obstacleAvoidance.planner.prepareStaticSolverGeometry(prepared, 7, 19);
    seed.position_units = [-5 0; -4.95 1; -4.7 1; 5 0];
    [motion, details] = bmtpEngine.solveClockCorridor(seed, geometry, initial, goal, limits, options);
    verifyFalse(testCase, motion.Success);
    verifyEqual(testCase, motion.TerminationReason, "freeAxisReachabilityBound");
    verifyEqual(testCase, details.ConicSolver.CallCount, 0);
end

function [obstacle, geometry, initial, goal, limits, options, seed] = request(swap, origin)
    initial = struct('time_s', origin, 'position_units', [-5 0], 'velocity_units_s', [0 0], 'acceleration_units_s2', [0 0]);
    goal = initial; goal.time_s = origin + 12; goal.position_units = [5 0];
    limits = struct('maxVelocity_units_s', [2 3], 'maxAcceleration_units_s2', [1 2], 'maxJerk_units_s3', [2 4], 'xInterval_units', [-20 20], 'yInterval_units', [-20 20]);
    options = obstacleAvoidance.input.resolvePlannerOptions();
    vertices = [-.2 -.3; .2 -.3; .2 .3; -.2 .3];
    route = [-5 0; -1 1; 1 1; 5 0];
    if swap
        vertices = fliplr(vertices); route = fliplr(route);
        initial.position_units = fliplr(initial.position_units); goal.position_units = fliplr(goal.position_units);
        limits.maxVelocity_units_s = fliplr(limits.maxVelocity_units_s);
        limits.maxAcceleration_units_s2 = fliplr(limits.maxAcceleration_units_s2);
        limits.maxJerk_units_s3 = fliplr(limits.maxJerk_units_s3);
    end
    obstacle = obstacleAvoidance.obstacles.createObstacle("clock detour", origin, vertices(:, 1), vertices(:, 2), 0);
    prepared = obstacleAvoidance.obstacles.prepareObstacles(obstacle);
    geometry = obstacleAvoidance.planner.prepareStaticSolverGeometry(prepared, origin, origin + 12);
    seed = obstacleAvoidance.search.createEmptyPathGuess();
    seed.Index = 4; seed.Source = "clockTest"; seed.position_units = route;
    seed.tau = [0; .4; .6; 1]; seed.Length_units = sum(vecnorm(diff(route), 2, 2));
end
