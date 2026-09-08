function tests = testClockGuide
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testClockGuide.m')
% PURPOSE: Check general kinematic-bound detours, clock shifts, and axis symmetry.
% INPUTS: MATLAB unit test framework and unchanged historical examples.
% OUTPUTS: Function-based tests.
% UNITS: Coordinate units, seconds, and physical derivatives.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'),fullfile(root,'examples'));
    testCase.TestData.Circle = exampleMovingCircleNoWrap(struct('PlotOutputs',false,'Verbose',false));
end

function testMovingCircleAtKinematicBound(testCase)
    r = testCase.TestData.Circle;
    verifyTrue(testCase,r.Success,r.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
    verifyTrue(testCase,r.SolverDiagnostics.LowerBoundAttempt.Passed);
    verifyEqual(testCase,r.ArrivalTime_s,8.5,'AbsTol',1e-8);
    verifyLessThanOrEqual(testCase,r.MotionLength_units,12.4537884589941);
    verifyEqual(testCase,r.SolverDiagnostics.ConicSolver.CallCount,r.SolverDiagnostics.TrajectorySocpCount);
end

function testStaticAndRotatingDetourQuality(testCase)
    names = {'exampleDenseConcaveObstacle','exampleAlternatingSlalom','exampleMovingRotatingObstacleField'};
    time_s = [8.5,10.550093893364,9.04166666666667];
    length_units = [12.7611045181781,16.0347535809872,20.716208785];
    for k = 1:numel(names)
        r = feval(names{k},struct('PlotOutputs',false,'Verbose',false));
        verifyTrue(testCase,r.Success,r.Message);
        verifyTrue(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
        verifyTrue(testCase,r.SolverDiagnostics.LowerBoundAttempt.Passed);
        verifyLessThanOrEqual(testCase,r.ArrivalTime_s,time_s(k)+1e-8);
        verifyLessThanOrEqual(testCase,r.MotionLength_units,length_units(k));
    end
end

function testAbsoluteClockShift(testCase)
    r = testCase.TestData.Circle;
    obstacle = r.Inputs.obstacles; obstacle.time_s = obstacle.time_s+7;
    initial = r.Inputs.initialState; initial.time_s = initial.time_s+7;
    goal = r.Inputs.goalState; goal.time_s = goal.time_s+7;
    shifted = planner(obstacle,initial,goal,r.Limits,r.Options);
    verifyTrue(testCase,shifted.Success,shifted.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(shifted).Passed);
    verifyEqual(testCase,shifted.ArrivalTime_s,15.5,'AbsTol',1e-8);
    verifyEqual(testCase,shifted.MotionLength_units,r.MotionLength_units,'AbsTol',1e-6);
end

function testCoordinateExchange(testCase)
    r = testCase.TestData.Circle;
    source = r.Inputs.obstacles;
    obstacle = obstacleAvoidance.obstacles.createObstacle('coordinate exchange',source.time_s, ...
        source.originalY_units,source.originalX_units,0.1);
    initial = r.Inputs.initialState; initial.position_units = fliplr(initial.position_units);
    goal = r.Inputs.goalState; goal.position_units = fliplr(goal.position_units);
    limits = r.Limits;
    limits.xInterval_units = r.Limits.yInterval_units;
    limits.yInterval_units = r.Limits.xInterval_units;
    exchanged = planner(obstacle,initial,goal,limits,r.Options);
    verifyTrue(testCase,exchanged.Success,exchanged.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(exchanged).Passed);
    verifyEqual(testCase,exchanged.ArrivalTime_s,8.5,'AbsTol',1e-8);
    verifyLessThanOrEqual(testCase,exchanged.MotionLength_units,12.4537884589941);
end

function testZeroClockEdgesCannotEscapeCavity(testCase)
    r = exampleStaticUShapedObstacle(struct('PlotOutputs',false,'Verbose',false));
    verifyTrue(testCase,r.Success,r.Message);
    verifyFalse(testCase,r.SolverDiagnostics.LowerBoundAttempt.Passed);
    verifyEqual(testCase,r.SolverDiagnostics.LowerBoundAttempt.TrajectorySocpCount,0);
end
