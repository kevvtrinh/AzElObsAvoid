function tests = testClockGuide
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testClockGuide.m')
% PURPOSE: Check general kinematic-bound detours, clock shifts, and axis symmetry.
% INPUTS: MATLAB unit test framework and maintained examples.
% OUTPUTS: Function-based tests.
% UNITS: Coordinate units, seconds, and physical derivatives.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'),fullfile(root,'examples'));
    testCase.TestData.Circle = exampleMovingCircleNoWrap(struct('PlotOutputs',false,'Verbose',false));
end

function testMovingCircleC3Timing(testCase)
    r = testCase.TestData.Circle;
    verifyTrue(testCase,r.Success,r.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
    verifyEqual(testCase,r.Polynomial.Degree,5);
    verifyGreaterThanOrEqual(testCase,r.ArrivalTime_s,8.5);
    verifyLessThanOrEqual(testCase,r.ArrivalTime_s,r.Inputs.goalState.time_s);
    verifyEqual(testCase,r.SolverDiagnostics.ConicSolver.CallCount,r.SolverDiagnostics.TrajectorySocpCount);
end

function testInitiallyOccupiedGoalCanClearBeforeArrival(testCase)
    vertices = [3.5,-0.5;4.5,-0.5;4.5,0.5;3.5,0.5];
    raised = vertices+[0,6];
    obstacle = obstacleAvoidance.obstacles.createObstacle('departing goal obstacle',[0;1;12], ...
        {vertices(:,1);raised(:,1);raised(:,1)},{vertices(:,2);raised(:,2);raised(:,2)},0.1);
    initial = struct('time_s',0,'position_units',[-4,0]);
    goal = struct('time_s',12,'position_units',[4,0]);
    limits = struct('maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[2,2],'maxJerk_units_s3',[4,4]);
    r = planner(obstacle,initial,goal,limits,struct('GoalTimeMode',"earliestArrival"));
    verifyTrue(testCase,r.Success,r.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
    verifyEqual(testCase,r.VisibilityGraph.SearchKind,"c3DepartureSchedule");
    verifyEmpty(testCase,r.VisibilityGraph.AcceptedNodeIndex);
    verifyTrue(testCase,obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(obstacle,4,0,0));
    verifyFalse(testCase,obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(obstacle,4,0,r.ArrivalTime_s));
end

function testStaticAndMovingDetourQuality(testCase)
    names = {'exampleDenseConcaveObstacle','exampleAlternatingSlalom', ...
        'exampleMovingRotatingObstacleField','exampleMovingObstacle220'};
    % Historical C2 quality gates remain reported by runExampleBenchmarks.
    for k = 1:numel(names)
        r = feval(names{k},struct('PlotOutputs',false,'Verbose',false));
        verifyTrue(testCase,r.Success,r.Message);
        verifyTrue(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
        verifyEqual(testCase,r.Polynomial.Degree,5);
        verifyLessThanOrEqual(testCase,r.ArrivalTime_s,r.Inputs.goalState.time_s);
        verifyTrue(testCase,isfinite(r.MotionLength_units));
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
    verifyEqual(testCase,shifted.ArrivalTime_s,r.ArrivalTime_s+7,'AbsTol',1e-8);
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
    verifyEqual(testCase,exchanged.ArrivalTime_s,r.ArrivalTime_s,'AbsTol',1e-8);
    % The local fixed-time solve can choose a different feasible detour.
    verifyLessThanOrEqual(testCase,exchanged.MotionLength_units, ...
        norm(exchanged.Limits.maxVelocity_units_s)*exchanged.TrajectoryDuration_s);
end

function testZeroClockEdgesCannotEscapeCavity(testCase)
    r = exampleStaticUShapedObstacle(struct('PlotOutputs',false,'Verbose',false));
    verifyTrue(testCase,r.Success,r.Message);
    verifyFalse(testCase,r.SolverDiagnostics.LowerBoundAttempt.Passed);
    verifyEqual(testCase,r.SolverDiagnostics.LowerBoundAttempt.TrajectorySocpCount,0);
end
