function tests = testEarliestTarget
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testEarliestTarget.m')
% PURPOSE: Check earliest target reachability across all sampled intervals.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Function-based tests.
% UNITS: Coordinate units, seconds, and their derivatives.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'),fullfile(root,'examples'));
    testCase.TestData.Result = exampleInterceptMovingTargetEarliest(struct('PlotOutputs',false,'Verbose',false));
end

function testHistoricalMinimumTime(testCase)
    r = testCase.TestData.Result;
    verifyTrue(testCase,r.Success,r.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
    verifyEqual(testCase,r.ArrivalTime_s,55/9,'AbsTol',1e-8);
    verifyEqual(testCase,r.MotionLength_units,norm([6+0.2*55/9,1+0.02*55/9]),'AbsTol',1e-8);
    verifyEqual(testCase,r.SolverDiagnostics.TrajectorySocpCount,0);
end

function testDisconnectedFeasibleTimes(testCase)
    source = testCase.TestData.Result;
    target = struct('time_s',[0;4;8;20],'position_units',[12,0;2,0;30,0;50,0]);
    goal = struct('time_s',20,'targetMotion',target);
    r = planner([],source.Inputs.initialState,goal,source.Limits,source.Options);
    verifyTrue(testCase,r.Success,r.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
    verifyEqual(testCase,r.ArrivalTime_s,(-9+sqrt(273))/2,'AbsTol',1e-8);
    verifyLessThan(testCase,r.ArrivalTime_s,4);
end

function testUnreachableTarget(testCase)
    source = testCase.TestData.Result;
    target = struct('time_s',[0;20],'position_units',[10,0;210,0]);
    goal = struct('time_s',20,'targetMotion',target);
    r = planner([],source.Inputs.initialState,goal,source.Limits,source.Options);
    verifyFalse(testCase,r.Success);
    verifyEqual(testCase,r.TerminationReason,"targetUnreachable");
    verifyEmpty(testCase,r.time_s);
end

function testShiftedClockAndSourceTampering(testCase)
    source = testCase.TestData.Result;
    initial = source.Inputs.initialState; initial.time_s = 7;
    goal = source.Inputs.goalState; goal.time_s = goal.time_s+7;
    goal.targetMotion.time_s = goal.targetMotion.time_s+7;
    r = planner([],initial,goal,source.Limits,source.Options);
    verifyTrue(testCase,r.Success,r.Message);
    verifyEqual(testCase,r.ArrivalTime_s,7+55/9,'AbsTol',1e-8);
    r.Inputs.goalState.targetMotion.position_units(2,1) = r.Inputs.goalState.targetMotion.position_units(2,1)+0.1;
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
end

function testStationaryTargetsAcrossTimingRegimes(testCase)
    initial = struct('time_s',0,'position_units',[0,0]);
    displacements = [0.1,0;4,0;12,0;0,-6];
    velocity = [2,2;0.25,2;2,2;2,2];
    acceleration = [1,1;10,10;1,1;1,1];
    expected_s = [4*(0.1/4)^(1/3);16+2*sqrt(0.25/2);8.5;5.5];
    for k = 1:4
        target = struct('time_s',[0;30],'position_units',repmat(displacements(k,:),2,1));
        goal = struct('time_s',30,'targetMotion',target);
        limits = struct('maxVelocity_units_s',velocity(k,:), ...
            'maxAcceleration_units_s2',acceleration(k,:),'maxJerk_units_s3',[2,2]);
        r = planner([],initial,goal,limits,struct('GoalTimeMode','earliestArrival'));
        verifyTrue(testCase,r.Success,r.Message);
        verifyTrue(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
        verifyEqual(testCase,r.ArrivalTime_s,expected_s(k),'AbsTol',1e-8);
    end
end
