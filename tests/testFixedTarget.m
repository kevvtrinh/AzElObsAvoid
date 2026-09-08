function tests = testFixedTarget
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testFixedTarget.m')
% PURPOSE: Verify target interpolation, physical straight motion, and rejection.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Function-based test results.
% UNITS: Coordinate units and seconds.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'),fullfile(root,'examples'));
    testCase.TestData.Result = exampleInterceptMovingTargetAtSetTime(struct('PlotOutputs',false,'Verbose',false));
end

function testFixedTargetMinimumJerk(testCase)
    r = testCase.TestData.Result;
    verifyTrue(testCase,r.Success,r.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
    verifyEqual(testCase,r.ArrivalTime_s,12,'AbsTol',1e-10);
    target = r.Inputs.goalState.targetMotion;
    expected = interp1(target.time_s,target.position_units,12,'pchip');
    verifyEqual(testCase,r.position_units(end,:),expected,'AbsTol',1e-10);
    verifyEqual(testCase,r.MotionLength_units,norm(expected-r.position_units(1,:)),'AbsTol',1e-10);
    verifyEqual(testCase,r.SolverDiagnostics.TrajectorySocpCount,0);
end

function testTargetSourceTampering(testCase)
    r = testCase.TestData.Result;
    r.Inputs.goalState.targetMotion.position_units(:,1) = r.Inputs.goalState.targetMotion.position_units(:,1)+1;
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
end

function testInvalidTargetDomain(testCase)
    r = testCase.TestData.Result;
    goal = r.Inputs.goalState; goal.time_s = 31;
    verifyError(testCase,@() planner([],r.Inputs.initialState,goal,r.Limits,r.Options),'planner:TargetTimeOutsideHistory');
    goal = r.Inputs.goalState; goal.targetMotion.time_s(2) = 0;
    verifyError(testCase,@() planner([],r.Inputs.initialState,goal,r.Limits,r.Options),'planner:InvalidTargetTime');
end

function testUnsupportedTargetMode(testCase)
    r = testCase.TestData.Result; options = r.Options; options.GoalTimeMode = "earliestArrival";
    verifyError(testCase,@() planner([],r.Inputs.initialState,r.Inputs.goalState,r.Limits,options),'planner:UnsupportedTargetMode');
end
