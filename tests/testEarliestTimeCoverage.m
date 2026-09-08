function tests = testEarliestTimeCoverage
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testEarliestTimeCoverage.m')
% PURPOSE: Require authoritative timed coverage for earliest-arrival motion.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Function-based tests.
% UNITS: Coordinate units and seconds.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'));
    x = [-0.5;0.5;0.5;-0.5]; y = [-0.5;-0.5;0.5;0.5];
    obstacle = obstacleAvoidance.obstacles.createObstacle('moving box',[0;12],{x;x},{y+2;y+3},0.1);
    initial = struct('time_s',0,'position_units',[-4,0]);
    goal = struct('time_s',12,'position_units',[4,0]);
    testCase.TestData.Result = planner(obstacle,initial,goal,struct(),struct('GoalTimeMode','earliestArrival'));
end

function testCompleteTimeCoverage(testCase)
    r = testCase.TestData.Result;
    verifyTrue(testCase,r.Success,r.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
    verifyLessThan(testCase,r.ArrivalTime_s,12);
    verifyEqual(testCase,r.PlaneCertificate.Coverage.ActiveTimeInterval_s,[0 12]);
end

function testMissingTimeCoverageRejected(testCase)
    r = testCase.TestData.Result;
    r.PlaneCertificate.Coverage = rmfield(r.PlaneCertificate.Coverage,'ActiveTimeInterval_s');
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
end

function testLaterSourceChangeRejected(testCase)
    r = testCase.TestData.Result;
    r.Inputs.obstacles.y_units{2} = r.Inputs.obstacles.y_units{2}-3;
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
end
