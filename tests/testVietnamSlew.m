function tests = testVietnamSlew
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testVietnamSlew.m')
% PURPOSE: Validate the unchanged Vietnam benchmark and reject corrupt motion
%          certificates, altered source geometry, and omitted temporal pairs.
% INPUTS: MATLAB unit test framework; checked-in Vietnam MAT input.
% OUTPUTS: Function-based test results.
% UNITS: Coordinate units and seconds.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'),fullfile(root,'examples'));
    testCase.TestData.Result = exampleVietnamKeepoutSlew(struct('PlotOutputs',false,'Verbose',false));
end

function testBenchmarkQuality(testCase)
    r = testCase.TestData.Result;
    verifyTrue(testCase,r.Success,r.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
    verifyEqual(testCase,r.ArrivalTime_s,30,'AbsTol',1e-8);
    verifyLessThanOrEqual(testCase,r.MotionLength_units,17.3053746209);
    verifyLessThanOrEqual(testCase,sum(vecnorm(diff(r.Route_units),2,2)),17.2979913158);
    verifyEqual(testCase,r.PlaneCertificate.AllPairCount,480);
    verifyEqual(testCase,r.SolverDiagnostics.ApplicablePairCount,240);
end

function testOmittedActivePairRejected(testCase)
    r = testCase.TestData.Result;
    index = find(r.PlaneCertificate.RegionActiveBySegment,1);
    r.PlaneCertificate.RegionActiveBySegment(index) = false;
    r.PlaneCertificate.AllPairCount = r.PlaneCertificate.AllPairCount-1;
    r.PlaneCertificate.VerifiedPairCount = r.PlaneCertificate.VerifiedPairCount-1;
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
end

function testAlteredActivityRejected(testCase)
    r = testCase.TestData.Result;
    r.PlaneCertificate.Coverage.ActiveTimeInterval_s(1,2) = 0;
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
end

function testAlteredSourceRejected(testCase)
    r = testCase.TestData.Result;
    r.Inputs.obstacles{1}.x_units{1} = r.Inputs.obstacles{1}.x_units{1}+1;
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
end

function testWeakenedCertificateClearanceRejected(testCase)
    r = testCase.TestData.Result;
    r.PlaneCertificate.RequiredGap_units = 0;
    r.PlaneCertificate.RoundoffReserve_units = 0;
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
end

function testShiftedAbsoluteClock(testCase)
    reference = testCase.TestData.Result;
    obstacles = reference.Inputs.obstacles;
    for k = 1:numel(obstacles), obstacles{k}.time_s = obstacles{k}.time_s+7; end
    initial = reference.Inputs.initialState; initial.time_s = initial.time_s+7;
    goal = reference.Inputs.goalState; goal.time_s = goal.time_s+7;
    shifted = planner(obstacles,initial,goal,reference.Limits,reference.Options);
    verifyTrue(testCase,shifted.Success,shifted.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(shifted).Passed);
    verifyEqual(testCase,shifted.ArrivalTime_s,37,'AbsTol',1e-8);
    verifyEqual(testCase,shifted.MotionLength_units,reference.MotionLength_units,'AbsTol',1e-6);
end
