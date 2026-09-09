function tests = testStaticFixedArrival
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testStaticFixedArrival.m')
% PURPOSE: Check the shared fixed-clock objective on a protected static detour.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Function-based tests.
% UNITS: Coordinate units and seconds.
tests = functiontests(localfunctions);
end

function testAlternatingTargetOcclusion(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'),fullfile(root,'examples'));
    r = exampleStraightTargetAlternatingOcclusion(struct('PlotOutputs',false,'Verbose',false));
    verifyTrue(testCase,r.Success,r.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
    verifyEqual(testCase,r.TrajectoryDuration_s,20.8695652173913,'AbsTol',1e-8);
    verifyLessThanOrEqual(testCase,r.MotionLength_units,13.6104156606847);
    verifyLessThan(testCase,sum(totalJerkVariation(r)),6);
end

function testTargetExitsObstacle(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'),fullfile(root,'examples'));
    r = exampleTargetExitsObstacle(struct('PlotOutputs',false,'Verbose',false));
    verifyTrue(testCase,r.Success,r.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
    verifyEqual(testCase,r.TrajectoryDuration_s,24,'AbsTol',1e-8);
    verifyLessThanOrEqual(testCase,r.MotionLength_units,20.685146756819);
    verifyLessThan(testCase,sum(totalJerkVariation(r)),7);
    verifyEqual(testCase,r.SolverDiagnostics.ConstraintRepresentation,"fixedClockElasticSocp");
    verifyGreaterThan(testCase,sum(vecnorm(diff(r.Route_units),2,2)),norm(r.Route_units(end,:)-r.Route_units(1,:)));
end
