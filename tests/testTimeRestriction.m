function tests = testTimeRestriction
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testTimeRestriction.m')
% PURPOSE: Verify exact interval restriction and full-history direct motion.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Function-based tests.
% UNITS: Coordinate units and seconds.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'),fullfile(root,'examples'));
    testCase.TestData.Result = exampleFourAcceleratingCircles(struct('PlotOutputs',false,'Verbose',false));
end

function testPolynomialIdentity(testCase)
    rng(421);
    controls = randn(9,2);
    for interval = [0 0.2 0.4 0;1 0.7 1 0.1]
        restricted = bmtpEngine.restrictBezier(controls,interval);
        tau = linspace(0,1,41).'; physicalTau = interval(1)+diff(interval)*tau;
        original = zeros(41,2); actual = original;
        for k = 0:8
            original = original+nchoosek(8,k)*(physicalTau.^k.*(1-physicalTau).^(8-k))*controls(k+1,:);
            actual = actual+nchoosek(8,k)*(tau.^k.*(1-tau).^(8-k))*restricted(k+1,:);
        end
        verifyEqual(testCase,actual,original,'AbsTol',2e-14);
    end
end

function testDenseHistoryWithoutOptimizerSpans(testCase)
    r = testCase.TestData.Result;
    verifyTrue(testCase,r.Success,r.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
    verifyEqual(testCase,r.TrajectoryDuration_s,22,'AbsTol',1e-10);
    verifyEqual(testCase,r.MotionLength_units,20,'AbsTol',1e-10);
    verifyEqual(testCase,r.Polynomial.SegmentCount,2);
    verifyEqual(testCase,r.PlaneCertificate.VerifiedPairCount,880);
    verifyEqual(testCase,r.SolverDiagnostics.TrajectorySocpCount,0);
end

function testForgedTimeCoverageRejected(testCase)
    r = testCase.TestData.Result;
    r.PlaneCertificate.Coverage.ActiveTimeInterval_s = r.PlaneCertificate.Coverage.ActiveTimeInterval_s+1;
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
end

function testInteriorSourceChangeRejected(testCase)
    r = testCase.TestData.Result;
    r.Inputs.obstacles(1).x_units{80} = r.Inputs.obstacles(1).x_units{80}+0.1;
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
end
