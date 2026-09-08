function tests = testEarliestRefinement
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testEarliestRefinement.m')
% PURPOSE: Check general earliest-arrival refinement and physical C2 export.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Function-based tests.
% UNITS: Coordinate units and seconds.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'),fullfile(root,'examples'));
end

function testOpposingUQuality(testCase)
    r = exampleTwoOpposingUVisibilityGraph(struct('PlotOutputs',false,'Verbose',false));
    verifyTrue(testCase,r.Success,r.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
    verifyLessThanOrEqual(testCase,r.TrajectoryDuration_s,22.1006280522374);
    verifyLessThanOrEqual(testCase,r.MotionLength_units,24.2057644137651);
    verifyTrue(testCase,r.SolverDiagnostics.LowerBoundAttempt.Passed);
    verifyEqual(testCase,r.SolverDiagnostics.ConicSolver.CallCount,r.SolverDiagnostics.TrajectorySocpCount);
end

function testSharedPhysicalDerivatives(testCase)
    controls = [-4+8*[0;0;0;0;0.5;1;1;1;1],zeros(9,1)];
    spans = zeros(2,9,2);
    spans(1,:,:) = bmtpEngine.restrictBezier(controls,[0,0.01]);
    spans(2,:,:) = bmtpEngine.restrictBezier(controls,[0.01,1]);
    spans(1,end,1) = spans(1,end,1)+1e-11;
    durations_s = [0.1;9.9];
    differences = diff(spans,3,2)*336;
    initialGap = abs(differences(1,end,1)/durations_s(1)^3-differences(2,1,1)/durations_s(2)^3);
    verifyGreaterThan(testCase,initialGap,1e-8);
    p = bmtpEngine.createPowerPolynomial(spans,durations_s,7);
    for order = 0:2
        indices = order:8;
        coefficients = p.positionPower_units(:,:,indices+1).*reshape(factorial(indices)./factorial(indices-order),1,1,[])./durations_s.^order;
        verifyEqual(testCase,sum(coefficients(1,:,:),3),coefficients(2,:,1),'AbsTol',1e-8);
    end
    verifyEqual(testCase,p.positionPower_units(1,:,1),[-4,0],'AbsTol',1e-12);
    verifyEqual(testCase,sum(p.positionPower_units(2,:,:),3),[4,0],'AbsTol',1e-12);
end
