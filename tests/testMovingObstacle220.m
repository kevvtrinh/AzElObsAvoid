function tests = testMovingObstacle220
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testMovingObstacle220.m')
% PURPOSE: Keep the dense moving-obstacle benchmark in the compact core suite.
% INPUTS: MATLAB unit test framework and maintained 220-vertex example.
% OUTPUTS: Independent validity, quality, and exercised-fast-path checks.
% UNITS: Coordinate units, seconds, and physical derivatives.
tests = functiontests(localfunctions);
end

function setupOnce(~)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'),fullfile(root,'examples'));
end

function testMovingObstacle220Quality(testCase)
    result = exampleMovingObstacle220(struct('PlotOutputs',false,'Verbose',false));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.TrajectoryDuration_s,230,'AbsTol',1e-10);
    referenceLength_units=121.503236303671;
    verifyLessThanOrEqual(testCase,abs( ...
        result.MotionLength_units/referenceLength_units-1),0.01);
    verifyGreaterThan(testCase,result.PlaneCertificate.MinimumSignedGap_units,0);
    verifyTrue(testCase,result.VisibilityGraph.GraphIsFullyEnumerated);
    verifyGreaterThan(testCase,result.PlaneCertificate.CachedGeometryPairCount,0);
    verifyGreaterThan(testCase,result.SolverDiagnostics.FullPlaneUpdateSkippedCount,0);
    verifyGreaterThan(testCase, ...
        result.SolverDiagnostics.ConstraintRowPairVerificationCount,0);
    verifyEqual(testCase, ...
        result.SolverDiagnostics.ExistingPlanePairVerificationCount,0);
    verifyTrue(testCase,result.SolverDiagnostics.ConstraintGenerationComplete);
end
