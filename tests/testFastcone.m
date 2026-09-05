function tests=testFastcone
%% Section 0: Header & Readme
% SYNTAX: results=runtests('tests/testFastcone.m')
% PURPOSE: Check public solver recovery, analytical planes, and BMTP routing.
% INPUTS: None. MATLAB and Optimization Toolbox are required.
% OUTPUTS: MATLAB function-based unit tests.
% UNITS: Test conic coordinates use arbitrary consistent units.
tests=functiontests(localfunctions);
end
function setupOnce(testCase)
% Use the package from this checkout and preserve ordinary solver tolerances.
root=fileparts(fileparts(mfilename('fullpath'))); addpath(root,fullfile(root,'trajectory'));
testCase.TestData.Root=root;
testCase.TestData.Options=optimoptions('coneprog','Display','none', ...
    'ConstraintTolerance',1e-8,'OptimalityTolerance',1e-8,'MaxIterations',150);
end
function testKnownOptimumAndExecutedMethod(testCase)
% The MATLAB kernel certifies this problem without a native binary.
cone=secondordercone(eye(2),zeros(2,1),zeros(2,1),-1);
args={[-1;0],cone,[],[],[0 1],.3,[-2;-2],[2;2],testCase.TestData.Options};
[x,f,flag,out]=fastcone.solve(args{:});
verifyGreaterThan(testCase,flag,0); verifyEqual(testCase,f,-sqrt(.91),'AbsTol',2e-7);
verifyLessThanOrEqual(testCase,fastcone.residual(x,args{:}),1e-8);
verifyFalse(testCase,out.NativeAvailable);
verifyFalse(testCase,out.NativeAccepted);
verifyTrue(testCase,out.MatlabAccepted);
verifyFalse(testCase,out.FallbackUsed);
stats=fastcone.accumulate(fastcone.accumulate(),out);
verifyEqual(testCase,stats.CallCount,1);
verifyEqual(testCase,stats.RecoveryCount,0);
verifyEqual(testCase,stats.NativeAcceptedCount,0);
verifyEqual(testCase,stats.MatlabAcceptedCount,1);
verifyEqual(testCase,stats.AnalyticalPlaneCount,0);
end
function testInfeasibilityRetainsOriginalRecoveryFlag(testCase)
% A failed prototype remains unresolved until the reference solver is run.
args={1,[], -1,-2,[],[],0,1,testCase.TestData.Options};
[~,~,flag,out]=fastcone.solve(args{:});
verifyLessThanOrEqual(testCase,flag,0); verifyTrue(testCase,out.FallbackUsed);
verifyFalse(testCase,out.NativeAccepted); verifyEqual(testCase,out.Method,'coneprog recovery');
end
function testBmtpPlaneUsesAnalyticalEntry(testCase)
% An interior contact fixes both plane normals and has an exact geometric bound.
points=[-3 0;-2.2 .3;-2 .5;-3 1]; vertices=[0 -2;1 -2;1 2;0 2];
[plane,flag,out]=bmtpEngine.solveSeparatingLine(points,vertices,.01,0,testCase.TestData.Options);
verifyEqual(testCase,flag,1); verifyTrue(testCase,plane.Verified);
verifyFalse(testCase,out.FallbackUsed); verifyFalse(testCase,out.NativeAccepted);
stats=fastcone.accumulate(fastcone.accumulate(),out);
verifyEqual(testCase,stats.AnalyticalPlaneCount,1);
end
function testProductionConicCallSitesAreWired(testCase)
% Both construction paths must use the public fastcone entry point.
for name=["solveTrajectoryStep.m","solveSeparatingLine.m"]
    source=fileread(fullfile(testCase.TestData.Root,'trajectory','+bmtpEngine',name));
    if name=="solveSeparatingLine.m"
        verifyTrue(testCase,contains(source,'solver=@fastcone.solve'));
        verifyTrue(testCase,contains(source,'= solver('));
    else
        verifyTrue(testCase,contains(source,'= fastcone.solve('));
    end
    verifyFalse(testCase,contains(source,'= coneprog('));
end
end
function testFastconeHasNoPlannerGeometryDependency(testCase)
% Conic algebra accepts explicit arrays and cannot call the obstacle planner.
files=dir(fullfile(testCase.TestData.Root,'trajectory','+fastcone','*.m'));
for k=1:numel(files)
    source=fileread(fullfile(files(k).folder,files(k).name));
    verifyFalse(testCase,contains(source,'obstacleAvoidance.'));
end
end
function testProductionHasNoNativeArtifacts(testCase)
% The shipped solver must remain runnable without a native build or binary.
folder=fullfile(testCase.TestData.Root,'trajectory','+fastcone');
verifyEmpty(testCase,dir(fullfile(folder,'**','*.mex*')));
verifyEmpty(testCase,dir(fullfile(folder,'**','*.cpp')));
verifyFalse(testCase,isfile(fullfile(folder,'build.m')));
verifyFalse(testCase,isfolder(fullfile(folder,'third_party')));
end
