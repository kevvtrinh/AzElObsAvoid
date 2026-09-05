function tests=testFastconeInfeasibility
% Protect raw-equation infeasibility proofs and tolerance-expanded feasibility.
tests=functiontests(localfunctions);
end
function setupOnce(~)
root=fileparts(fileparts(mfilename('fullpath'))); addpath(root,fullfile(root,'trajectory'));
end
function testCoupledConeFailureHasOriginalWitness(testCase)
options=optimoptions('coneprog','Display','none','ConstraintTolerance',1e-8,'OptimalityTolerance',1e-8);
cone=secondordercone(eye(2),[0;0],[0;0],-1);
args={[1;0],cone,[-1 -1],-2,[],[],[-2;-2],[2;2],options};
[x,~,flag,out]=fastcone.solve(args{:});
verifyEqual(testCase,flag,-2); verifyEmpty(testCase,x);
verifyTrue(testCase,out.CertifiedInfeasible); verifyFalse(testCase,out.FallbackUsed);
c=out.Prototype.originalInfeasibility;
verifyGreaterThan(testCase,c.Margin,0);
stats=fastcone.accumulate(fastcone.accumulate(),out);
verifyEqual(testCase,stats.CertifiedInfeasibleCount,1);
verifyEqual(testCase,stats.AnalyticalPlaneCount,0);
verifyEqual(testCase,stats.RecoveryCount,0);
end
function testToleranceExpandedLinearAndConeControls(testCase)
tolerance=1e-8;
for rhs=[.5 1-1e-10 1.5]
    c=fastcone.infeasibilityCertificate([1 1],rhs,1,1,[1 1],1,[0;0],[1;1],1,1,tolerance);
    verifyEqual(testCase,c.Certified,rhs==.5);
end
G=[0 0;-1 0;0 -1]; z=[1;-1;0];
for x=[2 1+1e-10 .5]
    c=fastcone.infeasibilityCertificate(G,[1;0;0],z,0,[1 0],x,[-3;-3],[3;3],1,1,tolerance);
    verifyEqual(testCase,c.Certified,x==2);
end
end
function testConstructedFeasibleProgramsCannotCertifyFailure(testCase)
rng(58031,'twister');
for trial=1:64
    n=4; A=randn(7,n); x=randn(n,1); C=randn(2,n);
    E=randn(2,n); d=E*x; b=A*x+rand(7,1);
    G=[A;zeros(1,n);-C]; h=[b;norm(C*x)+rand;0;0];
    tail=randn(2,1); dual=[rand(7,1);norm(tail)+rand;tail];
    c=fastcone.infeasibilityCertificate(G,h,dual,7,E,d,x-1,x+1,[1 2],[1 2],1e-8);
    verifyFalse(testCase,c.Certified);
end
end
