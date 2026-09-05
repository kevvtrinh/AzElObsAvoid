function tests=testFastconeKernel
tests=functiontests(localfunctions);
end
function setupOnce(testCase)
root=fileparts(fileparts(mfilename('fullpath'))); addpath(root,fullfile(root,'trajectory'));
testCase.TestData.Options=optimoptions('coneprog','Display','none', ...
    'ConstraintTolerance',1e-8,'OptimalityTolerance',1e-8,'MaxIterations',150);
assumeTrue(testCase,exist(fullfile(fileparts(fileparts(mfilename('fullpath'))),'trajectory','+fastcone',['core.' mexext]),'file')==3,'Build the optional native core first.');
end
function testKnownLorentzOptimum(testCase)
cone=secondordercone(eye(2),[0;0],[0;0],-1);
args={[-1;0],cone,[],[],[0 1],.3,[-2;-2],[2;2],testCase.TestData.Options};
[x,f,flag,out]=fastcone.solveNative(args{:});
verifyEqual(testCase,flag,1,out.message);
verifyFalse(testCase,out.nativeUnsupported);
verifyEqual(testCase,x,[sqrt(.91);.3],'AbsTol',2e-7);
verifyEqual(testCase,f,-sqrt(.91),'AbsTol',2e-7);
verifyLessThanOrEqual(testCase,fastcone.residual(x,args{:}),args{9}.ConstraintTolerance);
verifyLessThanOrEqual(testCase,abs(out.certifiedObjectiveGap),args{9}.OptimalityTolerance*(1+abs(f)));
end
function testCubicTimeConeOptimum(testCase)
cones(1)=secondordercone([0 2 0 0;1 0 -1 0],[0;0],[1;0;1;0],0);
cones(2)=secondordercone([0 0 2 0;0 1 0 -1],[0;0],[0;1;0;1],0);
args={[0;0;0;1],cones,[],[],[1 0 0 0],1,[1;2;0;0],[1;10;100;1000],testCase.TestData.Options};
[x,f,flag,out]=fastcone.solveNative(args{:});
verifyEqual(testCase,flag,1,out.message);
verifyEqual(testCase,x,[1;2;4;8],'AbsTol',2e-6);
verifyEqual(testCase,f,8,'AbsTol',2e-6);
verifyLessThanOrEqual(testCase,fastcone.residual(x,args{:}),args{9}.ConstraintTolerance);
verifyLessThanOrEqual(testCase,abs(out.certifiedObjectiveGap),args{9}.OptimalityTolerance*(1+abs(f)));
end
function testEliminatedNormOptimum(testCase)
cone=secondordercone([1 0 0;0 1 0],[0;0],[0;0;1],0);
args={[0;0;1],cone,[],[],[1 0 0;0 1 0],[3;4],[-10;-10;0],[10;10;10],testCase.TestData.Options};
[x,f,flag,out]=fastcone.solveNative(args{:});
verifyEqual(testCase,flag,1,out.message);
verifyEqual(testCase,x,[3;4;5],'AbsTol',2e-7);
verifyEqual(testCase,f,5,'AbsTol',2e-7);
verifyLessThanOrEqual(testCase,fastcone.residual(x,args{:}),args{9}.ConstraintTolerance);
end
function testInfeasibleRowRemainsUnresolved(testCase)
cone=secondordercone([0;0],[0;0],0,-1);
[~,~,flag]=fastcone.solveNative(1,cone,-1,-2,[],[],0,1,testCase.TestData.Options);
verifyEqual(testCase,flag,0);
end
function testInvalidNativeDimensionsRaiseError(testCase)
verifyError(testCase,@invalidCall,'fastcone:nativeCore');
end
function invalidCall
[~,~,~]=fastcone.core(sparse(1),zeros(2,1),1,0,1,1,0,1e-8,1e-8,100,0,1,struct());
end
function testMixedAnalyticalNormProfiles(testCase)
n=7; selector=eye(n); zero=zeros(2,n);
cones(1)=secondordercone(zero,[0;0],selector(:,3),0);
cones(2)=secondordercone([selector(1,:);zeros(1,n)],[.5;0],selector(:,4),0);
cones(3)=secondordercone([zeros(1,n);selector(1,:)],[0;.5],selector(:,5),0);
cones(4)=secondordercone(zero,[3;4],selector(:,6),0);
cones(5)=secondordercone(selector(1:2,:),[.5;-1],selector(:,7),0);
cones(6)=secondordercone(zero,[.3;.4],zeros(n,1),-1);
args={[0;0;ones(5,1)],cones,[],[],[],[],[2;1;zeros(5,1)],[4;4;10*ones(5,1)],testCase.TestData.Options};
[x,f,flag,out]=fastcone.solveNative(args{:});
verifyEqual(testCase,flag,1,out.message);
verifyEqual(testCase,x,[2;1;0;1.5;1.5;5;2.5],'AbsTol',2e-6);
verifyEqual(testCase,f,10.5,'AbsTol',2e-6);
verifyLessThanOrEqual(testCase,fastcone.residual(x,args{:}),args{9}.ConstraintTolerance);
verifyLessThanOrEqual(testCase,abs(out.certifiedObjectiveGap),args{9}.OptimalityTolerance*(1+abs(f)));
end
function testEmptyConeArrayLinearProgram(testCase)
args={[1;2],[],[],[],[1 1],1,[0;0],[1;1],testCase.TestData.Options};
[x,f,flag,out]=fastcone.solveNative(args{:});
verifyEqual(testCase,flag,1,out.message);
verifyEqual(testCase,x,[1;0],'AbsTol',2e-7); verifyEqual(testCase,f,1,'AbsTol',2e-7);
end
