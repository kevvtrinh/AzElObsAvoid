function tests = testFastconePlanes
% Compare geometric certificates with a separate high-accuracy conic oracle.
tests = functiontests(localfunctions);
end

function setupOnce(~)
root=fileparts(fileparts(mfilename('fullpath'))); addpath(root,fullfile(root,'trajectory'));
end

function testRandomSeparatedAndOverlappingPolygons(testCase)
rng(107,'twister');
acceptedCount = 0; oracleStalls = 0;
options = optimoptions('coneprog','Display','none', ...
    'ConstraintTolerance',1e-10,'OptimalityTolerance',1e-10);
for k = 1:32
    angle = (0:5)'*pi/3 + .12*k;
    vertices = [cos(angle),sin(angle)]+[.1*k 0];
    points = [linspace(-2,2,8)',.4*randn(8,1)]+[0 1.5*(mod(k,3)-1)];
    [A,b,cones] = makeProblem(points,vertices,.01);
    [x,value,accepted,certificate] = fastcone.separatingLine(A,b,1e-8);
    [oracle,reference,flag,~,lambda] = coneprog([zeros(6,1);1],cones,A,b,[],[],[],[],options);
    oracleStalls = oracleStalls+(flag~=1);
    % A flag of -7 at 1e-10 is not itself an oracle. Certify its result using
    % original primal equations and a separate, feasible conic dual witness.
    dualBound = independentDualBound(A,b,lambda.ineqlin);
    verifyLessThanOrEqual(testCase,reference-dualBound,2e-7);
    verifyLessThanOrEqual(testCase,fastcone.residual(oracle,[zeros(6,1);1],cones,A,b,[],[],[],[]),1e-8);
    verifyLessThanOrEqual(testCase,certificate.LowerBound,reference+2e-7);
    verifyLessThanOrEqual(testCase,reference,value+2e-7);
    verifyLessThanOrEqual(testCase,max(A*x-b),1e-12);
    verifyLessThanOrEqual(testCase,max(vecnorm(reshape(x(1:4),2,[])',2,2)),1+1e-12);
    if accepted
        acceptedCount = acceptedCount+1;
        verifyLessThanOrEqual(testCase,abs(value-reference),2e-7);
    end
end
verifyGreaterThan(testCase,acceptedCount,0);
fprintf('Independent oracle certificates: 32; strict solver stopping flags other than 1: %d.\n',oracleStalls);
end

function bound = independentDualBound(A,b,multipliers)
% Any convex obstacle combinations and simplex row weights give a weak-dual
% bound. This witness uses only norms and matrix products, no polygon projector.
first = find(A(:,5)==-1 & A(:,7)==0); second = find(A(:,6)==-1 & A(:,7)==0);
products = find(A(:,7)==-1);
weights = max(0,multipliers(products)); weights=weights/sum(weights);
row = weights'*A(products,1:6); vertices=-A(first,1:2);
obstacle0=max(0,multipliers(first)); obstacle1=max(0,multipliers(second));
if sum(obstacle0)==0, obstacle0=ones(size(obstacle0)); end
if sum(obstacle1)==0, obstacle1=ones(size(obstacle1)); end
center0=(obstacle0'/sum(obstacle0))*vertices;
center1=(obstacle1'/sum(obstacle1))*vertices;
bound=-b(first(1))-norm(row(1:2)-row(5)*center0)-norm(row(3:4)-row(6)*center1);
end

function testTranslationAndScaling(testCase)
points = [-3 0;-2 0;-1 0;0 0]; vertices = [1 -1;2 -1;2 1;1 1];
for scale = [1e-3 1 1e3]
    for shift = [-10 0 10]
        [A,b,~] = makeProblem(scale*points+shift,scale*vertices+shift,.01*scale);
        [~,value,accepted,c] = fastcone.separatingLine(A,b,1e-10);
        verifyTrue(testCase,accepted);
        verifyFalse(testCase,c.Unique);
        verifyEqual(testCase,value,-.99*scale,'AbsTol',1e-10*max(1,scale));
        verifyLessThanOrEqual(testCase,c.Gap,1e-10*max(1,scale));
    end
end
end

function testInteriorContactDeterminesBothNormals(testCase)
points=[-3 0;-2.2 .3;-2 .5;-3 1]; vertices=[0 -2;1 -2;1 2;0 2];
[A,b,~]=makeProblem(points,vertices,.01);
[~,~,accepted,c]=fastcone.separatingLine(A,b,1e-10);
verifyTrue(testCase,accepted); verifyTrue(testCase,c.Unique);
end

function testEndpointCenterPreservesCertifiedOptimum(testCase)
points=[-3 0;-2 0;-1 0;0 0]; vertices=[1 -1;2 -1;2 1;1 1];
for reverse=[false true]
    for scale=[.001 1 1000]
        p=points; if reverse, p=flipud(p); end
        [A,b,cones]=makeProblem(scale*p+[13 -7],scale*vertices+[13 -7],.01*scale);
        [x,value,accepted,c]=fastcone.separatingLine(A,b,1e-10);
        verifyTrue(testCase,accepted);
        [center,centered]=fastcone.centerEndpointPlane(x,A,b,c.ContactRow);
        verifyTrue(testCase,centered);
        verifyEqual(testCase,center(7),value);
        verifyLessThanOrEqual(testCase,fastcone.residual(center,[zeros(6,1);1], ...
            cones,A,b,[],[],[],[]),1e-9*max(1,scale));
        verifyLessThan(testCase,min(norm(center(1:2)),norm(center(3:4))),.99);
    end
end
end

function [A,b,cones] = makeProblem(points,vertices,target)
degree = size(points,1)-1; beta = (0:degree+1)'/(degree+1); alpha=1-beta;
m = size(vertices,1);
A = zeros(2*m+degree+2,7); b = zeros(size(A,1),1);
A(1:m,1:2)=-vertices; A(1:m,5)=-1;
A(m+(1:m),3:4)=-vertices; A(m+(1:m),6)=-1; b(1:2*m)=-target;
rows = 2*m+(1:degree+2);
A(rows(1:end-1),1:2)=alpha(1:end-1).*points;
A(rows(2:end),3:4)=beta(2:end).*points;
A(rows,5:6)=[alpha beta]; A(rows,7)=-1;
for k = 1:2
    C=zeros(2,7); C(:,2*k-1:2*k)=eye(2);
    cones(k)=secondordercone(C,zeros(2,1),zeros(7,1),-1); %#ok<AGROW>
end
end
