function [x,value,accepted,certificate]=twoContacts(A,b,tolerance,pairIndices)
%% Section 0: Header & Readme
% SYNTAX: [x,value,accepted,certificate] = fastcone.twoContacts(A,b,tolerance)
% PURPOSE: Solve the distance derivative of a convex two-contact mixture.
% INPUTS: Canonical plane inequalities, tolerance and optional contact indices.
% OUTPUTS: Candidate, objective, acceptance and weak-dual mixture certificate.
% UNITS: Input coordinates. The safeguarded scalar root is iterative.

%% Section 1: Minimize The Mixture Bound And Recover Plane Normals
[x,value,accepted,certificate]=fastcone.separatingLine(A,b,tolerance);
if accepted, return; end
obstacleRows=find(A(:,5)==-1 & A(:,7)==0);
vertices=-full(A(obstacleRows,1:2)); target=-b(obstacleRows(1));
rows=full(A(A(:,7)==-1,1:6));
first=certificate.ContactRow;
[~,second]=max(rows*x(1:6));
if nargin==4, first=pairIndices(1); second=pairIndices(2); end
if first==second, return; end
[lambda,bound,pairIterations]=pairMinimum(rows(first,:),rows(second,:),vertices,tolerance);
blend=(1-lambda)*rows(first,:)+lambda*rows(second,:);
normal=zeros(2,2); radius=zeros(2,1);
for axis=1:2
    weight=blend(4+axis); q=blend(2*axis-1:2*axis)';
    if weight>0
        [projection,~]=projectPoint(q/weight,vertices);
        residual=q-weight*projection;
    else
        residual=q;
    end
    radius(axis)=norm(residual);
    if radius(axis)>0, normal(:,axis)=-residual/radius(axis); end
end
offset=target-min(vertices*normal,[],1);
x=[normal(:);offset';0]; value=max(rows*x(1:6)); x(7)=value;
lower=target-bound; gap=max(0,value-lower);
accepted=gap<=tolerance*(1+abs(value));
certificate=struct('LowerBound',lower,'Gap',gap,'ContactRows',[first second], ...
    'Weights',[1-lambda lambda],'Profile','two Bernstein contacts', ...
    'Iterations',pairIterations,'Unique',all(radius>0),'Radius',radius, ...
    'PairDerivative',(rows(second,:)-rows(first,:))*x(1:6));
end

function [bestLambda,bestValue,iteration]=pairMinimum(first,second,vertices,tolerance)
% Safeguarded roots of the analytical derivative of two polygon distances.
iteration=0;
[atZero,gradientZero]=evaluatePair(0,first,second,vertices);
[atOne,gradientOne]=evaluatePair(1,first,second,vertices);
if gradientZero>=0, bestLambda=0; bestValue=atZero; return; end
if gradientOne<=0, bestLambda=1; bestValue=atOne; return; end
lo=0; hi=1; lambda=.5; bestValue=Inf; bestLambda=lambda;
for iteration=1:128
    [value,gradient,curvature]=evaluatePair(lambda,first,second,vertices);
    if value<bestValue, bestValue=value; bestLambda=lambda; end
    if abs(gradient)*max(lambda,1-lambda)<=0.1*tolerance*max(1,value)
        bestLambda=lambda; bestValue=value; break;
    end
    if gradient>0, hi=lambda; else, lo=lambda; end
    if hi-lo<=32*eps*max(1,abs(lambda)), break; end
    trial=lambda-gradient/curvature;
    if ~isfinite(trial) || trial<=lo+(hi-lo)/4 || trial>=hi-(hi-lo)/4
        trial=(lo+hi)/2;
    end
    lambda=trial;
end
end

function [value,gradient,curvature]=evaluatePair(lambda,first,second,vertices)
value=0; gradient=0; curvature=0; delta=second-first; blend=first+lambda*delta;
for axis=1:2
    q=blend(2*axis-1:2*axis)'; weight=blend(4+axis);
    dq=delta(2*axis-1:2*axis)'; dw=delta(4+axis);
    if weight>0
        [point,P]=projectPoint(q/weight,vertices);
        r=q-weight*point; dr=P*(dq-dw*point);
    else
        r=q; dr=dq;
    end
    radius=norm(r); value=value+radius;
    if radius>0
        slope=(r'*dr)/radius;
        gradient=gradient+slope;
        curvature=curvature+max(0,dr'*dr-slope^2)/radius;
    end
end
end
function [point,P]=projectPoint(q,vertices)
% Return the closest polygon point and its affine-feature normal projector.
next=vertices([2:end 1],:); edge=next-vertices;
delta=q'-vertices;
cross=edge(:,1).*delta(:,2)-edge(:,2).*delta(:,1);
area=sum(vertices(:,1).*next(:,2)-vertices(:,2).*next(:,1));
if area~=0 && (all(cross>=0) || all(cross<=0))
    point=q; P=zeros(2); return;
end
lengthSquared=sum(edge.^2,2);
tau=max(0,min(1,sum(delta.*edge,2)./max(lengthSquared,realmin)));
points=vertices+tau.*edge;
[~,which]=min(sum((points-q').^2,2)); point=points(which,:)';
if tau(which)>0 && tau(which)<1 && lengthSquared(which)>0
    e=edge(which,:)'; P=eye(2)-(e*e')/lengthSquared(which);
else
    P=eye(2);
end
end
