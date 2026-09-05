function [x,value,accepted,certificate]=contactPlanes(A,b,tolerance,triples,vertexPairs)
%% Section 0: Header & Readme
% SYNTAX: [x,value,accepted,certificate] = fastcone.contactPlanes(A,b,tolerance)
% PURPOSE: Propose and certify multiple-contact separating planes.
% INPUTS: Canonical seven-column plane inequalities and objective tolerance.
% OUTPUTS: Candidate, objective, acceptance and weak-dual bound certificate.
% UNITS: Input coordinates. Unresolved profiles require reference recovery.
% Optional triples and vertexPairs restrict equation proposals only; all
% supplied inequalities remain in the objective and feasibility checks.

%% Section 1: Order Contact Proposals Then Solve Unit-Normal Quadratics
obstacle=find(A(:,5)==-1 & A(:,7)==0);
V=-full(A(obstacle,1:2)); target=-b(obstacle(1));
R=full(A(A(:,7)==-1,1:6)); m=size(R,1); nv=size(V,1);
x=[]; value=Inf; accepted=false; certificate=struct('Gap',Inf,'Unique',false);
if m<3 || m*(m-1)*(m-2)/6*nv^2>2e5, return; end
if nargin<4
    [proposal,~,~,initial]=fastcone.separatingLine(A,b,tolerance);
    first=initial.ContactRow; [~,second]=max(R*proposal(1:6));
    if first~=second
        third=setdiff(1:m,[first second]);
        priority=sort([repmat([first;second],1,numel(third));third],1);
        [~,vertex0]=min(V*proposal(1:2)); [~,vertex1]=min(V*proposal(3:4));
        [x,value,accepted,certificate]=fastcone.contactPlanes(A,b,tolerance,priority,[vertex0;vertex1]);
        if accepted, return; end
    end
    [x,value,accepted,certificate]=fastcone.exchangeContacts(A,b,tolerance);
    if accepted, return; end
    [x,value,accepted,certificate]=fastcone.fourContacts(A,b,tolerance);
    return;
end
nt=size(triples,2);
if nargin<5
    [v0,v1]=ndgrid(1:nv,1:nv);
else
    v0=vertexPairs(1,:); v1=vertexPairs(2,:);
end
pairCount=numel(v0);
v0=repelem(v0(:)',nt); v1=repelem(v1(:)',nt);
ids=repmat(triples,1,pairCount);
C0x=reshape(R(ids,1),size(ids))-reshape(R(ids,5),size(ids)).*V(v0,1)';
C0y=reshape(R(ids,2),size(ids))-reshape(R(ids,5),size(ids)).*V(v0,2)';
C1x=reshape(R(ids,3),size(ids))-reshape(R(ids,6),size(ids)).*V(v1,1)';
C1y=reshape(R(ids,4),size(ids))-reshape(R(ids,6),size(ids)).*V(v1,2)';
D0x=C0x(2:3,:)-C0x(1,:); D0y=C0y(2:3,:)-C0y(1,:);
D1x=C1x(2:3,:)-C1x(1,:); D1y=C1y(2:3,:)-C1y(1,:);
det0=D0x(1,:).*D0y(2,:)-D0x(2,:).*D0y(1,:);
det1=D1x(1,:).*D1y(2,:)-D1x(2,:).*D1y(1,:);
swap=abs(det0)>abs(det1);
Lx=D1x; Ly=D1y; Rx=D0x; Ry=D0y; determinant=det1;
Lx(:,swap)=D0x(:,swap); Ly(:,swap)=D0y(:,swap);
Rx(:,swap)=D1x(:,swap); Ry(:,swap)=D1y(:,swap); determinant(swap)=det0(swap);
M00=-(Ly(2,:).*Rx(1,:)-Ly(1,:).*Rx(2,:))./determinant;
M01=-(Ly(2,:).*Ry(1,:)-Ly(1,:).*Ry(2,:))./determinant;
M10=-(-Lx(2,:).*Rx(1,:)+Lx(1,:).*Rx(2,:))./determinant;
M11=-(-Lx(2,:).*Ry(1,:)+Lx(1,:).*Ry(2,:))./determinant;
Q00=M00.^2+M10.^2; Q11=M01.^2+M11.^2; Q01=M00.*M01+M10.*M11;
average=(Q00+Q11)/2; delta=(Q00-Q11)/2; amplitude=hypot(delta,Q01);
possible=find(isfinite(amplitude) & amplitude>0 & abs(1-average)<=amplitude);
if isempty(possible), return; end
phase=atan2(Q01(possible),delta(possible));
opening=acos((1-average(possible))./amplitude(possible));
theta=[(phase+opening)/2,(phase-opening)/2,(phase+opening)/2+pi,(phase-opening)/2+pi];
profile=repmat(possible,1,4);
a=cos(theta); c=sin(theta);
d=M00(profile).*a+M01(profile).*c; e=M10(profile).*a+M11(profile).*c;
n0x=a; n0y=c; n1x=d; n1y=e; swapped=swap(profile);
n0x(swapped)=d(swapped); n0y(swapped)=e(swapped);
n1x(swapped)=a(swapped); n1y(swapped)=c(swapped);
% A support-vertex profile is admissible only in that vertex's normal cone.
support0=V(v0(profile),1)'.*n0x+V(v0(profile),2)'.*n0y;
support1=V(v1(profile),1)'.*n1x+V(v1(profile),2)'.*n1y;
roundoff=128*eps*max(1,max(abs(V),[],'all'));
keep=all(V*[n0x;n0y]>=support0-roundoff,1) & ...
    all(V*[n1x;n1y]>=support1-roundoff,1);
if ~any(keep), return; end
profile=profile(keep); n0x=n0x(keep); n0y=n0y(keep); n1x=n1x(keep); n1y=n1y(keep);
t0=-C0x(:,profile).*n0y+C0y(:,profile).*n0x;
t1=-C1x(:,profile).*n1y+C1y(:,profile).*n1x;
w=[t0(2,:).*t1(3,:)-t0(3,:).*t1(2,:); ...
   t0(3,:).*t1(1,:)-t0(1,:).*t1(3,:); ...
   t0(1,:).*t1(2,:)-t0(2,:).*t1(1,:)];
w=w./sum(w,1);
keep=all(isfinite(w) & w>=-128*eps,1) & ...
    hypot(n0x,n0y)<=1+tolerance & hypot(n1x,n1y)<=1+tolerance;
if ~any(keep), return; end
profile=profile(keep); w=max(0,w(:,keep)); w=w./sum(w,1);
n0x=n0x(keep); n0y=n0y(keep); n1x=n1x(keep); n1y=n1y(keep);
normals=[n0x;n0y;n1x;n1y];
lower=target-hypot(sum(C0x(:,profile).*w,1),sum(C0y(:,profile).*w,1))- ...
    hypot(sum(C1x(:,profile).*w,1),sum(C1y(:,profile).*w,1));
offset0=target-min(V*normals(1:2,:),[],1);
offset1=target-min(V*normals(3:4,:),[],1);
points=[normals;offset0;offset1]; values=max(R*points,[],1);
gaps=max(0,values-lower); [gap,best]=min(gaps);
x=[points(:,best);values(best)]; value=values(best);
accepted=gap<=tolerance*(1+abs(value));
certificate=struct('Gap',gap,'LowerBound',lower(best),'Unique',true, ...
    'ContactRows',ids(:,profile(best)),'Weights',w(:,best), ...
    'Profile','three Bernstein contacts, direct quadratic equations', ...
    'EnumeratedProfiles',size(ids,2));
end
