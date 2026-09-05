function [x,value,accepted,certificate]=fourContacts(A,b,tolerance)
%% Section 0: Header & Readme
% SYNTAX: [x,value,accepted,certificate] = fastcone.fourContacts(A,b,tolerance)
% PURPOSE: Certify four-contact planes using explicit determinant equations.
% INPUTS: Canonical plane inequalities and objective tolerance.
% OUTPUTS: Candidate, objective, acceptance and nonnegative dual certificate.
% UNITS: Input coordinates. Unresolved profiles require reference recovery.
% Four contacts leave a one-dimensional null space. Intersect it with the
% product of two unit disks, then certify the resulting normal pair by a
% nonnegative four-contact dual combination. All equations are explicit.

%% Section 1: Propose Contacts And Intersect Their Null Space With Two Disks
obstacle=find(A(:,5)==-1 & A(:,7)==0);
V=-full(A(obstacle,1:2)); target=-b(obstacle(1));
R=full(A(A(:,7)==-1,1:6)); m=size(R,1); nv=size(V,1);
x=[]; value=Inf; accepted=false; certificate=struct('Gap',Inf,'Unique',false);
if m<4, return; end
[proposal,~,~,initial]=fastcone.separatingLine(A,b,tolerance);
first=initial.ContactRow; [~,second]=max(R*proposal(1:6));
if first==second, return; end
other=setdiff(1:m,[first second]); pairs=nchoosek(other,2)';
contacts=sort([repmat([first;second],1,size(pairs,2));pairs],1);
nt=size(contacts,2); if nt*nv^2>2e5, return; end
[v0,v1]=ndgrid(1:nv,1:nv);
v0=repelem(v0(:)',nt); v1=repelem(v1(:)',nt); ids=repmat(contacts,1,nv^2);
C0x=reshape(R(ids,1),size(ids))-reshape(R(ids,5),size(ids)).*V(v0,1)';
C0y=reshape(R(ids,2),size(ids))-reshape(R(ids,5),size(ids)).*V(v0,2)';
C1x=reshape(R(ids,3),size(ids))-reshape(R(ids,6),size(ids)).*V(v1,1)';
C1y=reshape(R(ids,4),size(ids))-reshape(R(ids,6),size(ids)).*V(v1,2)';
D0x=C0x(2:4,:)-C0x(1,:); D0y=C0y(2:4,:)-C0y(1,:);
D1x=C1x(2:4,:)-C1x(1,:); D1y=C1y(2:4,:)-C1y(1,:);
q=[det3(D0y,D1x,D1y);-det3(D0x,D1x,D1y); ...
    det3(D0x,D0y,D1y);-det3(D0x,D0y,D1x)];
radius0=hypot(q(1,:),q(2,:)); radius1=hypot(q(3,:),q(4,:));
scale=max(radius0,radius1); possible=find(scale>0 & isfinite(scale));
if isempty(possible), return; end
normals=q(:,possible)./scale(possible); normals=[normals,-normals];
profile=[possible possible];
support0=sum(V(v0(profile),:)'.*normals(1:2,:),1);
support1=sum(V(v1(profile),:)'.*normals(3:4,:),1);
roundoff=128*eps*max(1,max(abs(V),[],'all'));
keep=all(V*normals(1:2,:)>=support0-roundoff,1) & ...
    all(V*normals(3:4,:)>=support1-roundoff,1);
if ~any(keep), return; end
profile=profile(keep); normals=normals(:,keep);
unit0=radius0(profile)>=radius1(profile);
n0x=normals(1,:); n0y=normals(2,:); n1x=normals(3,:); n1y=normals(4,:);
t0=-C0x(:,profile).*n0y+C0y(:,profile).*n0x;
t1=-C1x(:,profile).*n1y+C1y(:,profile).*n1x;
H0=C0x(:,profile); H1=C0y(:,profile); H2=t1;
H0(:,unit0)=t0(:,unit0); H1(:,unit0)=C1x(:,profile(unit0)); H2(:,unit0)=C1y(:,profile(unit0));
w=[det3(H0(2:4,:),H1(2:4,:),H2(2:4,:)); ...
    -det3(H0([1 3 4],:),H1([1 3 4],:),H2([1 3 4],:)); ...
    det3(H0([1 2 4],:),H1([1 2 4],:),H2([1 2 4],:)); ...
    -det3(H0(1:3,:),H1(1:3,:),H2(1:3,:))];
w=w./sum(w,1);
keep=all(isfinite(w) & w>=-128*eps,1);
if ~any(keep), return; end
profile=profile(keep); normals=normals(:,keep); w=max(0,w(:,keep)); w=w./sum(w,1);
lower=target-hypot(sum(C0x(:,profile).*w,1),sum(C0y(:,profile).*w,1))- ...
    hypot(sum(C1x(:,profile).*w,1),sum(C1y(:,profile).*w,1));
offset0=target-min(V*normals(1:2,:),[],1); offset1=target-min(V*normals(3:4,:),[],1);
points=[normals;offset0;offset1]; values=max(R*points,[],1);
gaps=max(0,values-lower); [gap,best]=min(gaps);
x=[points(:,best);values(best)]; value=values(best);
accepted=gap<=tolerance*(1+abs(value));
certificate=struct('Gap',gap,'LowerBound',lower(best),'Unique',true, ...
    'ContactRows',ids(:,profile(best)),'Weights',w(:,best), ...
    'Profile','four Bernstein contacts, direct determinant equations');
end

function value=det3(a,b,c)
% Batched scalar 3-by-3 determinants, with column vectors stored by profile.
value=a(1,:).*(b(2,:).*c(3,:)-b(3,:).*c(2,:))- ...
      a(2,:).*(b(1,:).*c(3,:)-b(3,:).*c(1,:))+ ...
      a(3,:).*(b(1,:).*c(2,:)-b(2,:).*c(1,:));
end
