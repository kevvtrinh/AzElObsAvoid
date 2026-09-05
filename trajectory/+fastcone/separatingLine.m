function [x, fval, accepted, certificate] = separatingLine(A,b,tolerance)
%% Section 0: Header & Readme
% SYNTAX: [x,fval,accepted,certificate] = fastcone.separatingLine(A,b,tolerance)
% PURPOSE: Certified closed-form single-contact profiles for HS plane SOCPs.
% INPUTS: Exact seven-column inequalities from HS solveSeparatingLine; tolerance.
% OUTPUTS: Feasible normals/offsets/margin, objective, certified status and bound.
% UNITS: The same coordinate units as A and b.
% Multiple-contact cases return accepted=false, never a false optimum.

%% Section 1: Recover The Convex Obstacle And Bernstein Products

obstacleRows = find(A(:,5)==-1 & A(:,7)==0);
vertices = -full(A(obstacleRows,1:2));
target = -b(obstacleRows(1));
rows = full(A(A(:,7)==-1,1:6));
alpha = rows(:,5); beta = rows(:,6);
q0 = rows(:,1:2)./max(alpha,realmin);
q1 = rows(:,3:4)./max(beta,realmin);
[n0,dist0] = closestNormals(q0,vertices);
[n1,dist1] = closestNormals(q1,vertices);
lower = target-alpha.*dist0-beta.*dist1;
[lowerBound,which] = max(lower);
normal0 = n0(which,:); normal1 = n1(which,:);
% A zero-weight normal is unconstrained by the contact. Use the other normal
% as an analytical feasible candidate; the bound proves acceptance if exact.
if alpha(which)==0, normal0 = normal1; end
if beta(which)==0, normal1 = normal0; end
offset0 = target-min(vertices*normal0');
offset1 = target-min(vertices*normal1');
x = [normal0';normal1';offset0;offset1;0];
fval = max(rows*x(1:6)); x(7) = fval;
gap = max(0,fval-lowerBound);
accepted = gap <= tolerance*(1+abs(fval));
certificate = struct('LowerBound',lowerBound,'Gap',gap, ...
    'ContactRow',which,'Profile','single Bernstein contact','Iterations',0, ...
    'Unique',alpha(which)>0 && beta(which)>0 && dist0(which)>0 && dist1(which)>0);
end

function [normal,distance] = closestNormals(points,vertices)
% Exact projection onto a convex polygon: interior or closest edge/vertex.
next = vertices([2:end 1],:); edge = next-vertices;
dx = points(:,1)-vertices(:,1)'; dy = points(:,2)-vertices(:,2)';
cross = edge(:,1)'.*dy-edge(:,2)'.*dx;
inside = all(cross>=0,2) | all(cross<=0,2);
if sum(vertices(:,1).*next(:,2)-vertices(:,2).*next(:,1))==0, inside(:)=false; end
lengthSquared = sum(edge.^2,2)';
tau = max(0,min(1,(dx.*edge(:,1)'+dy.*edge(:,2)')./max(lengthSquared,realmin)));
deltaX = tau.*edge(:,1)'-dx; deltaY = tau.*edge(:,2)'-dy;
[squared,index] = min(deltaX.^2+deltaY.^2,[],2);
distance = sqrt(squared); distance(inside) = 0;
linearIndex = sub2ind(size(deltaX),(1:size(points,1))',index);
normal = [deltaX(linearIndex),deltaY(linearIndex)]./max(distance,realmin);
normal(inside | distance==0,:) = 0;
end
