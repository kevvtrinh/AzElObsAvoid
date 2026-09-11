function [plane,exitFlag,output] = solveMaximumMarginLine(controlPoint_units,vertices_units,target_units,reserve_units,options)
%% Section 0: Header & Readme
% SYNTAX: [plane,exitFlag,output] = bmtpEngine.solveMaximumMarginLine(controls,vertices,target,reserve,options)
% PURPOSE: Solve and exactly verify a degree-one maximum-margin separating line between one
%   Bezier span and one convex exclusion region.
% INPUTS: Control and obstacle vertices in units, nonnegative clearance and reserve, cone options.
% OUTPUTS: Separating-line record, coneprog exit flag, and measured solver diagnostics.
% UNITS: Position, offsets, margin, target, and reserve are coordinate units.

%% Section 1: Assemble And Solve The Plane SOCP
degree=size(controlPoint_units,1)-1;
variableCount=7;
offsetIndex=5:6;
marginIndex=7;
A=zeros(2*size(vertices_units,1)+degree+2,variableCount);
b=zeros(size(A,1),1);
row=0;
for endpoint=0:1
    targets=row+(1:size(vertices_units,1));
    A(targets,endpoint*2+(1:2))=-vertices_units;
    A(targets,offsetIndex(endpoint+1))=-1;
    b(targets)=-target_units;
    row=targets(end);
end
beta=(0:degree+1).'/(degree+1);
alpha=1-beta;
productRows=zeros(degree+2,variableCount);
productRows(1:end-1,1:2)=alpha(1:end-1).*controlPoint_units;
productRows(2:end,3:4)=beta(2:end).*controlPoint_units;
productRows(:,offsetIndex)=[alpha,beta];
targets=row+(1:degree+2);
A(targets,:)=productRows;
A(targets,marginIndex)=-1;
f=zeros(variableCount,1);
f(marginIndex)=1;
emptyCone=secondordercone(zeros(2,variableCount),zeros(2,1),zeros(variableCount,1),-1);
cones=repmat(emptyCone,2,1);
for endpoint=0:1
    coneA=zeros(2,variableCount);
    coneA(:,endpoint*2+(1:2))=eye(2);
    cones(endpoint+1)=secondordercone(coneA,zeros(2,1),zeros(variableCount,1),-1);
end
timer=tic;
[x,~,exitFlag,output]=coneprog(f,cones,A,b,[],[],[],[],options);
output.TotalTime_s=toc(timer);
plane=struct('Active',false,'Verified',false,'ExitFlag',exitFlag, ...
    'Normal',zeros(2,2),'Offset_units',zeros(1,2),'SignedGap_units',NaN);
if isempty(x) || any(~isfinite(x)), return; end
plane.Active=true;
plane.Normal=reshape(x(1:4),2,[]).';
plane.Offset_units=x(offsetIndex).';
plane=bmtpEngine.verifySeparatingLine( ...
    plane,controlPoint_units,vertices_units,reserve_units,target_units);
end
