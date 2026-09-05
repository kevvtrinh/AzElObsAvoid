function certificate=infeasibilityCertificate(G,h,z,linearCount,E,d,lb,ub,dependent,activeEqualities,tolerance)
%% Section 0: Header & Readme
% SYNTAX: certificate = fastcone.infeasibilityCertificate(G,h,z,nl,E,d,lb,ub,dependent,activeEqualities,tolerance)
% PURPOSE: Certify infeasibility using the original conic and equality arrays.
% INPUTS: Original program and box, candidate dual and equality pivot indices.
% OUTPUTS: Certified status, dual witness, bound, margin and arithmetic allowance.
% UNITS: Original unscaled coordinates and absolute constraint tolerance.
% Any raw cone dual and equality multiplier give a bound on the zero objective.
% A positive bound excludes even the original tolerance-expanded feasible set.
%% Section 1: Form A Raw Dual Witness And Bound The Relaxed Feasible Set
n=numel(lb); certificate=struct('Certified',false,'Margin',-Inf);
if any(~isfinite(lb) | ~isfinite(ub)), return; end
z=z/max(1,norm(z,Inf)); z(1:linearCount)=max(0,z(1:linearCount));
for head=linearCount+1:3:numel(h)
    radius=hypot(z(head+1),z(head+2));
    z(head)=max(z(head),radius)+4*eps(max(1,radius));
end
gradient=G'*z; lambda=zeros(size(E,1),1);
if ~isempty(dependent) && numel(activeEqualities)==numel(dependent)
    lambda(activeEqualities)=-(E(activeEqualities,dependent)'\gradient(dependent));
end
if any(~isfinite(lambda)), return; end
gradient=gradient+E'*lambda;
boxRoundoff=4*eps(max(1,max(abs(lb),abs(ub))));
boxLower=lb-tolerance-boxRoundoff; boxUpper=ub+tolerance+boxRoundoff;
endpoints=boxLower; endpoints(gradient<0)=boxUpper(gradient<0);
% Scalar inequalities and cone heads may each violate by tolerance. Equality
% residuals may have either sign; expanded bounds are also retained in the box.
relaxation=tolerance*(sum(z(1:linearCount))+sum(z(linearCount+1:3:end))+sum(abs(lambda)));
bound=-h'*z-d'*lambda+gradient'*endpoints-relaxation;
scale=abs(h)'*abs(z)+abs(d)'*abs(lambda)+ ...
    (abs(G)'*abs(z)+abs(E)'*abs(lambda))'*max(abs(boxLower),abs(boxUpper))+relaxation;
roundoff=(size(G,1)+size(E,1)+n+16)*eps*max(1,scale);
certificate=struct('Certified',isfinite(bound)&&bound>roundoff, ...
    'Margin',bound-roundoff,'LowerBound',bound,'RoundoffAllowance',roundoff, ...
    'Tolerance',tolerance,'Dual',z,'EqualityDual',lambda);
end
