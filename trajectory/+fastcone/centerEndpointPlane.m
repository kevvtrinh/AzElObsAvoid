function [x,accepted,iterations] = centerEndpointPlane(x,A,b,contact)
%% Section 0: Header & Readme
% SYNTAX: [x,accepted,iterations]=fastcone.centerEndpointPlane(x,A,b,contact)
% PURPOSE: Select the analytic center of a single-endpoint optimum plane face.
% INPUTS: Certified single-contact optimum, HS rows, and Bernstein contact row.
% OUTPUTS: Same-objective plane, convergence status, Newton iteration count.
% UNITS: Same position/offset units as the original HS program.
% The margin and determined endpoint plane stay fixed. Only a free normal and
% offset are centered. This is a three-variable Newton solve with explicit
% derivatives, not a closed-form center or an assumed coneprog tie match.

%% Section 1: Identify The Free Endpoint And A Strictly Feasible Interior
accepted=false; iterations=0; original=x;
product=find(A(:,7)==-1); row=A(product(contact),:);
if row(5)==0, free=[1 2 5];
elseif row(6)==0, free=[3 4 6];
else, return; end
obstacle=find(A(:,free(3))==-1 & A(:,7)==0);
vertices=-A(obstacle,free(1:2)); origin=mean(vertices,1);
other=x; other(free)=0;
rhs=b-A*other; C=full(A(:,free));
active=any(C~=0,2); C=C(active,:); rhs=rhs(active);
% Local offset removes large coordinate translations from the Newton system.
C(:,1:2)=C(:,1:2)-C(:,3).*origin;
normal=x(free(1:2)); target=-b(obstacle(1));
support=min((vertices-origin)*normal);
anchor=[normal;-support]; zero=[0;0;target];
rate=C*anchor; slack=rhs-C*zero;
lower=0; upper=1;
if any(rate<0), lower=max(lower,max(slack(rate<0)./rate(rate<0))); end
if any(rate>0), upper=min(upper,min(slack(rate>0)./rate(rate>0))); end
if lower>=upper, return; end
rho=(lower+upper)/2; q=zero+rho*anchor;
remaining=rhs-C*q; positive=C(:,3)>0;
delta=min(remaining(positive)./C(positive,3));
if isempty(delta) || delta<=0 || ~isfinite(delta), return; end
q(3)=q(3)+delta/2;
if any(rhs-C*q<=0) || sum(q(1:2).^2)>=1, return; end

%% Section 2: Explicit Gradient And Hessian On The Optimum Face
for iterations=1:40
    s=rhs-C*q; disk=1-sum(q(1:2).^2); n=[q(1:2);0];
    gradient=C'*(1./s)+2*n/disk;
    hessian=C'*(C./s.^2)+diag([2 2 0])/disk+4*(n*n')/disk^2;
    [factor,flag]=chol(hessian); if flag~=0, return; end
    step=-(factor\(factor'\gradient)); decrement=-gradient'*step;
    if decrement<1e-12, accepted=true; break; end
    value=-sum(log(s))-log(disk); fraction=1;
    for search=1:40
        trial=q+fraction*step; st=rhs-C*trial; dt=1-sum(trial(1:2).^2);
        if all(st>0) && dt>0 && -sum(log(st))-log(dt)<=value-.01*fraction*decrement
            break;
        end
        fraction=fraction/2;
    end
    if search==40, return; end
    q=trial;
end
if accepted
    x(free)=[q(1:2);q(3)-origin*q(1:2)];
    if max(A*x-b)>1e-9*(1+norm(b,inf)) || norm(x(free(1:2)))>1
        x=original; accepted=false;
    end
end
end
