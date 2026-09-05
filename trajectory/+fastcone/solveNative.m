function [x, fval, exitflag, output] = solveNative(f, cones, A, b, E, d, lb, ub, options)
%% Section 0: Header & Readme
% SYNTAX: [x,fval,exitflag,output] = fastcone.solveNative(f,cones,A,b,E,d,lb,ub,options)
% PURPOSE: Experimental primal-dual solver for HS's scalar and Lorentz cones.
% INPUTS: The nine positional arguments used by MATLAB coneprog.
% OUTPUTS: Candidate, objective, status (1 converged, 0 unresolved), diagnostics.
% UNITS: Identical to the input program. No motion model is changed.
% This prototype uses exact Jordan-block equations, not a general infeasibility
% certificate. Unresolved programs must never be interpreted as infeasible.

%% Section 1: Normalize The Affine Conic Program
timer = tic;
original = {f,cones,A,b,E,d,lb,ub,options};
if exist(fullfile(fileparts(mfilename('fullpath')),['core.' mexext]),'file')~=3
    x=[]; fval=NaN; exitflag=0;
    output=struct('iterations',0,'message', ...
        'Native fastcone kernel unavailable. Run fastcone.build to enable it.');
    return
end
n = numel(f); f = f(:);
if isempty(A), A = sparse(0,n); b = zeros(0,1); end
if isempty(E), E = sparse(0,n); d = zeros(0,1); end
if isempty(lb), lb = -inf(n,1); else, lb = lb(:); end
if isempty(ub), ub = inf(n,1); else, ub = ub(:); end
tol = options.ConstraintTolerance;
optTol = options.OptimalityTolerance;
maxIter = options.MaxIterations;
fixed = isfinite(lb) & lb == ub;
xFixed = zeros(n,1); xFixed(fixed) = lb(fixed);
free = find(~fixed); nf = numel(free);
cost = f(free); offset = f'*xFixed;
b = b(:)-A*xFixed; A = sparse(A(:,free));
d = d(:)-E*xFixed; E = sparse(E(:,free));
emptyEq = full(max(abs(E),[],2)) == 0;
if any(abs(d(emptyEq)) > tol)
    x = []; fval = NaN; exitflag = 0;
    output = struct('message','Inconsistent fixed equalities.','iterations',0);
    return
end
E(emptyEq,:) = []; d(emptyEq) = [];
I = speye(nf);
lower = isfinite(lb(free)); upper = isfinite(ub(free));
G = [A; -I(lower,:); I(upper,:)];
h = [b; -lb(free(lower)); ub(free(upper))];
rowScale = max(full(max(abs(G),[],2)), abs(h));
rowScale = max(rowScale,1e-12);
G = G./rowScale; h = h./rowScale;
linearCount = size(G,1);
% Extract every cone once, then apply each 3-row analytical profile in a batch.
coneCount=numel(cones); coneA={};
if coneCount>0, coneA={cones.A}; end
if any(cellfun('size',coneA,1)~=2)
    x=[]; fval=NaN; exitflag=0;
    output=struct('iterations',0,'message','Unsupported native cone dimension.', ...
        'nativeUnsupported',true,'totalTime_s',toc(timer));
    return
end
if coneCount==0
    originalConeG=sparse(0,n); originalConeH=zeros(0,1);
    C=sparse(0,nf); rhs=zeros(0,1); coneSizes=zeros(0,1);
else
    head=-sparse(horzcat(cones.d)'); tail=-sparse(vertcat(coneA{:}));
    packedG=[head;tail]; packedH=[-[cones.gamma]';-vertcat(cones.b)];
    order=reshape([1:coneCount;coneCount+(1:2:2*coneCount);coneCount+(2:2:2*coneCount)],[],1);
    originalConeG=packedG(order,:); originalConeH=full(packedH(order));
    C=originalConeG(:,free); rhs=originalConeH-originalConeG*xFixed;
    nonzero=any(reshape(full(any(C,2)),3,[]),1)';
    blocks=reshape(rhs,3,[]);
    constantSatisfied=~nonzero & blocks(1,:)'>=hypot(blocks(2,:)',blocks(3,:)');
    scales=max(reshape(max(full(max(abs(C),[],2)),abs(rhs)),3,[]),[],1)';
    scales=max(scales,1e-12); scales=repelem(scales,3); scales=scales(:);
    keepRows=repelem(~constantSatisfied,3); keepRows=keepRows(:);
    C=C(keepRows,:)./scales(keepRows); rhs=rhs(keepRows)./scales(keepRows);
    coneSizes=3*ones(nnz(~constantSatisfied),1);
end
G=[G;C]; h=[h;rhs];
% Eliminate the exact endpoint/continuity equalities before Newton iteration.
% The pivot equations are solved, not penalized, so their physical meaning is
% unchanged. This also removes the indefinite KKT block from every iteration.
base = zeros(nf,1); basis = speye(nf);
sourceIndex=free; analyticalEquality=false;
if ~isempty(E)
    if numel(cones)>=2
        firstTimeIndex=find(cones(1).d,1,'first');
        if ~isempty(firstTimeIndex)
            controlCount=firstTimeIndex-1;
            segments=1+(size(original{5},1)-13)/8;
            degree=controlCount/(2*segments)-1;
            if controlCount>=1 && ~any(fixed(1:controlCount))
                [base,basis,independent,analyticalEquality]=fastcone.hsEqualityBasis( ...
                    E,d,controlCount,segments,degree);
            end
        end
    end
    if ~analyticalEquality
        [~,R,permutation] = qr(full(E),'vector');
        diagonal=diag(R(1:min(size(R)),1:min(size(R))));
        rankE = nnz(abs(diagonal) > 1e-11*max(1,norm(R,inf)));
        if rankE ~= size(E,1)
            error('fastcone:DependentEqualities','Equality rows need rank reduction.');
        end
        dependent = permutation(1:rankE); independent = permutation(rankE+1:end);
        pivot = E(:,dependent); base=zeros(nf,1);
        base(dependent) = pivot\d;
        basis = sparse(nf,numel(independent));
        basis(independent,:) = speye(numel(independent));
        basis(dependent,:) = -(pivot\E(:,independent));
    end
    h = h-G*base; G = G*basis; cost = basis'*cost;
    sourceIndex=free(independent);
    nf = numel(independent); E = sparse(0,nf); d = zeros(0,1);
end
objectiveConstant=offset+f(free)'*base;
% A constant norm is an affine inequality. A one-component norm is two
% affine inequalities. These exact profiles include fixed endpoint travel
% edges and avoid imposing a nonexistent strict interior on a redundant cone.
heads=(linearCount+1:3:size(G,1))'; first=heads+1; second=heads+2;
firstNonzero=full(any(G(first,:),2)); secondNonzero=full(any(G(second,:),2));
firstActive=firstNonzero | h(first)~=0; secondActive=secondNonzero | h(second)~=0;
zero=~firstActive & ~secondActive; one=xor(firstActive,secondActive);
constantBoth=firstActive & secondActive & ~firstNonzero & ~secondNonzero;
keep=~(zero | one | constantBoth);
counts=double(zero | constantBoth)+2*double(one); offsets=cumsum(counts)-counts;
extraG=sparse(sum(counts),nf); extraH=zeros(sum(counts),1);
simple=zero | constantBoth; target=offsets(simple)+1;
extraG(target,:)=G(heads(simple),:); extraH(target)=h(heads(simple));
constantTarget=offsets(constantBoth)+1;
extraH(constantTarget)=extraH(constantTarget)-hypot(h(first(constantBoth)),h(second(constantBoth)));
selectedTail=first(one)+double(~firstActive(one)); plus=offsets(one)+1; minus=plus+1;
extraG(plus,:)=G(heads(one),:)+G(selectedTail,:); extraH(plus)=h(heads(one))+h(selectedTail);
extraG(minus,:)=G(heads(one),:)-G(selectedTail,:); extraH(minus)=h(heads(one))-h(selectedTail);
newG=[G(1:linearCount,:);extraG]; newH=[h(1:linearCount);extraH];
constantRows=full(max(abs(newG),[],2))==0;
if any(newH(constantRows)<0)
    x=[]; fval=NaN; exitflag=0;
    output=struct('iterations',0,'message','An eliminated constant inequality has a negative bound.'); return
end
keptRows=reshape([heads(keep)';first(keep)';second(keep)'],[],1);
keptG=G(keptRows,:); keptH=h(keptRows);
newG(constantRows,:)=[]; newH(constantRows)=[];
linearCount=size(newG,1); coneSizes=3*ones(nnz(keep),1);
G=[newG;keptG]; h=[newH;keptH];
% Scale columns without changing any conic inequality.
columnScale = 1./max(full(max(abs([G;E]),[],1))',1e-8);
G = G.*columnScale'; E = E.*columnScale'; cost = cost.*columnScale;
costScale = max(norm(cost,inf),1e-8); cost = cost/costScale;
reducedLower=lb(sourceIndex)./columnScale;
reducedUpper=ub(sourceIndex)./columnScale;
% A single affine row can prove exact infeasibility from the inherited box.
% Return unresolved for the adapter: coneprog may still accept a near-boundary
% program at its scaled numerical tolerance, which remains visible recovery.
positive=max(G(1:linearCount,:),0); negative=min(G(1:linearCount,:),0);
finiteLower=reducedLower; finiteLower(~isfinite(finiteLower))=0;
finiteUpper=reducedUpper; finiteUpper(~isfinite(finiteUpper))=0;
minimumRows=positive*finiteLower+negative*finiteUpper;
unboundedRows=positive*double(isinf(reducedLower))>0 | ...
    (-negative)*double(isinf(reducedUpper))>0;
minimumRows(unboundedRows)=-inf;
roundoff=128*eps.*max(1,abs(positive)*abs(finiteLower)+abs(negative)*abs(finiteUpper)+abs(h(1:linearCount)));
if any(minimumRows-roundoff>h(1:linearCount))
    x=[]; fval=NaN; exitflag=0;
    output=struct('iterations',0,'message','A linear row conflicts with inherited variable bounds.');
    return
end
if ~isempty(E)
    eqScale = max(full(max(abs(E),[],2)),1e-12);
    E = E./eqScale; d = d./eqScale;
end
% Offsets of the exact Lorentz Jordan blocks; all other blocks are scalar.
starts = linearCount + 1 + [0; cumsum(coneSizes(1:end-1))];
starts = starts(1:numel(coneSizes));
m = numel(h); ne = size(E,1);
identity = [ones(linearCount,1); zeros(m-linearCount,1)];
identity(starts) = 1;
blockCount = linearCount + numel(coneSizes);
z = identity; s = identity;
xReduced = zeros(nf,1); y = zeros(ne,1);
setupTime = toc(timer);
exitflag = 0; message = 'Iteration limit or numerical failure.';
primal = inf; dual = inf; gap = inf;
certifiedLower=-inf; certifiedGap=inf; boundCertificate=struct();
originalResidual=inf;

%% Section 2: Native Evaluation Of The Same Analytical Cone Equations
% Independently recheck the native certificate in the original MATLAB inputs.
% Unsupported cone dimensions return unresolved for explicit coneprog recovery.
physicalBase=xFixed; physicalBase(free)=base;
physicalMap=sparse(n,nf); physicalMap(free,:)=basis.*columnScale';
originalA=original{3}; originalE=original{5};
if isempty(originalA), originalA=sparse(0,n); end
if isempty(originalE), originalE=sparse(0,n); end
validation=struct('A',sparse(originalA),'E',sparse(originalE), ...
    'Map',physicalMap,'Base',full(physicalBase), ...
    'B',full(original{4}(:)),'D',full(original{6}(:)), ...
    'Lower',full(lb),'Upper',full(ub), ...
    'ConeG',sparse(originalConeG),'ConeH',full(originalConeH));
setupTime=toc(timer);
[xReduced,z,stats]=fastcone.core(G,full(h),full(cost), ...
    full(reducedLower),full(reducedUpper),double(linearCount),double(numel(coneSizes)), ...
    tol,optTol,double(maxIter),objectiveConstant,costScale,validation);
x=physicalBase+physicalMap*xReduced; fval=f'*x;
exitflag=stats(2);
originalResidual=fastcone.residual(x,original{:});
[bound,boundCertificate]=fastcone.dualBoxBound(cost,G,h,z, ...
    reducedLower,reducedUpper,linearCount,coneSizes);
certifiedLower=objectiveConstant+costScale*bound;
certifiedGap=fval-certifiedLower;
if exitflag==1 && (~isfinite(originalResidual) || originalResidual>tol || ...
        ~isfinite(certifiedGap) || abs(certifiedGap)>optTol*(1+abs(fval)))
    exitflag=0; message='Independent MATLAB validation rejected the native result.';
elseif exitflag==1
    message='Original primal residual and independent MATLAB dual objective bound passed.';
else
    message='Native iteration limit or numerical failure; unresolved.';
end
output=struct('iterations',stats(1),'primalfeasibility',stats(3), ...
    'dualfeasibility',stats(4),'dualitygap',stats(5),'message',message, ...
    'algorithm','experimental MEX analytical Jordan-block predictor-corrector', ...
    'setupTime_s',setupTime,'totalTime_s',toc(timer), ...
    'linearCount',linearCount,'coneCount',numel(coneSizes),'variableCount',nf, ...
    'objectiveOffset',offset,'originalPrimalResidual',originalResidual, ...
    'analyticalEquality',analyticalEquality,'certifiedLowerBound',certifiedLower, ...
    'certifiedObjectiveGap',certifiedGap,'boundCertificate',boundCertificate, ...
    'nativeTime_s',stats(11),'nativeAnalyses',stats(9), ...
    'nativeRegularizedFactors',stats(10),'nativeStats',stats,'nativeUnsupported',false);
end
