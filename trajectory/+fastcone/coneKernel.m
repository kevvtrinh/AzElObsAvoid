function [x, z, stats] = coneKernel(G, h, cost, lb, ub, ...
        linearCount, coneCount, tolerance, optimalityTolerance, ...
        maximumIterations, constant, costScale, validation)
%% Section 0: Header & Readme
% SYNTAX: [x,z,stats] = fastcone.coneKernel(G,h,c,lb,ub,nl,nc,tol,optTol,maxIter,k,scale,validation)
% PURPOSE: MATLAB predictor-corrector for prepared scalar and 3D Lorentz blocks.
% INPUTS: Scaled conic program, inherited box, tolerances and physical map.
% OUTPUTS: Primal/dual candidates and numerical convergence diagnostics.
% UNITS: Scaled internal coordinates; physical checks use the supplied map.
% Internal entry point: unresolved status does not establish infeasibility.

%% Section 1: Prepare Sparse Products And Iterate Cone Equations

timer = tic;
[rowCount, variableCount] = size(G);
assert(rowCount == linearCount + 3 * coneCount && numel(h)==rowCount && ...
    numel(cost)==variableCount && numel(lb)==variableCount && numel(ub)==variableCount, ...
    'fastcone:KernelDimensions','Prepared conic dimensions must agree.');
iteration=0;
transposeG = G';
identity = [ones(linearCount, 1); repmat([1; 0; 0], coneCount, 1)];
s = identity;
z = identity;
x = zeros(variableCount, 1);
[gramMap, gramRows, gramColumns, diagonalSlots, order] = prepareGram(G, linearCount, coneCount);
inverseOrder=zeros(variableCount,1); inverseOrder(order)=1:variableCount;
orderedRows=max(inverseOrder(gramRows),inverseOrder(gramColumns));
orderedColumns=min(inverseOrder(gramRows),inverseOrder(gramColumns));
regularizedCount = 0;
flag = 0;
primal = Inf; dual = Inf; gap = Inf;
lowerBound = -Inf; objectiveGap = Inf; originalResidual = Inf;
infeasibilityMargin=0;
finiteBox=all(isfinite(lb) & isfinite(ub));
if finiteBox
    rayErrorWeights=abs(h)+abs(G)*max(abs(lb),abs(ub));
    rayErrorFactor=(rowCount+variableCount+8)*eps;
end
for iteration = 1:maximumIterations
    residualG = G * x + s - h;
    residualDual = cost + transposeG * z;
    gap = s' * z;
    primal = norm(residualG, Inf) / (1 + norm(h, Inf));
    dual = norm(residualDual, Inf) / (1 + norm(cost, Inf));
    relativeGap = gap / (1 + abs(cost' * x));
    % A growing gap only schedules a proof check; it never proves failure.
    if finiteBox && gap>linearCount+coneCount
        ray=z/max(1,norm(z,Inf)); ray(1:linearCount)=max(0,ray(1:linearCount));
        for head=linearCount+1:3:rowCount
            radius=hypot(ray(head+1),ray(head+2));
            ray(head)=max(ray(head),radius)+4*eps(max(1,radius));
        end
        rayBound=fastcone.dualBoxBound(zeros(variableCount,1),G,h,ray,lb,ub, ...
            linearCount,3*ones(coneCount,1));
        roundoff=rayErrorFactor*max(1,rayErrorWeights'*abs(ray));
        if rayBound>roundoff
            infeasibilityMargin=rayBound-roundoff;
            break;
        end
    end
    if primal <= tolerance * 0.1 && relativeGap <= sqrt(optimalityTolerance)
        bound = fastcone.dualBoxBound(cost, G, h, z, lb, ub, ...
            linearCount, 3 * ones(coneCount, 1));
        lowerBound = constant + costScale * bound;
        value = constant + costScale * (cost' * x);
        objectiveGap = value - lowerBound;
        if abs(objectiveGap) <= optimalityTolerance * (1 + abs(value))
            originalResidual = physicalResidual(x, validation);
            if originalResidual <= tolerance
                flag = 1;
                break;
            end
        end
    end
    mu = gap / max(1, linearCount + coneCount);
    [weights, inverse, direct] = scalingBlocks(s, z, linearCount);
    W = weights; inverseD = inverse; D = direct;
    v = blockMultiply(inverseD, s, linearCount);
    gramValues=gramMap*weights;
    equilibration = 1 ./ sqrt(max(realmin, gramValues(diagonalSlots)));
    gramValues=gramValues.*equilibration(gramRows).*equilibration(gramColumns);
    if any(~isfinite(gramValues))
        break;
    end
    orderedK=sparse(orderedRows,orderedColumns,gramValues,variableCount,variableCount);
    factored = false;
    for regularization = [0, 1e-14, 1e-12, 1e-10, 1e-8]
        [factor, status] = chol(orderedK + ...
            regularization * speye(variableCount), 'lower');
        if status == 0
            factored = true;
            regularizedCount = regularizedCount + (regularization > 0);
            break;
        end
    end
    if ~factored
        break;
    end
    coneRight = -z + blockMultiply(W,residualG,linearCount);
    [dx, ds, dz] = direction(coneRight, W, ...
        residualG, residualDual, G, transposeG, equilibration, order, factor, linearCount);
    if any(~isfinite([dx; ds; dz]))
        break;
    end
    primalStep = boundary(s, ds, linearCount);
    dualStep = boundary(z, dz, linearCount);
    affineMu = (s + primalStep * ds)' * (z + dualStep * dz) / ...
        max(1, linearCount + coneCount);
    sigma = min(1, max(0, max(0, affineMu / mu)^3));
    correction=centeringCorrection(s,ds,dz,v,inverseD,D,sigma*mu,linearCount);
    [dx, ds, dz] = direction(coneRight+correction, W, ...
        residualG, residualDual, G, transposeG, equilibration, order, factor, linearCount);
    primalStep = min(1, 0.995 * boundary(s, ds, linearCount));
    dualStep = min(1, 0.995 * boundary(z, dz, linearCount));
    if any(~isfinite([dx; ds; dz])) || min(primalStep, dualStep) < 1e-14
        break;
    end
    x = x + primalStep * dx;
    s = s + primalStep * ds;
    z = z + dualStep * dz;
end
if flag ~= 1
    originalResidual = physicalResidual(x, validation);
end
stats = [iteration; flag; primal; dual; gap; lowerBound; objectiveGap; ...
    originalResidual; 1; regularizedCount; toc(timer); variableCount; nnz(G); infeasibilityMargin];
end

function [map, rows, columns, diagonal, order] = prepareGram(G,nl,nc)
% Compile fixed coefficients of the lower triangle of G'*W*G once per solve.
n=size(G,2);
[variables,~,values]=find(G');
persistent cache
inputPattern=spones(G);
if ~isempty(cache) && cache.LinearCount==nl && cache.ConeCount==nc && isequal(inputPattern,cache.Pattern)
    coefficients=values(cache.Left).*values(cache.Right); coefficients(1:n)=0;
    map=sparse(cache.Destination,cache.Weight,coefficients,numel(cache.Rows),nl+9*nc);
    rows=cache.Rows; columns=cache.Columns; diagonal=cache.Diagonal; order=cache.Order;
    return;
end
counts=full(sum(spones(G),2)); starts=cumsum([1;counts]);
widths=unique(counts(1:nl));
keyParts=cell(numel(widths)+9*nc+1,1); weightParts=keyParts; coefficientParts=keyParts;
leftParts=keyParts; rightParts=keyParts;
part=1; keyParts{part}=(1:n)'*(n+1)-n;
weightParts{part}=ones(n,1); coefficientParts{part}=zeros(n,1);
leftParts{part}=ones(n,1); rightParts{part}=ones(n,1);
for width=widths'
    if width==0, continue; end
    selected=find(counts(1:nl)==width);
    slots=starts(selected)'+(0:width-1)';
    localVariables=reshape(variables(slots),size(slots));
    localValues=reshape(values(slots),size(slots));
    [left,right]=find(tril(ones(width)));
    localKeys=localVariables(left,:)+(localVariables(right,:)-1)*n;
    localCoefficients=localValues(left,:).*localValues(right,:);
    localWeight=repmat(selected',numel(left),1);
    part=part+1; keyParts{part}=localKeys(:); weightParts{part}=localWeight(:);
    coefficientParts{part}=localCoefficients(:);
    leftSlots=slots(left,:); rightSlots=slots(right,:);
    leftParts{part}=leftSlots(:); rightParts{part}=rightSlots(:);
end
% Batch equal-width row products across all Lorentz blocks. Each weight is
% still one exact entry of a 3-by-3 scaling block; no matrix term is dropped.
coneRows=reshape(nl+(1:3*nc),3,nc);
leftRows=reshape(coneRows([1 2 3 1 2 3 1 2 3],:),[],1);
rightRows=reshape(coneRows([1 1 1 2 2 2 3 3 3],:),[],1);
widthPairs=unique([counts(leftRows),counts(rightRows)],'rows');
for group=1:size(widthPairs,1)
    aWidth=widthPairs(group,1); bWidth=widthPairs(group,2);
    if aWidth==0 || bWidth==0, continue; end
    selected=find(counts(leftRows)==aWidth & counts(rightRows)==bWidth);
    [aOffset,bOffset]=ndgrid(0:aWidth-1,0:bWidth-1);
    ai=starts(leftRows(selected))'+aOffset(:);
    bi=starts(rightRows(selected))'+bOffset(:);
    blockWeight=repmat(nl+selected',numel(aOffset),1);
    ai=ai(:); bi=bi(:); blockWeight=blockWeight(:);
    keep=variables(ai)>=variables(bi); ai=ai(keep); bi=bi(keep);
    part=part+1; keyParts{part}=variables(ai)+(variables(bi)-1)*n;
    weightParts{part}=blockWeight(keep);
    coefficientParts{part}=values(ai).*values(bi);
    leftParts{part}=ai; rightParts{part}=bi;
end
keys=vertcat(keyParts{:}); weight=vertcat(weightParts{:}); coefficients=vertcat(coefficientParts{:});
sourceRows=mod(keys-1,n)+1; sourceColumns=floor((keys-1)/n)+1;
pattern=sparse(sourceRows,sourceColumns,1,n,n);
[rows,columns]=find(pattern); keys=rows+(columns-1)*n;
lookup=sparse(rows,columns,1:numel(rows),n,n);
destination=full(lookup(sourceRows+(sourceColumns-1)*n));
map=sparse(destination,weight,coefficients,numel(keys),nl+9*nc);
rows=mod(keys-1,n)+1; columns=floor((keys-1)/n)+1;
diagonal=find(rows==columns);
order=symamd(pattern+pattern');
cache=struct('LinearCount',nl,'ConeCount',nc,'Pattern',inputPattern, ...
    'Left',vertcat(leftParts{:}),'Right',vertcat(rightParts{:}), ...
    'Destination',destination,'Weight',weight,'Rows',rows,'Columns',columns, ...
    'Diagonal',diagonal,'Order',order);
end

function [dx, ds, dz] = direction(coneRight, W, rg, rd, G, GT, ...
        equilibration, order, factor, linearCount)
% Reuse one Cholesky factor for predictor and corrector right-hand sides.
right = equilibration .* (-rd - GT * coneRight);
dx = zeros(size(right));
dx(order) = factor' \ (factor \ right(order));
dx = equilibration .* dx;
gx = G * dx;
ds = -rg - gx;
dz = coneRight + blockMultiply(W, gx, linearCount);
end

function correction=centeringCorrection(s,ds,dz,v,inverse,D,target,nl)
correction=zeros(size(s));
correction(1:nl)=(target-ds(1:nl).*dz(1:nl))./s(1:nl);
if numel(s)>nl
    a=blockMultiply(inverse(nl+1:end),ds(nl+1:end),0);
    b=blockMultiply(D(nl+1:end),dz(nl+1:end),0);
    rhs=-jordan(a,b,0); rhs(1:3:end)=rhs(1:3:end)+target;
    correction(nl+1:end)=blockMultiply(inverse(nl+1:end), ...
        jordanInverse(v(nl+1:end),rhs,0),0);
end
end

function out=blockMultiply(weights,value,nl)
out=zeros(size(value)); out(1:nl)=weights(1:nl).*value(1:nl);
for row=nl+1:3:numel(value)
    k=nl+3*(row-nl-1);
    a=value(row); b=value(row+1); c=value(row+2);
    out(row)=weights(k+1)*a+weights(k+4)*b+weights(k+7)*c;
    out(row+1)=weights(k+2)*a+weights(k+5)*b+weights(k+8)*c;
    out(row+2)=weights(k+3)*a+weights(k+6)*b+weights(k+9)*c;
end
end

function out = jordan(a, b, linearCount)
out=a.*b;
for k=linearCount+1:3:numel(a)
    out(k)=a(k)*b(k)+a(k+1)*b(k+1)+a(k+2)*b(k+2);
    out(k+1)=a(k)*b(k+1)+b(k)*a(k+1);
    out(k+2)=a(k)*b(k+2)+b(k)*a(k+2);
end
end

function out = jordanInverse(a, b, linearCount)
% Solve the Lorentz arrow matrix analytically, preserving its determinant.
out=b./a;
for k=linearCount+1:3:numel(a)
    head=(a(k)*b(k)-a(k+1)*b(k+1)-a(k+2)*b(k+2))/ ...
        (a(k)^2-a(k+1)^2-a(k+2)^2);
    out(k)=head;
    out(k+1)=(b(k+1)-a(k+1)*head)/a(k);
    out(k+2)=(b(k+2)-a(k+2)*head)/a(k);
end
end

function alpha = boundary(a, b, linearCount)
% Use the first positive cone-boundary root, with a stable quadratic formula.
alpha = 1;
for k=1:linearCount
    if b(k)<0, alpha=min(alpha,-a(k)/b(k)); end
end
for k=linearCount+1:3:numel(a)
    t=a(k)+alpha*b(k); u=a(k+1)+alpha*b(k+1); v=a(k+2)+alpha*b(k+2);
    if t>=sqrt(u*u+v*v), continue; end
    qa=b(k)^2-b(k+1)^2-b(k+2)^2;
    qb=2*(a(k)*b(k)-a(k+1)*b(k+1)-a(k+2)*b(k+2));
    qc=a(k)^2-a(k+1)^2-a(k+2)^2;
    if abs(qa)<eps*max(1,abs(qb))
        r=-qc/qb;
        if r>0 && isfinite(r), alpha=min(alpha,r); end
    else
        directionSign=1; if qb<0, directionSign=-1; end
        q=-0.5*(qb+directionSign*sqrt(max(0,qb*qb-4*qa*qc)));
        r=q/qa; if r>0 && isfinite(r), alpha=min(alpha,r); end
        r=qc/q; if r>0 && isfinite(r), alpha=min(alpha,r); end
    end
    if b(k)<0, alpha=min(alpha,-a(k)/b(k)); end
end
alpha = max(0, alpha);
end

function [weights, inverse, direct] = scalingBlocks(s, z, nl)
% Scalar expressions avoid allocating many 3-by-cone-count intermediates.
nc=(numel(s)-nl)/3;
weights=zeros(nl+9*nc,1); inverse=weights; direct=weights;
ratio=z(1:nl)./s(1:nl); weights(1:nl)=ratio;
inverse(1:nl)=sqrt(ratio); direct(1:nl)=1./sqrt(ratio);
for cone=1:nc
    row=nl+3*(cone-1); slot=nl+9*(cone-1);
    da=sqrt(max(realmin,s(row+1)^2-s(row+2)^2-s(row+3)^2));
    db=sqrt(max(realmin,z(row+1)^2-z(row+2)^2-z(row+3)^2));
    a=s(row+1)/da; b=s(row+2)/da; c=s(row+3)/da;
    d=z(row+1)/db; e=z(row+2)/db; f=z(row+3)/db;
    determinant=da/db; root=sqrt(determinant);
    multiplier=root/sqrt(2*(1+a*d+b*e+c*f));
    wt=(a+d)*multiplier; wx=(b-e)*multiplier; wy=(c-f)*multiplier;
    denominator=determinant*(wt+root); squared=determinant^2;
    inverse(slot+(1:9))=[wt/determinant;-wx/determinant;-wy/determinant; ...
        -wx/determinant;1/root+wx^2/denominator;wx*wy/denominator; ...
        -wy/determinant;wx*wy/denominator;1/root+wy^2/denominator];
    direct(slot+(1:9))=[wt;wx;wy;wx;root+wx^2/(wt+root);wx*wy/(wt+root); ...
        wy;wx*wy/(wt+root);root+wy^2/(wt+root)];
    weights(slot+(1:9))=[(wt^2+wx^2+wy^2)/squared;-2*wt*wx/squared; ...
        -2*wt*wy/squared;-2*wt*wx/squared;1/determinant+2*wx^2/squared; ...
        2*wx*wy/squared;-2*wt*wy/squared;2*wx*wy/squared;1/determinant+2*wy^2/squared];
end
end

function value = physicalResidual(q, validation)
% Evaluate feasibility in the original physical coordinates and cone scaling.
x = validation.Base + validation.Map * q;
if any(~isfinite(x))
    value = Inf;
    return;
end
value = max([0; validation.A * x - validation.B; ...
    abs(validation.E * x - validation.D); validation.Lower - x; x - validation.Upper]);
blocks = reshape(validation.ConeH - validation.ConeG * x, 3, []);
if ~isempty(blocks)
    value = max(value, max(hypot(blocks(2, :), blocks(3, :)) - blocks(1, :)));
end
end
