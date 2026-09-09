function planes=removeRedundantPlanes(planes,limits,reserve_units)
%% Section 0: Header & Readme
% SYNTAX: planes = bmtpEngine.removeRedundantPlanes(planes,limits,reserve_units)
% PURPOSE: Reduce solver rows without changing the unrelaxed feasible corridor.
% INPUTS: Active solver planes, validated workspace limits, trajectory reserve.
% OUTPUTS: Plane array with provably implied halfspaces marked inactive. The
%   caller must still validate motion against every original source region.
% UNITS: Coordinate units; normals and combination weights are dimensionless.

%% Section 1: Include The Existing Workspace Bounds In The Implication Proof
% Remove only halfspaces proved implied by other retained halfspaces.
% A nonnegative combination of two normals proves implication at both
% affine offset endpoints. The workspace bounds the roundoff residual.
domain_units=[limits.xInterval_units;limits.yInterval_units];
coordinateBound_units=max(abs(domain_units),[],2).';
workspaceNormals=[1,0;-1,0;0,1;0,-1];
workspaceOffsets_units=repmat([-domain_units(1,2);domain_units(1,1); ...
    -domain_units(2,2);domain_units(2,1)],1,2);

%% Section 2: Remove Only Sequentially Redundant Affine Halfspaces
for span=1:size(planes,1)
    active=find([planes(span,:).Active]);
    % Bound this preprocessing for large local sets; they retain every row.
    if numel(active)<3 || numel(active)>64, continue; end
    constant=arrayfun(@(p) isequal(p.Normal(1,:),p.Normal(2,:)),planes(span,active));
    active=active(constant); count=numel(active);
    normals=zeros(count,2); offsets_units=zeros(count,2);
    for k=1:count
        normals(k,:)=planes(span,active(k)).Normal(1,:);
        offsets_units(k,:)=planes(span,active(k)).Offset_units+reserve_units;
    end
    normals=[normals;workspaceNormals]; offsets_units=[offsets_units;workspaceOffsets_units];
    kept=true(count+4,1);
    scale_units=max([1;abs(offsets_units(:));coordinateBound_units(:)]);
    for k=count:-1:1
        others=find(kept); others(others==k)=[];
        [first,second]=find(triu(true(numel(others)),1));
        first=others(first); second=others(second);
        a=normals(first,:); b=normals(second,:); target=normals(k,:);
        determinant=a(:,1).*b(:,2)-a(:,2).*b(:,1);
        lambda=[(target(1)*b(:,2)-target(2)*b(:,1))./determinant, ...
            (a(:,1)*target(2)-a(:,2)*target(1))./determinant];
        valid=abs(determinant)>64*eps & all(isfinite(lambda) & lambda>=0,2);
        lambda=lambda(valid,:); first=first(valid); second=second(valid);
        defect=target-lambda(:,1).*normals(first,:)-lambda(:,2).*normals(second,:);
        guard_units=abs(defect)*coordinateBound_units.'+128*eps(scale_units)*(1+sum(lambda,2));
        implied_units=lambda(:,1).*offsets_units(first,:)+lambda(:,2).*offsets_units(second,:);
        if any(all(offsets_units(k,:)+guard_units<=implied_units,2))
            kept(k)=false; planes(span,active(k)).Active=false;
        end
    end
end
end
