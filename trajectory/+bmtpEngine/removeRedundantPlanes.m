function planes=removeRedundantPlanes(planes,limits,reserve_units,useAffineOneSourceProof)
%% Section 0: Header & Readme
% SYNTAX: planes = bmtpEngine.removeRedundantPlanes(planes,limits,reserve_units,useAffineOneSourceProof)
% PURPOSE: Reduce solver rows without changing the unrelaxed feasible corridor.
% INPUTS: Active solver planes, validated workspace limits, trajectory reserve, and whether a
%   one-generator lifted proof may remove affine-normal planes.
% OUTPUTS: Plane array with provably implied halfspaces marked inactive. The
%   caller must still validate motion against every original source region.
% UNITS: Coordinate units; normals and combination weights are dimensionless.

%% Section 1: Include The Existing Workspace Bounds In The Implication Proof
if nargin<4, useAffineOneSourceProof=false; end
% Remove only halfspaces proved implied by other retained halfspaces.
% Fixed-clock refinement uses the historical two-generator proof for constant
% normals. Collision discovery uses one scale in the lifted endpoint space.
domain_units=[limits.xInterval_units;limits.yInterval_units];
coordinateBound_units=max(abs(domain_units),[],2).';
workspaceNormals=[1,0;-1,0;0,1;0,-1];
workspaceOffsets_units=repmat([-domain_units(1,2);domain_units(1,1); ...
    -domain_units(2,2);domain_units(2,1)],1,2);

%% Section 2: Remove Only Sequentially Redundant Affine Halfspaces
for span=1:size(planes,1)
    active=find([planes(span,:).Active]);
    % Bound this preprocessing for large local sets; they retain every row.
    if numel(active)<2 || numel(active)>64, continue; end
    if useAffineOneSourceProof
        count=numel(active);
        normals=zeros(count+8,4); offsets_units=zeros(count+8,2);
        for k=1:count
            normals(k,:)=reshape(planes(span,active(k)).Normal.',1,[]);
            offsets_units(k,:)=planes(span,active(k)).Offset_units+reserve_units;
        end
        % Workspace bounds apply independently to the two controls in each
        % Bernstein product row, so lift one copy into each plane endpoint.
        for k=1:4
            normals(count+k,1:2)=workspaceNormals(k,:);
            offsets_units(count+k,1)=workspaceOffsets_units(k,1);
            normals(count+4+k,3:4)=workspaceNormals(k,:);
            offsets_units(count+4+k,2)=workspaceOffsets_units(k,1);
        end
        kept=true(count+8,1);
        scale_units=max([1;abs(offsets_units(:));coordinateBound_units(:)]);
        for k=count:-1:1
            sources=find(kept); sources(sources==k)=[];
            denominator=sum(normals(sources,:).^2,2);
            lambda=(normals(sources,:)*normals(k,:).')./denominator;
            valid=isfinite(lambda) & lambda>=0 & denominator>realmin;
            sources=sources(valid); lambda=lambda(valid);
            residual=normals(k,:)-lambda.*normals(sources,:);
            guard_units=[abs(residual(:,1:2))*coordinateBound_units.', ...
                abs(residual(:,3:4))*coordinateBound_units.'];
            guard_units=guard_units+128*eps(scale_units)*(1+lambda);
            implied_units=lambda.*offsets_units(sources,:);
            if any(all(offsets_units(k,:)+guard_units<=implied_units,2))
                kept(k)=false; planes(span,active(k)).Active=false;
            end
        end
        continue;
    end
    if numel(active)<3, continue; end
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
