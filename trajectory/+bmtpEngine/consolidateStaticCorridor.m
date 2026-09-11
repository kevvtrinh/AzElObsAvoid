function [planes,diagnostics] = consolidateStaticCorridor(sourcePlanes,regions_units,separatingLineGeometry,limits,target_units,reserve_units)
%% Section 0: Header & Readme
% SYNTAX: [planes,diagnostics] = bmtpEngine.consolidateStaticCorridor(
%   sourcePlanes,regions,separatingGeometry,limits,target,reserve)
% PURPOSE: Merge consecutive span-local static corridor cells into the
%   minimum sequence of collision-free convex supersets. Every merged cell
%   contains each source cell it replaces, so consolidation cannot remove a
%   trajectory feasible in the original fixed-plane subproblem.
% INPUTS: sourcePlanes (S-by-R struct) Static span-local separating planes.
%   regions_units (R-by-1 cell) Exact protected convex exclusion regions.
%   separatingLineGeometry (R-by-1 cell) Cached exact supporting directions.
%   limits (scalar struct) Workspace bounds. target_units and reserve_units
%   (nonnegative scalars) Obstacle- and trajectory-side separation amounts.
% OUTPUTS: planes (S-by-R struct) Shared verified plane grid. diagnostics
%   records exact group ranges and constraint counts.
% UNITS: Coordinate units; normals are dimensionless.

%% Section 1: Validate And Reconstruct Every Source Cell
assert(isstruct(sourcePlanes) && ismatrix(sourcePlanes), ...
    'bmtpEngine:InvalidCorridorPlanes','Source planes must be a two-dimensional struct array.');
assert(iscell(regions_units) && iscolumn(regions_units) && ...
    size(sourcePlanes,2)==numel(regions_units), ...
    'bmtpEngine:InvalidCorridorRegions','Static regions must match the plane columns.');
assert(iscell(separatingLineGeometry) && isequal(size(separatingLineGeometry),size(regions_units)), ...
    'bmtpEngine:InvalidCorridorGeometry','Separating geometry must match the source regions.');
validateattributes(target_units,{'numeric'},{'real','finite','scalar','nonnegative'});
validateattributes(reserve_units,{'numeric'},{'real','finite','scalar','nonnegative'});
segmentCount=size(sourcePlanes,1);
regionCount=numel(regions_units);
planes=sourcePlanes;
timer=tic;
if segmentCount==0 || regionCount==0
    ranges=zeros(0,2);
    if segmentCount>0, ranges=[1,segmentCount]; end
    diagnostics=createDiagnostics(true,ranges,nnz([sourcePlanes.Active]), ...
        nnz([sourcePlanes.Active]),0,toc(timer),"");
    return;
end

sourceCells=cell(segmentCount,1);
for segmentIndex=1:segmentCount
    sourceCells{segmentIndex}=intersectHalfspaces(sourcePlanes(segmentIndex,:),limits,reserve_units);
    if isempty(sourceCells{segmentIndex})
        diagnostics=createDiagnostics(false,zeros(0,2),nnz([sourcePlanes.Active]), ...
            nnz([sourcePlanes.Active]),0,toc(timer),"A source corridor cell was empty.");
        return;
    end
end

%% Section 2: Enumerate Collision-Free Convex Supersets
% The convex hull of a range contains every source cell in that range. Once
% its hull cannot be separated from a protected region, adding another cell
% cannot restore separability, so the range search stops exactly.
emptyPlane=createEmptyPlane();
rangeIsVerified=false(segmentCount);
rangePlaneCount=inf(segmentCount);
rangePlanes=cell(segmentCount);
for firstSpan=1:segmentCount
    points_units=zeros(0,2);
    for lastSpan=firstSpan:segmentCount
        points_units=[points_units;sourceCells{lastSpan}]; %#ok<AGROW>
        hull_units=convexHullVertices(points_units);
        candidatePlanes=createPlanes(hull_units,regions_units,separatingLineGeometry, ...
            target_units,reserve_units,emptyPlane);
        if ~all([candidatePlanes.Verified]), break; end
        candidatePlanes=bmtpEngine.removeRedundantPlanes(candidatePlanes,limits,reserve_units);
        rangeIsVerified(firstSpan,lastSpan)=true;
        rangePlaneCount(firstSpan,lastSpan)=nnz([candidatePlanes.Active]);
        rangePlanes{firstSpan,lastSpan}=candidatePlanes;
    end
end

%% Section 3: Minimize Cell Count And Then Solver Rows
bestGroupCount=inf(segmentCount+1,1);
bestSpanPlaneCount=inf(segmentCount+1,1);
bestGroupPlaneCount=inf(segmentCount+1,1);
previousFirst=zeros(segmentCount+1,1);
bestGroupCount(1)=0;
bestSpanPlaneCount(1)=0;
bestGroupPlaneCount(1)=0;
for lastSpan=1:segmentCount
    for firstSpan=1:lastSpan
        if ~rangeIsVerified(firstSpan,lastSpan), continue; end
        candidateGroupCount=bestGroupCount(firstSpan)+1;
        candidateSpanPlaneCount=bestSpanPlaneCount(firstSpan)+ ...
            (lastSpan-firstSpan+1)*rangePlaneCount(firstSpan,lastSpan);
        candidateGroupPlaneCount=bestGroupPlaneCount(firstSpan)+rangePlaneCount(firstSpan,lastSpan);
        if candidateGroupCount<bestGroupCount(lastSpan+1) || ...
                (candidateGroupCount==bestGroupCount(lastSpan+1) && ...
                (candidateSpanPlaneCount<bestSpanPlaneCount(lastSpan+1) || ...
                (candidateSpanPlaneCount==bestSpanPlaneCount(lastSpan+1) && ...
                candidateGroupPlaneCount<bestGroupPlaneCount(lastSpan+1))))
            bestGroupCount(lastSpan+1)=candidateGroupCount;
            bestSpanPlaneCount(lastSpan+1)=candidateSpanPlaneCount;
            bestGroupPlaneCount(lastSpan+1)=candidateGroupPlaneCount;
            previousFirst(lastSpan+1)=firstSpan;
        end
    end
end
if ~isfinite(bestGroupCount(end))
    diagnostics=createDiagnostics(false,zeros(0,2),nnz([sourcePlanes.Active]), ...
        nnz([sourcePlanes.Active]),0,toc(timer),"No complete verified merged-cell cover exists.");
    return;
end
ranges=zeros(bestGroupCount(end),2);
lastSpan=segmentCount;
for groupIndex=bestGroupCount(end):-1:1
    firstSpan=previousFirst(lastSpan+1);
    ranges(groupIndex,:)=[firstSpan,lastSpan];
    selectedPlanes=rangePlanes{firstSpan,lastSpan};
    planes(firstSpan:lastSpan,:)=repmat(selectedPlanes,lastSpan-firstSpan+1,1);
    lastSpan=firstSpan-1;
end

%% Section 4: Report The Exact Reduction
diagnostics=createDiagnostics(true,ranges,nnz([sourcePlanes.Active]), ...
    nnz([planes.Active]),bestGroupPlaneCount(end),toc(timer), ...
    "Every merged convex superset was verified against every protected source region.");
end

%% Section 5: Local Functions
function vertices_units=intersectHalfspaces(spanPlanes,limits,reserve_units)
    A=[1,0;-1,0;0,1;0,-1];
    b=[limits.xInterval_units(2);-limits.xInterval_units(1); ...
        limits.yInterval_units(2);-limits.yInterval_units(1)];
    for regionIndex=1:numel(spanPlanes)
        plane=spanPlanes(regionIndex);
        if ~plane.Active, continue; end
        if ~isequal(plane.Normal(1,:),plane.Normal(2,:)) || ...
                ~isequal(plane.Offset_units(1),plane.Offset_units(2))
            vertices_units=zeros(0,2);
            return;
        end
        A(end+1,:)=plane.Normal(1,:); %#ok<AGROW>
        b(end+1,1)=-reserve_units-plane.Offset_units(1); %#ok<AGROW>
    end
    scale_units=max([1;abs(b)]);
    roundoff_units=256*eps(scale_units);
    candidates=zeros(0,2);
    for firstRow=1:size(A,1)-1
        for secondRow=firstRow+1:size(A,1)
            pair=A([firstRow,secondRow],:);
            determinant=det(pair);
            if abs(determinant)<=64*eps, continue; end
            point_units=pair\b([firstRow,secondRow]);
            if all(A*point_units<=b+roundoff_units)
                candidates(end+1,:)=point_units.'; %#ok<AGROW>
            end
        end
    end
    candidates=uniquetol(candidates,128*eps(scale_units),'ByRows',true);
    if size(candidates,1)<3
        vertices_units=zeros(0,2);
        return;
    end
    vertices_units=convexHullVertices(candidates);
end

function planes=createPlanes(hull_units,regions_units,separatingLineGeometry,target_units,reserve_units,emptyPlane)
    regionCount=numel(regions_units);
    planes=repmat(emptyPlane,1,regionCount);
    for regionIndex=1:regionCount
        planes(regionIndex)=createStaticPlane(hull_units,regions_units{regionIndex}, ...
            separatingLineGeometry{regionIndex},target_units,reserve_units,emptyPlane);
    end
end

function plane=createStaticPlane(hull_units,region_units,geometry,target_units,reserve_units,emptyPlane)
    hullEdges_units=diff([hull_units;hull_units(1,:)],1,1);
    edgeLength_units=vecnorm(hullEdges_units,2,2);
    hullEdges_units=hullEdges_units(edgeLength_units>0,:);
    edgeLength_units=edgeLength_units(edgeLength_units>0);
    hullNormals=[-hullEdges_units(:,2),hullEdges_units(:,1)]./edgeLength_units;
    positiveNormals=[geometry.PositiveNormals;hullNormals];
    normals=[positiveNormals;-positiveNormals];
    plane=emptyPlane;
    if isempty(normals), return; end
    obstacleSupport_units=min(region_units*normals.',[],1);
    trajectorySupport_units=max(hull_units*normals.',[],1);
    [~,normalIndex]=max(obstacleSupport_units-trajectorySupport_units);
    plane.Active=true;
    plane.ExitFlag=1;
    plane.Normal=repmat(normals(normalIndex,:),2,1);
    plane.Offset_units=repmat(target_units-obstacleSupport_units(normalIndex),1,2);
    plane=bmtpEngine.verifySeparatingLine(plane,hull_units,region_units,reserve_units,target_units);
end

function hull_units=convexHullVertices(points_units)
    points_units=unique(points_units,'rows','stable');
    if size(points_units,1)<3 || rank(points_units(2:end,:)-points_units(1,:))<2
        hull_units=points_units;
        return;
    end
    hullIndex=convhull(points_units(:,1),points_units(:,2),'Simplify',true);
    hull_units=points_units(hullIndex(1:end-1),:);
end

function plane=createEmptyPlane()
    plane=struct('Active',false,'Verified',false,'ExitFlag',-2, ...
        'Normal',zeros(2,2),'Offset_units',zeros(1,2),'SignedGap_units',NaN);
end

function diagnostics=createDiagnostics(complete,ranges,originalCount,retainedCount,groupPlaneCount,elapsed_s,message)
    applied=complete && ~isempty(ranges) && size(ranges,1)<ranges(end,2);
    diagnostics=struct('CompleteVerifiedCover',logical(complete), ...
        'Applied',logical(applied),'GroupSpanIndex',ranges, ...
        'GroupCount',size(ranges,1),'OriginalSpanPlaneCount',originalCount, ...
        'RetainedSpanPlaneCount',retainedCount,'GroupPlaneCount',groupPlaneCount, ...
        'ElapsedTime_s',elapsed_s,'Message',string(message));
end
