function [supported, shape, regions_units, counts, timing_s] = createSweptCorrespondingCells(lower_units, upper_units, safetyMargin_units, usesSourceIndex)
%% Section 0: Header & Readme
% SYNTAX: [supported,shape,regions_units,counts,timing_s] =
%   obstacleAvoidance.obstacles.createSweptCorrespondingCells(lower_units,upper_units,margin_units)
%   ... = createSweptCorrespondingCells(lower_units,upper_units,margin_units,usesSourceIndex)
% PURPOSE: Conservatively enclose a corresponding original ring over one interval.
%   Every triangle of the lower sample's constrained triangulation is carried
%   to the upper sample by vertex correspondence. A point carried by a
%   triangle's barycentric map is (1-tau)*a+tau*b with a in the start
%   triangle and b in the end triangle, hence lies in
%   conv(startTriangle union endTriangle). The union of those hulls covers
%   the image of the piecewise-linear map, and therefore the interpolated
%   polygon whenever its ring is simple. The margin is applied exactly once
%   as the four corners of an axis-aligned square of half-width
%   sqrt(2)*safetyMargin_units at every hull vertex: a square-join buffer of
%   distance d (the constructor's protection rule) reaches at most d*sqrt(2)
%   from any source point, so that square contains it. The union is then
%   covered at the interval's own resolution: it is clipped to grid squares
%   whose side is the largest vertex displacement of the interval (plus the
%   margin square), and each clipped piece is replaced by the convex hull of
%   its vertices. Each hull contains its piece and lies inside its square,
%   so the cover contains the union, fills only concavities narrower than a
%   displacement the sweep has already blurred, and bounds the cell count by
%   the swept extent over that displacement instead of by coastline detail,
%   and a declared budget of grid squares per interval bounds it further.
%   The union of the cover pieces is the interval's enclosure; that union is
%   then repartitioned exactly by the shared longest-shared-edge-first convex
%   merge, which keeps the enclosed point set and returns fewer cells than
%   the grid pieces themselves.
% INPUTS: Original single rings with equal vertex counts and the nonnegative
%   absolute safety margin. usesSourceIndex (logical, default false) keeps
%   the supplied index correspondence; otherwise the shared circular
%   alignment rule reorders the upper ring.
% OUTPUTS: supported (logical) false when the rings do not correspond or the
%   triangulation introduces non-source vertices. shape (polyshape) Static
%   enclosure of the interval, equal to the union of the returned cells.
%   regions_units (cell) Convex cells whose union is the enclosure. counts
%   (1-by-2) Triangle count and final cell count. timing_s (1-by-4)
%   Triangulation, hull, union, and cover wall times.
% UNITS: Coordinates and margin are coordinate units; timing is seconds.

%% Section 1: Require Source Correspondence
validateattributes(safetyMargin_units,{'numeric'},{'real','finite','scalar','nonnegative'});
if nargin < 4, usesSourceIndex = false; end
supported = false; shape = polyshape(); regions_units = cell(0,1);
counts = [0,0]; timing_s = zeros(1,4);
if size(lower_units,1)<3 || ~isequal(size(lower_units),size(upper_units)) || ...
        size(lower_units,2)~=2 || ~all(isfinite([lower_units;upper_units]),'all')
    return;
end
if size(unique(lower_units,'rows'),1)~=size(lower_units,1) || ...
        size(unique(upper_units,'rows'),1)~=size(upper_units,1)
    return;
end
if ~usesSourceIndex
    upper_units = obstacleAvoidance.obstacles.alignCorrespondingRing(lower_units,upper_units);
end
lowerShape = obstacleAvoidance.geometry.boundaryToShape(lower_units(:,1),lower_units(:,2));
if isempty(lowerShape.Vertices) || area(lowerShape) <= 0
    return;
end

%% Section 2: Triangulate The Original Sample On Its Source Vertices
% Triangles give the tightest hulls: a merged convex face would sweep the
% hull of a larger vertex set. Every source vertex must appear so the
% piecewise-linear map covers the complete interpolated boundary.
stageTimer = tic;
mesh = triangulation(lowerShape);
[found,sourceIndices] = ismember(mesh.Points,lower_units,'rows');
if ~all(found) || numel(unique(sourceIndices))~=size(lower_units,1)
    return;
end
faceIndices = sourceIndices(mesh.ConnectivityList);
timing_s(1) = toc(stageTimer);
counts(1) = size(faceIndices,1);
if counts(1) == 0, return; end

%% Section 3: Hull Every Carried Triangle With Its Margin Squares
stageTimer = tic;
halfWidth_units = sqrt(2)*safetyMargin_units;
marginCorners_units = halfWidth_units*[-1,-1;-1,1;1,1;1,-1];
sweeps(1,counts(1)) = polyshape();
for faceIndex = 1:counts(1)
    indices = faceIndices(faceIndex,:);
    vertices_units = [lower_units(indices,:);upper_units(indices,:)];
    if safetyMargin_units > 0
        vertices_units = reshape(permute( ...
            vertices_units+permute(marginCorners_units,[3,2,1]),[1,3,2]),[],2);
    end
    hullIndex = convhull(vertices_units(:,1),vertices_units(:,2));
    sweeps(faceIndex) = polyshape(vertices_units(hullIndex(1:end-1),:), ...
        'Simplify',false,'KeepCollinearPoints',true);
end
timing_s(2) = toc(stageTimer);

%% Section 4: Union The Sweeps
stageTimer = tic;
sweptUnion = balancedUnion(sweeps);
timing_s(3) = toc(stageTimer);
if isempty(sweptUnion.Vertices)
    return;
end

%% Section 5: Cover The Union At The Interval's Displacement Scale
stageTimer = tic;
% The cover is never finer than the sweep's own blur, and never finer than
% a declared budget of grid squares per interval: a coarser cover is only
% more conservative, so the budget trades routes for tractability, never
% validity. It mirrors the visibility search's pair-work budget.
cellBudget = 256;
extent_units = max(sweptUnion.Vertices,[],1)-min(sweptUnion.Vertices,[],1);
resolution_units = max([max(vecnorm(upper_units-lower_units,2,2)) + 2*halfWidth_units, ...
    sqrt(prod(max(extent_units,eps))/cellBudget)]);
if resolution_units <= 0
    % No vertex moved and no margin applies: the union is the sample itself.
    shape = sweptUnion;
    regions_units = obstacleAvoidance.geometry.convexRegions(shape,true);
else
    minimum_units = min(sweptUnion.Vertices,[],1);
    maximum_units = max(sweptUnion.Vertices,[],1);
    columnCount = max(1,ceil((maximum_units(1)-minimum_units(1))/resolution_units));
    rowCount = max(1,ceil((maximum_units(2)-minimum_units(2))/resolution_units));
    regions_units = cell(0,1);
    for columnIndex = 1:columnCount
        xRange_units = minimum_units(1)+[columnIndex-1,columnIndex]*resolution_units;
        for rowIndex = 1:rowCount
            yRange_units = minimum_units(2)+[rowIndex-1,rowIndex]*resolution_units;
            square = polyshape([xRange_units(1),yRange_units(1);xRange_units(2),yRange_units(1); ...
                xRange_units(2),yRange_units(2);xRange_units(1),yRange_units(2)]);
            piece = intersect(sweptUnion,square);
            if isempty(piece.Vertices) || area(piece) <= 0, continue; end
            for component = regions(piece).'
                vertices_units = component.Vertices(all(isfinite(component.Vertices),2),:);
                if size(unique(vertices_units,'rows'),1) < 3, continue; end
                hullIndex = convhull(vertices_units(:,1),vertices_units(:,2));
                regions_units{end+1,1} = vertices_units(hullIndex(1:end-1),:); %#ok<AGROW>
            end
        end
    end
    % The enclosure used by point queries is the union of the cover pieces.
    % The cells are an exact convex repartition of that union, so queries
    % and cells agree exactly and the cell count follows the enclosure's
    % shape rather than the grid.
    cover(1,numel(regions_units)) = polyshape();
    for regionIndex = 1:numel(regions_units)
        cover(regionIndex) = polyshape(regions_units{regionIndex},'Simplify',false,'KeepCollinearPoints',true);
    end
    shape = balancedUnion(cover);
    regions_units = obstacleAvoidance.geometry.convexRegions(shape,true);
end
timing_s(4) = toc(stageTimer);
counts(2) = numel(regions_units);
supported = ~isempty(regions_units);
end

%% Section 6: Local Functions
function shape = balancedUnion(pieces)
    % Keep intermediate boundaries local instead of repeatedly unioning the
    % complete accumulated boundary with one more piece.
    while numel(pieces) > 1
        pairCount = floor(numel(pieces)/2);
        merged = union(pieces(1:2:2*pairCount),pieces(2:2:2*pairCount), ...
            'KeepCollinearPoints',true);
        if mod(numel(pieces),2), merged(end+1) = pieces(end); end %#ok<AGROW>
        pieces = merged;
    end
    shape = pieces;
end
