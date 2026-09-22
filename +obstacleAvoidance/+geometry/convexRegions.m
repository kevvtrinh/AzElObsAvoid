function regions_units = convexRegions(shape)
%% Section 0: Header & Readme
% SYNTAX
%   regions_units = obstacleAvoidance.geometry.convexRegions(shape)
%**************************************************************************
% PURPOSE
%   - Divide the polygon's occupied area into convex pieces: polygons with
%     no inward corners. Preserve holes and return the pieces in a repeatable
%     order for later obstacle-separation checks.
%**************************************************************************
% INPUTS
%   - shape (scalar polyshape)
%       Valid polygon to divide into convex pieces.
%**************************************************************************
% OUTPUTS
%   - regions_units (N-by-1 cell array)
%       One [x y] vertex array per piece. Together, the pieces cover exactly
%       the input polygon. An empty shape returns a 0-by-1 cell array;
%       invalid input throws an error.
%**************************************************************************
% UNITS
%   - Region vertices are coordinate units.
%**************************************************************************

%% Section 1: Divide Each Separate Part Into Convex Pieces

regions_units = cell(0, 1);
shapeParts    = regions(shape);
for partIndex = 1:numel(shapeParts)
    shapePart      = shapeParts(partIndex);
    vertices_units = shapePart.Vertices;

    % A part with no holes is convex when its boundary turns only left or
    % only right. Keep it whole in that case. Zero marks a straight edge,
    % so extra points along a straight edge do not change this decision.
    partHasOneBoundary = shapePart.NumHoles == 0 && all(isfinite(vertices_units), 'all');
    if partHasOneBoundary
        edgeVectors_units        = circshift(vertices_units, -1) - vertices_units;
        nextEdgeVectors_units    = circshift(edgeVectors_units, -1);
        turnCrossProducts_units2 = edgeVectors_units(:, 1) .* nextEdgeVectors_units(:, 2) - ...
            edgeVectors_units(:, 2) .* nextEdgeVectors_units(:, 1);
        partIsConvex = all(turnCrossProducts_units2 >= 0) || all(turnCrossProducts_units2 <= 0);
        if partIsConvex
            regions_units{end + 1, 1} = vertices_units; %#ok<AGROW>
            continue;
        end
    end

    % Triangles cover the occupied area without filling holes. Join adjacent
    % triangles when their combined boundary still has no inward corners.
    triangleMesh        = triangulation(shapePart);
    regionVertexIndices = mergeAdjacentTriangles(triangleMesh);
    for regionIndex = 1:numel(regionVertexIndices)
        regions_units{end + 1, 1} = triangleMesh.Points(regionVertexIndices{regionIndex}, :); %#ok<AGROW>
    end
end

%% Section 2: Give The Regions A Repeatable Order

% Sort by minimum x, minimum y, maximum x, maximum y, then area. Repeated
% calls with the same geometry therefore use the same region ordering.
if numel(regions_units) > 1
    regionSortKeys = zeros(numel(regions_units), 5);
    for regionIndex = 1:numel(regions_units)
        vertices_units = regions_units{regionIndex};
        regionSortKeys(regionIndex, :) = [min(vertices_units, [], 1), max(vertices_units, [], 1), ...
            polyarea(vertices_units(:, 1), vertices_units(:, 2))];
    end
    [~, sortOrder] = sortrows(regionSortKeys, 1:size(regionSortKeys, 2));
    regions_units = regions_units(sortOrder);
end
end

%% Section 3: Local Functions

function regionVertexIndices = mergeAdjacentTriangles(triangleMesh)
    % Start with one region per triangle. Joining two regions removes their
    % shared edge while keeping the original vertices and occupied area.
    % Keep a merge only when the combined boundary remains convex.
    triangleCount = size(triangleMesh.ConnectivityList, 1);

    regionVertexIndices        = mat2cell(triangleMesh.ConnectivityList, ones(triangleCount, 1), 3);
    neighboringTriangleIndices = neighbors(triangleMesh);
    mergedRegionIndex          = (1:triangleCount).';

    % List each neighboring triangle pair once. Missing neighbors are NaN;
    % requiring first index < second index removes the reverse duplicates.
    triangleIndexGrid     = repmat(mergedRegionIndex, 1, 3);
    adjacentTrianglePairs = [triangleIndexGrid(:), neighboringTriangleIndices(:)];
    adjacentTrianglePairs = adjacentTrianglePairs(isfinite(adjacentTrianglePairs(:, 2)) & ...
        adjacentTrianglePairs(:, 1) < adjacentTrianglePairs(:, 2), :);
    if ~isempty(adjacentTrianglePairs)
        % Neighbor column k lies across the edge opposite vertex k. For
        % example, column 1 identifies the edge joining vertices 2 and 3.
        % Find both endpoints of every shared edge together.
        vertexColumns = repmat(1:3, triangleCount, 1);
        pairIsValid   = isfinite(neighboringTriangleIndices) & ...
            triangleIndexGrid < neighboringTriangleIndices;
        oppositeVertexColumn  = vertexColumns(pairIsValid);
        firstEndpointColumn   = mod(oppositeVertexColumn, 3) + 1;
        secondEndpointColumn  = mod(oppositeVertexColumn + 1, 3) + 1;
        triangleVertexIndices = triangleMesh.ConnectivityList;

        firstEndpointLinearIndex = sub2ind( ...
            size(triangleVertexIndices), adjacentTrianglePairs(:, 1), firstEndpointColumn);
        secondEndpointLinearIndex = sub2ind( ...
            size(triangleVertexIndices), adjacentTrianglePairs(:, 1), secondEndpointColumn);
        edgeStart_units = triangleMesh.Points(triangleVertexIndices(firstEndpointLinearIndex), :);
        edgeEnd_units   = triangleMesh.Points(triangleVertexIndices(secondEndpointLinearIndex), :);

        % Put the smaller x endpoint first, using y to break an x tie. Both
        % triangles then describe their shared edge in the same order.
        endpointsNeedSwap = edgeStart_units(:, 1) > edgeEnd_units(:, 1) | ...
            (edgeStart_units(:, 1) == edgeEnd_units(:, 1) & edgeStart_units(:, 2) > edgeEnd_units(:, 2));
        savedEdgeStart_units = edgeStart_units(endpointsNeedSwap, :);
        edgeStart_units(endpointsNeedSwap, :) = edgeEnd_units(endpointsNeedSwap, :);
        edgeEnd_units(endpointsNeedSwap, :)   = savedEdgeStart_units;

        % Try longer shared edges first. Endpoint coordinates break length
        % ties so the merge order is repeatable.
        sharedEdgeSortKeys = [-sum((edgeEnd_units - edgeStart_units) .^ 2, 2), ...
            edgeStart_units, edgeEnd_units];
        [~, sharedEdgeOrder] = sortrows(sharedEdgeSortKeys, 1:size(sharedEdgeSortKeys, 2));
        adjacentTrianglePairs = adjacentTrianglePairs(sharedEdgeOrder, :);
    end

    % Repeat until a full pass can no longer join any regions.
    mergedAnyRegion = true;
    while mergedAnyRegion
        mergedAnyRegion = false;
        for pairIndex = 1:size(adjacentTrianglePairs, 1)
            % Earlier merges may have put either triangle into a larger
            % region. Follow its saved indices to the region that is active now.
            firstRegionIndex  = adjacentTrianglePairs(pairIndex, 1);
            secondRegionIndex = adjacentTrianglePairs(pairIndex, 2);
            while mergedRegionIndex(firstRegionIndex) ~= firstRegionIndex
                firstRegionIndex = mergedRegionIndex(firstRegionIndex);
            end
            while mergedRegionIndex(secondRegionIndex) ~= secondRegionIndex
                secondRegionIndex = mergedRegionIndex(secondRegionIndex);
            end
            if firstRegionIndex == secondRegionIndex
                continue;
            end
            firstRegionVertexIndices  = regionVertexIndices{firstRegionIndex};
            secondRegionVertexIndices = regionVertexIndices{secondRegionIndex};

            % Find the vertices shared by these regions. Compare all pairs
            % for small regions (at most 1024 comparisons); use intersect for
            % larger ones to avoid creating a large temporary array.
            if numel(firstRegionVertexIndices) * numel(secondRegionVertexIndices) <= 1024
                sharedVertexIndices = sort(firstRegionVertexIndices( ...
                    any(firstRegionVertexIndices(:) == secondRegionVertexIndices(:).', 2)));
            else
                sharedVertexIndices = intersect(firstRegionVertexIndices, secondRegionVertexIndices);
            end
            if numel(sharedVertexIndices) ~= 2
                continue;
            end

            % The shared vertices must be neighbors along both boundaries.
            % Walk the common edge in opposite directions so removing it joins
            % the two boundaries into one loop.
            firstSharedEdgeIndex = find(firstRegionVertexIndices == sharedVertexIndices(1));
            if firstRegionVertexIndices(mod(firstSharedEdgeIndex, numel(firstRegionVertexIndices)) + 1) ~= ...
                    sharedVertexIndices(2)
                firstSharedEdgeIndex = find(firstRegionVertexIndices == sharedVertexIndices(2));
                if firstRegionVertexIndices(mod(firstSharedEdgeIndex, numel(firstRegionVertexIndices)) + 1) ~= ...
                        sharedVertexIndices(1)
                    continue;
                end
            end
            sharedEdgeEndVertexIndex = firstRegionVertexIndices( ...
                mod(firstSharedEdgeIndex, numel(firstRegionVertexIndices)) + 1);
            secondSharedEdgeIndex   = find(secondRegionVertexIndices == sharedEdgeEndVertexIndex);
            if secondRegionVertexIndices(mod(secondSharedEdgeIndex, numel(secondRegionVertexIndices)) + 1) ~= ...
                    firstRegionVertexIndices(firstSharedEdgeIndex)
                secondRegionVertexIndices = fliplr(secondRegionVertexIndices);
                secondSharedEdgeIndex     = find(secondRegionVertexIndices == sharedEdgeEndVertexIndex);
                if secondRegionVertexIndices(mod(secondSharedEdgeIndex, numel(secondRegionVertexIndices)) + 1) ~= ...
                        firstRegionVertexIndices(firstSharedEdgeIndex)
                    continue;
                end
            end

            % Start both vertex lists just after the shared edge. Combine
            % the lists, keeping each shared endpoint once.
            firstRegionVertexIndices  = firstRegionVertexIndices( ...
                [firstSharedEdgeIndex + 1:end, 1:firstSharedEdgeIndex]);
            secondRegionVertexIndices = secondRegionVertexIndices( ...
                [secondSharedEdgeIndex + 1:end, 1:secondSharedEdgeIndex]);
            mergedVertexIndices = [firstRegionVertexIndices, secondRegionVertexIndices(2:end - 1)];

            % Cross-product signs describe left and right turns. Mixed signs
            % mean an inward corner, so leave those regions separate.
            mergedVertices_units     = triangleMesh.Points(mergedVertexIndices, :);
            edgeVectors_units        = circshift(mergedVertices_units, -1) - mergedVertices_units;
            nextEdgeVectors_units    = circshift(edgeVectors_units, -1);
            turnCrossProducts_units2 = edgeVectors_units(:, 1) .* nextEdgeVectors_units(:, 2) - ...
                edgeVectors_units(:, 2) .* nextEdgeVectors_units(:, 1);
            mergeStaysConvex = all(turnCrossProducts_units2 >= 0) || all(turnCrossProducts_units2 <= 0);
            if ~mergeStaysConvex
                continue;
            end

            % Keep the merged boundary under the first region's index. Later
            % pairs involving the second region will follow this saved link.
            regionVertexIndices{firstRegionIndex}  = mergedVertexIndices;
            regionVertexIndices{secondRegionIndex} = [];
            mergedRegionIndex(secondRegionIndex) = firstRegionIndex;
            mergedAnyRegion = true;
        end
    end
    regionVertexIndices = regionVertexIndices(~cellfun(@isempty, regionVertexIndices));
end
