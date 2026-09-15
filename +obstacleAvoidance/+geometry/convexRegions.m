function regions_units = convexRegions(shape)
%% Section 0: Header & Readme
% SYNTAX
%   regions_units = obstacleAvoidance.geometry.convexRegions(shape)
%**************************************************************************
% PURPOSE
%   - Cover a polygon, including holes, with ordered convex regions.
%**************************************************************************
% INPUTS
%   - shape (scalar polyshape)
%       Valid polygon geometry to decompose.
%**************************************************************************
% OUTPUTS
%   - regions_units (N-by-1 cell array)
%       Convex vertex arrays whose union exactly covers the input shape. An
%       empty shape returns a 0-by-1 cell array; invalid input throws.
%**************************************************************************
% UNITS
%   - Region vertices are coordinate units.
%**************************************************************************

%% Section 1: Keep Convex Components Or Triangulate Exact Geometry

regions_units = cell(0, 1);
components    = regions(shape);
for componentIndex = 1:numel(components)
    component      = components(componentIndex);
    vertices_units = component.Vertices;

    % A simply connected component with a single turn sign is already convex
    % and is retained exactly, without introducing triangulation vertices.
    componentIsSimplyConnected = component.NumHoles == 0 && all(isfinite(vertices_units), 'all');
    if componentIsSimplyConnected
        edges_units    = circshift(vertices_units, -1) - vertices_units;
        nextEdge_units = circshift(edges_units, -1);
        turns_units2   = edges_units(:, 1) .* nextEdge_units(:, 2) - edges_units(:, 2) .* nextEdge_units(:, 1);
        componentIsConvex = all(turns_units2 >= 0) || all(turns_units2 <= 0);
        if componentIsConvex
            regions_units{end + 1, 1} = vertices_units; %#ok<AGROW>
            continue;
        end
    end

    mesh  = triangulation(component);
    faces = mergeConvexFaces(mesh);
    for faceIndex = 1:numel(faces)
        regions_units{end + 1, 1} = mesh.Points(faces{faceIndex}, :); %#ok<AGROW>
    end
end

%% Section 2: Order The Regions Deterministically

if numel(regions_units) > 1
    sortKeys = zeros(numel(regions_units), 5);
    for regionIndex = 1:numel(regions_units)
        vertices_units = regions_units{regionIndex};
        sortKeys(regionIndex, :) = [min(vertices_units, [], 1), max(vertices_units, [], 1), ...
            polyarea(vertices_units(:, 1), vertices_units(:, 2))];
    end
    [~, sortOrder] = sortrows(sortKeys, 1:size(sortKeys, 2));
    regions_units = regions_units(sortOrder);
end
end

%% Section 3: Local Functions

function faces = mergeConvexFaces(mesh)
    % Every accepted merge removes one shared diagonal from two exact faces.
    % Original vertices and the occupied union remain unchanged. Concave or
    % multiply connected unions are rejected, with no geometric tolerance.
    faceCount     = size(mesh.ConnectivityList, 1);
    faces         = mat2cell(mesh.ConnectivityList, ones(faceCount, 1), 3);
    adjacentFaces = neighbors(mesh);
    owner         = (1:faceCount).';

    faceIndexGrid = repmat(owner, 1, 3);
    pairs         = [faceIndexGrid(:), adjacentFaces(:)];
    pairs         = pairs(isfinite(pairs(:, 2)) & pairs(:, 1) < pairs(:, 2), :);
    if ~isempty(pairs)
        % Neighbour column k is across the edge opposite triangle vertex k.
        % Assemble those same shared endpoints in one batch, without a set
        % intersection and sort for every mesh edge.
        edgeColumns         = repmat(1:3, faceCount, 1);
        pairIsValid          = isfinite(adjacentFaces) & faceIndexGrid < adjacentFaces;
        oppositeVertexIndex = edgeColumns(pairIsValid);
        firstEndpointIndex   = mod(oppositeVertexIndex, 3) + 1;
        secondEndpointIndex  = mod(oppositeVertexIndex + 1, 3) + 1;
        connectivity         = mesh.ConnectivityList;

        firstLinearIndex  = sub2ind(size(connectivity), pairs(:, 1), firstEndpointIndex);
        secondLinearIndex = sub2ind(size(connectivity), pairs(:, 1), secondEndpointIndex);
        endpointA_units   = mesh.Points(connectivity(firstLinearIndex), :);
        endpointB_units   = mesh.Points(connectivity(secondLinearIndex), :);

        % Order each endpoint pair canonically so the shared-edge sort key is
        % independent of the triangle that reported the edge.
        endpointsNeedSwap = endpointA_units(:, 1) > endpointB_units(:, 1) | ...
            (endpointA_units(:, 1) == endpointB_units(:, 1) & endpointA_units(:, 2) > endpointB_units(:, 2));
        temporary_units = endpointA_units(endpointsNeedSwap, :);
        endpointA_units(endpointsNeedSwap, :) = endpointB_units(endpointsNeedSwap, :);
        endpointB_units(endpointsNeedSwap, :) = temporary_units;
        edgeKey_units = [-sum((endpointB_units - endpointA_units).^2, 2), ...
            endpointA_units, endpointB_units];
        [~, edgeOrder] = sortrows(edgeKey_units, 1:size(edgeKey_units, 2));
        pairs = pairs(edgeOrder, :);
    end

    mergedAnyFace = true;
    while mergedAnyFace
        mergedAnyFace = false;
        for pairIndex = 1:size(pairs, 1)
            leftRoot  = pairs(pairIndex, 1);
            rightRoot = pairs(pairIndex, 2);
            while owner(leftRoot) ~= leftRoot
                leftRoot = owner(leftRoot);
            end
            while owner(rightRoot) ~= rightRoot
                rightRoot = owner(rightRoot);
            end
            if leftRoot == rightRoot
                continue;
            end
            leftFace  = faces{leftRoot};
            rightFace = faces{rightRoot};

            % Face indices are unique. Avoid general set-operation setup for
            % short faces, while bounding the temporary comparison array.
            if numel(leftFace) * numel(rightFace) <= 1024
                sharedVertexIndices = sort(leftFace(any(leftFace(:) == rightFace(:).', 2)));
            else
                sharedVertexIndices = intersect(leftFace, rightFace);
            end
            if numel(sharedVertexIndices) ~= 2
                continue;
            end

            % Locate the shared diagonal as a directed edge of the left face,
            % then require the right face to traverse it in the opposite sense.
            leftIndex = find(leftFace == sharedVertexIndices(1));
            if leftFace(mod(leftIndex, numel(leftFace)) + 1) ~= sharedVertexIndices(2)
                leftIndex = find(leftFace == sharedVertexIndices(2));
                if leftFace(mod(leftIndex, numel(leftFace)) + 1) ~= sharedVertexIndices(1)
                    continue;
                end
            end
            diagonalEnd = leftFace(mod(leftIndex, numel(leftFace)) + 1);
            rightIndex  = find(rightFace == diagonalEnd);
            if rightFace(mod(rightIndex, numel(rightFace)) + 1) ~= leftFace(leftIndex)
                rightFace = fliplr(rightFace);
                rightIndex = find(rightFace == diagonalEnd);
                if rightFace(mod(rightIndex, numel(rightFace)) + 1) ~= leftFace(leftIndex)
                    continue;
                end
            end

            % Rotate both rings to start after the diagonal, then splice out
            % the duplicated diagonal endpoints.
            leftFace = leftFace([leftIndex + 1:end, 1:leftIndex]);
            rightFace = rightFace([rightIndex + 1:end, 1:rightIndex]);
            mergedFace = [leftFace, rightFace(2:end - 1)];

            points_units   = mesh.Points(mergedFace, :);
            edges_units    = circshift(points_units, -1) - points_units;
            nextEdge_units = circshift(edges_units, -1);
            turns_units2   = edges_units(:, 1) .* nextEdge_units(:, 2) - edges_units(:, 2) .* nextEdge_units(:, 1);
            mergeStaysConvex = all(turns_units2 >= 0) || all(turns_units2 <= 0);
            if ~mergeStaysConvex
                continue;
            end

            faces{leftRoot}  = mergedFace;
            faces{rightRoot} = [];
            owner(rightRoot)  = leftRoot;
            mergedAnyFace     = true;
        end
    end
    faces = faces(~cellfun(@isempty, faces));
end
