function regions_units = convexRegions(shape, longestSharedEdgeFirst)
%% Section 0: Header & Readme
% SYNTAX: regions_units = obstacleAvoidance.geometry.convexRegions(shape)
%         regions_units = obstacleAvoidance.geometry.convexRegions(shape,true)
% PURPOSE: Exactly cover a polygon (including holes) with convex regions.
% INPUTS: A valid polyshape and optional deterministic merge ordering.
% OUTPUTS: Column cell array of convex vertex arrays.
% UNITS: Coordinate units.

%% Section 1: Keep Convex Components Or Triangulate Exact Geometry
if nargin < 2, longestSharedEdgeFirst = false; end
regions_units = cell(0, 1);
components = regions(shape);
for k = 1:numel(components)
    vertices_units = components(k).Vertices;
    if components(k).NumHoles == 0 && all(isfinite(vertices_units), 'all')
        edges_units = circshift(vertices_units, -1) - vertices_units;
        next_units = circshift(edges_units, -1);
        turns_units2 = edges_units(:,1).*next_units(:,2) - edges_units(:,2).*next_units(:,1);
        if all(turns_units2 >= 0) || all(turns_units2 <= 0)
            regions_units{end+1,1} = vertices_units; %#ok<AGROW>
            continue;
        end
    end
    mesh = triangulation(components(k));
    faces = mergeConvexFaces(mesh,longestSharedEdgeFirst);
    for j = 1:numel(faces)
        regions_units{end+1,1} = mesh.Points(faces{j},:); %#ok<AGROW>
    end
end
if longestSharedEdgeFirst && numel(regions_units) > 1
    sortKeys = zeros(numel(regions_units),5);
    for regionIndex = 1:numel(regions_units)
        vertices_units = regions_units{regionIndex};
        sortKeys(regionIndex,:) = [min(vertices_units,[],1), ...
            max(vertices_units,[],1),polyarea(vertices_units(:,1),vertices_units(:,2))];
    end
    [~,order] = sortrows(sortKeys,1:size(sortKeys,2));
    regions_units = regions_units(order);
end
end

%% Section 2: Remove Interior Diagonals Without Changing Geometry
function faces = mergeConvexFaces(mesh,longestSharedEdgeFirst)
    % Every accepted merge removes one shared diagonal from two exact faces.
    % Original vertices and the occupied union remain unchanged. Concave or
    % multiply connected unions are rejected, with no geometric tolerance.
    count = size(mesh.ConnectivityList,1);
    faces = mat2cell(mesh.ConnectivityList,ones(count,1),3);
    adjacent = neighbors(mesh);
    owner = (1:count).';
    first = repmat(owner,1,3);
    pairs = [first(:),adjacent(:)];
    pairs = pairs(isfinite(pairs(:,2)) & pairs(:,1)<pairs(:,2),:);
    if longestSharedEdgeFirst && ~isempty(pairs)
        % Neighbour column k is across the edge opposite triangle vertex k.
        % Assemble those same shared endpoints in one batch, without a set
        % intersection and sort for every mesh edge.
        edgeColumns = repmat(1:3,count,1);
        validPair = isfinite(adjacent) & first<adjacent;
        opposite = edgeColumns(validPair);
        firstEndpoint = mod(opposite,3)+1;
        secondEndpoint = mod(opposite+1,3)+1;
        connectivity = mesh.ConnectivityList;
        endpointA_units = mesh.Points(connectivity(sub2ind(size(connectivity),pairs(:,1),firstEndpoint)),:);
        endpointB_units = mesh.Points(connectivity(sub2ind(size(connectivity),pairs(:,1),secondEndpoint)),:);
        swap = endpointA_units(:,1)>endpointB_units(:,1) | ...
            (endpointA_units(:,1)==endpointB_units(:,1) & endpointA_units(:,2)>endpointB_units(:,2));
        temporary_units = endpointA_units(swap,:);
        endpointA_units(swap,:) = endpointB_units(swap,:);
        endpointB_units(swap,:) = temporary_units;
        edgeKey_units = [-sum((endpointB_units-endpointA_units).^2,2),endpointA_units,endpointB_units];
        [~,order] = sortrows(edgeKey_units,1:size(edgeKey_units,2));
        pairs = pairs(order,:);
    end
    changed = true;
    while changed
        changed = false;
        for k = 1:size(pairs,1)
            left = pairs(k,1); right = pairs(k,2);
            while owner(left)~=left, left = owner(left); end
            while owner(right)~=right, right = owner(right); end
            if left==right, continue; end
            a = faces{left}; b = faces{right};
            % Face indices are unique. Avoid general set-operation setup for
            % short faces, while bounding the temporary comparison array.
            if numel(a)*numel(b)<=1024
                shared = sort(a(any(a(:)==b(:).',2)));
            else
                shared = intersect(a,b);
            end
            if numel(shared)~=2, continue; end
            index = find(a==shared(1));
            if a(mod(index,numel(a))+1)~=shared(2)
                index = find(a==shared(2));
                if a(mod(index,numel(a))+1)~=shared(1), continue; end
            end
            next = a(mod(index,numel(a))+1);
            other = find(b==next);
            if b(mod(other,numel(b))+1)~=a(index)
                b = fliplr(b); other = find(b==next);
                if b(mod(other,numel(b))+1)~=a(index), continue; end
            end
            a = a([index+1:end,1:index]);
            b = b([other+1:end,1:other]);
            merged = [a,b(2:end-1)];
            points_units = mesh.Points(merged,:);
            edges_units = circshift(points_units,-1)-points_units;
            nextEdge_units = circshift(edges_units,-1);
            turns_units2 = edges_units(:,1).*nextEdge_units(:,2)-edges_units(:,2).*nextEdge_units(:,1);
            if ~(all(turns_units2>=0) || all(turns_units2<=0)), continue; end
            faces{left} = merged; faces{right} = [];
            owner(right) = left;
            changed = true;
        end
    end
    faces = faces(~cellfun(@isempty,faces));
end
