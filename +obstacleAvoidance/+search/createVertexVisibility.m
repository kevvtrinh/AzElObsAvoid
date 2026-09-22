function vertexVisibility = createVertexVisibility(obstacleSnapshot, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   vertexVisibility = obstacleAvoidance.search.createVertexVisibility( ...
%       obstacleSnapshot, limits, options)
%**************************************************************************
% PURPOSE
%   - Check the straight connection between every pair of obstacle-boundary
%     vertices. A connection is visible when it does not enter an obstacle.
%   - Save these checks so createVisibilityGraph can add different start and
%     goal positions without checking the same obstacle-vertex pairs again.
%**************************************************************************
% INPUTS
%   - obstacleSnapshot (protected obstacle array)
%       Polygon geometry at the planning time.
%   - limits (scalar struct)
%       Workspace limits.
%   - options (scalar struct)
%       Numerical tolerance options.
%**************************************************************************
% OUTPUTS
%   - vertexVisibility (scalar struct)
%       Combined obstacle shape, boundary edges, workspace vertices, and
%       all accepted and rejected vertex pairs. Corner data identifies
%       directions that point into an obstacle. Invalid input throws an error.
%**************************************************************************
% UNITS
%   - Geometry and edge lengths are coordinate units.
%**************************************************************************

%% Section 1: Combine The Obstacle Shapes And List Their Edges

% Combine overlapping obstacles before listing vertices, so boundaries
% hidden inside another obstacle do not become route corners.
tolerance_units = options.ConstraintTolerance;
occupiedShape   = polyshape();
for obstacleIndex = 1:numel(obstacleSnapshot)
    occupiedShape = union(occupiedShape, obstacleSnapshot(obstacleIndex).ProtectedShape);
end
[edgeStart_units, edgeEnd_units] = obstacleAvoidance.geometry.boundaryToEdges(occupiedShape, 0);
edgeVector_units = edgeEnd_units - edgeStart_units;
boundary_units   = occupiedShape.Vertices;
vertices_units   = boundary_units(all(isfinite(boundary_units), 2), :);

% Only vertices strictly inside the workspace can become graph nodes.
vertexIsInsideWorkspace = ...
    vertices_units(:, 1) > limits.xInterval_units(1) & vertices_units(:, 1) < limits.xInterval_units(2) & ...
    vertices_units(:, 2) > limits.yInterval_units(1) & vertices_units(:, 2) < limits.yInterval_units(2);
vertexVisibility = struct( ...
    'Shape',                    occupiedShape, ...
    'Boundary_units',           boundary_units, ...
    'EdgeStart_units',          edgeStart_units, ...
    'EdgeEnd_units',            edgeEnd_units, ...
    'EdgeVector_units',         edgeVector_units, ...
    'EdgeBounds_units',         [min(edgeStart_units, edgeEnd_units), max(edgeStart_units, edgeEnd_units)], ...
    'EdgeSide',                 occupiedEdgeSides(occupiedShape, edgeStart_units, edgeEnd_units), ...
    'ParallelTolerance_units2', tolerance_units * max(1, vecnorm(edgeVector_units, 2, 2)), ...
    'Tolerance_units',          tolerance_units, ...
    'WorkspaceMinimum_units',   [limits.xInterval_units(1), limits.yInterval_units(1)], ...
    'WorkspaceMaximum_units',   [limits.xInterval_units(2), limits.yInterval_units(2)], ...
    'Vertices_units',           unique(vertices_units(vertexIsInsideWorkspace, :), 'rows', 'stable'));

%% Section 2: Check The Straight Line Between Every Pair Of Vertices

% A corner cone records directions leading immediately into an obstacle.
% The segment check uses it to reject those directions before intersections.
vertexCount = size(vertexVisibility.Vertices_units, 1);
vertexVisibility.VertexCones = obstacleAvoidance.search.createNodeCones( ...
    vertexVisibility, vertexVisibility.Vertices_units);

% Check each unordered pair once: N vertices give N x (N - 1) / 2 pairs.
% Boundary contact is allowed; entering the obstacle interior is rejected.
maximumPairCount    = vertexCount * (vertexCount - 1) / 2;
acceptedVertexPairs = zeros(maximumPairCount, 2);
rejectedVertexPairs = zeros(maximumPairCount, 2);
acceptedCount       = 0;
rejectedCount       = 0;
for firstVertexIndex = 1:vertexCount - 1
    secondVertexIndices = (firstVertexIndex + 1:vertexCount).';
    segmentIsClear = obstacleAvoidance.search.classifyVisibilitySegments( ...
        vertexVisibility, vertexVisibility.Vertices_units, vertexVisibility.VertexCones, ...
        firstVertexIndex, secondVertexIndices);

    acceptedVertexIndices = secondVertexIndices(segmentIsClear);
    acceptedRowIndices    = acceptedCount + (1:numel(acceptedVertexIndices));
    acceptedVertexPairs(acceptedRowIndices, :) = ...
        [repmat(firstVertexIndex, numel(acceptedVertexIndices), 1), acceptedVertexIndices];
    acceptedCount = acceptedCount + numel(acceptedVertexIndices);

    rejectedVertexIndices = secondVertexIndices(~segmentIsClear);
    rejectedRowIndices    = rejectedCount + (1:numel(rejectedVertexIndices));
    rejectedVertexPairs(rejectedRowIndices, :) = ...
        [repmat(firstVertexIndex, numel(rejectedVertexIndices), 1), rejectedVertexIndices];
    rejectedCount = rejectedCount + numel(rejectedVertexIndices);
end
vertexVisibility.VertexPairAccepted = acceptedVertexPairs(1:acceptedCount, :);
vertexVisibility.VertexPairRejected = rejectedVertexPairs(1:rejectedCount, :);
end

%% Section 3: Local Functions

function occupiedSide = occupiedEdgeSides(occupiedShape, edgeStart_units, edgeEnd_units)
    % Triangles filling the obstacle identify its occupied side at each edge,
    % including holes and either boundary direction. Looking from edge start
    % to edge end: +1 means left, -1 means right, and 0 means unresolved.
    occupiedSide = zeros(size(edgeStart_units, 1), 1);
    if isempty(edgeStart_units)
        return
    end
    shapeTriangulation    = triangulation(occupiedShape);
    triangleVertexIndices = shapeTriangulation.ConnectivityList;
    triangleEdgeIndices   = [triangleVertexIndices(:, [1, 2]); ...
        triangleVertexIndices(:, [2, 3]); triangleVertexIndices(:, [3, 1])];
    % Pair each triangle edge with its opposite vertex, which lies on the
    % occupied side. Match edges without depending on their direction.
    oppositeVertexIndices = [triangleVertexIndices(:, 3); triangleVertexIndices(:, 1); triangleVertexIndices(:, 2)];
    [edgeStartWasFound, edgeStartVertexIndex] = ismember(edgeStart_units, shapeTriangulation.Points, 'rows');
    [edgeEndWasFound, edgeEndVertexIndex]     = ismember(edgeEnd_units, shapeTriangulation.Points, 'rows');
    [triangleEdgeWasFound, matchingTriangleEdgeIndex] = ismember( ...
        sort([edgeStartVertexIndex, edgeEndVertexIndex], 2), sort(triangleEdgeIndices, 2), 'rows');
    edgeHasInteriorTriangle = edgeStartWasFound & edgeEndWasFound & triangleEdgeWasFound;
    edgeVector_units        = edgeEnd_units - edgeStart_units;

    % The cross-product sign gives the side containing the opposite vertex.
    interiorVertexIndices = oppositeVertexIndices(matchingTriangleEdgeIndex(edgeHasInteriorTriangle));
    towardOppositeVertex_units = ...
        shapeTriangulation.Points(interiorVertexIndices, :) - edgeStart_units(edgeHasInteriorTriangle, :);
    occupiedSide(edgeHasInteriorTriangle) = ...
        sign(edgeVector_units(edgeHasInteriorTriangle, 1) .* towardOppositeVertex_units(:, 2) - ...
        edgeVector_units(edgeHasInteriorTriangle, 2) .* towardOppositeVertex_units(:, 1));
end
