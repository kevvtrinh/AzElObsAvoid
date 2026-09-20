function vertexVisibility = createVertexVisibility(scene, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   vertexVisibility = obstacleAvoidance.search.createVertexVisibility(scene, limits, options)
%**************************************************************************
% PURPOSE
%   - Classify exact visibility between every pair of protected boundary
%     vertices once per snapshot, so any number of start and goal pairs can
%     be attached by createVisibilityGraph without repeating that work.
%**************************************************************************
% INPUTS
%   - scene (protected obstacle array)
%       Polygon geometry at the planning time.
%   - limits (scalar struct)
%       Workspace limits.
%   - options (scalar struct)
%       Numerical tolerance options.
%**************************************************************************
% OUTPUTS
%   - vertexVisibility (scalar struct)
%       Occupied union, boundary edges with their occupied sides, the
%       in-workspace vertices with their corner cones, and every accepted
%       and rejected vertex pair. Invalid input throws an error.
%**************************************************************************
% UNITS
%   - Geometry and edge lengths are coordinate units.
%**************************************************************************

%% Section 1: Form The Occupied Union And Boundary Edge Arrays

tolerance_units = options.ConstraintTolerance;
shape           = polyshape();
for sceneIndex = 1:numel(scene)
    shape = union(shape, scene(sceneIndex).ProtectedShape);
end
[edgeStart_units, edgeEnd_units] = obstacleAvoidance.geometry.boundaryToEdges(shape, 0);
edgeVector_units = edgeEnd_units - edgeStart_units;
boundary_units   = shape.Vertices;
vertices_units   = boundary_units(all(isfinite(boundary_units), 2), :);
inWorkspace = vertices_units(:, 1) > limits.xInterval_units(1) & vertices_units(:, 1) < limits.xInterval_units(2) & ...
    vertices_units(:, 2) > limits.yInterval_units(1) & vertices_units(:, 2) < limits.yInterval_units(2);
vertexVisibility = struct( ...
    'Shape',                    shape, ...
    'Boundary_units',           boundary_units, ...
    'EdgeStart_units',          edgeStart_units, ...
    'EdgeEnd_units',            edgeEnd_units, ...
    'EdgeVector_units',         edgeVector_units, ...
    'EdgeBounds_units',         [min(edgeStart_units, edgeEnd_units), max(edgeStart_units, edgeEnd_units)], ...
    'EdgeSide',                 occupiedEdgeSides(shape, edgeStart_units, edgeEnd_units), ...
    'ParallelTolerance_units2', tolerance_units * max(1, vecnorm(edgeVector_units, 2, 2)), ...
    'Tolerance_units',          tolerance_units, ...
    'WorkspaceMinimum_units',   [limits.xInterval_units(1), limits.yInterval_units(1)], ...
    'WorkspaceMaximum_units',   [limits.xInterval_units(2), limits.yInterval_units(2)], ...
    'Vertices_units',           unique(vertices_units(inWorkspace, :), 'rows', 'stable'));

%% Section 2: Classify Every Vertex Pair

vertexCount          = size(vertexVisibility.Vertices_units, 1);
vertexVisibility.VertexCones = obstacleAvoidance.search.createNodeCones(vertexVisibility, vertexVisibility.Vertices_units);
maximumPairCount     = vertexCount * (vertexCount - 1) / 2;
accepted             = zeros(maximumPairCount, 2);
rejected             = zeros(maximumPairCount, 2);
acceptedCount        = 0;
rejectedCount        = 0;
for firstVertex = 1:vertexCount - 1
    secondVertex = (firstVertex + 1:vertexCount).';
    clear        = obstacleAvoidance.search.classifyVisibilitySegments( ...
        vertexVisibility, vertexVisibility.Vertices_units, vertexVisibility.VertexCones, firstVertex, secondVertex);
    acceptedVertex = secondVertex(clear);
    acceptedRows   = acceptedCount + (1:numel(acceptedVertex));
    accepted(acceptedRows, :) = [repmat(firstVertex, numel(acceptedVertex), 1), acceptedVertex];
    acceptedCount  = acceptedCount + numel(acceptedVertex);
    rejectedVertex = secondVertex(~clear);
    rejectedRows   = rejectedCount + (1:numel(rejectedVertex));
    rejected(rejectedRows, :) = [repmat(firstVertex, numel(rejectedVertex), 1), rejectedVertex];
    rejectedCount  = rejectedCount + numel(rejectedVertex);
end
vertexVisibility.VertexPairAccepted = accepted(1:acceptedCount, :);
vertexVisibility.VertexPairRejected = rejected(1:rejectedCount, :);
end

%% Section 3: Local Functions

function side = occupiedEdgeSides(shape, edgeStart_units, edgeEnd_units)
    % Use filled triangles to establish the occupied side of each boundary
    % edge without assuming ring winding, including hole boundaries.
    side = zeros(size(edgeStart_units, 1), 1);
    if isempty(edgeStart_units)
        return
    end
    mesh      = triangulation(shape);
    faces     = mesh.ConnectivityList;
    meshEdges = [faces(:, [1, 2]); faces(:, [2, 3]); faces(:, [3, 1])];
    opposite  = [faces(:, 3); faces(:, 1); faces(:, 2)];
    [firstFound, first] = ismember(edgeStart_units, mesh.Points, 'rows');
    [lastFound, last]   = ismember(edgeEnd_units, mesh.Points, 'rows');
    [edgeFound, face]   = ismember(sort([first, last], 2), sort(meshEdges, 2), 'rows');
    known            = firstFound & lastFound & edgeFound;
    edgeVector_units = edgeEnd_units - edgeStart_units;
    direction_units  = mesh.Points(opposite(face(known)), :) - edgeStart_units(known, :);
    side(known) = sign(edgeVector_units(known, 1) .* direction_units(:, 2) - ...
        edgeVector_units(known, 2) .* direction_units(:, 1));
end
