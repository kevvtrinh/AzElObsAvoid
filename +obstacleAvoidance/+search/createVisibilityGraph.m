function visibilityGraph = createVisibilityGraph(vertexVisibility, start_units, goal_units)
%% Section 0: Header & Readme
% SYNTAX
%   visibilityGraph = obstacleAvoidance.search.createVisibilityGraph( ...
%       vertexVisibility, start_units, goal_units)
%**************************************************************************
% PURPOSE
%   - Add start and goal positions to the saved obstacle-vertex graph,
%     then find the shortest route through its clear connections. BMTP still
%     needs to turn this spatial route into motion that satisfies the limits.
%**************************************************************************
% INPUTS
%   - vertexVisibility (scalar struct)
%       Obstacle-boundary vertices and their saved connection checks from
%       createVertexVisibility.
%   - start_units (1-by-2 numeric row)
%       Route start position.
%   - goal_units (1-by-2 numeric row)
%       Route goal position, distinct from the start.
%**************************************************************************
% OUTPUTS
%   - visibilityGraph (scalar struct)
%       Node positions, clear and blocked connections, and the shortest route.
%       A disconnected graph returns IsConnected = false and an empty route;
%       invalid input throws an error.
%**************************************************************************
% UNITS
%   - Geometry and route lengths are coordinate units.
%**************************************************************************

%% Section 1: Add The Start And Goal Positions

validateattributes(start_units, {'numeric'}, {'real', 'finite', 'size', [1, 2]});
validateattributes(goal_units, {'numeric'}, {'real', 'finite', 'size', [1, 2]});
% Put start and goal first, then remove duplicate obstacle vertices. This
% keeps the distinct endpoints at nodes 1 and 2 for the route search.
tolerance_units = vertexVisibility.Tolerance_units;
nodes_units     = unique([start_units; goal_units; vertexVisibility.Vertices_units], 'rows', 'stable');
startIsClear    = pointHasObstacleClearance(start_units, vertexVisibility, tolerance_units);
goalIsClear     = pointHasObstacleClearance(goal_units, vertexVisibility, tolerance_units);
endpoints_units = [start_units; goal_units];
endpointIsInsideWorkspace = all(endpoints_units >= vertexVisibility.WorkspaceMinimum_units & ...
    endpoints_units <= vertexVisibility.WorkspaceMaximum_units, 2);
startIsClear = startIsClear && endpointIsInsideWorkspace(1);
goalIsClear  = goalIsClear && endpointIsInsideWorkspace(2);

%% Section 2: Check Endpoint Connections And Reuse Obstacle Connections

nodeCount              = size(nodes_units, 1);
acceptedNodePairs      = zeros(0, 2);
connectionLength_units = zeros(0, 1);
rejectedNodePairs      = zeros(0, 2);
% Occupied or out-of-workspace endpoints cannot have a valid route.
if startIsClear && goalIsClear
    nodeCornerCones = obstacleAvoidance.search.createNodeCones(vertexVisibility, nodes_units);
    % Groups 1 and 2 hold new start/goal connections. Group 3 will hold
    % reusable connections between the remaining obstacle vertices.
    acceptedPairGroups = cell(3, 1);
    rejectedPairGroups = cell(3, 1);
    for firstNodeIndex = 1:2
        secondNodeIndices = (firstNodeIndex + 1:nodeCount).';
        segmentIsClear = obstacleAvoidance.search.classifyVisibilitySegments( ...
            vertexVisibility, nodes_units, nodeCornerCones, firstNodeIndex, secondNodeIndices);
        acceptedPairGroups{firstNodeIndex} = ...
            [repmat(firstNodeIndex, nnz(segmentIsClear), 1), secondNodeIndices(segmentIsClear)];
        rejectedPairGroups{firstNodeIndex} = ...
            [repmat(firstNodeIndex, nnz(~segmentIsClear), 1), secondNodeIndices(~segmentIsClear)];
    end
    % The start and goal are nodes 1 and 2. A vertex at either position
    % already has its connections checked above. Reuse all other saved vertex
    % connections, translating their indices into this graph's node list.
    [~, nodeIndexByVertex] = ismember(vertexVisibility.Vertices_units, nodes_units, 'rows');
    vertexIsSeparateNode = nodeIndexByVertex > 2;
    acceptedPairGroups{3} = mapVertexPairsToNodes( ...
        vertexVisibility.VertexPairAccepted, nodeIndexByVertex, vertexIsSeparateNode);
    rejectedPairGroups{3} = mapVertexPairsToNodes( ...
        vertexVisibility.VertexPairRejected, nodeIndexByVertex, vertexIsSeparateNode);
    acceptedNodePairs = vertcat(acceptedPairGroups{:});
    rejectedNodePairs = vertcat(rejectedPairGroups{:});
    connectionLength_units = vecnorm( ...
        nodes_units(acceptedNodePairs(:, 2), :) - nodes_units(acceptedNodePairs(:, 1), :), 2, 2);
end

%% Section 3: Find The Shortest Route And Return The Graph

% Each connection costs its physical length. The shortest path therefore
% minimizes route distance, without yet considering travel time or acceleration.
routeNodeIndices  = zeros(1, 0);
route_units       = zeros(0, 2);
routeLength_units = Inf;
if startIsClear && goalIsClear
    visibilityNetwork = graph( ...
        acceptedNodePairs(:, 1), acceptedNodePairs(:, 2), connectionLength_units, nodeCount);
    [routeNodeIndices, routeLength_units] = shortestpath(visibilityNetwork, 1, 2, 'Method', 'positive');
    route_units = nodes_units(routeNodeIndices, :);
end
visibilityGraph = struct( ...
    'NodePosition_units',     nodes_units, ...
    'AcceptedNodeIndex',      acceptedNodePairs, ...
    'RejectedNodeIndex',      rejectedNodePairs, ...
    'Route_units',            route_units, ...
    'RouteLength_units',      routeLength_units, ...
    'IsConnected',            ~isempty(routeNodeIndices), ...
    'ExpandedCount',          nodeCount, ...
    'GraphIsFullyEnumerated', startIsClear && goalIsClear);
end

%% Section 4: Local Functions

function pointIsClear = pointHasObstacleClearance(point_units, vertexVisibility, tolerance_units)
    % Start and goal need clearance from the protected obstacle boundary.
    % Unlike route corners, they cannot lie on that boundary or within the
    % clearance tolerance.
    if isempty(vertexVisibility.EdgeStart_units)
        pointIsClear = true;
        return;
    end
    pointIsInsideOrOnBoundary = inpolygon(point_units(1), point_units(2), ...
        vertexVisibility.Boundary_units(:, 1), vertexVisibility.Boundary_units(:, 2));

    % Project the point onto each edge's line. Clamp the fraction to [0, 1]
    % so the closest point stays on the edge: 0 = start, 1 = end.
    pointOffset_units = point_units - vertexVisibility.EdgeStart_units;
    edgeVector_units  = vertexVisibility.EdgeVector_units;
    closestPointFraction = sum(pointOffset_units .* edgeVector_units, 2) ./ sum(edgeVector_units .^ 2, 2);
    closestPointFraction = min(1, max(0, closestPointFraction));
    distanceToEdge_units = vecnorm(pointOffset_units - closestPointFraction .* edgeVector_units, 2, 2);
    pointIsClear        = ~pointIsInsideOrOnBoundary && all(distanceToEdge_units > tolerance_units);
end

function nodePairs = mapVertexPairsToNodes(vertexPairs, nodeIndexByVertex, vertexIsSeparateNode)
    % Translate obstacle-vertex indices into this graph's node indices.
    % Skip vertices merged with start or goal because those connections
    % were checked directly. Preserve the saved pair order.
    pairUsesSeparateVertices = vertexIsSeparateNode(vertexPairs(:, 1)) & vertexIsSeparateNode(vertexPairs(:, 2));
    nodePairs = reshape(nodeIndexByVertex(vertexPairs(pairUsesSeparateVertices, :)), [], 2);
end
