function visibilityGraph = createVisibilityGraph(skeleton, start_units, goal_units)
%% Section 0: Header & Readme
% SYNTAX
%   visibilityGraph = obstacleAvoidance.search.createVisibilityGraph( ...
%       skeleton, start_units, goal_units)
%**************************************************************************
% PURPOSE
%   - Attach one start and one goal to a visibility skeleton and select the
%     shortest exact route through the classified vertex pairs.
%**************************************************************************
% INPUTS
%   - skeleton (scalar struct)
%       Classified vertex visibility from createVisibilitySkeleton.
%   - start_units (1-by-2 numeric row)
%       Route start position.
%   - goal_units (1-by-2 numeric row)
%       Route goal position.
%**************************************************************************
% OUTPUTS
%   - visibilityGraph (scalar struct)
%       Boundary nodes, classified edges, shortest route, and connectivity.
%       A disconnected graph returns IsConnected = false and an empty route;
%       invalid input throws an error.
%**************************************************************************
% UNITS
%   - Geometry and route lengths are coordinate units.
%**************************************************************************

%% Section 1: Place The Endpoints Among The Skeleton Vertices

validateattributes(start_units, {'numeric'}, {'real', 'finite', 'size', [1, 2]});
validateattributes(goal_units, {'numeric'}, {'real', 'finite', 'size', [1, 2]});
tolerance_units = skeleton.Tolerance_units;
nodes_units     = unique([start_units; goal_units; skeleton.Vertices_units], 'rows', 'stable');
sourceFree      = pointIsFree(start_units, skeleton, tolerance_units);
goalFree        = pointIsFree(goal_units, skeleton, tolerance_units);
endpoints_units = [start_units; goal_units];
workspaceFree   = all(endpoints_units >= skeleton.WorkspaceMinimum_units & ...
    endpoints_units <= skeleton.WorkspaceMaximum_units, 2);
sourceFree = sourceFree && workspaceFree(1);
goalFree   = goalFree && workspaceFree(2);

%% Section 2: Classify The Endpoint Segments And Read Back The Vertex Pairs

nodeCount     = size(nodes_units, 1);
accepted      = zeros(0, 2);
weights_units = zeros(0, 1);
rejected      = zeros(0, 2);
if sourceFree && goalFree
    cones         = obstacleAvoidance.search.createNodeCones(skeleton, nodes_units);
    acceptedParts = cell(3, 1);
    rejectedParts = cell(3, 1);
    for firstNode = 1:2
        secondNode = (firstNode + 1:nodeCount).';
        clear      = obstacleAvoidance.search.classifyVisibilitySegments( ...
            skeleton, nodes_units, cones, firstNode, secondNode);
        acceptedParts{firstNode} = [repmat(firstNode, nnz(clear), 1), secondNode(clear)];
        rejectedParts{firstNode} = [repmat(firstNode, nnz(~clear), 1), secondNode(~clear)];
    end
    % A vertex that coincides with an endpoint was merged into node 1 or 2
    % and its segments were classified above; every other vertex pair keeps
    % the skeleton's verdict, renumbered into this node list.
    [~, nodeOfVertex] = ismember(skeleton.Vertices_units, nodes_units, 'rows');
    vertexRetained    = nodeOfVertex > 2;
    acceptedParts{3}  = renumberPairs(skeleton.VertexPairAccepted, nodeOfVertex, vertexRetained);
    rejectedParts{3}  = renumberPairs(skeleton.VertexPairRejected, nodeOfVertex, vertexRetained);
    accepted      = vertcat(acceptedParts{:});
    rejected      = vertcat(rejectedParts{:});
    weights_units = vecnorm(nodes_units(accepted(:, 2), :) - nodes_units(accepted(:, 1), :), 2, 2);
end

%% Section 3: Select The Shortest Route And Return Full Evidence

routeIndex        = zeros(1, 0);
route_units       = zeros(0, 2);
routeLength_units = Inf;
if sourceFree && goalFree
    visibilityNetwork = graph(accepted(:, 1), accepted(:, 2), weights_units, nodeCount);
    [routeIndex, routeLength_units] = shortestpath(visibilityNetwork, 1, 2, 'Method', 'positive');
    route_units = nodes_units(routeIndex, :);
end
visibilityGraph = struct( ...
    'NodePosition_units',    nodes_units, ...
    'AcceptedNodeIndex',     accepted, ...
    'RejectedNodeIndex',     rejected, ...
    'Route_units',           route_units, ...
    'RouteLength_units',     routeLength_units, ...
    'IsConnected',           ~isempty(routeIndex), ...
    'ExpandedCount',         nodeCount, ...
    'GraphIsFullyEnumerated', sourceFree && goalFree);
end

%% Section 4: Local Functions

function free = pointIsFree(point_units, skeleton, tolerance_units)
    % Boundary contact is blocked at the graph's exact clearance tolerance.
    if isempty(skeleton.EdgeStart_units)
        free = true;
        return;
    end
    inside = inpolygon(point_units(1), point_units(2), ...
        skeleton.Boundary_units(:, 1), skeleton.Boundary_units(:, 2));
    edge_units = skeleton.EdgeVector_units;
    fraction = min(1, max(0, sum((point_units - skeleton.EdgeStart_units) .* edge_units, 2) ./ sum(edge_units.^2, 2)));
    distance_units = vecnorm(point_units - skeleton.EdgeStart_units - fraction .* edge_units, 2, 2);
    free = ~inside && all(distance_units > tolerance_units);
end

function pairs = renumberPairs(vertexPairs, nodeOfVertex, vertexRetained)
    % Keep the skeleton's pair order; it already ascends by first then
    % second vertex, and node numbering preserves vertex order.
    keep  = vertexRetained(vertexPairs(:, 1)) & vertexRetained(vertexPairs(:, 2));
    pairs = reshape(nodeOfVertex(vertexPairs(keep, :)), [], 2);
end
