function nodeCornerDirections = createNodeCones(vertexVisibility, nodePositions_units)
%% Section 0: Header & Readme
% SYNTAX
%   nodeCornerDirections = obstacleAvoidance.search.createNodeCones( ...
%       vertexVisibility, nodePositions_units)
%**************************************************************************
% PURPOSE
%   - Record the two edges meeting at each obstacle corner and which side
%     they enclose. The visibility check uses this to reject a connection
%     that immediately enters the obstacle from a corner.
%**************************************************************************
% INPUTS
%   - vertexVisibility (scalar struct)
%       Boundary edges and their occupied sides from createVertexVisibility.
%   - nodePositions_units (N-by-2 numeric array)
%       Route-point positions. Points away from a boundary corner do not
%       use this direction check.
%**************************************************************************
% OUTPUTS
%   - nodeCornerDirections (scalar struct)
%       Incoming and outgoing edge vectors, occupied side, and outward-corner
%       flag (Convex) for each node. Enabled is false when the two edges do
%       not establish a clear corner; the full segment check still applies.
%**************************************************************************
% UNITS
%   - Positions and edge vectors are coordinate units.
%**************************************************************************

%% Section 1: Match Each Node To Its Boundary Edges

nodeCount = size(nodePositions_units, 1);
nodeCornerDirections = struct( ...
    'Incoming', zeros(nodeCount, 2), ...
    'Outgoing', zeros(nodeCount, 2), ...
    'Side',     zeros(nodeCount, 1), ...
    'Convex',   false(nodeCount, 1), ...
    'Enabled',  false(nodeCount, 1));
if isempty(vertexVisibility.EdgeStart_units)
    return
end

% An incoming edge ends at the node; an outgoing edge starts there.
% Count every match so a shared junction is not mistaken for a simple corner.
[hasIncomingEdge, incomingEdgeIndices] = ismember(nodePositions_units, vertexVisibility.EdgeEnd_units, 'rows');
[hasOutgoingEdge, outgoingEdgeIndices] = ismember(nodePositions_units, vertexVisibility.EdgeStart_units, 'rows');
[boundaryEndpointMatchesNode, matchedNodeIndices] = ...
    ismember(vertexVisibility.EdgeEnd_units, nodePositions_units, 'rows');
incomingEdgeCounts = accumarray(matchedNodeIndices(boundaryEndpointMatchesNode), 1, [nodeCount, 1]);
[boundaryEndpointMatchesNode, matchedNodeIndices] = ...
    ismember(vertexVisibility.EdgeStart_units, nodePositions_units, 'rows');
outgoingEdgeCounts = accumarray(matchedNodeIndices(boundaryEndpointMatchesNode), 1, [nodeCount, 1]);

% Missing matches return index 0. Use a valid placeholder for reading arrays;
% Enabled below stays false for these nodes.
incomingEdgeIndices = max(1, incomingEdgeIndices);
outgoingEdgeIndices = max(1, outgoingEdgeIndices);

%% Section 2: Resolve Each Corner's Turn And Occupied Side

nodeCornerDirections.Incoming = vertexVisibility.EdgeVector_units(incomingEdgeIndices, :);
nodeCornerDirections.Outgoing = vertexVisibility.EdgeVector_units(outgoingEdgeIndices, :);
nodeCornerDirections.Side     = vertexVisibility.EdgeSide(incomingEdgeIndices);

% Multiplying the turn by the occupied-side sign gives the same corner
% classification for either clockwise or counterclockwise boundaries.
% Positive means an outward corner; negative means an inward corner.
cornerTurn_units2 = nodeCornerDirections.Side .* ...
    (nodeCornerDirections.Incoming(:, 1) .* nodeCornerDirections.Outgoing(:, 2) - ...
    nodeCornerDirections.Incoming(:, 2) .* nodeCornerDirections.Outgoing(:, 1));

% Only use this shortcut for one clear pair of edges. Nearly straight
% corners, unknown occupied sides, and multiple matches use the full check.
hasOneIncomingEdge = incomingEdgeCounts == 1;
hasOneOutgoingEdge = outgoingEdgeCounts == 1;
edgesHaveSameOccupiedSide = ...
    vertexVisibility.EdgeSide(incomingEdgeIndices) == vertexVisibility.EdgeSide(outgoingEdgeIndices) & ...
    vertexVisibility.EdgeSide(incomingEdgeIndices) ~= 0;
cornerTurnScale_units2     = vecnorm(nodeCornerDirections.Incoming, 2, 2) .* vecnorm(nodeCornerDirections.Outgoing, 2, 2);
cornerTurnExceedsTolerance = abs(cornerTurn_units2) > vertexVisibility.Tolerance_units * max(1, cornerTurnScale_units2);
nodeCornerDirections.Enabled = hasIncomingEdge & hasOutgoingEdge & ...
    hasOneIncomingEdge & hasOneOutgoingEdge & edgesHaveSameOccupiedSide & cornerTurnExceedsTolerance;
nodeCornerDirections.Convex  = cornerTurn_units2 > 0;
end
