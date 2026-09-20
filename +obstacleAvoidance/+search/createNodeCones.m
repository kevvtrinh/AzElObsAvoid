function cones = createNodeCones(skeleton, nodes_units)
%% Section 0: Header & Readme
% SYNTAX
%   cones = obstacleAvoidance.search.createNodeCones(skeleton, nodes_units)
%**************************************************************************
% PURPOSE
%   - Describe the boundary corner at each node so a segment that leaves
%     it strictly into the occupied side can be rejected without any
%     intersection test.
%**************************************************************************
% INPUTS
%   - skeleton (scalar struct)
%       Boundary edges and their occupied sides from createVisibilitySkeleton.
%   - nodes_units (N-by-2 numeric array)
%       Node positions. A node that is not a boundary vertex gets a
%       disabled cone.
%**************************************************************************
% OUTPUTS
%   - cones (scalar struct)
%       Incoming and outgoing edge vectors, occupied side, convexity, and
%       an Enabled flag per node, each indexed like nodes_units.
%**************************************************************************
% UNITS
%   - Positions and edge vectors are coordinate units.
%**************************************************************************

%% Section 1: Match Each Node To Its Boundary Edges

count = size(nodes_units, 1);
cones = struct('Incoming', zeros(count, 2), 'Outgoing', zeros(count, 2), ...
    'Side', zeros(count, 1), 'Convex', false(count, 1), 'Enabled', false(count, 1));
if isempty(skeleton.EdgeStart_units)
    return
end
[inFound, incoming]  = ismember(nodes_units, skeleton.EdgeEnd_units, 'rows');
[outFound, outgoing] = ismember(nodes_units, skeleton.EdgeStart_units, 'rows');
[mapped, node]       = ismember(skeleton.EdgeEnd_units, nodes_units, 'rows');
inCount              = accumarray(node(mapped), 1, [count, 1]);
[mapped, node]       = ismember(skeleton.EdgeStart_units, nodes_units, 'rows');
outCount             = accumarray(node(mapped), 1, [count, 1]);
incoming             = max(1, incoming);
outgoing             = max(1, outgoing);

%% Section 2: Resolve Each Corner's Turn And Occupied Side

cones.Incoming = skeleton.EdgeVector_units(incoming, :);
cones.Outgoing = skeleton.EdgeVector_units(outgoing, :);
cones.Side     = skeleton.EdgeSide(incoming);
turn_units2    = cones.Side .* (cones.Incoming(:, 1) .* cones.Outgoing(:, 2) - ...
    cones.Incoming(:, 2) .* cones.Outgoing(:, 1));
hasOneIncoming    = inCount == 1;
hasOneOutgoing    = outCount == 1;
hasConsistentSide = skeleton.EdgeSide(incoming) == skeleton.EdgeSide(outgoing) & ...
    skeleton.EdgeSide(incoming) ~= 0;
turnScale_units2  = vecnorm(cones.Incoming, 2, 2) .* vecnorm(cones.Outgoing, 2, 2);
turnIsResolved    = abs(turn_units2) > skeleton.Tolerance_units * max(1, turnScale_units2);
cones.Enabled = inFound & outFound & hasOneIncoming & hasOneOutgoing & hasConsistentSide & turnIsResolved;
cones.Convex  = turn_units2 > 0;
end
