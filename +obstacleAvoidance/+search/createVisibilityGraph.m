function visibilityGraph = createVisibilityGraph(preparedObstacles, start_units, goal_units, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   visibilityGraph = obstacleAvoidance.search.createVisibilityGraph( ...
%       preparedObstacles, start_units, goal_units, limits, options)
%
% PURPOSE
%   - Build an exhaustive visibility graph around static convex obstacles.
%   - Return the shortest polyline from start to goal when one exists.
%
% INPUTS
%   - preparedObstacles: output from prepareObstacles.
%   - start_units, goal_units: finite 1-by-2 endpoint rows.
%   - limits: resolved xInterval_units and yInterval_units.
%   - options: resolved numerical tolerance.
%
% OUTPUTS
%   - visibilityGraph: nodes, accepted/rejected edges, costs, and Route_units.
%
% UNITS
%   - Positions and distances are coordinate units.

%% Section 1: Create Endpoint And Exact Boundary Nodes

tolerance_units = max(1e-12, options.ConstraintTolerance);
sourceFree = pointIsFree(start_units, preparedObstacles, tolerance_units);
goalFree   = pointIsFree(goal_units, preparedObstacles, tolerance_units);
nodePosition_units = [start_units; goal_units];
for obstacleIndex = 1:numel(preparedObstacles)
    candidate_units = preparedObstacles(obstacleIndex).ProtectedVertices_units;
    insideWorkspace = candidate_units(:, 1) >= limits.xInterval_units(1) & candidate_units(:, 1) <= limits.xInterval_units(2) & candidate_units(:, 2) >= limits.yInterval_units(1) & candidate_units(:, 2) <= limits.yInterval_units(2);
    candidate_units = candidate_units(insideWorkspace, :);
    keep = false(size(candidate_units, 1), 1);
    for candidateIndex = 1:size(candidate_units, 1)
        keep(candidateIndex) = boundaryNodeIsAvailable(candidate_units(candidateIndex, :), obstacleIndex, preparedObstacles, tolerance_units);
    end
    nodePosition_units = [nodePosition_units; candidate_units(keep, :)]; %#ok<AGROW>
end
nodePosition_units = unique(nodePosition_units, "rows", "stable");
nodeCount = size(nodePosition_units, 1);

%% Section 2: Check Every Candidate Segment

maximumEdgeCount = nodeCount * (nodeCount - 1) / 2;
acceptedNodeIndex = zeros(maximumEdgeCount, 2);
acceptedWeight_units = zeros(maximumEdgeCount, 1);
rejectedNodeIndex = zeros(maximumEdgeCount, 2);
acceptedCount = 0;
rejectedCount = 0;
if sourceFree && goalFree
    for firstNodeIndex = 1:nodeCount - 1
        for secondNodeIndex = firstNodeIndex + 1:nodeCount
            first_units  = nodePosition_units(firstNodeIndex, :);
            second_units = nodePosition_units(secondNodeIndex, :);
            if segmentIsClear(first_units, second_units, preparedObstacles, tolerance_units)
                acceptedCount = acceptedCount + 1;
                acceptedNodeIndex(acceptedCount, :) = [firstNodeIndex secondNodeIndex];
                acceptedWeight_units(acceptedCount) = norm(second_units - first_units);
            else
                rejectedCount = rejectedCount + 1;
                rejectedNodeIndex(rejectedCount, :) = [firstNodeIndex secondNodeIndex];
            end
        end
    end
end
acceptedNodeIndex = acceptedNodeIndex(1:acceptedCount, :);
acceptedWeight_units = acceptedWeight_units(1:acceptedCount);
rejectedNodeIndex = rejectedNodeIndex(1:rejectedCount, :);

%% Section 3: Select The Shortest Visibility Route

routeNodeIndex = zeros(1, 0);
routeLength_units = Inf;
if sourceFree && goalFree
    visibilityNetwork = graph(acceptedNodeIndex(:, 1), acceptedNodeIndex(:, 2), acceptedWeight_units, nodeCount);
    [routeNodeIndex, routeLength_units] = shortestpath(visibilityNetwork, 1, 2, "Method", "positive");
end
route_units = zeros(0, 2);
if ~isempty(routeNodeIndex)
    route_units = nodePosition_units(routeNodeIndex, :);
end
visibilityGraph = struct();
visibilityGraph.NodePosition_units    = nodePosition_units;
visibilityGraph.AcceptedNodeIndex     = acceptedNodeIndex;
visibilityGraph.AcceptedWeight_units  = acceptedWeight_units;
visibilityGraph.RejectedNodeIndex     = rejectedNodeIndex;
visibilityGraph.RouteNodeIndex        = routeNodeIndex;
visibilityGraph.Route_units           = route_units;
visibilityGraph.RouteLength_units     = routeLength_units;
visibilityGraph.SourceFree            = sourceFree;
visibilityGraph.GoalFree              = goalFree;
visibilityGraph.IsConnected           = ~isempty(routeNodeIndex);
end

%% Section 4: Local Functions

function clear = pointIsFree(point_units, preparedObstacles, tolerance_units)
    % Treat obstacle interiors and boundaries as occupied.
    clear = true;
    for obstacleIndex = 1:numel(preparedObstacles)
        shape = preparedObstacles(obstacleIndex).ProtectedShape;
        if isinterior(shape, point_units(1), point_units(2)) || pointTouchesBoundary(point_units, preparedObstacles(obstacleIndex).ProtectedVertices_units, tolerance_units)
            clear = false;
            return;
        end
    end
end

function available = boundaryNodeIsAvailable(point_units, ownerIndex, preparedObstacles, tolerance_units)
    % Keep an exact owner vertex unless another obstacle occupies it.
    available = true;
    for obstacleIndex = 1:numel(preparedObstacles)
        if obstacleIndex == ownerIndex
            continue;
        end
        shape = preparedObstacles(obstacleIndex).ProtectedShape;
        if isinterior(shape, point_units(1), point_units(2)) || pointTouchesBoundary(point_units, preparedObstacles(obstacleIndex).ProtectedVertices_units, tolerance_units)
            available = false;
            return;
        end
    end
end

function touches = pointTouchesBoundary(point_units, vertices_units, tolerance_units)
    % Check point-to-edge distance on the closed polygon ring.
    touches = false;
    for edgeIndex = 1:size(vertices_units, 1)
        nextIndex = mod(edgeIndex, size(vertices_units, 1)) + 1;
        if pointSegmentDistance(point_units, vertices_units(edgeIndex, :), vertices_units(nextIndex, :)) <= tolerance_units
            touches = true;
            return;
        end
    end
end

function distance_units = pointSegmentDistance(point_units, first_units, second_units)
    % Return Euclidean distance from one point to a closed segment.
    edge_units = second_units - first_units;
    denominator_units2 = dot(edge_units, edge_units);
    if denominator_units2 == 0
        distance_units = norm(point_units - first_units);
        return;
    end
    fraction = min(1, max(0, dot(point_units - first_units, edge_units) / denominator_units2));
    distance_units = norm(point_units - (first_units + fraction * edge_units));
end

function clear = segmentIsClear(first_units, second_units, preparedObstacles, tolerance_units)
    % Allow tangent endpoint contacts and polygon edges, but never an interior crossing.
    clear = true;
    for obstacleIndex = 1:numel(preparedObstacles)
        vertices_units = preparedObstacles(obstacleIndex).ProtectedVertices_units;
        for edgeIndex = 1:size(vertices_units, 1)
            nextIndex = mod(edgeIndex, size(vertices_units, 1)) + 1;
            if segmentEdgeContactIsForbidden(first_units, second_units, vertices_units(edgeIndex, :), vertices_units(nextIndex, :), tolerance_units)
                clear = false;
                return;
            end
        end
        midpoint_units = 0.5 * (first_units + second_units);
        midpointOnBoundary = pointTouchesBoundary(midpoint_units, vertices_units, tolerance_units);
        if isinterior(preparedObstacles(obstacleIndex).ProtectedShape, midpoint_units(1), midpoint_units(2)) && ~midpointOnBoundary
            clear = false;
            return;
        end
    end
end

function forbidden = segmentEdgeContactIsForbidden(firstStart_units, firstEnd_units, secondStart_units, secondEnd_units, tolerance_units)
    % Permit contact only at a candidate endpoint or along one polygon edge.
    [intersects, collinear, overlapLength_units] = segmentIntersection(firstStart_units, firstEnd_units, secondStart_units, secondEnd_units, tolerance_units);
    if ~intersects
        forbidden = false;
        return;
    end
    firstOnEdge  = pointSegmentDistance(firstStart_units, secondStart_units, secondEnd_units) <= tolerance_units;
    secondOnEdge = pointSegmentDistance(firstEnd_units, secondStart_units, secondEnd_units) <= tolerance_units;
    if collinear && overlapLength_units > tolerance_units
        forbidden = ~(firstOnEdge && secondOnEdge);
        return;
    end
    firstIsEdgeVertex = norm(firstStart_units - secondStart_units) <= tolerance_units || norm(firstStart_units - secondEnd_units) <= tolerance_units;
    secondIsEdgeVertex = norm(firstEnd_units - secondStart_units) <= tolerance_units || norm(firstEnd_units - secondEnd_units) <= tolerance_units;
    forbidden = ~(firstOnEdge && firstIsEdgeVertex) && ~(secondOnEdge && secondIsEdgeVertex);
end

function [intersects, collinear, overlapLength_units] = segmentIntersection(firstStart_units, firstEnd_units, secondStart_units, secondEnd_units, tolerance_units)
    % Solve the two-segment intersection parameters, including collinear overlap.
    firstDirection_units  = firstEnd_units - firstStart_units;
    secondDirection_units = secondEnd_units - secondStart_units;
    offset_units          = secondStart_units - firstStart_units;
    denominator_units2    = cross2(firstDirection_units, secondDirection_units);
    scale_units           = max([1, norm(firstDirection_units), norm(secondDirection_units)]);
    crossTolerance_units2 = tolerance_units * scale_units;
    if abs(denominator_units2) > crossTolerance_units2
        firstFraction  = cross2(offset_units, secondDirection_units) / denominator_units2;
        secondFraction = cross2(offset_units, firstDirection_units) / denominator_units2;
        parameterTolerance = tolerance_units / scale_units;
        intersects = firstFraction >= -parameterTolerance && firstFraction <= 1 + parameterTolerance && secondFraction >= -parameterTolerance && secondFraction <= 1 + parameterTolerance;
        collinear = false;
        overlapLength_units = 0;
        return;
    end
    if abs(cross2(offset_units, firstDirection_units)) > crossTolerance_units2
        intersects = false;
        collinear = false;
        overlapLength_units = 0;
        return;
    end
    collinear = true;
    firstLength_units2 = dot(firstDirection_units, firstDirection_units);
    if firstLength_units2 == 0
        intersects = pointSegmentDistance(firstStart_units, secondStart_units, secondEnd_units) <= tolerance_units;
        overlapLength_units = 0;
        return;
    end
    projection = [dot(secondStart_units - firstStart_units, firstDirection_units), dot(secondEnd_units - firstStart_units, firstDirection_units)] / firstLength_units2;
    overlapFraction = min(max(projection), 1) - max(min(projection), 0);
    intersects = overlapFraction >= -tolerance_units / scale_units;
    overlapLength_units = max(0, overlapFraction) * norm(firstDirection_units);
end

function value = cross2(first_units, second_units)
    % Return the signed two-dimensional cross product.
    value = first_units(1) * second_units(2) - first_units(2) * second_units(1);
end
