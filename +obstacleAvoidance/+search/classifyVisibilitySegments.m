function clear = classifyVisibilitySegments(skeleton, nodes_units, cones, firstNode, secondNodes)
%% Section 0: Header & Readme
% SYNTAX
%   clear = obstacleAvoidance.search.classifyVisibilitySegments( ...
%       skeleton, nodes_units, cones, firstNode, secondNodes)
%**************************************************************************
% PURPOSE
%   - Decide exactly which segments from one node to a set of nodes stay
%     clear of the occupied union: corner cones reject strictly inward
%     departures, every boundary contact partitions the segment, and each
%     partition midpoint is tested for interior occupancy.
%**************************************************************************
% INPUTS
%   - skeleton (scalar struct)
%       Occupied union and boundary edges from createVisibilitySkeleton.
%   - nodes_units (N-by-2 numeric array)
%       Node positions that firstNode and secondNodes index.
%   - cones (scalar struct)
%       Corner cones for nodes_units from createNodeCones.
%   - firstNode (positive integer scalar)
%       Index of the shared segment start.
%   - secondNodes (positive integer column)
%       Indices of the segment ends.
%**************************************************************************
% OUTPUTS
%   - clear (logical column)
%       True where the segment from firstNode to that node is clear.
%**************************************************************************
% UNITS
%   - Positions are coordinate units.
%**************************************************************************

%% Section 1: Reject Inward Departures, Then Test Every Contact Partition

locallyBlocked = entersObstacle(firstNode, secondNodes, nodes_units, cones, skeleton.Tolerance_units);
checkIndex     = find(~locallyBlocked);
clear          = false(size(secondNodes));
[checked, queryPoints_units, queryOwner] = segmentIntervals( ...
    nodes_units(firstNode, :), nodes_units(secondNodes(checkIndex), :), ...
    skeleton.EdgeStart_units, skeleton.EdgeEnd_units, skeleton.EdgeVector_units, ...
    skeleton.EdgeBounds_units, skeleton.ParallelTolerance_units2, skeleton.Tolerance_units);
if ~isempty(queryOwner)
    [inside, on] = inpolygon(queryPoints_units(:, 1), queryPoints_units(:, 2), ...
        skeleton.Boundary_units(:, 1), skeleton.Boundary_units(:, 2));
    checked(queryOwner(inside & ~on)) = false;
end
clear(checkIndex) = checked;
end

%% Section 2: Local Functions

function blocked = entersObstacle(first, last, nodes_units, cones, tolerance_units)
    % Strictly inward directions are certainly blocked. Tangencies and
    % ambiguous corners continue to the complete segment predicate.
    direction_units = nodes_units(last, :) - nodes_units(first, :);
    blocked = false(size(direction_units, 1), 1);
    endpoints = {repmat(first, size(last)), last};
    for endpointIndex = 1:2
        indices = endpoints{endpointIndex};
        incoming = cones.Incoming(indices, :);
        outgoing = cones.Outgoing(indices, :);
        side = cones.Side(indices);
        incomingSide = side .* (incoming(:, 1) .* direction_units(:, 2) - incoming(:, 2) .* direction_units(:, 1));
        outgoingSide = side .* (outgoing(:, 1) .* direction_units(:, 2) - outgoing(:, 2) .* direction_units(:, 1));
        coordinateScale_units = max(1, max(abs(nodes_units(indices, :)), [], 2));
        roundoff = 64 * eps(coordinateScale_units) .* max(1, vecnorm(direction_units, 2, 2));
        inward = incomingSide > (tolerance_units + roundoff) .* vecnorm(incoming, 2, 2);
        outward = outgoingSide > (tolerance_units + roundoff) .* vecnorm(outgoing, 2, 2);
        blocked = blocked | (cones.Enabled(indices) & ...
            ((cones.Convex(indices) & inward & outward) | ...
            (~cones.Convex(indices) & (inward | outward))));
        direction_units = -direction_units;
    end
end

function [clear, midpoints_units, owners] = segmentIntervals(first_units, last_units, edgeStart_units, edgeEnd_units, edge_units, bounds_units, parallelTolerance_units2, tolerance_units)
    % Partition each segment at every exact boundary contact.
    clear = true(size(last_units, 1), 1);
    midpoints_units = zeros(0, 2);
    owners = zeros(0, 1);
    if isempty(edgeStart_units)
        return;
    end
    offset_units = edgeStart_units - first_units;
    endOffset_units = edgeEnd_units - first_units;
    numerator_units2 = offset_units(:, 1) .* edge_units(:, 2) - offset_units(:, 2) .* edge_units(:, 1);
    % Bound temporary edge-by-candidate arrays independently of scene size.
    blockSize = max(1, floor(2^18 / size(edge_units, 1)));
    points = cell(size(last_units, 1), 1);
    pointOwners = points;
    for blockStart = 1:blockSize:size(last_units, 1)
        indices = blockStart:min(size(last_units, 1), blockStart + blockSize - 1);
        direction_units = last_units(indices, :) - first_units;
        lengths_units = vecnorm(direction_units, 2, 2).';
        parameterTolerance = tolerance_units ./ max(lengths_units, realmin);
        lower_units = min(first_units, last_units(indices, :)) - tolerance_units;
        upper_units = max(first_units, last_units(indices, :)) + tolerance_units;
        relevant = bounds_units(:, 3) >= lower_units(:, 1).' & bounds_units(:, 4) >= lower_units(:, 2).' & ...
            bounds_units(:, 1) <= upper_units(:, 1).' & bounds_units(:, 2) <= upper_units(:, 2).';
        % Rows outside every candidate AABB cannot intersect any segment in
        % this block. Remove only those rows before the exact cross products.
        relevantEdgeMask  = any(relevant, 2);
        relevant          = relevant(relevantEdgeMask, :);
        localEdges_units  = edge_units(relevantEdgeMask, :);
        localOffset_units = offset_units(relevantEdgeMask, :);
        localEndOffset_units = endOffset_units(relevantEdgeMask, :);
        denominator_units2 = localEdges_units(:, 2) * direction_units(:, 1).' - ...
            localEdges_units(:, 1) * direction_units(:, 2).';
        crossOffset_units2 = localOffset_units(:, 1) * direction_units(:, 2).' - ...
            localOffset_units(:, 2) * direction_units(:, 1).';
        nonparallel = abs(denominator_units2) > parallelTolerance_units2(relevantEdgeMask);
        t = numerator_units2(relevantEdgeMask) ./ denominator_units2;
        u = crossOffset_units2 ./ denominator_units2;
        tIsInterior = t > parameterTolerance & t < 1 - parameterTolerance;
        uIsInterior = u > parameterTolerance & u < 1 - parameterTolerance;
        crosses = relevant & nonparallel & tIsInterior & uIsInterior;
        clear(indices) = ~any(crosses, 1).';
        % Retain every contact-partition interval, including collinear edges.
        contact = relevant & nonparallel & t >= 0 & t <= 1 & ...
            u >= -parameterTolerance & u <= 1 + parameterTolerance;
        collinear = relevant & ~nonparallel & abs(crossOffset_units2) <= tolerance_units * lengths_units;
        for candidateIndex = find(clear(indices)).'
            projection = (localOffset_units(collinear(:, candidateIndex), :) * direction_units(candidateIndex, :).') / ...
                sum(direction_units(candidateIndex, :).^2);
            endProjection = (localEndOffset_units(collinear(:, candidateIndex), :) * direction_units(candidateIndex, :).') / ...
                sum(direction_units(candidateIndex, :).^2);
            cuts = unique([0; 1; t(contact(:, candidateIndex), candidateIndex); ...
                min(1, max(0, projection)); min(1, max(0, endProjection))]);
            midpointFraction = (cuts(1:end - 1) + cuts(2:end)) / 2;
            points{indices(candidateIndex)} = first_units + midpointFraction .* direction_units(candidateIndex, :);
            pointOwners{indices(candidateIndex)} = repmat(indices(candidateIndex), numel(cuts) - 1, 1);
        end
    end
    midpoints_units = vertcat(points{:});
    owners = vertcat(pointOwners{:});
end
