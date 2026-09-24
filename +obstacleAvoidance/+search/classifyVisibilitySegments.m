function isVisible = classifyVisibilitySegments( ...
    vertexVisibility, nodePositions_units, nodeCornerDirections, startNodeIndex, endNodeIndices)
%% Section 0: Header & Readme
% SYNTAX
%   isVisible = obstacleAvoidance.search.classifyVisibilitySegments( ...
%       vertexVisibility, nodePositions_units, nodeCornerDirections, startNodeIndex, endNodeIndices)
%**************************************************************************
% PURPOSE
%   - Find connections from one node to several others that avoid obstacle
%     interiors. The visibility graph may touch or follow a boundary.
%   - Keep only connections that are tangent at each corner they touch. A
%     shortest route never bends at a corner it cuts through, so this gives
%     the reduced visibility graph: the same shortest routes with far fewer
%     connections to check.
%   - Reject directions that enter an obstacle or cut a corner at either
%     endpoint, then split each remaining segment at its boundary contacts
%     and test the pieces. The returned motion still needs independent
%     collision validation.
%**************************************************************************
% INPUTS
%   - vertexVisibility (scalar struct)
%       Combined obstacle shape and boundary edges from createVertexVisibility.
%   - nodePositions_units (N-by-2 numeric array)
%       Positions indexed by startNodeIndex and endNodeIndices.
%   - nodeCornerDirections (scalar struct)
%       The two edges at each corner and the side occupied by the obstacle,
%       as returned by createNodeCones.
%   - startNodeIndex (positive integer scalar)
%       Index of the shared segment start.
%   - endNodeIndices (positive integer column)
%       Indices of the segment ends.
%**************************************************************************
% OUTPUTS
%   - isVisible (logical column)
%       True where the segment to that end node avoids the obstacle interior
%       and is tangent at each corner it touches.
%**************************************************************************
% UNITS
%   - Positions are coordinate units.
%**************************************************************************

%% Section 1: Reject Segments That Enter An Obstacle Or Cut A Corner

% A shortest route turns at a corner only when it continues straight past
% it, so the line through the segment keeps both boundary edges at that
% corner on one side. Every other segment is dropped before the full check.
segmentIsRejected = segmentEntersObstacleAtEndpoint( ...
    startNodeIndex, endNodeIndices, nodePositions_units, nodeCornerDirections, vertexVisibility.Tolerance_units) | ...
    ~segmentIsTangentAtCorners( ...
    startNodeIndex, endNodeIndices, nodePositions_units, nodeCornerDirections, vertexVisibility.Tolerance_units);
segmentsToCheck = find(~segmentIsRejected);
isVisible       = false(size(endNodeIndices));

%% Section 2: Check The Pieces Between Boundary Contacts

% The helper rejects crossings through boundary edges and returns the
% remaining pieces' midpoints. Each midpoint keeps its segment index so
% an inside point can reject the correct connection.
[segmentIsVisible, intervalMidpoints_units, midpointSegmentIndices] = splitSegmentsAtBoundaryContacts( ...
    nodePositions_units(startNodeIndex, :), nodePositions_units(endNodeIndices(segmentsToCheck), :), ...
    vertexVisibility.EdgeStart_units, vertexVisibility.EdgeEnd_units, vertexVisibility.EdgeVector_units, ...
    vertexVisibility.EdgeBounds_units, vertexVisibility.ParallelTolerance_units2, vertexVisibility.Tolerance_units);
if ~isempty(midpointSegmentIndices)
    [pointIsInsideOrOnBoundary, pointIsOnBoundary] = inpolygon( ...
        intervalMidpoints_units(:, 1), intervalMidpoints_units(:, 2), ...
        vertexVisibility.Boundary_units(:, 1), vertexVisibility.Boundary_units(:, 2));
    % A point on the boundary is allowed in this route graph. Only points
    % strictly inside the filled shape reject a connection.
    segmentIsVisible(midpointSegmentIndices(pointIsInsideOrOnBoundary & ~pointIsOnBoundary)) = false;
end
isVisible(segmentsToCheck) = segmentIsVisible;
end

%% Section 3: Local Functions

function endpointEntersObstacle = segmentEntersObstacleAtEndpoint( ...
        startNodeIndex, endNodeIndices, nodePositions_units, nodeCornerDirections, tolerance_units)
    % Check the direction leaving each endpoint. Directions along an edge
    % or too close to its tolerance continue to the full segment check.
    segmentVector_units  = nodePositions_units(endNodeIndices, :) - nodePositions_units(startNodeIndex, :);
    endpointEntersObstacle = false(size(segmentVector_units, 1), 1);
    endpointIndexLists     = {repmat(startNodeIndex, size(endNodeIndices)), endNodeIndices};
    for endpointIndex = 1:2
        endpointNodeIndices       = endpointIndexLists{endpointIndex};
        incomingEdgeVectors_units = nodeCornerDirections.Incoming(endpointNodeIndices, :);
        outgoingEdgeVectors_units = nodeCornerDirections.Outgoing(endpointNodeIndices, :);
        occupiedSideSigns         = nodeCornerDirections.Side(endpointNodeIndices);
        incomingSideValue_units2 = occupiedSideSigns .* ...
            (incomingEdgeVectors_units(:, 1) .* segmentVector_units(:, 2) - ...
            incomingEdgeVectors_units(:, 2) .* segmentVector_units(:, 1));
        outgoingSideValue_units2 = occupiedSideSigns .* ...
            (outgoingEdgeVectors_units(:, 1) .* segmentVector_units(:, 2) - ...
            outgoingEdgeVectors_units(:, 2) .* segmentVector_units(:, 1));
        coordinateScale_units      = max(1, max(abs(nodePositions_units(endpointNodeIndices, :)), [], 2));
        directionRoundingAllowance = 64 * eps(coordinateScale_units) .* max(1, vecnorm(segmentVector_units, 2, 2));
        insideIncomingEdge = incomingSideValue_units2 > ...
            (tolerance_units + directionRoundingAllowance) .* vecnorm(incomingEdgeVectors_units, 2, 2);
        insideOutgoingEdge = outgoingSideValue_units2 > ...
            (tolerance_units + directionRoundingAllowance) .* vecnorm(outgoingEdgeVectors_units, 2, 2);

        % At an outward-pointing corner, both edge tests must point inside.
        % At an inward corner, entering either occupied side is enough.
        endpointEntersObstacle = endpointEntersObstacle | (nodeCornerDirections.Enabled(endpointNodeIndices) & ...
            ((nodeCornerDirections.Convex(endpointNodeIndices) & insideIncomingEdge & insideOutgoingEdge) | ...
            (~nodeCornerDirections.Convex(endpointNodeIndices) & (insideIncomingEdge | insideOutgoingEdge))));

        % Reverse direction to check the same segment from its other end.
        segmentVector_units  = -segmentVector_units;
    end
end

function segmentIsTangent = segmentIsTangentAtCorners( ...
        startNodeIndex, endNodeIndices, nodePositions_units, nodeCornerDirections, tolerance_units)
    % At each corner the segment touches, compare the two boundary edges
    % leaving that corner against the segment line. Edges on opposite sides
    % mean the line cuts through the corner, so the segment is not tangent.
    % Nodes without a clear corner (Enabled false) are never rejected here.
    segmentVector_units = nodePositions_units(endNodeIndices, :) - nodePositions_units(startNodeIndex, :);
    segmentIsTangent    = true(size(segmentVector_units, 1), 1);
    endpointIndexLists  = {repmat(startNodeIndex, size(endNodeIndices)), endNodeIndices};
    for endpointIndex = 1:2
        endpointNodeIndices = endpointIndexLists{endpointIndex};
        % Both edges as directions leaving the corner.
        towardPreviousVertex_units = -nodeCornerDirections.Incoming(endpointNodeIndices, :);
        towardNextVertex_units     = nodeCornerDirections.Outgoing(endpointNodeIndices, :);

        % Cross product / edge length = distance of the edge end from the
        % segment line. Within tolerance counts as on the line (side 0).
        coordinateScale_units      = max(1, max(abs(nodePositions_units(endpointNodeIndices, :)), [], 2));
        directionRoundingAllowance = 64 * eps(coordinateScale_units) .* max(1, vecnorm(segmentVector_units, 2, 2));
        sideAllowance_units        = tolerance_units + directionRoundingAllowance;
        previousSideValue_units2 = segmentVector_units(:, 1) .* towardPreviousVertex_units(:, 2) - ...
            segmentVector_units(:, 2) .* towardPreviousVertex_units(:, 1);
        nextSideValue_units2 = segmentVector_units(:, 1) .* towardNextVertex_units(:, 2) - ...
            segmentVector_units(:, 2) .* towardNextVertex_units(:, 1);
        previousSide = sign(previousSideValue_units2) .* (abs(previousSideValue_units2) > ...
            sideAllowance_units .* vecnorm(towardPreviousVertex_units, 2, 2));
        nextSide = sign(nextSideValue_units2) .* (abs(nextSideValue_units2) > ...
            sideAllowance_units .* vecnorm(towardNextVertex_units, 2, 2));

        segmentIsTangent = segmentIsTangent & ...
            ~(nodeCornerDirections.Enabled(endpointNodeIndices) & previousSide .* nextSide < 0);
        segmentVector_units = -segmentVector_units;
    end
end

function [isVisible, intervalMidpoints_units, midpointSegmentIndices] = ...
        splitSegmentsAtBoundaryContacts(segmentStart_units, segmentEnd_units, ...
        boundaryStart_units, boundaryEnd_units, boundaryVector_units, boundaryBounds_units, ...
        parallelTolerance_units2, tolerance_units)
    % Split each surviving segment at every boundary contact. Its pieces
    % cannot change between inside and outside without another contact,
    % so one midpoint per piece is enough for the final polygon check.
    isVisible = true(size(segmentEnd_units, 1), 1);

    intervalMidpoints_units = zeros(0, 2);
    midpointSegmentIndices  = zeros(0, 1);
    if isempty(boundaryStart_units)
        return;
    end
    boundaryStartOffset_units         = boundaryStart_units - segmentStart_units;
    boundaryEndOffset_units           = boundaryEnd_units - segmentStart_units;
    boundaryOffsetCrossProduct_units2 = boundaryStartOffset_units(:, 1) .* boundaryVector_units(:, 2) - ...
        boundaryStartOffset_units(:, 2) .* boundaryVector_units(:, 1);

    % Limit temporary edge x segment arrays by processing small batches.
    % This controls memory use; every candidate segment is still checked.
    segmentsPerBatch        = max(1, floor(2^18 / size(boundaryVector_units, 1)));
    midpointsBySegment      = cell(size(segmentEnd_units, 1), 1);
    midpointOwnersBySegment = midpointsBySegment;
    for firstSegmentIndex = 1:segmentsPerBatch:size(segmentEnd_units, 1)
        segmentIndices       = firstSegmentIndex:min(size(segmentEnd_units, 1), firstSegmentIndex + segmentsPerBatch - 1);
        segmentVector_units  = segmentEnd_units(segmentIndices, :) - segmentStart_units;
        segmentLengths_units = vecnorm(segmentVector_units, 2, 2).';

        intersectionFractionTolerance = tolerance_units ./ max(segmentLengths_units, realmin);
        segmentMinimum_units = min(segmentStart_units, segmentEnd_units(segmentIndices, :)) - tolerance_units;
        segmentMaximum_units = max(segmentStart_units, segmentEnd_units(segmentIndices, :)) + tolerance_units;
        edgeBoxesOverlap   = boundaryBounds_units(:, 3) >= segmentMinimum_units(:, 1).' & ...
            boundaryBounds_units(:, 4) >= segmentMinimum_units(:, 2).' & ...
            boundaryBounds_units(:, 1) <= segmentMaximum_units(:, 1).' & ...
            boundaryBounds_units(:, 2) <= segmentMaximum_units(:, 2).';

        % A boundary edge cannot meet a segment whose x/y box is separate.
        % Remove edges outside all boxes in this batch before doing the
        % intersection calculations.
        edgeCouldMeetBatch = any(edgeBoxesOverlap, 2);
        edgeBoxesOverlap   = edgeBoxesOverlap(edgeCouldMeetBatch, :);

        candidateBoundaryVectors_units = boundaryVector_units(edgeCouldMeetBatch, :);
        candidateStartOffsets_units    = boundaryStartOffset_units(edgeCouldMeetBatch, :);
        candidateEndOffsets_units      = boundaryEndOffset_units(edgeCouldMeetBatch, :);
        directionCrossProduct_units2 = candidateBoundaryVectors_units(:, 2) * segmentVector_units(:, 1).' - ...
            candidateBoundaryVectors_units(:, 1) * segmentVector_units(:, 2).';
        lineOffsetCrossProduct_units2 = candidateStartOffsets_units(:, 1) * segmentVector_units(:, 2).' - ...
            candidateStartOffsets_units(:, 2) * segmentVector_units(:, 1).';
        pairIsNonparallel = abs(directionCrossProduct_units2) > parallelTolerance_units2(edgeCouldMeetBatch);

        % Fractions locate the crossing on each finite line: 0 = its start,
        % 1 = its end. Crossing through both line interiors enters an obstacle.
        segmentIntersectionFraction = ...
            boundaryOffsetCrossProduct_units2(edgeCouldMeetBatch) ./ directionCrossProduct_units2;
        boundaryIntersectionFraction = lineOffsetCrossProduct_units2 ./ directionCrossProduct_units2;
        intersectionIsWithinSegment  = segmentIntersectionFraction > intersectionFractionTolerance & ...
            segmentIntersectionFraction < 1 - intersectionFractionTolerance;
        intersectionIsWithinBoundary = boundaryIntersectionFraction > intersectionFractionTolerance & ...
            boundaryIntersectionFraction < 1 - intersectionFractionTolerance;
        pairCrossesInteriors = edgeBoxesOverlap & pairIsNonparallel & ...
            intersectionIsWithinSegment & intersectionIsWithinBoundary;
        isVisible(segmentIndices) = ~any(pairCrossesInteriors, 1).';

        % Keep every boundary contact, including endpoints of an overlap
        % along the same line. These contacts divide the segment into pieces.
        pairHasBoundaryContact = edgeBoxesOverlap & pairIsNonparallel & ...
            segmentIntersectionFraction >= 0 & segmentIntersectionFraction <= 1 & ...
            boundaryIntersectionFraction >= -intersectionFractionTolerance & ...
            boundaryIntersectionFraction <= 1 + intersectionFractionTolerance;
        pairSharesLine = edgeBoxesOverlap & ~pairIsNonparallel & ...
            abs(lineOffsetCrossProduct_units2) <= tolerance_units * segmentLengths_units;
        for batchSegmentIndex = find(isVisible(segmentIndices)).'
            boundaryStartFractions = (candidateStartOffsets_units(pairSharesLine(:, batchSegmentIndex), :) * ...
                segmentVector_units(batchSegmentIndex, :).') / ...
                sum(segmentVector_units(batchSegmentIndex, :).^2);
            boundaryEndFractions = (candidateEndOffsets_units(pairSharesLine(:, batchSegmentIndex), :) * ...
                segmentVector_units(batchSegmentIndex, :).') / ...
                sum(segmentVector_units(batchSegmentIndex, :).^2);
            contactFractions = unique([0; 1; ...
                segmentIntersectionFraction(pairHasBoundaryContact(:, batchSegmentIndex), batchSegmentIndex); ...
                min(1, max(0, boundaryStartFractions)); min(1, max(0, boundaryEndFractions))]);
            midpointFractions = (contactFractions(1:end - 1) + contactFractions(2:end)) / 2;
            midpointsBySegment{segmentIndices(batchSegmentIndex)} = ...
                segmentStart_units + midpointFractions .* segmentVector_units(batchSegmentIndex, :);
            midpointOwnersBySegment{segmentIndices(batchSegmentIndex)} = ...
                repmat(segmentIndices(batchSegmentIndex), numel(contactFractions) - 1, 1);
        end
    end
    intervalMidpoints_units = vertcat(midpointsBySegment{:});
    midpointSegmentIndices  = vertcat(midpointOwnersBySegment{:});
end
