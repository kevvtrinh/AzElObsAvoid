function [verified, alignedEndVertices_units, startRegions_units, endRegions_units, ...
    geometryModel, hasExactPartition, partitionReused, dependsOnPrevious] = alignVerifiedSingleRing( ...
    startSample, endSample, reusableStartRegions_units, proofRequest)
%% Section 0: Header & Readme
% SYNTAX
%   [verified, alignedEndVertices_units, startRegions_units, endRegions_units, ...
%       geometryModel, hasExactPartition, partitionReused, dependsOnPrevious] = ...
%       obstacleAvoidance.obstacles.alignVerifiedSingleRing( ...
%       startSample, endSample, reusableStartRegions_units, proofRequest)
%**************************************************************************
% PURPOSE
%   - Check the obstacle's motion between two recorded samples, including
%     every time between them. Match the protected boundary vertices, then
%     verify either one convex region or a set of convex pieces that covers
%     the complete polygon throughout the movement.
%   - Try translation first. For a changing shape, verify that the boundary
%     and its pieces do not cross, fold, or lose the required coverage.
%**************************************************************************
% INPUTS
%   - startSample, endSample (scalar structs)
%       Recorded obstacle positions at the interval start and end. X_units
%       and Y_units describe the protected boundary; NaN separates boundary
%       loops. Shape contains the same geometry as a polyshape.
%   - reusableStartRegions_units (cell column)
%       Verified regions at the end of the preceding interval, or empty.
%   - proofRequest (scalar struct)
%       Three logical controls. PreserveAlignment keeps the supplied vertex
%       order. TranslationOnly checks only translation with that order.
%       DeferTranslation postpones a detected translation so preparation can
%       reuse the preceding interval's regions once they are available.
%**************************************************************************
% OUTPUTS
%   - verified (logical scalar)
%       True when the interval's motion is verified. Use the returned geometry
%       only when this is true. A deferred check also returns false.
%   - alignedEndVertices_units (K-by-2 numeric)
%       End-sample vertices matched to the start sample's vertex order.
%   - startRegions_units, endRegions_units (cell columns)
%       Convex pieces of the verified partition at the interval start and
%       end; empty when one convex boundary represents the whole obstacle.
%   - geometryModel (string)
%       Name of the verified motion model.
%   - hasExactPartition, partitionReused, dependsOnPrevious (logical scalars)
%       Whether the partition is exact, was reused from the preceding
%       interval, or must wait for it.
%**************************************************************************
% UNITS
%   - Coordinate units; each vertex row is [x y].
%**************************************************************************

%% Section 1: Check The Samples And Verification Controls

startX_units = startSample.X_units;
startY_units = startSample.Y_units;
startShape   = startSample.Shape;
endX_units   = endSample.X_units;
endY_units   = endSample.Y_units;
endShape     = endSample.Shape;

preserveAlignment = proofRequest.PreserveAlignment;
translationOnly   = proofRequest.TranslationOnly;
deferTranslation  = proofRequest.DeferTranslation;

validateattributes(startX_units, {'numeric'}, {'real', 'vector'});
validateattributes(startY_units, {'numeric'}, {'real', 'numel', numel(startX_units)});
validateattributes(endX_units, {'numeric'}, {'real', 'vector'});
validateattributes(endY_units, {'numeric'}, {'real', 'numel', numel(endX_units)});
validateattributes(startShape, {'polyshape'}, {'scalar'});
validateattributes(endShape, {'polyshape'}, {'scalar'});
validateattributes(reusableStartRegions_units, {'cell'}, {});
validateattributes(preserveAlignment, {'logical'}, {'scalar'});
validateattributes(translationOnly, {'logical'}, {'scalar'});
validateattributes(deferTranslation, {'logical'}, {'scalar'});

%% Section 2: Check Translation With The Supplied Vertex Order

% Start with an unverified result. Translation means every matching vertex
% moves by the same [dx dy]; this can preserve several boundary loops at once.
dependsOnPrevious = false;

startVertices_units = [startX_units(:), startY_units(:)];
endVertices_units   = [endX_units(:), endY_units(:)];

verified = false;
alignedEndVertices_units = zeros(0, 2);
startRegions_units       = cell(0, 1);
endRegions_units         = cell(0, 1);

geometryModel       = "";
hasExactPartition   = false;
partitionReused     = false;

startVertexIsFinite = all(isfinite(startVertices_units), 2);
endVertexIsFinite   = all(isfinite(endVertices_units), 2);

% NaN separators must occur in the same rows before matching points by index.
if isequal(startVertexIsFinite, endVertexIsFinite) && nnz(startVertexIsFinite) >= 3
    finiteVertexDisplacements_units = endVertices_units(startVertexIsFinite, :) - ...
        startVertices_units(startVertexIsFinite, :);
    coordinateScale_units           = max([1; abs(startVertices_units(startVertexIsFinite, 1)); ...
        abs(startVertices_units(startVertexIsFinite, 2)); abs(endVertices_units(endVertexIsFinite, 1)); ...
        abs(endVertices_units(endVertexIsFinite, 2))]);
    translationTolerance_units = 512 * eps(coordinateScale_units);
    if max(abs(finiteVertexDisplacements_units - finiteVertexDisplacements_units(1, :)), [], 'all') <= ...
            translationTolerance_units
        if deferTranslation
            dependsOnPrevious = true;
            return;
        end
        alignedEndVertices_units = endVertices_units;
        startRegions_units       = reusableStartRegions_units;
        if isempty(startRegions_units)
            startRegions_units = obstacleAvoidance.geometry.convexRegions(startShape);
        else
            partitionReused = true;
        end
        endRegions_units = cellfun(@(region) region + finiteVertexDisplacements_units(1, :), ...
            startRegions_units, 'UniformOutput', false);
        verified          = true;
        geometryModel     = "linearCorrespondingConvexPartition";
        hasExactPartition = true;
        return;
    end
end

%% Section 3: Align One Boundary And Check Its Motion

% Other motion checks need one boundary loop with the same vertex count at
% both times. A translation-only request ends here if its first check failed.
samplesHaveOneBoundary = size(startVertices_units, 1) >= 3 && ...
    isequal(size(startVertices_units), size(endVertices_units)) && ...
    all(isfinite(startVertices_units), "all") && all(isfinite(endVertices_units), "all");
if ~samplesHaveOneBoundary || translationOnly
    return;
end

% Identical boundaries need no reordering.
if isequal(startVertices_units, endVertices_units)
    alignedEndVertices_units = endVertices_units;
    verified      = true;
    geometryModel = "linearCorrespondingVertices";
    return;
end
if preserveAlignment
    alignedEndVertices_units = endVertices_units;
else
    alignedEndVertices_units = obstacleAvoidance.obstacles.alignCorrespondingRing( ...
        startVertices_units, endVertices_units);
end
vertexDisplacement_units   = alignedEndVertices_units - startVertices_units;
coordinateScale_units      = max([1; abs(startVertices_units(:)); abs(alignedEndVertices_units(:))]);
translationTolerance_units = 512 * eps(coordinateScale_units);
vertexMotionIsTranslation  = max(abs(vertexDisplacement_units - vertexDisplacement_units(1, :)), [], "all") <= ...
    translationTolerance_units;
if vertexMotionIsTranslation
    if deferTranslation
        dependsOnPrevious = true;
        return;
    end
    % Translation preserves every piece of the starting partition. Shift
    % those pieces directly so their vertex order stays matched, even when
    % the supplied end boundary needed a different starting vertex or direction.
    startRegions_units = reusableStartRegions_units;
    if isempty(startRegions_units)
        startRegions_units = obstacleAvoidance.geometry.convexRegions(startShape);
    else
        partitionReused = true;
    end
    endRegions_units = cellfun(@(region) region + vertexDisplacement_units(1, :), ...
        startRegions_units, 'UniformOutput', false);
    verified = ~isempty(startRegions_units);
    if verified
        geometryModel     = "linearCorrespondingConvexPartition";
        hasExactPartition = true;
        return;
    end
end

% If every boundary turn keeps the same nonzero direction throughout the
% interval, the complete moving polygon can remain one convex region.
boundaryStaysConvex = remainsStrictlyConvex( ...
    startVertices_units, alignedEndVertices_units, coordinateScale_units);
verified = boundaryStaysConvex;
if verified
    geometryModel = "linearCorrespondingVertices";
    return;
end

%% Section 4: Verify Motion Using Convex Pieces

% A single affine map moves every vertex by the same translation, rotation,
% scale, and shear rules. An exact, non-collapsing map preserves the boundary;
% otherwise check moving edges directly for crossings.
[affineMotionIsVerified, exactAffineMotionIsVerified] = verifyAffineMotion( ...
    startVertices_units, alignedEndVertices_units, coordinateScale_units);
boundaryMotionIsVerified = exactAffineMotionIsVerified;
if ~boundaryMotionIsVerified
    boundaryMotionIsVerified = movingBoundaryRemainsSimple( ...
        startVertices_units, alignedEndVertices_units, coordinateScale_units);
end
[partitionIsVerified, startRegions_units, endRegions_units] = ...
    createVerifiedMovingPartition( ...
    startVertices_units, alignedEndVertices_units, startShape, endShape, ...
    coordinateScale_units, boundaryMotionIsVerified);
verified = partitionIsVerified && (affineMotionIsVerified || boundaryMotionIsVerified);
if verified
    geometryModel     = "linearCorrespondingConvexPartition";
    hasExactPartition = true;
    return;
end

% No model was verified. Clear the proposed geometry so it is not used.
alignedEndVertices_units = zeros(0, 2);
startRegions_units       = cell(0, 1);
endRegions_units         = cell(0, 1);
end

%% Section 5: Local Functions

function [verified, exactMotionIsVerified] = verifyAffineMotion( ...
        startVertices_units, endVertices_units, coordinateScale_units)
    % Fit one affine transform to all vertices. Check that its interpolation
    % from the start shape never collapses or flips the shape: the transform's
    % determinant must stay positive for the full interval.

    % The column of ones lets the fitted transform include translation.
    startPointMatrix       = [startVertices_units, ones(size(startVertices_units, 1), 1)];
    affineTransform        = startPointMatrix \ endVertices_units;
    mappingError_units     = startPointMatrix * affineTransform - endVertices_units;
    mappingTolerance_units = 4096 * eps(coordinateScale_units);
    if max(abs(mappingError_units), [], 'all') > mappingTolerance_units
        verified = false;
        exactMotionIsVerified = false;
        return;
    end
    linearTransform         = affineTransform(1:2, :);
    transformChange         = linearTransform - eye(2);
    determinantCoefficients = [det(transformChange), ...
        transformChange(1, 1) + transformChange(2, 2), 1];

    % A quadratic reaches its minimum or maximum at an endpoint or where its
    % slope is zero. Check those fractions, with 0 = start and 1 = end.
    checkFractions = [0; 1];
    if determinantCoefficients(1) ~= 0
        turningPointFraction = -determinantCoefficients(2) / (2 * determinantCoefficients(1));
        if turningPointFraction > 0 && turningPointFraction < 1
            checkFractions(end + 1, 1) = turningPointFraction;
        end
    end
    determinantValues    = polyval(determinantCoefficients, checkFractions);
    determinantTolerance = 4096 * eps(max(1, norm(linearTransform, 'fro') ^ 2));
    verified = all(determinantValues > determinantTolerance);

    % Only an exact fit lets the caller skip the moving-edge crossing check.
    exactMotionIsVerified = verified && all(mappingError_units == 0, 'all');
end

function [verified, startRegions_units, endRegions_units] = createVerifiedMovingPartition( ...
        startVertices_units, endVertices_units, startShape, endShape, coordinateScale_units, ...
        boundaryMotionIsVerified)
    % Divide the start shape into convex pieces and move their vertices to
    % the matching end positions. Each piece must stay convex the whole time.
    verified           = false;
    startRegions_units = obstacleAvoidance.geometry.convexRegions(startShape);
    endRegions_units   = cell(size(startRegions_units));
    sourceVertexIndicesByRegion = cell(size(startRegions_units));
    for regionIndex = 1:numel(startRegions_units)
        [vertexIsFromStartBoundary, sourceVertexIndices] = ismember( ...
            startRegions_units{regionIndex}, startVertices_units, 'rows');
        % Each piece needs distinct vertices from the supplied boundary.
        % Their indices determine the end positions; a new vertex would have
        % no supplied path to follow.
        if ~all(vertexIsFromStartBoundary) || numel(unique(sourceVertexIndices)) ~= numel(sourceVertexIndices)
            startRegions_units = cell(0, 1);
            endRegions_units   = cell(0, 1);
            return;
        end
        sourceVertexIndicesByRegion{regionIndex} = sourceVertexIndices;
        endRegions_units{regionIndex} = endVertices_units(sourceVertexIndices, :);
        % A larger piece may bend inward during motion even though its
        % triangles stay valid. Try the finer triangle partition in that case.
        if ~remainsStrictlyConvex(startRegions_units{regionIndex}, ...
                endRegions_units{regionIndex}, coordinateScale_units)
            [verified, startRegions_units, endRegions_units] = createVerifiedMovingTriangles( ...
                startVertices_units, endVertices_units, startShape, endShape, ...
                coordinateScale_units, boundaryMotionIsVerified);
            return;
        end
    end
    verified = partitionMatchesEndpointShapes( ...
        startRegions_units, endRegions_units, sourceVertexIndicesByRegion, ...
        startVertices_units, endVertices_units, startShape, endShape, boundaryMotionIsVerified);
end

function [verified, startRegions_units, endRegions_units] = createVerifiedMovingTriangles( ...
        startVertices_units, endVertices_units, startShape, endShape, coordinateScale_units, ...
        boundaryMotionIsVerified)
    % Check a finer partition containing one region per triangle. Every
    % triangle vertex must come from the supplied boundary so its path is known.
    verified           = false;
    startRegions_units = cell(0, 1);
    endRegions_units   = cell(0, 1);
    triangleMesh       = triangulation(startShape);
    [vertexIsFromStartBoundary, sourceVertexIndices] = ismember(triangleMesh.Points, startVertices_units, 'rows');
    if ~all(vertexIsFromStartBoundary) || numel(unique(sourceVertexIndices)) ~= numel(sourceVertexIndices)
        return;
    end
    triangleCount      = size(triangleMesh.ConnectivityList, 1);
    startRegions_units = cell(triangleCount, 1);
    endRegions_units   = cell(triangleCount, 1);
    sourceVertexIndicesByRegion = cell(triangleCount, 1);
    for triangleIndex = 1:triangleCount
        triangleVertexIndices = triangleMesh.ConnectivityList(triangleIndex, :);
        sourceVertexIndicesByRegion{triangleIndex} = sourceVertexIndices(triangleVertexIndices);
        startRegions_units{triangleIndex} = triangleMesh.Points(triangleVertexIndices, :);
        endRegions_units{triangleIndex}   = endVertices_units(sourceVertexIndicesByRegion{triangleIndex}, :);
        if ~remainsStrictlyConvex(startRegions_units{triangleIndex}, ...
                endRegions_units{triangleIndex}, coordinateScale_units)
            startRegions_units = cell(0, 1);
            endRegions_units   = cell(0, 1);
            return;
        end
    end
    verified = partitionMatchesEndpointShapes( ...
        startRegions_units, endRegions_units, sourceVertexIndicesByRegion, ...
        startVertices_units, endVertices_units, startShape, endShape, boundaryMotionIsVerified);
end

function verified = partitionMatchesEndpointShapes( ...
        startRegions_units, endRegions_units, sourceVertexIndicesByRegion, ...
        startVertices_units, endVertices_units, startShape, endShape, boundaryMotionIsVerified)
    % Verify that the pieces cover the supplied start and end shapes. First
    % check their shared-edge structure when boundary motion is already proven;
    % otherwise compare the area belonging to only one of the two shapes.
    if boundaryMotionIsVerified && partitionHasMatchingEdges( ...
            sourceVertexIndicesByRegion, startVertices_units, endVertices_units, startShape, endShape)
        verified = true;
        return;
    end
    combinedStartShape   = combineRegions(startRegions_units);
    combinedEndShape     = combineRegions(endRegions_units);
    areaScale_units2     = max([1, area(startShape), area(endShape)]);
    areaTolerance_units2 = 4096 * eps(areaScale_units2);
    verified = area(xor(combinedStartShape, startShape)) <= areaTolerance_units2 && ...
        area(xor(combinedEndShape, endShape)) <= areaTolerance_units2;
end

function verified = partitionHasMatchingEdges( ...
        sourceVertexIndicesByRegion, startVertices_units, endVertices_units, startShape, endShape)
    % Outer edges must reproduce the supplied boundary. Every interior edge
    % must appear twice, once in each direction, so adjacent pieces meet along
    % the same edge. The caller also checks each piece and the moving boundary.
    vertexCount = size(startVertices_units, 1);
    verified    = isequal(size(startVertices_units), size(endVertices_units)) && ...
        size(unique(startVertices_units, 'rows'), 1) == vertexCount && ...
        size(unique(endVertices_units, 'rows'), 1) == vertexCount && ...
        shapeHasSameBoundary(startShape, startVertices_units) && ...
        shapeHasSameBoundary(endShape, endVertices_units) && ...
        all(cellfun(@numel, sourceVertexIndicesByRegion) >= 3);
    if ~verified || isempty(sourceVertexIndicesByRegion)
        return;
    end

    edgeCount = sum(cellfun(@numel, sourceVertexIndicesByRegion));
    edgeStartVertexIndices = zeros(edgeCount, 1);
    edgeEndVertexIndices   = zeros(edgeCount, 1);
    firstEdgeIndex         = 1;
    for regionIndex = 1:numel(sourceVertexIndicesByRegion)
        sourceVertexIndices = sourceVertexIndicesByRegion{regionIndex}(:);
        lastEdgeIndex       = firstEdgeIndex + numel(sourceVertexIndices) - 1;
        edgeStartVertexIndices(firstEdgeIndex:lastEdgeIndex) = sourceVertexIndices;
        edgeEndVertexIndices(firstEdgeIndex:lastEdgeIndex)   = circshift(sourceVertexIndices, -1);
        firstEdgeIndex = lastEdgeIndex + 1;
    end
    % [2 5] and [5 2] describe the same edge. Count them together, then check
    % below that a shared interior edge appears once in each direction.
    sortedEdgeVertexPairs = [min(edgeStartVertexIndices, edgeEndVertexIndices), ...
        max(edgeStartVertexIndices, edgeEndVertexIndices)];
    [uniqueEdgeVertexPairs, ~, edgeGroupIndex] = unique(sortedEdgeVertexPairs, 'rows');
    edgeUseCount        = accumarray(edgeGroupIndex, 1);
    forwardEdgeUseCount = accumarray(edgeGroupIndex, edgeStartVertexIndices < edgeEndVertexIndices);

    % An interior edge belongs to two pieces, traversed in opposite directions.
    % A connected partition without holes also satisfies V - E + F = 1, where
    % V = vertices, E = unique edges, and F = pieces (excluding the outside).
    if any(edgeUseCount > 2) || ...
            any(forwardEdgeUseCount(edgeUseCount == 2) ~= 1) || ...
            vertexCount - size(uniqueEdgeVertexPairs, 1) + numel(sourceVertexIndicesByRegion) ~= 1
        verified = false;
        return;
    end

    nextSourceVertexIndices = [2:vertexCount, 1].';
    expectedBoundaryEdges   = sortrows([ ...
        min((1:vertexCount).', nextSourceVertexIndices), ...
        max((1:vertexCount).', nextSourceVertexIndices)]);
    actualBoundaryEdges   = sortrows(uniqueEdgeVertexPairs(edgeUseCount == 1, :));
    edgeIsOnBoundary      = edgeUseCount(edgeGroupIndex) == 1;
    directedBoundaryEdges = sortrows([ ...
        edgeStartVertexIndices(edgeIsOnBoundary), edgeEndVertexIndices(edgeIsOnBoundary)]);
    forwardBoundaryEdges = sortrows([(1:vertexCount).', nextSourceVertexIndices]);
    reverseBoundaryEdges = sortrows([nextSourceVertexIndices, (1:vertexCount).']);
    verified = isequal(actualBoundaryEdges, expectedBoundaryEdges) && ...
        (isequal(directedBoundaryEdges, forwardBoundaryEdges) || ...
        isequal(directedBoundaryEdges, reverseBoundaryEdges));
end

function verified = shapeHasSameBoundary(shape, vertices_units)
    % Check that polyshape kept one hole-free boundary with the supplied
    % vertices and edges, allowing a different starting vertex or direction.
    shapeParts = regions(shape);
    if numel(shapeParts) ~= 1 || shapeParts.NumHoles ~= 0
        verified = false;
        return;
    end
    shapeVertices_units = shapeParts.Vertices;
    [shapeVertexIsFromInput, shapeSourceVertexIndices] = ismember( ...
        shapeVertices_units, vertices_units, 'rows');
    if ~isequal(size(shapeVertices_units), size(vertices_units)) || ...
            ~all(shapeVertexIsFromInput) || ...
            numel(unique(shapeSourceVertexIndices)) ~= numel(shapeSourceVertexIndices)
        verified = false;
        return;
    end
    nextShapeVertexIndices = circshift(shapeSourceVertexIndices, -1);
    shapeEdgeVertexPairs   = sortrows([ ...
        min(shapeSourceVertexIndices, nextShapeVertexIndices), ...
        max(shapeSourceVertexIndices, nextShapeVertexIndices)]);
    vertexCount = size(vertices_units, 1);
    nextSourceVertexIndices = [2:vertexCount, 1].';
    sourceEdgeVertexPairs   = sortrows([ ...
        min((1:vertexCount).', nextSourceVertexIndices), max((1:vertexCount).', nextSourceVertexIndices)]);
    verified = isequal(shapeEdgeVertexPairs, sourceEdgeVertexPairs);
end

function shape = combineRegions(regions_units)
    % Combine the pieces while retaining extra points along straight edges.
    shape = polyshape();
    for regionIndex = 1:numel(regions_units)
        regionVertices_units = regions_units{regionIndex};
        shape = union(shape, polyshape( ...
            regionVertices_units, 'Simplify', false, 'KeepCollinearPoints', true));
    end
end

function verified = movingBoundaryRemainsSimple(startVertices_units, endVertices_units, coordinateScale_units)
    % Edge crossing can change only when an endpoint lines up with the other
    % edge. Find those event times from quadratic equations, then check
    % each event and one time between consecutive events.
    verified    = true;
    vertexCount = size(startVertices_units, 1);
    vertexDisplacement_units    = endVertices_units - startVertices_units;
    orientationTolerance_units2 = 4096 * eps(coordinateScale_units ^ 2);
    positionTolerance_units     = 4096 * eps(coordinateScale_units);

    % Vertices move along straight lines between samples. These coordinate
    % bounds therefore contain the whole moving edge. Nonoverlapping bounds
    % prove that a pair of edges cannot cross during the interval.
    nextVertexIndices = [2:vertexCount, 1];
    edgeMinimum_units = min(min(startVertices_units, startVertices_units(nextVertexIndices, :)), ...
        min(endVertices_units, endVertices_units(nextVertexIndices, :)));
    edgeMaximum_units = max(max(startVertices_units, startVertices_units(nextVertexIndices, :)), ...
        max(endVertices_units, endVertices_units(nextVertexIndices, :)));
    for firstEdgeIndex = 1:vertexCount
        firstEdgeEndIndex = mod(firstEdgeIndex, vertexCount) + 1;
        laterEdgeIndices  = (firstEdgeIndex + 1:vertexCount).';
        edgeBoundsOverlap = all(edgeMaximum_units(firstEdgeIndex, :) >= ...
            edgeMinimum_units(laterEdgeIndices, :) - positionTolerance_units & ...
            edgeMaximum_units(laterEdgeIndices, :) >= ...
            edgeMinimum_units(firstEdgeIndex, :) - positionTolerance_units, 2);
        for secondEdgeIndex = reshape(laterEdgeIndices(edgeBoundsOverlap), 1, [])
            secondEdgeEndIndex   = mod(secondEdgeIndex, vertexCount) + 1;
            edgesShareAnEndpoint = secondEdgeIndex == firstEdgeEndIndex || secondEdgeEndIndex == firstEdgeIndex;
            if edgesShareAnEndpoint
                continue;
            end
            edgeAndPointIndices = [firstEdgeIndex, firstEdgeEndIndex, secondEdgeIndex; ...
                firstEdgeIndex, firstEdgeEndIndex, secondEdgeEndIndex; ...
                secondEdgeIndex, secondEdgeEndIndex, firstEdgeIndex; ...
                secondEdgeIndex, secondEdgeEndIndex, firstEdgeEndIndex];
            eventFractions = [0; 1];
            for edgeAndPointRow = 1:4
                vertexIndices = edgeAndPointIndices(edgeAndPointRow, :);
                orientationCoefficients_units2 = createOrientationPolynomial( ...
                    startVertices_units(vertexIndices, :), vertexDisplacement_units(vertexIndices, :));
                orientationIsIndeterminate = all(abs(orientationCoefficients_units2) <= orientationTolerance_units2);
                % Nearly zero coefficients cannot establish which side of
                % the edge this point stays on. Leave this motion unverified.
                if orientationIsIndeterminate
                    verified = false;
                    return;
                end
                rootFractions  = realPolynomialRoots(orientationCoefficients_units2, orientationTolerance_units2);
                eventFractions = [eventFractions; rootFractions]; %#ok<AGROW>
            end
            eventFractions = unique(min(1, max(0, eventFractions)));
            checkFractions = [eventFractions; (eventFractions(1:end - 1) + eventFractions(2:end)) / 2];
            for intervalFraction = reshape(checkFractions, 1, [])
                vertices_units = startVertices_units + intervalFraction * vertexDisplacement_units;
                if segmentsIntersect(vertices_units(firstEdgeIndex, :), vertices_units(firstEdgeEndIndex, :), ...
                        vertices_units(secondEdgeIndex, :), vertices_units(secondEdgeEndIndex, :), ...
                        orientationTolerance_units2, positionTolerance_units)
                    verified = false;
                    return;
                end
            end
        end
    end
end

function orientationCoefficients_units2 = createOrientationPolynomial(vertices_units, vertexDisplacement_units)
    % For one moving edge and another moving point, calculate which side of
    % the edge the point lies on. Its signed cross product is a quadratic in
    % interval fraction u. The coefficients multiply [u^2, u, 1], in that order.
    firstEdgeVector_units   = vertices_units(2, :) - vertices_units(1, :);
    pointOffset_units       = vertices_units(3, :) - vertices_units(1, :);
    edgeVectorChange_units  = vertexDisplacement_units(2, :) - vertexDisplacement_units(1, :);
    pointOffsetChange_units = vertexDisplacement_units(3, :) - vertexDisplacement_units(1, :);
    orientationCoefficients_units2 = [ ...
        obstacleAvoidance.geometry.cross2d(edgeVectorChange_units, pointOffsetChange_units), ...
        obstacleAvoidance.geometry.cross2d(edgeVectorChange_units, pointOffset_units) + ...
        obstacleAvoidance.geometry.cross2d(firstEdgeVector_units, pointOffsetChange_units), ...
        obstacleAvoidance.geometry.cross2d(firstEdgeVector_units, pointOffset_units)];
end

function rootFractions = realPolynomialRoots(orientationCoefficients_units2, coefficientTolerance_units2)
    % Find the interval fractions where the orientation polynomial is zero.
    % A negligible quadratic term leaves a linear equation; negligible linear
    % and quadratic terms leave no isolated zero to check.
    quadraticTermIsNegligible = abs(orientationCoefficients_units2(1)) <= coefficientTolerance_units2;
    linearTermIsNegligible    = abs(orientationCoefficients_units2(2)) <= coefficientTolerance_units2;
    if quadraticTermIsNegligible && linearTermIsNegligible
        rootFractions = zeros(0, 1);
    elseif quadraticTermIsNegligible
        rootFractions = -orientationCoefficients_units2(3) / orientationCoefficients_units2(2);
    else
        candidateRoots     = roots(orientationCoefficients_units2);
        imaginaryTolerance = 1024 * eps(max(1, max(abs(candidateRoots))));
        rootFractions      = real(candidateRoots(abs(imag(candidateRoots)) <= imaginaryTolerance));
    end
    fractionTolerance = 1024 * eps;
    rootFractions     = rootFractions(rootFractions >= -fractionTolerance & rootFractions <= 1 + fractionTolerance);
end

function intersects = segmentsIntersect(firstStart_units, firstEnd_units, ...
        secondStart_units, secondEnd_units, orientationTolerance_units2, positionTolerance_units)
    % Two segments meet when their coordinate bounds overlap and each crosses
    % or touches the other's line. Values within the orientation tolerance
    % count as zero, so touching endpoints and edges are included.
    firstDirection_units     = firstEnd_units - firstStart_units;
    secondDirection_units    = secondEnd_units - secondStart_units;
    orientationValues_units2 = [obstacleAvoidance.geometry.cross2d( ...
        firstDirection_units, secondStart_units - firstStart_units), ...
        obstacleAvoidance.geometry.cross2d( ...
        firstDirection_units, secondEnd_units - firstStart_units), ...
        obstacleAvoidance.geometry.cross2d( ...
        secondDirection_units, firstStart_units - secondStart_units), ...
        obstacleAvoidance.geometry.cross2d( ...
        secondDirection_units, firstEnd_units - secondStart_units)];
    orientationSigns = sign(orientationValues_units2);
    orientationSigns(abs(orientationValues_units2) <= orientationTolerance_units2) = 0;
    overlapMinimum_units = max(min([firstStart_units; firstEnd_units]), ...
        min([secondStart_units; secondEnd_units]));
    overlapMaximum_units = min(max([firstStart_units; firstEnd_units]), ...
        max([secondStart_units; secondEnd_units]));
    boundingBoxesOverlap = all(overlapMinimum_units <= overlapMaximum_units + positionTolerance_units);
    intersects           = boundingBoxesOverlap && ...
        orientationSigns(1) * orientationSigns(2) <= 0 && orientationSigns(3) * orientationSigns(4) <= 0;
end

function verified = remainsStrictlyConvex(startVertices_units, endVertices_units, coordinateScale_units)
    % Every boundary turn must keep the same direction for the full motion.
    % Reject straight or reversed turns, including changes between samples.
    startEdgeVectors_units        = circshift(startVertices_units, -1, 1) - startVertices_units;
    endEdgeVectors_units          = circshift(endVertices_units, -1, 1) - endVertices_units;
    startTurnCrossProducts_units2 = obstacleAvoidance.geometry.cross2d( ...
        startEdgeVectors_units, circshift(startEdgeVectors_units, -1, 1));
    boundaryDirection    = sign(sum(startTurnCrossProducts_units2));
    turnTolerance_units2 = 4096 * eps(coordinateScale_units ^ 2);
    if boundaryDirection == 0 || any(boundaryDirection * startTurnCrossProducts_units2 <= turnTolerance_units2)
        verified = false;
        return;
    end
    edgeVectorChanges_units    = endEdgeVectors_units - startEdgeVectors_units;
    nextStartEdgeVectors_units  = circshift(startEdgeVectors_units, -1, 1);
    nextEdgeVectorChanges_units = circshift(edgeVectorChanges_units, -1, 1);
    constantCoefficients_units2 = obstacleAvoidance.geometry.cross2d( ...
        startEdgeVectors_units, nextStartEdgeVectors_units);
    linearCoefficients_units2 = obstacleAvoidance.geometry.cross2d( ...
        edgeVectorChanges_units, nextStartEdgeVectors_units) + ...
        obstacleAvoidance.geometry.cross2d(startEdgeVectors_units, nextEdgeVectorChanges_units);
    quadraticCoefficients_units2 = obstacleAvoidance.geometry.cross2d( ...
        edgeVectorChanges_units, nextEdgeVectorChanges_units);
    verified = true;
    for vertexIndex = 1:size(startVertices_units, 1)
        % Check the endpoints and the fraction where the quadratic's slope
        % is zero. Together these include its minimum and maximum turns.
        checkFractions = [0; 1];
        if quadraticCoefficients_units2(vertexIndex) ~= 0
            turningPointFraction = -linearCoefficients_units2(vertexIndex) / ...
                (2 * quadraticCoefficients_units2(vertexIndex));
            if turningPointFraction > 0 && turningPointFraction < 1
                checkFractions(end + 1, 1) = turningPointFraction; %#ok<AGROW>
            end
        end
        turnCrossProducts_units2 = constantCoefficients_units2(vertexIndex) + ...
            linearCoefficients_units2(vertexIndex) * checkFractions + ...
            quadraticCoefficients_units2(vertexIndex) * checkFractions .^ 2;
        if any(boundaryDirection * turnCrossProducts_units2 <= turnTolerance_units2)
            verified = false;
            return;
        end
    end
end
