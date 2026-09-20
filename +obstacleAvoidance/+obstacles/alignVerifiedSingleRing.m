function [verified, alignedUpper_units, startRegions_units, endRegions_units, ...
    geometryModel, hasExactPartition, partitionReused, dependsOnPrevious] = alignVerifiedSingleRing( ...
    lowerX_units, lowerY_units, upperX_units, upperY_units, lowerShape, ...
    upperShape, reusableStartRegions_units, preserveAlignment, translationOnly, deferTranslation)
%% Section 0: Header & Readme
% SYNTAX
%   [verified, alignedUpper_units, startRegions_units, endRegions_units, ...
%       geometryModel, hasExactPartition, partitionReused, dependsOnPrevious] = ...
%       obstacleAvoidance.obstacles.alignVerifiedSingleRing( ...
%       lowerX_units, lowerY_units, upperX_units, upperY_units, lowerShape, ...
%       upperShape, reusableStartRegions_units, preserveAlignment, ...
%       translationOnly, deferTranslation)
%**************************************************************************
% PURPOSE
%   - Certify one source interval of a moving obstacle: align the two
%     protected sample rings, then prove either one moving convex region or
%     an exact moving convex partition of the complete interpolated polygon.
%     A partition is accepted when every cell stays strictly convex over the
%     interval, the motion is a positive-determinant global affine map or
%     keeps the moving boundary simple, and the partition keeps exact ring
%     topology or the endpoint areas.
%**************************************************************************
% INPUTS
%   - lowerX_units, lowerY_units, upperX_units, upperY_units (numeric vectors)
%       Protected sample rings at the interval start and end, NaN-separated
%       when a sample has several rings.
%   - lowerShape, upperShape (polyshape)
%       The same two samples as shapes.
%   - reusableStartRegions_units (cell column)
%       End regions of a certified preceding interval, or empty.
%   - preserveAlignment (logical scalar)
%       Keep the given vertex order instead of realigning.
%   - translationOnly (logical scalar)
%       Stop after the index-preserving translation check.
%   - deferTranslation (logical scalar)
%       Report a translation candidate as depending on the previous
%       interval instead of resolving it here (batch preparation).
%**************************************************************************
% OUTPUTS
%   - verified (logical scalar)
%       True when a certificate was found. When false, including the
%       deferred case, every other output keeps its empty default.
%   - alignedUpper_units (K-by-2 numeric)
%       The end ring aligned to the start ring's vertex order.
%   - startRegions_units, endRegions_units (cell columns)
%       Convex cells of the certified partition at the interval start and
%       end; empty when the whole ring is the one convex region.
%   - geometryModel (string)
%       Name of the certificate that succeeded.
%   - hasExactPartition, partitionReused, dependsOnPrevious (logical scalars)
%       Whether the partition is exact, was reused from the preceding
%       interval, or must wait for it.
%**************************************************************************
% UNITS
%   - Coordinate units, [x y] columns.
%**************************************************************************

%% Section 1: Validate The Rings And The Certificate Requests

validateattributes(lowerX_units, {'numeric'}, {'real', 'vector'});
validateattributes(lowerY_units, {'numeric'}, {'real', 'numel', numel(lowerX_units)});
validateattributes(upperX_units, {'numeric'}, {'real', 'vector'});
validateattributes(upperY_units, {'numeric'}, {'real', 'numel', numel(upperX_units)});
validateattributes(lowerShape, {'polyshape'}, {'scalar'});
validateattributes(upperShape, {'polyshape'}, {'scalar'});
validateattributes(reusableStartRegions_units, {'cell'}, {});
validateattributes(preserveAlignment, {'logical'}, {'scalar'});
validateattributes(translationOnly, {'logical'}, {'scalar'});
validateattributes(deferTranslation, {'logical'}, {'scalar'});

%% Section 2: Align The Rings And Certify The Motion

% Align rings, then certify either one moving convex region or an exact
% moving convex partition of the complete interpolated polygon.
% translationOnly stops after the index-preserving translation check.
dependsOnPrevious = false;
lower_units        = [lowerX_units(:), lowerY_units(:)];
upper_units        = [upperX_units(:), upperY_units(:)];
verified           = false;
alignedUpper_units = zeros(0, 2);
startRegions_units = cell(0, 1);
endRegions_units   = cell(0, 1);
geometryModel      = "";
hasExactPartition  = false;
partitionReused    = false;
lowerFinite = all(isfinite(lower_units), 2);
upperFinite = all(isfinite(upper_units), 2);
if isequal(lowerFinite, upperFinite) && nnz(lowerFinite) >= 3
    finiteDelta_units = upper_units(lowerFinite, :) - lower_units(lowerFinite, :);
    coordinateScale_units = max([1; abs(lower_units(lowerFinite, 1)); ...
        abs(lower_units(lowerFinite, 2)); abs(upper_units(upperFinite, 1)); ...
        abs(upper_units(upperFinite, 2))]);
    translationTolerance_units = 512 * eps(coordinateScale_units);
    if max(abs(finiteDelta_units - finiteDelta_units(1, :)), [], 'all') <= translationTolerance_units
        if deferTranslation
            dependsOnPrevious = true;
            return;
        end
        alignedUpper_units = upper_units;
        startRegions_units = reusableStartRegions_units;
        if isempty(startRegions_units)
            startRegions_units = obstacleAvoidance.geometry.convexRegions(lowerShape);
        else
            partitionReused = true;
        end
        endRegions_units = cellfun(@(region) region + finiteDelta_units(1, :), ...
            startRegions_units, 'UniformOutput', false);
        verified         = true;
        geometryModel    = "linearCorrespondingConvexPartition";
        hasExactPartition = true;
        return;
    end
end
isSingleRing = size(lower_units, 1) >= 3 && ...
    isequal(size(lower_units), size(upper_units)) && ...
    all(isfinite(lower_units), "all") && all(isfinite(upper_units), "all");
if ~isSingleRing || translationOnly
    return;
end

% Identical rings already attain the first possible zero-distance match.
if isequal(lower_units, upper_units)
    alignedUpper_units = upper_units;
    verified           = true;
    geometryModel      = "linearCorrespondingVertices";
    return;
end
if preserveAlignment
    alignedUpper_units = upper_units;
else
    alignedUpper_units = obstacleAvoidance.obstacles.alignCorrespondingRing(lower_units, upper_units);
end
delta_units                = alignedUpper_units - lower_units;
coordinateScale_units      = max([1; abs(lower_units(:)); abs(alignedUpper_units(:))]);
translationTolerance_units = 512 * eps(coordinateScale_units);
isTranslation              = max(abs(delta_units - delta_units(1, :)), [], "all") <= ...
    translationTolerance_units;
if isTranslation
    if deferTranslation
        dependsOnPrevious = true;
        return;
    end
    % A translation preserves every face of one exact partition. Build
    % the terminal faces by translating those same faces; this avoids
    % relying on polyshape's vertex ordering after cyclic/reversed input.
    startRegions_units = reusableStartRegions_units;
    if isempty(startRegions_units)
        startRegions_units = obstacleAvoidance.geometry.convexRegions(lowerShape);
    else
        partitionReused = true;
    end
    endRegions_units = cellfun(@(region) region + delta_units(1, :), ...
        startRegions_units, 'UniformOutput', false);
    verified = ~isempty(startRegions_units);
    if verified
        geometryModel = "linearCorrespondingConvexPartition";
        hasExactPartition = true;
        return;
    end
end
isConvexMotion = remainsStrictlyConvex( ...
    lower_units, alignedUpper_units, coordinateScale_units);
verified = isConvexMotion;
if verified
    geometryModel = "linearCorrespondingVertices";
    return;
end
[globalAffineVerified, exactGlobalAffineVerified] = verifiedGlobalAffineMap( ...
    lower_units, alignedUpper_units, coordinateScale_units);
topologyMotionVerified = exactGlobalAffineVerified;
if ~topologyMotionVerified
    topologyMotionVerified = movingBoundaryRemainsSimple( ...
        lower_units, alignedUpper_units, coordinateScale_units);
end
[partitionVerified, startRegions_units, endRegions_units] = ...
    createVerifiedMovingPartition( ...
    lower_units, alignedUpper_units, lowerShape, upperShape, ...
    coordinateScale_units, topologyMotionVerified);
verified = partitionVerified && (globalAffineVerified || topologyMotionVerified);
if verified
    geometryModel = "linearCorrespondingConvexPartition";
    hasExactPartition = true;
    return;
end
% Every verified branch above returned, so this cleanup is unconditional.
alignedUpper_units = zeros(0, 2);
startRegions_units = cell(0, 1);
endRegions_units   = cell(0, 1);
end

%% Section 3: Local Functions

function [verified, exactVerified] = verifiedGlobalAffineMap( ...
        lower_units, upper_units, coordinateScale_units)
    % A single affine map preserves every edge and face. Its linear blend
    % with identity is valid when the determinant stays strictly positive.
    source = [lower_units, ones(size(lower_units, 1), 1)];
    transformation = source \ upper_units;
    residual_units = source * transformation - upper_units;
    residualTolerance_units = 4096 * eps(coordinateScale_units);
    if max(abs(residual_units), [], 'all') > residualTolerance_units
        verified = false;
        exactVerified = false;
        return;
    end
    linearMap = transformation(1:2, :);
    deltaMap = linearMap - eye(2);
    determinantCoefficients = [det(deltaMap), ...
        deltaMap(1, 1) + deltaMap(2, 2), 1];
    candidateTau = [0; 1];
    if determinantCoefficients(1) ~= 0
        stationaryTau = -determinantCoefficients(2) / (2 * determinantCoefficients(1));
        if stationaryTau > 0 && stationaryTau < 1
            candidateTau(end + 1, 1) = stationaryTau;
        end
    end
    determinants = polyval(determinantCoefficients, candidateTau);
    determinantTolerance = 4096 * eps(max(1, norm(linearMap, 'fro') ^ 2));
    verified = all(determinants > determinantTolerance);
    exactVerified = verified && all(residual_units == 0, 'all');
end

function [verified, startRegions_units, endRegions_units] = createVerifiedMovingPartition( ...
        lower_units, upper_units, lowerShape, upperShape, coordinateScale_units, ...
        topologyMotionVerified)
    % Carry one exact lower-sample partition through the supplied vertex
    % correspondence. Every face must remain convex for the whole interval.
    verified = false;
    startRegions_units = obstacleAvoidance.geometry.convexRegions(lowerShape);
    endRegions_units = cell(size(startRegions_units));
    sourceIndexRegions = cell(size(startRegions_units));
    for regionIndex = 1:numel(startRegions_units)
        [isSourceVertex, sourceIndex] = ismember( ...
            startRegions_units{regionIndex}, lower_units, 'rows');
        if ~all(isSourceVertex) || numel(unique(sourceIndex)) ~= numel(sourceIndex)
            startRegions_units = cell(0, 1);
            endRegions_units = cell(0, 1);
            return;
        end
        sourceIndexRegions{regionIndex} = sourceIndex;
        endRegions_units{regionIndex} = upper_units(sourceIndex, :);
        if ~remainsStrictlyConvex(startRegions_units{regionIndex}, ...
                endRegions_units{regionIndex}, coordinateScale_units)
            [verified, startRegions_units, endRegions_units] = createVerifiedMovingTriangles( ...
                lower_units, upper_units, lowerShape, upperShape, ...
                coordinateScale_units, topologyMotionVerified);
            return;
        end
    end
    verified = partitionMatchesEndpointShapes( ...
        startRegions_units, endRegions_units, sourceIndexRegions, ...
        lower_units, upper_units, lowerShape, upperShape, topologyMotionVerified);
end

function [verified, startRegions_units, endRegions_units] = createVerifiedMovingTriangles( ...
        lower_units, upper_units, lowerShape, upperShape, coordinateScale_units, ...
        topologyMotionVerified)
    % A triangle mesh is the non-heuristic fallback partition. It is
    % accepted only when every mesh point is an original corresponding vertex.
    verified = false;
    startRegions_units = cell(0, 1);
    endRegions_units   = cell(0, 1);
    mesh = triangulation(lowerShape);
    [isSourceVertex, sourceIndex] = ismember(mesh.Points, lower_units, 'rows');
    if ~all(isSourceVertex) || numel(unique(sourceIndex)) ~= numel(sourceIndex)
        return;
    end
    faceCount = size(mesh.ConnectivityList, 1);
    startRegions_units = cell(faceCount, 1);
    endRegions_units   = cell(faceCount, 1);
    sourceIndexRegions = cell(faceCount, 1);
    for faceIndex = 1:faceCount
        pointIndex = mesh.ConnectivityList(faceIndex, :);
        sourceIndexRegions{faceIndex} = sourceIndex(pointIndex);
        startRegions_units{faceIndex} = mesh.Points(pointIndex, :);
        endRegions_units{faceIndex} = upper_units(sourceIndexRegions{faceIndex}, :);
        if ~remainsStrictlyConvex(startRegions_units{faceIndex}, ...
                endRegions_units{faceIndex}, coordinateScale_units)
            startRegions_units = cell(0, 1);
            endRegions_units = cell(0, 1);
            return;
        end
    end
    verified = partitionMatchesEndpointShapes( ...
        startRegions_units, endRegions_units, sourceIndexRegions, ...
        lower_units, upper_units, lowerShape, upperShape, topologyMotionVerified);
end

function verified = partitionMatchesEndpointShapes( ...
        startRegions_units, endRegions_units, sourceIndexRegions, ...
        lower_units, upper_units, lowerShape, upperShape, topologyMotionVerified)
    % Endpoint Boolean equality catches mapping or triangulation changes
    % before the continuous face certificates are trusted.
    if topologyMotionVerified && partitionHasExactRingTopology( ...
            sourceIndexRegions, lower_units, upper_units, lowerShape, upperShape)
        verified = true;
        return;
    end
    startUnion = unionRegions(startRegions_units);
    endUnion = unionRegions(endRegions_units);
    areaScale_units2 = max([1, area(lowerShape), area(upperShape)]);
    areaTolerance_units2 = 4096 * eps(areaScale_units2);
    verified = area(xor(startUnion, lowerShape)) <= areaTolerance_units2 && ...
        area(xor(endUnion, upperShape)) <= areaTolerance_units2;
end

function verified = partitionHasExactRingTopology( ...
        sourceIndexRegions, lower_units, upper_units, lowerShape, upperShape)
    % For an unsimplified simple ring, a partition is carried exactly when
    % every outer edge is one source edge and every interior edge is shared
    % once in each direction. Strict face-motion and boundary certificates
    % are applied by the caller after this endpoint topology check.
    vertexCount = size(lower_units, 1);
    verified = isequal(size(lower_units), size(upper_units)) && ...
        size(unique(lower_units, 'rows'), 1) == vertexCount && ...
        size(unique(upper_units, 'rows'), 1) == vertexCount && ...
        shapeHasExactRingBoundary(lowerShape, lower_units) && ...
        shapeHasExactRingBoundary(upperShape, upper_units) && ...
        all(cellfun(@numel, sourceIndexRegions) >= 3);
    if ~verified || isempty(sourceIndexRegions)
        return;
    end

    edgeCount = sum(cellfun(@numel, sourceIndexRegions));
    edgeStart = zeros(edgeCount, 1);
    edgeEnd   = zeros(edgeCount, 1);
    firstEdge = 1;
    for regionIndex = 1:numel(sourceIndexRegions)
        sourceIndex = sourceIndexRegions{regionIndex}(:);
        finalEdge = firstEdge + numel(sourceIndex) - 1;
        edgeStart(firstEdge:finalEdge) = sourceIndex;
        edgeEnd(firstEdge:finalEdge)   = circshift(sourceIndex, -1);
        firstEdge = finalEdge + 1;
    end
    undirectedEdges = [min(edgeStart, edgeEnd), max(edgeStart, edgeEnd)];
    [uniqueEdges, ~, edgeGroup] = unique(undirectedEdges, 'rows');
    multiplicity = accumarray(edgeGroup, 1);
    forwardCount = accumarray(edgeGroup, edgeStart < edgeEnd);
    if any(multiplicity > 2) || ...
            any(forwardCount(multiplicity == 2) ~= 1) || ...
            vertexCount - size(uniqueEdges, 1) + numel(sourceIndexRegions) ~= 1
        verified = false;
        return;
    end

    nextSourceIndex = [2:vertexCount, 1].';
    expectedBoundaryEdges = sortrows([ ...
        min((1:vertexCount).', nextSourceIndex), ...
        max((1:vertexCount).', nextSourceIndex)]);
    actualBoundaryEdges = sortrows(uniqueEdges(multiplicity == 1, :));
    boundaryOccurrence = multiplicity(edgeGroup) == 1;
    directedBoundaryEdges = sortrows([ ...
        edgeStart(boundaryOccurrence), edgeEnd(boundaryOccurrence)]);
    forwardBoundaryEdges = sortrows([(1:vertexCount).', nextSourceIndex]);
    reverseBoundaryEdges = sortrows([nextSourceIndex, (1:vertexCount).']);
    verified = isequal(actualBoundaryEdges, expectedBoundaryEdges) && ...
        (isequal(directedBoundaryEdges, forwardBoundaryEdges) || ...
        isequal(directedBoundaryEdges, reverseBoundaryEdges));
end

function verified = shapeHasExactRingBoundary(shape, vertices_units)
    components = regions(shape);
    if numel(components) ~= 1 || components.NumHoles ~= 0
        verified = false;
        return;
    end
    shapeVertices_units = components.Vertices;
    [shapeVertexIsSource, shapeSourceIndex] = ismember( ...
        shapeVertices_units, vertices_units, 'rows');
    if ~isequal(size(shapeVertices_units), size(vertices_units)) || ...
            ~all(shapeVertexIsSource) || numel(unique(shapeSourceIndex)) ~= numel(shapeSourceIndex)
        verified = false;
        return;
    end
    nextShapeIndex = circshift(shapeSourceIndex, -1);
    shapeEdges = sortrows([ ...
        min(shapeSourceIndex, nextShapeIndex), max(shapeSourceIndex, nextShapeIndex)]);
    vertexCount = size(vertices_units, 1);
    nextSourceIndex = [2:vertexCount, 1].';
    sourceEdges = sortrows([ ...
        min((1:vertexCount).', nextSourceIndex), max((1:vertexCount).', nextSourceIndex)]);
    verified = isequal(shapeEdges, sourceEdges);
end

function shape = unionRegions(regions_units)
    % Union exact convex faces without simplifying away collinear vertices.
    shape = polyshape();
    for regionIndex = 1:numel(regions_units)
        region_units = regions_units{regionIndex};
        shape = union(shape, polyshape( ...
            region_units, 'Simplify', false, 'KeepCollinearPoints', true));
    end
end

function verified = movingBoundaryRemainsSimple(lower_units, upper_units, coordinateScale_units)
    % A crossing can begin or end only when one endpoint becomes collinear
    % with the other moving edge. Split time at every such quadratic root,
    % then test the roots and the open intervals between them.
    verified                    = true;
    vertexCount                 = size(lower_units, 1);
    delta_units                 = upper_units - lower_units;
    orientationTolerance_units2 = 4096 * eps(coordinateScale_units ^ 2);
    positionTolerance_units     = 4096 * eps(coordinateScale_units);

    % Each endpoint is affine in time. These bounds contain every point on
    % each moving edge throughout the interval, so disjoint ranges certify
    % separation without solving any orientation polynomial.
    nextIndex         = [2:vertexCount, 1];
    edgeMinimum_units = min(min(lower_units, lower_units(nextIndex, :)), ...
        min(upper_units, upper_units(nextIndex, :)));
    edgeMaximum_units = max(max(lower_units, lower_units(nextIndex, :)), ...
        max(upper_units, upper_units(nextIndex, :)));
    for firstIndex = 1:vertexCount
        firstNext     = mod(firstIndex, vertexCount) + 1;
        secondIndices = (firstIndex + 1:vertexCount).';
        overlapping   = all(edgeMaximum_units(firstIndex, :) >= ...
            edgeMinimum_units(secondIndices, :) - positionTolerance_units & ...
            edgeMaximum_units(secondIndices, :) >= ...
            edgeMinimum_units(firstIndex, :) - positionTolerance_units, 2);
        for secondIndex = reshape(secondIndices(overlapping), 1, [])
            secondNext = mod(secondIndex, vertexCount) + 1;
            edgesShareAnEndpoint = secondIndex == firstNext || secondNext == firstIndex;
            if edgesShareAnEndpoint
                continue;
            end
            triples = [firstIndex, firstNext, secondIndex; ...
                firstIndex, firstNext, secondNext; ...
                secondIndex, secondNext, firstIndex; ...
                secondIndex, secondNext, firstNext];
            criticalTau = [0; 1];
            for tripleIndex = 1:4
                indices      = triples(tripleIndex, :);
                coefficients = orientationCoefficients( ...
                    lower_units(indices, :), delta_units(indices, :));
                orientationIsIndeterminate = all(abs(coefficients) <= orientationTolerance_units2);
                if orientationIsIndeterminate
                    verified = false;
                    return;
                end
                rootsTau    = realPolynomialRoots(coefficients, orientationTolerance_units2);
                criticalTau = [criticalTau; rootsTau]; %#ok<AGROW>
            end
            criticalTau = unique(min(1, max(0, criticalTau)));
            probeTau    = [criticalTau; (criticalTau(1:end - 1) + criticalTau(2:end)) / 2];
            for tau = reshape(probeTau, 1, [])
                points_units = lower_units + tau * delta_units;
                if segmentsIntersect(points_units(firstIndex, :), points_units(firstNext, :), ...
                        points_units(secondIndex, :), points_units(secondNext, :), ...
                        orientationTolerance_units2, positionTolerance_units)
                    verified = false;
                    return;
                end
            end
        end
    end
end

function coefficients = orientationCoefficients(points_units, delta_units)
    % Expand the triple's signed area as a quadratic in the interval fraction.
    firstEdge_units   = points_units(2, :) - points_units(1, :);
    secondEdge_units  = points_units(3, :) - points_units(1, :);
    firstDelta_units  = delta_units(2, :) - delta_units(1, :);
    secondDelta_units = delta_units(3, :) - delta_units(1, :);
    coefficients = [obstacleAvoidance.geometry.cross2d(firstDelta_units, secondDelta_units), ...
        obstacleAvoidance.geometry.cross2d(firstDelta_units, secondEdge_units) + ...
        obstacleAvoidance.geometry.cross2d(firstEdge_units, secondDelta_units), ...
        obstacleAvoidance.geometry.cross2d(firstEdge_units, secondEdge_units)];
end

function rootsTau = realPolynomialRoots(coefficients, tolerance)
    % Return the real roots inside [0, 1], degrading to the linear case when
    % the leading coefficients are within tolerance of zero.
    leadingIsNegligible = abs(coefficients(1)) <= tolerance;
    linearIsNegligible  = abs(coefficients(2)) <= tolerance;
    if leadingIsNegligible && linearIsNegligible
        rootsTau = zeros(0, 1);
    elseif leadingIsNegligible
        rootsTau = -coefficients(3) / coefficients(2);
    else
        candidate          = roots(coefficients);
        imaginaryTolerance = 1024 * eps(max(1, max(abs(candidate))));
        rootsTau           = real(candidate(abs(imag(candidate)) <= imaginaryTolerance));
    end
    timeTolerance = 1024 * eps;
    rootsTau = rootsTau(rootsTau >= -timeTolerance & rootsTau <= 1 + timeTolerance);
end

function intersects = segmentsIntersect(firstStart_units, firstEnd_units, ...
        secondStart_units, secondEnd_units, orientationTolerance_units2, positionTolerance_units)
    % Report a closed-segment intersection using tolerance-snapped orientations.
    firstDirection_units  = firstEnd_units - firstStart_units;
    secondDirection_units = secondEnd_units - secondStart_units;
    orientations = [obstacleAvoidance.geometry.cross2d( ...
        firstDirection_units, secondStart_units - firstStart_units), ...
        obstacleAvoidance.geometry.cross2d( ...
        firstDirection_units, secondEnd_units - firstStart_units), ...
        obstacleAvoidance.geometry.cross2d( ...
        secondDirection_units, firstStart_units - secondStart_units), ...
        obstacleAvoidance.geometry.cross2d( ...
        secondDirection_units, firstEnd_units - secondStart_units)];
    signs = sign(orientations);
    signs(abs(orientations) <= orientationTolerance_units2) = 0;
    lowerCorner_units = max(min([firstStart_units; firstEnd_units]), ...
        min([secondStart_units; secondEnd_units]));
    upperCorner_units = min(max([firstStart_units; firstEnd_units]), ...
        max([secondStart_units; secondEnd_units]));
    boundingBoxesOverlap = all(lowerCorner_units <= upperCorner_units + positionTolerance_units);
    intersects = boundingBoxesOverlap && ...
        signs(1) * signs(2) <= 0 && signs(3) * signs(4) <= 0;
end

function verified = remainsStrictlyConvex(lower_units, upper_units, coordinateScale_units)
    % Check that interpolated turns keep the same nonzero sign on [0, 1].
    lowerEdge_units      = circshift(lower_units, -1, 1) - lower_units;
    upperEdge_units      = circshift(upper_units, -1, 1) - upper_units;
    lowerTurn_units2 = obstacleAvoidance.geometry.cross2d( ...
        lowerEdge_units, circshift(lowerEdge_units, -1, 1));
    orientation          = sign(sum(lowerTurn_units2));
    turnTolerance_units2 = 4096 * eps(coordinateScale_units ^ 2);
    if orientation == 0 || any(orientation * lowerTurn_units2 <= turnTolerance_units2)
        verified = false;
        return;
    end
    edgeDelta_units     = upperEdge_units - lowerEdge_units;
    nextLowerEdge_units = circshift(lowerEdge_units, -1, 1);
    nextEdgeDelta_units = circshift(edgeDelta_units, -1, 1);
    constant_units2 = obstacleAvoidance.geometry.cross2d( ...
        lowerEdge_units, nextLowerEdge_units);
    linear_units2 = obstacleAvoidance.geometry.cross2d( ...
        edgeDelta_units, nextLowerEdge_units) + ...
        obstacleAvoidance.geometry.cross2d(lowerEdge_units, nextEdgeDelta_units);
    quadratic_units2 = obstacleAvoidance.geometry.cross2d( ...
        edgeDelta_units, nextEdgeDelta_units);
    verified            = true;
    for vertexIndex = 1:size(lower_units, 1)
        candidateTau = [0; 1];
        if quadratic_units2(vertexIndex) ~= 0
            stationaryTau = -linear_units2(vertexIndex) / (2 * quadratic_units2(vertexIndex));
            if stationaryTau > 0 && stationaryTau < 1
                candidateTau(end + 1, 1) = stationaryTau; %#ok<AGROW>
            end
        end
        turn_units2 = constant_units2(vertexIndex) + ...
            linear_units2(vertexIndex) * candidateTau + ...
            quadratic_units2(vertexIndex) * candidateTau .^ 2;
        if any(orientation * turn_units2 <= turnTolerance_units2)
            verified = false;
            return;
        end
    end
end
