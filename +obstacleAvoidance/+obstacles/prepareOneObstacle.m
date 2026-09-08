function obstacle = prepareOneObstacle(obstacle, preparationVersion, sourceSnapshot)
%% Section 0: Header & Readme
% SYNTAX
%   obstacle = obstacleAvoidance.obstacles.prepareOneObstacle( ...
%       obstacle, preparationVersion, sourceSnapshot)
%
% PURPOSE
%   - Prepare one complete obstacle history for repeated geometry queries.
%   - Retain the interval method, bounds, edges, motion, and static status.
%
% INPUTS
%   - obstacle (scalar canonical obstacle struct)
%       Protected and original source histories remain unchanged.
%   - preparationVersion (positive integer scalar)
%       Version written into the internal preparation record.
%   - sourceSnapshot (scalar struct)
%       Source fields assembled by prepareObstacles for cache validation.
%
% OUTPUTS
%   - obstacle (scalar canonical obstacle struct)
%       InternalPreparation contains reusable source-derived geometry data.
%
% UNITS
%   - Geometry is coordinate units, time is seconds, and speed is coordinate units per second.
%

%% Section 1: Prepare Sample Geometry

% Cache shapes, bounds, and edges for repeated queries.

validateattributes(preparationVersion, {'numeric'}, {'real', 'finite', 'scalar', 'integer', 'positive'});
sampleCount                    = numel(obstacle.time_s);
intervalCount                  = max(0, sampleCount - 1);
sampleShapes                   = cell(sampleCount, 1);
unionShapes                    = cell(intervalCount, 1);
deltaX_units               = cell(intervalCount, 1);
deltaY_units             = cell(intervalCount, 1);
matchingTopology               = false(intervalCount, 1);
intervalSpeed_units_s            = Inf(intervalCount, 1);
intervalGeometryMethod         = strings(intervalCount, 1);
historyBounds_units              = [Inf -Inf Inf -Inf];
sampleBounds_units               = NaN(sampleCount, 4);
sampleEdgeStart_units            = cell(sampleCount, 1);
sampleEdgeEnd_units              = cell(sampleCount, 1);
sampleBoundaryRunBounds        = cell(sampleCount, 1);
intervalBounds_units             = NaN(intervalCount, 4);
intervalUnionEdgeStart_units     = cell(intervalCount, 1);
intervalUnionEdgeEnd_units       = cell(intervalCount, 1);
intervalUnionBoundaryRunBounds = cell(intervalCount, 1);
% Process each sample in temporal order and accumulate its result.
for sampleIndex = 1:sampleCount
    x_units   = double(obstacle.x_units{sampleIndex}(:));
    y_units = double(obstacle.y_units{sampleIndex}(:));
    finiteVertex  = isfinite(x_units) & isfinite(y_units);
    if any(finiteVertex)
        historyBounds_units = [ ...
            min(historyBounds_units(1), min(x_units(finiteVertex))), max(historyBounds_units(2), max(x_units(finiteVertex))), min(historyBounds_units(3), min(y_units(finiteVertex))), max(historyBounds_units(4), max(y_units(finiteVertex)))];
    end
    sampleShapes{sampleIndex} = obstacleAvoidance.geometry.boundaryToShape(x_units, y_units);
    [sampleBounds_units(sampleIndex, :), ...
        sampleEdgeStart_units{sampleIndex}, ...
        sampleEdgeEnd_units{sampleIndex}, ...
        sampleBoundaryRunBounds{sampleIndex}] = createShapeCache(sampleShapes{sampleIndex});
end

%% Section 2: Check Every History Interval

% Interpolate only when vertex correspondence is verified.
% Otherwise use geometry that conservatively covers the interval.

intervalDuration_s = diff(double(obstacle.time_s(:)));
% Process each interval while assembling the complete motion or interval result.
for intervalIndex = 1:intervalCount
    lowerX_units   = double(obstacle.x_units{intervalIndex}(:));
    lowerY_units = double(obstacle.y_units{intervalIndex}(:));
    upperX_units   = double(obstacle.x_units{intervalIndex + 1}(:));
    upperY_units = double(obstacle.y_units{intervalIndex + 1}(:));
    [matchingTopology(intervalIndex), alignedUpper_units] = alignVerifiedSingleRing(lowerX_units, lowerY_units, upperX_units, upperY_units);
    if matchingTopology(intervalIndex)
        deltaX_units{intervalIndex} = alignedUpper_units(:, 1) - lowerX_units;
        deltaY_units{intervalIndex} = alignedUpper_units(:, 2) - lowerY_units;
        finiteVertex = isfinite(lowerX_units) & isfinite(lowerY_units);
        speed_units_s  = hypot(deltaX_units{intervalIndex}(finiteVertex), deltaY_units{intervalIndex}(finiteVertex)) / intervalDuration_s(intervalIndex);
        intervalSpeed_units_s(intervalIndex) = max([0; speed_units_s]);
        intervalGeometryMethod(intervalIndex) = "linearCorrespondingVertices";
    else
        [shapesAreEquivalent, shapesAreNested] = compareShapes(sampleShapes{intervalIndex}, sampleShapes{intervalIndex + 1});
        if shapesAreEquivalent
            unionShapes{intervalIndex} = sampleShapes{intervalIndex};
            intervalGeometryMethod(intervalIndex) = "staticEquivalentSamples";
        elseif shapesAreNested
            % Use the exact union for nested shapes to preserve holes and concavities.
            unionShapes{intervalIndex} = union(sampleShapes{intervalIndex}, sampleShapes{intervalIndex + 1});
            intervalGeometryMethod(intervalIndex) = "conservativeNestedEndpointUnion";
        else
            unionShapes{intervalIndex} = createEndpointConvexHull(lowerX_units, lowerY_units, upperX_units, upperY_units);
            intervalGeometryMethod(intervalIndex) = "conservativeEndpointConvexHull";
        end
        intervalSpeed_units_s(intervalIndex) = 0;
    end
    intervalVertices_units = [ ...
        lowerX_units, lowerY_units; ...
        upperX_units, upperY_units];
    intervalBounds_units(intervalIndex, :) = finiteBounds(intervalVertices_units);
    if ~isempty(unionShapes{intervalIndex})
        [~, intervalUnionEdgeStart_units{intervalIndex}, ...
            intervalUnionEdgeEnd_units{intervalIndex}, ...
            intervalUnionBoundaryRunBounds{intervalIndex}] = createShapeCache(unionShapes{intervalIndex});
    end
end

%% Section 3: Calculate Speed Bounds And Static Status

% Cache sample speeds and whether the whole history is static.

sampleSpeed_units_s = zeros(sampleCount, 1);
% Process each interval while assembling the complete motion or interval result.
for intervalIndex = 1:intervalCount
    sampleSpeed_units_s(intervalIndex) = max(sampleSpeed_units_s(intervalIndex), intervalSpeed_units_s(intervalIndex));
    sampleSpeed_units_s(intervalIndex + 1) = max(sampleSpeed_units_s(intervalIndex + 1), intervalSpeed_units_s(intervalIndex));
end
staticInterval  = intervalGeometryMethod == "staticEquivalentSamples" | (intervalGeometryMethod == "linearCorrespondingVertices" & intervalSpeed_units_s == 0);
isTimeInvariant = sampleCount > 0 && (sampleCount == 1 || all(staticInterval));
staticShape     = polyshape();
if isTimeInvariant
    staticShape = sampleShapes{1};
end

%% Section 4: Create The Prepared Obstacle Record

% Save source data so later calls can detect stale caches.

preparation = struct("PreparationVersion", preparationVersion, ...
    "SourceSnapshot", sourceSnapshot, ...
    "HistoryBounds_units", historyBounds_units, ...
    "SampleShapes", {sampleShapes}, ...
    "SampleBounds_units", sampleBounds_units, ...
    "SampleEdgeStart_units", {sampleEdgeStart_units}, ...
    "SampleEdgeEnd_units", {sampleEdgeEnd_units}, ...
    "SampleBoundaryRunBounds", {sampleBoundaryRunBounds}, ...
    "IntervalUnionShapes", {unionShapes}, ...
    "IntervalBounds_units", intervalBounds_units, ...
    "IntervalUnionEdgeStart_units", {intervalUnionEdgeStart_units}, ...
    "IntervalUnionEdgeEnd_units", {intervalUnionEdgeEnd_units}, ...
    "IntervalUnionBoundaryRunBounds", ...
    {intervalUnionBoundaryRunBounds}, ...
    "DeltaX_units", {deltaX_units}, ...
    "DeltaY_units", {deltaY_units}, ...
    "MatchingTopology", matchingTopology, ...
    "IntervalGeometryModel", intervalGeometryMethod, ...
    "IntervalSpeedBound_units_s", intervalSpeed_units_s, ...
    "SelectedEdgeQueryIsExact", false, ...
    "SampleSpeedBound_units_s", sampleSpeed_units_s, ...
    "IsTimeInvariant", isTimeInvariant, ...
    "StaticShape", staticShape);
obstacle.InternalPreparation = preparation;
end

%% Section 5: Local Functions


function [bounds_units, edgeStart_units, edgeEnd_units, runBounds_units] = createShapeCache(shape)
    % Cache bounds and edges for repeated queries.
    vertices_units = shape.Vertices;
    bounds_units   = finiteBounds(vertices_units);
    [edgeStart_units, edgeEnd_units] = obstacleAvoidance.geometry.boundaryToEdges(shape, 0);
    [x_units, y_units] = boundary(shape);
    boundary_units  = [double(x_units(:)), double(y_units(:))];
    finiteRow     = all(isfinite(boundary_units), 2);
    runStart      = find(finiteRow & [true; ~finiteRow(1:end - 1)]);
    runEnd        = find(finiteRow & [~finiteRow(2:end); true]);
    runBounds_units = NaN(numel(runStart), 4);
    % Process each run needed to build shape cache.
    for runIndex = 1:numel(runStart)
        runBounds_units(runIndex, :) = finiteBounds(boundary_units(runStart(runIndex):runEnd(runIndex), :));
    end
end

function bounds_units = finiteBounds(vertices_units)
    % Return [minimum x, maximum x, minimum y, maximum y].
    finiteVertices_units = vertices_units(all(isfinite(vertices_units), 2), :);
    if isempty(finiteVertices_units)
        bounds_units = [Inf -Inf Inf -Inf];
    else
        bounds_units = [ ...
            min(finiteVertices_units(:, 1)), max(finiteVertices_units(:, 1)), min(finiteVertices_units(:, 2)), max(finiteVertices_units(:, 2))];
    end
end

function [verified, alignedUpper_units] = alignVerifiedSingleRing(lowerX_units, lowerY_units, upperX_units, upperY_units)
    % Normalize rings and check whether linear vertex interpolation is safe.
    lower_units        = [lowerX_units(:), lowerY_units(:)];
    upper_units        = [upperX_units(:), upperY_units(:)];
    verified         = false;
    alignedUpper_units = zeros(0, 2);
    isSingleRing     = size(lower_units, 1) >= 3 && isequal(size(lower_units), size(upper_units)) && all(isfinite(lower_units), "all") && all(isfinite(upper_units), "all");
    if ~isSingleRing
        return;
    end
    vertexCount   = size(lower_units, 1);
    bestCost_units2 = Inf;
    % Process each orientation needed to complete align verified single ring.
    for orientationIndex = 1:2
        orientedUpper_units = upper_units;
        if orientationIndex == 2
            orientedUpper_units = flipud(orientedUpper_units);
        end
        % Repeat the shift alternatives needed to refine the current solution.
        for shiftCount = 0:vertexCount - 1
            candidateUpper_units = circshift(orientedUpper_units, shiftCount, 1);
            cost_units2          = sum((candidateUpper_units - lower_units) .^ 2, "all");
            % Use the lower-cost vertex correspondence; ties retain the earlier deterministic match.
            if cost_units2 < bestCost_units2
                bestCost_units2    = cost_units2;
                alignedUpper_units = candidateUpper_units;
            end
        end
    end
    delta_units                = alignedUpper_units - lower_units;
    coordinateScale_units      = max([ 1; abs(lower_units(:)); abs(alignedUpper_units(:))]);
    translationTolerance_units = 512 * eps(coordinateScale_units);
    isTranslation            = max(abs(delta_units - delta_units(1, :)), [], "all") <= translationTolerance_units;
    verified                 = isTranslation || remainsStrictlyConvex(lower_units, alignedUpper_units, coordinateScale_units);
    if ~verified
        alignedUpper_units = zeros(0, 2);
    end
end

function verified = remainsStrictlyConvex(lower_units, upper_units, coordinateScale_units)
    % Check that interpolated turns keep the same nonzero sign on [0, 1].
    lowerEdge_units      = circshift(lower_units, -1, 1) - lower_units;
    upperEdge_units      = circshift(upper_units, -1, 1) - upper_units;
    lowerTurn_units2     = cross2d(lowerEdge_units, circshift(lowerEdge_units, -1, 1));
    orientation        = sign(sum(lowerTurn_units2));
    turnTolerance_units2 = 4096 * eps(coordinateScale_units ^ 2);
    if orientation == 0 || any(orientation * lowerTurn_units2 <= turnTolerance_units2)
        verified = false;
        return;
    end
    edgeDelta_units     = upperEdge_units - lowerEdge_units;
    nextLowerEdge_units = circshift(lowerEdge_units, -1, 1);
    nextEdgeDelta_units = circshift(edgeDelta_units, -1, 1);
    constant_units2     = cross2d(lowerEdge_units, nextLowerEdge_units);
    linear_units2       = cross2d(edgeDelta_units, nextLowerEdge_units) + cross2d(lowerEdge_units, nextEdgeDelta_units);
    quadratic_units2    = cross2d(edgeDelta_units, nextEdgeDelta_units);
    verified          = true;
    % Process each geometric vertex while constructing or checking the region topology.
    for vertexIndex = 1:size(lower_units, 1)
        candidateTau = [0; 1];
        if quadratic_units2(vertexIndex) ~= 0
            stationaryTau = -linear_units2(vertexIndex) / (2 * quadratic_units2(vertexIndex));
            if stationaryTau > 0 && stationaryTau < 1
                candidateTau(end + 1, 1) = stationaryTau; %#ok<AGROW>
            end
        end
        turn_units2 = constant_units2(vertexIndex) + linear_units2(vertexIndex) * candidateTau + quadratic_units2(vertexIndex) * candidateTau .^ 2;
        if any(orientation * turn_units2 <= turnTolerance_units2)
            verified = false;
            return;
        end
    end
end

function value = cross2d(first_units, second_units)
    % Return row-wise signed two-dimensional cross products.
    value = first_units(:, 1) .* second_units(:, 2) - first_units(:, 2) .* second_units(:, 1);
end

function [equivalent, nested] = compareShapes(firstShape, secondShape)
    % Check equality and containment using shape differences.
    areaScale_units2     = max([1, area(firstShape), area(secondShape)]);
    areaTolerance_units2 = 512 * eps(areaScale_units2);
    firstIsContained   = area(subtract(firstShape, secondShape)) <= areaTolerance_units2;
    secondIsContained  = area(subtract(secondShape, firstShape)) <= areaTolerance_units2;
    equivalent         = firstIsContained && secondIsContained;
    nested             = firstIsContained || secondIsContained;
end

function shape = createEndpointConvexHull(lowerX_units, lowerY_units, upperX_units, upperY_units)
    % Enclose both endpoint shapes and their linear vertex paths.
    vertices_units = [ ...
        lowerX_units(:), lowerY_units(:); upperX_units(:), upperY_units(:)];
    vertices_units = unique(vertices_units(all(isfinite(vertices_units), 2), :), "rows", "stable");
    if size(vertices_units, 1) < 3
        shape = polyshape();
        return;
    end
    hullIndex = convhull(vertices_units(:, 1), vertices_units(:, 2));
    shape     = polyshape(vertices_units(hullIndex(1:end - 1), :), "Simplify", false, "KeepCollinearPoints", true);
end
