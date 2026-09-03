function obstacle = prepareOneObstacle(obstacle, preparationVersion)
%% Section 0: Header & Readme
% SYNTAX
%   obstacle = obstacleAvoidance.obstacles.prepareOneObstacle( ...
%       obstacle, preparationVersion)
%**************************************************************************
% PURPOSE
%   - Prepare one complete obstacle history for repeated geometry queries.
%   - Retain the interval method, bounds, edges, motion, and static status.
%**************************************************************************
% INPUTS
%   - obstacle (scalar canonical obstacle struct)
%       Protected and original source histories remain unchanged.
%   - preparationVersion (positive integer scalar)
%       Version written into the internal preparation record.
%**************************************************************************
% OUTPUTS
%   - obstacle (scalar canonical obstacle struct)
%       InternalPreparation contains reusable source-derived geometry data.
%**************************************************************************
% UNITS
%   - Geometry is degrees, time is seconds, and speed is degrees per second.
%**************************************************************************

%% Section 1: Prepare Sample Geometry

% Later interval checks and occupancy queries need the same shapes, bounds,
% edges, and boundary runs. Create them once from every authoritative sample.

validateattributes(preparationVersion, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'positive'});
sampleCount = numel(obstacle.time_s);
intervalCount = max(0, sampleCount - 1);
sampleShapes = cell(sampleCount, 1);
unionShapes = cell(intervalCount, 1);
deltaAzimuth_deg = cell(intervalCount, 1);
deltaElevation_deg = cell(intervalCount, 1);
matchingTopology = false(intervalCount, 1);
intervalSpeed_deg_s = Inf(intervalCount, 1);
intervalGeometryMethod = strings(intervalCount, 1);
historyBounds_deg = [Inf -Inf Inf -Inf];
sampleBounds_deg = NaN(sampleCount, 4);
sampleEdgeStart_deg = cell(sampleCount, 1);
sampleEdgeEnd_deg = cell(sampleCount, 1);
sampleBoundaryRunBounds = cell(sampleCount, 1);
intervalBounds_deg = NaN(intervalCount, 4);
intervalUnionEdgeStart_deg = cell(intervalCount, 1);
intervalUnionEdgeEnd_deg = cell(intervalCount, 1);
intervalUnionBoundaryRunBounds = cell(intervalCount, 1);
for sampleIndex = 1:sampleCount
    azimuth_deg = double(obstacle.az_deg{sampleIndex}(:));
    elevation_deg = double(obstacle.el_deg{sampleIndex}(:));
    finiteVertex = isfinite(azimuth_deg) & isfinite(elevation_deg);
    if any(finiteVertex)
        historyBounds_deg = [ ...
            min(historyBounds_deg(1), min(azimuth_deg(finiteVertex))), ...
            max(historyBounds_deg(2), max(azimuth_deg(finiteVertex))), ...
            min(historyBounds_deg(3), min(elevation_deg(finiteVertex))), ...
            max(historyBounds_deg(4), max(elevation_deg(finiteVertex)))];
    end
    sampleShapes{sampleIndex} = ...
        obstacleAvoidance.geometry.boundaryToShape( ...
        azimuth_deg, elevation_deg);
    [sampleBounds_deg(sampleIndex, :), ...
        sampleEdgeStart_deg{sampleIndex}, ...
        sampleEdgeEnd_deg{sampleIndex}, ...
        sampleBoundaryRunBounds{sampleIndex}] = ...
        createShapeCache(sampleShapes{sampleIndex});
end

%% Section 2: Check Every History Interval

% A verified corresponding-vertex interval can be queried by interpolation.
% Other intervals need conservative source-derived geometry so later planning
% cannot assume a correspondence that was not proved by the obstacle history.

intervalDuration_s = diff(double(obstacle.time_s(:)));
for intervalIndex = 1:intervalCount
    lowerAzimuth_deg = double(obstacle.az_deg{intervalIndex}(:));
    lowerElevation_deg = double(obstacle.el_deg{intervalIndex}(:));
    upperAzimuth_deg = double(obstacle.az_deg{intervalIndex + 1}(:));
    upperElevation_deg = double(obstacle.el_deg{intervalIndex + 1}(:));
    [matchingTopology(intervalIndex), alignedUpper_deg] = ...
        alignVerifiedSingleRing( ...
        lowerAzimuth_deg, lowerElevation_deg, ...
        upperAzimuth_deg, upperElevation_deg);
    if matchingTopology(intervalIndex)
        deltaAzimuth_deg{intervalIndex} = ...
            alignedUpper_deg(:, 1) - lowerAzimuth_deg;
        deltaElevation_deg{intervalIndex} = ...
            alignedUpper_deg(:, 2) - lowerElevation_deg;
        finiteVertex = isfinite(lowerAzimuth_deg) & ...
            isfinite(lowerElevation_deg);
        speed_deg_s = hypot( ...
            deltaAzimuth_deg{intervalIndex}(finiteVertex), ...
            deltaElevation_deg{intervalIndex}(finiteVertex)) / ...
            intervalDuration_s(intervalIndex);
        intervalSpeed_deg_s(intervalIndex) = max([0; speed_deg_s]);
        intervalGeometryMethod(intervalIndex) = ...
            "linearCorrespondingVertices";
    else
        [shapesAreEquivalent, shapesAreNested] = compareShapes( ...
            sampleShapes{intervalIndex}, sampleShapes{intervalIndex + 1});
        if shapesAreEquivalent
            unionShapes{intervalIndex} = sampleShapes{intervalIndex};
            intervalGeometryMethod(intervalIndex) = ...
                "staticEquivalentSamples";
        elseif shapesAreNested
            % A nested transition uses its exact union without filling a real
            % hole or concavity as a convex hull would.
            unionShapes{intervalIndex} = union( ...
                sampleShapes{intervalIndex}, ...
                sampleShapes{intervalIndex + 1});
            intervalGeometryMethod(intervalIndex) = ...
                "conservativeNestedEndpointUnion";
        else
            unionShapes{intervalIndex} = createEndpointConvexHull( ...
                lowerAzimuth_deg, lowerElevation_deg, ...
                upperAzimuth_deg, upperElevation_deg);
            intervalGeometryMethod(intervalIndex) = ...
                "conservativeEndpointConvexHull";
        end
        intervalSpeed_deg_s(intervalIndex) = 0;
    end
    intervalVertices_deg = [ ...
        lowerAzimuth_deg, lowerElevation_deg; ...
        upperAzimuth_deg, upperElevation_deg];
    intervalBounds_deg(intervalIndex, :) = ...
        finiteBounds(intervalVertices_deg);
    if ~isempty(unionShapes{intervalIndex})
        [~, intervalUnionEdgeStart_deg{intervalIndex}, ...
            intervalUnionEdgeEnd_deg{intervalIndex}, ...
            intervalUnionBoundaryRunBounds{intervalIndex}] = ...
            createShapeCache(unionShapes{intervalIndex});
    end
end

%% Section 3: Calculate Speed Bounds And Static Status

% Occupancy checks use adjacent interval speeds at each sample. Solver routing
% also needs to know whether this complete history is truly time invariant.

sampleSpeed_deg_s = zeros(sampleCount, 1);
for intervalIndex = 1:intervalCount
    sampleSpeed_deg_s(intervalIndex) = max( ...
        sampleSpeed_deg_s(intervalIndex), ...
        intervalSpeed_deg_s(intervalIndex));
    sampleSpeed_deg_s(intervalIndex + 1) = max( ...
        sampleSpeed_deg_s(intervalIndex + 1), ...
        intervalSpeed_deg_s(intervalIndex));
end
staticInterval = intervalGeometryMethod == "staticEquivalentSamples" | ...
    (intervalGeometryMethod == "linearCorrespondingVertices" & ...
    intervalSpeed_deg_s == 0);
isTimeInvariant = sampleCount > 0 && ...
    (sampleCount == 1 || all(staticInterval));
staticShape = polyshape();
if isTimeInvariant
    staticShape = sampleShapes{1};
end

%% Section 4: Create The Prepared Obstacle Record

% Keep a source snapshot beside every derived value. Collection preparation
% can then reject stale caches without guessing which source field changed.

preparation = struct( ...
    "PreparationVersion", preparationVersion, ...
    "SourceSnapshot", createSourceSnapshot(obstacle), ...
    "HistoryBounds_deg", historyBounds_deg, ...
    "SampleShapes", {sampleShapes}, ...
    "SampleBounds_deg", sampleBounds_deg, ...
    "SampleEdgeStart_deg", {sampleEdgeStart_deg}, ...
    "SampleEdgeEnd_deg", {sampleEdgeEnd_deg}, ...
    "SampleBoundaryRunBounds", {sampleBoundaryRunBounds}, ...
    "IntervalUnionShapes", {unionShapes}, ...
    "IntervalBounds_deg", intervalBounds_deg, ...
    "IntervalUnionEdgeStart_deg", {intervalUnionEdgeStart_deg}, ...
    "IntervalUnionEdgeEnd_deg", {intervalUnionEdgeEnd_deg}, ...
    "IntervalUnionBoundaryRunBounds", ...
    {intervalUnionBoundaryRunBounds}, ...
    "DeltaAzimuth_deg", {deltaAzimuth_deg}, ...
    "DeltaElevation_deg", {deltaElevation_deg}, ...
    "MatchingTopology", matchingTopology, ...
    "IntervalGeometryModel", intervalGeometryMethod, ...
    "IntervalSpeedBound_deg_s", intervalSpeed_deg_s, ...
    "SelectedEdgeQueryIsExact", false, ...
    "SampleSpeedBound_deg_s", sampleSpeed_deg_s, ...
    "IsTimeInvariant", isTimeInvariant, ...
    "StaticShape", staticShape);
obstacle.InternalPreparation = preparation;
end

%% Section 5: Local Functions

function snapshot = createSourceSnapshot(obstacle)
% Retain an exact immutable copy of every canonical public source field.
snapshot = struct( ...
    "targetName", obstacle.targetName, ...
    "time_s", obstacle.time_s, ...
    "az_deg", {obstacle.az_deg}, ...
    "el_deg", {obstacle.el_deg}, ...
    "originalAz_deg", {obstacle.originalAz_deg}, ...
    "originalEl_deg", {obstacle.originalEl_deg}, ...
    "safetyMargin_deg", obstacle.safetyMargin_deg, ...
    "status", obstacle.status);
end

function [bounds_deg, edgeStart_deg, edgeEnd_deg, runBounds_deg] = ...
        createShapeCache(shape)
% Cache bounds and ordered edges once for repeated source-derived queries.
vertices_deg = shape.Vertices;
bounds_deg = finiteBounds(vertices_deg);
[edgeStart_deg, edgeEnd_deg] = ...
    obstacleAvoidance.geometry.boundaryToEdges(shape, 0);
[azimuth_deg, elevation_deg] = boundary(shape);
boundary_deg = [double(azimuth_deg(:)), double(elevation_deg(:))];
finiteRow = all(isfinite(boundary_deg), 2);
runStart = find(finiteRow & [true; ~finiteRow(1:end - 1)]);
runEnd = find(finiteRow & [~finiteRow(2:end); true]);
runBounds_deg = NaN(numel(runStart), 4);
for runIndex = 1:numel(runStart)
    runBounds_deg(runIndex, :) = finiteBounds( ...
        boundary_deg(runStart(runIndex):runEnd(runIndex), :));
end
end

function bounds_deg = finiteBounds(vertices_deg)
% Return [minimum azimuth, maximum azimuth, minimum elevation, maximum elevation].
finiteVertices_deg = vertices_deg(all(isfinite(vertices_deg), 2), :);
if isempty(finiteVertices_deg)
    bounds_deg = [Inf -Inf Inf -Inf];
else
    bounds_deg = [ ...
        min(finiteVertices_deg(:, 1)), max(finiteVertices_deg(:, 1)), ...
        min(finiteVertices_deg(:, 2)), max(finiteVertices_deg(:, 2))];
end
end

function [verified, alignedUpper_deg] = alignVerifiedSingleRing( ...
        lowerAzimuth_deg, lowerElevation_deg, ...
        upperAzimuth_deg, upperElevation_deg)
% Canonicalize ring representation and prove safe linear interpolation.
lower_deg = [lowerAzimuth_deg(:), lowerElevation_deg(:)];
upper_deg = [upperAzimuth_deg(:), upperElevation_deg(:)];
verified = false;
alignedUpper_deg = zeros(0, 2);
isSingleRing = size(lower_deg, 1) >= 3 && ...
    isequal(size(lower_deg), size(upper_deg)) && ...
    all(isfinite(lower_deg), "all") && all(isfinite(upper_deg), "all");
if ~isSingleRing
    return;
end
vertexCount = size(lower_deg, 1);
bestCost_deg2 = Inf;
for orientationIndex = 1:2
    orientedUpper_deg = upper_deg;
    if orientationIndex == 2
        orientedUpper_deg = flipud(orientedUpper_deg);
    end
    for shiftCount = 0:vertexCount - 1
        candidateUpper_deg = circshift(orientedUpper_deg, shiftCount, 1);
        cost_deg2 = sum((candidateUpper_deg - lower_deg) .^ 2, "all");
        if cost_deg2 < bestCost_deg2
            bestCost_deg2 = cost_deg2;
            alignedUpper_deg = candidateUpper_deg;
        end
    end
end
delta_deg = alignedUpper_deg - lower_deg;
coordinateScale_deg = max([ ...
    1; abs(lower_deg(:)); abs(alignedUpper_deg(:))]);
translationTolerance_deg = 512 * eps(coordinateScale_deg);
isTranslation = max(abs(delta_deg - delta_deg(1, :)), [], "all") <= ...
    translationTolerance_deg;
verified = isTranslation || ...
    remainsStrictlyConvex(lower_deg, alignedUpper_deg, coordinateScale_deg);
if ~verified
    alignedUpper_deg = zeros(0, 2);
end
end

function verified = remainsStrictlyConvex( ...
        lower_deg, upper_deg, coordinateScale_deg)
% Prove every interpolated turn retains one nonzero orientation on [0, 1].
lowerEdge_deg = circshift(lower_deg, -1, 1) - lower_deg;
upperEdge_deg = circshift(upper_deg, -1, 1) - upper_deg;
lowerTurn_deg2 = cross2d( ...
    lowerEdge_deg, circshift(lowerEdge_deg, -1, 1));
orientation = sign(sum(lowerTurn_deg2));
turnTolerance_deg2 = 4096 * eps(coordinateScale_deg ^ 2);
if orientation == 0 || ...
        any(orientation * lowerTurn_deg2 <= turnTolerance_deg2)
    verified = false;
    return;
end
edgeDelta_deg = upperEdge_deg - lowerEdge_deg;
nextLowerEdge_deg = circshift(lowerEdge_deg, -1, 1);
nextEdgeDelta_deg = circshift(edgeDelta_deg, -1, 1);
constant_deg2 = cross2d(lowerEdge_deg, nextLowerEdge_deg);
linear_deg2 = cross2d(edgeDelta_deg, nextLowerEdge_deg) + ...
    cross2d(lowerEdge_deg, nextEdgeDelta_deg);
quadratic_deg2 = cross2d(edgeDelta_deg, nextEdgeDelta_deg);
verified = true;
for vertexIndex = 1:size(lower_deg, 1)
    candidateTau = [0; 1];
    if quadratic_deg2(vertexIndex) ~= 0
        stationaryTau = -linear_deg2(vertexIndex) / ...
            (2 * quadratic_deg2(vertexIndex));
        if stationaryTau > 0 && stationaryTau < 1
            candidateTau(end + 1, 1) = stationaryTau; %#ok<AGROW>
        end
    end
    turn_deg2 = constant_deg2(vertexIndex) + ...
        linear_deg2(vertexIndex) * candidateTau + ...
        quadratic_deg2(vertexIndex) * candidateTau .^ 2;
    if any(orientation * turn_deg2 <= turnTolerance_deg2)
        verified = false;
        return;
    end
end
end

function value = cross2d(first_deg, second_deg)
% Return row-wise signed two-dimensional cross products.
value = first_deg(:, 1) .* second_deg(:, 2) - ...
    first_deg(:, 2) .* second_deg(:, 1);
end

function [equivalent, nested] = compareShapes(firstShape, secondShape)
% Classify equality and containment from one pair of Boolean differences.
areaScale_deg2 = max([1, area(firstShape), area(secondShape)]);
areaTolerance_deg2 = 512 * eps(areaScale_deg2);
firstIsContained = area(subtract(firstShape, secondShape)) <= ...
    areaTolerance_deg2;
secondIsContained = area(subtract(secondShape, firstShape)) <= ...
    areaTolerance_deg2;
equivalent = firstIsContained && secondIsContained;
nested = firstIsContained || secondIsContained;
end

function shape = createEndpointConvexHull( ...
        lowerAzimuth_deg, lowerElevation_deg, ...
        upperAzimuth_deg, upperElevation_deg)
% Enclose endpoint shapes and admitted linear vertex paths between them.
vertices_deg = [ ...
    lowerAzimuth_deg(:), lowerElevation_deg(:); ...
    upperAzimuth_deg(:), upperElevation_deg(:)];
vertices_deg = unique( ...
    vertices_deg(all(isfinite(vertices_deg), 2), :), "rows", "stable");
if size(vertices_deg, 1) < 3
    shape = polyshape();
    return;
end
hullIndex = convhull(vertices_deg(:, 1), vertices_deg(:, 2));
shape = polyshape(vertices_deg(hullIndex(1:end - 1), :), ...
    "Simplify", false, "KeepCollinearPoints", true);
end
