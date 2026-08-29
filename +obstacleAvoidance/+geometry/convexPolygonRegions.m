function convexRegions = convexPolygonRegions(shape)
%% Section 0: Header & Readme
% SYNTAX
%   convexRegions = obstacleAvoidance.geometry.convexPolygonRegions(shape)
%**************************************************************************
% PURPOSE
%   - Decompose occupied polygon geometry into exact convex regions.
%   - Greedily remove triangulation diagonals for complex outlines without
%     filling gaps or holes; retain the established triangles for small cases.
%**************************************************************************
% INPUTS
%   - shape (scalar polyshape)
%       Possibly disconnected or nonconvex occupied geometry.
%**************************************************************************
% OUTPUTS
%   - convexRegions (column polyshape array)
%       Interior-disjoint convex polygons whose union equals shape.
%**************************************************************************
% UNITS
%   - Geometry coordinates retain the input shape's angular degree units.
%**************************************************************************

%% Section 1: Triangulate And Coarsen Every Connected Region

if ~isa(shape, "polyshape") || ~isscalar(shape)
    error("convexPolygonRegions:InvalidShape", ...
        "shape must be a scalar polyshape.");
end
connectedRegions = regions(shape);
convexRegions = polyshape.empty(0, 1);
sortKeys = zeros(0, 5);
minimumTriangleCountForCoarsening = 65;
regionTriangulations = cell(numel(connectedRegions), 1);
totalTriangleCount = 0;
for connectedRegionIndex = 1:numel(connectedRegions)
    regionTriangulations{connectedRegionIndex} = ...
        triangulation(connectedRegions(connectedRegionIndex));
    totalTriangleCount = totalTriangleCount + size( ...
        regionTriangulations{connectedRegionIndex}.ConnectivityList, 1);
end
coarsenComplexOutline = ...
    totalTriangleCount >= minimumTriangleCountForCoarsening;
usedCoarsening = false;
for connectedRegionIndex = 1:numel(connectedRegions)
    connectedRegion = connectedRegions(connectedRegionIndex);
    finiteVertex_deg = connectedRegion.Vertices;
    finiteVertex_deg = finiteVertex_deg( ...
        all(isfinite(finiteVertex_deg), 2), :);
    hullIndex = convhull(finiteVertex_deg(:, 1), finiteVertex_deg(:, 2));
    hull = polyshape(finiteVertex_deg(hullIndex(1:end - 1), :), ...
        "Simplify", false, "KeepCollinearPoints", true);
    areaTolerance_deg2 = 256 * eps(max(1, area(hull)));
    if abs(area(hull) - area(connectedRegion)) <= areaTolerance_deg2
        convexRegions(end + 1, 1) = connectedRegion; %#ok<AGROW>
        sortKeys(end + 1, :) = regionSortKey(connectedRegion); %#ok<AGROW>
        continue;
    end
    regionTriangulation = regionTriangulations{connectedRegionIndex};
    point_deg = double(regionTriangulation.Points);
    triangleVertexIndex = double(regionTriangulation.ConnectivityList);
    if isempty(triangleVertexIndex)
        continue;
    end
    if ~coarsenComplexOutline
        for triangleIndex = 1:size(triangleVertexIndex, 1)
            triangle_deg = point_deg(triangleVertexIndex(triangleIndex, :), :);
            triangle = polyshape(triangle_deg(:, 1), triangle_deg(:, 2), ...
                "Simplify", false);
            if area(triangle) > 0
                convexRegions(end + 1, 1) = triangle; %#ok<AGROW>
                sortKeys(end + 1, :) = regionSortKey(triangle); %#ok<AGROW>
            end
        end
        continue;
    end
    usedCoarsening = true;
    [cellCycles, edgeTriangleIndex] = ...
        createTriangulationCells(point_deg, triangleVertexIndex);
    [cellCycles, isActive] = coarsenCells( ...
        point_deg, cellCycles, edgeTriangleIndex);
    for cellIndex = reshape(find(isActive), 1, [])
        cycle = cellCycles{cellIndex};
        region = polyshape(point_deg(cycle, :), ...
            "Simplify", false, "KeepCollinearPoints", true);
        if area(region) <= 0
            error("convexPolygonRegions:InvalidMergedCell", ...
                "A coarsened cell has nonpositive area.");
        end
        convexRegions(end + 1, 1) = region; %#ok<AGROW>
        sortKeys(end + 1, :) = regionSortKey(region); %#ok<AGROW>
    end
end
if usedCoarsening && ~isempty(convexRegions)
    [~, order] = sortrows(sortKeys, 1:size(sortKeys, 2));
    convexRegions = convexRegions(order);
end
end

%% Section 2: Local Functions

function key = regionSortKey(region)
% Create deterministic spatial ordering evidence for mixed coarsened output.
vertices_deg = region.Vertices;
vertices_deg = vertices_deg(all(isfinite(vertices_deg), 2), :);
key = [min(vertices_deg, [], 1), max(vertices_deg, [], 1), area(region)];
end

function [cellCycles, edgeTriangleIndex] = ...
        createTriangulationCells(point_deg, triangleVertexIndex)
% Normalize triangles and create one deterministic list of internal edges.
triangleCount = size(triangleVertexIndex, 1);
cellCycles = cell(triangleCount, 1);
for triangleIndex = 1:triangleCount
    cycle = triangleVertexIndex(triangleIndex, :);
    orientation = orientationSign(point_deg(cycle(1), :), ...
        point_deg(cycle(2), :), point_deg(cycle(3), :));
    if orientation == 0
        error("convexPolygonRegions:DegenerateTriangulation", ...
            "MATLAB triangulation returned a zero-area triangle.");
    elseif orientation < 0
        cycle([2 3]) = cycle([3 2]);
    end
    triangleVertexIndex(triangleIndex, :) = cycle;
    cellCycles{triangleIndex} = cycle;
end

edgeStartIndex = [triangleVertexIndex(:, 1); ...
    triangleVertexIndex(:, 2); triangleVertexIndex(:, 3)];
edgeEndIndex = [triangleVertexIndex(:, 2); ...
    triangleVertexIndex(:, 3); triangleVertexIndex(:, 1)];
edgeOwnerIndex = repmat((1:triangleCount).', 3, 1);
edgeKey = sort([edgeStartIndex, edgeEndIndex], 2);
[edgeKey, order] = sortrows(edgeKey, [1 2]);
edgeStartIndex = edgeStartIndex(order);
edgeEndIndex = edgeEndIndex(order);
edgeOwnerIndex = edgeOwnerIndex(order);

maximumInternalEdgeCount = floor(size(edgeKey, 1) / 2);
edgeTriangleIndex = zeros(maximumInternalEdgeCount, 2);
edgeVertexIndex = zeros(maximumInternalEdgeCount, 2);
internalEdgeCount = 0;
groupStartIndex = 1;
while groupStartIndex <= size(edgeKey, 1)
    groupEndIndex = groupStartIndex;
    while groupEndIndex < size(edgeKey, 1) && ...
            isequal(edgeKey(groupEndIndex + 1, :), edgeKey(groupStartIndex, :))
        groupEndIndex = groupEndIndex + 1;
    end
    multiplicity = groupEndIndex - groupStartIndex + 1;
    if multiplicity > 2
        error("convexPolygonRegions:NonmanifoldTriangulation", ...
            "An atomic triangulation edge has more than two owners.");
    elseif multiplicity == 2
        indices = groupStartIndex:groupEndIndex;
        directionsAreOpposite = ...
            edgeStartIndex(indices(1)) == edgeEndIndex(indices(2)) && ...
            edgeEndIndex(indices(1)) == edgeStartIndex(indices(2));
        ownersDiffer = edgeOwnerIndex(indices(1)) ~= edgeOwnerIndex(indices(2));
        if ~(directionsAreOpposite && ownersDiffer)
            error("convexPolygonRegions:InconsistentTriangulation", ...
                "Shared triangulation edges must have opposite directions.");
        end
        internalEdgeCount = internalEdgeCount + 1;
        edgeTriangleIndex(internalEdgeCount, :) = ...
            edgeOwnerIndex(indices).';
        edgeVertexIndex(internalEdgeCount, :) = edgeKey(groupStartIndex, :);
    end
    groupStartIndex = groupEndIndex + 1;
end
edgeTriangleIndex = edgeTriangleIndex(1:internalEdgeCount, :);
edgeVertexIndex = edgeVertexIndex(1:internalEdgeCount, :);

if internalEdgeCount > 0
    firstPoint_deg = point_deg(edgeVertexIndex(:, 1), :);
    secondPoint_deg = point_deg(edgeVertexIndex(:, 2), :);
    swapEndpoints = firstPoint_deg(:, 1) > secondPoint_deg(:, 1) | ...
        (firstPoint_deg(:, 1) == secondPoint_deg(:, 1) & ...
        firstPoint_deg(:, 2) > secondPoint_deg(:, 2));
    lowerPoint_deg = firstPoint_deg;
    upperPoint_deg = secondPoint_deg;
    lowerPoint_deg(swapEndpoints, :) = secondPoint_deg(swapEndpoints, :);
    upperPoint_deg(swapEndpoints, :) = firstPoint_deg(swapEndpoints, :);
    edgeLengthSquared_deg2 = sum((secondPoint_deg - firstPoint_deg) .^ 2, 2);
    priority = [-edgeLengthSquared_deg2, lowerPoint_deg, upperPoint_deg, ...
        edgeVertexIndex];
    [~, order] = sortrows(priority, 1:size(priority, 2));
    edgeTriangleIndex = edgeTriangleIndex(order, :);
end
end

function [cellCycles, isActive] = coarsenCells( ...
        point_deg, cellCycles, edgeTriangleIndex)
% Reconsider only incident diagonals when a neighboring convex cell grows.
cellCount = numel(cellCycles);
edgeCount = size(edgeTriangleIndex, 1);
parentIndex = (1:cellCount).';
isActive = true(cellCount, 1);
incidentEdgeIndex = cell(cellCount, 1);
for cellIndex = 1:cellCount
    incidentEdgeIndex{cellIndex} = find( ...
        any(edgeTriangleIndex == cellIndex, 2));
end
isPending = true(edgeCount, 1);
while any(isPending)
    edgeIndex = find(isPending, 1, "first");
    isPending(edgeIndex) = false;
    firstRootIndex = findRoot(parentIndex, edgeTriangleIndex(edgeIndex, 1));
    secondRootIndex = findRoot(parentIndex, edgeTriangleIndex(edgeIndex, 2));
    if firstRootIndex == secondRootIndex
        continue;
    end
    [mergedCycle, boundaryIsValid] = mergeBoundaryCycles( ...
        cellCycles{firstRootIndex}, cellCycles{secondRootIndex}, ...
        size(point_deg, 1));
    if ~boundaryIsValid || ~cycleIsConvex(point_deg, mergedCycle)
        continue;
    end
    retainedRootIndex = min(firstRootIndex, secondRootIndex);
    removedRootIndex = max(firstRootIndex, secondRootIndex);
    parentIndex(removedRootIndex) = retainedRootIndex;
    cellCycles{retainedRootIndex} = canonicalizeCycle( ...
        point_deg, mergedCycle);
    isActive(removedRootIndex) = false;
    affectedEdges = unique([incidentEdgeIndex{firstRootIndex}; ...
        incidentEdgeIndex{secondRootIndex}]);
    incidentEdgeIndex{retainedRootIndex} = affectedEdges;
    incidentEdgeIndex{removedRootIndex} = zeros(0, 1);
    isPending(affectedEdges) = true;
end
end

function rootIndex = findRoot(parentIndex, cellIndex)
% Follow the deterministic disjoint-set forest to its current live root.
rootIndex = cellIndex;
while parentIndex(rootIndex) ~= rootIndex
    rootIndex = parentIndex(rootIndex);
end
end

function [mergedCycle, isValid] = mergeBoundaryCycles( ...
        firstCycle, secondCycle, pointCount)
% Cancel identical shared atomic edges and reconstruct the surviving cycle.
firstCycle = firstCycle(:);
secondCycle = secondCycle(:);
edgeStartIndex = [firstCycle; secondCycle];
edgeEndIndex = [firstCycle([2:end 1]); secondCycle([2:end 1])];
edgeKey = sort([edgeStartIndex, edgeEndIndex], 2);
[edgeKey, order] = sortrows(edgeKey, [1 2]);
edgeStartIndex = edgeStartIndex(order);
edgeEndIndex = edgeEndIndex(order);
keepEdge = true(size(edgeStartIndex));
sharedEdgeCount = 0;
isValid = true;
groupStartIndex = 1;
while groupStartIndex <= size(edgeKey, 1)
    groupEndIndex = groupStartIndex;
    while groupEndIndex < size(edgeKey, 1) && ...
            isequal(edgeKey(groupEndIndex + 1, :), edgeKey(groupStartIndex, :))
        groupEndIndex = groupEndIndex + 1;
    end
    multiplicity = groupEndIndex - groupStartIndex + 1;
    if multiplicity == 2
        indices = groupStartIndex:groupEndIndex;
        isOpposite = edgeStartIndex(indices(1)) == edgeEndIndex(indices(2)) && ...
            edgeEndIndex(indices(1)) == edgeStartIndex(indices(2));
        if ~isOpposite
            isValid = false;
            break;
        end
        keepEdge(indices) = false;
        sharedEdgeCount = sharedEdgeCount + 1;
    elseif multiplicity > 2
        isValid = false;
        break;
    end
    groupStartIndex = groupEndIndex + 1;
end
if ~isValid || sharedEdgeCount == 0
    mergedCycle = zeros(1, 0);
    isValid = false;
    return;
end
edgeStartIndex = edgeStartIndex(keepEdge);
edgeEndIndex = edgeEndIndex(keepEdge);
if numel(unique(edgeStartIndex)) ~= numel(edgeStartIndex) || ...
        numel(unique(edgeEndIndex)) ~= numel(edgeEndIndex)
    mergedCycle = zeros(1, 0);
    isValid = false;
    return;
end
nextVertexIndex = zeros(pointCount, 1);
nextVertexIndex(edgeStartIndex) = edgeEndIndex;
mergedCycle = zeros(1, numel(edgeStartIndex));
currentVertexIndex = min(edgeStartIndex);
for vertexIndex = 1:numel(mergedCycle)
    mergedCycle(vertexIndex) = currentVertexIndex;
    currentVertexIndex = nextVertexIndex(currentVertexIndex);
    if currentVertexIndex == 0
        isValid = false;
        break;
    end
end
isValid = isValid && currentVertexIndex == mergedCycle(1) && ...
    numel(unique(mergedCycle)) == numel(mergedCycle);
if ~isValid
    mergedCycle = zeros(1, 0);
end
end

function isConvex = cycleIsConvex(point_deg, cycle)
% Accept a simple CCW cycle only when every exact turn is nonnegative.
if numel(cycle) < 3
    isConvex = false;
    return;
end
hasPositiveTurn = false;
for vertexIndex = 1:numel(cycle)
    previousIndex = mod(vertexIndex - 2, numel(cycle)) + 1;
    nextIndex = mod(vertexIndex, numel(cycle)) + 1;
    orientation = orientationSign(point_deg(cycle(previousIndex), :), ...
        point_deg(cycle(vertexIndex), :), point_deg(cycle(nextIndex), :));
    if orientation < 0
        isConvex = false;
        return;
    end
    hasPositiveTurn = hasPositiveTurn || orientation > 0;
end
isConvex = hasPositiveTurn;
end

function cycle = canonicalizeCycle(point_deg, cycle)
% Rotate one CCW cycle to its lexicographically first coordinate and index.
coordinates = point_deg(cycle, :);
[~, order] = sortrows([coordinates, cycle(:)], [1 2 3]);
startIndex = order(1);
cycle = circshift(cycle(:).', 1 - startIndex);
end

function orientation = orientationSign(firstPoint, secondPoint, thirdPoint)
% Return the exact sign of the 2-D orientation determinant for double inputs.
firstDelta = secondPoint - firstPoint;
secondDelta = thirdPoint - firstPoint;
leftProduct = firstDelta(1) * secondDelta(2);
rightProduct = firstDelta(2) * secondDelta(1);
determinant = leftProduct - rightProduct;
errorBound = (3 + 16 * eps) * eps * ...
    (abs(leftProduct) + abs(rightProduct));
if abs(determinant) > errorBound
    orientation = sign(determinant);
    return;
end
firstX = differenceExpansion(secondPoint(1), firstPoint(1));
firstY = differenceExpansion(secondPoint(2), firstPoint(2));
secondX = differenceExpansion(thirdPoint(1), firstPoint(1));
secondY = differenceExpansion(thirdPoint(2), firstPoint(2));
leftExpansion = multiplyExpansions(firstX, secondY);
rightExpansion = multiplyExpansions(firstY, secondX);
determinantExpansion = addExpansions(leftExpansion, -rightExpansion);
nonzeroIndex = find(determinantExpansion ~= 0, 1, "last");
if isempty(nonzeroIndex)
    orientation = 0;
else
    orientation = sign(determinantExpansion(nonzeroIndex));
end
end

function expansion = differenceExpansion(firstValue, secondValue)
% Represent one floating-point subtraction exactly as a two-term expansion.
head = firstValue - secondValue;
secondVirtual = firstValue - head;
firstVirtual = head + secondVirtual;
secondRoundoff = secondVirtual - secondValue;
firstRoundoff = firstValue - firstVirtual;
tail = firstRoundoff + secondRoundoff;
expansion = nonzeroExpansion([tail head]);
end

function product = multiplyExpansions(firstExpansion, secondExpansion)
% Multiply two short nonoverlapping expansions without losing roundoff terms.
product = 0;
for firstIndex = 1:numel(firstExpansion)
    for secondIndex = 1:numel(secondExpansion)
        term = productExpansion( ...
            firstExpansion(firstIndex), secondExpansion(secondIndex));
        product = addExpansions(product, term);
    end
end
end

function expansion = productExpansion(firstValue, secondValue)
% Apply Dekker splitting to represent one product exactly as two terms.
head = firstValue * secondValue;
splitter = 134217729;
firstSplit = splitter * firstValue;
firstHigh = firstSplit - (firstSplit - firstValue);
firstLow = firstValue - firstHigh;
secondSplit = splitter * secondValue;
secondHigh = secondSplit - (secondSplit - secondValue);
secondLow = secondValue - secondHigh;
firstError = head - firstHigh * secondHigh;
secondError = firstError - firstLow * secondHigh;
thirdError = secondError - firstHigh * secondLow;
tail = firstLow * secondLow - thirdError;
expansion = nonzeroExpansion([tail head]);
end

function result = addExpansions(firstExpansion, secondExpansion)
% Add exact expansions one component at a time using error-free TwoSum.
result = firstExpansion;
for componentIndex = 1:numel(secondExpansion)
    result = growExpansion(result, secondExpansion(componentIndex));
end
result = nonzeroExpansion(result);
end

function result = growExpansion(expansion, value)
% Add one scalar exactly while retaining increasing-magnitude components.
result = zeros(1, numel(expansion) + 1);
resultCount = 0;
accumulator = value;
for componentIndex = 1:numel(expansion)
    [accumulator, roundoff] = twoSum(accumulator, expansion(componentIndex));
    if roundoff ~= 0
        resultCount = resultCount + 1;
        result(resultCount) = roundoff;
    end
end
if accumulator ~= 0 || resultCount == 0
    resultCount = resultCount + 1;
    result(resultCount) = accumulator;
end
result = result(1:resultCount);
end

function [head, tail] = twoSum(firstValue, secondValue)
% Return a rounded sum and its exact floating-point residual.
head = firstValue + secondValue;
secondVirtual = head - firstValue;
firstVirtual = head - secondVirtual;
secondRoundoff = secondValue - secondVirtual;
firstRoundoff = firstValue - firstVirtual;
tail = firstRoundoff + secondRoundoff;
end

function expansion = nonzeroExpansion(expansion)
% Remove zero components while retaining one zero for an exact zero value.
expansion = expansion(expansion ~= 0);
if isempty(expansion)
    expansion = 0;
end
end
