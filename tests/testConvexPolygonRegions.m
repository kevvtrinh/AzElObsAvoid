function tests = testConvexPolygonRegions
%% Section 0: Header & Readme
% SYNTAX
%   tests = testConvexPolygonRegions
%**************************************************************************
% PURPOSE
%   - Verify deterministic exact-union convex triangulation coarsening.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest function test array)
%**************************************************************************
% UNITS
%   - Test geometry coordinates are dimensionless degree-like values.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(~)
% Add both maintained production roots for direct package execution.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot, fullfile(repositoryRoot, "trajectory"));
end

function testConvexRectangleCoarsensToOneCell(testCase)
% Remove the rectangle's triangulation diagonal without changing its union.
shape = polyshape([0 4 4 0], [0 0 2 2], ...
    "Simplify", false, "KeepCollinearPoints", true);
verifyGreaterThan(testCase, triangulationCellCount(shape), 1);
convexRegions = obstacleAvoidance.geometry.convexPolygonRegions(shape);
verifyNumElements(testCase, convexRegions, 1);
verifyExactDecomposition(testCase, shape, convexRegions);
end

function testConcaveNotchRetainsSmallTriangulation(testCase)
% Preserve the established small-case partition and a tiny exact reflex notch.
notchDepth = 2 ^ -40;
vertices = [0 0; 4 0; 4 1; 2 1 - notchDepth; 0 1];
shape = polyshape(vertices, ...
    "Simplify", false, "KeepCollinearPoints", true);
triangleCount = triangulationCellCount(shape);
convexRegions = obstacleAvoidance.geometry.convexPolygonRegions(shape);
verifyEqual(testCase, numel(convexRegions), triangleCount);
verifyExactDecomposition(testCase, shape, convexRegions);
end

function testLShapeRetainsSmallTriangulation(testCase)
% Keep small-case solver basins stable without filling the missing corner.
vertices = [0 0; 3 0; 3 1; 1 1; 1 3; 0 3];
shape = polyshape(vertices, ...
    "Simplify", false, "KeepCollinearPoints", true);
triangleCount = triangulationCellCount(shape);
convexRegions = obstacleAvoidance.geometry.convexPolygonRegions(shape);
verifyEqual(testCase, numel(convexRegions), triangleCount);
verifyExactDecomposition(testCase, shape, convexRegions);
verifyFalse(testCase, anyRegionContains(convexRegions, [2 2]));
end

function testLargeOutlineCoarsensExactly(testCase)
% Activate exact diagonal removal only after the complex-outline threshold.
topX = (80:-1:0).';
topY = 2 + 0.2 * mod(topX, 2);
vertices = [0 0; 80 0; topX, topY];
shape = polyshape(vertices, ...
    "Simplify", false, "KeepCollinearPoints", true);
triangleCount = triangulationCellCount(shape);
verifyGreaterThanOrEqual(testCase, triangleCount, 65);
convexRegions = obstacleAvoidance.geometry.convexPolygonRegions(shape);
verifyLessThan(testCase, numel(convexRegions), triangleCount);
verifyExactDecomposition(testCase, shape, convexRegions);
end

function testHoleAndDisconnectedRegionRemainEmpty(testCase)
% Prevent diagonal removal from bridging a hole or disconnected component.
outer = polyshape([0 6 6 0], [0 0 6 6]);
hole = polyshape([2 4 4 2], [2 2 4 4]);
island = polyshape([8 10 10 8], [1 1 3 3]);
shape = union(subtract(outer, hole), island);
convexRegions = obstacleAvoidance.geometry.convexPolygonRegions(shape);
verifyExactDecomposition(testCase, shape, convexRegions);
verifyFalse(testCase, anyRegionContains(convexRegions, [3 3]));
verifyFalse(testCase, anyRegionContains(convexRegions, [7 2]));
end

function testCollinearBoundaryAndInputOrderAreDeterministic(testCase)
% Retain collinear atomic vertices and canonicalize equivalent input orders.
vertices = [0 0; 2 0; 4 0; 4 2; 2 2; 0 2];
firstShape = polyshape(vertices, ...
    "Simplify", false, "KeepCollinearPoints", true);
secondShape = polyshape(flipud(circshift(vertices, 2, 1)), ...
    "Simplify", false, "KeepCollinearPoints", true);
firstRegions = obstacleAvoidance.geometry.convexPolygonRegions(firstShape);
repeatRegions = obstacleAvoidance.geometry.convexPolygonRegions(firstShape);
secondRegions = obstacleAvoidance.geometry.convexPolygonRegions(secondShape);
verifyNumElements(testCase, firstRegions, 1);
verifyEqual(testCase, regionSignatures(firstRegions), ...
    regionSignatures(repeatRegions));
verifyEqual(testCase, regionSignatures(firstRegions), ...
    regionSignatures(secondRegions));
verifyExactDecomposition(testCase, firstShape, firstRegions);
end

function verifyExactDecomposition(testCase, shape, convexRegions)
% Check convexity, disjoint interiors, and both directions of union equality.
combined = polyshape();
areaScale = max(1, area(shape));
areaTolerance = 2 ^ 20 * eps(areaScale);
for regionIndex = 1:numel(convexRegions)
    region = convexRegions(regionIndex);
    vertices = region.Vertices;
    vertices = vertices(all(isfinite(vertices), 2), :);
    hullIndex = convhull(vertices(:, 1), vertices(:, 2));
    hull = polyshape(vertices(hullIndex(1:end - 1), :), ...
        "Simplify", false, "KeepCollinearPoints", true);
    verifyLessThanOrEqual(testCase, ...
        abs(area(hull) - area(region)), areaTolerance);
    for previousIndex = 1:regionIndex - 1
        verifyLessThanOrEqual(testCase, ...
            area(intersect(region, convexRegions(previousIndex))), ...
            areaTolerance);
    end
    combined = union(combined, region);
end
missingArea = area(subtract(shape, combined));
extraArea = area(subtract(combined, shape));
verifyLessThanOrEqual(testCase, missingArea, areaTolerance);
verifyLessThanOrEqual(testCase, extraArea, areaTolerance);
end

function count = triangulationCellCount(shape)
% Count cells in MATLAB's authoritative connected-region triangulations.
connectedRegions = regions(shape);
count = 0;
for regionIndex = 1:numel(connectedRegions)
    regionTriangulation = triangulation(connectedRegions(regionIndex));
    count = count + size(regionTriangulation.ConnectivityList, 1);
end
end

function isContained = anyRegionContains(convexRegions, point)
% Return whether any convex output contains or touches one probe point.
isContained = false;
for regionIndex = 1:numel(convexRegions)
    isContained = isContained || isinterior( ...
        convexRegions(regionIndex), point(1), point(2));
end
end

function signatures = regionSignatures(convexRegions)
% Canonicalize public polyshape vertices for deterministic comparison.
signatures = cell(numel(convexRegions), 1);
for regionIndex = 1:numel(convexRegions)
    vertices = convexRegions(regionIndex).Vertices;
    vertices = vertices(all(isfinite(vertices), 2), :);
    [~, order] = sortrows(vertices, [1 2]);
    firstIndex = order(1);
    vertices = circshift(vertices, 1 - firstIndex, 1);
    if signedArea(vertices) < 0
        vertices = [vertices(1, :); flipud(vertices(2:end, :))];
    end
    signatures{regionIndex} = vertices;
end
end

function areaTwice = signedArea(vertices)
% Return twice the oriented shoelace area for a simple cycle.
nextVertices = vertices([2:end 1], :);
areaTwice = sum(vertices(:, 1) .* nextVertices(:, 2) - ...
    vertices(:, 2) .* nextVertices(:, 1));
end
