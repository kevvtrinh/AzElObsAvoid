function [modelIsSupported, enclosureShape, regions_units, regionCounts, constructionTime_s] = createMovingCells( ...
    startVertices_units, endVertices_units, safetyMargin_units, preserveAlignment)
%% Section 0: Header & Readme
% SYNTAX
%   [modelIsSupported, enclosureShape, regions_units, regionCounts, constructionTime_s] = ...
%       obstacleAvoidance.obstacles.createMovingCells( ...
%       startVertices_units, endVertices_units, safetyMargin_units)
%   [modelIsSupported, enclosureShape, regions_units, regionCounts, constructionTime_s] = ...
%       obstacleAvoidance.obstacles.createMovingCells( ...
%       startVertices_units, endVertices_units, safetyMargin_units, preserveAlignment)
%**************************************************************************
% PURPOSE
%   - Build convex regions that enclose the obstacle's movement for the whole
%     interval, including its margin. The regions remain occupied throughout
%     that interval, so they may include space the obstacle does not use.
%   - Match the two boundary samples, divide the start shape into triangles,
%     enclose each triangle's movement, then combine and cover those enclosures.
%**************************************************************************
% INPUTS
%   - startVertices_units (N-by-2 numeric array)
%       Original boundary vertices at the interval start, before adding a margin.
%   - endVertices_units (N-by-2 numeric array)
%       Original boundary at the interval end, with the same vertex count.
%   - safetyMargin_units (nonnegative numeric scalar)
%       Margin to include around the original boundary's movement.
%   - preserveAlignment (logical scalar, optional; default false)
%       True when matching vertex indices already define the supplied motion.
%       Otherwise choose the closest vertex ordering before building enclosures.
%**************************************************************************
% OUTPUTS
%   - modelIsSupported (logical scalar)
%       True when the supplied boundaries support this enclosure model.
%   - enclosureShape (polyshape)
%       Combined occupied area of the enclosure, or empty when unsupported.
%   - regions_units (cell array)
%       Convex pieces that together cover exactly enclosureShape.
%   - regionCounts (1-by-2 numeric row)
%       [starting triangle count, final convex region count].
%   - constructionTime_s (1-by-4 numeric row)
%       Times for triangulation, triangle enclosures, combining enclosures,
%       and building the final cover. Invalid margin input throws an error;
%       unsupported boundary geometry returns modelIsSupported = false.
%**************************************************************************
% UNITS
%   - Coordinates and margin use coordinate units; timing uses seconds.
%**************************************************************************

%% Section 1: Check And Match The Boundary Vertices

validateattributes(safetyMargin_units, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
if nargin < 4
    preserveAlignment = false;
end

modelIsSupported   = false;
enclosureShape     = polyshape();
regions_units      = cell(0, 1);
regionCounts       = [0, 0];
constructionTime_s = zeros(1, 4);
if size(startVertices_units, 1) < 3 || ~isequal(size(startVertices_units), size(endVertices_units)) || ...
        size(startVertices_units, 2) ~= 2 || ~all(isfinite([startVertices_units; endVertices_units]), 'all')
    return;
end
if size(unique(startVertices_units, 'rows'), 1) ~= size(startVertices_units, 1) || ...
        size(unique(endVertices_units, 'rows'), 1) ~= size(endVertices_units, 1)
    return;
end
if ~preserveAlignment
    endVertices_units = obstacleAvoidance.obstacles.alignCorrespondingRing( ...
        startVertices_units, endVertices_units);
end
startShape = obstacleAvoidance.geometry.boundaryToShape( ...
    startVertices_units(:, 1), startVertices_units(:, 2));
if isempty(startShape.Vertices) || area(startShape) <= 0
    return;
end

%% Section 2: Divide The Start Shape Into Triangles

% Enclose each triangle's movement separately. One hull around a larger
% piece can include more unused space. Every boundary vertex must appear
% in the triangles so the enclosure covers the complete boundary's movement.
stageTimer   = tic;
triangleMesh = triangulation(startShape);
[vertexIsFromStartBoundary, sourceVertexIndices] = ismember( ...
    triangleMesh.Points, startVertices_units, 'rows');
if ~all(vertexIsFromStartBoundary) || numel(unique(sourceVertexIndices)) ~= size(startVertices_units, 1)
    return;
end
triangleVertexIndices = sourceVertexIndices(triangleMesh.ConnectivityList);
constructionTime_s(1) = toc(stageTimer);
regionCounts(1)       = size(triangleVertexIndices, 1);
if regionCounts(1) == 0
    return;
end

%% Section 3: Enclose Each Triangle's Movement And Margin

stageTimer = tic;

% Start from original vertices and include the margin once. A square with
% half-width sqrt(2) x margin contains the square-corner protection used by
% obstacle preparation, including corners that extend beyond the edge margin.
marginHalfWidth_units = sqrt(2) * safetyMargin_units;
marginCorners_units   = marginHalfWidth_units * [-1, -1; -1, 1; 1, 1; 1, -1];
triangleEnclosures(1, regionCounts(1)) = polyshape();
for triangleIndex = 1:regionCounts(1)
    vertexIndices  = triangleVertexIndices(triangleIndex, :);
    vertices_units = [startVertices_units(vertexIndices, :); endVertices_units(vertexIndices, :)];

    % Add all four margin-square corners at each start and end vertex. Their
    % common hull encloses every intermediate straight-line vertex position.
    if safetyMargin_units > 0
        vertices_units = reshape(permute( ...
            vertices_units + permute(marginCorners_units, [3, 2, 1]), [1, 3, 2]), [], 2);
    end
    hullVertexIndices = convhull(vertices_units(:, 1), vertices_units(:, 2));
    triangleEnclosures(triangleIndex) = polyshape(vertices_units(hullVertexIndices(1:end - 1), :), ...
        'Simplify', false, 'KeepCollinearPoints', true);
end
constructionTime_s(2) = toc(stageTimer);

%% Section 4: Combine The Triangle Enclosures

stageTimer = tic;
combinedTriangleEnclosure = combineShapesInPairs(triangleEnclosures);
constructionTime_s(3) = toc(stageTimer);
if isempty(combinedTriangleEnclosure.Vertices)
    return;
end

%% Section 5: Divide The Enclosure Into Convex Regions

stageTimer = tic;

% Set grid width from the largest vertex movement plus the margin width.
% Also use a minimum width based on a target of 256 squares over the bounding
% box; rounding the row and column counts can exceed that target.
% Taking hulls inside these squares may add occupied area and remove routes,
% but must keep all space already covered by the triangle enclosures.
gridCellTarget      = 256;
enclosureSize_units = max(combinedTriangleEnclosure.Vertices, [], 1) - ...
    min(combinedTriangleEnclosure.Vertices, [], 1);
gridWidth_units = max([max(vecnorm(endVertices_units - startVertices_units, 2, 2)) + 2 * marginHalfWidth_units, ...
    sqrt(prod(max(enclosureSize_units, eps)) / gridCellTarget)]);
if gridWidth_units <= 0
    % If no grid width is available, divide the combined shape directly.
    enclosureShape = combinedTriangleEnclosure;
    regions_units  = obstacleAvoidance.geometry.convexRegions(enclosureShape);
else
    enclosureMinimum_units = min(combinedTriangleEnclosure.Vertices, [], 1);
    enclosureMaximum_units = max(combinedTriangleEnclosure.Vertices, [], 1);
    gridColumnCount        = max(1, ceil((enclosureMaximum_units(1) - enclosureMinimum_units(1)) / gridWidth_units));
    gridRowCount           = max(1, ceil((enclosureMaximum_units(2) - enclosureMinimum_units(2)) / gridWidth_units));
    regions_units          = cell(0, 1);
    for columnIndex = 1:gridColumnCount
        xRange_units = enclosureMinimum_units(1) + [columnIndex - 1, columnIndex] * gridWidth_units;
        for rowIndex = 1:gridRowCount
            yRange_units = enclosureMinimum_units(2) + [rowIndex - 1, rowIndex] * gridWidth_units;
            gridSquare   = polyshape([xRange_units(1), yRange_units(1); xRange_units(2), yRange_units(1); ...
                xRange_units(2), yRange_units(2); xRange_units(1), yRange_units(2)]);
            shapeInsideSquare = intersect(combinedTriangleEnclosure, gridSquare);
            if isempty(shapeInsideSquare.Vertices) || area(shapeInsideSquare) <= 0
                continue;
            end
            for shapePart = regions(shapeInsideSquare).'
                vertices_units = shapePart.Vertices(all(isfinite(shapePart.Vertices), 2), :);
                if size(unique(vertices_units, 'rows'), 1) < 3
                    continue;
                end
                hullVertexIndices = convhull(vertices_units(:, 1), vertices_units(:, 2));
                regions_units{end + 1, 1} = vertices_units(hullVertexIndices(1:end - 1), :); %#ok<AGROW>
            end
        end
    end

    % Use the same covered area for point checks and for motion planning.
    % Combine the hulls, then divide their union into convex pieces so the
    % returned polygon and region list describe exactly the same enclosure.
    coverShapes(1, numel(regions_units)) = polyshape();
    for regionIndex = 1:numel(regions_units)
        coverShapes(regionIndex) = polyshape( ...
            regions_units{regionIndex}, 'Simplify', false, 'KeepCollinearPoints', true);
    end
    enclosureShape = combineShapesInPairs(coverShapes);
    regions_units  = obstacleAvoidance.geometry.convexRegions(enclosureShape);
end
constructionTime_s(4) = toc(stageTimer);
regionCounts(2)       = numel(regions_units);
modelIsSupported = ~isempty(regions_units);
end

%% Section 6: Local Functions

function enclosureShape = combineShapesInPairs(shapesToCombine)
    % Combine neighboring pairs, then combine those results in pairs again.
    % This keeps early operations small. Carry an unpaired final shape into
    % the next pass unchanged.
    while numel(shapesToCombine) > 1
        pairCount      = floor(numel(shapesToCombine) / 2);
        combinedShapes = union(shapesToCombine(1:2:2 * pairCount), shapesToCombine(2:2:2 * pairCount), ...
            'KeepCollinearPoints', true);
        if mod(numel(shapesToCombine), 2)
            combinedShapes(end + 1) = shapesToCombine(end); %#ok<AGROW>
        end
        shapesToCombine = combinedShapes;
    end
    enclosureShape = shapesToCombine;
end
