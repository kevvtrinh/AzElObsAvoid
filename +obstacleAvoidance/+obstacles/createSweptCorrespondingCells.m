function [supported, shape, regions_units, counts, timing_s] = createSweptCorrespondingCells( ...
    lower_units, upper_units, safetyMargin_units, usesSourceIndex)
%% Section 0: Header & Readme
% SYNTAX
%   [supported, shape, regions_units, counts, timing_s] = ...
%       obstacleAvoidance.obstacles.createSweptCorrespondingCells( ...
%       lower_units, upper_units, safetyMargin_units)
%   [supported, shape, regions_units, counts, timing_s] = ...
%       obstacleAvoidance.obstacles.createSweptCorrespondingCells( ...
%       lower_units, upper_units, safetyMargin_units, usesSourceIndex)
%**************************************************************************
% PURPOSE
%   - Build a conservative swept enclosure for corresponding obstacle rings.
%**************************************************************************
% INPUTS
%   - lower_units (N-by-2 numeric array)
%       Original ring at the interval start.
%   - upper_units (N-by-2 numeric array)
%       Original ring at the interval end, matching lower_units.
%   - safetyMargin_units (nonnegative numeric scalar)
%       Margin included in the conservative swept enclosure.
%   - usesSourceIndex (logical scalar, optional; default false)
%       Whether incoming vertex indices already define correspondence.
%**************************************************************************
% OUTPUTS
%   - supported (logical scalar)
%       True when the source rings admit a swept enclosure.
%   - shape (polyshape)
%       Union of the conservative swept cells, or empty when unsupported.
%   - regions_units (cell array)
%       Convex regions that exactly repartition shape.
%   - counts (1-by-2 numeric row)
%       Triangulation-face and final-region counts.
%   - timing_s (1-by-4 numeric row)
%       Elapsed time for the four construction phases. Invalid margin input
%       throws an error; unsupported ring geometry returns supported = false.
%**************************************************************************
% UNITS
%   - Coordinates and margin use coordinate units; timing uses seconds.
%**************************************************************************

%% Section 1: Require Source Correspondence

validateattributes(safetyMargin_units, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
if nargin < 4
    usesSourceIndex = false;
end
supported     = false;
shape         = polyshape();
regions_units = cell(0, 1);
counts        = [0, 0];
timing_s      = zeros(1, 4);
if size(lower_units, 1) < 3 || ~isequal(size(lower_units), size(upper_units)) || ...
        size(lower_units, 2) ~= 2 || ~all(isfinite([lower_units; upper_units]), 'all')
    return;
end
if size(unique(lower_units, 'rows'), 1) ~= size(lower_units, 1) || ...
        size(unique(upper_units, 'rows'), 1) ~= size(upper_units, 1)
    return;
end
if ~usesSourceIndex
    upper_units = obstacleAvoidance.obstacles.alignCorrespondingRing(lower_units, upper_units);
end
lowerShape = obstacleAvoidance.geometry.boundaryToShape(lower_units(:, 1), lower_units(:, 2));
if isempty(lowerShape.Vertices) || area(lowerShape) <= 0
    return;
end

%% Section 2: Triangulate The Original Sample On Its Source Vertices

% Triangles give the tightest hulls: a merged convex face would sweep the
% hull of a larger vertex set. Every source vertex must appear so the
% piecewise-linear map covers the complete interpolated boundary.
stageTimer              = tic;
mesh                    = triangulation(lowerShape);
[found, sourceIndices] = ismember(mesh.Points, lower_units, 'rows');
if ~all(found) || numel(unique(sourceIndices)) ~= size(lower_units, 1)
    return;
end
faceIndices = sourceIndices(mesh.ConnectivityList);
timing_s(1) = toc(stageTimer);
counts(1)   = size(faceIndices, 1);
if counts(1) == 0
    return;
end

%% Section 3: Hull Every Carried Triangle With Its Margin Squares

stageTimer = tic;
halfWidth_units     = sqrt(2) * safetyMargin_units;
marginCorners_units = halfWidth_units * [-1, -1; -1, 1; 1, 1; 1, -1];
sweeps(1, counts(1)) = polyshape();
for faceIndex = 1:counts(1)
    indices = faceIndices(faceIndex, :);
    vertices_units = [lower_units(indices, :); upper_units(indices, :)];
    if safetyMargin_units > 0
        vertices_units = reshape(permute( ...
            vertices_units + permute(marginCorners_units, [3, 2, 1]), [1, 3, 2]), [], 2);
    end
    hullIndex = convhull(vertices_units(:, 1), vertices_units(:, 2));
    sweeps(faceIndex) = polyshape(vertices_units(hullIndex(1:end - 1), :), ...
        'Simplify', false, 'KeepCollinearPoints', true);
end
timing_s(2) = toc(stageTimer);

%% Section 4: Union The Sweeps

stageTimer = tic;
sweptUnion = balancedUnion(sweeps);
timing_s(3) = toc(stageTimer);
if isempty(sweptUnion.Vertices)
    return;
end

%% Section 5: Cover The Union At The Interval's Displacement Scale

stageTimer = tic;
% The cover is never finer than the sweep's own blur, and never finer than
% a declared budget of grid squares per interval: a coarser cover is only
% more conservative, so the budget trades routes for tractability, never
% validity. It mirrors the visibility search's pair-work budget.
cellBudget       = 256;
extent_units     = max(sweptUnion.Vertices, [], 1) - min(sweptUnion.Vertices, [], 1);
resolution_units = max([max(vecnorm(upper_units - lower_units, 2, 2)) + 2 * halfWidth_units, ...
    sqrt(prod(max(extent_units, eps)) / cellBudget)]);
if resolution_units <= 0
    % No vertex moved and no margin applies: the union is the sample itself.
    shape = sweptUnion;
    regions_units = obstacleAvoidance.geometry.convexRegions(shape);
else
    minimum_units = min(sweptUnion.Vertices, [], 1);
    maximum_units = max(sweptUnion.Vertices, [], 1);
    columnCount = max(1, ceil((maximum_units(1) - minimum_units(1)) / resolution_units));
    rowCount = max(1, ceil((maximum_units(2) - minimum_units(2)) / resolution_units));
    regions_units = cell(0, 1);
    for columnIndex = 1:columnCount
        xRange_units = minimum_units(1) + [columnIndex - 1, columnIndex] * resolution_units;
        for rowIndex = 1:rowCount
            yRange_units = minimum_units(2) + [rowIndex - 1, rowIndex] * resolution_units;
            squareShape = polyshape([xRange_units(1), yRange_units(1); xRange_units(2), yRange_units(1); ...
                xRange_units(2), yRange_units(2); xRange_units(1), yRange_units(2)]);
            pieceShape = intersect(sweptUnion, squareShape);
            if isempty(pieceShape.Vertices) || area(pieceShape) <= 0
                continue;
            end
            for componentShape = regions(pieceShape).'
                vertices_units = componentShape.Vertices(all(isfinite(componentShape.Vertices), 2), :);
                if size(unique(vertices_units, 'rows'), 1) < 3
                    continue;
                end
                hullIndex = convhull(vertices_units(:, 1), vertices_units(:, 2));
                regions_units{end + 1, 1} = vertices_units(hullIndex(1:end - 1), :); %#ok<AGROW>
            end
        end
    end
    % The enclosure used by point queries is the union of the cover pieces.
    % The cells are an exact convex repartition of that union, so queries
    % and cells agree exactly and the cell count follows the enclosure's
    % shape rather than the grid.
    cover(1, numel(regions_units)) = polyshape();
    for regionIndex = 1:numel(regions_units)
        cover(regionIndex) = polyshape( ...
            regions_units{regionIndex}, 'Simplify', false, 'KeepCollinearPoints', true);
    end
    shape = balancedUnion(cover);
    regions_units = obstacleAvoidance.geometry.convexRegions(shape);
end
timing_s(4) = toc(stageTimer);
counts(2) = numel(regions_units);
supported = ~isempty(regions_units);
end

%% Section 6: Local Functions

function shape = balancedUnion(pieces)
    % Keep intermediate boundaries local instead of repeatedly unioning the
    % complete accumulated boundary with one more piece.
    while numel(pieces) > 1
        pairCount = floor(numel(pieces) / 2);
        merged    = union(pieces(1:2:2 * pairCount), pieces(2:2:2 * pairCount), ...
            'KeepCollinearPoints', true);
        if mod(numel(pieces), 2)
            merged(end + 1) = pieces(end); %#ok<AGROW>
        end
        pieces = merged;
    end
    shape = pieces;
end
