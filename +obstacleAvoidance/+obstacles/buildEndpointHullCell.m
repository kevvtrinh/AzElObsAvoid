function [supported, shape, regions_units, addedArea_units2] = ...
    buildEndpointHullCell(lower_units, upper_units, lowerShape, upperShape)
%% Section 0: Header & Readme
% SYNTAX
%   [supported, shape, regions_units, addedArea_units2] = ...
%       obstacleAvoidance.obstacles.buildEndpointHullCell( ...
%       lower_units, upper_units, lowerShape, upperShape)
%**************************************************************************
% PURPOSE
%   - Build one convex interval enclosure from both protected endpoints.
%   - With unknown vertex correspondence, every linear interpolant
%     (1-t)*a+t*b, for any lower vertex a and upper vertex b, lies in the
%     convex hull of all endpoint vertices. No ring identity is assumed.
%   - An empty endpoint contributes no vertices: the other endpoint's hull
%     occupies the full interval, so emptiness never clears its neighbors.
%**************************************************************************
% INPUTS
%   - lower_units (N-by-2 numeric array)
%       Protected lower-sample rings separated by paired nonfinite rows.
%   - upper_units (M-by-2 numeric array)
%       Protected upper-sample rings separated by paired nonfinite rows.
%   - lowerShape (scalar polyshape)
%       Cached protected shape of the lower sample.
%   - upperShape (scalar polyshape)
%       Cached protected shape of the upper sample.
%**************************************************************************
% OUTPUTS
%   - supported (logical scalar)
%       True when at least one ring exists and every ring has at least three
%       distinct vertices and a positive-area hull. Invalid input returns false.
%   - shape (polyshape)
%       Hull of every finite endpoint vertex, or empty when unsupported.
%   - regions_units (cell array)
%       One convex vertex array, or empty when unsupported.
%   - addedArea_units2 (nonnegative numeric scalar)
%       Hull area minus the area of the union of both protected end shapes.
%**************************************************************************
% UNITS
%   - Geometry uses coordinate units; area uses coordinate units squared.
%**************************************************************************

%% Section 1: Validate And Extract Boundary Rings

supported        = false;
shape            = polyshape();
regions_units    = cell(0, 1);
addedArea_units2 = 0;
[lowerIsValid, lowerRings_units] = splitRings(lower_units);
[upperIsValid, upperRings_units] = splitRings(upper_units);
if ~lowerIsValid || ~upperIsValid || ...
        isempty(lowerRings_units) && isempty(upperRings_units)
    return;
end

%% Section 2: Enclose Every Possible Cross-Sample Correspondence

allRings_units = [lowerRings_units; upperRings_units];
vertices_units = vertcat(allRings_units{:});
hullIndex      = convhull(vertices_units(:, 1), vertices_units(:, 2));
region_units   = vertices_units(hullIndex(1:end - 1), :);
endpointHull   = polyshape(region_units, 'Simplify', false, 'KeepCollinearPoints', true);
if isempty(endpointHull.Vertices) || area(endpointHull) <= 0
    return;
end

%% Section 3: Measure The Conservative Added Area

endShapeUnion = union(lowerShape, upperShape);
% Subtraction measures hull area minus the contained endpoint union without
% cancellation between two large, nearly equal scalar areas.
addedArea_units2 = area(subtract(endpointHull, endShapeUnion));
supported        = isfinite(addedArea_units2) && addedArea_units2 >= 0;
if supported
    shape         = endpointHull;
    regions_units = {region_units};
else
    addedArea_units2 = 0;
end
end

%% Section 4: Local Functions

function [valid, rings_units] = splitRings(vertices_units)
    % Split paired-nonfinite separators without repairing input geometry.
    valid       = isnumeric(vertices_units) && isreal(vertices_units) && ...
        size(vertices_units, 2) == 2;
    rings_units = cell(0, 1);
    if ~valid || isempty(vertices_units)
        return;
    end
    finiteCoordinate = isfinite(vertices_units);
    if any(xor(finiteCoordinate(:, 1), finiteCoordinate(:, 2)))
        valid = false;
        return;
    end
    finiteRow   = all(finiteCoordinate, 2);
    startIndex  = find(finiteRow & [true; ~finiteRow(1:end - 1)]);
    endIndex    = find(finiteRow & [~finiteRow(2:end); true]);
    rings_units = cell(numel(startIndex), 1);
    for ringIndex = 1:numel(startIndex)
        ring_units       = vertices_units(startIndex(ringIndex):endIndex(ringIndex), :);
        uniqueRing_units = unique(ring_units, 'rows', 'stable');
        if size(uniqueRing_units, 1) < 3
            valid       = false;
            rings_units = cell(0, 1);
            return;
        end
        % Three distinct points must also span positive area. This exact
        % collinearity check avoids asking Qhull to process a degenerate ring.
        relative_units = uniqueRing_units(2:end, :) - uniqueRing_units(1, :);
        signedDoubleArea_units2 = relative_units(1, 1) * relative_units(2:end, 2) - ...
            relative_units(1, 2) * relative_units(2:end, 1);
        if ~any(signedDoubleArea_units2 ~= 0)
            valid       = false;
            rings_units = cell(0, 1);
            return;
        end
        hullIndex = convhull(ring_units(:, 1), ring_units(:, 2));
        ringHull  = polyshape(ring_units(hullIndex(1:end - 1), :), ...
            'Simplify', false, 'KeepCollinearPoints', true);
        if isempty(ringHull.Vertices) || area(ringHull) <= 0
            valid       = false;
            rings_units = cell(0, 1);
            return;
        end
        rings_units{ringIndex} = ring_units;
    end
end

