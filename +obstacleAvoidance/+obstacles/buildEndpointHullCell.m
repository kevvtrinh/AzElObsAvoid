function [modelIsSupported, enclosureShape, regions_units, addedArea_units2] = ...
    buildEndpointHullCell(startBoundary_units, endBoundary_units, startShape, endShape)
%% Section 0: Header & Readme
% SYNTAX
%   [modelIsSupported, enclosureShape, regions_units, addedArea_units2] = ...
%       obstacleAvoidance.obstacles.buildEndpointHullCell( ...
%       startBoundary_units, endBoundary_units, startShape, endShape)
%**************************************************************************
% PURPOSE
%   - Enclose both protected samples with one convex hull: imagine a rubber
%     band stretched around every boundary point from both samples.
%   - Use that whole enclosure for the interval when vertex matches are
%     unknown. Every straight path between any start and end vertex stays
%     inside it, although the enclosure may fill holes and add occupied area.
%   - If one sample is empty, the other sample's hull still occupies the
%     whole interval. An empty sample does not make nearby times obstacle-free.
%**************************************************************************
% INPUTS
%   - startBoundary_units (N-by-2 numeric array)
%       Protected start boundary. Nonfinite rows separate boundary loops.
%   - endBoundary_units (M-by-2 numeric array)
%       Protected end boundary, with the same separator convention.
%   - startShape (scalar polyshape)
%       Prepared protected shape at the interval start.
%   - endShape (scalar polyshape)
%       Prepared protected shape at the interval end.
%**************************************************************************
% OUTPUTS
%   - modelIsSupported (logical scalar)
%       True when at least one boundary loop exists and every loop has at least
%       three distinct vertices and a positive-area hull. Invalid boundary data
%       returns false.
%   - enclosureShape (polyshape)
%       Hull of all finite vertices from both samples, or empty when unsupported.
%   - regions_units (cell array)
%       One [x y] vertex array for the enclosure, or empty when unsupported.
%   - addedArea_units2 (nonnegative numeric scalar)
%       Area added beyond the two protected samples combined.
%**************************************************************************
% UNITS
%   - Geometry uses coordinate units; area uses coordinate units squared.
%**************************************************************************

%% Section 1: Check And Separate The Boundary Loops

modelIsSupported = false;
enclosureShape   = polyshape();
regions_units    = cell(0, 1);
addedArea_units2 = 0;

[startBoundaryIsValid, startRings_units] = splitBoundaryLoops(startBoundary_units);
[endBoundaryIsValid, endRings_units]     = splitBoundaryLoops(endBoundary_units);
if ~startBoundaryIsValid || ~endBoundaryIsValid || ...
        isempty(startRings_units) && isempty(endRings_units)
    return;
end

%% Section 2: Enclose Both Samples With One Convex Hull

allRings_units    = [startRings_units; endRings_units];
vertices_units    = vertcat(allRings_units{:});
hullVertexIndices = convhull(vertices_units(:, 1), vertices_units(:, 2));

% The hull repeats its first vertex at the end. Store that corner only once.
enclosureVertices_units = vertices_units(hullVertexIndices(1:end - 1), :);
endpointHullShape       = polyshape(enclosureVertices_units, 'Simplify', false, 'KeepCollinearPoints', true);
if isempty(endpointHullShape.Vertices) || area(endpointHullShape) <= 0
    return;
end

%% Section 3: Measure The Extra Occupied Area

combinedEndpointShape = union(startShape, endShape);

% Subtract the combined endpoint shapes from the enclosure, then measure
% what remains. This avoids losing a small area difference when subtracting
% two large, nearly equal area values.
addedArea_units2 = area(subtract(endpointHullShape, combinedEndpointShape));
modelIsSupported = isfinite(addedArea_units2) && addedArea_units2 >= 0;
if modelIsSupported
    enclosureShape = endpointHullShape;
    regions_units  = {enclosureVertices_units};
else
    addedArea_units2 = 0;
end
end

%% Section 4: Local Functions

function [boundaryIsValid, rings_units] = splitBoundaryLoops(vertices_units)
    % Separate closed boundary loops and reject incomplete separators.
    % Each loop must contain enough distinct points to enclose an area.
    boundaryIsValid = isnumeric(vertices_units) && isreal(vertices_units) && ...
        size(vertices_units, 2) == 2;
    rings_units = cell(0, 1);
    if ~boundaryIsValid || isempty(vertices_units)
        return;
    end
    coordinateIsFinite = isfinite(vertices_units);
    % A separator fills both columns: [NaN NaN] is valid; [NaN 2] is not.
    if any(xor(coordinateIsFinite(:, 1), coordinateIsFinite(:, 2)))
        boundaryIsValid = false;
        return;
    end
    vertexIsFinite = all(coordinateIsFinite, 2);
    ringStartIndex = find(vertexIsFinite & [true; ~vertexIsFinite(1:end - 1)]);
    ringEndIndex   = find(vertexIsFinite & [~vertexIsFinite(2:end); true]);
    rings_units    = cell(numel(ringStartIndex), 1);
    for ringIndex = 1:numel(ringStartIndex)
        ringVertices_units       = vertices_units(ringStartIndex(ringIndex):ringEndIndex(ringIndex), :);
        uniqueRingVertices_units = unique(ringVertices_units, 'rows', 'stable');
        if size(uniqueRingVertices_units, 1) < 3
            boundaryIsValid = false;
            rings_units     = cell(0, 1);
            return;
        end
        % Distinct points can still lie on one straight line. Each cross
        % product below equals twice a triangle's signed area; all zeros mean
        % there is no area to enclose, so do not call the hull builder.
        vertexOffsets_units     = uniqueRingVertices_units(2:end, :) - uniqueRingVertices_units(1, :);
        signedDoubleArea_units2 = vertexOffsets_units(1, 1) * vertexOffsets_units(2:end, 2) - ...
            vertexOffsets_units(1, 2) * vertexOffsets_units(2:end, 1);
        if ~any(signedDoubleArea_units2 ~= 0)
            boundaryIsValid = false;
            rings_units     = cell(0, 1);
            return;
        end
        hullVertexIndices = convhull(ringVertices_units(:, 1), ringVertices_units(:, 2));
        ringHull          = polyshape(ringVertices_units(hullVertexIndices(1:end - 1), :), ...
            'Simplify', false, 'KeepCollinearPoints', true);
        if isempty(ringHull.Vertices) || area(ringHull) <= 0
            boundaryIsValid = false;
            rings_units     = cell(0, 1);
            return;
        end
        rings_units{ringIndex} = ringVertices_units;
    end
end

