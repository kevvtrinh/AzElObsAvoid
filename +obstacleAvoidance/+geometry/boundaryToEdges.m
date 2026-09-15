function [edgeStart_units, edgeEnd_units] = boundaryToEdges(shape, closureTolerance_units)
%% Section 0: Header & Readme
% SYNTAX
%   [edgeStart_units, edgeEnd_units] = ...
%       obstacleAvoidance.geometry.boundaryToEdges(shape, closureTolerance_units)
%**************************************************************************
% PURPOSE
%   - Convert every boundary ring into ordered start and end edge rows.
%**************************************************************************
% INPUTS
%   - shape (scalar polyshape)
%       Polygon geometry whose boundary order is retained.
%   - closureTolerance_units (nonnegative finite scalar)
%       Distance used to recognize a repeated closing vertex.
%**************************************************************************
% OUTPUTS
%   - edgeStart_units (N-by-2 numeric array)
%       Start points in deterministic boundary order.
%   - edgeEnd_units (N-by-2 numeric array)
%       Matching end points. A shape without a usable ring returns 0-by-2
%       arrays; invalid input throws an error.
%**************************************************************************
% UNITS
%   - Geometry and closure tolerance are coordinate units.
%**************************************************************************

%% Section 1: Validate Inputs And Locate The NaN-Separated Rings

if ~isa(shape, "polyshape") || ~isscalar(shape)
    error("boundaryToEdges:InvalidShape", "shape must be a scalar polyshape.");
end
validateattributes(closureTolerance_units, {'numeric'}, {'scalar', 'real', 'finite', 'nonnegative'});

[x_units, y_units] = boundary(shape);
boundaryPosition_units = [double(x_units(:)), double(y_units(:))];

% Each maximal run of finite rows is one ring, in the order polyshape reports.
vertexIsFinite = all(isfinite(boundaryPosition_units), 2);
runStartIndex  = find(vertexIsFinite & [true; ~vertexIsFinite(1:end - 1)]);
runEndIndex    = find(vertexIsFinite & [~vertexIsFinite(2:end); true]);

%% Section 2: Close Every Valid Ring Into Matched Edge Rows

emptyEdges_units      = zeros(0, 2);
edgeStartByRing_units = repmat({emptyEdges_units}, numel(runStartIndex), 1);
edgeEndByRing_units   = repmat({emptyEdges_units}, numel(runStartIndex), 1);

for runIndex = 1:numel(runStartIndex)
    ring_units = boundaryPosition_units(runStartIndex(runIndex):runEndIndex(runIndex), :);

    % A ring with fewer than two distinct vertices cannot produce a segment.
    if size(ring_units, 1) < 2
        continue;
    end

    % Drop a repeated closing vertex so the wrap-around edge is not duplicated.
    if norm(ring_units(end, :) - ring_units(1, :)) <= closureTolerance_units
        ring_units(end, :) = [];
    end
    if size(ring_units, 1) < 2
        continue;
    end

    % Connect adjacent vertices, then the last vertex back to the first.
    edgeStartByRing_units{runIndex} = ring_units;
    edgeEndByRing_units{runIndex}   = ring_units([2:end, 1], :);
end
edgeStart_units = vertcat(edgeStartByRing_units{:});
edgeEnd_units   = vertcat(edgeEndByRing_units{:});
end
