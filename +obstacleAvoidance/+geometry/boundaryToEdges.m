function [edgeStart_units, edgeEnd_units] = boundaryToEdges(shape, closureTolerance_units)
%% Section 0: Header & Readme
% SYNTAX
%   [edgeStart_units, edgeEnd_units] = ...
%       obstacleAvoidance.geometry.boundaryToEdges(shape, closureTolerance_units)
%**************************************************************************
% PURPOSE
%   - List the start and end points of every polygon edge, including the
%     boundaries of holes and separate outlines.
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

%% Section 1: Check Inputs And Locate Each Boundary Loop

if ~isa(shape, "polyshape") || ~isscalar(shape)
    error("boundaryToEdges:InvalidShape", "shape must be a scalar polyshape.");
end
validateattributes(closureTolerance_units, {'numeric'}, {'scalar', 'real', 'finite', 'nonnegative'});

[x_units, y_units] = boundary(shape);
boundaryPosition_units = [double(x_units(:)), double(y_units(:))];

% A ring is one closed boundary loop. Rows containing NaN separate the loops;
% each consecutive group of finite [x y] rows supplies one loop's vertices.
vertexIsFinite = all(isfinite(boundaryPosition_units), 2);
ringStartIndex = find(vertexIsFinite & [true; ~vertexIsFinite(1:end - 1)]);
ringEndIndex   = find(vertexIsFinite & [~vertexIsFinite(2:end); true]);

%% Section 2: Connect The Vertices Of Each Boundary Loop

emptyEdges_units      = zeros(0, 2);
edgeStartByRing_units = repmat({emptyEdges_units}, numel(ringStartIndex), 1);
edgeEndByRing_units   = repmat({emptyEdges_units}, numel(ringStartIndex), 1);

for ringIndex = 1:numel(ringStartIndex)
    ringVertices_units = boundaryPosition_units(ringStartIndex(ringIndex):ringEndIndex(ringIndex), :);

    % At least two vertices are needed to form an edge.
    if size(ringVertices_units, 1) < 2
        continue;
    end

    % Treat first and last points within the closure tolerance as the same
    % corner. Keep it once; the edge assembly below closes the loop.
    if norm(ringVertices_units(end, :) - ringVertices_units(1, :)) <= closureTolerance_units
        ringVertices_units(end, :) = [];
    end
    if size(ringVertices_units, 1) < 2
        continue;
    end

    % Connect adjacent vertices, then the last vertex back to the first.
    edgeStartByRing_units{ringIndex} = ringVertices_units;
    edgeEndByRing_units{ringIndex}   = ringVertices_units([2:end, 1], :);
end
edgeStart_units = vertcat(edgeStartByRing_units{:});
edgeEnd_units   = vertcat(edgeEndByRing_units{:});
end
