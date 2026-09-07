function [edgeStart_units, edgeEnd_units] = boundaryToEdges(shape, closureTolerance_units)
%% Section 0: Header & Readme
% SYNTAX
%   [edgeStart_units, edgeEnd_units] = ...
%       obstacleAvoidance.geometry.boundaryToEdges(shape, closureTolerance_units)
%
% PURPOSE
%   - Convert every connected boundary ring into explicit start/end edge rows.
%     Visibility and clearance code can then share one deterministic edge order
%     instead of each implementing NaN-separator and ring-closure rules.
%
% INPUTS
%   - shape (scalar polyshape)
%       Polygon geometry whose boundary traversal order is retained.
%   - closureTolerance_units (nonnegative finite scalar)
%       Distance for recognizing a repeated final ring vertex.
%
% OUTPUTS
%   - edgeStart_units, edgeEnd_units (N-by-2 arrays)
%       Matched edge endpoints in deterministic boundary order.
%
% UNITS
%   - Shape vertices, edge endpoints, and tolerance are coordinate units.
%

%% Section 1: Validate And Split NaN-Separated Boundary Rings

% Split NaN-separated polygon rings.

if ~isa(shape, "polyshape") || ~isscalar(shape)
    error("boundaryToEdges:InvalidShape", "shape must be a scalar polyshape.");
end
validateattributes(closureTolerance_units, {'numeric'}, {'scalar', 'real', 'finite', 'nonnegative'});
[x_units, y_units] = boundary(shape);
boundaryPosition_units = [double(x_units(:)), double(y_units(:))];
finiteRow            = all(isfinite(boundaryPosition_units), 2);
% Find the start and end of each finite run.
runStart = find(finiteRow & [true; ~finiteRow(1:end - 1)]);
runEnd   = find(finiteRow & [~finiteRow(2:end); true]);

%% Section 2: Close Every Valid Ring Into Matched Edge Rows

% Connect adjacent vertices and close each ring.

% Remove a repeated closing vertex before creating edges.
emptyEdges_units      = zeros(0, 2);
edgeStartByRing_units = repmat({emptyEdges_units}, numel(runStart), 1);
edgeEndByRing_units   = repmat({emptyEdges_units}, numel(runStart), 1);

for runIndex = 1:numel(runStart)
    % Rings with fewer than two distinct vertices cannot produce a segment.
    ring_units = boundaryPosition_units(runStart(runIndex):runEnd(runIndex), :);
    if size(ring_units, 1) < 2
        continue;
    end
    if norm(ring_units(end, :) - ring_units(1, :)) <= closureTolerance_units
        ring_units(end, :) = [];
    end
    if size(ring_units, 1) < 2
        continue;
    end
    edgeStartByRing_units{runIndex} = ring_units;
    % Connect the last vertex to the first.
    edgeEndByRing_units{runIndex} = ring_units([2:end 1], :);
end
edgeStart_units = vertcat(edgeStartByRing_units{:});
edgeEnd_units   = vertcat(edgeEndByRing_units{:});
end
