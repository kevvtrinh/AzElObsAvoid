function [edgeStart_deg, edgeEnd_deg] = boundaryToEdges(shape, closureTolerance_deg)
%% Section 0: Header & Readme
% SYNTAX
%   [edgeStart_deg, edgeEnd_deg] = azElGeometry.boundaryToEdges(shape, closureTolerance_deg)
%**************************************************************************
% PURPOSE
%   - Convert every connected boundary ring into explicit start/end edge rows.
%     Visibility and clearance code can then share one deterministic edge order
%     instead of each implementing NaN-separator and ring-closure rules.
%**************************************************************************
% INPUTS
%   - shape (scalar polyshape)
%       Polygon geometry whose boundary traversal order is retained.
%   - closureTolerance_deg (nonnegative finite scalar)
%       Distance for recognizing a repeated final ring vertex.
%**************************************************************************
% OUTPUTS
%   - edgeStart_deg, edgeEnd_deg (N-by-2 arrays)
%       Matched edge endpoints in deterministic boundary order.
%**************************************************************************
% UNITS
%   - Shape vertices, edge endpoints, and tolerance are degrees.
%**************************************************************************

%% Section 1: Validate And Split NaN-Separated Boundary Rings

if ~isa(shape, "polyshape") || ~isscalar(shape)
    error("azElGeometry:boundaryToEdges:InvalidShape", ...
        "shape must be a scalar polyshape.");
end
validateattributes(closureTolerance_deg, {'numeric'}, {'scalar', 'real', 'finite', 'nonnegative'});
[azimuth_deg, elevation_deg] = boundary(shape);
boundaryPosition_deg = [double(azimuth_deg(:)), double(elevation_deg(:))];
finiteRow = all(isfinite(boundaryPosition_deg), 2);
runStart = find(finiteRow & [true; ~finiteRow(1:end - 1)]);
runEnd = find(finiteRow & [~finiteRow(2:end); true]);

%% Section 2: Close Every Valid Ring Into Matched Edge Rows

% polyshape may repeat a ring's first point at the end. Remove only that
% representation duplicate, then connect the final unique point to the first.
emptyEdges_deg = zeros(0, 2);
edgeStartByRing_deg = repmat({emptyEdges_deg}, numel(runStart), 1);
edgeEndByRing_deg = repmat({emptyEdges_deg}, numel(runStart), 1);

% Convert each finite boundary run into one explicitly closed edge ring.
for runIndex = 1:numel(runStart)
    ring_deg = boundaryPosition_deg( runStart(runIndex):runEnd(runIndex), :);
    if size(ring_deg, 1) < 2
        continue;
    end
    if norm(ring_deg(end, :) - ring_deg(1, :)) <= closureTolerance_deg
        ring_deg(end, :) = [];
    end
    if size(ring_deg, 1) < 2
        continue;
    end
    edgeStartByRing_deg{runIndex} = ring_deg;
    edgeEndByRing_deg{runIndex} = ring_deg([2:end 1], :);
end
edgeStart_deg = vertcat(edgeStartByRing_deg{:});
edgeEnd_deg = vertcat(edgeEndByRing_deg{:});
end
