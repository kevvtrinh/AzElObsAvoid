function [edgeStart_deg, edgeEnd_deg] = canonicalBoundaryToEdges(geometry)
%% Section 0: Header & Readme
% SYNTAX
%   [edgeStart_deg, edgeEnd_deg] = ...
%       obstacleAvoidance.geometry.canonicalBoundaryToEdges(geometry)
%**************************************************************************
% PURPOSE
%   - Convert NaN-separated canonical boundary rings into deterministic
%     closed edge rows.
%**************************************************************************
% INPUTS
%   - geometry (scalar canonical boundary struct)
%       azimuth_deg and elevation_deg are matched coordinate vectors. NaN
%       rows separate boundary rings, which may be explicitly closed.
%**************************************************************************
% OUTPUTS
%   - edgeStart_deg, edgeEnd_deg (N-by-2 numeric arrays)
%       Matched directed edge endpoints in canonical boundary order.
%**************************************************************************
% UNITS
%   - Boundary coordinates and edge endpoints are degrees.
%**************************************************************************

%% Section 1: Split NaN-Separated Boundary Rings

% Canonical boundaries use one NaN row between closed rings. Find each finite
% run so edges never connect two separate polygon parts.

position_deg = [geometry.azimuth_deg, geometry.elevation_deg];
finiteRows = all(isfinite(position_deg), 2);
% Padding with false values makes diff produce +1 at every ring entrance and
% -1 immediately after every ring exit, including rings at array boundaries.
regionChanges = diff([false; finiteRows; false]);
regionStarts = find(regionChanges == 1);
regionStops = find(regionChanges == -1) - 1;
edgeStart_deg = zeros(0, 2);
edgeEnd_deg = zeros(0, 2);

%% Section 2: Close Every Valid Ring Into Edge Rows

% Convert each ring to start and end rows. Skip rings with too few distinct
% points to form an edge. Unexpected skips indicate malformed canonical data.

for regionIndex = 1:numel(regionStarts)
    vertices_deg = position_deg( ...
        regionStarts(regionIndex):regionStops(regionIndex), :);
    if size(vertices_deg, 1) < 2
        continue;
    end
    if all(vertices_deg(1, :) == vertices_deg(end, :))
        % Remove a repeated closing vertex so the zero-length duplicate edge is
        % not returned; circular indexing below still closes the polygon.
        vertices_deg(end, :) = [];
    end
    edgeStart_deg = [edgeStart_deg; vertices_deg]; %#ok<AGROW>
    % Growth is bounded by the number of boundary vertices and preserves ring
    % order, which is useful when an edge index is reported diagnostically.
    edgeEnd_deg = [edgeEnd_deg; ...
        vertices_deg([2:end 1], :)]; %#ok<AGROW>
end
end
