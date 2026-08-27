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

position_deg = [geometry.azimuth_deg, geometry.elevation_deg];
finiteRows = all(isfinite(position_deg), 2);
regionChanges = diff([false; finiteRows; false]);
regionStarts = find(regionChanges == 1);
regionStops = find(regionChanges == -1) - 1;
edgeStart_deg = zeros(0, 2);
edgeEnd_deg = zeros(0, 2);

%% Section 2: Close Every Valid Ring Into Edge Rows

for regionIndex = 1:numel(regionStarts)
    vertices_deg = position_deg( ...
        regionStarts(regionIndex):regionStops(regionIndex), :);
    if size(vertices_deg, 1) < 2
        continue;
    end
    if all(vertices_deg(1, :) == vertices_deg(end, :))
        vertices_deg(end, :) = [];
    end
    edgeStart_deg = [edgeStart_deg; vertices_deg]; %#ok<AGROW>
    edgeEnd_deg = [edgeEnd_deg; ...
        vertices_deg([2:end 1], :)]; %#ok<AGROW>
end
end
