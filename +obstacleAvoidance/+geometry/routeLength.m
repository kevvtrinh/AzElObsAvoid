function length_units = routeLength(route_units)
%% Section 0: Header & Readme
% SYNTAX
%   length_units = obstacleAvoidance.geometry.routeLength(route_units)
%
% PURPOSE
%   - Measure the Euclidean length of an ordered planar polyline.
%
% INPUTS
%   - route_units (N-by-2 numeric matrix)
%       Ordered [x y] points; adjacent rows form segments.
%
% OUTPUTS
%   - length_units (nonnegative numeric scalar)
%       Sum of all adjacent Euclidean segment lengths.
%
% UNITS
%   - Input coordinates and returned length are coordinate units.
%

%% Section 1: Sum Adjacent Segment Lengths

length_units = sum(vecnorm(diff(route_units, 1, 1), 2, 2));
end
