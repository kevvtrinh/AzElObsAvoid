function length_deg = routeLength(route_deg)
%% Section 0: Header & Readme
% SYNTAX
%   length_deg = obstacleAvoidance.geometry.routeLength(route_deg)
%**************************************************************************
% PURPOSE
%   - Measure the Euclidean length of an ordered planar polyline.
%**************************************************************************
% INPUTS
%   - route_deg (N-by-2 numeric matrix)
%       Ordered [azimuth elevation] points; adjacent rows form segments.
%**************************************************************************
% OUTPUTS
%   - length_deg (nonnegative numeric scalar)
%       Sum of all adjacent Euclidean segment lengths.
%**************************************************************************
% UNITS
%   - Input coordinates and returned length are degrees.
%**************************************************************************

%% Section 1: Sum Adjacent Segment Lengths

length_deg = sum(vecnorm(diff(route_deg, 1, 1), 2, 2));
end
