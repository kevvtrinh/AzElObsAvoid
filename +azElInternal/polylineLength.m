function length_deg = polylineLength(points_deg)
%% Section 0: Header & Readme
% SYNTAX
%   length_deg = azElInternal.polylineLength(points_deg)
%**************************************************************************
% PURPOSE
%   - Compute the Euclidean length of a two-axis polyline.
%**************************************************************************
% INPUTS
%   - points_deg (N-by-2 numeric matrix)
%       Ordered [azimuth elevation] points. Callers validate coordinates at
%       the public boundary. Zero or one point defines a zero-length path.
%**************************************************************************
% OUTPUTS
%   - length_deg (nonnegative numeric scalar)
%       Sum of the lengths of all consecutive line segments.
%**************************************************************************
% UNITS
%   - Input coordinates and output length are degrees.
%**************************************************************************

%% Section 1: Compute Consecutive Segment Lengths

if size(points_deg, 1) < 2
    length_deg = 0;
    return;
end

step_deg = diff(points_deg, 1, 1);
length_deg = sum(hypot(step_deg(:, 1), step_deg(:, 2)));
end
