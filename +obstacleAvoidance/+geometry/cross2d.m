function crossProduct_units2 = cross2d(firstVector_units, secondVector_units)
%% Section 0: Header & Readme
% SYNTAX
%   crossProduct_units2 = obstacleAvoidance.geometry.cross2d(firstVector_units, secondVector_units)
%**************************************************************************
% PURPOSE
%   - Return row-wise signed two-dimensional cross products.
%**************************************************************************
% INPUTS
%   - firstVector_units (N-by-2 numeric array)
%       First [x y] vector in each row.
%   - secondVector_units (N-by-2 numeric array)
%       Matching second [x y] vector in each row.
%**************************************************************************
% OUTPUTS
%   - crossProduct_units2 (N-by-1 numeric vector)
%       Positive means a left turn from the first vector to the second;
%       negative means a right turn. Zero means parallel vectors or a zero vector.
%**************************************************************************
% UNITS
%   - Inputs use coordinate units; outputs use squared coordinate units.
%**************************************************************************

%% Section 1: Compute Row-Wise Cross Products

% Example: [1 0] points right and [0 1] points up; that left turn gives +1.
crossProduct_units2 = firstVector_units(:, 1) .* secondVector_units(:, 2) - ...
    firstVector_units(:, 2) .* secondVector_units(:, 1);
end
