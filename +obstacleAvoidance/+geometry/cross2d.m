function value = cross2d(first_units, second_units)
%% Section 0: Header & Readme
% SYNTAX
%   value = obstacleAvoidance.geometry.cross2d(first_units, second_units)
%**************************************************************************
% PURPOSE
%   - Return row-wise signed two-dimensional cross products.
%**************************************************************************
% INPUTS
%   - first_units (N-by-2 numeric array)
%       First planar vector in each row.
%   - second_units (N-by-2 numeric array)
%       Second planar vector in each row.
%**************************************************************************
% OUTPUTS
%   - value (N-by-1 numeric vector)
%       Signed planar cross product for each pair of rows.
%**************************************************************************
% UNITS
%   - Inputs use coordinate units; outputs use squared coordinate units.
%**************************************************************************

%% Section 1: Compute Row-Wise Cross Products

% Return row-wise signed two-dimensional cross products.
value = first_units(:, 1) .* second_units(:, 2) - first_units(:, 2) .* second_units(:, 1);
end
