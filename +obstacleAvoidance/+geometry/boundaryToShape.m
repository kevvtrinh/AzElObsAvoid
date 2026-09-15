function shape = boundaryToShape(x_units, y_units)
%% Section 0: Header & Readme
% SYNTAX
%   shape = obstacleAvoidance.geometry.boundaryToShape(x_units, y_units)
%**************************************************************************
% PURPOSE
%   - Convert NaN-separated boundary coordinates into a polyshape.
%**************************************************************************
% INPUTS
%   - x_units (numeric vector)
%       Boundary x coordinates with nonfinite ring separators.
%   - y_units (numeric vector)
%       Matching boundary y coordinates with paired separators.
%**************************************************************************
% OUTPUTS
%   - shape (scalar polyshape)
%       Unsimplified geometry that preserves collinear vertices. Fewer than
%       three finite vertices returns an empty polyshape; invalid input
%       throws an error.
%**************************************************************************
% UNITS
%   - Boundary coordinates are coordinate units.
%**************************************************************************

%% Section 1: Construct The Shape Without Reinterpreting Geometry

% Fewer than three finite vertices enclose no area.
vertexIsFinite = isfinite(x_units) & isfinite(y_units);
if nnz(vertexIsFinite) < 3
    shape = polyshape();
    return
end

% Keep collinear vertices and disable simplification to preserve vertex
% correspondence between moving-obstacle samples.
shape = polyshape(x_units, y_units, "Simplify", false, "KeepCollinearPoints", true);
end
