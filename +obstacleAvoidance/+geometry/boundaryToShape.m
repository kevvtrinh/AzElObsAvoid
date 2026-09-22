function shape = boundaryToShape(x_units, y_units)
%% Section 0: Header & Readme
% SYNTAX
%   shape = obstacleAvoidance.geometry.boundaryToShape(x_units, y_units)
%**************************************************************************
% PURPOSE
%   - Build a polygon from boundary points. NaN entries separate closed
%     boundary loops, including holes and separate outlines.
%**************************************************************************
% INPUTS
%   - x_units (numeric vector)
%       Boundary x coordinates; NaN separates boundary loops.
%   - y_units (numeric vector)
%       Matching y coordinates, with NaN at the same separator positions.
%**************************************************************************
% OUTPUTS
%   - shape (scalar polyshape)
%       Polygon that retains supplied points along straight edges. Fewer
%       than three finite vertices returns an empty polygon; invalid input
%       throws an error.
%**************************************************************************
% UNITS
%   - Boundary coordinates are coordinate units.
%**************************************************************************

%% Section 1: Construct The Polygon From Its Boundary Points

% Fewer than three finite vertices enclose no area.
vertexIsFinite = isfinite(x_units) & isfinite(y_units);
if nnz(vertexIsFinite) < 3
    shape = polyshape();
    return
end

% Keep extra points along straight edges and turn off simplification.
% Moving-obstacle samples may match points by index; removing points could
% make the same index refer to different boundary locations at different times.
shape = polyshape(x_units, y_units, "Simplify", false, "KeepCollinearPoints", true);
end
