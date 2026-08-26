function shape = boundaryShape(azimuth_deg, elevation_deg)
%% Section 0: Header & Readme
% SYNTAX
%   shape = azElGeometry.boundaryShape(azimuth_deg, elevation_deg)
%**************************************************************************
% PURPOSE
%   - Translate the repository's NaN-separated boundary format into MATLAB's
%     polygon representation without silently dropping meaningful collinear
%     vertices. All planner modules use this adapter instead of interpreting
%     boundary separators independently.
%**************************************************************************
% INPUTS
%   - azimuth_deg, elevation_deg (matched numeric vectors)
%       Paired finite vertices with paired nonfinite ring separators.
%**************************************************************************
% OUTPUTS
%   - shape (scalar polyshape)
%       Unsimplified geometry preserving collinear boundary vertices.
%**************************************************************************
% UNITS
%   - Boundary coordinates are degrees.
%**************************************************************************

%% Section 1: Construct The Shape Without Reinterpreting Geometry

% Fewer than three finite vertices cannot enclose occupied area. Returning an
% empty polyshape gives callers one consistent inactive-geometry value.
finiteVertex = isfinite(azimuth_deg) & isfinite(elevation_deg);
if nnz(finiteVertex) < 3
    shape = polyshape();
    return;
end
% Simplification stays disabled because vertex correspondence across dynamic
% slices is needed to interpolate matching obstacle topology safely.
shape = polyshape(azimuth_deg, elevation_deg, "Simplify", false, "KeepCollinearPoints", true);
end
