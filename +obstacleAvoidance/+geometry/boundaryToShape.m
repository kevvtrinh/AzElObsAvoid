function shape = boundaryToShape(azimuth_deg, elevation_deg)
%% Section 0: Header & Readme
% SYNTAX
%   shape = obstacleAvoidance.geometry.boundaryToShape(azimuth_deg, elevation_deg)
%
% PURPOSE
%   - Translate the repository's NaN-separated boundary format into MATLAB's
%     polygon representation without silently dropping meaningful collinear
%     vertices. All planner modules use this adapter instead of interpreting
%     boundary separators independently.
%
% INPUTS
%   - azimuth_deg, elevation_deg (matched numeric vectors)
%       Paired finite vertices with paired nonfinite ring separators.
%
% OUTPUTS
%   - shape (scalar polyshape)
%       Unsimplified geometry preserving collinear boundary vertices.
%
% UNITS
%   - Boundary coordinates are degrees.
%

%% Section 1: Construct The Shape Without Reinterpreting Geometry

% Preserve ring order without repairing the geometry.

% Fewer than three finite vertices enclose no area.
finiteVertex = isfinite(azimuth_deg) & isfinite(elevation_deg);
if nnz(finiteVertex) < 3
    shape = polyshape();
    return;
end
% Keep collinear vertices and disable simplification to preserve
% vertex correspondence between moving-obstacle samples.
shape = polyshape(azimuth_deg, elevation_deg, "Simplify", false, "KeepCollinearPoints", true);
end
