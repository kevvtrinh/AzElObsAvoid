function separatingPlanes = verifyStaticSeparatingLines( ...
    separatingPlanes, controlPoint_units, regions_units, roundoffReserve_units, separationTarget_units)
%% Section 0: Header & Readme
% SYNTAX
%   separatingPlanes = bmtpEngine.separation.verifyStaticSeparatingLines( ...
%       separatingPlanes, controlPoint_units, regions_units, ...
%       roundoffReserve_units, separationTarget_units)
%**************************************************************************
% PURPOSE
%   - Check one motion curve against several static convex obstacles.
%     Give the moving-region verifier identical start and end polygons so
%     both paths use the same full-interval separation rules.
%**************************************************************************
% INPUTS
%   - separatingPlanes (R-element struct array)
%       One proposed separating line for each of R static regions.
%   - controlPoint_units (N-by-2 numeric array)
%       Bezier controls for the same motion segment against all R regions.
%   - regions_units (R-by-1 cell array)
%       Prepared convex polygons, one N-by-2 [x, y] array per cell.
%   - roundoffReserve_units (nonnegative numeric scalar)
%       Extra curve-side margin for numerical rounding.
%   - separationTarget_units (nonnegative numeric scalar)
%       Minimum required obstacle-side value at each line.
%**************************************************************************
% OUTPUTS
%   - separatingPlanes (R-element struct array)
%       Updated offsets and conservative SignedGap_units bounds. Verified
%       is true only where the complete interval passes the line checks.
%**************************************************************************
% UNITS
%   - Position, offsets, target, reserve, and gap are coordinate units;
%     normals are dimensionless.
%**************************************************************************

%% Section 1: Check The Batch And Reuse The Moving-Region Proof

regionCount = numel(regions_units);
assert(numel(separatingPlanes) == regionCount, ...
    'bmtpEngine:InvalidPlaneBatch', ...
    'Every static source region requires a plane.');
if regionCount == 0
    return
end

% A static vertex is in the same place at both interval ends. Passing each
% region as both inputs makes its interpolated motion constant, while the
% line itself may still change. The shared verifier checks the full time.
separatingPlanes = bmtpEngine.separation.verifyMovingSeparatingLines( ...
    separatingPlanes, controlPoint_units, regions_units, regions_units, ...
    roundoffReserve_units, separationTarget_units);
end
