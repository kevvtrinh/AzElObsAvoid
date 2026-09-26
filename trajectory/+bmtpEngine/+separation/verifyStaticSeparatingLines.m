function separatingPlanes = verifyStaticSeparatingLines( ...
    separatingPlanes, controlPoint_units, regions_units, roundoffReserve_units, separationTarget_units)
%% Section 0: Header & Readme
% SYNTAX
%   separatingPlanes = bmtpEngine.separation.verifyStaticSeparatingLines( ...
%       separatingPlanes, controlPoint_units, regions_units, ...
%       roundoffReserve_units, separationTarget_units)
%**************************************************************************
% PURPOSE
%   - Check one curve against several static convex regions together by
%     using the moving-region check with equal start and end vertices.
%**************************************************************************
% INPUTS
%   - separatingPlanes (R-element struct array)
%       One separating line per static convex obstacle region.
%   - controlPoint_units (N-by-2 numeric array)
%       Common Bezier control points for one trajectory segment.
%   - regions_units (R-by-1 cell array)
%       Vertices of each static convex obstacle region.
%   - roundoffReserve_units (nonnegative numeric scalar)
%       Numerical reserve applied on the trajectory side.
%   - separationTarget_units (nonnegative numeric scalar)
%       Required obstacle-side separation target.
%**************************************************************************
% OUTPUTS
%   - separatingPlanes (R-element struct array)
%       Updated offsets and gap bounds. Verified is true only where every
%       separation condition passes for the entire interval.
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

% Static vertices have the same position at both ends of the interval.
% The moving-region proof then uses the same complete-interval bounds.
separatingPlanes = bmtpEngine.separation.verifyMovingSeparatingLines( ...
    separatingPlanes, controlPoint_units, regions_units, regions_units, ...
    roundoffReserve_units, separationTarget_units);
end
