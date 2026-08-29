function corridor = createSeedCorridor(seed, segmentCount, segmentBreakTau)
%% Section 0: Header & Readme
% SYNTAX
%   corridor = obstacleAvoidance.planner.createSeedCorridor(seed, segmentCount)
%   corridor = obstacleAvoidance.planner.createSeedCorridor( ...
%       seed, segmentCount, segmentBreakTau)
%**************************************************************************
% PURPOSE
%   - Create one exterior convex support per polynomial segment and occupied
%     seed-envelope region for continuous corridor certification.
%**************************************************************************
% INPUTS
%   - seed (scalar topology-seed struct)
%       position_deg, tau, and optional CorridorBoundary_deg.
%   - segmentCount (positive integer scalar)
%       Number of trajectory polynomial segments.
%   - segmentBreakTau (N+1 numeric vector, optional)
%       Strictly increasing normalized segment boundaries; default uniform.
%**************************************************************************
% OUTPUTS
%   - corridor (structure array)
%       Segment/region supports; empty when no complete support is available.
%**************************************************************************
% UNITS
%   - Position, boundary offsets, and clearance are degrees.
%**************************************************************************

%% Section 1: Resolve Envelope Geometry

validateattributes(segmentCount, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'positive'});
if nargin < 3 || isempty(segmentBreakTau)
    segmentBreakTau = (0:segmentCount).' / segmentCount;
end
validateattributes(segmentBreakTau, {'numeric'}, ...
    {'real', 'finite', 'vector', 'numel', segmentCount + 1});
segmentBreakTau = double(segmentBreakTau(:));
if abs(segmentBreakTau(1)) > 32 * eps || abs(segmentBreakTau(end) - 1) > 32 * eps || ...
        any(diff(segmentBreakTau) <= 0)
    error("createSeedCorridor:InvalidSegmentBreakTau", ...
        "segmentBreakTau must strictly increase from zero to one.");
end
template = struct("SegmentIndex", 0, "RegionIndex", 0, ...
    "Normal", [0 0], "BoundaryOffset_deg", 0, "Clearance_deg", 0);
corridor = repmat(template, 0, 1);
if ~isstruct(seed) || ~isscalar(seed) || ~isfield(seed, "CorridorBoundary_deg") || ...
        isempty(seed.CorridorBoundary_deg)
    return;
end
boundary_deg = double(seed.CorridorBoundary_deg);
if size(boundary_deg, 2) ~= 2 || ...
        any(xor(isfinite(boundary_deg(:, 1)), isfinite(boundary_deg(:, 2))))
    error("createSeedCorridor:InvalidBoundary", ...
        "CorridorBoundary_deg must contain paired finite coordinates.");
end
shape = polyshape(boundary_deg(:, 1), boundary_deg(:, 2), "Simplify", true);
regions = obstacleAvoidance.geometry.convexPolygonRegions(shape);
if isempty(regions)
    return;
end

%% Section 2: Create Complete Exterior Supports

middleTau = (segmentBreakTau(1:end - 1) + segmentBreakTau(2:end)) / 2;
middle_deg = interp1(seed.tau, seed.position_deg, middleTau, "linear");
corridor = repmat(template, segmentCount * numel(regions), 1);
recordCount = 0;
for segmentIndex = 1:segmentCount
    for regionIndex = 1:numel(regions)
        [distance_deg, nearest_deg] = obstacleAvoidance.geometry.pointPolygonClearance( ...
            regions(regionIndex), middle_deg(segmentIndex, :));
        if distance_deg <= 1e-12
            continue;
        end
        normal = (middle_deg(segmentIndex, :) - nearest_deg) / distance_deg;
        vertices_deg = regions(regionIndex).Vertices;
        vertices_deg = vertices_deg(all(isfinite(vertices_deg), 2), :);
        boundaryOffset_deg = max(vertices_deg * normal.');
        seedClearance_deg = middle_deg(segmentIndex, :) * normal.' - boundaryOffset_deg;
        if seedClearance_deg <= 0
            continue;
        end
        recordCount = recordCount + 1;
        corridor(recordCount) = struct("SegmentIndex", segmentIndex, ...
            "RegionIndex", regionIndex, "Normal", normal, ...
            "BoundaryOffset_deg", boundaryOffset_deg, ...
            "Clearance_deg", max(1e-4, 0.25 * seedClearance_deg));
    end
end
corridor = corridor(1:recordCount);
end
