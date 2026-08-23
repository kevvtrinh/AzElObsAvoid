function corridor = buildSeedCorridor(seed, segmentCount)
%% Section 0: Header & Readme
% SYNTAX
%   corridor = azElPlannerMethods.hs3.internal.validation.buildSeedCorridor(seed, segmentCount)
%**************************************************************************
% PURPOSE
%   - Build continuous convex support corridors from seed-only polygons.
%**************************************************************************
% INPUTS
%   - seed (scalar topology-seed struct)
%       position_deg, tau, and CorridorBoundary_deg define the route and
%       optional conservative seed geometry.
%   - segmentCount (positive integer scalar)
%       Number of uniform trajectory polynomial segments.
%**************************************************************************
% OUTPUTS
%   - corridor (structure array)
%       Segment index, outward normal, boundary offset, and clearance for
%       each convex seed region that the segment must remain outside.
%**************************************************************************
% UNITS
%   - Position, boundary offsets, and clearance are degrees.
%**************************************************************************

%% Section 1: Validate Inputs & Resolve Geometry

validateattributes(segmentCount, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'positive'});
template = struct( ...
    "SegmentIndex", 0, "RegionIndex", 0, "Normal", [0 0], ...
    "BoundaryOffset_deg", 0, "Clearance_deg", 0);
corridor = repmat(template, 0, 1);
if ~isstruct(seed) || ~isscalar(seed) || ...
        ~isfield(seed, "CorridorBoundary_deg") || ...
        isempty(seed.CorridorBoundary_deg)
    return;
end
boundary_deg = double(seed.CorridorBoundary_deg);
validateattributes(boundary_deg, {'numeric'}, ...
    {'real', '2d', 'ncols', 2});
if any(xor(isfinite(boundary_deg(:, 1)), ...
        isfinite(boundary_deg(:, 2))))
    error("buildSeedCorridor:InvalidBoundary", ...
        "CorridorBoundary_deg must use paired finite vertices and " + ...
        "paired nonfinite separators.");
end
corridorShape = polyshape( ...
    boundary_deg(:, 1), boundary_deg(:, 2), "Simplify", true);
corridorRegions = regions(corridorShape);
if isempty(corridorRegions)
    return;
end

%% Section 2: Select One Convex Support Per Segment And Region

segmentMidpointTau = ((1:segmentCount).' - 0.5) / segmentCount;
seedMidpoint_deg = interp1( ...
    seed.tau, seed.position_deg, segmentMidpointTau, "linear");
maximumRecordCount = segmentCount * numel(corridorRegions);
corridor = repmat(template, maximumRecordCount, 1);
recordCount = 0;

% Visit every trajectory segment because each one needs an independent
% supporting half-space for every applicable envelope region.
for segmentIndex = 1:segmentCount
    point_deg = seedMidpoint_deg(segmentIndex, :);

    % Compare this segment midpoint with every disconnected corridor region.
    for regionIndex = 1:numel(corridorRegions)
        vertices_deg = corridorRegions(regionIndex).Vertices;
        vertices_deg = vertices_deg(all(isfinite(vertices_deg), 2), :);
        [nearestPoint_deg, distance_deg] = nearestBoundaryPoint(vertices_deg, point_deg);
        if distance_deg <= 1e-12
            continue;
        end
        outwardNormal = (point_deg - nearestPoint_deg) / distance_deg;
        boundaryOffset_deg = max(vertices_deg * outwardNormal.');
        seedClearance_deg = point_deg * outwardNormal.' - ...
            boundaryOffset_deg;
        if seedClearance_deg <= 0
            continue;
        end
        recordCount = recordCount + 1;
        corridor(recordCount).SegmentIndex = segmentIndex;
        corridor(recordCount).RegionIndex = regionIndex;
        corridor(recordCount).Normal = outwardNormal;
        corridor(recordCount).BoundaryOffset_deg = boundaryOffset_deg;
        corridor(recordCount).Clearance_deg = max(1e-4, 0.25 * seedClearance_deg);
    end
end
corridor = corridor(1:recordCount);
end


function [nearestPoint_deg, minimumDistance_deg] = nearestBoundaryPoint( ...
        vertices_deg, point_deg)
% Project one point onto the nearest closed polygon edge.
edgeStart_deg = vertices_deg;
edgeEnd_deg = vertices_deg([2:end 1], :);
edgeDelta_deg = edgeEnd_deg - edgeStart_deg;
edgeLengthSquared_deg2 = sum(edgeDelta_deg.^2, 2);
validEdge = edgeLengthSquared_deg2 > eps;
edgeStart_deg = edgeStart_deg(validEdge, :);
edgeDelta_deg = edgeDelta_deg(validEdge, :);
edgeLengthSquared_deg2 = edgeLengthSquared_deg2(validEdge);
fraction = sum((point_deg - edgeStart_deg) .* edgeDelta_deg, 2) ./ ...
    edgeLengthSquared_deg2;
fraction = min(1, max(0, fraction));
projectedPoint_deg = edgeStart_deg + fraction .* edgeDelta_deg;
distance_deg = vecnorm(projectedPoint_deg - point_deg, 2, 2);
[minimumDistance_deg, edgeIndex] = min(distance_deg);
nearestPoint_deg = projectedPoint_deg(edgeIndex, :);
end
