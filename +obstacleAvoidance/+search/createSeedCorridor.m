function corridor = createSeedCorridor(seed, segmentCount)
%% Section 0: Header & Readme
% SYNTAX
%   corridor = obstacleAvoidance.search.createSeedCorridor(seed, segmentCount)
%**************************************************************************
% PURPOSE
%   - Convert seed-envelope geometry into linear outside-half-space records for
%     every polynomial segment. The route midpoint chooses the free side of
%     each convex obstacle region; certification later verifies the complete
%     polynomial, not just that midpoint.
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

% The seed corridor is optional evidence. Return an empty corridor when its
% geometry is absent or invalid. Do not reduce the final collision checks.

% An empty boundary means the caller intentionally requested exact obstacle
% validation without a static corridor certificate.
validateattributes(segmentCount, {'numeric'}, {'real', 'finite', 'scalar', 'integer', 'positive'});
template = struct( "SegmentIndex", 0, "RegionIndex", 0, "Normal", [0 0], "BoundaryOffset_deg", 0, "Clearance_deg", 0);
corridor = repmat(template, 0, 1);
if ~isstruct(seed) || ~isscalar(seed) || ~isfield(seed, "CorridorBoundary_deg") || isempty(seed.CorridorBoundary_deg)
    return;
end
boundary_deg = double(seed.CorridorBoundary_deg);
validateattributes(boundary_deg, {'numeric'}, {'real', '2d', 'ncols', 2});
if any(xor(isfinite(boundary_deg(:, 1)), isfinite(boundary_deg(:, 2))))
    error("createSeedCorridor:InvalidBoundary", ...
        "CorridorBoundary_deg must use paired finite vertices and " + "paired nonfinite separators.");
end
corridorShape = polyshape( boundary_deg(:, 1), boundary_deg(:, 2), "Simplify", true);
corridorRegions = obstacleAvoidance.geometry.convexPolygonRegions(corridorShape);
if isempty(corridorRegions)
    return;
end

%% Section 2: Select One Convex Support Per Segment And Region

% Select one support line for each segment and each convex region. Point the
% normal toward the seed segment. The solver keeps all position coefficients
% beyond this line. This keeps the full segment outside the region.

% For a convex occupied region, the closest boundary point defines an outward
% normal toward the seed. Requiring the whole polynomial projection to stay
% beyond that support keeps the span on the same free side continuously.
segmentMidpointTau = ((1:segmentCount).' - 0.5) / segmentCount;
seedMidpoint_deg = interp1( seed.tau, seed.position_deg, segmentMidpointTau, "linear");
maximumRecordCount = segmentCount * numel(corridorRegions);
corridor = repmat(template, maximumRecordCount, 1);
recordCount = 0;

% Build certificate records for every trajectory segment.
for segmentIndex = 1:segmentCount
    point_deg = seedMidpoint_deg(segmentIndex, :);

    % Give this segment one outward support constraint for each convex occupied region.
    for regionIndex = 1:numel(corridorRegions)
        vertices_deg = corridorRegions(regionIndex).Vertices;
        vertices_deg = vertices_deg(all(isfinite(vertices_deg), 2), :);
        [distance_deg, nearestPoint_deg] = obstacleAvoidance.geometry.pointPolygonClearance( ...
            corridorRegions(regionIndex), point_deg);
        if distance_deg <= 1e-12
            % A seed that touches the region has no reliable outward direction.
            % Omit this optional evidence instead of selecting a direction.
            % A seed midpoint inside or on a region cannot define a safe
            % exterior support and therefore produces no certificate record.
            continue;
        end
        outwardNormal = (point_deg - nearestPoint_deg) / distance_deg;
        boundaryOffset_deg = max(vertices_deg * outwardNormal.');
        seedClearance_deg = point_deg * outwardNormal.' - boundaryOffset_deg;
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
