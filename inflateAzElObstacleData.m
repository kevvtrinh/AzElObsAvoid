function inflatedAzElData = inflateAzElObstacleData( ...
        azElData, safetyMargin_deg)
%% Section 0: Header & Readme
% SYNTAX
%   inflatedAzElData = inflateAzElObstacleData( ...
%       azElData, safetyMargin_deg)
%**************************************************************************
% PURPOSE
%   - Rebuild protected boundaries from stored original geometry using one
%     absolute safety margin.
%   - Preserve compatibility for makeAzElObstacleData while preventing
%     cumulative inflation when canonical data is normalized repeatedly.
%**************************************************************************
% INPUTS
%   - azElData (canonical obstacle struct, array, or nested cell array)
%       Obstacle histories containing originalAz_deg and originalEl_deg.
%   - safetyMargin_deg (nonnegative scalar)
%       Absolute construction margin for every returned obstacle.
%**************************************************************************
% OUTPUTS
%   - inflatedAzElData (canonical obstacle struct array)
%       Records whose az_deg and el_deg fields are protected boundaries.
%**************************************************************************
% UNITS
%   - Polygon coordinates and safetyMargin_deg are degrees.

validateattributes(safetyMargin_deg, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'nonnegative'});
inflatedAzElData = combineAzElObstacles(azElData);

%% Section 1: Rebuild Protected Geometry From Original Geometry
% The requested margin is absolute. Rebuilding from original boundaries
% makes repeated calls idempotent and prevents downstream double inflation.
for obstacleIndex = 1:numel(inflatedAzElData)
    obstacle = inflatedAzElData(obstacleIndex);
    obstacle.safetyMargin_deg = double(safetyMargin_deg);
    for sampleIndex = 1:numel(obstacle.time_s)
        originalAzimuth_deg = double( ...
            obstacle.originalAz_deg{sampleIndex}(:));
        originalElevation_deg = double( ...
            obstacle.originalEl_deg{sampleIndex}(:));
        [protectedAzimuth_deg, protectedElevation_deg] = ...
            inflateAzElPolygonSlice(originalAzimuth_deg, ...
            originalElevation_deg, safetyMargin_deg);
        obstacle.az_deg{sampleIndex} = protectedAzimuth_deg;
        obstacle.el_deg{sampleIndex} = protectedElevation_deg;
    end
    inflatedAzElData(obstacleIndex) = ...
        normalizeAzElTimeObstacleData(obstacle);
end
end

function [protectedAzimuth_deg, protectedElevation_deg] = ...
        inflateAzElPolygonSlice(azimuth_deg, elevation_deg, ...
        safetyMargin_deg)
%% Section 0: Header & Readme
% SYNTAX
%   [protectedAzimuth_deg, protectedElevation_deg] = ...
%       inflateAzElPolygonSlice(azimuth_deg, elevation_deg, ...
%       safetyMargin_deg)
%**************************************************************************
% PURPOSE
%   - Apply one topology-aware outward buffer to a complete polygon slice.
%   - Preserve disconnected regions and shrink holes rather than buffering
%     each NaN-separated ring as an independent solid region.
%**************************************************************************
% INPUTS
%   - azimuth_deg, elevation_deg (numeric column vectors)
%       Complete NaN-separated boundary topology for one time slice.
%   - safetyMargin_deg (nonnegative finite scalar)
%       Absolute outward buffer distance.
%**************************************************************************
% OUTPUTS
%   - protectedAzimuth_deg, protectedElevation_deg (numeric columns)
%       Buffered boundary with NaN separators retained.
%**************************************************************************
% UNITS
%   - Coordinates and safetyMargin_deg are degrees.

validateattributes(azimuth_deg, {'numeric'}, {'real', 'column'});
validateattributes(elevation_deg, {'numeric'}, {'real', 'column'});
validateattributes(safetyMargin_deg, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'});
if numel(azimuth_deg) ~= numel(elevation_deg)
    error("inflateAzElPolygonSlice:BoundarySizeMismatch", ...
        "Azimuth and elevation boundaries must have equal lengths.");
end
azimuth_deg = double(azimuth_deg);
elevation_deg = double(elevation_deg);
safetyMargin_deg = double(safetyMargin_deg);
if safetyMargin_deg == 0
    protectedAzimuth_deg = azimuth_deg;
    protectedElevation_deg = elevation_deg;
    return;
end
% Canonical topology accepts any paired nonfinite separator. Polyshape only
% accepts NaN separators, so normalize the representation without changing
% ring membership.
pairedNonfinite = ~isfinite(azimuth_deg) & ~isfinite(elevation_deg);
azimuth_deg(pairedNonfinite) = NaN;
elevation_deg(pairedNonfinite) = NaN;
pairedFinite = isfinite(azimuth_deg) & isfinite(elevation_deg);
if nnz(pairedFinite) < 3
    protectedAzimuth_deg = zeros(0, 1);
    protectedElevation_deg = zeros(0, 1);
    return;
end

sourcePolygon = polyshape(azimuth_deg, elevation_deg, ...
    "Simplify", true, "KeepCollinearPoints", true);
if isempty(sourcePolygon.Vertices) || area(sourcePolygon) <= 0
    error("inflateAzElPolygonSlice:DegeneratePolygon", ...
        "The boundary slice does not define a nonzero-area polygon.");
end
% Square joints are conservative at convex corners and avoid the dense arc
% sampling that would create many artificial retiming corners.
protectedPolygon = polybuffer( ...
    sourcePolygon, safetyMargin_deg, "JointType", "square");

[protectedAzimuth_deg, protectedElevation_deg] = ...
    boundary(protectedPolygon);
protectedAzimuth_deg = double(protectedAzimuth_deg(:));
protectedElevation_deg = double(protectedElevation_deg(:));
while ~isempty(protectedAzimuth_deg) && ...
        ~isfinite(protectedAzimuth_deg(end)) && ...
        ~isfinite(protectedElevation_deg(end))
    protectedAzimuth_deg(end) = [];
    protectedElevation_deg(end) = [];
end
end
