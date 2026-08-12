function inflatedAzElData = inflateAzElObstacleData( ...
        azElData, safetyMargin_deg)
%% Section 0: Header & Readme
% SYNTAX
%   inflatedAzElData = inflateAzElObstacleData( ...
%       azElData, safetyMargin_deg)
%**************************************************************************
% PURPOSE
%   - Apply a Euclidean safety margin to every az/el polygon slice before
%     buildAzElTimeObstacleField packs the geometry.
%   - Return ordinary canonical azElData so planning, visualization, and
%     collision checks all use the same prebuilt polygon boundary.
%**************************************************************************
% INPUTS
%   - azElData (canonical obstacle struct, array, or nested cell array)
%       Original obstacle polygon histories.
%   - safetyMargin_deg (nonnegative scalar)
%       Clearance added to every polygon boundary in az/el degrees.
%**************************************************************************
% OUTPUTS
%   - inflatedAzElData (canonical obstacle struct array)
%       Safety-inflated polygon histories ready for field construction.
%**************************************************************************
% UNITS
%   - Polygon coordinates and safetyMargin_deg are degrees.

validateattributes(safetyMargin_deg, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'nonnegative'});
inflatedAzElData = combineAzElObstacles(azElData);
if safetyMargin_deg <= 0
    return;
end

%% Section 1: Inflate Every Finite Ring In Every Time Slice
for obstacleIndex = 1:numel(inflatedAzElData)
    obstacle = inflatedAzElData(obstacleIndex);
    obstacle.targetName = string(obstacle.targetName) + ...
        sprintf(" + %.3f deg safety margin", safetyMargin_deg);
    obstacle.safetyMargin_deg = ...
        double(obstacle.safetyMargin_deg) + safetyMargin_deg;
    for sampleIndex = 1:numel(obstacle.time_s)
        rawAzimuth_deg = double(obstacle.az_deg{sampleIndex}(:));
        rawElevation_deg = double(obstacle.el_deg{sampleIndex}(:));
        finiteVertex = isfinite(rawAzimuth_deg) & ...
            isfinite(rawElevation_deg);
        ringChanges = diff([false; finiteVertex; false]);
        ringStart = find(ringChanges == 1);
        ringStop = find(ringChanges == -1) - 1;
        inflatedAzimuth_deg = zeros(0, 1);
        inflatedElevation_deg = zeros(0, 1);
        outputRingCount = 0;
        for ringIndex = 1:numel(ringStart)
            rows = ringStart(ringIndex):ringStop(ringIndex);
            rawRegion_deg = [ ...
                rawAzimuth_deg(rows), rawElevation_deg(rows)];
            bufferedRegions_deg = inflateAzElPolygonRegion( ...
                rawRegion_deg, safetyMargin_deg);
            for bufferedRegionIndex = 1:numel(bufferedRegions_deg)
                outputRingCount = outputRingCount + 1;
                bufferedRegion_deg = ...
                    bufferedRegions_deg{bufferedRegionIndex};
                if outputRingCount > 1
                    inflatedAzimuth_deg(end + 1, 1) = NaN; %#ok<AGROW>
                    inflatedElevation_deg(end + 1, 1) = NaN; %#ok<AGROW>
                end
                inflatedAzimuth_deg = [inflatedAzimuth_deg; ...
                    bufferedRegion_deg(:, 1)]; %#ok<AGROW>
                inflatedElevation_deg = [inflatedElevation_deg; ...
                    bufferedRegion_deg(:, 2)]; %#ok<AGROW>
            end
        end
        if outputRingCount == 0
            obstacle.az_deg{sampleIndex} = zeros(0, 1);
            obstacle.el_deg{sampleIndex} = zeros(0, 1);
            continue;
        end
        obstacle.az_deg{sampleIndex} = inflatedAzimuth_deg;
        obstacle.el_deg{sampleIndex} = inflatedElevation_deg;
    end
    inflatedAzElData(obstacleIndex) = ...
        normalizeAzElTimeObstacleData(obstacle);
end
end

function bufferedRegions_deg = inflateAzElPolygonRegion( ...
        region_deg, safetyMargin_deg)
%% Section 0: Header & Readme
% SYNTAX
%   bufferedRegions_deg = inflateAzElPolygonRegion( ...
%       region_deg, safetyMargin_deg)
%**************************************************************************
% PURPOSE
%   - Apply a compact conservative outward buffer to one finite
%     azimuth/elevation polygon ring.
%   - Preserve all connected output components as independently packable
%     polygon regions.
%**************************************************************************
% INPUTS
%   - region_deg (N-by-2 finite numeric matrix, N >= 3)
%       One polygon ring in [azimuth elevation] coordinates.
%   - safetyMargin_deg (nonnegative finite scalar)
%       Minimum outward-buffer distance. Square joints conservatively add
%       slightly more clearance near convex corners.
%**************************************************************************
% OUTPUTS
%   - bufferedRegions_deg (column cell array)
%       One boundary matrix per connected buffered polygon component.
%       NaN rows inside a component retain any interior hole boundaries.
%**************************************************************************
% UNITS
%   - Coordinates and safetyMargin_deg are degrees.

validateattributes(region_deg, {'numeric'}, ...
    {'real', 'finite', '2d', 'ncols', 2});
validateattributes(safetyMargin_deg, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'});
region_deg = double(region_deg);
safetyMargin_deg = double(safetyMargin_deg);
if size(region_deg, 1) < 3
    error("inflateAzElPolygonRegion:TooFewVertices", ...
        "region_deg must contain at least three finite vertices.");
end

% Remove only an explicit closing duplicate. polyshape owns all remaining
% topology cleanup and reports a consistently ordered boundary afterward.
coordinateScale_deg = max(1, max(abs(region_deg), [], "all"));
duplicateTolerance_deg = 1e-12 * coordinateScale_deg;
if size(region_deg, 1) > 3 && norm( ...
        region_deg(end, :) - region_deg(1, :)) <= duplicateTolerance_deg
    region_deg(end, :) = [];
end

sourcePolygon = polyshape( ...
    region_deg(:, 1), region_deg(:, 2), ...
    "Simplify", true, "KeepCollinearPoints", true);
if isempty(sourcePolygon.Vertices) || area(sourcePolygon) <= 0
    error("inflateAzElPolygonRegion:DegeneratePolygon", ...
        "region_deg does not define a nonzero-area polygon.");
end
if safetyMargin_deg == 0
    bufferedPolygon = sourcePolygon;
else
    % Square joints form a compact conservative envelope of the round
    % Euclidean buffer. This avoids hundreds of tiny arc edges that an
    % exact-polyline retimer would otherwise treat as separate corners,
    % while never reducing the requested edge clearance.
    bufferedPolygon = polybuffer( ...
        sourcePolygon, safetyMargin_deg, "JointType", "square");
end

polygonComponents = regions(bufferedPolygon);
bufferedRegions_deg = cell(numel(polygonComponents), 1);
for componentIndex = 1:numel(polygonComponents)
    [azimuth_deg, elevation_deg] = boundary( ...
        polygonComponents(componentIndex));
    componentBoundary_deg = [ ...
        double(azimuth_deg(:)), double(elevation_deg(:))];
    % boundary may include an all-NaN separator at its end. Removing only
    % terminal separators avoids creating an empty ring in the packer.
    while ~isempty(componentBoundary_deg) && ...
            all(~isfinite(componentBoundary_deg(end, :)))
        componentBoundary_deg(end, :) = [];
    end
    bufferedRegions_deg{componentIndex} = componentBoundary_deg;
end
bufferedRegions_deg = bufferedRegions_deg(~cellfun( ...
    @(boundary_deg) size(boundary_deg, 1) < 3, bufferedRegions_deg));
end
