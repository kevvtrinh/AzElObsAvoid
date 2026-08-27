function [obstacle, history, scenario] = createGeographicRegionObstacle( regionName, time_s, safetyMargin_deg, options)
%% Section 0: Header & Readme
% SYNTAX
%   [obstacle, history, scenario] = createGeographicRegionObstacle( ...
%       regionName, time_s, safetyMargin_deg)
%   [obstacle, history, scenario] = createGeographicRegionObstacle( ...
%       regionName, time_s, safetyMargin_deg, options)
%**************************************************************************
% PURPOSE
%   - Build a static, full-resolution geographic obstacle for the maintained
%     Hawaii, Croatia, and Philippines extreme-polygon sequence.
%   - Derive route endpoints from polygon occupancy so every region blocks
%     its direct request without embedding a preferred detour.
%**************************************************************************
% INPUTS
%   - regionName (scalar text)
%       Hawaii, Croatia, or Philippines.
%   - time_s (strictly increasing numeric vector)
%       Static obstacle validity times.
%   - safetyMargin_deg (nonnegative numeric scalar)
%       Euclidean protection margin owned by obstacle construction.
%   - options (scalar struct, optional; default struct())
%       .Verbose prints source and geometry diagnostics (default false).
%**************************************************************************
% OUTPUTS
%   - obstacle (canonical protected static obstacle)
%   - history (scalar struct)
%       Source files, window, unprotected boundary, and vertex diagnostics.
%   - scenario (scalar struct)
%       Region name plus automatically derived initial and goal positions.
%**************************************************************************
% UNITS
%   - Longitude/latitude are treated as azimuth/elevation degrees; time is
%     seconds and the safety margin is degrees.
%**************************************************************************

%% Section 1: Validate Inputs & Apply Defaults

% Accept only the maintained region names. Each name selects a map window and
% setup values. These values describe source data and do not select a route.

if nargin < 4 || isempty(options)
    options = struct();
end
if ~isstruct(options) || ~isscalar(options)
    error("createGeographicRegionObstacle:InvalidOptions", "options must be a scalar struct.");
end
defaultOptions = struct("Verbose", false);
[resolvedOptions, unknownOptionFields] = obstacleAvoidance.input.resolveOptions(defaultOptions, options);
if ~isempty(unknownOptionFields)
    warning("createGeographicRegionObstacle:UnknownOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", strjoin(unknownOptionFields, ", "));
end
verbose = obstacleAvoidance.input.normalizeLogicalScalar( ...
    resolvedOptions.Verbose, "Verbose", "createGeographicRegionObstacle:InvalidVerbose");
resolvedOptions.Verbose = verbose;
validateattributes(time_s, {'numeric'}, {'real','finite','nonempty','increasing'});
time_s = double(time_s(:));
validateattributes(safetyMargin_deg, {'numeric'}, {'real','finite','scalar','nonnegative'});
regionName = lower(strtrim(string(regionName)));
if ~isscalar(regionName)
    error("createGeographicRegionObstacle:InvalidRegion", "regionName must be scalar text.");
end
if regionName == "philippine"
    regionName = "philippines";
end
supportedRegions = ["hawaii" "croatia" "philippines"];
if ~any(regionName == supportedRegions)
    error("createGeographicRegionObstacle:UnsupportedRegion", "regionName must be Hawaii, Croatia, or Philippines.");
end

%% Section 2: Load & Clip Full-Resolution Geographic Boundaries

% Read land polygons whose bounds overlap the region window. Clip their union to
% the window. Then increase edge sample density without changing occupied area.

if regionName == "hawaii"
    sourceFile = which("usastatehi.shp");
    if isempty(sourceFile)
        error("createGeographicRegionObstacle:MappingToolboxRequired", ...
            "Mapping Toolbox file usastatehi.shp was not found.");
    end
    boundaries = shaperead(sourceFile, "UseGeoCoords", true);
    stateName = string({boundaries.Name});
    selectedBoundary = boundaries(stateName == "Hawaii");
    if numel(selectedBoundary) ~= 1
        error("createGeographicRegionObstacle:HawaiiNotFound", ...
            "Expected one Hawaii boundary in usastatehi.shp; found %d.", numel(selectedBoundary));
    end
    regionShape = polyshape( ...
        selectedBoundary.Lon, selectedBoundary.Lat, "Simplify", false, "KeepCollinearPoints", true);
    regionWindow_deg = [-161.2 -154.5 18.5 22.8];
else
    sourceFile = which("landareas.shp");
    if isempty(sourceFile)
        error("createGeographicRegionObstacle:MappingToolboxRequired", ...
            "Mapping Toolbox file landareas.shp was not found.");
    end
    if regionName == "croatia"
        regionWindow_deg = [13.2 19.6 42.2 46.9];
    else
        regionWindow_deg = [116.7 126.8 4.5 20.7];
    end
    clippingShape = rectanglePolyshape(regionWindow_deg);
    landBoundaries = shaperead(sourceFile, "UseGeoCoords", true);
    regionShape = polyshape();
    selectedRecordCount = 0;

    % Join each land polygon whose bounding box overlaps the requested window.
    for boundaryIndex = 1:numel(landBoundaries)
        boundaryBounds_deg = landBoundaries(boundaryIndex).BoundingBox;
        overlapsWindow = boundaryBounds_deg(2, 1) >= ...
            regionWindow_deg(1) && boundaryBounds_deg(1, 1) <= ...
            regionWindow_deg(2) && boundaryBounds_deg(2, 2) >= ...
            regionWindow_deg(3) && boundaryBounds_deg(1, 2) <= regionWindow_deg(4);
        if ~overlapsWindow
            continue;
        end
        landShape = polyshape( ...
            landBoundaries(boundaryIndex).Lon, ...
            landBoundaries(boundaryIndex).Lat, "Simplify", false, "KeepCollinearPoints", true);
        clippedShape = intersect(landShape, clippingShape);
        if isempty(clippedShape.Vertices) || area(clippedShape) <= 0
            continue;
        end
        regionShape = union(regionShape, clippedShape);
        selectedRecordCount = selectedRecordCount + 1;
    end
    if selectedRecordCount == 0
        error("createGeographicRegionObstacle:EmptyRegion", ...
            "No land boundary intersected the %s region window.", regionName);
    end
end
if isempty(regionShape.Vertices) || area(regionShape) <= 0
    error("createGeographicRegionObstacle:EmptyRegion", ...
        "The %s source boundary did not produce occupied area.", regionName);
end
[longitude_deg, latitude_deg] = boundary(regionShape);
finiteBoundary = isfinite(longitude_deg) & isfinite(latitude_deg);
if nnz(finiteBoundary) < 3
    error("createGeographicRegionObstacle:EmptyRegion", ...
        "The %s source boundary has fewer than three finite vertices.", regionName);
end
nativeVertexCount = nnz(finiteBoundary);
% Use the planner collision-check spacing. A long source edge must not make this
% dense-boundary example easier than trajectory validation.
maximumBoundarySpacing_deg = 0.02;
[longitude_deg, latitude_deg] = densifyBoundaryRings( longitude_deg, latitude_deg, maximumBoundarySpacing_deg);
finiteBoundary = isfinite(longitude_deg) & isfinite(latitude_deg);

%% Section 3: Derive A Directly Blocked Request

% Test candidate horizontal lines through the protected polygon. Select the line
% with the most interior samples. Put endpoints outside the polygon on that line.
% This method guarantees a blocked direct request without selecting a detour.

finiteLongitude_deg = longitude_deg(finiteBoundary);
finiteLatitude_deg = latitude_deg(finiteBoundary);
minimumLongitude_deg = min(finiteLongitude_deg);
maximumLongitude_deg = max(finiteLongitude_deg);
minimumLatitude_deg = min(finiteLatitude_deg);
maximumLatitude_deg = max(finiteLatitude_deg);
longitudeCandidates_deg = linspace( minimumLongitude_deg, maximumLongitude_deg, 161).';
latitudeProbe_deg = linspace( minimumLatitude_deg, maximumLatitude_deg, 321);
insideCount = zeros(size(longitudeCandidates_deg));

% Test each candidate longitude. Keep the line with the most interior samples.
for longitudeIndex = 1:numel(longitudeCandidates_deg)
    probeLongitude_deg = repmat( longitudeCandidates_deg(longitudeIndex), size(latitudeProbe_deg));
    insideCount(longitudeIndex) = nnz(isinterior( regionShape, probeLongitude_deg, latitudeProbe_deg));
end
[maximumInsideCount, selectedLongitudeIndex] = max(insideCount);
if maximumInsideCount == 0
    error("createGeographicRegionObstacle:NoBlockedMeridian", ...
        "Could not derive a blocked direct request through %s.", regionName);
end
routeLongitude_deg = longitudeCandidates_deg(selectedLongitudeIndex);
latitudeSpan_deg = maximumLatitude_deg - minimumLatitude_deg;
endpointClearance_deg = max(1, 0.15 * latitudeSpan_deg);
initialPosition_deg = [ routeLongitude_deg, minimumLatitude_deg - endpointClearance_deg];
goalPosition_deg = [ routeLongitude_deg, maximumLatitude_deg + endpointClearance_deg];

%% Section 4: Construct The Canonical Protected Obstacle

% Pass the unprotected boundary and margin to the public obstacle constructor.
% Keep source files, clipping bounds, and vertex counts in the history output.

displayName = upper(extractBefore(regionName, 2)) + extractAfter(regionName, 1);
constructionOptions = struct("Verbose", verbose);
obstacle = obstacleAvoidance.obstacles.createObstacle( ...
    displayName + " geographic region", time_s, longitude_deg, latitude_deg, safetyMargin_deg, constructionOptions);
history = struct( ...
    "RegionName", displayName, ...
    "time_s", time_s, ...
    "sourceFile", string(sourceFile), ...
    "sourceWindow_deg", regionWindow_deg, ...
    "sourceLongitude_deg", longitude_deg, ...
    "sourceLatitude_deg", latitude_deg, ...
    "nativeSourceVertexCount", nativeVertexCount, ...
    "sourceVertexCount", nnz(finiteBoundary), ...
    "sourceArea_deg2", area(regionShape), ...
    "maximumBoundarySpacing_deg", maximumBoundarySpacing_deg, "Options", resolvedOptions);
scenario = struct( ...
    "RegionName", displayName, ...
    "initialPosition_deg", initialPosition_deg, ...
    "goalPosition_deg", goalPosition_deg, ...
    "DirectRouteLongitude_deg", routeLongitude_deg, "EndpointClearance_deg", endpointClearance_deg);
if verbose
    fprintf("[region obstacle] %s: %d vertices, area %.3f deg^2.\n", ...
        displayName, history.sourceVertexCount, history.sourceArea_deg2);
end
end


function shape = rectanglePolyshape(bounds_deg)
% Create the clipping rectangle for one geographic region.
shape = polyshape( bounds_deg([1 2 2 1]), bounds_deg([3 3 4 4]), "Simplify", false, "KeepCollinearPoints", true);
end

function [denseX_deg, denseY_deg] = densifyBoundaryRings( x_deg, y_deg, maximumSpacing_deg)
% Add collinear edge samples. Do not change polygon occupancy. The extra samples
% stress dense-boundary storage and validation.
x_deg = double(x_deg(:));
y_deg = double(y_deg(:));
finiteRows = isfinite(x_deg) & isfinite(y_deg);
ringTransition = diff([false; finiteRows; false]);
ringStart = find(ringTransition == 1);
ringStop = find(ringTransition == -1) - 1;
denseXByRing_deg = cell(numel(ringStart), 1);
denseYByRing_deg = cell(numel(ringStart), 1);

% Add samples to each finite boundary ring. Keep ring separators unchanged.
for ringIndex = 1:numel(ringStart)
    ringRows = ringStart(ringIndex):ringStop(ringIndex);
    ringX_deg = x_deg(ringRows);
    ringY_deg = y_deg(ringRows);
    if numel(ringX_deg) > 3 && hypot(ringX_deg(end) - ringX_deg(1), ringY_deg(end) - ringY_deg(1)) <= 1e-12
        ringX_deg(end) = [];
        ringY_deg(end) = [];
    end
    nextX_deg = circshift(ringX_deg, -1);
    nextY_deg = circshift(ringY_deg, -1);
    edgeLength_deg = hypot( nextX_deg - ringX_deg, nextY_deg - ringY_deg);
    subdivisionCount = max(1, ceil( edgeLength_deg ./ maximumSpacing_deg));
    denseVertexCount = sum(subdivisionCount);
    denseRingX_deg = zeros(denseVertexCount, 1);
    denseRingY_deg = zeros(denseVertexCount, 1);
    nextWriteIndex = 1;

    % Subdivide each closed-ring edge based on its angular length.
    for edgeIndex = 1:numel(ringX_deg)
        edgeFraction = (0:subdivisionCount(edgeIndex) - 1).' ./ subdivisionCount(edgeIndex);
        writeCount = numel(edgeFraction);
        writeRows = nextWriteIndex:nextWriteIndex + writeCount - 1;
        denseRingX_deg(writeRows) = ringX_deg(edgeIndex) + ...
            edgeFraction .* (nextX_deg(edgeIndex) - ringX_deg(edgeIndex));
        denseRingY_deg(writeRows) = ringY_deg(edgeIndex) + ...
            edgeFraction .* (nextY_deg(edgeIndex) - ringY_deg(edgeIndex));
        nextWriteIndex = nextWriteIndex + writeCount;
    end
    denseXByRing_deg{ringIndex} = denseRingX_deg;
    denseYByRing_deg{ringIndex} = denseRingY_deg;
end
separator = {NaN};
denseXWithSeparator_deg = [denseXByRing_deg, separator(ones( numel(denseXByRing_deg), 1))].';
denseYWithSeparator_deg = [denseYByRing_deg, separator(ones( numel(denseYByRing_deg), 1))].';
denseX_deg = vertcat(denseXWithSeparator_deg{:});
denseY_deg = vertcat(denseYWithSeparator_deg{:});
denseX_deg(end) = [];
denseY_deg(end) = [];
end
