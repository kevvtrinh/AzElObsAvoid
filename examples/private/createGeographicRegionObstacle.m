function [obstacle, history, scenario] = createGeographicRegionObstacle( ...
        regionName, time_s, safetyMargin_units, options)
%% Section 0: Header & Readme
% SYNTAX
%   [obstacle, history, scenario] = createGeographicRegionObstacle( ...
%       regionName, time_s, safetyMargin_units)
%   [obstacle, history, scenario] = createGeographicRegionObstacle( ...
%       regionName, time_s, safetyMargin_units, options)
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
%   - safetyMargin_units (nonnegative numeric scalar)
%       Euclidean protection margin owned by obstacle construction.
%   - options (scalar struct, optional; default struct())
%       .Verbose prints source and geometry diagnostics (default false).
%**************************************************************************
% OUTPUTS
%   - obstacle (scalar obstacle struct)
%       Canonical protected static obstacle.
%   - history (scalar struct)
%       Window, unprotected boundary, and vertex diagnostics.
%   - scenario (scalar struct)
%       Automatically derived initial and goal positions.
%       Invalid input throws an error.
%**************************************************************************
% UNITS
%   - Longitude/latitude are treated as x/y coordinate units; time is
%     seconds and the safety margin is coordinate units.
%**************************************************************************

%% Section 1: Validate Inputs And Apply Defaults

% Accept only the maintained region names. Each name selects a map window and
% setup values. These values describe source data and do not select a route.

if nargin < 4 || isempty(options)
    options = struct();
end
if ~isstruct(options) || ~isscalar(options)
    error("createGeographicRegionObstacle:InvalidOptions", "options must be a scalar struct.");
end
defaultOptions         = struct();
defaultOptions.Verbose = false;
[resolvedOptions, unknownOptionFields] = obstacleAvoidance.input.resolveOptions(defaultOptions, options);
if ~isempty(unknownOptionFields)
    warning("createGeographicRegionObstacle:UnknownOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", ...
        strjoin(unknownOptionFields, ", "));
end
verbose = obstacleAvoidance.input.normalizeLogicalScalar( ...
    resolvedOptions.Verbose, "Verbose", ...
    "createGeographicRegionObstacle:InvalidVerbose");
resolvedOptions.Verbose = verbose;
validateattributes(time_s, {'numeric'}, {'real', 'finite', 'nonempty', 'increasing'});
time_s = double(time_s(:));
validateattributes(safetyMargin_units, ...
    {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
regionName = lower(strtrim(string(regionName)));
if ~isscalar(regionName)
    error("createGeographicRegionObstacle:InvalidRegion", "regionName must be scalar text.");
end
if regionName == "philippine"
    regionName = "philippines";
end
supportedRegions = ["hawaii" "croatia" "philippines"];
if ~any(regionName == supportedRegions)
    error("createGeographicRegionObstacle:UnsupportedRegion", ...
        "regionName must be Hawaii, Croatia, or Philippines.");
end

%% Section 2: Load And Clip Full-Resolution Geographic Boundaries

% Read land polygons whose bounds overlap the region window. Clip their union to
% the window. Then increase edge sample density without changing occupied area.

if regionName == "hawaii"
    sourceFile = which("usastatehi.shp");
    if isempty(sourceFile)
        error("createGeographicRegionObstacle:MappingToolboxRequired", ...
            "Mapping Toolbox file usastatehi.shp was not found.");
    end
    boundaries       = shaperead(sourceFile, "UseGeoCoords", true);
    stateNames       = string({boundaries.Name});
    selectedBoundary = boundaries(stateNames == "Hawaii");
    if numel(selectedBoundary) ~= 1
        error("createGeographicRegionObstacle:HawaiiNotFound", ...
            "Expected one Hawaii boundary in usastatehi.shp; found %d.", ...
            numel(selectedBoundary));
    end
    regionShape = polyshape(selectedBoundary.Lon, selectedBoundary.Lat, ...
        "Simplify", false, "KeepCollinearPoints", true);
    regionWindow_units = [-161.2 -154.5 18.5 22.8];
else
    sourceFile = which("landareas.shp");
    if isempty(sourceFile)
        error("createGeographicRegionObstacle:MappingToolboxRequired", ...
            "Mapping Toolbox file landareas.shp was not found.");
    end
    if regionName == "croatia"
        regionWindow_units = [13.2 19.6 42.2 46.9];
    else
        regionWindow_units = [116.7 126.8 4.5 20.7];
    end
    clippingShape       = rectanglePolyshape(regionWindow_units);
    landBoundaries      = shaperead(sourceFile, "UseGeoCoords", true);
    regionShape         = polyshape();
    selectedRecordCount = 0;

    % Join each land polygon whose bounding box overlaps the requested window.
    for boundaryIndex = 1:numel(landBoundaries)
        boundaryBounds_units = landBoundaries(boundaryIndex).BoundingBox;
        overlapsWindow       = boundaryBounds_units(2, 1) >= regionWindow_units(1) && ...
            boundaryBounds_units(1, 1) <= regionWindow_units(2) && ...
            boundaryBounds_units(2, 2) >= regionWindow_units(3) && ...
            boundaryBounds_units(1, 2) <= regionWindow_units(4);
        if ~overlapsWindow
            continue
        end
        landShape = polyshape( ...
            landBoundaries(boundaryIndex).Lon, landBoundaries(boundaryIndex).Lat, ...
            "Simplify", false, "KeepCollinearPoints", true);
        clippedShape = intersect(landShape, clippingShape);
        if isempty(clippedShape.Vertices) || area(clippedShape) <= 0
            continue
        end
        regionShape         = union(regionShape, clippedShape);
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
[longitude_units, latitude_units] = boundary(regionShape);
boundaryRowsAreFinite = isfinite(longitude_units) & isfinite(latitude_units);
if nnz(boundaryRowsAreFinite) < 3
    error("createGeographicRegionObstacle:EmptyRegion", ...
        "The %s source boundary has fewer than three finite vertices.", regionName);
end
nativeVertexCount = nnz(boundaryRowsAreFinite);
% Use the planner collision-check spacing. A long source edge must not make this
% dense-boundary example easier than trajectory validation.
maximumBoundarySpacing_units = 0.02;
[longitude_units, latitude_units] = densifyBoundaryRings( ...
    longitude_units, latitude_units, maximumBoundarySpacing_units);
boundaryRowsAreFinite = isfinite(longitude_units) & isfinite(latitude_units);

%% Section 3: Derive A Directly Blocked Request

% Test candidate vertical lines through the protected polygon. Select the line
% with the most interior samples. Put endpoints outside the polygon on that line.
% This method guarantees a blocked direct request without selecting a detour.

finiteLongitude_units     = longitude_units(boundaryRowsAreFinite);
finiteLatitude_units      = latitude_units(boundaryRowsAreFinite);
minimumLongitude_units    = min(finiteLongitude_units);
maximumLongitude_units    = max(finiteLongitude_units);
minimumLatitude_units     = min(finiteLatitude_units);
maximumLatitude_units     = max(finiteLatitude_units);
longitudeCandidates_units = linspace(minimumLongitude_units, maximumLongitude_units, 161).';
latitudeProbe_units       = linspace(minimumLatitude_units, maximumLatitude_units, 321);
insideCounts              = zeros(size(longitudeCandidates_units));

% Test each candidate longitude. Keep the line with the most interior samples.
for longitudeIndex = 1:numel(longitudeCandidates_units)
    probeLongitude_units = repmat( ...
        longitudeCandidates_units(longitudeIndex), size(latitudeProbe_units));
    insideCounts(longitudeIndex) = nnz( ...
        isinterior(regionShape, probeLongitude_units, latitudeProbe_units));
end
[maximumInsideCount, selectedLongitudeIndex] = max(insideCounts);
if maximumInsideCount == 0
    error("createGeographicRegionObstacle:NoBlockedMeridian", ...
        "Could not derive a blocked direct request through %s.", regionName);
end
routeLongitude_units    = longitudeCandidates_units(selectedLongitudeIndex);
latitudeSpan_units      = maximumLatitude_units - minimumLatitude_units;
endpointClearance_units = max(1, 0.15 * latitudeSpan_units);
initialPosition_units   = [routeLongitude_units, minimumLatitude_units - endpointClearance_units];
goalPosition_units      = [routeLongitude_units, maximumLatitude_units + endpointClearance_units];

%% Section 4: Construct The Canonical Protected Obstacle

% Pass the unprotected boundary and margin to the public obstacle constructor.
% Keep clipping bounds and vertex counts in the history output.

displayName         = upper(extractBefore(regionName, 2)) + extractAfter(regionName, 1);
constructionOptions = struct("Verbose", verbose);
obstacle = obstacleAvoidance.obstacles.createObstacle( ...
    displayName + " geographic region", time_s, longitude_units, ...
    latitude_units, safetyMargin_units, constructionOptions);
history = struct( ...
    "time_s",                       time_s, ...
    "sourceWindow_units",           regionWindow_units, ...
    "sourceLongitude_units",        longitude_units, ...
    "sourceLatitude_units",         latitude_units, ...
    "nativeSourceVertexCount",      nativeVertexCount, ...
    "sourceVertexCount",            nnz(boundaryRowsAreFinite), ...
    "sourceArea_units2",            area(regionShape), ...
    "maximumBoundarySpacing_units", maximumBoundarySpacing_units, ...
    "Options",                      resolvedOptions);
scenario = struct( ...
    "initialPosition_units",          initialPosition_units, ...
    "goalPosition_units",             goalPosition_units, ...
    "DirectRouteLongitude_units",     routeLongitude_units, ...
    "EndpointClearance_units",        endpointClearance_units);
if verbose
    fprintf("[region obstacle] %s: %d vertices, area %.3f units^2.\n", ...
        displayName, history.sourceVertexCount, history.sourceArea_units2);
end
end

%% Section 5: Local Functions

function shape = rectanglePolyshape(bounds_units)
    % Create the clipping rectangle for one geographic region.
    shape = polyshape(bounds_units([1 2 2 1]), bounds_units([3 3 4 4]), ...
        "Simplify", false, "KeepCollinearPoints", true);
end

function [denseX_units, denseY_units] = densifyBoundaryRings(x_units, y_units, maximumSpacing_units)
    % Add collinear edge samples without changing polygon occupancy. The
    % extra samples stress dense-boundary storage and validation.
    x_units            = double(x_units(:));
    y_units            = double(y_units(:));
    finiteRows         = isfinite(x_units) & isfinite(y_units);
    ringTransitions    = diff([false; finiteRows; false]);
    ringStartIndices   = find(ringTransitions == 1);
    ringStopIndices    = find(ringTransitions == -1) - 1;
    denseXByRing_units = cell(numel(ringStartIndices), 1);
    denseYByRing_units = cell(numel(ringStartIndices), 1);

    % Add samples to each finite boundary ring. Keep ring separators unchanged.
    for ringIndex = 1:numel(ringStartIndices)
        ringRows    = ringStartIndices(ringIndex):ringStopIndices(ringIndex);
        ringX_units = x_units(ringRows);
        ringY_units = y_units(ringRows);
        ringIsExplicitlyClosed = numel(ringX_units) > 3 && ...
            hypot(ringX_units(end) - ringX_units(1), ...
            ringY_units(end) - ringY_units(1)) <= 1e-12;
        if ringIsExplicitlyClosed
            ringX_units(end) = [];
            ringY_units(end) = [];
        end
        nextX_units       = circshift(ringX_units, -1);
        nextY_units       = circshift(ringY_units, -1);
        edgeLength_units  = hypot(nextX_units - ringX_units, nextY_units - ringY_units);
        subdivisionCounts = max(1, ceil(edgeLength_units ./ maximumSpacing_units));
        denseVertexCount  = sum(subdivisionCounts);
        denseRingX_units  = zeros(denseVertexCount, 1);
        denseRingY_units  = zeros(denseVertexCount, 1);
        nextWriteIndex    = 1;

        % Subdivide each closed-ring edge based on its coordinate length.
        for edgeIndex = 1:numel(ringX_units)
            edgeFraction = (0:subdivisionCounts(edgeIndex) - 1).' ./ subdivisionCounts(edgeIndex);
            writeCount   = numel(edgeFraction);
            writeRows    = nextWriteIndex:nextWriteIndex + writeCount - 1;
            denseRingX_units(writeRows) = ringX_units(edgeIndex) + ...
                edgeFraction .* (nextX_units(edgeIndex) - ringX_units(edgeIndex));
            denseRingY_units(writeRows) = ringY_units(edgeIndex) + ...
                edgeFraction .* (nextY_units(edgeIndex) - ringY_units(edgeIndex));
            nextWriteIndex = nextWriteIndex + writeCount;
        end
        denseXByRing_units{ringIndex} = denseRingX_units;
        denseYByRing_units{ringIndex} = denseRingY_units;
    end
    separator                 = {NaN};
    denseXWithSeparator_units = [denseXByRing_units, separator(ones(numel(denseXByRing_units), 1))].';
    denseYWithSeparator_units = [denseYByRing_units, separator(ones(numel(denseYByRing_units), 1))].';
    denseX_units              = vertcat(denseXWithSeparator_units{:});
    denseY_units              = vertcat(denseYWithSeparator_units{:});
    denseX_units(end) = [];
    denseY_units(end) = [];
end
