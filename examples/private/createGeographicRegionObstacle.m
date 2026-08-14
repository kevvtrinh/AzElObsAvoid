function [obstacle, history, scenario] = ...
        createGeographicRegionObstacle( ...
        regionName, time_s, safetyMargin_deg, options)
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
if nargin < 4 || isempty(options)
    options = struct();
end
if ~isstruct(options) || ~isscalar(options)
    error("createGeographicRegionObstacle:InvalidOptions", ...
        "options must be a scalar struct.");
end
defaultOptions = struct("Verbose", false);
unknownOptionFields = setdiff( ...
    fieldnames(options), fieldnames(defaultOptions), "stable");
if ~isempty(unknownOptionFields)
    warning("createGeographicRegionObstacle:UnknownOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", ...
        strjoin(string(unknownOptionFields), ", "));
    options = rmfield(options, unknownOptionFields);
end
resolvedOptions = defaultOptions;
optionNames = fieldnames(options);
for optionIndex = 1:numel(optionNames)
    optionName = optionNames{optionIndex};
    if ~isempty(options.(optionName))
        resolvedOptions.(optionName) = options.(optionName);
    end
end
validateattributes(resolvedOptions.Verbose, ...
    {'logical','numeric'}, {'scalar'});
verbose = logical(resolvedOptions.Verbose);
resolvedOptions.Verbose = verbose;
validateattributes(time_s, {'numeric'}, ...
    {'real','finite','nonempty','increasing'});
time_s = double(time_s(:));
validateattributes(safetyMargin_deg, {'numeric'}, ...
    {'real','finite','scalar','nonnegative'});
regionName = lower(strtrim(string(regionName)));
if ~isscalar(regionName)
    error("createGeographicRegionObstacle:InvalidRegion", ...
        "regionName must be scalar text.");
end
if regionName == "philippine"
    regionName = "philippines";
end
supportedRegions = ["hawaii" "croatia" "philippines"];
if ~any(regionName == supportedRegions)
    error("createGeographicRegionObstacle:UnsupportedRegion", ...
        "regionName must be Hawaii, Croatia, or Philippines.");
end

%% Section 2: Load & Clip Full-Resolution Geographic Boundaries
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
            "Expected one Hawaii boundary in usastatehi.shp; found %d.", ...
            numel(selectedBoundary));
    end
    regionShape = polyshape( ...
        selectedBoundary.Lon, selectedBoundary.Lat, ...
        "Simplify", false, "KeepCollinearPoints", true);
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
    for boundaryIndex = 1:numel(landBoundaries)
        boundaryBounds_deg = landBoundaries(boundaryIndex).BoundingBox;
        overlapsWindow = boundaryBounds_deg(2, 1) >= ...
            regionWindow_deg(1) && boundaryBounds_deg(1, 1) <= ...
            regionWindow_deg(2) && boundaryBounds_deg(2, 2) >= ...
            regionWindow_deg(3) && boundaryBounds_deg(1, 2) <= ...
            regionWindow_deg(4);
        if ~overlapsWindow
            continue;
        end
        landShape = polyshape( ...
            landBoundaries(boundaryIndex).Lon, ...
            landBoundaries(boundaryIndex).Lat, ...
            "Simplify", false, "KeepCollinearPoints", true);
        clippedShape = intersect(landShape, clippingShape);
        if isempty(clippedShape.Vertices) || area(clippedShape) <= 0
            continue;
        end
        regionShape = union(regionShape, clippedShape);
        selectedRecordCount = selectedRecordCount + 1;
    end
    if selectedRecordCount == 0
        error("createGeographicRegionObstacle:EmptyRegion", ...
            "No land boundary intersected the %s region window.", ...
            regionName);
    end
end
if isempty(regionShape.Vertices) || area(regionShape) <= 0
    error("createGeographicRegionObstacle:EmptyRegion", ...
        "The %s source boundary did not produce occupied area.", ...
        regionName);
end
[longitude_deg, latitude_deg] = boundary(regionShape);
finiteBoundary = isfinite(longitude_deg) & isfinite(latitude_deg);
if nnz(finiteBoundary) < 3
    error("createGeographicRegionObstacle:EmptyRegion", ...
        "The %s source boundary has fewer than three finite vertices.", ...
        regionName);
end
nativeVertexCount = nnz(finiteBoundary);
% Match the planner's default collision-check spacing so a single long
% source edge cannot make this density regression easier than the returned
% trajectory validation that it is intended to exercise.
maximumBoundarySpacing_deg = 0.02;
[longitude_deg, latitude_deg] = densifyBoundaryRings( ...
    longitude_deg, latitude_deg, maximumBoundarySpacing_deg);
finiteBoundary = isfinite(longitude_deg) & isfinite(latitude_deg);

%% Section 3: Derive A Directly Blocked Request
finiteLongitude_deg = longitude_deg(finiteBoundary);
finiteLatitude_deg = latitude_deg(finiteBoundary);
minimumLongitude_deg = min(finiteLongitude_deg);
maximumLongitude_deg = max(finiteLongitude_deg);
minimumLatitude_deg = min(finiteLatitude_deg);
maximumLatitude_deg = max(finiteLatitude_deg);
longitudeCandidates_deg = linspace( ...
    minimumLongitude_deg, maximumLongitude_deg, 161).';
latitudeProbe_deg = linspace( ...
    minimumLatitude_deg, maximumLatitude_deg, 321);
insideCount = zeros(size(longitudeCandidates_deg));
for longitudeIndex = 1:numel(longitudeCandidates_deg)
    probeLongitude_deg = repmat( ...
        longitudeCandidates_deg(longitudeIndex), ...
        size(latitudeProbe_deg));
    insideCount(longitudeIndex) = nnz(isinterior( ...
        regionShape, probeLongitude_deg, latitudeProbe_deg));
end
[maximumInsideCount, selectedLongitudeIndex] = max(insideCount);
if maximumInsideCount == 0
    error("createGeographicRegionObstacle:NoBlockedMeridian", ...
        "Could not derive a blocked direct request through %s.", ...
        regionName);
end
routeLongitude_deg = ...
    longitudeCandidates_deg(selectedLongitudeIndex);
latitudeSpan_deg = maximumLatitude_deg - minimumLatitude_deg;
endpointClearance_deg = max(1, 0.15 * latitudeSpan_deg);
initialPosition_deg = [ ...
    routeLongitude_deg, minimumLatitude_deg - endpointClearance_deg];
goalPosition_deg = [ ...
    routeLongitude_deg, maximumLatitude_deg + endpointClearance_deg];

%% Section 4: Construct The Canonical Protected Obstacle
displayName = upper(extractBefore(regionName, 2)) + ...
    extractAfter(regionName, 1);
constructionOptions = struct("Verbose", verbose);
obstacle = makeAzElObstacleData( ...
    displayName + " geographic region", time_s, ...
    longitude_deg, latitude_deg, safetyMargin_deg, ...
    constructionOptions);
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
    "maximumBoundarySpacing_deg", maximumBoundarySpacing_deg, ...
    "Options", resolvedOptions);
scenario = struct( ...
    "RegionName", displayName, ...
    "initialPosition_deg", initialPosition_deg, ...
    "goalPosition_deg", goalPosition_deg, ...
    "DirectRouteLongitude_deg", routeLongitude_deg, ...
    "EndpointClearance_deg", endpointClearance_deg);
if verbose
    fprintf("[region obstacle] %s: %d vertices, area %.3f deg^2.\n", ...
        displayName, history.sourceVertexCount, history.sourceArea_deg2);
end
end

%% Section 5: Local Functions
function shape = rectanglePolyshape(bounds_deg)
%% Section 0: Header & Readme
% SYNTAX
%   shape = rectanglePolyshape(bounds_deg)
%**************************************************************************
% PURPOSE
%   - Construct the geographic clipping rectangle for one region window.
%**************************************************************************
% INPUTS
%   - bounds_deg (1-by-4 [azimuthMin azimuthMax elevationMin elevationMax])
%**************************************************************************
% OUTPUTS
%   - shape (scalar polyshape)
%**************************************************************************
% UNITS
%   - Bounds are degrees.
%**************************************************************************
shape = polyshape( ...
    bounds_deg([1 2 2 1]), bounds_deg([3 3 4 4]), ...
    "Simplify", false, "KeepCollinearPoints", true);
end

function [denseX_deg, denseY_deg] = densifyBoundaryRings( ...
        x_deg, y_deg, maximumSpacing_deg)
%% Section 0: Header & Readme
% SYNTAX
%   [denseX_deg, denseY_deg] = densifyBoundaryRings( ...
%       x_deg, y_deg, maximumSpacing_deg)
%**************************************************************************
% PURPOSE
%   - Add collinear edge samples without changing polygon occupancy so each
%     geographic case also stresses dense-boundary packing and validation.
%**************************************************************************
% INPUTS
%   - x_deg, y_deg (matching boundary vectors with paired separators)
%   - maximumSpacing_deg (positive scalar maximum adjacent spacing)
%**************************************************************************
% OUTPUTS
%   - denseX_deg, denseY_deg (matching NaN-separated boundary vectors)
%**************************************************************************
% UNITS
%   - Coordinates and maximum spacing are degrees.
%**************************************************************************
x_deg = double(x_deg(:));
y_deg = double(y_deg(:));
finiteRows = isfinite(x_deg) & isfinite(y_deg);
ringTransition = diff([false; finiteRows; false]);
ringStart = find(ringTransition == 1);
ringStop = find(ringTransition == -1) - 1;
denseXByRing_deg = cell(numel(ringStart), 1);
denseYByRing_deg = cell(numel(ringStart), 1);
for ringIndex = 1:numel(ringStart)
    ringRows = ringStart(ringIndex):ringStop(ringIndex);
    ringX_deg = x_deg(ringRows);
    ringY_deg = y_deg(ringRows);
    if numel(ringX_deg) > 3 && ...
            hypot(ringX_deg(end) - ringX_deg(1), ...
            ringY_deg(end) - ringY_deg(1)) <= 1e-12
        ringX_deg(end) = [];
        ringY_deg(end) = [];
    end
    nextX_deg = circshift(ringX_deg, -1);
    nextY_deg = circshift(ringY_deg, -1);
    edgeLength_deg = hypot( ...
        nextX_deg - ringX_deg, nextY_deg - ringY_deg);
    subdivisionCount = max(1, ceil( ...
        edgeLength_deg ./ maximumSpacing_deg));
    denseVertexCount = sum(subdivisionCount);
    denseRingX_deg = zeros(denseVertexCount, 1);
    denseRingY_deg = zeros(denseVertexCount, 1);
    nextWriteIndex = 1;
    for edgeIndex = 1:numel(ringX_deg)
        edgeFraction = (0:subdivisionCount(edgeIndex) - 1).' ./ ...
            subdivisionCount(edgeIndex);
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
denseXWithSeparator_deg = [denseXByRing_deg, separator(ones( ...
    numel(denseXByRing_deg), 1))].';
denseYWithSeparator_deg = [denseYByRing_deg, separator(ones( ...
    numel(denseYByRing_deg), 1))].';
denseX_deg = vertcat(denseXWithSeparator_deg{:});
denseY_deg = vertcat(denseYWithSeparator_deg{:});
denseX_deg(end) = [];
denseY_deg(end) = [];
end
