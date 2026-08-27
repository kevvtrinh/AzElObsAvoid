function azElData = makeAzElObstacleData(obstacleInput, varargin)
%% Section 0: Header & Readme
% SYNTAX
%   azElData = azElObstacles.makeAzElObstacleData( ...
%       obstacleName, time_s, azimuthBoundary_deg, elevationBoundary_deg)
%   azElData = azElObstacles.makeAzElObstacleData( ...
%       obstacleName, time_s, azimuthBoundary_deg, ...
%       elevationBoundary_deg, safetyMargin_deg)
%   azElData = azElObstacles.makeAzElObstacleData( ...
%       obstacleName, time_s, azimuthBoundary_deg, ...
%       elevationBoundary_deg, safetyMargin_deg, constructionOptions)
%   azElData = azElObstacles.makeAzElObstacleData(canonicalObstacle)
%   azElData = azElObstacles.makeAzElObstacleData( ...
%       canonicalObstacles, safetyMargin_deg)
%   azElData = azElObstacles.makeAzElObstacleData( ...
%       canonicalObstacles, safetyMargin_deg, constructionOptions)
%**************************************************************************
% PURPOSE
%   - Own canonical Az/El obstacle construction, normalization, and
%     absolute safety-margin reconstruction in one public function.
%   - Retain original and protected polygon histories without cumulative
%     inflation when canonical records are combined or rebuilt.
%**************************************************************************
% INPUTS
%   - obstacleInput (scalar text, scalar canonical struct, struct array,
%       nested cell array, or empty numeric input)
%       Text starts construction. A lone scalar struct is normalized. A
%       canonical container followed by a margin is rebuilt.
%   - varargin
%       Construction: increasing time_s, matching numeric or cell boundary
%       histories, optional nonnegative margin, and optional scalar options.
%       Rebuild: nonnegative absolute margin and optional scalar options.
%       constructionOptions.Verbose prints protection progress (false).
%**************************************************************************
% OUTPUTS
%   - azElData (canonical scalar or column struct array)
%       Validated obstacles with original geometry, protected geometry,
%       construction margin, sample status, and stable field order.
%**************************************************************************
% UNITS
%   - Boundary coordinates and safety margins are degrees; time is seconds.
%**************************************************************************

%% Section 1: Select The Supported Construction Operation

if nargin == 0
    error("makeAzElObstacleData:MissingInput", ...
        "Obstacle construction or canonical obstacle input is required.");
end
isCanonicalContainer = isstruct(obstacleInput) || ...
    iscell(obstacleInput) || ...
    (isnumeric(obstacleInput) && isempty(obstacleInput));
if isCanonicalContainer && nargin == 1
    azElData = normalizeCanonicalObstacle(obstacleInput);
    return;
elseif isCanonicalContainer && nargin >= 2 && nargin <= 3
    safetyMargin_deg = varargin{1};
    if nargin < 3 || isempty(varargin{2})
        constructionOptions = struct();
    else
        constructionOptions = varargin{2};
    end
    azElData = rebuildProtectedObstacles( ...
        obstacleInput, safetyMargin_deg, constructionOptions);
    return;
end
if nargin < 4 || nargin > 6
    error("makeAzElObstacleData:InvalidCall", ...
        "Construction requires name, time, azimuth, and elevation; " + ...
        "canonical rebuild requires obstacle data and an absolute margin.");
end

%% Section 2: Construct One Raw Canonical Record

obstacleName = obstacleInput;
time_s = double(varargin{1}(:));
azimuthBoundary_deg = varargin{2};
elevationBoundary_deg = varargin{3};
if nargin < 5 || isempty(varargin{4})
    safetyMargin_deg = 0;
else
    safetyMargin_deg = varargin{4};
end
if nargin < 6 || isempty(varargin{5})
    constructionOptions = struct();
else
    constructionOptions = varargin{5};
end
validateattributes(safetyMargin_deg, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'nonnegative'});
sampleCount = numel(time_s);
if ~iscell(azimuthBoundary_deg)
    azimuthBoundary_deg = repmat( ...
        {double(azimuthBoundary_deg(:))}, sampleCount, 1);
end
if ~iscell(elevationBoundary_deg)
    elevationBoundary_deg = repmat( ...
        {double(elevationBoundary_deg(:))}, sampleCount, 1);
end
rawObstacle = struct( ...
    "targetName", string(obstacleName), ...
    "time_s", time_s, ...
    "az_deg", {reshape(azimuthBoundary_deg, [], 1)}, ...
    "el_deg", {reshape(elevationBoundary_deg, [], 1)}, ...
    "originalAz_deg", {reshape(azimuthBoundary_deg, [], 1)}, ...
    "originalEl_deg", {reshape(elevationBoundary_deg, [], 1)}, ...
    "safetyMargin_deg", 0, ...
    "status", repmat("visible", sampleCount, 1));

%% Section 3: Normalize And Protect The Constructed Record

% Construction and imported canonical data share the same schema gate.
% Absolute reconstruction always starts from original geometry so repeated
% calls cannot accumulate a safety margin.
azElData = normalizeCanonicalObstacle(rawObstacle);
[verbose, ~] = resolveProtectionOptions(constructionOptions);
azElData = protectCanonicalObstacles( ...
    azElData, safetyMargin_deg, verbose);
end

function azElData = normalizeCanonicalObstacle(inputData)
%% Section 0: Header & Readme
% Validate and column-normalize one canonical obstacle record. Established
% normalization identifiers remain stable for malformed imported records.

%% Section 1: Validate Structure And Sample Time

requiredFields = ["targetName", "time_s", "az_deg", "el_deg", "status"];
hasRequiredStructure = isstruct(inputData) && isscalar(inputData);
if hasRequiredStructure
    hasRequiredStructure = all(isfield( ...
        inputData, cellstr(requiredFields)));
end
if ~hasRequiredStructure
    error("normalizeAzElTimeObstacleData:InvalidInput", ...
        "azElData must be a scalar canonical obstacle record with " + ...
        "targetName, time_s, az_deg, el_deg, and status.");
end
targetName = string(inputData.targetName);
if ~isscalar(targetName) || strlength(strtrim(targetName)) == 0
    error("normalizeAzElTimeObstacleData:InvalidTargetName", ...
        "targetName must be nonempty scalar text.");
end
validateattributes(inputData.time_s, ...
    {'numeric'}, {'vector', 'real', 'finite'});
time_s = double(inputData.time_s(:));
sampleCount = numel(time_s);
if sampleCount == 0 || any(diff(time_s) <= 0)
    error("normalizeAzElTimeObstacleData:InvalidTime", ...
        "time_s must be nonempty and strictly increasing.");
end

%% Section 2: Validate Protected Boundary Slices

hasCellBoundaries = iscell(inputData.az_deg) && ...
    iscell(inputData.el_deg);
hasMatchingSampleCounts = numel(inputData.az_deg) == sampleCount && ...
    numel(inputData.el_deg) == sampleCount;
if ~hasCellBoundaries || ~hasMatchingSampleCounts
    error("normalizeAzElTimeObstacleData:InvalidBoundary", ...
        "az_deg and el_deg must be cell arrays matching time_s.");
end
[azimuthSlices_deg, elevationSlices_deg, ...
    removedProtectedRegionCount, protectedRemovalBySample] = ...
    normalizeBoundaryHistory( ...
    inputData.az_deg, inputData.el_deg, sampleCount, "protected");

%% Section 3: Preserve Original Geometry And Margin

hasOriginalBoundaries = isfield(inputData, "originalAz_deg") && ...
    isfield(inputData, "originalEl_deg");
hasOnlyOneOriginalBoundary = xor( ...
    isfield(inputData, "originalAz_deg"), ...
    isfield(inputData, "originalEl_deg"));
if hasOnlyOneOriginalBoundary
    error("normalizeAzElTimeObstacleData:IncompleteOriginalBoundary", ...
        "originalAz_deg and originalEl_deg must either both be " + ...
        "present or both be absent.");
end
if hasOriginalBoundaries
    originalHistoryIsValid = iscell(inputData.originalAz_deg) && ...
        iscell(inputData.originalEl_deg) && ...
        numel(inputData.originalAz_deg) == sampleCount && ...
        numel(inputData.originalEl_deg) == sampleCount;
    if ~originalHistoryIsValid
        error("normalizeAzElTimeObstacleData:InvalidOriginalBoundary", ...
            "originalAz_deg and originalEl_deg must be cell arrays " + ...
            "matching time_s.");
    end
    [originalAzimuthSlices_deg, originalElevationSlices_deg, ...
        removedOriginalRegionCount, originalRemovalBySample] = ...
        normalizeBoundaryHistory( ...
        inputData.originalAz_deg, inputData.originalEl_deg, ...
        sampleCount, "original");
else
    originalAzimuthSlices_deg = azimuthSlices_deg;
    originalElevationSlices_deg = elevationSlices_deg;
    removedOriginalRegionCount = 0;
    originalRemovalBySample = false(sampleCount, 1);
end
removedRegionCount = removedProtectedRegionCount + ...
    removedOriginalRegionCount;
if removedRegionCount > 0
    removalBySample = protectedRemovalBySample | originalRemovalBySample;
    warning("normalizeAzElTimeObstacleData:RemovedTwoVertexRegions", ...
        "Obstacle '%s' removed %d protected and %d original " + ...
        "two-vertex regions across %d time slices. Two vertices cannot " + ...
        "enclose occupied area; all remaining regions were unchanged.", ...
        targetName, removedProtectedRegionCount, ...
        removedOriginalRegionCount, nnz(removalBySample));
end
if isfield(inputData, "safetyMargin_deg")
    safetyMargin_deg = inputData.safetyMargin_deg;
else
    safetyMargin_deg = 0;
end
validateattributes(safetyMargin_deg, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'nonnegative'});
safetyMargin_deg = double(safetyMargin_deg);
if safetyMargin_deg > 0 && ~hasOriginalBoundaries
    error("normalizeAzElTimeObstacleData:MissingOriginalBoundary", ...
        "A positive safetyMargin_deg requires originalAz_deg and " + ...
        "originalEl_deg. Construct protected obstacles with " + ...
        "makeAzElObstacleData.");
end

%% Section 4: Normalize Status And Assemble Output

statusBySample = string(inputData.status);
if isscalar(statusBySample)
    statusBySample = repmat(statusBySample, sampleCount, 1);
elseif numel(statusBySample) ~= sampleCount
    error("normalizeAzElTimeObstacleData:StatusSizeMismatch", ...
        "status must contain one value per time sample.");
else
    statusBySample = statusBySample(:);
end
azElData = struct( ...
    "targetName", targetName, ...
    "time_s", time_s, ...
    "az_deg", {azimuthSlices_deg}, ...
    "el_deg", {elevationSlices_deg}, ...
    "originalAz_deg", {originalAzimuthSlices_deg}, ...
    "originalEl_deg", {originalElevationSlices_deg}, ...
    "safetyMargin_deg", safetyMargin_deg, ...
    "status", statusBySample);
end

function [azimuthSlices_deg, elevationSlices_deg, removedRegionCount, ...
        removalBySample] = normalizeBoundaryHistory( ...
        azimuthInput_deg, elevationInput_deg, sampleCount, boundaryRole)
%% Section 0: Header & Readme
% Normalize one protected or original boundary history uniformly.

azimuthSlices_deg = reshape(azimuthInput_deg, [], 1);
elevationSlices_deg = reshape(elevationInput_deg, [], 1);
removedRegionCount = 0;
removalBySample = false(sampleCount, 1);
roleIndex = 1 + (boundaryRole == "original");
mismatchIdentifiers = [ ...
    "normalizeAzElTimeObstacleData:BoundarySizeMismatch", ...
    "normalizeAzElTimeObstacleData:OriginalBoundarySizeMismatch"];
allFieldNames = [ ...
    "az_deg", "el_deg"; ...
    "originalAz_deg", "originalEl_deg"];
mismatchIdentifier = mismatchIdentifiers(roleIndex);
fieldNames = allFieldNames(roleIndex, :);
for sampleIndex = 1:sampleCount
    validateattributes(azimuthSlices_deg{sampleIndex}, ...
        {'numeric'}, {'vector', 'real'});
    validateattributes(elevationSlices_deg{sampleIndex}, ...
        {'numeric'}, {'vector', 'real'});
    if numel(azimuthSlices_deg{sampleIndex}) ~= ...
            numel(elevationSlices_deg{sampleIndex})
        error(mismatchIdentifier, ...
            "%s and %s slice %d must have equal lengths.", ...
            fieldNames(1), fieldNames(2), sampleIndex);
    end
    azimuthSlices_deg{sampleIndex} = double( ...
        azimuthSlices_deg{sampleIndex}(:));
    elevationSlices_deg{sampleIndex} = double( ...
        elevationSlices_deg{sampleIndex}(:));
    [azimuthSlices_deg{sampleIndex}, ...
        elevationSlices_deg{sampleIndex}, removedAtSampleCount] = ...
        normalizeBoundarySliceTopology( ...
        azimuthSlices_deg{sampleIndex}, ...
        elevationSlices_deg{sampleIndex}, sampleIndex, boundaryRole);
    removedRegionCount = removedRegionCount + removedAtSampleCount;
    removalBySample(sampleIndex) = removedAtSampleCount > 0;
end
end

function [azimuth_deg, elevation_deg, removedRegionCount] = ...
        normalizeBoundarySliceTopology( ...
        azimuth_deg, elevation_deg, sampleIndex, boundaryRole)
%% Section 0: Header & Readme
% Reject mismatched separators and one-vertex regions, remove two-vertex
% regions, and preserve valid polygon regions in caller order.

%% Section 1: Classify Boundary Regions

boundaryRoleText = char(boundaryRole);
azimuthIsFinite = isfinite(azimuth_deg);
elevationIsFinite = isfinite(elevation_deg);
if any(xor(azimuthIsFinite, elevationIsFinite))
    messageFormat = ...
        "The %s azimuth/elevation boundary at slice %d must use " + ...
        "paired finite vertices and paired nonfinite separators.";
    error("normalizeAzElTimeObstacleData:UnpairedNonfiniteBoundary", ...
        messageFormat, boundaryRoleText, sampleIndex);
end
regionChanges = diff([false; azimuthIsFinite; false]);
regionStarts = find(regionChanges == 1);
regionStops = find(regionChanges == -1) - 1;
regionVertexCount = regionStops - regionStarts + 1;
oneVertexRegionIndex = find(regionVertexCount == 1, 1, "first");
if ~isempty(oneVertexRegionIndex)
    error("normalizeAzElTimeObstacleData:BoundaryRingTooShort", ...
        "The %s boundary region %d at slice %d has one finite vertex; " + ...
        "a nonempty region requires at least three vertices.", ...
        boundaryRoleText, oneVertexRegionIndex, sampleIndex);
end
removeRegion = regionVertexCount == 2;
removedRegionCount = nnz(removeRegion);
if removedRegionCount == 0
    return;
end

%% Section 2: Rebuild Retained Regions

retainedRegionIndex = find(~removeRegion);
if isempty(retainedRegionIndex)
    azimuth_deg = zeros(0, 1);
    elevation_deg = zeros(0, 1);
    return;
end
retainedAzimuth_deg = cell(numel(retainedRegionIndex), 1);
retainedElevation_deg = cell(numel(retainedRegionIndex), 1);
for retainedIndex = 1:numel(retainedRegionIndex)
    regionIndex = retainedRegionIndex(retainedIndex);
    rows = regionStarts(regionIndex):regionStops(regionIndex);
    retainedAzimuth_deg{retainedIndex} = azimuth_deg(rows);
    retainedElevation_deg{retainedIndex} = elevation_deg(rows);
end
separatorCount = numel(retainedRegionIndex) - 1;
outputVertexCount = sum(regionVertexCount(~removeRegion)) + separatorCount;
azimuth_deg = NaN(outputVertexCount, 1);
elevation_deg = NaN(outputVertexCount, 1);
outputIndex = 1;
for retainedIndex = 1:numel(retainedRegionIndex)
    currentRegionVertexCount = numel( ...
        retainedAzimuth_deg{retainedIndex});
    outputRows = ...
        outputIndex:outputIndex + currentRegionVertexCount - 1;
    azimuth_deg(outputRows) = retainedAzimuth_deg{retainedIndex};
    elevation_deg(outputRows) = retainedElevation_deg{retainedIndex};
    outputIndex = outputRows(end) + 2;
end
end

function rebuiltObstacles = rebuildProtectedObstacles( ...
        obstacleInput, safetyMargin_deg, optionOverrides)
%% Section 0: Header & Readme
% Normalize an arbitrary canonical container and rebuild protected geometry
% from retained original boundaries using one absolute margin.

validateattributes(safetyMargin_deg, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'nonnegative'});
[verbose, ~] = resolveProtectionOptions(optionOverrides);
rebuiltObstacles = azElObstacles.combineAzElObstacles(obstacleInput);
rebuiltObstacles = protectCanonicalObstacles( ...
    rebuiltObstacles, safetyMargin_deg, verbose);
end

function [verbose, resolvedOptions] = resolveProtectionOptions( ...
        optionOverrides)
%% Section 0: Header & Readme
% Resolve the obstacle-construction option for fresh and rebuild calls.

if ~isstruct(optionOverrides) || ~isscalar(optionOverrides)
    error("inflateAzElObstacleData:InvalidOptions", ...
        "options must be a scalar struct.");
end
defaultOptions = struct("Verbose", false);
[resolvedOptions, unknownOptionNames] = azElInput.resolveOptions( ...
    defaultOptions, optionOverrides);
if ~isempty(unknownOptionNames)
    warning("inflateAzElObstacleData:UnknownOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", ...
        strjoin(unknownOptionNames, ", "));
end
verbose = azElInput.normalizeLogicalScalar( ...
    resolvedOptions.Verbose, "Verbose", ...
    "inflateAzElObstacleData:InvalidVerbose");
resolvedOptions.Verbose = verbose;
end

function protectedObstacles = protectCanonicalObstacles( ...
        canonicalObstacles, safetyMargin_deg, verbose)
%% Section 0: Header & Readme
% Apply one absolute construction margin to normalized canonical obstacles.

protectedObstacles = canonicalObstacles;
for obstacleIndex = 1:numel(protectedObstacles)
    obstacle = protectedObstacles(obstacleIndex);
    obstacle.safetyMargin_deg = double(safetyMargin_deg);
    sampleCount = numel(obstacle.time_s);
    progressStride = max(1, ceil(sampleCount / 10));
    if verbose
        fprintf("[az/el protect] obstacle %d/%d '%s': %d slices.\n", ...
            obstacleIndex, numel(protectedObstacles), ...
            obstacle.targetName, sampleCount);
    end
    protectedAzimuthBySlice_deg = cell(sampleCount, 1);
    protectedElevationBySlice_deg = cell(sampleCount, 1);
    originalAzimuthBySlice_deg = obstacle.originalAz_deg;
    originalElevationBySlice_deg = obstacle.originalEl_deg;
    [templateProtectedAzimuth_deg, ...
        templateProtectedElevation_deg, templateSourceCount, ...
        templateProtectedCount] = protectOneSlice( ...
        originalAzimuthBySlice_deg{1}, ...
        originalElevationBySlice_deg{1}, safetyMargin_deg);
    for sampleIndex = 1:sampleCount
        [isTranslatedCopy, translation_deg] = translatedCopyOffset( ...
            originalAzimuthBySlice_deg{1}, ...
            originalElevationBySlice_deg{1}, ...
            originalAzimuthBySlice_deg{sampleIndex}, ...
            originalElevationBySlice_deg{sampleIndex});
        if isTranslatedCopy
            protectedAzimuthBySlice_deg{sampleIndex} = ...
                templateProtectedAzimuth_deg + translation_deg(1);
            protectedElevationBySlice_deg{sampleIndex} = ...
                templateProtectedElevation_deg + translation_deg(2);
            sourceCount = templateSourceCount;
            protectedCount = templateProtectedCount;
        else
            [protectedAzimuthBySlice_deg{sampleIndex}, ...
                protectedElevationBySlice_deg{sampleIndex}, ...
                sourceCount, protectedCount] = protectOneSlice( ...
                originalAzimuthBySlice_deg{sampleIndex}, ...
                originalElevationBySlice_deg{sampleIndex}, ...
                safetyMargin_deg);
        end
        reportProgress = sampleIndex == 1 || ...
            sampleIndex == sampleCount || ...
            mod(sampleIndex, progressStride) == 0;
        if verbose && reportProgress
            printProtectionProgress(struct( ...
                "Index", sampleIndex, ...
                "Count", sampleCount, ...
                "Time_s", obstacle.time_s(sampleIndex), ...
                "SourceCount", sourceCount, ...
                "ProtectedCount", protectedCount));
        end
    end
    obstacle.az_deg = protectedAzimuthBySlice_deg;
    obstacle.el_deg = protectedElevationBySlice_deg;
    protectedObstacles(obstacleIndex) = ...
        normalizeCanonicalObstacle(obstacle);
end
end

function [isTranslatedCopy, translation_deg] = translatedCopyOffset( ...
        templateAzimuth_deg, templateElevation_deg, ...
        sampleAzimuth_deg, sampleElevation_deg)
%% Section 0: Header & Readme
% Detect an exact rigid translation so one polygon buffer can be reused.

templatePosition_deg = [ ...
    double(templateAzimuth_deg(:)), double(templateElevation_deg(:))];
samplePosition_deg = [ ...
    double(sampleAzimuth_deg(:)), double(sampleElevation_deg(:))];
translation_deg = [0 0];
isTranslatedCopy = isequal( ...
    size(templatePosition_deg), size(samplePosition_deg));
if ~isTranslatedCopy
    return;
end
templateFinite = all(isfinite(templatePosition_deg), 2);
sampleFinite = all(isfinite(samplePosition_deg), 2);
isTranslatedCopy = isequal(templateFinite, sampleFinite);
if ~isTranslatedCopy || ~any(templateFinite)
    return;
end
offset_deg = samplePosition_deg(templateFinite, :) - ...
    templatePosition_deg(templateFinite, :);
translation_deg = offset_deg(1, :);
coordinateScale_deg = max(1, max(abs([ ...
    templatePosition_deg(templateFinite, :); ...
    samplePosition_deg(sampleFinite, :)]), [], "all"));
translationTolerance_deg = 256 * eps(coordinateScale_deg);
isTranslatedCopy = max(abs( ...
    offset_deg - translation_deg), [], "all") <= ...
    translationTolerance_deg;
end

function [protectedAzimuth_deg, protectedElevation_deg, ...
        sourceVertexCount, protectedVertexCount] = protectOneSlice( ...
        originalAzimuth_deg, originalElevation_deg, safetyMargin_deg)
%% Section 0: Header & Readme
% Protect one polygon slice and retain before/after vertex counts.

originalAzimuth_deg = double(originalAzimuth_deg(:));
originalElevation_deg = double(originalElevation_deg(:));
[protectedAzimuth_deg, protectedElevation_deg] = ...
    inflatePolygonSlice( ...
    originalAzimuth_deg, originalElevation_deg, safetyMargin_deg);
sourceVertexCount = nnz( ...
    isfinite(originalAzimuth_deg) & isfinite(originalElevation_deg));
protectedVertexCount = nnz( ...
    isfinite(protectedAzimuth_deg) & ...
    isfinite(protectedElevation_deg));
end

function printProtectionProgress(progress)
%% Section 0: Header & Readme
% Print deterministic completed-slice protection progress.

fprintf( ...
    "[az/el protect] slice %d/%d at t=%.3f s: " + ...
    "%d source -> %d protected vertices.\n", ...
    progress.Index, progress.Count, progress.Time_s, ...
    progress.SourceCount, progress.ProtectedCount);
end

function [protectedAzimuth_deg, protectedElevation_deg] = ...
        inflatePolygonSlice( ...
        azimuth_deg, elevation_deg, safetyMargin_deg)
%% Section 0: Header & Readme
% Apply one topology-aware outward buffer to a complete polygon slice.

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
pairedNonfinite = ~isfinite(azimuth_deg) & ~isfinite(elevation_deg);
azimuth_deg(pairedNonfinite) = NaN;
elevation_deg(pairedNonfinite) = NaN;
pairedFinite = isfinite(azimuth_deg) & isfinite(elevation_deg);
if nnz(pairedFinite) < 3
    protectedAzimuth_deg = zeros(0, 1);
    protectedElevation_deg = zeros(0, 1);
    return;
end
sourcePolygon = polyshape( ...
    azimuth_deg, elevation_deg, ...
    "Simplify", true, "KeepCollinearPoints", true);
if isempty(sourcePolygon.Vertices) || area(sourcePolygon) <= 0
    error("inflateAzElPolygonSlice:DegeneratePolygon", ...
        "The boundary slice does not define a nonzero-area polygon.");
end
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
