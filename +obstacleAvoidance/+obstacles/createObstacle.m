function obstacleData = createObstacle(obstacleInput, varargin)
%% Section 0: Header & Readme
% SYNTAX
%   obstacleData = obstacleAvoidance.obstacles.createObstacle( ...
%       obstacleName, time_s, azimuthBoundary_deg, elevationBoundary_deg)
%   obstacleData = obstacleAvoidance.obstacles.createObstacle( ...
%       obstacleName, time_s, azimuthBoundary_deg, ...
%       elevationBoundary_deg, safetyMargin_deg)
%   obstacleData = obstacleAvoidance.obstacles.createObstacle( ...
%       obstacleName, time_s, azimuthBoundary_deg, ...
%       elevationBoundary_deg, safetyMargin_deg, constructionOptions)
%   obstacleData = obstacleAvoidance.obstacles.createObstacle(canonicalObstacle)
%   obstacleData = obstacleAvoidance.obstacles.createObstacle( ...
%       canonicalObstacles, safetyMargin_deg)
%   obstacleData = obstacleAvoidance.obstacles.createObstacle( ...
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
%   - obstacleData (canonical scalar or column struct array)
%       Validated obstacles with original geometry, protected geometry,
%       construction margin, sample status, and stable field order.
%**************************************************************************
% UNITS
%   - Boundary coordinates and safety margins are degrees; time is seconds.
%**************************************************************************

%% Section 1: Select The Supported Construction Operation

% A zero-input call returns defaults. Other calls create or normalize obstacle
% data. This dispatch keeps all public construction paths on one implementation.

% This function supports three related operations through one public entry
% point: create a record from raw coordinates, normalize an existing record, or
% rebuild existing protected geometry with a new absolute margin. Classifying
% the first input here keeps those paths separate and prevents a structure from
% being mistaken for an obstacle name.
if nargin == 0
    error("createObstacle:MissingInput", ...
        "Obstacle construction or canonical obstacle input is required.");
end
isCanonicalContainer = isstruct(obstacleInput) || ...
    iscell(obstacleInput) || ...
    (isnumeric(obstacleInput) && isempty(obstacleInput));
if isCanonicalContainer && nargin == 1
    obstacleData = normalizeCanonicalObstacle(obstacleInput);
    return;
elseif isCanonicalContainer && nargin >= 2 && nargin <= 3
    safetyMargin_deg = varargin{1};
    if nargin < 3 || isempty(varargin{2})
        constructionOptions = struct();
    else
        constructionOptions = varargin{2};
    end
    % Rebuild means replace the previous margin, not add another layer.
    obstacleData = rebuildProtectedObstacles( ...
        obstacleInput, safetyMargin_deg, constructionOptions);
    return;
end
if nargin < 4 || nargin > 6
    error("createObstacle:InvalidCall", ...
        "Construction requires name, time, azimuth, and elevation; " + ...
        "canonical rebuild requires obstacle data and an absolute margin.");
end

%% Section 2: Construct One Raw Canonical Record

% Collect name, time, boundary history, active spans, and safety margin in one
% raw record. Do not apply protection in this step. The next step owns that work.

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
    % A numeric boundary describes a static shape. Replicate it across all
    % supplied times so static and changing obstacles share one history format.
    azimuthBoundary_deg = repmat( ...
        {double(azimuthBoundary_deg(:))}, sampleCount, 1);
end
if ~iscell(elevationBoundary_deg)
    elevationBoundary_deg = repmat( ...
        {double(elevationBoundary_deg(:))}, sampleCount, 1);
end
% Both protected and original fields initially contain the source geometry.
% Protection below changes only az_deg and el_deg, preserving an uninflated
% copy for future absolute-margin rebuilds.
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

% Validate the raw record, preserve original boundaries, and create protected
% boundaries. Protected boundaries include the margin used by collision checks.

% Construction and imported canonical data share the same format check.
% Absolute reconstruction always starts from original geometry so repeated
% calls cannot accumulate a safety margin.
obstacleData = normalizeCanonicalObstacle(rawObstacle);
[verbose, ~] = resolveProtectionOptions(constructionOptions);
obstacleData = protectCanonicalObstacles( ...
    obstacleData, safetyMargin_deg, verbose);
end

function obstacleData = normalizeCanonicalObstacle(inputData)
% Validate and column-normalize one canonical obstacle record. Established
% normalization identifiers remain stable for malformed imported records.

%% Section 1: Validate Structure And Sample Time

% Require increasing finite sample times and one boundary slice per time. A
% failure here usually means history arrays were assembled with different sizes.

requiredFields = ["targetName", "time_s", "az_deg", "el_deg", "status"];
hasRequiredStructure = isstruct(inputData) && isscalar(inputData);
if hasRequiredStructure
    hasRequiredStructure = all(isfield( ...
        inputData, cellstr(requiredFields)));
end
if ~hasRequiredStructure
    error("createObstacle:InvalidInput", ...
        "obstacleData must be a scalar canonical obstacle record with " + ...
        "targetName, time_s, az_deg, el_deg, and status.");
end
targetName = string(inputData.targetName);
if ~isscalar(targetName) || strlength(strtrim(targetName)) == 0
    error("createObstacle:InvalidTargetName", ...
        "targetName must be nonempty scalar text.");
end
validateattributes(inputData.time_s, ...
    {'numeric'}, {'vector', 'real', 'finite'});
time_s = double(inputData.time_s(:));
% Time is stored as a column to align cell row i with time row i. Strict
% increase guarantees every later interpolation interval has positive length.
sampleCount = numel(time_s);
if sampleCount == 0 || any(diff(time_s) <= 0)
    error("createObstacle:InvalidTime", ...
        "time_s must be nonempty and strictly increasing.");
end

%% Section 2: Validate Protected Boundary Slices

% Check each boundary ring before polygon operations. Remove no valid geometry
% silently. Warnings identify a sample that lost an invalid or tiny region.

% az_deg and el_deg are parallel cell histories. Each cell contains one full
% polygon boundary, including paired nonfinite separators between rings.
hasCellBoundaries = iscell(inputData.az_deg) && ...
    iscell(inputData.el_deg);
hasMatchingSampleCounts = numel(inputData.az_deg) == sampleCount && ...
    numel(inputData.el_deg) == sampleCount;
if ~hasCellBoundaries || ~hasMatchingSampleCounts
    error("createObstacle:InvalidBoundary", ...
        "az_deg and el_deg must be cell arrays matching time_s.");
end
[azimuthSlices_deg, elevationSlices_deg, ...
    removedProtectedRegionCount, protectedRemovalBySample] = ...
    normalizeBoundaryHistory( ...
    inputData.az_deg, inputData.el_deg, sampleCount, "protected");

%% Section 3: Preserve Original Geometry And Margin

% Keep original and protected boundaries as separate data. Apply the safety
% margin once to original geometry. Double margins often start in this section
% or in a caller that supplied already protected vertices as original data.

% Protected and original histories must either both be available or both be
% absent. Keeping the source boundary is what makes changing an existing safety
% margin an absolute operation instead of cumulative geometric inflation.
hasOriginalBoundaries = isfield(inputData, "originalAz_deg") && ...
    isfield(inputData, "originalEl_deg");
hasOnlyOneOriginalBoundary = xor( ...
    isfield(inputData, "originalAz_deg"), ...
    isfield(inputData, "originalEl_deg"));
if hasOnlyOneOriginalBoundary
    error("createObstacle:IncompleteOriginalBoundary", ...
        "originalAz_deg and originalEl_deg must either both be " + ...
        "present or both be absent.");
end
if hasOriginalBoundaries
    originalHistoryIsValid = iscell(inputData.originalAz_deg) && ...
        iscell(inputData.originalEl_deg) && ...
        numel(inputData.originalAz_deg) == sampleCount && ...
        numel(inputData.originalEl_deg) == sampleCount;
    if ~originalHistoryIsValid
        error("createObstacle:InvalidOriginalBoundary", ...
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
    % Two points define a line segment, not occupied area. Removing such rings
    % is visible through one summary warning; one-point rings remain errors
    % because they more strongly suggest malformed input.
    removalBySample = protectedRemovalBySample | originalRemovalBySample;
    warning("createObstacle:RemovedTwoVertexRegions", ...
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
    % Without source geometry there is no reliable way to determine whether the
    % stored protected boundary has already been inflated once or many times.
    error("createObstacle:MissingOriginalBoundary", ...
        "A positive safetyMargin_deg requires originalAz_deg and " + ...
        "originalEl_deg. Construct protected obstacles with " + ...
        "createObstacle.");
end

%% Section 4: Normalize Status And Assemble Output

% Normalize active time spans and write fields in stable order. Active spans
% control when an obstacle exists. They do not change its stored boundary data.

% A scalar status applies to the whole history. A vector can describe visibility
% or source state independently at each time sample.
statusBySample = string(inputData.status);
if isscalar(statusBySample)
    statusBySample = repmat(statusBySample, sampleCount, 1);
elseif numel(statusBySample) ~= sampleCount
    error("createObstacle:StatusSizeMismatch", ...
        "status must contain one value per time sample.");
else
    statusBySample = statusBySample(:);
end
obstacleData = struct( ...
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
% Normalize one protected or original boundary history uniformly. boundaryRole
% selects field-specific error identifiers while the geometry rules stay equal.

azimuthSlices_deg = reshape(azimuthInput_deg, [], 1);
elevationSlices_deg = reshape(elevationInput_deg, [], 1);
removedRegionCount = 0;
removalBySample = false(sampleCount, 1);
roleIndex = 1 + (boundaryRole == "original");
% Logical comparison yields 0 or 1; adding one selects the protected or original
% row from the paired identifier and field-name tables.
mismatchIdentifiers = [ ...
    "createObstacle:BoundarySizeMismatch", ...
    "createObstacle:OriginalBoundarySizeMismatch"];
allFieldNames = [ ...
    "az_deg", "el_deg"; ...
    "originalAz_deg", "originalEl_deg"];
mismatchIdentifier = mismatchIdentifiers(roleIndex);
fieldNames = allFieldNames(roleIndex, :);
for sampleIndex = 1:sampleCount
    % Process slices independently so diagnostics identify the exact bad sample
    % and dynamic histories may legitimately use different vertex counts.
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
% Reject mismatched separators and one-vertex regions, remove two-vertex
% regions, and preserve valid polygon regions in caller order.

%% Section 1: Classify Boundary Regions

% Classify each ring by area and nesting. Outer rings add occupied area. Inner
% rings can describe holes. Check ring direction and containment when a hole is
% interpreted as a separate obstacle.

boundaryRoleText = char(boundaryRole);
azimuthIsFinite = isfinite(azimuth_deg);
elevationIsFinite = isfinite(elevation_deg);
if any(xor(azimuthIsFinite, elevationIsFinite))
    messageFormat = ...
        "The %s azimuth/elevation boundary at slice %d must use " + ...
        "paired finite vertices and paired nonfinite separators.";
    error("createObstacle:UnpairedNonfiniteBoundary", ...
        messageFormat, boundaryRoleText, sampleIndex);
end
regionChanges = diff([false; azimuthIsFinite; false]);
% Each consecutive run of finite coordinate pairs is one ring. Padding with
% false values lets the same transition logic detect rings at both ends.
regionStarts = find(regionChanges == 1);
regionStops = find(regionChanges == -1) - 1;
regionVertexCount = regionStops - regionStarts + 1;
oneVertexRegionIndex = find(regionVertexCount == 1, 1, "first");
if ~isempty(oneVertexRegionIndex)
    error("createObstacle:BoundaryRingTooShort", ...
        "The %s boundary region %d at slice %d has one finite vertex; " + ...
        "a nonempty region requires at least three vertices.", ...
        boundaryRoleText, oneVertexRegionIndex, sampleIndex);
end
removeRegion = regionVertexCount == 2;
% A two-vertex ring encloses zero area and is safe to remove explicitly. A ring
% with three or more vertices is retained here; its actual area is checked when
% polygon operations require it.
removedRegionCount = nnz(removeRegion);
if removedRegionCount == 0
    return;
end

%% Section 2: Rebuild Retained Regions

% Rebuild only valid classified regions. Preserve NaN separators so later edge
% and shape functions keep disconnected parts separate.

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
% Exactly one NaN row is inserted between retained rings and none after the last
% ring. Preallocation makes the output size and separator placement explicit.
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
    % Skip one preinitialized NaN row before writing the next region.
end
end

function rebuiltObstacles = rebuildProtectedObstacles( ...
        obstacleInput, safetyMargin_deg, optionOverrides)
% Normalize an arbitrary canonical container and rebuild protected geometry
% from retained original boundaries using one absolute margin.
% combineObstacles first normalizes every supported nested input form. Protection
% then starts from each record's originalAz_deg/originalEl_deg history.

validateattributes(safetyMargin_deg, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'nonnegative'});
[verbose, ~] = resolveProtectionOptions(optionOverrides);
rebuiltObstacles = obstacleAvoidance.obstacles.combineObstacles(obstacleInput);
rebuiltObstacles = protectCanonicalObstacles( ...
    rebuiltObstacles, safetyMargin_deg, verbose);
end

function [verbose, resolvedOptions] = resolveProtectionOptions( ...
        optionOverrides)
% Resolve the obstacle-construction option for fresh and rebuild calls.

if ~isstruct(optionOverrides) || ~isscalar(optionOverrides)
    error("createObstacle:InvalidProtectionOptions", ...
        "options must be a scalar struct.");
end
defaultOptions = struct("Verbose", false);
[resolvedOptions, unknownOptionNames] = obstacleAvoidance.input.resolveOptions( ...
    defaultOptions, optionOverrides);
if ~isempty(unknownOptionNames)
    warning("createObstacle:UnknownProtectionOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", ...
        strjoin(unknownOptionNames, ", "));
end
verbose = obstacleAvoidance.input.normalizeLogicalScalar( ...
    resolvedOptions.Verbose, "Verbose", ...
        "createObstacle:InvalidVerbose");
resolvedOptions.Verbose = verbose;
end

function protectedObstacles = protectCanonicalObstacles( ...
        canonicalObstacles, safetyMargin_deg, verbose)
% Apply one absolute construction margin to normalized canonical obstacles.

protectedObstacles = canonicalObstacles;
for obstacleIndex = 1:numel(protectedObstacles)
    % Obstacles may use unrelated time histories and shapes, so process and
    % normalize each record independently.
    obstacle = protectedObstacles(obstacleIndex);
    obstacle.safetyMargin_deg = double(safetyMargin_deg);
    sampleCount = numel(obstacle.time_s);
    progressStride = max(1, ceil(sampleCount / 10));
    % At most about ten periodic messages are printed for long histories.
    if verbose
        fprintf("[az/el protect] obstacle %d/%d '%s': %d slices.\n", ...
            obstacleIndex, numel(protectedObstacles), ...
            obstacle.targetName, sampleCount);
    end
    protectedAzimuthBySlice_deg = cell(sampleCount, 1);
    protectedElevationBySlice_deg = cell(sampleCount, 1);
    originalAzimuthBySlice_deg = obstacle.originalAz_deg;
    originalElevationBySlice_deg = obstacle.originalEl_deg;
    % Buffer the first slice once. A pure translation of that slice can move
    % this protected template exactly, avoiding repeated polygon buffering.
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
            % Translation commutes with a fixed-distance outward buffer: moving
            % the protected template gives the same geometry as buffering the
            % moved source polygon.
            protectedAzimuthBySlice_deg{sampleIndex} = ...
                templateProtectedAzimuth_deg + translation_deg(1);
            protectedElevationBySlice_deg{sampleIndex} = ...
                templateProtectedElevation_deg + translation_deg(2);
            sourceCount = templateSourceCount;
            protectedCount = templateProtectedCount;
        else
            % Rotation, deformation, topology changes, and vertex reordering all
            % require independent protection of the current source slice.
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
% Equal separator locations preserve the same ring and vertex correspondence.
if ~isTranslatedCopy || ~any(templateFinite)
    return;
end
offset_deg = samplePosition_deg(templateFinite, :) - ...
    templatePosition_deg(templateFinite, :);
translation_deg = offset_deg(1, :);
% A rigid translation gives every finite vertex the same two-axis offset. Use
% the first vertex as the candidate and compare all remaining offsets to it.
coordinateScale_deg = max(1, max(abs([ ...
    templatePosition_deg(templateFinite, :); ...
    samplePosition_deg(sampleFinite, :)]), [], "all"));
translationTolerance_deg = 256 * eps(coordinateScale_deg);
% Scale the floating-point tolerance to coordinate magnitude. It recognizes
% arithmetic roundoff without treating genuine deformation as translation.
isTranslatedCopy = max(abs( ...
    offset_deg - translation_deg), [], "all") <= ...
    translationTolerance_deg;
end

function [protectedAzimuth_deg, protectedElevation_deg, ...
        sourceVertexCount, protectedVertexCount] = protectOneSlice( ...
        originalAzimuth_deg, originalElevation_deg, safetyMargin_deg)
% Protect one polygon slice and retain before/after vertex counts.
% Counts help verbose reporting show how buffering changed geometric complexity.

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
% Apply one topology-aware outward buffer to a complete polygon slice.

validateattributes(azimuth_deg, {'numeric'}, {'real', 'column'});
validateattributes(elevation_deg, {'numeric'}, {'real', 'column'});
validateattributes(safetyMargin_deg, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'});
if numel(azimuth_deg) ~= numel(elevation_deg)
    error("createObstacle:BoundarySizeMismatch", ...
        "Azimuth and elevation boundaries must have equal lengths.");
end
azimuth_deg = double(azimuth_deg);
elevation_deg = double(elevation_deg);
safetyMargin_deg = double(safetyMargin_deg);
if safetyMargin_deg == 0
    % Preserve the source coordinates exactly when no protection is requested;
    % even a harmless polygon round trip could reorder rings or vertices.
    protectedAzimuth_deg = azimuth_deg;
    protectedElevation_deg = elevation_deg;
    return;
end
pairedNonfinite = ~isfinite(azimuth_deg) & ~isfinite(elevation_deg);
% Normalize any paired Inf/-Inf separators to NaN before handing the boundary
% to polyshape, whose multi-ring representation uses NaN delimiters.
azimuth_deg(pairedNonfinite) = NaN;
elevation_deg(pairedNonfinite) = NaN;
pairedFinite = isfinite(azimuth_deg) & isfinite(elevation_deg);
if nnz(pairedFinite) < 3
    % No polygonal area can be formed. Return the standard empty boundary rather
    % than asking polyshape to interpret a degenerate slice.
    protectedAzimuth_deg = zeros(0, 1);
    protectedElevation_deg = zeros(0, 1);
    return;
end
sourcePolygon = polyshape( ...
    azimuth_deg, elevation_deg, ...
    "Simplify", true, "KeepCollinearPoints", true);
if isempty(sourcePolygon.Vertices) || area(sourcePolygon) <= 0
    % Three input vertices may still be collinear or self-canceling. Area is the
    % final check that the boundary truly describes occupied space.
    error("createObstacle:DegeneratePolygon", ...
        "The boundary slice does not define a nonzero-area polygon.");
end
% A square joint keeps corners outside the requested Euclidean margin and
% avoids rounding them inward relative to sharp source vertices.
protectedPolygon = polybuffer( ...
    sourcePolygon, safetyMargin_deg, "JointType", "square");
[protectedAzimuth_deg, protectedElevation_deg] = ...
    boundary(protectedPolygon);
protectedAzimuth_deg = double(protectedAzimuth_deg(:));
protectedElevation_deg = double(protectedElevation_deg(:));
% MATLAB may return one or more trailing separators. They separate nothing
% and can confuse topology comparisons, so remove only trailing pairs.
while ~isempty(protectedAzimuth_deg) && ...
        ~isfinite(protectedAzimuth_deg(end)) && ...
        ~isfinite(protectedElevation_deg(end))
    protectedAzimuth_deg(end) = [];
    protectedElevation_deg(end) = [];
end
end
