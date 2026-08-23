function azElData = normalizeTimeObstacleData(inputData)
%% Section 0: Header & Readme
% SYNTAX
%   azElData = azElPlannerMethods.corridor.normalizeTimeObstacleData(inputData)
%**************************************************************************
% PURPOSE
%   - Validate and column-normalize one canonical azElData record.
%**************************************************************************
% INPUTS
%   - inputData (scalar struct)
%       targetName, time_s, az_deg, el_deg, and status are required.
%       Paired nonfinite rows separate regions. A two-vertex region is
%       removed with one warning because it cannot enclose occupied area.
%**************************************************************************
% OUTPUTS
%   - azElData (scalar struct)
%       Validated canonical obstacle record with final and original
%       boundaries plus the construction-time safety margin.
%**************************************************************************
% UNITS
%   - az_deg and el_deg are degrees; time_s is seconds.
%**************************************************************************

%% Section 1: Validate Structure & Sample Time

requiredFields = ["targetName", "time_s", "az_deg", "el_deg", "status"];
hasRequiredStructure = isstruct(inputData) && isscalar(inputData);
if hasRequiredStructure
    hasRequiredStructure = all(isfield(inputData, cellstr(requiredFields)));
end
if ~hasRequiredStructure
    error("normalizeAzElTimeObstacleData:InvalidInput", ...
        "azElData must be a scalar canonical obstacle record with " + ...
        "targetName, time_s, az_deg, el_deg, and status.");
end
targetName = string(inputData.targetName);
if ~isscalar(targetName) || strlength(strtrim(targetName)) == 0
    error("normalizeAzElTimeObstacleData:InvalidTargetName", "targetName must be nonempty scalar text.");
end
validateattributes(inputData.time_s, {'numeric'}, {'vector', 'real', 'finite'});
time_s = double(inputData.time_s(:));
sampleCount = numel(time_s);
% Strict ordering is required because nearest-slice lookup and temporal
% padding both assume that adjacent row indices are adjacent in time.
if sampleCount == 0 || any(diff(time_s) <= 0)
    error("normalizeAzElTimeObstacleData:InvalidTime", "time_s must be nonempty and strictly increasing.");
end

%% Section 2: Validate Boundary Slices

hasCellBoundaries = iscell(inputData.az_deg) && iscell(inputData.el_deg);
hasMatchingAzimuthSamples = numel(inputData.az_deg) == sampleCount;
hasMatchingElevationSamples = numel(inputData.el_deg) == sampleCount;
matchingSampleFlags = [ hasMatchingAzimuthSamples, hasMatchingElevationSamples];
hasMatchingSampleCounts = all(matchingSampleFlags);
if ~hasCellBoundaries || ~hasMatchingSampleCounts
    error("normalizeAzElTimeObstacleData:InvalidBoundary", "az_deg and el_deg must be cell arrays matching time_s.");
end

% Column-oriented cell arrays give all downstream packers one predictable
% shape while allowing each time slice to contain a different vertex count.
[azimuthSlices_deg, elevationSlices_deg, ...
    removedProtectedRegionCount, protectedRemovalBySample] = normalizeBoundaryHistory(inputData.az_deg, inputData.el_deg, ...
    sampleCount, "protected");

%% Section 3: Preserve Original Geometry & Construction Margin

hasOriginalBoundaries = isfield(inputData, "originalAz_deg") && isfield(inputData, "originalEl_deg");
hasOnlyOneOriginalBoundary = xor( isfield(inputData, "originalAz_deg"), isfield(inputData, "originalEl_deg"));
if hasOnlyOneOriginalBoundary
    error("normalizeAzElTimeObstacleData:IncompleteOriginalBoundary", ...
        "originalAz_deg and originalEl_deg must either both be " + "present or both be absent.");
end
if hasOriginalBoundaries
    if ~iscell(inputData.originalAz_deg) || ...
            ~iscell(inputData.originalEl_deg) || ...
            numel(inputData.originalAz_deg) ~= sampleCount || numel(inputData.originalEl_deg) ~= sampleCount
        error("normalizeAzElTimeObstacleData:InvalidOriginalBoundary", ...
            "originalAz_deg and originalEl_deg must be cell arrays " + "matching time_s.");
    end
    [originalAzimuthSlices_deg, originalElevationSlices_deg, ...
        removedOriginalRegionCount, originalRemovalBySample] = normalizeBoundaryHistory(inputData.originalAz_deg, ...
        inputData.originalEl_deg, sampleCount, "original");
else
    originalAzimuthSlices_deg = azimuthSlices_deg;
    originalElevationSlices_deg = elevationSlices_deg;
    removedOriginalRegionCount = 0;
    originalRemovalBySample = false(sampleCount, 1);
end

removedRegionCount = removedProtectedRegionCount + removedOriginalRegionCount;
if removedRegionCount > 0
    removalBySample = protectedRemovalBySample | originalRemovalBySample;
    warning("normalizeAzElTimeObstacleData:RemovedTwoVertexRegions", ...
        "Obstacle '%s' removed %d protected and %d original " + ...
        "two-vertex regions across %d time slices. Two vertices cannot " + ...
        "enclose occupied area; all remaining regions were unchanged.", ...
        targetName, removedProtectedRegionCount, removedOriginalRegionCount, nnz(removalBySample));
end
if isfield(inputData, "safetyMargin_deg")
    safetyMargin_deg = inputData.safetyMargin_deg;
else
    safetyMargin_deg = 0;
end
validateattributes(safetyMargin_deg, {'numeric'}, {'scalar', 'real', 'finite', 'nonnegative'});
safetyMargin_deg = double(safetyMargin_deg);
if safetyMargin_deg > 0 && ~hasOriginalBoundaries
    error("normalizeAzElTimeObstacleData:MissingOriginalBoundary", ...
        "A positive safetyMargin_deg requires originalAz_deg and " + ...
        "originalEl_deg. Construct protected obstacles with " + "makeAzElObstacleData.");
end

%% Section 4: Normalize Status & Assemble The Output

statusBySample = string(inputData.status);
% A scalar status is shorthand for a uniform history. Status is preserved
% rather than interpreted here because the obstacle-field builder owns the
% policy for which labels produce active obstacle geometry.
if isscalar(statusBySample)
    statusBySample = repmat(statusBySample, sampleCount, 1);
elseif numel(statusBySample) ~= sampleCount
    error("normalizeAzElTimeObstacleData:StatusSizeMismatch", "status must contain one value per time sample.");
else
    statusBySample = statusBySample(:);
end

azElData = struct( ...
    "targetName", targetName, ...
    "time_s", time_s, ...
    "az_deg", {azimuthSlices_deg}, ...
    "el_deg", {elevationSlices_deg}, ...
    "originalAz_deg", {originalAzimuthSlices_deg}, ...
    "originalEl_deg", {originalElevationSlices_deg}, "safetyMargin_deg", safetyMargin_deg, "status", statusBySample);
end


function [azimuthSlices_deg, elevationSlices_deg, removedRegionCount, ...
        removalBySample] = normalizeBoundaryHistory( azimuthInput_deg, elevationInput_deg, sampleCount, boundaryRole)
% Normalize one protected or original boundary history uniformly.
% SYNTAX
%   [azimuthSlices_deg, elevationSlices_deg, removedRegionCount, ...
%       removalBySample] = normalizeBoundaryHistory(azimuthInput_deg, ...
%       elevationInput_deg, sampleCount, boundaryRole)
%**************************************************************************
% INPUTS
%   - azimuthInput_deg, elevationInput_deg (cell arrays), paired histories.
%   - sampleCount (positive integer scalar), required history length.
%   - boundaryRole (scalar string), either "protected" or "original".
%**************************************************************************
% OUTPUTS
%   - azimuthSlices_deg, elevationSlices_deg (sampleCount-by-1 cells).
%   - removedRegionCount (nonnegative integer scalar), total removals.
%   - removalBySample (sampleCount-by-1 logical), samples with removals.
%**************************************************************************
% UNITS
%   - Boundary coordinates are degrees; counts are dimensionless.
%**************************************************************************
azimuthSlices_deg = reshape(azimuthInput_deg, [], 1);
elevationSlices_deg = reshape(elevationInput_deg, [], 1);
removedRegionCount = 0;
removalBySample = false(sampleCount, 1);
roleIndex = 1 + (boundaryRole == "original");
mismatchIdentifiers = [ ...
    "normalizeAzElTimeObstacleData:BoundarySizeMismatch", "normalizeAzElTimeObstacleData:OriginalBoundarySizeMismatch"];
allFieldNames = ["az_deg", "el_deg"; "originalAz_deg", "originalEl_deg"];
mismatchIdentifier = mismatchIdentifiers(roleIndex);
fieldNames = allFieldNames(roleIndex, :);

% Validate and normalize every time slice so paired azimuth/elevation histories stay aligned.
for sampleIndex = 1:sampleCount
    validateattributes(azimuthSlices_deg{sampleIndex}, {'numeric'}, {'vector', 'real'});
    validateattributes(elevationSlices_deg{sampleIndex}, {'numeric'}, {'vector', 'real'});
    if numel(azimuthSlices_deg{sampleIndex}) ~= numel(elevationSlices_deg{sampleIndex})
        error(mismatchIdentifier, ...
            "%s and %s slice %d must have equal lengths.", fieldNames(1), fieldNames(2), sampleIndex);
    end
    azimuthSlices_deg{sampleIndex} = double( azimuthSlices_deg{sampleIndex}(:));
    elevationSlices_deg{sampleIndex} = double( elevationSlices_deg{sampleIndex}(:));
    [azimuthSlices_deg{sampleIndex}, ...
        elevationSlices_deg{sampleIndex}, removedAtSampleCount] = normalizeBoundarySliceTopology(azimuthSlices_deg{sampleIndex}, ...
        elevationSlices_deg{sampleIndex}, sampleIndex, boundaryRole);
    removedRegionCount = removedRegionCount + removedAtSampleCount;
    removalBySample(sampleIndex) = removedAtSampleCount > 0;
end
end

function [azimuth_deg, elevation_deg, removedRegionCount] = normalizeBoundarySliceTopology(azimuth_deg, elevation_deg, ...
        sampleIndex, boundaryRole)
% Reject mismatched separators and one-vertex regions.
boundaryRoleText = char(boundaryRole);
azimuthIsFinite = isfinite(azimuth_deg);
elevationIsFinite = isfinite(elevation_deg);
if any(xor(azimuthIsFinite, elevationIsFinite))
    messageFormat = "The %s azimuth/elevation boundary at slice %d must use " + ...
        "paired finite vertices and paired nonfinite separators.";
    error("normalizeAzElTimeObstacleData:UnpairedNonfiniteBoundary", messageFormat, boundaryRoleText, sampleIndex);
end

%% Section 2: Classify Boundary Regions

finiteVertexMask = azimuthIsFinite;
regionChanges = diff([false; finiteVertexMask; false]);
regionStarts = find(regionChanges == 1);
regionStops = find(regionChanges == -1) - 1;
regionVertexCount = regionStops - regionStarts + 1;
oneVertexRegionIndex = find(regionVertexCount == 1, 1, "first");
if ~isempty(oneVertexRegionIndex)
    error("normalizeAzElTimeObstacleData:BoundaryRingTooShort", ...
        "The %s boundary region %d at slice %d has one finite vertex; " + ...
        "a nonempty region requires at least three vertices.", boundaryRoleText, oneVertexRegionIndex, sampleIndex);
end
removeRegion = regionVertexCount == 2;
removedRegionCount = nnz(removeRegion);
if removedRegionCount == 0
    return;
end

%% Section 3: Rebuild The Remaining Regions

retainedRegionIndex = find(~removeRegion);
if isempty(retainedRegionIndex)
    azimuth_deg = zeros(0, 1);
    elevation_deg = zeros(0, 1);
    return;
end
retainedAzimuth_deg = cell(numel(retainedRegionIndex), 1);
retainedElevation_deg = cell(numel(retainedRegionIndex), 1);

% Copy each surviving polygon region into its own temporary boundary slice.
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

% Reassemble the retained regions into one NaN-separated boundary vector.
for retainedIndex = 1:numel(retainedRegionIndex)
    currentRegionVertexCount = numel( retainedAzimuth_deg{retainedIndex});
    outputRows = outputIndex:outputIndex + currentRegionVertexCount - 1;
    azimuth_deg(outputRows) = retainedAzimuth_deg{retainedIndex};
    elevation_deg(outputRows) = retainedElevation_deg{retainedIndex};
    outputIndex = outputRows(end) + 2;
end
end
