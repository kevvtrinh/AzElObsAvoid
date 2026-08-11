function azElData = normalizeAzElTimeObstacleData(azElData)
%% Section 0: Header & Readme
% SYNTAX
%   azElData = normalizeAzElTimeObstacleData(azElData)
%**************************************************************************
% PURPOSE
%   - Validate and normalize one canonical time-indexed az/el obstacle.
%**************************************************************************
% INPUTS
%   - azElData (scalar struct)
%       Record with targetName, time_s, az_deg, el_deg, and status fields.
%**************************************************************************
% OUTPUTS
%   - azElData (scalar struct)
%       Canonical record with column time, boundary cells, and status.
%**************************************************************************
% UNITS
%   - Coordinates are degrees and timestamps are seconds.

%% Section 1: Validate The Record Shape
requiredFields = ["targetName", "time_s", "az_deg", "el_deg", ...
    "status"];
if ~isstruct(azElData) || ~isscalar(azElData)
    error("normalizeAzElTimeObstacleData:InvalidRecord", ...
        "Each obstacle must be one scalar structure.");
end
for fieldIndex = 1:numel(requiredFields)
    fieldName = requiredFields(fieldIndex);
    if ~isfield(azElData, fieldName)
        error("normalizeAzElTimeObstacleData:MissingField", ...
            "Obstacle data is missing required field '%s'.", fieldName);
    end
end

targetName = string(azElData.targetName);
if ~isscalar(targetName) || strlength(strtrim(targetName)) == 0
    error("normalizeAzElTimeObstacleData:InvalidTargetName", ...
        "targetName must be nonempty scalar text.");
end

time_s = double(azElData.time_s(:));
validateattributes(time_s, "numeric", ["real", "finite", ...
    "nonempty", "increasing"], mfilename, "time_s");

%% Section 2: Normalize Boundary Slices
sampleCount = numel(time_s);
if ~iscell(azElData.az_deg) || ~iscell(azElData.el_deg)
    error("normalizeAzElTimeObstacleData:BoundaryCellsRequired", ...
        "az_deg and el_deg must be cell arrays in canonical records.");
end
azimuthSlices_deg = reshape(azElData.az_deg, [], 1);
elevationSlices_deg = reshape(azElData.el_deg, [], 1);
if numel(azimuthSlices_deg) ~= sampleCount || ...
        numel(elevationSlices_deg) ~= sampleCount
    error("normalizeAzElTimeObstacleData:SampleCountMismatch", ...
        ["Boundary cell counts must match time_s. Received %d azimuth " ...
        "slices, %d elevation slices, and %d times."], ...
        numel(azimuthSlices_deg), numel(elevationSlices_deg), sampleCount);
end

for sampleIndex = 1:sampleCount
    azimuthBoundary_deg = double(azimuthSlices_deg{sampleIndex}(:));
    elevationBoundary_deg = double(elevationSlices_deg{sampleIndex}(:));
    if numel(azimuthBoundary_deg) ~= numel(elevationBoundary_deg)
        error("normalizeAzElTimeObstacleData:BoundaryLengthMismatch", ...
            ["Obstacle '%s' sample %d has %d azimuth values and %d " ...
            "elevation values."], targetName, sampleIndex, ...
            numel(azimuthBoundary_deg), numel(elevationBoundary_deg));
    end
    pairedFinite = isfinite(azimuthBoundary_deg) == ...
        isfinite(elevationBoundary_deg);
    if ~all(pairedFinite)
        error("normalizeAzElTimeObstacleData:UnpairedSeparator", ...
            ["Obstacle '%s' sample %d contains an unpaired nonfinite " ...
            "region separator."], targetName, sampleIndex);
    end
    finiteCoordinates = azimuthBoundary_deg(isfinite(azimuthBoundary_deg));
    finiteElevations = elevationBoundary_deg( ...
        isfinite(elevationBoundary_deg));
    if any(~isfinite(finiteCoordinates)) || ...
            any(~isfinite(finiteElevations))
        error("normalizeAzElTimeObstacleData:InvalidCoordinate", ...
            "Obstacle coordinates must be finite or paired separators.");
    end
    validateRegionLengths(azimuthBoundary_deg, targetName, sampleIndex);
    azimuthSlices_deg{sampleIndex} = azimuthBoundary_deg;
    elevationSlices_deg{sampleIndex} = elevationBoundary_deg;
end

%% Section 3: Normalize Status & Assemble
status = string(azElData.status);
if isscalar(status)
    status = repmat(status, sampleCount, 1);
else
    status = reshape(status, [], 1);
end
if numel(status) ~= sampleCount
    error("normalizeAzElTimeObstacleData:StatusCountMismatch", ...
        "status must be scalar text or contain one value per time sample.");
end

azElData = struct( ...
    "targetName", targetName, ...
    "time_s", time_s, ...
    "az_deg", {azimuthSlices_deg}, ...
    "el_deg", {elevationSlices_deg}, ...
    "status", status);
end

function validateRegionLengths(boundary_deg, targetName, sampleIndex)
%% Section 0: Header & Readme
% SYNTAX
%   validateRegionLengths(boundary_deg, targetName, sampleIndex)
%**************************************************************************
% PURPOSE
%   - Require at least three finite vertices in every nonempty region.
%**************************************************************************
% INPUTS
%   - boundary_deg (numeric vector)
%       One coordinate vector with nonfinite region separators.
%   - targetName (scalar string)
%       Obstacle name used in diagnostics.
%   - sampleIndex (positive integer)
%       Sample index used in diagnostics.
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - boundary_deg is measured in degrees.

if isempty(boundary_deg)
    return;
end
separatorMask = ~isfinite(boundary_deg);
separatorIndices = [0; find(separatorMask); numel(boundary_deg) + 1];
for regionIndex = 1:(numel(separatorIndices) - 1)
    vertexCount = separatorIndices(regionIndex + 1) - ...
        separatorIndices(regionIndex) - 1;
    if vertexCount > 0 && vertexCount < 3
        error("normalizeAzElTimeObstacleData:TooFewVertices", ...
            ["Obstacle '%s' sample %d region %d has %d vertices; " ...
            "at least three are required."], targetName, sampleIndex, ...
            regionIndex, vertexCount);
    end
end
end
