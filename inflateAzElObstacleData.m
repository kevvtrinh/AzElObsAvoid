function inflatedAzElData = inflateAzElObstacleData( azElData, safetyMargin_deg, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   inflatedAzElData = inflateAzElObstacleData( ...
%       azElData, safetyMargin_deg)
%   inflatedAzElData = inflateAzElObstacleData( ...
%       azElData, safetyMargin_deg, optionOverrides)
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
%   - optionOverrides (scalar struct, optional; default struct())
%       .Verbose prints periodic protection progress and final counts.
%**************************************************************************
% OUTPUTS
%   - inflatedAzElData (canonical obstacle struct array)
%       Records whose az_deg and el_deg fields are protected boundaries.
%**************************************************************************
% UNITS
%   - Polygon coordinates and safetyMargin_deg are degrees.
%**************************************************************************

%% Section 1: Resolve Options & Normalize Obstacles

validateattributes(safetyMargin_deg, {'numeric'}, {'scalar', 'real', 'finite', 'nonnegative'});
if nargin < 3 || isempty(optionOverrides)
    optionOverrides = struct();
end
if ~isstruct(optionOverrides) || ~isscalar(optionOverrides)
    error("inflateAzElObstacleData:InvalidOptions", "options must be a scalar struct.");
end
defaultOptions = struct("Verbose", false);
[resolvedOptions, unknownOptionNames] = azElInternal.resolveOptions(defaultOptions, optionOverrides);
if ~isempty(unknownOptionNames)
    warning("inflateAzElObstacleData:UnknownOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", strjoin(unknownOptionNames, ", "));
end
verbose = azElInternal.normalizeLogicalScalar( ...
    resolvedOptions.Verbose, "Verbose", "inflateAzElObstacleData:InvalidVerbose");
inflatedAzElData = combineAzElObstacles(azElData);

%% Section 2: Rebuild Protected Geometry From Original Geometry

% The requested margin is absolute. Rebuilding from original boundaries
% makes repeated calls idempotent and prevents downstream double inflation.
for obstacleIndex = 1:numel(inflatedAzElData)
    obstacle = inflatedAzElData(obstacleIndex);
    obstacle.safetyMargin_deg = double(safetyMargin_deg);
    sampleCount = numel(obstacle.time_s);
    progressStride = max(1, ceil(sampleCount / 10));
    if verbose
        fprintf("[az/el protect] obstacle %d/%d '%s': %d slices.\n", ...
            obstacleIndex, numel(inflatedAzElData), obstacle.targetName, sampleCount);
    end
    protectedAzimuthBySlice_deg = cell(sampleCount, 1);
    protectedElevationBySlice_deg = cell(sampleCount, 1);
    originalAzimuthBySlice_deg = obstacle.originalAz_deg;
    originalElevationBySlice_deg = obstacle.originalEl_deg;
    sampleTime_s = obstacle.time_s;
    [templateProtectedAzimuth_deg, ...
        templateProtectedElevation_deg, templateSourceCount, ...
        templateProtectedCount] = protectOneSlice( ...
        originalAzimuthBySlice_deg{1}, originalElevationBySlice_deg{1}, safetyMargin_deg);

    for sampleIndex = 1:sampleCount
        [isTranslatedCopy, translation_deg] = translatedCopyOffset( ...
            originalAzimuthBySlice_deg{1}, ...
            originalElevationBySlice_deg{1}, ...
            originalAzimuthBySlice_deg{sampleIndex}, originalElevationBySlice_deg{sampleIndex});
        if isTranslatedCopy
            protectedAzimuthBySlice_deg{sampleIndex} = templateProtectedAzimuth_deg + translation_deg(1);
            protectedElevationBySlice_deg{sampleIndex} = templateProtectedElevation_deg + translation_deg(2);
            sourceCount = templateSourceCount;
            protectedCount = templateProtectedCount;
        else
            [protectedAzimuthBySlice_deg{sampleIndex}, ...
                protectedElevationBySlice_deg{sampleIndex}, ...
                sourceCount, protectedCount] = protectOneSlice( ...
                originalAzimuthBySlice_deg{sampleIndex}, originalElevationBySlice_deg{sampleIndex}, safetyMargin_deg);
        end
        reportProgress = sampleIndex == 1 || sampleIndex == sampleCount || mod(sampleIndex, progressStride) == 0;
        if verbose && reportProgress
            printProtectionProgress(struct( ...
                "Index", sampleIndex, "Count", sampleCount, ...
                "Time_s", sampleTime_s(sampleIndex), "SourceCount", sourceCount, "ProtectedCount", protectedCount));
        end
    end
    obstacle.az_deg = protectedAzimuthBySlice_deg;
    obstacle.el_deg = protectedElevationBySlice_deg;
    inflatedAzElData(obstacleIndex) = normalizeAzElTimeObstacleData(obstacle);
end
end

%% Section 3: Local Functions

function [isTranslatedCopy, translation_deg] = translatedCopyOffset( ...
        templateAzimuth_deg, templateElevation_deg, sampleAzimuth_deg, sampleElevation_deg)
% Detect rigid translation so one exact polygon buffer can be reused.
templatePosition_deg = [double(templateAzimuth_deg(:)), double(templateElevation_deg(:))];
samplePosition_deg = [double(sampleAzimuth_deg(:)), double(sampleElevation_deg(:))];
translation_deg = [0 0];
isTranslatedCopy = isequal(size(templatePosition_deg), size(samplePosition_deg));
if ~isTranslatedCopy
    return;
end
templateFinite = all(isfinite(templatePosition_deg), 2);
sampleFinite = all(isfinite(samplePosition_deg), 2);
isTranslatedCopy = isequal(templateFinite, sampleFinite);
if ~isTranslatedCopy || ~any(templateFinite)
    return;
end
offset_deg = samplePosition_deg(templateFinite, :) - templatePosition_deg(templateFinite, :);
translation_deg = offset_deg(1, :);
coordinateScale_deg = max(1, max(abs([templatePosition_deg(templateFinite, :); ...
    samplePosition_deg(sampleFinite, :)]), [], "all"));
translationTolerance_deg = 256 * eps(coordinateScale_deg);
isTranslatedCopy = max(abs(offset_deg - translation_deg), [], "all") <= translationTolerance_deg;
end

function [protectedAzimuth_deg, protectedElevation_deg, ...
        sourceVertexCount, protectedVertexCount] = protectOneSlice( ...
        originalAzimuth_deg, originalElevation_deg, safetyMargin_deg)
% Protect one polygon slice for serial or parallel execution.
originalAzimuth_deg = double(originalAzimuth_deg(:));
originalElevation_deg = double(originalElevation_deg(:));
[protectedAzimuth_deg, protectedElevation_deg] = inflateAzElPolygonSlice(originalAzimuth_deg, ...
    originalElevation_deg, safetyMargin_deg);
sourceVertexCount = nnz(isfinite(originalAzimuth_deg) & isfinite(originalElevation_deg));
protectedVertexCount = nnz(isfinite(protectedAzimuth_deg) & isfinite(protectedElevation_deg));
end

function printProtectionProgress(progress)
% Report parallel slice completion on the MATLAB client.
fprintf( ...
    "[az/el protect] slice %d/%d at t=%.3f s: " + ...
    "%d source -> %d protected vertices.\n", ...
    progress.Index, progress.Count, progress.Time_s, progress.SourceCount, progress.ProtectedCount);
end

function [protectedAzimuth_deg, protectedElevation_deg] = inflateAzElPolygonSlice(azimuth_deg, elevation_deg, ...
        safetyMargin_deg)
% Apply one topology-aware outward buffer to a complete polygon slice.
validateattributes(azimuth_deg, {'numeric'}, {'real', 'column'});
validateattributes(elevation_deg, {'numeric'}, {'real', 'column'});
validateattributes(safetyMargin_deg, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
if numel(azimuth_deg) ~= numel(elevation_deg)
    error("inflateAzElPolygonSlice:BoundarySizeMismatch", "Azimuth and elevation boundaries must have equal lengths.");
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

sourcePolygon = polyshape(azimuth_deg, elevation_deg, "Simplify", true, "KeepCollinearPoints", true);
if isempty(sourcePolygon.Vertices) || area(sourcePolygon) <= 0
    error("inflateAzElPolygonSlice:DegeneratePolygon", "The boundary slice does not define a nonzero-area polygon.");
end
% Square joints are conservative at convex corners and avoid the dense arc
% sampling that would create many artificial retiming corners.
protectedPolygon = polybuffer( sourcePolygon, safetyMargin_deg, "JointType", "square");

[protectedAzimuth_deg, protectedElevation_deg] = boundary(protectedPolygon);
protectedAzimuth_deg = double(protectedAzimuth_deg(:));
protectedElevation_deg = double(protectedElevation_deg(:));

while ~isempty(protectedAzimuth_deg) && ~isfinite(protectedAzimuth_deg(end)) && ~isfinite(protectedElevation_deg(end))
    protectedAzimuth_deg(end) = [];
    protectedElevation_deg(end) = [];
end
end
