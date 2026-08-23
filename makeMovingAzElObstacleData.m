function [azElData, history] = makeMovingAzElObstacleData( ...
        obstacleName, time_s, sourceAzimuth_deg, sourceElevation_deg, sliceTransform, safetyMargin_deg, options)
%% Section 0: Header & Readme
% SYNTAX
%   [azElData, history] = makeMovingAzElObstacleData( ...
%       obstacleName, time_s, sourceAzimuth_deg, sourceElevation_deg, ...
%       sliceTransform, safetyMargin_deg, options)
%**************************************************************************
% PURPOSE
%   - Generate, validate, and protect an arbitrary moving/deforming dense
%     az/el obstacle history outside example scripts.
%   - Evaluate independent time slices deterministically in caller order.
%   - Make no rigid-body, shared-shape, shared-topology, or shared-vertex
%     assumption between slices.
%**************************************************************************
% INPUTS
%   - obstacleName (scalar text)
%   - time_s (nonempty increasing numeric vector)
%   - sourceAzimuth_deg, sourceElevation_deg (matching vectors)
%       A source boundary supplied to the callback. Paired NaN/Inf values
%       may separate rings.
%   - sliceTransform (function handle)
%       position_deg = sliceTransform(sourcePosition_deg,time_s,index).
%       It must return an arbitrary N-by-2 [azimuth elevation] boundary;
%       N and topology may differ independently at every time slice.
%   - safetyMargin_deg (nonnegative scalar)
%   - options (scalar struct, optional)
%       .Verbose prints periodic completed-slice progress (default false).
%**************************************************************************
% OUTPUTS
%   - azElData (canonical protected moving obstacle)
%   - history (scalar struct)
%       Source slice boundaries and geometry metrics.
%**************************************************************************
% UNITS
%   - Boundary positions are degrees and time_s is seconds. Area_deg2 is
%     square degrees; aspect ratio and vertex counts are dimensionless.
%**************************************************************************

%% Section 1: Validate Inputs & Apply Defaults

if nargin < 7 || isempty(options)
    options = struct();
end
if ~isstruct(options) || ~isscalar(options)
    error("makeMovingAzElObstacleData:InvalidOptions", "options must be a scalar struct.");
end
defaultOptions = struct("Verbose", false);
[resolvedOptions, unknownOptionFields] = azElInternal.resolveOptions(defaultOptions, options);
if ~isempty(unknownOptionFields)
    warning("makeMovingAzElObstacleData:UnknownOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", strjoin(unknownOptionFields, ", "));
end
verbose = azElInternal.normalizeLogicalScalar( ...
    resolvedOptions.Verbose, "Verbose", "makeMovingAzElObstacleData:InvalidVerbose");
resolvedOptions.Verbose = verbose;
if ~isa(sliceTransform, "function_handle")
    error("makeMovingAzElObstacleData:InvalidTransform", "sliceTransform must be a function handle.");
end
time_s = double(time_s(:));
validateattributes(time_s, {'numeric'}, {'real','finite','nonempty','increasing'});
sourceAzimuth_deg = double(sourceAzimuth_deg(:));
sourceElevation_deg = double(sourceElevation_deg(:));
if numel(sourceAzimuth_deg) ~= numel(sourceElevation_deg)
    error("makeMovingAzElObstacleData:BoundarySizeMismatch", ...
        "sourceAzimuth_deg and sourceElevation_deg must have equal size.");
end
if any(isfinite(sourceAzimuth_deg) ~= isfinite(sourceElevation_deg))
    error("makeMovingAzElObstacleData:UnpairedNonfiniteBoundary", ...
        "Source boundary separators must be paired in azimuth and elevation.");
end
validateattributes(safetyMargin_deg, {'numeric'}, {'real','finite','scalar','nonnegative'});

%% Section 2: Generate Independent Source Slices

sourcePosition_deg = [sourceAzimuth_deg, sourceElevation_deg];
sliceCount = numel(time_s);
progressStride = max(1, ceil(sliceCount / 10));
azimuthBySlice_deg = cell(sliceCount, 1);
elevationBySlice_deg = cell(sliceCount, 1);
vertexCount = zeros(sliceCount, 1);
area_deg2 = zeros(sliceCount, 1);
aspectRatio = zeros(sliceCount, 1);
centroid_deg = zeros(sliceCount, 2);
bounds_deg = zeros(sliceCount, 4);

for sampleIndex = 1:sliceCount
    [azimuthBySlice_deg{sampleIndex}, ...
        elevationBySlice_deg{sampleIndex}, vertexCount(sampleIndex), ...
        area_deg2(sampleIndex), aspectRatio(sampleIndex), ...
        centroid_deg(sampleIndex, :), bounds_deg(sampleIndex, :)] = generateOneSlice(sliceTransform, sourcePosition_deg, ...
        time_s(sampleIndex), sampleIndex);
    reportProgress = sampleIndex == 1 || sampleIndex == sliceCount || mod(sampleIndex, progressStride) == 0;
    if verbose && reportProgress
        printCompletedSlice(struct( ...
            "Index", sampleIndex, "Count", sliceCount, ...
            "Time_s", time_s(sampleIndex), "VertexCount", vertexCount(sampleIndex)));
    end
end

%% Section 3: Construct The Canonical Protected History

constructionOptions = struct("Verbose", verbose);
azElData = makeAzElObstacleData( ...
    obstacleName, time_s, azimuthBySlice_deg, elevationBySlice_deg, safetyMargin_deg, constructionOptions);
history = struct( ...
    "time_s", time_s, ...
    "azimuthBySlice_deg", {azimuthBySlice_deg}, ...
    "elevationBySlice_deg", {elevationBySlice_deg}, ...
    "vertexCount", vertexCount, ...
    "area_deg2", area_deg2, ...
    "aspectRatio", aspectRatio, "centroid_deg", centroid_deg, "bounds_deg", bounds_deg, "Options", resolvedOptions);
end

%% Section 4: Local Functions

function [azimuth_deg, elevation_deg, vertexCount, area_deg2, ...
        aspectRatio, centroid_deg, bounds_deg] = generateOneSlice( ...
        sliceTransform, sourcePosition_deg, sampleTime_s, sampleIndex)
% Evaluate and validate one independent moving-obstacle slice.
position_deg = sliceTransform( sourcePosition_deg, sampleTime_s, sampleIndex);
validateattributes(position_deg, {'numeric'}, {'real','2d','ncols',2,'nonempty'});
position_deg = double(position_deg);
if any(isfinite(position_deg(:, 1)) ~= isfinite(position_deg(:, 2)))
    error("makeMovingAzElObstacleData:UnpairedNonfiniteBoundary", ...
        "Slice %d returned unpaired boundary separators.", sampleIndex);
end
finiteRows = all(isfinite(position_deg), 2);
if nnz(finiteRows) < 3
    error("makeMovingAzElObstacleData:TooFewVertices", ...
        "Slice %d must return at least three finite vertices.", sampleIndex);
end
azimuth_deg = position_deg(:, 1);
elevation_deg = position_deg(:, 2);
finitePosition_deg = position_deg(finiteRows, :);
vertexCount = size(finitePosition_deg, 1);
minimum_deg = min(finitePosition_deg, [], 1);
maximum_deg = max(finitePosition_deg, [], 1);
bounds_deg = [minimum_deg, maximum_deg];
size_deg = maximum_deg - minimum_deg;
if size_deg(2) > 0
    aspectRatio = size_deg(1) / size_deg(2);
else
    aspectRatio = Inf;
end
centroid_deg = mean(finitePosition_deg, 1);
boundaryAzimuth_deg = azimuth_deg;
boundaryElevation_deg = elevation_deg;
boundaryAzimuth_deg(~finiteRows) = NaN;
boundaryElevation_deg(~finiteRows) = NaN;
sliceShape = polyshape( boundaryAzimuth_deg, boundaryElevation_deg, "Simplify", false);
area_deg2 = area(sliceShape);
end

function printCompletedSlice(progress)
% Report completion on the client without interleaved worker output.
fprintf( ...
    "[moving obstacle] slice %d/%d at t=%.3f s complete " + ...
    "(%d source vertices).\n", progress.Index, progress.Count, progress.Time_s, progress.VertexCount);
end
