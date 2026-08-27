function [obstacleData, history] = createMovingObstacle( ...
        obstacleName, time_s, sourceAzimuth_deg, sourceElevation_deg, sliceTransform, safetyMargin_deg, options)
%% Section 0: Header & Readme
% SYNTAX
%   [obstacleData, history] = obstacleAvoidance.obstacles.createMovingObstacle( ...
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
%   - obstacleData (canonical protected moving obstacle)
%   - history (scalar struct)
%       Source slice boundaries and geometry metrics.
%**************************************************************************
% UNITS
%   - Boundary positions are degrees and time_s is seconds. Area_deg2 is
%     square degrees; aspect ratio and vertex counts are dimensionless.
%**************************************************************************

%% Section 1: Validate Inputs & Apply Defaults

% Check that source geometry, time samples, and motion values have compatible
% sizes. Each output slice represents the obstacle at one absolute time.

% The callback is intentionally the only description of how the source
% boundary moves. This helper samples it in a predictable order, validates each
% returned polygon, and then passes the complete history to createObstacle for
% safety-margin processing.
if nargin < 7 || isempty(options)
    options = struct();
end
if ~isstruct(options) || ~isscalar(options)
    error("createMovingObstacle:InvalidOptions", "options must be a scalar struct.");
end
defaultOptions = struct("Verbose", false);
[resolvedOptions, unknownOptionFields] = obstacleAvoidance.input.resolveOptions(defaultOptions, options);
if ~isempty(unknownOptionFields)
    warning("createMovingObstacle:UnknownOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", strjoin(unknownOptionFields, ", "));
end
verbose = obstacleAvoidance.input.normalizeLogicalScalar( ...
    resolvedOptions.Verbose, "Verbose", "createMovingObstacle:InvalidVerbose");
resolvedOptions.Verbose = verbose;
if ~isa(sliceTransform, "function_handle")
    error("createMovingObstacle:InvalidTransform", "sliceTransform must be a function handle.");
end
time_s = double(time_s(:));
% A column time vector gives every history quantity the same sample-by-row
% orientation. Strict increase prevents duplicate or backward interpolation
% intervals.
validateattributes(time_s, {'numeric'}, {'real','finite','nonempty','increasing'});
sourceAzimuth_deg = double(sourceAzimuth_deg(:));
sourceElevation_deg = double(sourceElevation_deg(:));
if numel(sourceAzimuth_deg) ~= numel(sourceElevation_deg)
    error("createMovingObstacle:BoundarySizeMismatch", ...
        "sourceAzimuth_deg and sourceElevation_deg must have equal size.");
end
if any(isfinite(sourceAzimuth_deg) ~= isfinite(sourceElevation_deg))
    % A separator occupies one row across both coordinates. Pairing prevents a
    % nonfinite azimuth from being mistaken for a real vertex elevation.
    error("createMovingObstacle:UnpairedNonfiniteBoundary", ...
        "Source boundary separators must be paired in azimuth and elevation.");
end
validateattributes(safetyMargin_deg, {'numeric'}, {'real','finite','scalar','nonnegative'});

%% Section 2: Generate Independent Source Slices

% Transform the original geometry separately at each time. Do not transform a
% previous slice because repeated rotation and translation would accumulate
% numerical error.

sourcePosition_deg = [sourceAzimuth_deg, sourceElevation_deg];
sliceCount = numel(time_s);
progressStride = max(1, ceil(sliceCount / 10));
% Progress is limited to roughly ten intermediate updates so verbose output
% remains useful even when a history contains thousands of slices.
azimuthBySlice_deg = cell(sliceCount, 1);
elevationBySlice_deg = cell(sliceCount, 1);
vertexCount = zeros(sliceCount, 1);
area_deg2 = zeros(sliceCount, 1);
aspectRatio = zeros(sliceCount, 1);
centroid_deg = zeros(sliceCount, 2);
bounds_deg = zeros(sliceCount, 4);

% Evaluate the user transform at each requested time and retain both the
% boundary and diagnostics needed to audit that independently generated slice.
for sampleIndex = 1:sliceCount
    % Each call starts from the unchanged source boundary. The callback may
    % translate, rotate, deform, add rings, or change vertex count without any
    % hidden dependence on the previous slice.
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

% Pass all slices to the common constructor. It stores original geometry and
% applies the safety margin once. If protected geometry is wrong, compare the
% source slices before inspecting interpolation.

% createObstacle owns normalization and applies the safety margin exactly once.
% The separate history output retains unprotected source geometry and simple
% metrics for inspection; it does not modify the planner obstacle record.
constructionOptions = struct("Verbose", verbose);
obstacleData = obstacleAvoidance.obstacles.createObstacle( ...
    obstacleName, time_s, azimuthBySlice_deg, elevationBySlice_deg, safetyMargin_deg, constructionOptions);
history = struct( ...
    "time_s", time_s, ...
    "azimuthBySlice_deg", {azimuthBySlice_deg}, ...
    "elevationBySlice_deg", {elevationBySlice_deg}, ...
    "vertexCount", vertexCount, ...
    "area_deg2", area_deg2, ...
    "aspectRatio", aspectRatio, "centroid_deg", centroid_deg, "bounds_deg", bounds_deg, "Options", resolvedOptions);
end

function [azimuth_deg, elevation_deg, vertexCount, area_deg2, ...
        aspectRatio, centroid_deg, bounds_deg] = generateOneSlice( ...
        sliceTransform, sourcePosition_deg, sampleTime_s, sampleIndex)
% Evaluate and validate one independent moving-obstacle slice. sampleIndex is
% included in errors so a bad callback result can be reproduced directly.
position_deg = sliceTransform( sourcePosition_deg, sampleTime_s, sampleIndex);
validateattributes(position_deg, {'numeric'}, {'real','2d','ncols',2,'nonempty'});
position_deg = double(position_deg);
if any(isfinite(position_deg(:, 1)) ~= isfinite(position_deg(:, 2)))
    error("createMovingObstacle:UnpairedNonfiniteBoundary", ...
        "Slice %d returned unpaired boundary separators.", sampleIndex);
end
finiteRows = all(isfinite(position_deg), 2);
if nnz(finiteRows) < 3
    error("createMovingObstacle:TooFewVertices", ...
        "Slice %d must return at least three finite vertices.", sampleIndex);
end
azimuth_deg = position_deg(:, 1);
elevation_deg = position_deg(:, 2);
finitePosition_deg = position_deg(finiteRows, :);
% Metrics intentionally ignore separator rows. Their purpose is to describe the
% finite geometry, while the returned coordinate vectors retain separators for
% disconnected rings or holes.
vertexCount = size(finitePosition_deg, 1);
minimum_deg = min(finitePosition_deg, [], 1);
maximum_deg = max(finitePosition_deg, [], 1);
bounds_deg = [minimum_deg, maximum_deg];
size_deg = maximum_deg - minimum_deg;
if size_deg(2) > 0
    aspectRatio = size_deg(1) / size_deg(2);
else
    % A zero elevation span makes width/height undefined as a finite ratio.
    % Infinity states that degeneracy explicitly instead of dividing by zero.
    aspectRatio = Inf;
end
centroid_deg = mean(finitePosition_deg, 1);
boundaryAzimuth_deg = azimuth_deg;
boundaryElevation_deg = elevation_deg;
boundaryAzimuth_deg(~finiteRows) = NaN;
boundaryElevation_deg(~finiteRows) = NaN;
sliceShape = polyshape( boundaryAzimuth_deg, boundaryElevation_deg, "Simplify", false);
% Converting every nonfinite separator to NaN gives polyshape one consistent
% ring delimiter even if the callback used positive or negative infinity.
area_deg2 = area(sliceShape);
end

function printCompletedSlice(progress)
% Report completion on the client without interleaved worker output.
fprintf( ...
    "[moving obstacle] slice %d/%d at t=%.3f s complete " + ...
    "(%d source vertices).\n", progress.Index, progress.Count, progress.Time_s, progress.VertexCount);
end
