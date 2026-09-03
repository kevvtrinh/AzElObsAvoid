function [obstacleData, history] = createMovingObstacle( ...
        obstacleName, time_s, sourceAzimuth_deg, sourceElevation_deg, ...
        sliceTransform, safetyMargin_deg, options)
%% Section 0: Header & Readme
% SYNTAX
%   [obstacleData, history] = ...
%       obstacleAvoidance.obstacles.createMovingObstacle( ...
%       obstacleName, time_s, sourceAzimuth_deg, sourceElevation_deg, ...
%       sliceTransform, safetyMargin_deg)
%   [obstacleData, history] = ...
%       obstacleAvoidance.obstacles.createMovingObstacle( ...
%       obstacleName, time_s, sourceAzimuth_deg, sourceElevation_deg, ...
%       sliceTransform, safetyMargin_deg, options)
%**************************************************************************
% PURPOSE
%   - Create and protect an arbitrary moving or deforming obstacle history.
%   - Evaluate independent time slices deterministically in caller order.
%**************************************************************************
% INPUTS
%   - obstacleName (scalar text)
%   - time_s (nonempty increasing numeric vector)
%   - sourceAzimuth_deg, sourceElevation_deg (matching vectors)
%       Paired nonfinite rows may separate rings.
%   - sliceTransform (function handle)
%       position_deg = sliceTransform(sourcePosition_deg,time_s,index).
%       Output slices use the obstacle history contract: direct motion is
%       linear between verified corresponding vertices, not rigid arc motion.
%   - safetyMargin_deg (nonnegative scalar)
%   - options (scalar struct, optional; default struct())
%       Verbose prints bounded progress updates (default false).
%**************************************************************************
% OUTPUTS
%   - obstacleData (canonical protected moving obstacle)
%   - history (scalar struct)
%       Source slice boundaries, geometry metrics, and resolved options.
%**************************************************************************
% UNITS
%   - Position is degrees, time is seconds, and area is square degrees.
%   - See obstacle_history_contract.md for ring and fallback semantics.
%**************************************************************************

%% Section 1: Validate Inputs & Apply Defaults

if nargin < 7 || isempty(options)
    options = struct();
end
if ~isstruct(options) || ~isscalar(options)
    error("createMovingObstacle:InvalidOptions", "options must be a scalar struct.");
end
[resolvedOptions, unknownNames] = ...
    obstacleAvoidance.input.resolveOptions(struct("Verbose", false), options);
if ~isempty(unknownNames)
    warning("createMovingObstacle:UnknownOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", strjoin(unknownNames, ", "));
end
verbose = obstacleAvoidance.input.normalizeLogicalScalar( ...
    resolvedOptions.Verbose, "Verbose", "createMovingObstacle:InvalidVerbose");
resolvedOptions.Verbose = verbose;
if ~isa(sliceTransform, "function_handle")
    error("createMovingObstacle:InvalidTransform", "sliceTransform must be a function handle.");
end
time_s = double(time_s(:));
validateattributes(time_s, {'numeric'}, {'real', 'finite', 'nonempty', 'increasing'});
sourceAzimuth_deg = double(sourceAzimuth_deg(:));
sourceElevation_deg = double(sourceElevation_deg(:));
if numel(sourceAzimuth_deg) ~= numel(sourceElevation_deg)
    error("createMovingObstacle:BoundarySizeMismatch", ...
        "sourceAzimuth_deg and sourceElevation_deg must have equal size.");
end
if any(isfinite(sourceAzimuth_deg) ~= isfinite(sourceElevation_deg))
    error("createMovingObstacle:UnpairedNonfiniteBoundary", "Source separators must be paired.");
end
validateattributes(safetyMargin_deg, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});

%% Section 2: Create Independent Source Slices

sourcePosition_deg = [sourceAzimuth_deg, sourceElevation_deg];
sliceCount = numel(time_s);
azimuthBySlice_deg = cell(sliceCount, 1);
elevationBySlice_deg = cell(sliceCount, 1);
vertexCount = zeros(sliceCount, 1);
area_deg2 = zeros(sliceCount, 1);
aspectRatio = zeros(sliceCount, 1);
centroid_deg = zeros(sliceCount, 2);
bounds_deg = zeros(sliceCount, 4);
for sampleIndex = 1:sliceCount
    position_deg = sliceTransform(sourcePosition_deg, time_s(sampleIndex), sampleIndex);
    validateattributes(position_deg, {'numeric'}, {'real', '2d', 'ncols', 2, 'nonempty'});
    position_deg = double(position_deg);
    if any(isfinite(position_deg(:, 1)) ~= isfinite(position_deg(:, 2)))
        error("createMovingObstacle:UnpairedNonfiniteBoundary", ...
            "Slice %d returned unpaired separators.", sampleIndex);
    end
    finiteRows = all(isfinite(position_deg), 2);
    if nnz(finiteRows) < 3
        error("createMovingObstacle:TooFewVertices", ...
            "Slice %d must return at least three finite vertices.", sampleIndex);
    end
    finitePosition_deg = position_deg(finiteRows, :);
    minimum_deg = min(finitePosition_deg, [], 1);
    maximum_deg = max(finitePosition_deg, [], 1);
    size_deg = maximum_deg - minimum_deg;
    azimuthBySlice_deg{sampleIndex} = position_deg(:, 1);
    elevationBySlice_deg{sampleIndex} = position_deg(:, 2);
    vertexCount(sampleIndex) = nnz(finiteRows);
    centroid_deg(sampleIndex, :) = mean(finitePosition_deg, 1);
    bounds_deg(sampleIndex, :) = [minimum_deg, maximum_deg];
    aspectRatio(sampleIndex) = size_deg(1) / size_deg(2);
    if size_deg(2) == 0
        aspectRatio(sampleIndex) = Inf;
    end
    boundary_deg = position_deg;
    boundary_deg(~isfinite(boundary_deg)) = NaN;
    sliceShape = polyshape(boundary_deg(:, 1), boundary_deg(:, 2), "Simplify", false);
    area_deg2(sampleIndex) = area(sliceShape);
end

%% Section 3: Construct The Protected History

obstacleData = obstacleAvoidance.obstacles.createObstacle( ...
    obstacleName, time_s, azimuthBySlice_deg, elevationBySlice_deg, ...
    safetyMargin_deg, struct("Verbose", verbose));
history = struct("time_s", time_s, "azimuthBySlice_deg", {azimuthBySlice_deg}, ...
    "elevationBySlice_deg", {elevationBySlice_deg}, ...
    "vertexCount", vertexCount, "area_deg2", area_deg2, ...
    "aspectRatio", aspectRatio, "centroid_deg", centroid_deg, ...
    "bounds_deg", bounds_deg, "Options", resolvedOptions);
end
