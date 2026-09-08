function [obstacleData, history] = createMovingObstacle(obstacleName, time_s, sourceX_units, sourceY_units, sliceTransform, safetyMargin_units, options)
%% Section 0: Header & Readme
% SYNTAX
%   [obstacleData, history] = ...
%       obstacleAvoidance.obstacles.createMovingObstacle( ...
%       obstacleName, time_s, sourceX_units, sourceY_units, ...
%       sliceTransform, safetyMargin_units)
%   [obstacleData, history] = ...
%       obstacleAvoidance.obstacles.createMovingObstacle( ...
%       obstacleName, time_s, sourceX_units, sourceY_units, ...
%       sliceTransform, safetyMargin_units, options)
%
% PURPOSE
%   - Create and protect an arbitrary moving or deforming obstacle history.
%   - Evaluate independent time slices deterministically in caller order.
%
% INPUTS
%   - obstacleName (scalar text)
%   - time_s (nonempty increasing numeric vector)
%   - sourceX_units, sourceY_units (matching vectors)
%       Paired nonfinite rows may separate rings.
%   - sliceTransform (function handle)
%       position_units = sliceTransform(sourcePosition_units,time_s,index).
%       Output slices use the obstacle history contract: direct motion is
%       linear between verified corresponding vertices, not rigid arc motion.
%   - safetyMargin_units (nonnegative scalar)
%   - options (scalar struct, optional; default struct())
%       Verbose prints bounded progress updates (default false).
%
% OUTPUTS
%   - obstacleData (canonical protected moving obstacle)
%   - history (scalar struct)
%       Source slice boundaries, geometry metrics, and resolved options.
%
% UNITS
%   - Position is coordinate units, time is seconds, and area is square coordinate units.
%   - See obstacle_history_contract.md for ring and fallback semantics.
%

%% Section 1: Validate Inputs & Apply Defaults

if nargin < 7 || isempty(options)
    options = struct();
end
if ~isstruct(options) || ~isscalar(options)
    error("createMovingObstacle:InvalidOptions", "options must be a scalar struct.");
end
[resolvedOptions, unknownNames] = obstacleAvoidance.input.resolveOptions(struct("Verbose", false), options);
if ~isempty(unknownNames)
    warning("createMovingObstacle:UnknownOptions", "Ignoring unknown option fields: %s. No behavior changed.", strjoin(unknownNames, ", "));
end
verbose = obstacleAvoidance.input.normalizeLogicalScalar(resolvedOptions.Verbose, "Verbose", "createMovingObstacle:InvalidVerbose");
resolvedOptions.Verbose = verbose;
if ~isa(sliceTransform, "function_handle")
    error("createMovingObstacle:InvalidTransform", "sliceTransform must be a function handle.");
end
time_s = double(time_s(:));
validateattributes(time_s, {'numeric'}, {'real', 'finite', 'nonempty', 'increasing'});
sourceX_units   = double(sourceX_units(:));
sourceY_units = double(sourceY_units(:));
if numel(sourceX_units) ~= numel(sourceY_units)
    error("createMovingObstacle:BoundarySizeMismatch", "sourceX_units and sourceY_units must have equal size.");
end
if any(isfinite(sourceX_units) ~= isfinite(sourceY_units))
    error("createMovingObstacle:UnpairedNonfiniteBoundary", "Source separators must be paired.");
end
validateattributes(safetyMargin_units, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});

%% Section 2: Create Independent Source Slices

sourcePosition_units   = [sourceX_units, sourceY_units];
sliceCount           = numel(time_s);
xBySlice_units   = cell(sliceCount, 1);
yBySlice_units = cell(sliceCount, 1);
vertexCount          = zeros(sliceCount, 1);
area_units2            = zeros(sliceCount, 1);
aspectRatio          = zeros(sliceCount, 1);
centroid_units         = zeros(sliceCount, 2);
bounds_units           = zeros(sliceCount, 4);
% Process each sample in temporal order and accumulate its result.
for sampleIndex = 1:sliceCount
    position_units = sliceTransform(sourcePosition_units, time_s(sampleIndex), sampleIndex);
    validateattributes(position_units, {'numeric'}, {'real', '2d', 'ncols', 2, 'nonempty'});
    position_units = double(position_units);
    if any(isfinite(position_units(:, 1)) ~= isfinite(position_units(:, 2)))
        error("createMovingObstacle:UnpairedNonfiniteBoundary", "Slice %d returned unpaired separators.", sampleIndex);
    end
    finiteRows = all(isfinite(position_units), 2);
    if nnz(finiteRows) < 3
        error("createMovingObstacle:TooFewVertices", "Slice %d must return at least three finite vertices.", sampleIndex);
    end
    finitePosition_units = position_units(finiteRows, :);
    minimum_units        = min(finitePosition_units, [], 1);
    maximum_units        = max(finitePosition_units, [], 1);
    size_units           = maximum_units - minimum_units;
    xBySlice_units{sampleIndex} = position_units(:, 1);
    yBySlice_units{sampleIndex} = position_units(:, 2);
    vertexCount(sampleIndex) = nnz(finiteRows);
    centroid_units(sampleIndex, :) = mean(finitePosition_units, 1);
    bounds_units(sampleIndex, :) = [minimum_units, maximum_units];
    aspectRatio(sampleIndex) = size_units(1) / size_units(2);
    if size_units(2) == 0
        aspectRatio(sampleIndex) = Inf;
    end
    boundary_units = position_units;
    boundary_units(~isfinite(boundary_units)) = NaN;
    sliceShape = polyshape(boundary_units(:, 1), boundary_units(:, 2), "Simplify", false);
    area_units2(sampleIndex) = area(sliceShape);
end

%% Section 3: Construct The Protected History

obstacleData = obstacleAvoidance.obstacles.createObstacle(obstacleName, time_s, xBySlice_units, yBySlice_units, safetyMargin_units, struct("Verbose", verbose));
history      = struct("time_s", time_s, "xBySlice_units", {xBySlice_units}, ...
    "yBySlice_units", {yBySlice_units}, ...
    "vertexCount", vertexCount, "area_units2", area_units2, ...
    "aspectRatio", aspectRatio, "centroid_units", centroid_units, ...
    "bounds_units", bounds_units, "Options", resolvedOptions);
end
