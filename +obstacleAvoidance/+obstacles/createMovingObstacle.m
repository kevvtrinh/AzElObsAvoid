function [obstacleData, history] = createMovingObstacle(obstacleName, time_s, ...
    sourceX_units, sourceY_units, sliceTransform, safetyMargin_units, options)
%% Section 0: Header & Readme
% SYNTAX
%   [obstacleData, history] = obstacleAvoidance.obstacles.createMovingObstacle( ...
%       obstacleName, time_s, sourceX_units, sourceY_units, ...
%       sliceTransform, safetyMargin_units)
%   [obstacleData, history] = obstacleAvoidance.obstacles.createMovingObstacle( ...
%       obstacleName, time_s, sourceX_units, sourceY_units, ...
%       sliceTransform, safetyMargin_units, options)
%**************************************************************************
% PURPOSE
%   - Transform one original boundary at each requested time, then add its
%     protection margin and collect the samples into an obstacle history.
%   - Evaluate samples in the supplied order. Each transform starts from
%     the original boundary, so changes do not accumulate between samples.
%**************************************************************************
% INPUTS
%   - obstacleName (scalar text)
%       Nonempty obstacle name.
%   - time_s (increasing numeric vector)
%       Times at which the original boundary is transformed.
%   - sourceX_units (numeric vector)
%       Original boundary x coordinates.
%   - sourceY_units (numeric vector)
%       Original boundary y coordinates matching sourceX_units.
%   - sliceTransform (function handle)
%       Called as positions = sliceTransform(sourcePositions, time, index),
%       where sourcePositions contains [x y] rows and index starts at 1.
%       Return transformed [x y] rows while preserving vertex identity.
%   - safetyMargin_units (nonnegative numeric scalar)
%       Protection margin applied once to each transformed sample.
%   - options (scalar struct, optional; default struct())
%       Verbose defaults to false.
%**************************************************************************
% OUTPUTS
%   - obstacleData (scalar struct)
%       Standard obstacle record with original and protected sample boundaries.
%   - history (scalar struct)
%       Original sample boundaries, geometry measurements, and options.
%       centroid_units stores the mean boundary-vertex position; bounds_units
%       stores [xmin ymin xmax ymax]. Invalid input throws an error.
%**************************************************************************
% UNITS
%   - Position is coordinate units, time is seconds, and area is units^2.
%**************************************************************************

%% Section 1: Validate Inputs And Apply Defaults

if nargin < 7 || isempty(options)
    options = struct();
end
if ~isstruct(options) || ~isscalar(options)
    error("createMovingObstacle:InvalidOptions", "options must be a scalar struct.");
end
[resolvedOptions, unknownOptionNames] = obstacleAvoidance.input.resolveOptions( ...
    struct("Verbose", false), options);
if ~isempty(unknownOptionNames)
    warning("createMovingObstacle:UnknownOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", ...
        strjoin(unknownOptionNames, ", "));
end
verbose = obstacleAvoidance.input.normalizeLogicalScalar( ...
    resolvedOptions.Verbose, "Verbose", "createMovingObstacle:InvalidVerbose");
resolvedOptions.Verbose = verbose;
if ~isa(sliceTransform, "function_handle")
    error("createMovingObstacle:InvalidTransform", "sliceTransform must be a function handle.");
end
time_s = double(time_s(:));
validateattributes(time_s, {'numeric'}, {'real', 'finite', 'nonempty', 'increasing'});
sourceX_units = double(sourceX_units(:));
sourceY_units = double(sourceY_units(:));
if numel(sourceX_units) ~= numel(sourceY_units)
    error("createMovingObstacle:BoundarySizeMismatch", "sourceX_units and sourceY_units must have equal size.");
end
if any(isfinite(sourceX_units) ~= isfinite(sourceY_units))
    error("createMovingObstacle:UnpairedNonfiniteBoundary", "Source separators must be paired.");
end
validateattributes(safetyMargin_units, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});

%% Section 2: Calculate The Boundary At Each Sample Time

sourceVertices_units = [sourceX_units, sourceY_units];
sampleCount          = numel(time_s);
sampleX_units        = cell(sampleCount, 1);
sampleY_units        = cell(sampleCount, 1);

sampleVertexCounts       = zeros(sampleCount, 1);
sampleAreas_units2       = zeros(sampleCount, 1);
sampleAspectRatios       = zeros(sampleCount, 1);
meanVertexPosition_units = zeros(sampleCount, 2);
sampleBounds_units       = zeros(sampleCount, 4);

% Pass the original vertices each time. For example, a transform that adds
% [time 0] gives a 2-unit shift at 2 s, without adding the 1 s shift again.
for sampleIndex = 1:sampleCount
    sampleVertices_units = sliceTransform(sourceVertices_units, time_s(sampleIndex), sampleIndex);
    validateattributes(sampleVertices_units, {'numeric'}, {'real', '2d', 'ncols', 2, 'nonempty'});
    sampleVertices_units = double(sampleVertices_units);
    if any(isfinite(sampleVertices_units(:, 1)) ~= isfinite(sampleVertices_units(:, 2)))
        error("createMovingObstacle:UnpairedNonfiniteBoundary", "Slice %d returned unpaired separators.", sampleIndex);
    end
    vertexIsFinite = all(isfinite(sampleVertices_units), 2);
    if nnz(vertexIsFinite) < 3
        error("createMovingObstacle:TooFewVertices", "Slice %d must return at least three finite vertices.", sampleIndex);
    end

    % Measure the original sample before protection. Separator rows do not
    % contribute to the bounds, average vertex position, or vertex count.
    finiteVertices_units  = sampleVertices_units(vertexIsFinite, :);
    boundaryMinimum_units = min(finiteVertices_units, [], 1);
    boundaryMaximum_units = max(finiteVertices_units, [], 1);
    boundarySize_units    = boundaryMaximum_units - boundaryMinimum_units;
    sampleX_units{sampleIndex} = sampleVertices_units(:, 1);
    sampleY_units{sampleIndex} = sampleVertices_units(:, 2);
    sampleVertexCounts(sampleIndex)          = nnz(vertexIsFinite);
    meanVertexPosition_units(sampleIndex, :) = mean(finiteVertices_units, 1);
    sampleBounds_units(sampleIndex, :)       = [boundaryMinimum_units, boundaryMaximum_units];
    sampleAspectRatios(sampleIndex)          = boundarySize_units(1) / boundarySize_units(2);
    if boundarySize_units(2) == 0
        sampleAspectRatios(sampleIndex) = Inf;
    end

    % Polyshape uses NaN rows to separate boundary loops.
    polygonBoundary_units = sampleVertices_units;
    polygonBoundary_units(~isfinite(polygonBoundary_units)) = NaN;
    sampleShape = polyshape(polygonBoundary_units(:, 1), polygonBoundary_units(:, 2), "Simplify", false);
    sampleAreas_units2(sampleIndex) = area(sampleShape);
end

%% Section 3: Add Protection And Package The History

% The transform is expected to keep each row linked to the same original
% vertex. Record that matching rule so preparation uses those vertex paths.
% createObstacle adds the margin and retains both original and protected data.
constructionOptions = struct( ...
    "Verbose",              verbose, ...
    "vertexCorrespondence", "sourceIndex");
obstacleData = obstacleAvoidance.obstacles.createObstacle( ...
    obstacleName, time_s, sampleX_units, sampleY_units, ...
    safetyMargin_units, constructionOptions);
history = struct( ...
    "time_s",         time_s, ...
    "xBySlice_units", {sampleX_units}, ...
    "yBySlice_units", {sampleY_units}, ...
    "vertexCount",    sampleVertexCounts, ...
    "area_units2",    sampleAreas_units2, ...
    "aspectRatio",    sampleAspectRatios, ...
    "centroid_units", meanVertexPosition_units, ...
    "bounds_units",   sampleBounds_units, ...
    "Options",        resolvedOptions);
end
