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
%   - Own canonical obstacle construction and normalization.
%   - Rebuild protected histories from retained original geometry so an
%     absolute safety margin is applied exactly once.
%**************************************************************************
% INPUTS
%   - obstacleInput (scalar text or canonical obstacle container)
%   - varargin
%       Construction uses increasing time_s, matching boundary histories,
%       an optional nonnegative margin, and optional scalar options.
%       Rebuild uses an absolute margin and optional scalar options.
%       constructionOptions.Verbose defaults to false.
%       Paired nonfinite rows separate rings. Ring orientation and first
%       vertex are representation details. The status field is metadata and
%       does not deactivate physical geometry.
%**************************************************************************
% OUTPUTS
%   - obstacleData (canonical scalar or column struct array)
%       Original and protected histories, margin, status, and stable fields.
%**************************************************************************
% UNITS
%   - Boundary coordinates and safety margins are degrees; time is seconds.
%   - See obstacle_history_contract.md for between-sample semantics.
%**************************************************************************

%% Section 1: Select Construction Or Canonical Rebuild

if nargin == 0
    error("createObstacle:MissingInput", "Obstacle construction or canonical input is required.");
end
isContainer = isstruct(obstacleInput) || iscell(obstacleInput) || ...
    (isnumeric(obstacleInput) && isempty(obstacleInput));
if isContainer && nargin == 1
    obstacleData = normalizeOne(obstacleInput);
    return;
elseif isContainer && nargin >= 2 && nargin <= 3
    safetyMargin_deg = varargin{1};
    options = struct();
    if nargin == 3 && ~isempty(varargin{2})
        options = varargin{2};
    end
    validateattributes(safetyMargin_deg, {'numeric'}, {'scalar', 'real', 'finite', 'nonnegative'});
    verbose = resolveVerbose(options);
    obstacleData = obstacleAvoidance.obstacles.combineObstacles(obstacleInput);
    obstacleData = protectObstacles(obstacleData, safetyMargin_deg, verbose);
    return;
end
if nargin < 4 || nargin > 6
    error("createObstacle:InvalidCall", ...
        "Construction requires name, time, azimuth, and elevation.");
end

%% Section 2: Create And Protect One Raw Record

time_s = double(varargin{1}(:));
azimuthBySlice_deg = varargin{2};
elevationBySlice_deg = varargin{3};
safetyMargin_deg = 0;
options = struct();
if nargin >= 5 && ~isempty(varargin{4})
    safetyMargin_deg = varargin{4};
end
if nargin == 6 && ~isempty(varargin{5})
    options = varargin{5};
end
validateattributes(safetyMargin_deg, {'numeric'}, {'scalar', 'real', 'finite', 'nonnegative'});
sampleCount = numel(time_s);
if ~iscell(azimuthBySlice_deg)
    azimuthBySlice_deg = repmat({double(azimuthBySlice_deg(:))}, sampleCount, 1);
end
if ~iscell(elevationBySlice_deg)
    elevationBySlice_deg = repmat({double(elevationBySlice_deg(:))}, sampleCount, 1);
end
rawObstacle = struct("targetName", string(obstacleInput), "time_s", time_s, ...
    "az_deg", {reshape(azimuthBySlice_deg, [], 1)}, ...
    "el_deg", {reshape(elevationBySlice_deg, [], 1)}, ...
    "originalAz_deg", {reshape(azimuthBySlice_deg, [], 1)}, ...
    "originalEl_deg", {reshape(elevationBySlice_deg, [], 1)}, "safetyMargin_deg", 0, ...
    "status", repmat("visible", sampleCount, 1));
obstacleData = normalizeOne(rawObstacle);
obstacleData = protectObstacles(obstacleData, safetyMargin_deg, resolveVerbose(options));
end

function obstacle = normalizeOne(inputData)
% Validate and column-normalize one canonical record while dropping stale cache.
requiredFields = {'targetName', 'time_s', 'az_deg', 'el_deg', 'status'};
requireCondition(isstruct(inputData) && isscalar(inputData) && ...
    all(isfield(inputData, requiredFields)), "createObstacle:InvalidInput", ...
    "obstacleData must be one canonical obstacle record.");
targetName = string(inputData.targetName);
requireCondition(isscalar(targetName) && strlength(strtrim(targetName)) > 0, ...
    "createObstacle:InvalidTargetName", "targetName must be nonempty scalar text.");
validateattributes(inputData.time_s, {'numeric'}, {'vector', 'real', 'finite'});
time_s = double(inputData.time_s(:));
sampleCount = numel(time_s);
requireCondition(sampleCount > 0 && all(diff(time_s) > 0), ...
    "createObstacle:InvalidTime", "time_s must be nonempty and strictly increasing.");
boundariesAreValid = iscell(inputData.az_deg) && iscell(inputData.el_deg) && ...
    numel(inputData.az_deg) == sampleCount && numel(inputData.el_deg) == sampleCount;
requireCondition(boundariesAreValid, "createObstacle:InvalidBoundary", ...
    "az_deg and el_deg must be cell arrays matching time_s.");
[azimuthBySlice_deg, elevationBySlice_deg, ...
    protectedRemoved, protectedRemovalBySample] = normalizeHistory( ...
    inputData.az_deg, inputData.el_deg, sampleCount, "protected");
hasOriginalAzimuth = isfield(inputData, "originalAz_deg");
hasOriginalElevation = isfield(inputData, "originalEl_deg");
requireCondition(~xor(hasOriginalAzimuth, hasOriginalElevation), ...
    "createObstacle:IncompleteOriginalBoundary", ...
    "originalAz_deg and originalEl_deg must both be present or absent.");
if hasOriginalAzimuth
    originalIsValid = iscell(inputData.originalAz_deg) && iscell(inputData.originalEl_deg) && ...
        numel(inputData.originalAz_deg) == sampleCount && ...
        numel(inputData.originalEl_deg) == sampleCount;
    requireCondition(originalIsValid, "createObstacle:InvalidOriginalBoundary", ...
        "Original boundary cells must match time_s.");
    [originalAzimuthBySlice_deg, originalElevationBySlice_deg, ...
        originalRemoved, originalRemovalBySample] = normalizeHistory( ...
        inputData.originalAz_deg, inputData.originalEl_deg, sampleCount, "original");
else
    originalAzimuthBySlice_deg = azimuthBySlice_deg;
    originalElevationBySlice_deg = elevationBySlice_deg;
    originalRemoved = 0;
    originalRemovalBySample = false(sampleCount, 1);
end
if protectedRemoved + originalRemoved > 0
    removalBySample = protectedRemovalBySample | originalRemovalBySample;
    warning("createObstacle:RemovedTwoVertexRegions", ...
        "Obstacle '%s' removed %d protected and %d original two-vertex " + ...
        "regions across %d time slices; remaining regions were unchanged.", ...
        targetName, protectedRemoved, originalRemoved, nnz(removalBySample));
end
safetyMargin_deg = 0;
if isfield(inputData, "safetyMargin_deg")
    safetyMargin_deg = inputData.safetyMargin_deg;
end
validateattributes(safetyMargin_deg, {'numeric'}, {'scalar', 'real', 'finite', 'nonnegative'});
safetyMargin_deg = double(safetyMargin_deg);
requireCondition(safetyMargin_deg == 0 || hasOriginalAzimuth, ...
    "createObstacle:MissingOriginalBoundary", ...
    "Positive safetyMargin_deg requires retained original boundaries.");
status = string(inputData.status);
if isscalar(status)
    status = repmat(status, sampleCount, 1);
elseif numel(status) == sampleCount
    status = status(:);
else
    error("createObstacle:StatusSizeMismatch", "status must contain one value per time sample.");
end
obstacle = struct("targetName", targetName, "time_s", time_s, ...
    "az_deg", {azimuthBySlice_deg}, "el_deg", {elevationBySlice_deg}, ...
    "originalAz_deg", {originalAzimuthBySlice_deg}, ...
    "originalEl_deg", {originalElevationBySlice_deg}, ...
    "safetyMargin_deg", safetyMargin_deg, "status", status);
end

function [azimuthHistory_deg, elevationHistory_deg, removedCount, ...
        removalBySample] = normalizeHistory( ...
        azimuthInput_deg, elevationInput_deg, sampleCount, role)
% Normalize every protected or original slice with role-specific mismatch IDs.
azimuthHistory_deg = reshape(azimuthInput_deg, [], 1);
elevationHistory_deg = reshape(elevationInput_deg, [], 1);
removedCount = 0;
removalBySample = false(sampleCount, 1);
identifiers = ["createObstacle:BoundarySizeMismatch", ...
    "createObstacle:OriginalBoundarySizeMismatch"];
fieldNames = ["az_deg", "el_deg"; "originalAz_deg", "originalEl_deg"];
roleIndex = 1 + (role == "original");
for sampleIndex = 1:sampleCount
    validateattributes(azimuthHistory_deg{sampleIndex}, {'numeric'}, {'vector', 'real'});
    validateattributes(elevationHistory_deg{sampleIndex}, {'numeric'}, {'vector', 'real'});
    if numel(azimuthHistory_deg{sampleIndex}) ~= numel(elevationHistory_deg{sampleIndex})
        error(identifiers(roleIndex), "%s and %s slice %d must have equal lengths.", ...
            fieldNames(roleIndex, 1), fieldNames(roleIndex, 2), sampleIndex);
    end
    azimuth_deg = double(azimuthHistory_deg{sampleIndex}(:));
    elevation_deg = double(elevationHistory_deg{sampleIndex}(:));
    [azimuth_deg, elevation_deg, removed] = ...
        normalizeSlice(azimuth_deg, elevation_deg, sampleIndex, role);
    azimuthHistory_deg{sampleIndex} = azimuth_deg;
    elevationHistory_deg{sampleIndex} = elevation_deg;
    removedCount = removedCount + removed;
    removalBySample(sampleIndex) = removed > 0;
end
end

function [azimuth_deg, elevation_deg, removedCount] = ...
        normalizeSlice(azimuth_deg, elevation_deg, sampleIndex, role)
% Reject malformed rings, remove only zero-area two-vertex regions, keep order.
azimuthFinite = isfinite(azimuth_deg);
elevationFinite = isfinite(elevation_deg);
requireCondition(~any(xor(azimuthFinite, elevationFinite)), ...
    "createObstacle:UnpairedNonfiniteBoundary", ...
    "The %s boundary at slice %d must use paired separators.", role, sampleIndex);
changes = diff([false; azimuthFinite; false]);
regionStarts = find(changes == 1);
regionStops = find(changes == -1) - 1;
regionVertexCount = regionStops - regionStarts + 1;
oneVertexRegion = find(regionVertexCount == 1, 1, "first");
requireCondition(isempty(oneVertexRegion), "createObstacle:BoundaryRingTooShort", ...
    "The %s boundary region %d at slice %d has one finite vertex.", ...
    role, oneVertexRegion, sampleIndex);
removeRegion = regionVertexCount == 2;
removedCount = nnz(removeRegion);
if removedCount == 0
    return;
end
retainedRegions = find(~removeRegion);
if isempty(retainedRegions)
    azimuth_deg = zeros(0, 1);
    elevation_deg = zeros(0, 1);
    return;
end
outputCount = sum(regionVertexCount(retainedRegions)) + numel(retainedRegions) - 1;
newAzimuth_deg = NaN(outputCount, 1);
newElevation_deg = NaN(outputCount, 1);
writeIndex = 1;
for retainedIndex = 1:numel(retainedRegions)
    regionIndex = retainedRegions(retainedIndex);
    inputRows = regionStarts(regionIndex):regionStops(regionIndex);
    outputRows = writeIndex + (0:numel(inputRows) - 1);
    newAzimuth_deg(outputRows) = azimuth_deg(inputRows);
    newElevation_deg(outputRows) = elevation_deg(inputRows);
    writeIndex = outputRows(end) + 2;
end
azimuth_deg = newAzimuth_deg;
elevation_deg = newElevation_deg;
end

function verbose = resolveVerbose(options)
% Resolve the one construction option and warn once about ignored fields.
requireCondition(isstruct(options) && isscalar(options), ...
    "createObstacle:InvalidProtectionOptions", "options must be a scalar struct.");
[options, unknownNames] = obstacleAvoidance.input.resolveOptions( ...
    struct("Verbose", false), options);
if ~isempty(unknownNames)
    warning("createObstacle:UnknownProtectionOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", strjoin(unknownNames, ", "));
end
verbose = obstacleAvoidance.input.normalizeLogicalScalar( ...
    options.Verbose, "Verbose", "createObstacle:InvalidVerbose");
end

function obstacles = protectObstacles(obstacles, safetyMargin_deg, verbose)
% Rebuild each history; futures resize once per obstacle above 500,000 vertices.
for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacles(obstacleIndex);
    sampleCount = numel(obstacle.time_s);
    protectedAzimuth_deg = cell(sampleCount, 1);
    protectedElevation_deg = cell(sampleCount, 1);
    vertexCount = numel(vertcat(obstacle.originalAz_deg{:}));
    useBackgroundWorkers = false;
    if safetyMargin_deg > 0 && vertexCount >= 500000 && exist("backgroundPool", "builtin") == 5
        workerPool = backgroundPool;
        useBackgroundWorkers = workerPool.NumWorkers > 1 && ~workerPool.Busy;
    end
    if useBackgroundWorkers
        futures(1, sampleCount) = parallel.FevalFuture; %#ok<AGROW>
        for sampleIndex = 1:sampleCount
            futures(sampleIndex) = parfeval( ...
                workerPool, @inflateSlice, 2, ...
                obstacle.originalAz_deg{sampleIndex}, ...
                obstacle.originalEl_deg{sampleIndex}, safetyMargin_deg);
        end
        [protectedAzimuth_deg, protectedElevation_deg] = ...
            fetchOutputs(futures, "UniformOutput", false);
    else
        for sampleIndex = 1:sampleCount
            [protectedAzimuth_deg{sampleIndex}, ...
                protectedElevation_deg{sampleIndex}] = inflateSlice( ...
                obstacle.originalAz_deg{sampleIndex}, ...
                obstacle.originalEl_deg{sampleIndex}, safetyMargin_deg);
        end
    end
    if verbose
        fprintf("[az/el protect] obstacle %d/%d: %d slices complete.\n", ...
            obstacleIndex, numel(obstacles), sampleCount);
    end
    obstacle.az_deg = protectedAzimuth_deg;
    obstacle.el_deg = protectedElevation_deg;
    obstacle.safetyMargin_deg = double(safetyMargin_deg);
    obstacles(obstacleIndex) = normalizeOne(obstacle);
end
end

function [protectedAzimuth_deg, protectedElevation_deg] = ...
        inflateSlice(azimuth_deg, elevation_deg, safetyMargin_deg)
% Apply one square-joint outward buffer without reordering zero-margin input.
azimuth_deg = double(azimuth_deg(:));
elevation_deg = double(elevation_deg(:));
if safetyMargin_deg == 0
    protectedAzimuth_deg = azimuth_deg;
    protectedElevation_deg = elevation_deg;
    return;
end
azimuth_deg(~isfinite(azimuth_deg)) = NaN;
elevation_deg(~isfinite(elevation_deg)) = NaN;
if nnz(isfinite(azimuth_deg) & isfinite(elevation_deg)) < 3
    protectedAzimuth_deg = zeros(0, 1);
    protectedElevation_deg = zeros(0, 1);
    return;
end
sourceShape = polyshape(azimuth_deg, elevation_deg, "Simplify", true, "KeepCollinearPoints", true);
requireCondition(~isempty(sourceShape.Vertices) && area(sourceShape) > 0, ...
    "createObstacle:DegeneratePolygon", ...
    "The boundary slice does not define a nonzero-area polygon.");
protectedShape = polybuffer(sourceShape, safetyMargin_deg, "JointType", "square");
[protectedAzimuth_deg, protectedElevation_deg] = boundary(protectedShape);
protectedAzimuth_deg = double(protectedAzimuth_deg(:));
protectedElevation_deg = double(protectedElevation_deg(:));
lastFinite = find(isfinite(protectedAzimuth_deg) & isfinite(protectedElevation_deg), 1, "last");
protectedAzimuth_deg = protectedAzimuth_deg(1:lastFinite);
protectedElevation_deg = protectedElevation_deg(1:lastFinite);
end

function requireCondition(condition, identifier, message, varargin)
% Throw one stable public diagnostic when a structural invariant fails.
if ~condition
    error(identifier, message, varargin{:});
end
end
