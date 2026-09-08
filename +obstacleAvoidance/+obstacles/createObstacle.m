function obstacleData = createObstacle(obstacleInput, varargin)
%% Section 0: Header & Readme
% SYNTAX
%   obstacleData = obstacleAvoidance.obstacles.createObstacle( ...
%       obstacleName, time_s, xBoundary_units, yBoundary_units)
%   obstacleData = obstacleAvoidance.obstacles.createObstacle( ...
%       obstacleName, time_s, xBoundary_units, ...
%       yBoundary_units, safetyMargin_units)
%   obstacleData = obstacleAvoidance.obstacles.createObstacle( ...
%       obstacleName, time_s, xBoundary_units, ...
%       yBoundary_units, safetyMargin_units, constructionOptions)
%   obstacleData = obstacleAvoidance.obstacles.createObstacle(canonicalObstacle)
%   obstacleData = obstacleAvoidance.obstacles.createObstacle( ...
%       canonicalObstacles, safetyMargin_units)
%   obstacleData = obstacleAvoidance.obstacles.createObstacle( ...
%       canonicalObstacles, safetyMargin_units, constructionOptions)
%
% PURPOSE
%   - Own canonical obstacle construction and normalization.
%   - Rebuild protected histories from retained original geometry so an
%     absolute safety margin is applied exactly once.
%
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
%
% OUTPUTS
%   - obstacleData (canonical scalar or column struct array)
%       Original and protected histories, margin, status, and stable fields.
%
% UNITS
%   - Boundary coordinates and safety margins are coordinate units; time is seconds.
%   - See obstacle_history_contract.md for between-sample semantics.
%

%% Section 1: Select Construction Or Canonical Rebuild

if nargin == 0
    error("createObstacle:MissingInput", "Obstacle construction or canonical input is required.");
end
isContainer = isstruct(obstacleInput) || iscell(obstacleInput) || (isnumeric(obstacleInput) && isempty(obstacleInput));
if isContainer && nargin == 1
    obstacleData = normalizeOne(obstacleInput);
    return;
elseif isContainer && nargin >= 2 && nargin <= 3
    safetyMargin_units = varargin{1};
    options          = struct();
    if nargin == 3 && ~isempty(varargin{2})
        options = varargin{2};
    end
    validateattributes(safetyMargin_units, {'numeric'}, {'scalar', 'real', 'finite', 'nonnegative'});
    verbose      = resolveVerbose(options);
    obstacleData = obstacleAvoidance.obstacles.combineObstacles(obstacleInput);
    obstacleData = protectObstacles(obstacleData, safetyMargin_units, verbose);
    return;
end
if nargin < 4 || nargin > 6
    error("createObstacle:InvalidCall", "Construction requires name, time, x, and y.");
end

%% Section 2: Create And Protect One Raw Record

time_s               = double(varargin{1}(:));
xBySlice_units   = varargin{2};
yBySlice_units = varargin{3};
safetyMargin_units     = 0;
options              = struct();
if nargin >= 5 && ~isempty(varargin{4})
    safetyMargin_units = varargin{4};
end
if nargin == 6 && ~isempty(varargin{5})
    options = varargin{5};
end
validateattributes(safetyMargin_units, {'numeric'}, {'scalar', 'real', 'finite', 'nonnegative'});
sampleCount = numel(time_s);
if ~iscell(xBySlice_units)
    xBySlice_units = repmat({double(xBySlice_units(:))}, sampleCount, 1);
end
if ~iscell(yBySlice_units)
    yBySlice_units = repmat({double(yBySlice_units(:))}, sampleCount, 1);
end
rawObstacle = struct("targetName", string(obstacleInput), "time_s", time_s, ...
    "x_units", {reshape(xBySlice_units, [], 1)}, ...
    "y_units", {reshape(yBySlice_units, [], 1)}, ...
    "originalX_units", {reshape(xBySlice_units, [], 1)}, ...
    "originalY_units", {reshape(yBySlice_units, [], 1)}, "safetyMargin_units", 0, ...
    "status", repmat("visible", sampleCount, 1));
obstacleData = normalizeOne(rawObstacle);
obstacleData = protectObstacles(obstacleData, safetyMargin_units, resolveVerbose(options));
end

function obstacle = normalizeOne(inputData)
    % Validate the obstacle, use column vectors, and discard stale caches.
    requiredFields = {'targetName', 'time_s', 'x_units', 'y_units', 'status'};
    requireCondition(isstruct(inputData) && isscalar(inputData) && all(isfield(inputData, requiredFields)), "createObstacle:InvalidInput", "obstacleData must be one canonical obstacle record.");
    targetName = string(inputData.targetName);
    requireCondition(isscalar(targetName) && strlength(strtrim(targetName)) > 0, "createObstacle:InvalidTargetName", "targetName must be nonempty scalar text.");
    validateattributes(inputData.time_s, {'numeric'}, {'vector', 'real', 'finite'});
    time_s      = double(inputData.time_s(:));
    sampleCount = numel(time_s);
    requireCondition(sampleCount > 0 && all(diff(time_s) > 0), "createObstacle:InvalidTime", "time_s must be nonempty and strictly increasing.");
    boundariesAreValid = iscell(inputData.x_units) && iscell(inputData.y_units) && numel(inputData.x_units) == sampleCount && numel(inputData.y_units) == sampleCount;
    requireCondition(boundariesAreValid, "createObstacle:InvalidBoundary", "x_units and y_units must be cell arrays matching time_s.");
    [xBySlice_units, yBySlice_units, ...
        protectedRemoved, protectedRemovalBySample] = normalizeHistory(inputData.x_units, inputData.y_units, sampleCount, "protected");
    hasOriginalX   = isfield(inputData, "originalX_units");
    hasOriginalY = isfield(inputData, "originalY_units");
    requireCondition(~xor(hasOriginalX, hasOriginalY), "createObstacle:IncompleteOriginalBoundary", "originalX_units and originalY_units must both be present or absent.");
    if hasOriginalX
        originalIsValid = iscell(inputData.originalX_units) && iscell(inputData.originalY_units) && numel(inputData.originalX_units) == sampleCount && numel(inputData.originalY_units) == sampleCount;
        requireCondition(originalIsValid, "createObstacle:InvalidOriginalBoundary", "Original boundary cells must match time_s.");
        [originalXBySlice_units, originalYBySlice_units, ...
            originalRemoved, originalRemovalBySample] = normalizeHistory(inputData.originalX_units, inputData.originalY_units, sampleCount, "original");
    else
        originalXBySlice_units   = xBySlice_units;
        originalYBySlice_units = yBySlice_units;
        originalRemoved              = 0;
        originalRemovalBySample      = false(sampleCount, 1);
    end
    if protectedRemoved + originalRemoved > 0
        removalBySample = protectedRemovalBySample | originalRemovalBySample;
        warning("createObstacle:RemovedTwoVertexRegions", "Obstacle '%s' removed %d protected and %d original two-vertex " + "regions across %d time slices; remaining regions were unchanged.", targetName, protectedRemoved, originalRemoved, nnz(removalBySample));
    end
    safetyMargin_units = 0;
    if isfield(inputData, "safetyMargin_units")
        safetyMargin_units = inputData.safetyMargin_units;
    end
    validateattributes(safetyMargin_units, {'numeric'}, {'scalar', 'real', 'finite', 'nonnegative'});
    safetyMargin_units = double(safetyMargin_units);
    requireCondition(safetyMargin_units == 0 || hasOriginalX, "createObstacle:MissingOriginalBoundary", "Positive safetyMargin_units requires retained original boundaries.");
    status = string(inputData.status);
    if isscalar(status)
        status = repmat(status, sampleCount, 1);
    elseif numel(status) == sampleCount
        status = status(:);
    else
        error("createObstacle:StatusSizeMismatch", "status must contain one value per time sample.");
    end
    obstacle = struct("targetName", targetName, "time_s", time_s, ...
        "x_units", {xBySlice_units}, "y_units", {yBySlice_units}, ...
        "originalX_units", {originalXBySlice_units}, ...
        "originalY_units", {originalYBySlice_units}, ...
        "safetyMargin_units", safetyMargin_units, "status", status);
end

function [xHistory_units, yHistory_units, removedCount, removalBySample] = normalizeHistory(xInput_units, yInput_units, sampleCount, role)
    % Normalize original and protected slices with distinct error identifiers.
    xHistory_units   = reshape(xInput_units, [], 1);
    yHistory_units = reshape(yInput_units, [], 1);
    removedCount         = 0;
    removalBySample      = false(sampleCount, 1);
    identifiers          = ["createObstacle:BoundarySizeMismatch", ...
        "createObstacle:OriginalBoundarySizeMismatch"];
    fieldNames = ["x_units", "y_units"; "originalX_units", "originalY_units"];
    roleIndex  = 1 + (role == "original");
    % Process each sample in temporal order and accumulate its result.
    for sampleIndex = 1:sampleCount
        validateattributes(xHistory_units{sampleIndex}, {'numeric'}, {'vector', 'real'});
        validateattributes(yHistory_units{sampleIndex}, {'numeric'}, {'vector', 'real'});
        if numel(xHistory_units{sampleIndex}) ~= numel(yHistory_units{sampleIndex})
            error(identifiers(roleIndex), "%s and %s slice %d must have equal lengths.", fieldNames(roleIndex, 1), fieldNames(roleIndex, 2), sampleIndex);
        end
        x_units   = double(xHistory_units{sampleIndex}(:));
        y_units = double(yHistory_units{sampleIndex}(:));
        [x_units, y_units, removed] = normalizeSlice(x_units, y_units, sampleIndex, role);
        xHistory_units{sampleIndex} = x_units;
        yHistory_units{sampleIndex} = y_units;
        removedCount = removedCount + removed;
        removalBySample(sampleIndex) = removed > 0;
    end
end

function [x_units, y_units, removedCount] = normalizeSlice(x_units, y_units, sampleIndex, role)
    % Reject malformed rings; remove two-vertex regions with no area.
    xFinite   = isfinite(x_units);
    yFinite = isfinite(y_units);
    requireCondition(~any(xor(xFinite, yFinite)), "createObstacle:UnpairedNonfiniteBoundary", "The %s boundary at slice %d must use paired separators.", role, sampleIndex);
    changes           = diff([false; xFinite; false]);
    regionStarts      = find(changes == 1);
    regionStops       = find(changes == -1) - 1;
    regionVertexCount = regionStops - regionStarts + 1;
    oneVertexRegion   = find(regionVertexCount == 1, 1, "first");
    requireCondition(isempty(oneVertexRegion), "createObstacle:BoundaryRingTooShort", "The %s boundary region %d at slice %d has one finite vertex.", role, oneVertexRegion, sampleIndex);
    removeRegion = regionVertexCount == 2;
    removedCount = nnz(removeRegion);
    if removedCount == 0
        return;
    end
    retainedRegions = find(~removeRegion);
    if isempty(retainedRegions)
        x_units   = zeros(0, 1);
        y_units = zeros(0, 1);
        return;
    end
    outputCount      = sum(regionVertexCount(retainedRegions)) + numel(retainedRegions) - 1;
    newX_units   = NaN(outputCount, 1);
    newY_units = NaN(outputCount, 1);
    writeIndex       = 1;
    % Process each retained needed to prepare slice.
    for retainedIndex = 1:numel(retainedRegions)
        regionIndex = retainedRegions(retainedIndex);
        inputRows   = regionStarts(regionIndex):regionStops(regionIndex);
        outputRows  = writeIndex + (0:numel(inputRows) - 1);
        newX_units(outputRows) = x_units(inputRows);
        newY_units(outputRows) = y_units(inputRows);
        writeIndex = outputRows(end) + 2;
    end
    x_units   = newX_units;
    y_units = newY_units;
end

function verbose = resolveVerbose(options)
    % Resolve construction options and warn about unknown fields.
    requireCondition(isstruct(options) && isscalar(options), "createObstacle:InvalidProtectionOptions", "options must be a scalar struct.");
    [options, unknownNames] = obstacleAvoidance.input.resolveOptions(struct("Verbose", false), options);
    if ~isempty(unknownNames)
        warning("createObstacle:UnknownProtectionOptions", "Ignoring unknown option fields: %s. No behavior changed.", strjoin(unknownNames, ", "));
    end
    verbose = obstacleAvoidance.input.normalizeLogicalScalar(options.Verbose, "Verbose", "createObstacle:InvalidVerbose");
end

function obstacles = protectObstacles(obstacles, safetyMargin_units, verbose)
    % Buffer large histories in parallel when background workers are available.
    for obstacleIndex = 1:numel(obstacles)
        obstacle               = obstacles(obstacleIndex);
        sampleCount            = numel(obstacle.time_s);
        protectedX_units   = cell(sampleCount, 1);
        protectedY_units = cell(sampleCount, 1);
        vertexCount            = numel(vertcat(obstacle.originalX_units{:}));
        useBackgroundWorkers   = false;
        if safetyMargin_units > 0 && vertexCount >= 500000 && exist("backgroundPool", "builtin") == 5
            workerPool           = backgroundPool;
            useBackgroundWorkers = workerPool.NumWorkers > 1 && ~workerPool.Busy;
        end
        if useBackgroundWorkers
            futures(1, sampleCount) = parallel.FevalFuture; %#ok<AGROW>
            % Process each sample in temporal order and accumulate its result.
            for sampleIndex = 1:sampleCount
                futures(sampleIndex) = parfeval(workerPool, @inflateSlice, 2, obstacle.originalX_units{sampleIndex}, obstacle.originalY_units{sampleIndex}, safetyMargin_units);
            end
            [protectedX_units, protectedY_units] = fetchOutputs(futures, "UniformOutput", false);
        else
            % Process each sample in temporal order and accumulate its result.
            for sampleIndex = 1:sampleCount
                [protectedX_units{sampleIndex}, ...
                    protectedY_units{sampleIndex}] = inflateSlice(obstacle.originalX_units{sampleIndex}, obstacle.originalY_units{sampleIndex}, safetyMargin_units);
            end
        end
        if verbose
            fprintf("[x/y protect] obstacle %d/%d: %d slices complete.\n", obstacleIndex, numel(obstacles), sampleCount);
        end
        obstacle.x_units           = protectedX_units;
        obstacle.y_units           = protectedY_units;
        obstacle.safetyMargin_units = double(safetyMargin_units);
        obstacles(obstacleIndex) = normalizeOne(obstacle);
    end
end

function [protectedX_units, protectedY_units] = inflateSlice(x_units, y_units, safetyMargin_units)
    % Apply the margin with square joins; preserve order when the margin is zero.
    x_units   = double(x_units(:));
    y_units = double(y_units(:));
    if safetyMargin_units == 0
        protectedX_units   = x_units;
        protectedY_units = y_units;
        return;
    end
    x_units(~isfinite(x_units)) = NaN;
    y_units(~isfinite(y_units)) = NaN;
    if nnz(isfinite(x_units) & isfinite(y_units)) < 3
        protectedX_units   = zeros(0, 1);
        protectedY_units = zeros(0, 1);
        return;
    end
    sourceShape = polyshape(x_units, y_units, "Simplify", true, "KeepCollinearPoints", true);
    requireCondition(~isempty(sourceShape.Vertices) && area(sourceShape) > 0, "createObstacle:DegeneratePolygon", "The boundary slice does not define a nonzero-area polygon.");
    protectedShape = polybuffer(sourceShape, safetyMargin_units, "JointType", "square");
    [protectedX_units, protectedY_units] = boundary(protectedShape);
    protectedX_units   = double(protectedX_units(:));
    protectedY_units = double(protectedY_units(:));
    lastFinite             = find(isfinite(protectedX_units) & isfinite(protectedY_units), 1, "last");
    protectedX_units   = protectedX_units(1:lastFinite);
    protectedY_units = protectedY_units(1:lastFinite);
end

function requireCondition(condition, identifier, message, varargin)
    % Report invalid input with the supplied error identifier.
    if ~condition
        error(identifier, message, varargin{:});
    end
end
