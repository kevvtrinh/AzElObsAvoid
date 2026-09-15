function obstacleData = createObstacle(obstacleInput, varargin)
%% Section 0: Header & Readme
% SYNTAX
%   obstacleData = obstacleAvoidance.obstacles.createObstacle(canonicalObstacle)
%   obstacleData = obstacleAvoidance.obstacles.createObstacle( ...
%       canonicalObstacles, safetyMargin_units, constructionOptions)
%   obstacleData = obstacleAvoidance.obstacles.createObstacle( ...
%       obstacleName, time_s, xBoundary_units, yBoundary_units)
%   obstacleData = obstacleAvoidance.obstacles.createObstacle( ...
%       obstacleName, time_s, xBoundary_units, yBoundary_units, ...
%       safetyMargin_units, constructionOptions)
%**************************************************************************
% PURPOSE
%   - Construct and normalize canonical protected obstacle histories.
%   - Rebuild protection from original geometry so the margin applies once.
%**************************************************************************
% INPUTS
%   - obstacleInput (scalar text or canonical obstacle container)
%       Obstacle name for raw construction, or canonical data to normalize.
%   - time_s (numeric vector)
%       Strictly increasing raw-history sample times.
%   - xBoundary_units (numeric vector or cell array)
%       Raw boundary x-coordinates for one or more time samples.
%   - yBoundary_units (numeric vector or cell array)
%       Raw boundary y-coordinates matching xBoundary_units.
%   - safetyMargin_units (nonnegative numeric scalar, optional; default 0)
%       Protection margin rebuilt from the retained original geometry.
%   - constructionOptions (scalar struct, optional; default struct())
%       Verbose applies to both construction paths and defaults to false.
%       vertexCorrespondence applies only to raw construction and defaults
%       to circularCorrelation. Canonical records retain their declaration.
%**************************************************************************
% OUTPUTS
%   - obstacleData (canonical scalar or column struct array)
%       Original and protected histories plus normalization diagnostics.
%       Invalid input throws an error.
%**************************************************************************
% UNITS
%   - Boundary and margin use coordinate units; time uses seconds.
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
    safetyMargin_units = varargin{1};
    options = struct();
    if nargin == 3 && ~isempty(varargin{2})
        options = varargin{2};
    end
    validateattributes(safetyMargin_units, {'numeric'}, {'scalar', 'real', 'finite', 'nonnegative'});
    [verbose, ~] = resolveConstructionOptions(options, false);
    obstacleData = obstacleAvoidance.obstacles.combineObstacles(obstacleInput);
    obstacleData = protectObstacles(obstacleData, safetyMargin_units, verbose);
    return;
end
if nargin < 4 || nargin > 6
    error("createObstacle:InvalidCall", "Construction requires name, time, x, and y.");
end

%% Section 2: Create And Protect One Raw Record

time_s             = double(varargin{1}(:));
xBySlice_units     = varargin{2};
yBySlice_units     = varargin{3};
safetyMargin_units = 0;
options            = struct();
if nargin >= 5 && ~isempty(varargin{4})
    safetyMargin_units = varargin{4};
end
if nargin == 6 && ~isempty(varargin{5})
    options = varargin{5};
end
validateattributes(safetyMargin_units, {'numeric'}, {'scalar', 'real', 'finite', 'nonnegative'});
[verbose, vertexCorrespondence] = resolveConstructionOptions(options, true);
sampleCount = numel(time_s);
if ~iscell(xBySlice_units)
    xBySlice_units = repmat({double(xBySlice_units(:))}, sampleCount, 1);
end
if ~iscell(yBySlice_units)
    yBySlice_units = repmat({double(yBySlice_units(:))}, sampleCount, 1);
end
rawObstacle = struct( ...
    "targetName",           string(obstacleInput), ...
    "time_s",               time_s, ...
    "x_units",              {reshape(xBySlice_units, [], 1)}, ...
    "y_units",              {reshape(yBySlice_units, [], 1)}, ...
    "originalX_units",      {reshape(xBySlice_units, [], 1)}, ...
    "originalY_units",      {reshape(yBySlice_units, [], 1)}, ...
    "safetyMargin_units",   0, ...
    "status",               repmat("visible", sampleCount, 1), ...
    "vertexCorrespondence", vertexCorrespondence);
obstacleData = normalizeOne(rawObstacle);
obstacleData = protectObstacles(obstacleData, safetyMargin_units, verbose);
end

%% Section 3: Local Functions

function obstacle = normalizeOne(inputData)
    % Validate the obstacle, use column vectors, and discard stale caches.
    requiredFields = {'targetName', 'time_s', 'x_units', 'y_units', 'status'};
    inputIsCanonical = isstruct(inputData) && isscalar(inputData) && ...
        all(isfield(inputData, requiredFields));
    requireCondition(inputIsCanonical, "createObstacle:InvalidInput", ...
        "obstacleData must be one canonical obstacle record.");
    targetName = string(inputData.targetName);
    requireCondition(isscalar(targetName) && strlength(strtrim(targetName)) > 0, ...
        "createObstacle:InvalidTargetName", "targetName must be nonempty scalar text.");
    validateattributes(inputData.time_s, {'numeric'}, {'vector', 'real', 'finite'});
    time_s      = double(inputData.time_s(:));
    sampleCount = numel(time_s);
    requireCondition(sampleCount > 0 && all(diff(time_s) > 0), ...
        "createObstacle:InvalidTime", "time_s must be nonempty and strictly increasing.");
    xHistoryIsValid = iscell(inputData.x_units) && numel(inputData.x_units) == sampleCount;
    yHistoryIsValid = iscell(inputData.y_units) && numel(inputData.y_units) == sampleCount;
    requireCondition(xHistoryIsValid && yHistoryIsValid, "createObstacle:InvalidBoundary", ...
        "x_units and y_units must be cell arrays matching time_s.");
    [xBySlice_units, yBySlice_units, protectedRemoved, ...
        protectedRemovalBySample, protectedRepairBySample] = normalizeHistory( ...
        inputData.x_units, inputData.y_units, sampleCount, "protected");
    hasOriginalX = isfield(inputData, "originalX_units");
    hasOriginalY = isfield(inputData, "originalY_units");
    requireCondition(~xor(hasOriginalX, hasOriginalY), ...
        "createObstacle:IncompleteOriginalBoundary", ...
        "originalX_units and originalY_units must both be present or absent.");
    if hasOriginalX
        originalXIsValid = iscell(inputData.originalX_units) && ...
            numel(inputData.originalX_units) == sampleCount;
        originalYIsValid = iscell(inputData.originalY_units) && ...
            numel(inputData.originalY_units) == sampleCount;
        requireCondition(originalXIsValid && originalYIsValid, ...
            "createObstacle:InvalidOriginalBoundary", ...
            "Original boundary cells must match time_s.");
        protectedIsOriginal = isequaln(inputData.originalX_units, inputData.x_units) && ...
            isequaln(inputData.originalY_units, inputData.y_units);
        if protectedIsOriginal
            originalXBySlice_units   = xBySlice_units;
            originalYBySlice_units   = yBySlice_units;
            originalRemoved         = protectedRemoved;
            originalRemovalBySample = protectedRemovalBySample;
            originalRepairBySample  = protectedRepairBySample;
        else
            [originalXBySlice_units, originalYBySlice_units, originalRemoved, ...
                originalRemovalBySample, originalRepairBySample] = normalizeHistory( ...
                inputData.originalX_units, inputData.originalY_units, sampleCount, "original");
        end
    else
        originalXBySlice_units   = xBySlice_units;
        originalYBySlice_units   = yBySlice_units;
        originalRemoved         = [0, 0];
        originalRemovalBySample = false(sampleCount, 1);
        originalRepairBySample  = zeros(sampleCount, 3);
    end
    affectedSampleIndices = find(protectedRemovalBySample | originalRemovalBySample);
    normalization = struct( ...
        'Version',                           1, ...
        'SourceTime_s',                      time_s, ...
        'Roles',                             ["protected", "original"], ...
        'RemovedRegionCount',                [protectedRemoved(1), originalRemoved(1)], ...
        'RemovedDuplicateVertexCount',       [protectedRemoved(2), originalRemoved(2)], ...
        'AffectedSampleIndex',               affectedSampleIndices, ...
        'AffectedSampleTime_s',              time_s(affectedSampleIndices), ...
        'RemovedZigzagVertexCountBySample',  [protectedRepairBySample(:, 1), originalRepairBySample(:, 1)], ...
        'RemovedZigzagAreaBySample_units2',  [protectedRepairBySample(:, 2), originalRepairBySample(:, 2)], ...
        'AddedZigzagAreaBySample_units2',    [protectedRepairBySample(:, 3), originalRepairBySample(:, 3)], ...
        'Reasons',                           ["fewerThanThreeDistinctVertices", ...
                                              "exactDuplicateOrClosure", ...
                                              "selfCrossingZigzagRemoved"]);
    % Preserve cleanup provenance through canonical rebuilds and margin changes.
    % This metadata never controls occupancy or substitutes for source checks.
    if isfield(inputData, 'NormalizationDiagnostics')
        previous = inputData.NormalizationDiagnostics;
        requiredDiagnosticFields = {'Version', 'SourceTime_s', 'RemovedRegionCount', ...
            'RemovedDuplicateVertexCount', 'AffectedSampleIndex'};
        previousIsCompatible = isstruct(previous) && isscalar(previous) && ...
            all(isfield(previous, requiredDiagnosticFields)) && ...
            isequal(previous.Version, 1) && isequal(previous.SourceTime_s, time_s);
        if previousIsCompatible
            for name = ["RemovedRegionCount", "RemovedDuplicateVertexCount"]
                validateattributes(previous.(name), {'numeric'}, ...
                    {'real', 'finite', 'size', [1, 2], 'integer', 'nonnegative'});
                normalization.(name) = normalization.(name) + previous.(name);
            end
            repairFieldNames = ["RemovedZigzagVertexCountBySample", ...
                "RemovedZigzagAreaBySample_units2", "AddedZigzagAreaBySample_units2"];
            for name = repairFieldNames
                if isfield(previous, name)
                    validateattributes(previous.(name), {'numeric'}, ...
                        {'real', 'finite', 'size', [sampleCount, 2], 'nonnegative'});
                    normalization.(name) = normalization.(name) + previous.(name);
                end
            end
            validateattributes(previous.AffectedSampleIndex, {'numeric'}, ...
                {'real', 'finite', 'integer', 'positive', '<=', sampleCount});
            affectedSampleIndices = union(affectedSampleIndices, previous.AffectedSampleIndex(:));
            normalization.AffectedSampleIndex  = affectedSampleIndices;
            normalization.AffectedSampleTime_s = time_s(affectedSampleIndices);
        end
    end
    safetyMargin_units = 0;
    if isfield(inputData, "safetyMargin_units")
        safetyMargin_units = inputData.safetyMargin_units;
    end
    validateattributes(safetyMargin_units, {'numeric'}, {'scalar', 'real', 'finite', 'nonnegative'});
    safetyMargin_units = double(safetyMargin_units);
    requireCondition(safetyMargin_units == 0 || hasOriginalX, ...
        "createObstacle:MissingOriginalBoundary", ...
        "Positive safetyMargin_units requires retained original boundaries.");
    status = string(inputData.status);
    if isscalar(status)
        status = repmat(status, sampleCount, 1);
    elseif numel(status) == sampleCount
        status = status(:);
    else
        error("createObstacle:StatusSizeMismatch", "status must contain one value per time sample.");
    end
    % Normalize the reported declaration once so later geometry branches use
    % logical state rather than provenance text.
    vertexCorrespondence = "circularCorrelation";
    if isfield(inputData, "vertexCorrespondence") && ~isempty(inputData.vertexCorrespondence)
        vertexCorrespondence = string(inputData.vertexCorrespondence);
        requireCondition(isscalar(vertexCorrespondence) && ...
            any(vertexCorrespondence == ["circularCorrelation", "sourceIndex"]), ...
            "createObstacle:InvalidVertexCorrespondence", ...
            "vertexCorrespondence must be circularCorrelation or sourceIndex.");
    end
    usesSourceIndex = vertexCorrespondence == "sourceIndex";
    obstacle = struct( ...
        "targetName",               targetName, ...
        "time_s",                   time_s, ...
        "x_units",                  {xBySlice_units}, ...
        "y_units",                  {yBySlice_units}, ...
        "originalX_units",          {originalXBySlice_units}, ...
        "originalY_units",          {originalYBySlice_units}, ...
        "safetyMargin_units",       safetyMargin_units, ...
        "status",                   status, ...
        "NormalizationDiagnostics", normalization, ...
        "vertexCorrespondence",     vertexCorrespondence, ...
        "UsesSourceIndex",          usesSourceIndex);
end

function [xHistory_units, yHistory_units, removedCount, removalBySample, repairBySample] = ...
        normalizeHistory(xInput_units, yInput_units, sampleCount, role)
    % Normalize original and protected slices with distinct error identifiers.
    xHistory_units  = reshape(xInput_units, [], 1);
    yHistory_units  = reshape(yInput_units, [], 1);
    removedCount    = [0, 0];
    removalBySample = false(sampleCount, 1);
    repairBySample  = zeros(sampleCount, 3);
    identifiers     = ["createObstacle:BoundarySizeMismatch", ...
        "createObstacle:OriginalBoundarySizeMismatch"];
    fieldNames = ["x_units", "y_units"; "originalX_units", "originalY_units"];
    roleIndex  = 1 + (role == "original");
    for sampleIndex = 1:sampleCount
        validateattributes(xHistory_units{sampleIndex}, {'numeric'}, {'vector', 'real'});
        validateattributes(yHistory_units{sampleIndex}, {'numeric'}, {'vector', 'real'});
        if numel(xHistory_units{sampleIndex}) ~= numel(yHistory_units{sampleIndex})
            error(identifiers(roleIndex), "%s and %s slice %d must have equal lengths.", ...
                fieldNames(roleIndex, 1), fieldNames(roleIndex, 2), sampleIndex);
        end
        x_units = double(xHistory_units{sampleIndex}(:));
        y_units = double(yHistory_units{sampleIndex}(:));
        [x_units, y_units, removed, repair] = normalizeSlice(x_units, y_units, sampleIndex, role);
        xHistory_units{sampleIndex} = x_units;
        yHistory_units{sampleIndex} = y_units;
        removedCount = removedCount + removed;
        removalBySample(sampleIndex) = any(removed > 0) || repair(1) > 0;
        repairBySample(sampleIndex, :) = repair;
    end
end

function [x_units, y_units, removedCount, repair] = normalizeSlice( ...
        x_units, y_units, sampleIndex, role)
    % Normalize duplicates, then remove declared proper-crossing folds.
    xFinite = isfinite(x_units);
    yFinite = isfinite(y_units);
    requireCondition(~any(xor(xFinite, yFinite)), ...
        "createObstacle:UnpairedNonfiniteBoundary", ...
        "The %s boundary at slice %d must use paired separators.", role, sampleIndex);
    changes      = diff([false; xFinite; false]);
    regionStarts = find(changes == 1);
    regionStops  = find(changes == -1) - 1;
    rowsByRegion = cell(numel(regionStarts), 1);
    repair       = [0, 0, 0];
    removedCount = [0, 0];
    for regionIndex = 1:numel(regionStarts)
        rows = (regionStarts(regionIndex):regionStops(regionIndex)).';
        repeated = [false; diff(x_units(rows)) == 0 & diff(y_units(rows)) == 0];
        removedCount(2) = removedCount(2) + nnz(repeated);
        rows(repeated) = [];
        if numel(rows) > 1 && x_units(rows(1)) == x_units(rows(end)) && ...
                y_units(rows(1)) == y_units(rows(end))
            rows(end) = [];
            removedCount(2) = removedCount(2) + 1;
        end
        [retained, changedArea_units2] = removeCrossingZigzags([x_units(rows), y_units(rows)]);
        repair = repair + [numel(rows) - numel(retained), changedArea_units2];
        rows = rows(retained);
        hasAreaVertices = numel(rows) >= 3;
        if hasAreaVertices
            % The first two retained vertices differ. Alternating copies of
            % only those two points still enclose no area, without an area test.
            differsFromFirst = x_units(rows) ~= x_units(rows(1)) | y_units(rows) ~= y_units(rows(1));
            differsFromSecond = x_units(rows) ~= x_units(rows(2)) | y_units(rows) ~= y_units(rows(2));
            hasAreaVertices = any(differsFromFirst & differsFromSecond);
        end
        if hasAreaVertices
            rowsByRegion{regionIndex} = rows;
        else
            removedCount(1) = removedCount(1) + 1;
        end
    end
    if ~any(removedCount) && repair(1) == 0 && any(xFinite)
        return;
    end
    regionVertexCount = cellfun(@numel, rowsByRegion);
    retainedRegions = find(regionVertexCount > 0);
    if isempty(retainedRegions)
        x_units = zeros(0, 1);
        y_units = zeros(0, 1);
        return;
    end
    outputCount = sum(regionVertexCount(retainedRegions)) + numel(retainedRegions) - 1;
    newX_units  = NaN(outputCount, 1);
    newY_units  = NaN(outputCount, 1);
    writeIndex  = 1;
    for retainedIndex = 1:numel(retainedRegions)
        regionIndex = retainedRegions(retainedIndex);
        inputRows  = rowsByRegion{regionIndex};
        outputRows = writeIndex + (0:numel(inputRows) - 1);
        newX_units(outputRows) = x_units(inputRows);
        newY_units(outputRows) = y_units(inputRows);
        writeIndex = outputRows(end) + 2;
    end
    x_units = newX_units;
    y_units = newY_units;
end

function [retained, changedArea_units2] = removeCrossingZigzags(points_units)
    % Strict orientation signs identify proper crossings, without a tolerance.
    % Sorting edge boxes only omits pairs whose boxes are strictly disjoint.
    retained = (1:size(points_units, 1)).';
    changedArea_units2 = [0, 0];
    before = [];
    % Fast exit for rings that need no repair. A proper crossing always
    % introduces its intersection point into the simplified boundary, so a
    % single-region simplified ring whose vertices are exactly the supplied
    % ones (same count, all rows present) contains no proper crossing. This
    % filter only skips the scan; the scan below remains the authority.
    if size(points_units, 1) >= 3
        warningState = warning('off', 'MATLAB:polyshape:repairedBySimplify');
        restoreWarning = onCleanup(@()warning(warningState));
        simplified = polyshape(points_units, 'Simplify', true, 'KeepCollinearPoints', true);
        clear restoreWarning;
        if simplified.NumRegions == 1 && simplified.NumHoles == 0 && ...
                size(simplified.Vertices, 1) == size(points_units, 1) && ...
                all(ismember(simplified.Vertices, points_units, 'rows'))
            return;
        end
        before = simplified;
    end
    while numel(retained) >= 3
        vertices_units = points_units(retained, :);
        count          = size(vertices_units, 1);
        next           = [2:count, 1];
        ends_units     = vertices_units(next, :);
        minimum_units = min(vertices_units, ends_units);
        maximum_units = max(vertices_units, ends_units);
        if count ^ 2 <= 2 ^ 20
            % Bound the temporary dense broad phase to one million entries.
            overlap = minimum_units(:, 1) <= maximum_units(:, 1).' & ...
                maximum_units(:, 1) >= minimum_units(:, 1).' & ...
                minimum_units(:, 2) <= maximum_units(:, 2).' & ...
                maximum_units(:, 2) >= minimum_units(:, 2).';
            [firstIndices, secondIndices] = find(triu(overlap, 2));
        else
            [~, order] = sort(minimum_units(:, 1));
            active = zeros(0, 1);
            pairBlocks = cell(count, 1);
            for edgeIndex = reshape(order, 1, [])
                active = active(maximum_units(active, 1) >= minimum_units(edgeIndex, 1));
                candidates = active(maximum_units(active, 2) >= minimum_units(edgeIndex, 2) & ...
                    minimum_units(active, 2) <= maximum_units(edgeIndex, 2));
                pairBlocks{edgeIndex} = sort( ...
                    [repmat(edgeIndex, numel(candidates), 1), candidates], 2);
                active(end + 1, 1) = edgeIndex; %#ok<AGROW>
            end
            pairs = vertcat(pairBlocks{:});
            firstIndices  = pairs(:, 1);
            secondIndices = pairs(:, 2);
        end
        nonAdjacent = secondIndices ~= firstIndices + 1 & ...
            ~(firstIndices == 1 & secondIndices == count);
        firstIndices  = firstIndices(nonAdjacent);
        secondIndices = secondIndices(nonAdjacent);
        first_units    = vertices_units(firstIndices, :);
        last_units     = ends_units(firstIndices, :);
        other_units    = vertices_units(secondIndices, :);
        otherEnd_units = ends_units(secondIndices, :);
        direction_units      = last_units - first_units;
        otherDirection_units = otherEnd_units - other_units;
        % Orientation of each rival endpoint about the other moving edge.
        otherFromFirst_units    = other_units - first_units;
        otherEndFromFirst_units = otherEnd_units - first_units;
        firstFromOther_units    = first_units - other_units;
        lastFromOther_units     = last_units - other_units;
        a = cross2d(direction_units, otherFromFirst_units);
        b = cross2d(direction_units, otherEndFromFirst_units);
        c = cross2d(otherDirection_units, firstFromOther_units);
        d = cross2d(otherDirection_units, lastFromOther_units);
        crossing = sign(a) .* sign(b) < 0 & sign(c) .* sign(d) < 0;
        pairs = [firstIndices(crossing), secondIndices(crossing)];
        if isempty(pairs)
            break;
        end
        [~, order] = sortrows([diff(pairs, 1, 2), pairs(:, 1)], [1, 2]);
        best = pairs(order(1), :);
        % A self-crossing ring has no unique fill. Report the area removed
        % relative to MATLAB's explicit simplified fill, never hide it.
        retained(best(1) + 1:best(2)) = [];
    end
    if ~isempty(before) && numel(retained) < size(points_units, 1)
        after = polyshape();
        if numel(retained) >= 3
            after = polyshape(points_units(retained, :), ...
                'Simplify', true, 'KeepCollinearPoints', true);
        end
        changedArea_units2 = [area(subtract(before, after)), area(subtract(after, before))];
    end
end

function [verbose, vertexCorrespondence] = resolveConstructionOptions(options, isRawConstruction)
    % Resolve shared controls and the raw-only correspondence declaration.
    requireCondition(isstruct(options) && isscalar(options), ...
        "createObstacle:InvalidProtectionOptions", "options must be a scalar struct.");
    defaults             = struct("Verbose", false);
    vertexCorrespondence = "circularCorrelation";
    if isRawConstruction
        defaults.vertexCorrespondence = vertexCorrespondence;
    end
    [options, unknownNames] = obstacleAvoidance.input.resolveOptions(defaults, options);
    if ~isempty(unknownNames)
        warning("createObstacle:UnknownProtectionOptions", ...
            "Ignoring unknown option fields: %s. No behavior changed.", ...
            strjoin(unknownNames, ", "));
    end
    verbose = obstacleAvoidance.input.normalizeLogicalScalar( ...
        options.Verbose, "Verbose", "createObstacle:InvalidVerbose");
    if isRawConstruction
        vertexCorrespondence = options.vertexCorrespondence;
    end
end

function obstacles = protectObstacles(obstacles, safetyMargin_units, verbose)
    % Buffer large histories in parallel when background workers are available.
    for obstacleIndex = 1:numel(obstacles)
        obstacle             = obstacles(obstacleIndex);
        sampleCount          = numel(obstacle.time_s);
        protectedX_units     = cell(sampleCount, 1);
        protectedY_units     = cell(sampleCount, 1);
        vertexCount          = numel(vertcat(obstacle.originalX_units{:}));
        useBackgroundWorkers = false;
        if safetyMargin_units > 0 && vertexCount >= 500000 && exist("backgroundPool", "builtin") == 5
            workerPool           = backgroundPool;
            useBackgroundWorkers = workerPool.NumWorkers > 1 && ~workerPool.Busy;
        end
        if useBackgroundWorkers
            futures(1, sampleCount) = parallel.FevalFuture; %#ok<AGROW>
            for sampleIndex = 1:sampleCount
                futures(sampleIndex) = parfeval(workerPool, @inflateSlice, 2, ...
                    obstacle.originalX_units{sampleIndex}, ...
                    obstacle.originalY_units{sampleIndex}, safetyMargin_units);
            end
            [protectedX_units, protectedY_units] = fetchOutputs(futures, "UniformOutput", false);
        else
            for sampleIndex = 1:sampleCount
                [protectedX_units{sampleIndex}, protectedY_units{sampleIndex}] = inflateSlice( ...
                    obstacle.originalX_units{sampleIndex}, ...
                    obstacle.originalY_units{sampleIndex}, safetyMargin_units);
            end
        end
        if verbose
            fprintf("[x/y protect] obstacle %d/%d: %d slices complete.\n", ...
                obstacleIndex, numel(obstacles), sampleCount);
        end
        obstacle.x_units            = protectedX_units;
        obstacle.y_units            = protectedY_units;
        obstacle.safetyMargin_units = double(safetyMargin_units);
        if safetyMargin_units == 0
            % Retained originals already passed the same normalization rule.
            obstacles(obstacleIndex) = obstacle;
        else
            obstacles(obstacleIndex) = normalizeOne(obstacle);
        end
    end
end

function [protectedX_units, protectedY_units] = inflateSlice(x_units, y_units, safetyMargin_units)
    % Apply the margin with square joins; preserve order when the margin is zero.
    x_units = double(x_units(:));
    y_units = double(y_units(:));
    if safetyMargin_units == 0
        protectedX_units = x_units;
        protectedY_units = y_units;
        return;
    end
    x_units(~isfinite(x_units)) = NaN;
    y_units(~isfinite(y_units)) = NaN;
    if nnz(isfinite(x_units) & isfinite(y_units)) < 3
        protectedX_units = zeros(0, 1);
        protectedY_units = zeros(0, 1);
        return;
    end
    sourceShape = polyshape(x_units, y_units, "Simplify", true, "KeepCollinearPoints", true);
    requireCondition(~isempty(sourceShape.Vertices) && area(sourceShape) > 0, ...
        "createObstacle:DegeneratePolygon", ...
        "The boundary slice does not define a nonzero-area polygon.");
    protectedShape = polybuffer(sourceShape, safetyMargin_units, "JointType", "square");
    [protectedX_units, protectedY_units] = boundary(protectedShape);
    protectedX_units = double(protectedX_units(:));
    protectedY_units = double(protectedY_units(:));
    lastFinite = find(isfinite(protectedX_units) & isfinite(protectedY_units), 1, "last");
    protectedX_units = protectedX_units(1:lastFinite);
    protectedY_units = protectedY_units(1:lastFinite);
end

function value = cross2d(first_units, second_units)
    % Return row-wise signed two-dimensional cross products.
    value = first_units(:, 1) .* second_units(:, 2) - first_units(:, 2) .* second_units(:, 1);
end

function requireCondition(condition, identifier, message, varargin)
    % Report invalid input with the supplied error identifier.
    if ~condition
        error(identifier, message, varargin{:});
    end
end
