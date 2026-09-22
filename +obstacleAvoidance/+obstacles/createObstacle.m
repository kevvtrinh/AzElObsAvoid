function obstacleData = createObstacle(obstacleInput, varargin)
%% Section 0: Header & Readme
% SYNTAX
%   obstacleData = obstacleAvoidance.obstacles.createObstacle(standardObstacle)
%   obstacleData = obstacleAvoidance.obstacles.createObstacle( ...
%       standardObstacles, safetyMargin_units, constructionOptions)
%   obstacleData = obstacleAvoidance.obstacles.createObstacle( ...
%       obstacleName, time_s, xBoundary_units, yBoundary_units)
%   obstacleData = obstacleAvoidance.obstacles.createObstacle( ...
%       obstacleName, time_s, xBoundary_units, yBoundary_units, ...
%       safetyMargin_units, constructionOptions)
%**************************************************************************
% PURPOSE
%   - Build obstacle histories in the standard planner format, retaining
%     original boundaries separately from boundaries with a safety margin.
%   - When applying a margin, rebuild from original geometry so it applies once.
%**************************************************************************
% INPUTS
%   - obstacleInput (scalar text, obstacle record, or collection of records)
%       Name when supplying boundary coordinates. A single existing record
%       can be checked on its own; a collection also needs a margin argument.
%   - time_s (numeric vector)
%       Strictly increasing sample times. One sample represents a stationary
%       obstacle for all times; several samples describe its active time range.
%   - xBoundary_units (numeric vector or cell array)
%       Raw boundary x-coordinates for one or more time samples.
%   - yBoundary_units (numeric vector or cell array)
%       Raw boundary y-coordinates matching xBoundary_units.
%   - safetyMargin_units (nonnegative numeric scalar, optional; default 0)
%       Margin added to original geometry. New obstacles default to 0;
%       checking an existing record without this argument keeps its stored margin.
%   - constructionOptions (scalar struct, optional; default struct())
%       Verbose applies to both construction paths and defaults to false.
%       vertexCorrespondence applies only to new coordinate inputs. Its default,
%       circularCorrelation, allows changing the first vertex and ring direction
%       when matching samples. sourceIndex matches vertices by their given order.
%       Existing records keep their vertex-matching rule.
%**************************************************************************
% OUTPUTS
%   - obstacleData (scalar or column struct array)
%       Original and protected histories plus normalization diagnostics.
%       Invalid input throws an error.
%**************************************************************************
% UNITS
%   - Boundary and margin use coordinate units; time uses seconds.
%**************************************************************************

%% Section 1: Choose Raw Construction Or Rebuild An Existing Obstacle

if nargin == 0
    error("createObstacle:MissingInput", "Obstacle construction or canonical input is required.");
end
inputUsesRecordForm = isstruct(obstacleInput) || iscell(obstacleInput) || ...
    (isnumeric(obstacleInput) && isempty(obstacleInput));
if inputUsesRecordForm && nargin == 1
    % Check one existing record without applying another safety margin.
    obstacleData = normalizeObstacleRecord(obstacleInput);
    return;
elseif inputUsesRecordForm && nargin >= 2 && nargin <= 3
    % A supplied margin replaces the old margin, using original boundaries.
    safetyMargin_units = varargin{1};
    options            = struct();
    if nargin == 3 && ~isempty(varargin{2})
        options = varargin{2};
    end
    validateattributes(safetyMargin_units, {'numeric'}, {'scalar', 'real', 'finite', 'nonnegative'});
    [verbose, ~] = resolveConstructionOptions(options, false);
    obstacleData = obstacleAvoidance.obstacles.combineObstacles(obstacleInput);
    obstacleData = applySafetyMargin(obstacleData, safetyMargin_units, verbose);
    return;
end
if nargin < 4 || nargin > 6
    error("createObstacle:InvalidCall", "Construction requires name, time, x, and y.");
end

%% Section 2: Create And Protect One Raw Record

time_s             = double(varargin{1}(:));
xHistory_units     = varargin{2};
yHistory_units     = varargin{3};
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

% A numeric boundary repeats at every time. Use one cell per sample when
% the boundary changes, so x and y histories line up with time_s.
sampleCount = numel(time_s);
if ~iscell(xHistory_units)
    xHistory_units = repmat({double(xHistory_units(:))}, sampleCount, 1);
end
if ~iscell(yHistory_units)
    yHistory_units = repmat({double(yHistory_units(:))}, sampleCount, 1);
end

% Start with identical original and protected boundaries and margin = 0.
% Normalize first, then build the protected boundary with the requested margin.
rawObstacle = struct( ...
    "targetName",           string(obstacleInput), ...
    "time_s",               time_s, ...
    "x_units",              {reshape(xHistory_units, [], 1)}, ...
    "y_units",              {reshape(yHistory_units, [], 1)}, ...
    "originalX_units",      {reshape(xHistory_units, [], 1)}, ...
    "originalY_units",      {reshape(yHistory_units, [], 1)}, ...
    "safetyMargin_units",   0, ...
    "status",               repmat("visible", sampleCount, 1), ...
    "vertexCorrespondence", vertexCorrespondence);
obstacleData = normalizeObstacleRecord(rawObstacle);
obstacleData = applySafetyMargin(obstacleData, safetyMargin_units, verbose);
end

%% Section 3: Local Functions

function obstacle = normalizeObstacleRecord(obstacleRecord)
    % Validate one record and use column vectors throughout its histories.
    % Rebuild the output fields so old prepared geometry cannot survive changes.
    requiredFields          = {'targetName', 'time_s', 'x_units', 'y_units', 'status'};
    recordHasRequiredFields = isstruct(obstacleRecord) && isscalar(obstacleRecord) && ...
        all(isfield(obstacleRecord, requiredFields));
    requireCondition(recordHasRequiredFields, "createObstacle:InvalidInput", ...
        "obstacleData must be one canonical obstacle record.");

    targetName = string(obstacleRecord.targetName);
    requireCondition(isscalar(targetName) && strlength(strtrim(targetName)) > 0, ...
        "createObstacle:InvalidTargetName", "targetName must be nonempty scalar text.");

    validateattributes(obstacleRecord.time_s, {'numeric'}, {'vector', 'real', 'finite'});
    time_s      = double(obstacleRecord.time_s(:));
    sampleCount = numel(time_s);
    requireCondition(sampleCount > 0 && all(diff(time_s) > 0), ...
        "createObstacle:InvalidTime", "time_s must be nonempty and strictly increasing.");

    xHistoryIsValid = iscell(obstacleRecord.x_units) && numel(obstacleRecord.x_units) == sampleCount;
    yHistoryIsValid = iscell(obstacleRecord.y_units) && numel(obstacleRecord.y_units) == sampleCount;
    requireCondition(xHistoryIsValid && yHistoryIsValid, "createObstacle:InvalidBoundary", ...
        "x_units and y_units must be cell arrays matching time_s.");
    [xHistory_units, yHistory_units, protectedRemovalCounts, ...
        protectedSampleWasChanged, protectedRepairDetails] = normalizeBoundaryHistory( ...
        obstacleRecord.x_units, obstacleRecord.y_units, sampleCount, "protected");

    % Keep original boundaries separate: later margin changes must start from
    % those coordinates. If both histories are identical, reuse the checks above.
    hasOriginalX = isfield(obstacleRecord, "originalX_units");
    hasOriginalY = isfield(obstacleRecord, "originalY_units");
    requireCondition(~xor(hasOriginalX, hasOriginalY), ...
        "createObstacle:IncompleteOriginalBoundary", ...
        "originalX_units and originalY_units must both be present or absent.");
    if hasOriginalX
        originalXIsValid = iscell(obstacleRecord.originalX_units) && ...
            numel(obstacleRecord.originalX_units) == sampleCount;
        originalYIsValid = iscell(obstacleRecord.originalY_units) && ...
            numel(obstacleRecord.originalY_units) == sampleCount;
        requireCondition(originalXIsValid && originalYIsValid, ...
            "createObstacle:InvalidOriginalBoundary", ...
            "Original boundary cells must match time_s.");
        protectedIsOriginal = isequaln(obstacleRecord.originalX_units, obstacleRecord.x_units) && ...
            isequaln(obstacleRecord.originalY_units, obstacleRecord.y_units);
        if protectedIsOriginal
            originalXHistory_units   = xHistory_units;
            originalYHistory_units   = yHistory_units;
            originalRemovalCounts    = protectedRemovalCounts;
            originalSampleWasChanged = protectedSampleWasChanged;
            originalRepairDetails    = protectedRepairDetails;
        else
            [originalXHistory_units, originalYHistory_units, originalRemovalCounts, ...
                originalSampleWasChanged, originalRepairDetails] = normalizeBoundaryHistory( ...
                obstacleRecord.originalX_units, obstacleRecord.originalY_units, sampleCount, "original");
        end
    else
        % Without separate originals, these coordinates are the original shape.
        % A nonzero stored margin requires originals and is rejected below.
        originalXHistory_units   = xHistory_units;
        originalYHistory_units   = yHistory_units;
        originalRemovalCounts    = [0, 0];
        originalSampleWasChanged = false(sampleCount, 1);
        originalRepairDetails    = zeros(sampleCount, 3);
    end

    % Record what changed at each sample, separately for protected and original
    % boundaries. These measurements explain normalization to the caller.
    affectedSampleIndices    = find(protectedSampleWasChanged | originalSampleWasChanged);
    normalizationDiagnostics = struct( ...
        'Version',                          1, ...
        'SourceTime_s',                     time_s, ...
        'Roles',                            ["protected", "original"], ...
        'RemovedRegionCount',               [protectedRemovalCounts(1), originalRemovalCounts(1)], ...
        'RemovedDuplicateVertexCount',      [protectedRemovalCounts(2), originalRemovalCounts(2)], ...
        'AffectedSampleIndex',              affectedSampleIndices, ...
        'AffectedSampleTime_s',             time_s(affectedSampleIndices), ...
        'RemovedZigzagVertexCountBySample', [protectedRepairDetails(:, 1), originalRepairDetails(:, 1)], ...
        'RemovedZigzagAreaBySample_units2', [protectedRepairDetails(:, 2), originalRepairDetails(:, 2)], ...
        'AddedZigzagAreaBySample_units2',   [protectedRepairDetails(:, 3), originalRepairDetails(:, 3)], ...
        'Reasons',                          ["fewerThanThreeDistinctVertices", ...
                                              "exactDuplicateOrClosure", ...
                                              "selfCrossingZigzagRemoved"]);
    % Keep a record of removed or corrected vertices when rebuilding an
    % obstacle or changing its margin. These notes do not change the geometry
    % checks or determine which points are occupied.
    if isfield(obstacleRecord, 'NormalizationDiagnostics')
        previousDiagnostics      = obstacleRecord.NormalizationDiagnostics;
        requiredDiagnosticFields = {'Version', 'SourceTime_s', 'RemovedRegionCount', ...
            'RemovedDuplicateVertexCount', 'AffectedSampleIndex'};
        previousDiagnosticsMatchSampleTimes = isstruct(previousDiagnostics) && isscalar(previousDiagnostics) && ...
            all(isfield(previousDiagnostics, requiredDiagnosticFields)) && ...
            isequal(previousDiagnostics.Version, 1) && isequal(previousDiagnostics.SourceTime_s, time_s);
        if previousDiagnosticsMatchSampleTimes
            for fieldName = ["RemovedRegionCount", "RemovedDuplicateVertexCount"]
                validateattributes(previousDiagnostics.(fieldName), {'numeric'}, ...
                    {'real', 'finite', 'size', [1, 2], 'integer', 'nonnegative'});
                normalizationDiagnostics.(fieldName) = ...
                    normalizationDiagnostics.(fieldName) + previousDiagnostics.(fieldName);
            end
            repairFieldNames = ["RemovedZigzagVertexCountBySample", ...
                "RemovedZigzagAreaBySample_units2", "AddedZigzagAreaBySample_units2"];
            for fieldName = repairFieldNames
                if isfield(previousDiagnostics, fieldName)
                    validateattributes(previousDiagnostics.(fieldName), {'numeric'}, ...
                        {'real', 'finite', 'size', [sampleCount, 2], 'nonnegative'});
                    normalizationDiagnostics.(fieldName) = ...
                        normalizationDiagnostics.(fieldName) + previousDiagnostics.(fieldName);
                end
            end
            validateattributes(previousDiagnostics.AffectedSampleIndex, {'numeric'}, ...
                {'real', 'finite', 'integer', 'positive', '<=', sampleCount});
            affectedSampleIndices = union(affectedSampleIndices, previousDiagnostics.AffectedSampleIndex(:));

            normalizationDiagnostics.AffectedSampleIndex  = affectedSampleIndices;
            normalizationDiagnostics.AffectedSampleTime_s = time_s(affectedSampleIndices);
        end
    end

    safetyMargin_units = 0;
    if isfield(obstacleRecord, "safetyMargin_units")
        safetyMargin_units = obstacleRecord.safetyMargin_units;
    end
    validateattributes(safetyMargin_units, {'numeric'}, {'scalar', 'real', 'finite', 'nonnegative'});
    safetyMargin_units = double(safetyMargin_units);
    requireCondition(safetyMargin_units == 0 || hasOriginalX, ...
        "createObstacle:MissingOriginalBoundary", ...
        "Positive safetyMargin_units requires retained original boundaries.");

    status = string(obstacleRecord.status);
    if isscalar(status)
        status = repmat(status, sampleCount, 1);
    elseif numel(status) == sampleCount
        status = status(:);
    else
        error("createObstacle:StatusSizeMismatch", "status must contain one value per time sample.");
    end

    % sourceIndex declares that vertex i matches vertex i at the next sample.
    % circularCorrelation lets preparation align the starting vertex and direction.
    vertexCorrespondence = "circularCorrelation";
    if isfield(obstacleRecord, "vertexCorrespondence") && ~isempty(obstacleRecord.vertexCorrespondence)
        vertexCorrespondence = string(obstacleRecord.vertexCorrespondence);
        requireCondition(isscalar(vertexCorrespondence) && ...
            any(vertexCorrespondence == ["circularCorrelation", "sourceIndex"]), ...
            "createObstacle:InvalidVertexCorrespondence", ...
            "vertexCorrespondence must be circularCorrelation or sourceIndex.");
    end
    usesSourceIndex = vertexCorrespondence == "sourceIndex";
    obstacle        = struct( ...
        "targetName",               targetName, ...
        "time_s",                   time_s, ...
        "x_units",                  {xHistory_units}, ...
        "y_units",                  {yHistory_units}, ...
        "originalX_units",          {originalXHistory_units}, ...
        "originalY_units",          {originalYHistory_units}, ...
        "safetyMargin_units",       safetyMargin_units, ...
        "status",                   status, ...
        "NormalizationDiagnostics", normalizationDiagnostics, ...
        "vertexCorrespondence",     vertexCorrespondence, ...
        "UsesSourceIndex",          usesSourceIndex);
end

function [xHistory_units, yHistory_units, removalCounts, sampleWasChanged, sampleRepairDetails] = ...
        normalizeBoundaryHistory(xInput_units, yInput_units, sampleCount, boundaryRole)
    % Check each boundary sample, keeping original/protected error labels distinct.
    % removalCounts = [removed rings, removed duplicate vertices].
    % Repair columns = [removed vertex count, removed area, added area].
    xHistory_units       = reshape(xInput_units, [], 1);
    yHistory_units       = reshape(yInput_units, [], 1);
    removalCounts        = [0, 0];
    sampleWasChanged     = false(sampleCount, 1);
    sampleRepairDetails  = zeros(sampleCount, 3);
    sizeMismatchErrorIds = ["createObstacle:BoundarySizeMismatch", ...
        "createObstacle:OriginalBoundarySizeMismatch"];
    coordinateFieldNames = ["x_units", "y_units"; "originalX_units", "originalY_units"];
    boundaryRoleIndex    = 1 + (boundaryRole == "original");
    for sampleIndex = 1:sampleCount
        validateattributes(xHistory_units{sampleIndex}, {'numeric'}, {'vector', 'real'});
        validateattributes(yHistory_units{sampleIndex}, {'numeric'}, {'vector', 'real'});
        if numel(xHistory_units{sampleIndex}) ~= numel(yHistory_units{sampleIndex})
            error(sizeMismatchErrorIds(boundaryRoleIndex), "%s and %s slice %d must have equal lengths.", ...
                coordinateFieldNames(boundaryRoleIndex, 1), coordinateFieldNames(boundaryRoleIndex, 2), sampleIndex);
        end
        x_units = double(xHistory_units{sampleIndex}(:));
        y_units = double(yHistory_units{sampleIndex}(:));

        [x_units, y_units, sampleRemovalCounts, repairDetails] = ...
            normalizeBoundarySample(x_units, y_units, sampleIndex, boundaryRole);
        xHistory_units{sampleIndex} = x_units;
        yHistory_units{sampleIndex} = y_units;
        removalCounts                      = removalCounts + sampleRemovalCounts;
        sampleWasChanged(sampleIndex)      = any(sampleRemovalCounts > 0) || repairDetails(1) > 0;
        sampleRepairDetails(sampleIndex, :) = repairDetails;
    end
end

function [x_units, y_units, removalCounts, repairDetails] = normalizeBoundarySample( ...
        x_units, y_units, sampleIndex, boundaryRole)
    % Remove exact duplicates and crossing folds, recording every removal.
    % A ring is one polygon boundary; paired nonfinite rows separate rings.
    xIsFinite = isfinite(x_units);
    yIsFinite = isfinite(y_units);
    requireCondition(~any(xor(xIsFinite, yIsFinite)), ...
        "createObstacle:UnpairedNonfiniteBoundary", ...
        "The %s boundary at slice %d must use paired separators.", boundaryRole, sampleIndex);
    finiteRunChange   = diff([false; xIsFinite; false]);
    ringStartIndices  = find(finiteRunChange == 1);
    ringEndIndices    = find(finiteRunChange == -1) - 1;
    retainedRowsByRing = cell(numel(ringStartIndices), 1);
    repairDetails     = [0, 0, 0];
    removalCounts     = [0, 0];
    for ringIndex = 1:numel(ringStartIndices)
        % Remove only exactly repeated neighbors. Distinct nearby vertices
        % may describe a narrow feature and must remain.
        ringRowIndices        = (ringStartIndices(ringIndex):ringEndIndices(ringIndex)).';
        vertexRepeatsPrevious = [false; diff(x_units(ringRowIndices)) == 0 & diff(y_units(ringRowIndices)) == 0];

        removalCounts(2) = removalCounts(2) + nnz(vertexRepeatsPrevious);
        ringRowIndices(vertexRepeatsPrevious) = [];

        % Polygon rings close automatically; a final copy of the first vertex
        % would repeat that point when building the shape.
        if numel(ringRowIndices) > 1 && x_units(ringRowIndices(1)) == x_units(ringRowIndices(end)) && ...
                y_units(ringRowIndices(1)) == y_units(ringRowIndices(end))
            ringRowIndices(end) = [];
            removalCounts(2)    = removalCounts(2) + 1;
        end

        % Crossing edges do not form a simple boundary. Remove the detected
        % folds and keep measurements of the resulting area changes.
        [retainedVertexIndices, changedArea_units2] = ...
            removeCrossingZigzags([x_units(ringRowIndices), y_units(ringRowIndices)]);
        repairDetails = repairDetails + ...
            [numel(ringRowIndices) - numel(retainedVertexIndices), changedArea_units2];
        ringRowIndices           = ringRowIndices(retainedVertexIndices);
        hasThreeDistinctVertices = numel(ringRowIndices) >= 3;
        if hasThreeDistinctVertices
            % Several rows may repeat only two points, such as [A; B; A; B].
            % Require a third distinct point; two points cannot enclose an area.
            differsFromFirst = x_units(ringRowIndices) ~= x_units(ringRowIndices(1)) | ...
                y_units(ringRowIndices) ~= y_units(ringRowIndices(1));
            differsFromSecond = x_units(ringRowIndices) ~= x_units(ringRowIndices(2)) | ...
                y_units(ringRowIndices) ~= y_units(ringRowIndices(2));
            hasThreeDistinctVertices = any(differsFromFirst & differsFromSecond);
        end
        if hasThreeDistinctVertices
            retainedRowsByRing{ringIndex} = ringRowIndices;
        else
            removalCounts(1) = removalCounts(1) + 1;
        end
    end

    % Preserve the original ring order and separators when no vertices changed.
    if ~any(removalCounts) && repairDetails(1) == 0 && any(xIsFinite)
        return;
    end

    ringVertexCounts    = cellfun(@numel, retainedRowsByRing);
    retainedRingIndices = find(ringVertexCounts > 0);
    if isempty(retainedRingIndices)
        x_units = zeros(0, 1);
        y_units = zeros(0, 1);
        return;
    end
    % Leave one NaN row between retained rings, in both coordinate arrays.
    outputRowCount     = sum(ringVertexCounts(retainedRingIndices)) + numel(retainedRingIndices) - 1;
    normalizedX_units  = NaN(outputRowCount, 1);
    normalizedY_units  = NaN(outputRowCount, 1);
    nextOutputRowIndex = 1;
    for outputRingIndex = 1:numel(retainedRingIndices)
        ringIndex        = retainedRingIndices(outputRingIndex);
        inputRowIndices  = retainedRowsByRing{ringIndex};
        outputRowIndices = nextOutputRowIndex + (0:numel(inputRowIndices) - 1);

        normalizedX_units(outputRowIndices) = x_units(inputRowIndices);
        normalizedY_units(outputRowIndices) = y_units(inputRowIndices);
        nextOutputRowIndex = outputRowIndices(end) + 2;
    end
    x_units = normalizedX_units;
    y_units = normalizedY_units;
end

function [retainedVertexIndices, changedArea_units2] = removeCrossingZigzags(points_units)
    % Remove folds where non-neighboring edges cross through each other.
    % Touching or collinear edges are kept; only strict crossings are repaired.
    retainedVertexIndices = (1:size(points_units, 1)).';
    changedArea_units2    = [0, 0];
    shapeBeforeRepair     = [];
    while numel(retainedVertexIndices) >= 3
        vertices_units    = points_units(retainedVertexIndices, :);
        vertexCount       = size(vertices_units, 1);
        nextVertexIndices = [2:vertexCount, 1];
        edgeEnd_units     = vertices_units(nextVertexIndices, :);
        edgeMinimum_units = min(vertices_units, edgeEnd_units);
        edgeMaximum_units = max(vertices_units, edgeEnd_units);
        if vertexCount ^ 2 <= 2 ^ 20
            % Compare edge bounding boxes in one array when it needs at most
            % about one million entries. Disjoint boxes cannot contain a crossing.
            edgeBoxesOverlap = edgeMinimum_units(:, 1) <= edgeMaximum_units(:, 1).' & ...
                edgeMaximum_units(:, 1) >= edgeMinimum_units(:, 1).' & ...
                edgeMinimum_units(:, 2) <= edgeMaximum_units(:, 2).' & ...
                edgeMaximum_units(:, 2) >= edgeMinimum_units(:, 2).';
            [firstEdgeIndices, secondEdgeIndices] = find(triu(edgeBoxesOverlap, 2));
        else
            % For larger rings, scan boxes from left to right. Keep only boxes
            % that still overlap in x, then check their y ranges. This finds
            % the same candidate pairs without building the full pair array.
            [~, sortOrder]    = sort(edgeMinimum_units(:, 1));
            activeEdgeIndices = zeros(0, 1);
            edgePairsByIndex  = cell(vertexCount, 1);
            for edgeIndex = reshape(sortOrder, 1, [])
                activeEdgeIndices = activeEdgeIndices( ...
                    edgeMaximum_units(activeEdgeIndices, 1) >= edgeMinimum_units(edgeIndex, 1));
                overlappingEdgeIndices = activeEdgeIndices( ...
                    edgeMaximum_units(activeEdgeIndices, 2) >= edgeMinimum_units(edgeIndex, 2) & ...
                    edgeMinimum_units(activeEdgeIndices, 2) <= edgeMaximum_units(edgeIndex, 2));
                edgePairsByIndex{edgeIndex} = sort( ...
                    [repmat(edgeIndex, numel(overlappingEdgeIndices), 1), overlappingEdgeIndices], 2);
                activeEdgeIndices(end + 1, 1) = edgeIndex; %#ok<AGROW>
            end
            edgePairs         = vertcat(edgePairsByIndex{:});
            firstEdgeIndices  = edgePairs(:, 1);
            secondEdgeIndices = edgePairs(:, 2);
        end

        % Neighboring edges share a vertex, including the first and last edges.
        % Their shared endpoint is not a crossing that needs repair.
        edgesAreNonadjacent = secondEdgeIndices ~= firstEdgeIndices + 1 & ...
            ~(firstEdgeIndices == 1 & secondEdgeIndices == vertexCount);
        firstEdgeIndices          = firstEdgeIndices(edgesAreNonadjacent);
        secondEdgeIndices         = secondEdgeIndices(edgesAreNonadjacent);
        firstEdgeStart_units      = vertices_units(firstEdgeIndices, :);
        firstEdgeEnd_units        = edgeEnd_units(firstEdgeIndices, :);
        secondEdgeStart_units     = vertices_units(secondEdgeIndices, :);
        secondEdgeEnd_units       = edgeEnd_units(secondEdgeIndices, :);
        firstEdgeDirection_units  = firstEdgeEnd_units - firstEdgeStart_units;
        secondEdgeDirection_units = secondEdgeEnd_units - secondEdgeStart_units;

        % Cross-product signs tell which side of an edge each endpoint lies on.
        % A strict crossing requires opposite sides for both edge pairs;
        % a zero sign means touching or collinearity and does not qualify.
        toSecondStart_units = secondEdgeStart_units - firstEdgeStart_units;
        toSecondEnd_units   = secondEdgeEnd_units - firstEdgeStart_units;
        toFirstStart_units  = firstEdgeStart_units - secondEdgeStart_units;
        toFirstEnd_units    = firstEdgeEnd_units - secondEdgeStart_units;

        secondStartSide_units2 = obstacleAvoidance.geometry.cross2d(firstEdgeDirection_units, toSecondStart_units);
        secondEndSide_units2   = obstacleAvoidance.geometry.cross2d(firstEdgeDirection_units, toSecondEnd_units);
        firstStartSide_units2  = obstacleAvoidance.geometry.cross2d(secondEdgeDirection_units, toFirstStart_units);
        firstEndSide_units2    = obstacleAvoidance.geometry.cross2d(secondEdgeDirection_units, toFirstEnd_units);

        edgesCross = sign(secondStartSide_units2) .* sign(secondEndSide_units2) < 0 & ...
            sign(firstStartSide_units2) .* sign(firstEndSide_units2) < 0;
        edgePairs = [firstEdgeIndices(edgesCross), secondEdgeIndices(edgesCross)];
        if isempty(edgePairs)
            break;
        end
        if isempty(shapeBeforeRepair)
            % A crossing boundary has no single obvious interior. Use MATLAB's
            % simplified polygon as the reference for reporting area changes.
            % Build it only once, and only when a repair is actually needed.
            warningState      = warning('off', 'MATLAB:polyshape:repairedBySimplify');
            restoreWarning    = onCleanup(@()warning(warningState));
            shapeBeforeRepair = polyshape(points_units, ...
                'Simplify', true, 'KeepCollinearPoints', true);
            clear restoreWarning;
        end

        % Remove the fold with the fewest intervening vertices. If tied,
        % choose the pair with the lower first-edge index for a stable result.
        [~, sortOrder]   = sortrows([diff(edgePairs, 1, 2), edgePairs(:, 1)], [1, 2]);
        crossingToRemove = edgePairs(sortOrder(1), :);

        retainedVertexIndices(crossingToRemove(1) + 1:crossingToRemove(2)) = [];
    end

    % Report removed and added areas separately so neither change is hidden.
    if ~isempty(shapeBeforeRepair) && numel(retainedVertexIndices) < size(points_units, 1)
        shapeAfterRepair = polyshape();
        if numel(retainedVertexIndices) >= 3
            shapeAfterRepair = polyshape(points_units(retainedVertexIndices, :), ...
                'Simplify', true, 'KeepCollinearPoints', true);
        end
        changedArea_units2 = [area(subtract(shapeBeforeRepair, shapeAfterRepair)), ...
            area(subtract(shapeAfterRepair, shapeBeforeRepair))];
    end
end

function [verbose, vertexCorrespondence] = resolveConstructionOptions(options, isRawConstruction)
    % Both input forms can report progress. Only new coordinate inputs can
    % select a vertex-matching rule; existing records keep their stored rule.
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

function obstacles = applySafetyMargin(obstacles, safetyMargin_units, verbose)
    % Always add the margin to original boundaries, so changing a margin from
    % 1 to 2 produces a total margin of 2, not 3. Large histories can process
    % samples independently on available background workers.
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
                futures(sampleIndex) = parfeval(workerPool, @addMarginToSample, 2, ...
                    obstacle.originalX_units{sampleIndex}, ...
                    obstacle.originalY_units{sampleIndex}, safetyMargin_units);
            end
            [protectedX_units, protectedY_units] = fetchOutputs(futures, "UniformOutput", false);
        else
            for sampleIndex = 1:sampleCount
                [protectedX_units{sampleIndex}, protectedY_units{sampleIndex}] = addMarginToSample( ...
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
            % No offset changed the vertices; the originals already passed checks.
            obstacles(obstacleIndex) = obstacle;
        else
            obstacles(obstacleIndex) = normalizeObstacleRecord(obstacle);
        end
    end
end

function [protectedX_units, protectedY_units] = addMarginToSample(x_units, y_units, safetyMargin_units)
    % Expand one boundary by the margin, using straight corner joins.
    % A zero margin keeps the original coordinates and vertex order.
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

    % Keep separators between rings, but omit any trailing separator rows.
    lastFiniteVertexIndex = find(isfinite(protectedX_units) & isfinite(protectedY_units), 1, "last");
    protectedX_units      = protectedX_units(1:lastFiniteVertexIndex);
    protectedY_units      = protectedY_units(1:lastFiniteVertexIndex);
end

function requireCondition(condition, identifier, message, varargin)
    % Report invalid input with the supplied error identifier.
    if ~condition
        error(identifier, message, varargin{:});
    end
end
