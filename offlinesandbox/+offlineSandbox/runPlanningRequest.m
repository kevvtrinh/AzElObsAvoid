function [response, diagnosisBundle] = runPlanningRequest(requestFilePath, resultFilePath)
%% Section 0: Header & Readme
% SYNTAX
%   response = offlineSandbox.runPlanningRequest( ...
%       requestFilePath, resultFilePath)
%   [response, diagnosisBundle] = offlineSandbox.runPlanningRequest( ...
%       requestFilePath, resultFilePath)
%**************************************************************************
% PURPOSE
%   - Read one offline-sandbox JSON request, call the maintained public
%     planner, independently validate a successful motion, and write a
%     browser-oriented JSON result.
%**************************************************************************
% INPUTS
%   - requestFilePath (scalar text)
%       Existing offlineSandboxRequest/v1 JSON file.
%       Derivative limits must all be combined scalar magnitudes or all be
%       [x y] pairs, following the public planner contract.
%   - resultFilePath (scalar text)
%       Destination JSON file, not a folder. Its parent folder must already
%       exist. An existing file at this explicit path is replaced.
%**************************************************************************
% OUTPUTS
%   - response (scalar struct)
%       offlineSandboxResult/v1 record containing the stable planner status,
%       sampled motion, independent validation, obstacle histories, and
%       bounded search diagnostics. Expected no-path outcomes are written
%       with result.Success=false; invalid requests throw identified errors.
%   - diagnosisBundle (scalar struct)
%       Handle-free obstacleAvoidanceSandboxDiagnosis-v2 record containing
%       the exact canonical request, unprojected result, validation, browser
%       scene geometry, environment metadata, and reproduction commands.
%**************************************************************************
% UNITS
%   - Positions and polygon vertices are [x y] in coordinate units.
%   - Time is seconds. Derivatives use units/s, units/s^2, and units/s^3.
%**************************************************************************

%% Section 1: Read & Validate The JSON Request

requestFilePath = normalizeFilePath(requestFilePath, "requestFilePath");
resultFilePath  = normalizeFilePath(resultFilePath, "resultFilePath");
if ~isfile(requestFilePath)
    error("runPlanningRequest:RequestFileNotFound", "requestFilePath does not exist: %s", requestFilePath);
end
if isfolder(resultFilePath)
    error("runPlanningRequest:ResultPathIsFolder", "resultFilePath must name a JSON file, not a folder: %s", resultFilePath);
end
resultFolder = fileparts(resultFilePath);
if strlength(resultFolder) == 0
    resultFolder = string(pwd);
end
if ~isfolder(resultFolder)
    error("runPlanningRequest:ResultFolderNotFound", "The resultFilePath parent folder does not exist: %s", resultFolder);
end
requestFilePath = canonicalExistingPath(requestFilePath, "requestFilePath");
resultFilePath  = canonicalResultPath(resultFilePath, resultFolder);
if pathsReferToSameFile(requestFilePath, resultFilePath)
    error("runPlanningRequest:MatchingPaths", "requestFilePath and resultFilePath must be different files.");
end
resultFolder = fileparts(resultFilePath);

try
    request = jsondecode(fileread(requestFilePath));
catch exception
    error("runPlanningRequest:InvalidJson", "Could not decode request JSON '%s': %s", requestFilePath, exception.message);
end
requireScalarStruct(request, "request");
requireFields(request, ["schemaVersion", "requestId", "obstacles", "initialState", "goalState", "limits", "options"], "request");
schemaVersion = normalizeScalarText(request.schemaVersion, "request.schemaVersion");
if schemaVersion ~= "offlineSandboxRequest/v1"
    error("runPlanningRequest:UnsupportedSchemaVersion", "request.schemaVersion must be 'offlineSandboxRequest/v1'; got '%s'.", schemaVersion);
end
requestId = normalizeScalarText(request.requestId, "request.requestId");
if strlength(strtrim(requestId)) == 0
    error("runPlanningRequest:EmptyRequestId", "request.requestId must be nonempty scalar text.");
end

initialState = normalizeState(request.initialState, "request.initialState");
goalState    = normalizeState(request.goalState, "request.goalState");
options      = request.options;
requireScalarStruct(options, "request.options");

%% Section 2: Construct Canonical Obstacles

packageParent  = fileparts(fileparts(mfilename("fullpath")));
repositoryRoot = fileparts(packageParent);
addpath(repositoryRoot, fullfile(repositoryRoot, "trajectory"));
limits = request.limits;
requireScalarStruct(limits, "request.limits");

% The wire format contains original polygon keyframes. Only the public
% constructor applies the requested safety margin, exactly once.
obstacles = createObstacles(request.obstacles);

%% Section 3: Run The Public Planner & Independent Validator

[result, diagnosis] = planner(obstacles, initialState, goalState, limits, options);
if result.Success
    validation = obstacleAvoidance.validateTrajectory(result);
else
    validation = result.Validation;
end

%% Section 4: Project & Write The Browser Result

projectedObstacles = projectObstacles(result.Inputs.obstacles);
response           = struct("schemaVersion", "offlineSandboxResult/v1", ...
    "requestId", requestId, ...
    "generatedAtUtc", string(datetime("now", "TimeZone", "UTC", ...
        "Format", "yyyy-MM-dd'T'HH:mm:ss'Z'")), ...
    "result", projectPlannerResult(result, request), ...
    "diagnosis", projectSearchDiagnostics(result), ...
    "validation", validation, ...
    "obstacles", {projectedObstacles});
if nargout > 1
    request.initialState = initialState;
    request.goalState = goalState;
    diagnosisBundle = offlineSandbox.createDiagnosisBundle(request, result, validation, diagnosis);
end

% MATLAB's documented JSON conversion maps unavailable NaN/Inf values to
% null. The browser treats null as an explicitly unavailable diagnostic.
jsonText             = jsonencode(response, "PrettyPrint", true);
jsonBytes            = unicode2native(char(jsonText), "UTF-8");
temporaryResultPath  = string(tempname(resultFolder)) + ".json";
temporaryFileCleanup = onCleanup(@() deleteFileIfPresent(temporaryResultPath));
[fileIdentifier, openMessage] = fopen(temporaryResultPath, "w", "n");
if fileIdentifier < 0
    error("runPlanningRequest:ResultFileOpenFailed", "Could not create a temporary result beside '%s': %s", resultFilePath, openMessage);
end
try
    writtenByteCount = fwrite(fileIdentifier, jsonBytes, "uint8");
    if writtenByteCount ~= numel(jsonBytes)
        error("runPlanningRequest:ResultFileWriteFailed", "Only %d of %d UTF-8 bytes were written beside '%s'.", writtenByteCount, numel(jsonBytes), resultFilePath);
    end
    closeStatus = fclose(fileIdentifier);
catch exception
    fclose(fileIdentifier);
    rethrow(exception);
end
if closeStatus ~= 0
    error("runPlanningRequest:ResultFileCloseFailed", "The temporary result file for '%s' could not be closed cleanly.", resultFilePath);
end
[moveSucceeded, moveMessage] = movefile(temporaryResultPath, resultFilePath, "f");
if ~moveSucceeded || ~isfile(resultFilePath)
    error("runPlanningRequest:ResultFileReplaceFailed", "Could not replace resultFilePath '%s': %s", resultFilePath, moveMessage);
end
clear temporaryFileCleanup;

end

%% Section 5: Local Functions

function filePath = normalizeFilePath(value, argumentName)
    % Normalize one nonempty scalar text path without changing its location.
    isScalarText = (isstring(value) && isscalar(value)) || (ischar(value) && isrow(value));
    if ~isScalarText
        error("runPlanningRequest:InvalidFilePath", "%s must be nonempty scalar text.", argumentName);
    end
    filePath = string(value);
    if ismissing(filePath) || strlength(strtrim(filePath)) == 0
        error("runPlanningRequest:InvalidFilePath", "%s must be nonempty scalar text.", argumentName);
    end
end

function text = normalizeScalarText(value, fieldName)
    % Normalize one JSON text field and retain an actionable field name.
    isScalarText = (isstring(value) && isscalar(value)) || (ischar(value) && isrow(value));
    if ~isScalarText
        error("runPlanningRequest:InvalidTextField", "%s must be scalar text.", fieldName);
    end
    text = string(value);
    if ismissing(text)
        error("runPlanningRequest:InvalidTextField", "%s must not be missing text.", fieldName);
    end
end

function canonicalPath = canonicalExistingPath(filePath, argumentName)
    % Resolve an existing path so aliases cannot bypass path-safety checks.
    [status, attributes] = fileattrib(filePath);
    if ~status
        error("runPlanningRequest:PathResolutionFailed", "Could not resolve %s: %s", argumentName, filePath);
    end
    canonicalPath = string(attributes.Name);
end

function canonicalPath = canonicalResultPath(filePath, parentFolder)
    % Resolve an output through its existing parent without creating the file.
    if isfile(filePath)
        canonicalPath = canonicalExistingPath(filePath, "resultFilePath");
        return;
    end
    canonicalFolder = canonicalExistingPath(parentFolder, "resultFilePath parent folder");
    [~, fileName, extension] = fileparts(filePath);
    canonicalPath = fullfile(canonicalFolder, string(fileName) + string(extension));
end

function isSameFile = pathsReferToSameFile(firstPath, secondPath)
    % Compare canonical paths using the host file system's case convention.
    if ispc
        isSameFile = strcmpi(firstPath, secondPath);
    else
        isSameFile = strcmp(firstPath, secondPath);
    end
end

function requireScalarStruct(value, fieldName)
    % Require one JSON object where the adapter depends on named fields.
    if ~isstruct(value) || ~isscalar(value)
        error("runPlanningRequest:InvalidObject", "%s must be one JSON object.", fieldName);
    end
end

function requireFields(value, requiredNames, fieldName)
    % Report every missing required field in one deterministic error.
    missingNames = requiredNames(~isfield(value, cellstr(requiredNames)));
    if ~isempty(missingNames)
        error("runPlanningRequest:MissingFields", "%s is missing required fields: %s.", fieldName, strjoin(missingNames, ", "));
    end
end

function state = normalizeState(value, fieldName)
    % Normalize required endpoint fields and optional zero-order derivatives.
    requireScalarStruct(value, fieldName);
    requireFields(value, ["time_s", "position_units"], fieldName);
    validateattributes(value.time_s, {'numeric'}, {'real', 'finite', 'scalar'}, "runPlanningRequest", fieldName + ".time_s");
    state = value;
    state.time_s       = double(value.time_s);
    state.position_units = normalizePair(value.position_units, fieldName + ".position_units");
    optionalPairNames = ["velocity_units_s", "acceleration_units_s2"];
    % Process each name needed by the sandbox workflow.
    for name = optionalPairNames
        if isfield(value, name) && ~isempty(value.(name))
            state.(name) = normalizePair(value.(name), fieldName + "." + name);
        end
    end
end

function pair = normalizePair(value, fieldName)
    % Normalize one finite [x y] state pair.
    validateattributes(value, {'numeric'}, {'real', 'finite', 'vector', 'numel', 2}, "runPlanningRequest", fieldName);
    pair = reshape(double(value), 1, 2);
end

function obstacles = createObstacles(obstacleInput)
    % Convert wire-format keyframes through the canonical public constructor.
    if isnumeric(obstacleInput) && isempty(obstacleInput)
        obstacles = obstacleAvoidance.obstacles.combineObstacles();
        return;
    end
    if ~isstruct(obstacleInput)
        error("runPlanningRequest:InvalidObstacles", "request.obstacles must be a JSON array of objects or [].");
    end
    obstacleCells = cell(numel(obstacleInput), 1);
    % Process each obstacle needed by the sandbox workflow.
    for obstacleIndex = 1:numel(obstacleInput)
        obstacle = obstacleInput(obstacleIndex);
        context  = "request.obstacles(" + obstacleIndex + ")";
        requireFields(obstacle, ["name", "safetyMargin_units", "keyframes"], context);
        obstacleName = normalizeScalarText(obstacle.name, context + ".name");
        if strlength(strtrim(obstacleName)) == 0
            error("runPlanningRequest:EmptyObstacleName", "%s.name must be nonempty scalar text.", context);
        end
        validateattributes(obstacle.safetyMargin_units, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'}, "runPlanningRequest", context + ".safetyMargin_units");
        keyframes = obstacle.keyframes;
        if ~isstruct(keyframes) || isempty(keyframes)
            error("runPlanningRequest:InvalidObstacleKeyframes", "%s.keyframes must contain at least one JSON object.", context);
        end
        time_s               = zeros(numel(keyframes), 1);
        xBySlice_units   = cell(numel(keyframes), 1);
        yBySlice_units = cell(numel(keyframes), 1);
        % Process each sample needed by the sandbox workflow.
        for sampleIndex = 1:numel(keyframes)
            keyframeContext = context + ".keyframes(" + sampleIndex + ")";
            requireFields(keyframes(sampleIndex), ["time_s", "vertices_units"], keyframeContext);
            validateattributes(keyframes(sampleIndex).time_s, {'numeric'}, {'real', 'finite', 'scalar'}, "runPlanningRequest", keyframeContext + ".time_s");
            rawVertices_units    = keyframes(sampleIndex).vertices_units;
            isValidVertexArray = isnumeric(rawVertices_units) && isreal(rawVertices_units) && ismatrix(rawVertices_units) && size(rawVertices_units, 2) == 2 && size(rawVertices_units, 1) >= 3 && all(isfinite(rawVertices_units), "all");
            if ~isValidVertexArray
                error("runPlanningRequest:InvalidObstacleVertices", "%s.vertices_units must be a finite N-by-2 numeric array " + "with N >= 3.", keyframeContext);
            end
            vertices_units = double(rawVertices_units);
            time_s(sampleIndex) = double(keyframes(sampleIndex).time_s);
            xBySlice_units{sampleIndex} = vertices_units(:, 1);
            yBySlice_units{sampleIndex} = vertices_units(:, 2);
        end
        if any(diff(time_s) <= 0)
            error("runPlanningRequest:InvalidObstacleTime", "%s keyframe times must be strictly increasing.", context);
        end
        obstacleCells{obstacleIndex} = obstacleAvoidance.obstacles.createObstacle(obstacleName, time_s, xBySlice_units, yBySlice_units, double(obstacle.safetyMargin_units));
    end
    obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacleCells);
end

function projection = projectPlannerResult(result, request)
    % Export actual core results, retaining the original request separately.
    names = ["Success", "Message", "TerminationReason", "Options", "Route_units", ...
        "time_s", "position_units", "velocity_units_s", "acceleration_units_s2", ...
        "jerk_units_s3", "ArrivalTime_s", "TrajectoryDuration_s", "ElapsedTime_s"];
    projection = struct();
    for name = names
        projection.(name) = result.(name);
    end
    projection.Inputs = struct("initialState", result.Inputs.initialState, ...
        "goalState", result.Inputs.goalState, "limits", result.Limits);
    projection.Request = struct("initialState", request.initialState, ...
        "goalState", request.goalState, "limits", request.limits, "options", request.options);
    for name = ["RequestedLimits", "MotionLength_units", "IntegratedSquaredJerk_units2_s5", ...
            "MaximumConstraintViolation", "FixedArrivalTrialTime_s"]
        if isfield(result, name), projection.(name) = result.(name); end
    end
end

function projection = projectSearchDiagnostics(result)
    % Project only examined graph edges; unexamined pairs are not rejections.
    graph = result.VisibilityGraph;
    nodes_units = graph.NodePosition_units;
    accepted = graph.AcceptedNodeIndex;
    rejected = graph.RejectedNodeIndex;
    search = struct();
    search.Nodes_units = nodes_units;
    search.AcceptedEdges_units = [nodes_units(accepted(:, 1), :), nodes_units(accepted(:, 2), :)];
    search.RejectedEdges_units = [nodes_units(rejected(:, 1), :), nodes_units(rejected(:, 2), :)];
    search.NodeCount = size(nodes_units, 1);
    search.AcceptedEdgeCount = size(accepted, 1);
    search.RejectedTransitionCount = size(rejected, 1);
    search.ExpandedCount = graph.ExpandedCount;
    search.SearchKind = graph.SearchKind;
    search.GraphIsFullyEnumerated = graph.GraphIsFullyEnumerated;
    search.CollisionQueryCount = NaN;
    if isfield(graph, "CollisionQueryCount"), search.CollisionQueryCount = graph.CollisionQueryCount; end
    search.TraceDownsampleRule = "All returned graph nodes and examined edges are displayed. " + ...
        "Unexamined edges remain implicit; expanded-node identities and frontier are not recorded. " + ...
        "For moving obstacles this spatial guide alone does not certify timed collision freedom.";
    projection = struct("Planner", "build-core", "Search", search, ...
        "SolverDiagnostics", result.SolverDiagnostics);
    if isfield(result, "TemporalSearch"), projection.TemporalSearch = result.TemporalSearch; end
end

function projection = projectObstacles(obstacles)
    % Export original and protected keyframes for dependency-free animation.
    template = struct("Name", "", ...
        "time_s", zeros(0, 1), ...
        "status", strings(0, 1), ...
        "SafetyMargin_units", 0, ...
        "OriginalVerticesByTime_units", {cell(0, 1)}, ...
        "ProtectedVerticesByTime_units", {cell(0, 1)});
    projection = cell(numel(obstacles), 1);
    % Process each obstacle needed by the sandbox workflow.
    for obstacleIndex = 1:numel(obstacles)
        obstacle          = obstacles(obstacleIndex);
        sampleCount       = numel(obstacle.time_s);
        originalVertices  = cell(sampleCount, 1);
        protectedVertices = cell(sampleCount, 1);
        % Process each sample needed by the sandbox workflow.
        for sampleIndex = 1:sampleCount
            originalVertices{sampleIndex} = [ ...
                obstacle.originalX_units{sampleIndex}, ...
                obstacle.originalY_units{sampleIndex}];
            protectedVertices{sampleIndex} = [ ...
                obstacle.x_units{sampleIndex}, obstacle.y_units{sampleIndex}];
        end
        projectedObstacle = template;
        projectedObstacle.Name                        = obstacle.targetName;
        projectedObstacle.time_s                      = obstacle.time_s;
        projectedObstacle.status                      = obstacle.status;
        projectedObstacle.SafetyMargin_units            = obstacle.safetyMargin_units;
        projectedObstacle.OriginalVerticesByTime_units  = originalVertices;
        projectedObstacle.ProtectedVerticesByTime_units = protectedVertices;
        projection{obstacleIndex} = projectedObstacle;
    end
end

function deleteFileIfPresent(filePath)
    % Remove only the adapter-owned sibling temporary file after a failed write.
    if isfile(filePath)
        delete(filePath);
    end
end
