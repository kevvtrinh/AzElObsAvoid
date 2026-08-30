function response = runPlanningRequest(requestFilePath, resultFilePath)
%% Section 0: Header & Readme
% SYNTAX
%   response = offlineSandbox.runPlanningRequest( ...
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
%**************************************************************************
% UNITS
%   - Positions and polygon vertices are [azimuth elevation] in degrees.
%   - Time is seconds. Derivatives use deg/s, deg/s^2, and deg/s^3.
%**************************************************************************

%% Section 1: Read & Validate The JSON Request

requestFilePath = normalizeFilePath(requestFilePath, "requestFilePath");
resultFilePath = normalizeFilePath(resultFilePath, "resultFilePath");
if ~isfile(requestFilePath)
    error("runPlanningRequest:RequestFileNotFound", ...
        "requestFilePath does not exist: %s", requestFilePath);
end
if isfolder(resultFilePath)
    error("runPlanningRequest:ResultPathIsFolder", ...
        "resultFilePath must name a JSON file, not a folder: %s", ...
        resultFilePath);
end
resultFolder = fileparts(resultFilePath);
if strlength(resultFolder) == 0
    resultFolder = string(pwd);
end
if ~isfolder(resultFolder)
    error("runPlanningRequest:ResultFolderNotFound", ...
        "The resultFilePath parent folder does not exist: %s", ...
        resultFolder);
end
requestFilePath = canonicalExistingPath( ...
    requestFilePath, "requestFilePath");
resultFilePath = canonicalResultPath(resultFilePath, resultFolder);
if pathsReferToSameFile(requestFilePath, resultFilePath)
    error("runPlanningRequest:MatchingPaths", ...
        "requestFilePath and resultFilePath must be different files.");
end
resultFolder = fileparts(resultFilePath);

try
    request = jsondecode(fileread(requestFilePath));
catch exception
    error("runPlanningRequest:InvalidJson", ...
        "Could not decode request JSON '%s': %s", ...
        requestFilePath, exception.message);
end
requireScalarStruct(request, "request");
requireFields(request, ["schemaVersion", "requestId", "obstacles", ...
    "initialState", "goalState", "limits", "options"], "request");
schemaVersion = normalizeScalarText( ...
    request.schemaVersion, "request.schemaVersion");
if schemaVersion ~= "offlineSandboxRequest/v1"
    error("runPlanningRequest:UnsupportedSchemaVersion", ...
        "request.schemaVersion must be 'offlineSandboxRequest/v1'; got '%s'.", ...
        schemaVersion);
end
requestId = normalizeScalarText(request.requestId, "request.requestId");
if strlength(strtrim(requestId)) == 0
    error("runPlanningRequest:EmptyRequestId", ...
        "request.requestId must be nonempty scalar text.");
end

initialState = normalizeState(request.initialState, "request.initialState");
goalState = normalizeState(request.goalState, "request.goalState");
limits = normalizeLimits(request.limits);
options = request.options;
requireScalarStruct(options, "request.options");
if isfield(options, "CancellationCheckFcn")
    error("runPlanningRequest:UnsupportedCallbackOption", ...
        "request.options must not contain CancellationCheckFcn; JSON cannot " + ...
        "carry MATLAB callbacks.");
end

%% Section 2: Construct Canonical Obstacles

packageParent = fileparts(fileparts(mfilename("fullpath")));
repositoryRoot = fileparts(packageParent);
addpath(repositoryRoot, fullfile(repositoryRoot, "trajectory"));

% The wire format contains original polygon keyframes. Only the public
% constructor applies the requested safety margin, exactly once.
obstacles = createObstacles(request.obstacles);

%% Section 3: Run The Public Planner & Independent Validator

result = obstacleAvoidance.planTrajectory( ...
    obstacles, initialState, goalState, limits, options);
if result.Success
    validation = obstacleAvoidance.validateTrajectory(result);
else
    validation = result.Validation;
end

%% Section 4: Project & Write The Browser Result

projectedObstacles = projectObstacles(result.Inputs.obstacles);
response = struct( ...
    "schemaVersion", "offlineSandboxResult/v1", ...
    "requestId", requestId, ...
    "generatedAtUtc", string(datetime("now", "TimeZone", "UTC", ...
        "Format", "yyyy-MM-dd'T'HH:mm:ss'Z'")), ...
    "result", projectPlannerResult(result), ...
    "validation", validation, ...
    "obstacles", {projectedObstacles});

% MATLAB's documented JSON conversion maps unavailable NaN/Inf values to
% null. The browser treats null as an explicitly unavailable diagnostic.
jsonText = jsonencode(response, "PrettyPrint", true);
jsonBytes = unicode2native(char(jsonText), "UTF-8");
temporaryResultPath = string(tempname(resultFolder)) + ".json";
temporaryFileCleanup = onCleanup( ...
    @() deleteFileIfPresent(temporaryResultPath));
[fileIdentifier, openMessage] = fopen( ...
    temporaryResultPath, "w", "n");
if fileIdentifier < 0
    error("runPlanningRequest:ResultFileOpenFailed", ...
        "Could not create a temporary result beside '%s': %s", ...
        resultFilePath, openMessage);
end
try
    writtenByteCount = fwrite(fileIdentifier, jsonBytes, "uint8");
    if writtenByteCount ~= numel(jsonBytes)
        error("runPlanningRequest:ResultFileWriteFailed", ...
            "Only %d of %d UTF-8 bytes were written beside '%s'.", ...
            writtenByteCount, numel(jsonBytes), resultFilePath);
    end
    closeStatus = fclose(fileIdentifier);
catch exception
    fclose(fileIdentifier);
    rethrow(exception);
end
if closeStatus ~= 0
    error("runPlanningRequest:ResultFileCloseFailed", ...
        "The temporary result file for '%s' could not be closed cleanly.", ...
        resultFilePath);
end
[moveSucceeded, moveMessage] = movefile( ...
    temporaryResultPath, resultFilePath, "f");
if ~moveSucceeded || ~isfile(resultFilePath)
    error("runPlanningRequest:ResultFileReplaceFailed", ...
        "Could not replace resultFilePath '%s': %s", ...
        resultFilePath, moveMessage);
end
clear temporaryFileCleanup;

end

%% Section 5: Local Functions

function filePath = normalizeFilePath(value, argumentName)
% Normalize one nonempty scalar text path without changing its location.
isScalarText = (isstring(value) && isscalar(value)) || ...
    (ischar(value) && isrow(value));
if ~isScalarText
    error("runPlanningRequest:InvalidFilePath", ...
        "%s must be nonempty scalar text.", argumentName);
end
filePath = string(value);
if ismissing(filePath) || strlength(strtrim(filePath)) == 0
    error("runPlanningRequest:InvalidFilePath", ...
        "%s must be nonempty scalar text.", argumentName);
end
end

function text = normalizeScalarText(value, fieldName)
% Normalize one JSON text field and retain an actionable field name.
isScalarText = (isstring(value) && isscalar(value)) || ...
    (ischar(value) && isrow(value));
if ~isScalarText
    error("runPlanningRequest:InvalidTextField", ...
        "%s must be scalar text.", fieldName);
end
text = string(value);
if ismissing(text)
    error("runPlanningRequest:InvalidTextField", ...
        "%s must not be missing text.", fieldName);
end
end

function canonicalPath = canonicalExistingPath(filePath, argumentName)
% Resolve an existing path so aliases cannot bypass path-safety checks.
[status, attributes] = fileattrib(filePath);
if ~status
    error("runPlanningRequest:PathResolutionFailed", ...
        "Could not resolve %s: %s", argumentName, filePath);
end
canonicalPath = string(attributes.Name);
end

function canonicalPath = canonicalResultPath(filePath, parentFolder)
% Resolve an output through its existing parent without creating the file.
if isfile(filePath)
    canonicalPath = canonicalExistingPath(filePath, "resultFilePath");
    return;
end
canonicalFolder = canonicalExistingPath( ...
    parentFolder, "resultFilePath parent folder");
[~, fileName, extension] = fileparts(filePath);
canonicalPath = fullfile( ...
    canonicalFolder, string(fileName) + string(extension));
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
    error("runPlanningRequest:InvalidObject", ...
        "%s must be one JSON object.", fieldName);
end
end

function requireFields(value, requiredNames, fieldName)
% Report every missing required field in one deterministic error.
missingNames = requiredNames(~isfield(value, cellstr(requiredNames)));
if ~isempty(missingNames)
    error("runPlanningRequest:MissingFields", ...
        "%s is missing required fields: %s.", ...
        fieldName, strjoin(missingNames, ", "));
end
end

function state = normalizeState(value, fieldName)
% Normalize required endpoint fields and optional zero-order derivatives.
requireScalarStruct(value, fieldName);
requireFields(value, ["time_s", "position_deg"], fieldName);
validateattributes(value.time_s, {'numeric'}, ...
    {'real', 'finite', 'scalar'}, "runPlanningRequest", ...
    fieldName + ".time_s");
state = value;
state.time_s = double(value.time_s);
state.position_deg = normalizePair( ...
    value.position_deg, fieldName + ".position_deg", false);
optionalPairNames = ["velocity_deg_s", "acceleration_deg_s2"];
for name = optionalPairNames
    if isfield(value, name) && ~isempty(value.(name))
        state.(name) = normalizePair( ...
            value.(name), fieldName + "." + name, false);
    end
end
end

function limits = normalizeLimits(value)
% Normalize the three required physical pairs and optional workspace bounds.
requireScalarStruct(value, "request.limits");
requiredNames = ["maxVelocity_deg_s", "maxAcceleration_deg_s2", ...
    "maxJerk_deg_s3"];
requireFields(value, requiredNames, "request.limits");
limits = value;
for name = requiredNames
    limits.(name) = normalizePair( ...
        value.(name), "request.limits." + name, true);
end
intervalNames = ["azimuthInterval_deg", "elevationInterval_deg"];
for name = intervalNames
    if isfield(value, name) && ~isempty(value.(name))
        interval = normalizePair( ...
            value.(name), "request.limits." + name, false);
        if interval(2) <= interval(1)
            error("runPlanningRequest:InvalidWorkspaceInterval", ...
                "request.limits.%s must be an increasing [lower upper] pair.", ...
                name);
        end
        limits.(name) = interval;
    end
end
end

function pair = normalizePair(value, fieldName, mustBePositive)
% Normalize one finite [azimuth elevation] pair with an optional positivity rule.
validateattributes(value, {'numeric'}, ...
    {'real', 'finite', 'vector', 'numel', 2}, ...
    "runPlanningRequest", fieldName);
pair = reshape(double(value), 1, 2);
if mustBePositive && any(pair <= 0)
    error("runPlanningRequest:NonpositiveLimit", ...
        "%s values must both be positive.", fieldName);
end
end

function obstacles = createObstacles(obstacleInput)
% Convert wire-format keyframes through the canonical public constructor.
if isnumeric(obstacleInput) && isempty(obstacleInput)
    obstacles = obstacleAvoidance.obstacles.combineObstacles();
    return;
end
if ~isstruct(obstacleInput)
    error("runPlanningRequest:InvalidObstacles", ...
        "request.obstacles must be a JSON array of objects or [].");
end
obstacleCells = cell(numel(obstacleInput), 1);
for obstacleIndex = 1:numel(obstacleInput)
    obstacle = obstacleInput(obstacleIndex);
    context = "request.obstacles(" + obstacleIndex + ")";
    requireFields(obstacle, ["name", "safetyMargin_deg", "keyframes"], ...
        context);
    obstacleName = normalizeScalarText(obstacle.name, context + ".name");
    if strlength(strtrim(obstacleName)) == 0
        error("runPlanningRequest:EmptyObstacleName", ...
            "%s.name must be nonempty scalar text.", context);
    end
    validateattributes(obstacle.safetyMargin_deg, {'numeric'}, ...
        {'real', 'finite', 'scalar', 'nonnegative'}, ...
        "runPlanningRequest", context + ".safetyMargin_deg");
    keyframes = obstacle.keyframes;
    if ~isstruct(keyframes) || isempty(keyframes)
        error("runPlanningRequest:InvalidObstacleKeyframes", ...
            "%s.keyframes must contain at least one JSON object.", context);
    end
    time_s = zeros(numel(keyframes), 1);
    azimuthBySlice_deg = cell(numel(keyframes), 1);
    elevationBySlice_deg = cell(numel(keyframes), 1);
    for sampleIndex = 1:numel(keyframes)
        keyframeContext = context + ".keyframes(" + sampleIndex + ")";
        requireFields(keyframes(sampleIndex), ...
            ["time_s", "vertices_deg"], keyframeContext);
        validateattributes(keyframes(sampleIndex).time_s, {'numeric'}, ...
            {'real', 'finite', 'scalar'}, "runPlanningRequest", ...
            keyframeContext + ".time_s");
        rawVertices_deg = keyframes(sampleIndex).vertices_deg;
        isValidVertexArray = isnumeric(rawVertices_deg) && ...
            isreal(rawVertices_deg) && ismatrix(rawVertices_deg) && ...
            size(rawVertices_deg, 2) == 2 && ...
            size(rawVertices_deg, 1) >= 3 && ...
            all(isfinite(rawVertices_deg), "all");
        if ~isValidVertexArray
            error("runPlanningRequest:InvalidObstacleVertices", ...
                "%s.vertices_deg must be a finite N-by-2 numeric array " + ...
                "with N >= 3.", keyframeContext);
        end
        vertices_deg = double(rawVertices_deg);
        time_s(sampleIndex) = double(keyframes(sampleIndex).time_s);
        azimuthBySlice_deg{sampleIndex} = vertices_deg(:, 1);
        elevationBySlice_deg{sampleIndex} = vertices_deg(:, 2);
    end
    if any(diff(time_s) <= 0)
        error("runPlanningRequest:InvalidObstacleTime", ...
            "%s keyframe times must be strictly increasing.", context);
    end
    obstacleCells{obstacleIndex} = ...
        obstacleAvoidance.obstacles.createObstacle( ...
        obstacleName, time_s, azimuthBySlice_deg, ...
        elevationBySlice_deg, double(obstacle.safetyMargin_deg));
end
obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacleCells);
end

function projection = projectPlannerResult(result)
% Retain browser-relevant public fields without exporting solver internals.
resolvedOptions = result.Options;
if isfield(resolvedOptions, "CancellationCheckFcn")
    resolvedOptions.CancellationCheckFcn = [];
end
projection = struct( ...
    "Success", result.Success, ...
    "Message", result.Message, ...
    "TerminationReason", result.TerminationReason, ...
    "Options", resolvedOptions, ...
    "Inputs", struct( ...
        "initialState", result.Inputs.initialState, ...
        "goalState", result.Inputs.goalState, ...
        "limits", result.Inputs.limits), ...
    "SelectedSeedIndex", result.SelectedSeedIndex, ...
    "SelectedSeed_deg", result.SelectedSeed_deg, ...
    "time_s", result.time_s, ...
    "position_deg", result.position_deg, ...
    "velocity_deg_s", result.velocity_deg_s, ...
    "acceleration_deg_s2", result.acceleration_deg_s2, ...
    "jerk_deg_s3", result.jerk_deg_s3, ...
    "ArrivalTime_s", result.ArrivalTime_s, ...
    "TrajectoryDuration_s", result.TrajectoryDuration_s, ...
    "GoalHorizon_s", result.GoalHorizon_s, ...
    "ElapsedPlanningTime_s", result.ElapsedPlanningTime_s, ...
    "SearchDiagnostics", projectSearchDiagnostics(result));
end

function projection = projectSearchDiagnostics(result)
% Preserve stable counts, seed summaries, timing, and bounded plot evidence.
diagnostics = result.SearchDiagnostics;
seedSummaries = diagnostics.SeedSummaries;
seedFieldNames = ["SeedIndex", "SeedSource", "OptimizerFeasible", ...
    "ValidationPassed", "CollisionFree", "CollisionResolved", ...
    "MinimumClearance_deg", "UnresolvedIntervalCount", ...
    "ArrivalTime_s", "MotionDuration_s", "MotionLength_deg", ...
    "IntegratedSquaredJerk_deg2_s5", "MaximumConstraintViolation", ...
    "SeedPlanningElapsedTime_s", "TerminationReason", "Message"];
projectedSeeds = projectStructFields(seedSummaries, seedFieldNames);
projection = struct( ...
    "TerminationReason", diagnostics.TerminationReason, ...
    "AttemptedSeedCount", diagnostics.AttemptedSeedCount, ...
    "ValidatedCandidateCount", diagnostics.ValidatedCandidateCount, ...
    "BestPartialSeedIndex", diagnostics.BestPartialSeedIndex, ...
    "FirstValidatedMotionTime_s", ...
        diagnostics.FirstValidatedMotionTime_s, ...
    "SeedGenerationElapsedTime_s", ...
        diagnostics.SeedGenerationElapsedTime_s, ...
    "SeedSummaries", {projectedSeeds}, ...
    "StageTiming", diagnostics.StageTiming, ...
    "Grid", projectSearchGrid(diagnostics.Grid));
if isempty(projection.Grid.BestPartialRoute_deg) && ...
        diagnostics.BestPartialSeedIndex >= 1 && ...
        diagnostics.BestPartialSeedIndex <= numel(result.Seeds)
    projection.Grid.BestPartialRoute_deg = ...
        result.Seeds(diagnostics.BestPartialSeedIndex).position_deg;
end
end

function projection = projectSearchGrid(grid)
% Return one exact display schema even when route search was not required.
projection = struct( ...
    "Bounds_deg", [NaN NaN NaN NaN], ...
    "AcceptedEdges_deg", zeros(0, 4), ...
    "RejectedEdges_deg", zeros(0, 4), ...
    "ExploredNodes_deg", zeros(0, 2), ...
    "FrontierNodes_deg", zeros(0, 2), ...
    "BestPartialRoute_deg", zeros(0, 2), ...
    "Start_deg", [NaN NaN], ...
    "Goal_deg", [NaN NaN], ...
    "NodeCount", 0, ...
    "ExpandedCount", 0, ...
    "RejectedTransitionCount", 0, ...
    "GeneratedSeedCount", 0, ...
    "TraceDownsampleRule", "");
if ~isstruct(grid) || ~isscalar(grid)
    return;
end
fieldNames = string(fieldnames(projection));
for name = reshape(fieldNames, 1, [])
    if isfield(grid, name)
        projection.(name) = grid.(name);
    end
end
end

function projection = projectStructFields(records, fieldNames)
% Copy records into cells so JSON collections stay arrays at every cardinality.
projection = cell(numel(records), 1);
for recordIndex = 1:numel(records)
    projectedRecord = struct();
    for name = reshape(fieldNames, 1, [])
        projectedRecord.(name) = records(recordIndex).(name);
    end
    projection{recordIndex} = projectedRecord;
end
end

function projection = projectObstacles(obstacles)
% Export original and protected keyframes for dependency-free animation.
template = struct( ...
    "Name", "", ...
    "time_s", zeros(0, 1), ...
    "status", strings(0, 1), ...
    "SafetyMargin_deg", 0, ...
    "OriginalVerticesByTime_deg", {cell(0, 1)}, ...
    "ProtectedVerticesByTime_deg", {cell(0, 1)});
projection = cell(numel(obstacles), 1);
for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacles(obstacleIndex);
    sampleCount = numel(obstacle.time_s);
    originalVertices = cell(sampleCount, 1);
    protectedVertices = cell(sampleCount, 1);
    for sampleIndex = 1:sampleCount
        originalVertices{sampleIndex} = [ ...
            obstacle.originalAz_deg{sampleIndex}, ...
            obstacle.originalEl_deg{sampleIndex}];
        protectedVertices{sampleIndex} = [ ...
            obstacle.az_deg{sampleIndex}, obstacle.el_deg{sampleIndex}];
    end
    projectedObstacle = template;
    projectedObstacle.Name = obstacle.targetName;
    projectedObstacle.time_s = obstacle.time_s;
    projectedObstacle.status = obstacle.status;
    projectedObstacle.SafetyMargin_deg = ...
        obstacle.safetyMargin_deg;
    projectedObstacle.OriginalVerticesByTime_deg = ...
        originalVertices;
    projectedObstacle.ProtectedVerticesByTime_deg = ...
        protectedVertices;
    projection{obstacleIndex} = projectedObstacle;
end
end

function deleteFileIfPresent(filePath)
% Remove only the adapter-owned sibling temporary file after a failed write.
if isfile(filePath)
    delete(filePath);
end
end
