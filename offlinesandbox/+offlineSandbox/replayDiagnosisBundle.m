function [response, reproducedBundle] = replayDiagnosisBundle(bundleFilePath, resultFilePath)
%% Section 0: Header & Readme
% SYNTAX
%   response = offlineSandbox.replayDiagnosisBundle( ...
%       bundleFilePath, resultFilePath)
%   [response, reproducedBundle] = ...
%       offlineSandbox.replayDiagnosisBundle( ...
%       bundleFilePath, resultFilePath)
%**************************************************************************
% PURPOSE
%   - Recreate and run the canonical planner request stored in one diagnosis
%     bundle through the maintained offline-sandbox request adapter.
%**************************************************************************
% INPUTS
%   - bundleFilePath (nonempty scalar text)
%       Existing MAT file containing one scalar diagnosisBundle variable in
%       obstacleAvoidanceSandboxDiagnosis-v2 format.
%   - resultFilePath (nonempty scalar text)
%       Destination for the browser-oriented offlineSandboxResult/v1 JSON.
%**************************************************************************
% OUTPUTS
%   - response (scalar struct)
%       Browser result from a fresh planner run of the bundled inputs.
%   - reproducedBundle (scalar struct)
%       Fresh handle-free diagnosis bundle for the reproduced result.
%**************************************************************************
% UNITS
%   - Positions and vertices are [x y] in coordinate units. Time is
%     seconds, and obstacle motion is preserved as sampled positions.
%**************************************************************************

%% Section 1: Load & Validate The Diagnosis Bundle

bundleFilePath = normalizeInputPath(bundleFilePath);
if ~isfile(bundleFilePath)
    error("replayDiagnosisBundle:BundleFileNotFound", "bundleFilePath does not exist: %s", bundleFilePath);
end
try
    loaded = load(bundleFilePath, "diagnosisBundle");
catch exception
    error("replayDiagnosisBundle:InvalidMatFile", "Could not load diagnosisBundle from '%s': %s", bundleFilePath, exception.message);
end
if ~isfield(loaded, "diagnosisBundle") || ~isstruct(loaded.diagnosisBundle) || ~isscalar(loaded.diagnosisBundle)
    error("replayDiagnosisBundle:MissingDiagnosisBundle", "The MAT file must contain one scalar diagnosisBundle variable.");
end
diagnosisBundle      = loaded.diagnosisBundle;
requiredBundleFields = ["Format", "PlannerInputs", "PlannerOptions"];
requireFields(diagnosisBundle, requiredBundleFields, "diagnosisBundle");
if string(diagnosisBundle.Format) ~= "obstacleAvoidanceSandboxDiagnosis-v2"
    error("replayDiagnosisBundle:UnsupportedFormat", "diagnosisBundle.Format must be " + "'obstacleAvoidanceSandboxDiagnosis-v2'.");
end
plannerInputs = diagnosisBundle.PlannerInputs;
if ~isstruct(plannerInputs) || ~isscalar(plannerInputs)
    error("replayDiagnosisBundle:InvalidPlannerInputs", "diagnosisBundle.PlannerInputs must be one scalar structure.");
end
requireFields(plannerInputs, ["obstacles", "initialState", "goalState", "limits"], "diagnosisBundle.PlannerInputs");
if ~isstruct(diagnosisBundle.PlannerOptions) || ~isscalar(diagnosisBundle.PlannerOptions)
    error("replayDiagnosisBundle:InvalidPlannerOptions", "diagnosisBundle.PlannerOptions must be one scalar structure.");
end

%% Section 2: Recreate The Canonical Browser Request

plannerOptions = removeFunctionHandles(diagnosisBundle.PlannerOptions);
request        = struct("schemaVersion", "offlineSandboxRequest/v1", ...
    "requestId", "bundle-replay-" + string(java.util.UUID.randomUUID()), ...
    "obstacles", createWireObstacles(plannerInputs.obstacles), ...
    "initialState", plannerInputs.initialState, ...
    "goalState", plannerInputs.goalState, ...
    "limits", plannerInputs.limits, ...
    "options", plannerOptions);

requestFilePath = string(tempname) + ".json";
requestCleanup  = onCleanup(@() deleteFileIfPresent(requestFilePath));
writeJsonFile(requestFilePath, request);

%% Section 3: Run Through The Maintained Adapter

[response, reproducedBundle] = offlineSandbox.runPlanningRequest(requestFilePath, resultFilePath);
clear requestCleanup;

end

%% Section 4: Local Functions

function filePath = normalizeInputPath(value)
    % Normalize one explicit bundle path without changing its location.
    isScalarText = (isstring(value) && isscalar(value)) || (ischar(value) && isrow(value));
    if ~isScalarText || strlength(strtrim(string(value))) == 0
        error("replayDiagnosisBundle:InvalidBundleFilePath", "bundleFilePath must be nonempty scalar text.");
    end
    filePath = string(value);
end

function requireFields(value, requiredNames, context)
    % Report every missing reproduction field together.
    missingNames = requiredNames(~isfield(value, cellstr(requiredNames)));
    if ~isempty(missingNames)
        error("replayDiagnosisBundle:MissingFields", "%s is missing required fields: %s.", context, strjoin(missingNames, ", "));
    end
end

function obstacleInput = createWireObstacles(obstacles)
    % Convert original canonical histories back to the margin-free wire format.
    if isempty(obstacles)
        obstacleInput = struct.empty(0, 1);
        return;
    end
    if ~isstruct(obstacles)
        error("replayDiagnosisBundle:InvalidObstacles", "diagnosisBundle.PlannerInputs.obstacles must be a structure array.");
    end
    obstacleTemplate = struct("name", "", ...
        "safetyMargin_units", 0, ...
        "keyframes", struct.empty(0, 1));
    obstacleInput = repmat(obstacleTemplate, numel(obstacles), 1);
    requiredNames = ["targetName", "time_s", "originalX_units", ...
        "originalY_units", "safetyMargin_units"];
    % Process each obstacle needed by the sandbox workflow.
    for obstacleIndex = 1:numel(obstacles)
        obstacle = obstacles(obstacleIndex);
        context  = "diagnosisBundle.PlannerInputs.obstacles(" + obstacleIndex + ")";
        requireFields(obstacle, requiredNames, context);
        time_s = reshape(double(obstacle.time_s), [], 1);
        if numel(time_s) < 1 || any(~isfinite(time_s)) || any(diff(time_s) <= 0)
            error("replayDiagnosisBundle:InvalidObstacleTime", "%s.time_s must be finite and strictly increasing.", context);
        end
        if numel(obstacle.originalX_units) ~= numel(time_s) || numel(obstacle.originalY_units) ~= numel(time_s)
            error("replayDiagnosisBundle:ObstacleHistorySizeMismatch", "%s original coordinate histories must match time_s.", context);
        end
        keyframeTemplate = struct();
        keyframeTemplate.time_s       = 0;
        keyframeTemplate.vertices_units = zeros(0, 2);
        keyframes = repmat(keyframeTemplate, numel(time_s), 1);
        % Process each sample needed by the sandbox workflow.
        for sampleIndex = 1:numel(time_s)
            x_units   = reshape(double(obstacle.originalX_units{sampleIndex}), [], 1);
            y_units = reshape(double(obstacle.originalY_units{sampleIndex}), [], 1);
            vertices_units  = [x_units, y_units];
            if size(vertices_units, 1) < 3 || any(~isfinite(vertices_units), "all")
                error("replayDiagnosisBundle:InvalidObstacleVertices", "%s original slice %d must have at least three finite " + "[x y] vertices.", context, sampleIndex);
            end
            keyframes(sampleIndex).time_s = time_s(sampleIndex);
            keyframes(sampleIndex).vertices_units = vertices_units;
        end
        obstacleInput(obstacleIndex).name = string(obstacle.targetName);
        obstacleInput(obstacleIndex).safetyMargin_units = double(obstacle.safetyMargin_units);
        obstacleInput(obstacleIndex).keyframes = keyframes;
    end
end

function value = removeFunctionHandles(value)
    % Exclude callbacks from untrusted bundle data before planner invocation.
    if isa(value, "function_handle")
        value = [];
    elseif isstruct(value)
        names = string(fieldnames(value));
        % Process each element needed by the sandbox workflow.
        for elementIndex = 1:numel(value)
            % Process each name needed by the sandbox workflow.
            for name = reshape(names, 1, [])
                value(elementIndex).(name) = removeFunctionHandles(value(elementIndex).(name));
            end
        end
    elseif iscell(value)
        % Process each element needed by the sandbox workflow.
        for elementIndex = 1:numel(value)
            value{elementIndex} = removeFunctionHandles(value{elementIndex});
        end
    end
end

function writeJsonFile(filePath, value)
    % Write one UTF-8 request into an adapter-owned temporary file.
    bytes = unicode2native(char(jsonencode(value)), "UTF-8");
    [fileIdentifier, openMessage] = fopen(filePath, "w", "n");
    if fileIdentifier < 0
        error("replayDiagnosisBundle:RequestFileOpenFailed", "Could not create the temporary request: %s", openMessage);
    end
    try
        writtenByteCount = fwrite(fileIdentifier, bytes, "uint8");
        closeStatus      = fclose(fileIdentifier);
    catch exception
        fclose(fileIdentifier);
        rethrow(exception);
    end
    if writtenByteCount ~= numel(bytes) || closeStatus ~= 0
        error("replayDiagnosisBundle:RequestFileWriteFailed", "Could not write the complete temporary request.");
    end
end

function deleteFileIfPresent(filePath)
    % Remove only the helper-owned temporary JSON file.
    if isfile(filePath)
        delete(filePath);
    end
end
