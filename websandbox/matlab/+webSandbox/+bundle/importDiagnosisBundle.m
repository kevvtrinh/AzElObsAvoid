function scene = importDiagnosisBundle(filePath)
%% Section 0: Header & Readme
% SYNTAX
%   scene = webSandbox.bundle.importDiagnosisBundle(filePath)
%**************************************************************************
% PURPOSE
%   - Load a sandbox diagnosis-v2 bundle into the web scene schema.
%**************************************************************************
% INPUTS
%   - filePath (scalar text)
%       Existing .mat file containing diagnosisBundle from the MATLAB sandbox.
%**************************************************************************
% OUTPUTS
%   - scene (scalar struct)
%       JSON-safe trajectory request suitable for the web sandbox.
%**************************************************************************
% UNITS
%   - Positions are degrees and time is seconds.
%**************************************************************************

%% Section 1: Load And Validate The Sandbox Record

filePath = string(filePath);
if ~isscalar(filePath) || strlength(filePath) == 0 || ~isfile(filePath)
    error("webSandbox:importDiagnosisBundle:MissingFile", ...
        "filePath must name an existing sandbox .mat bundle.");
end
loaded = load(char(filePath), "diagnosisBundle");
if ~isfield(loaded, "diagnosisBundle") || ~isstruct(loaded.diagnosisBundle) || ...
        ~isscalar(loaded.diagnosisBundle)
    error("webSandbox:importDiagnosisBundle:MissingBundle", ...
        "The MAT file must contain one scalar diagnosisBundle.");
end
bundle = loaded.diagnosisBundle;
if ~isfield(bundle, "Format") || ...
        string(bundle.Format) ~= "obstacleAvoidanceSandboxDiagnosis-v2"
    error("webSandbox:importDiagnosisBundle:UnsupportedFormat", ...
        "Only obstacleAvoidanceSandboxDiagnosis-v2 bundles are supported.");
end
inputs = selectPlannerInputs(bundle);
requiredFields = ["obstacles", "initialState", "goalState", "limits"];
if ~all(isfield(inputs, requiredFields))
    error("webSandbox:importDiagnosisBundle:MissingPlannerInputs", ...
        "The bundle does not contain a complete goal-planning request.");
end

%% Section 2: Convert Canonical Obstacles To Browser Polygons

obstacles = inputs.obstacles;
descriptors = repmat(struct( ...
    "name", "", "time_s", zeros(0, 1), "safetyMargin_deg", 0, ...
    "slices", struct("vertices_deg", zeros(0, 2))), numel(obstacles), 1);
for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacles(obstacleIndex);
    descriptors(obstacleIndex) = convertObstacle(obstacle, obstacleIndex);
end
plannerOptions = struct();
if isfield(bundle, "PlannerOptions") && isstruct(bundle.PlannerOptions)
    plannerOptions = removeFunctionFields(bundle.PlannerOptions);
end
scene = struct( ...
    "mode", "trajectory", ...
    "obstacles", descriptors, ...
    "initialState", inputs.initialState, ...
    "goalState", inputs.goalState, ...
    "limits", inputs.limits, ...
    "plannerOptions", plannerOptions);
end

function inputs = selectPlannerInputs(bundle)
% Prefer normalized planner inputs, with the pre-run export as a fallback.
inputs = struct();
if isfield(bundle, "PlannerInputs") && isstruct(bundle.PlannerInputs) && ...
        isscalar(bundle.PlannerInputs)
    inputs = bundle.PlannerInputs;
end
if isempty(fieldnames(inputs)) && isfield(bundle, "ExportRequest") && ...
        isstruct(bundle.ExportRequest) && isscalar(bundle.ExportRequest) && ...
        isfield(bundle.ExportRequest, "PlannerInputs")
    inputs = bundle.ExportRequest.PlannerInputs;
end
end

function descriptor = convertObstacle(obstacle, obstacleIndex)
% Preserve original vertices and the one explicit safety margin from the bundle.
required = ["targetName", "time_s", "originalAz_deg", "originalEl_deg", ...
    "safetyMargin_deg"];
if ~isstruct(obstacle) || ~isscalar(obstacle) || ~all(isfield(obstacle, required))
    error("webSandbox:importDiagnosisBundle:InvalidObstacle", ...
        "Canonical obstacle %d is incomplete.", obstacleIndex);
end
time_s = double(obstacle.time_s(:));
sliceCount = numel(time_s);
if ~iscell(obstacle.originalAz_deg) || ~iscell(obstacle.originalEl_deg) || ...
        numel(obstacle.originalAz_deg) ~= sliceCount || ...
        numel(obstacle.originalEl_deg) ~= sliceCount
    error("webSandbox:importDiagnosisBundle:InvalidHistory", ...
        "Canonical obstacle %d has an invalid original-geometry history.", ...
        obstacleIndex);
end
slices = repmat(struct("vertices_deg", zeros(0, 2)), sliceCount, 1);
for sliceIndex = 1:sliceCount
    vertices_deg = [double(obstacle.originalAz_deg{sliceIndex}(:)), ...
        double(obstacle.originalEl_deg{sliceIndex}(:))];
    if any(~isfinite(vertices_deg), "all") || size(vertices_deg, 1) < 3
        error("webSandbox:importDiagnosisBundle:UnsupportedGeometry", ...
            "Obstacle %d slice %d is not one finite polygon.", ...
            obstacleIndex, sliceIndex);
    end
    slices(sliceIndex).vertices_deg = vertices_deg;
end
descriptor = struct( ...
    "name", string(obstacle.targetName), ...
    "time_s", time_s, ...
    "safetyMargin_deg", double(obstacle.safetyMargin_deg), ...
    "slices", slices);
end

function output = removeFunctionFields(input)
% JSON cannot carry callback handles; their omission changes no scene meaning.
output = input;
fieldNames = string(fieldnames(input));
for fieldName = reshape(fieldNames, 1, [])
    if isa(input.(fieldName), "function_handle")
        output = rmfield(output, fieldName);
    end
end
end
