function diagnosisBundle = createDiagnosisBundle(request, result, independentValidation, diagnosis)
%% Section 0: Header & Readme
% SYNTAX
%   diagnosisBundle = offlineSandbox.createDiagnosisBundle( ...
%       request, result, independentValidation, diagnosis)
%**************************************************************************
% PURPOSE
%   - Create the handle-free MATLAB diagnosis record saved by the HTML
%     sandbox after a live planning request.
%   - Preserve exact canonical planner data rather than the bounded browser
%     projection used for display.
%**************************************************************************
% INPUTS
%   - request (scalar struct)
%       Validated offlineSandboxRequest/v1 record used for the planner call.
%   - result (scalar struct)
%       Stable unprojected public planner result for that exact request.
%   - independentValidation (scalar struct)
%       Public validation record associated with result.
%   - diagnosis (optional scalar struct): the planner's second output.
%**************************************************************************
% OUTPUTS
%   - diagnosisBundle (scalar struct)
%       obstacleAvoidanceSandboxDiagnosis-v2 record compatible with the
%       diagnostic workflow used by the MATLAB sandbox. Function handles are
%       removed so the bundle can be inspected and shared safely.
%**************************************************************************
% UNITS
%   - Positions and polygon vertices are [azimuth elevation] in degrees.
%   - Time is seconds. Derivatives use deg/s, deg/s^2, and deg/s^3.
%**************************************************************************

%% Section 1: Validate And Sanitize Planner Records

if nargin < 4, diagnosis = struct(); end
if ~isstruct(request) || ~isscalar(request) || ~isfield(request, "requestId") || ~isfield(request, "obstacles")
    error("createDiagnosisBundle:InvalidRequest", "request must be one validated offline-sandbox request record.");
end
if ~isstruct(result) || ~isscalar(result) || ~all(isfield(result, {'Inputs', 'Options', 'Success', 'TerminationReason'}))
    error("createDiagnosisBundle:InvalidResult", "result must be one stable public planner result.");
end
if ~isstruct(independentValidation) || ~isscalar(independentValidation)
    error("createDiagnosisBundle:InvalidValidation", "independentValidation must be one scalar validation record.");
end

sanitizedResult = result;
sanitizedResult.Options = removeCallbacks(result.Options);
plannerOptions = sanitizedResult.Options;
plannerInputs  = sanitizedResult.Inputs;

%% Section 2: Preserve Browser And Canonical Scene Geometry

rawObstacleStrokes_deg       = cell(numel(request.obstacles), 1);
polygonObstaclePositions_deg = cell(numel(request.obstacles), 1);
% Process each obstacle needed by the sandbox workflow.
for obstacleIndex = 1:numel(request.obstacles)
    keyframes = request.obstacles(obstacleIndex).keyframes;
    rawObstacleStrokes_deg{obstacleIndex} = keyframes(1).vertices_deg;
    polygonObstaclePositions_deg{obstacleIndex} = keyframes(1).vertices_deg;
end

scene = struct("StartPosition_deg", request.initialState.position_deg, ...
    "GoalPosition_deg", request.goalState.position_deg, ...
    "RawObstacleStrokes_deg", {rawObstacleStrokes_deg}, ...
    "LineObstaclePositions_deg", {cell(0, 1)}, ...
    "PolygonObstaclePositions_deg", ...
        {polygonObstaclePositions_deg}, ...
    "PolygonMotionVectors_deg", NaN(numel(request.obstacles), 2), ...
    "PolygonMotionProfiles", ...
        repmat("keyframes", numel(request.obstacles), 1), "ObstacleKeyframes", {extractObstacleKeyframes(request.obstacles)}, "CanonicalObstacles", plannerInputs.obstacles, "ResolvedControls", struct("Limits", plannerInputs.limits, "Options", plannerOptions));

%% Section 3: Assemble Reproduction Metadata

environment = struct("MATLABVersion", string(version), ...
    "MATLABRelease", string(version('-release')), ...
    "Computer", string(computer), ...
    "WorkingDirectory", string(pwd));
reproduction = struct("LoadCommand", ...
        "loaded = load(filePath, 'diagnosisBundle');", "PlannerCommand", "[reproduced, reproducedDiagnosis] = obstacleAvoidance.planTrajectory(" + "diagnosisBundle.PlannerInputs.obstacles, " + "diagnosisBundle.PlannerInputs.initialState, " + "diagnosisBundle.PlannerInputs.goalState, " + "diagnosisBundle.PlannerInputs.limits, " + "diagnosisBundle.PlannerOptions);", "ValidationCommand", "reproducedValidation = " + "obstacleAvoidance.validateTrajectory(reproduced);");
exportRequest = struct("RequestId", string(request.requestId), ...
    "HasCompleteScene", true, ...
    "PlannerInputs", plannerInputs, ...
    "PlannerOptions", plannerOptions);
plannerLog = [ ...
    "[HTML Sandbox]"; ...
    "RequestId: " + string(request.requestId); ...
    "Success: " + string(logical(sanitizedResult.Success)); ...
    "TerminationReason: " + string(sanitizedResult.TerminationReason); ...
    "Message: " + string(sanitizedResult.Message)];

diagnosisBundle = struct("Format", "obstacleAvoidanceSandboxDiagnosis-v2", ...
    "CreatedUTC", string(datetime('now', 'TimeZone', 'UTC', ...
        'Format', 'yyyy-MM-dd''T''HH:mm:ss.SSSXXX')), ...
    "Mode", "goal", ...
    "PlanningState", "completed", ...
    "HasPlannerResult", true, ...
    "Environment", environment, ...
    "SandboxOptions", struct("Source", "htmlSandbox"), ...
    "Scene", scene, ...
    "ExportRequest", exportRequest, ...
    "PlannerInputs", plannerInputs, ...
    "PlannerOptions", plannerOptions, ...
    "Result", sanitizedResult, ...
    "Diagnosis", diagnosis, ...
    "IndependentValidation", independentValidation, ...
    "Status", string(sanitizedResult.Message), ...
    "PlannerLog", plannerLog, ...
    "Reproduction", reproduction);

end

%% Section 4: Local Functions

function value = removeCallbacks(value)
    % Remove function handles recursively without changing other diagnostic data.
    if isa(value, "function_handle")
        value = [];
    elseif isstruct(value)
        fieldNames = string(fieldnames(value));
        % Process each element needed by the sandbox workflow.
        for elementIndex = 1:numel(value)
            % Process each field name needed by the sandbox workflow.
            for fieldName = reshape(fieldNames, 1, [])
                value(elementIndex).(fieldName) = removeCallbacks(value(elementIndex).(fieldName));
            end
        end
    elseif iscell(value)
        % Process each element needed by the sandbox workflow.
        for elementIndex = 1:numel(value)
            value{elementIndex} = removeCallbacks(value{elementIndex});
        end
    end
end

function keyframes = extractObstacleKeyframes(obstacles)
    % Preserve the exact browser motion history without inferring profile controls.
    keyframes = cell(numel(obstacles), 1);
    % Process each obstacle needed by the sandbox workflow.
    for obstacleIndex = 1:numel(obstacles)
        keyframes{obstacleIndex} = obstacles(obstacleIndex).keyframes;
    end
end
