function validation = validateAzElExampleResult( ...
        result, scenarioLabel, requirements)
%% Section 0: Header & Readme
% SYNTAX
%   validation = validateAzElExampleResult(result, scenarioLabel)
%   validation = validateAzElExampleResult( ...
%       result, scenarioLabel, requirements)
%**************************************************************************
% PURPOSE
%   - Independently validate one maintained example result.
%   - Validate stable diagnostics for expected planning failures.
%**************************************************************************
% INPUTS
%   - result (scalar planAzElMotion result)
%   - scenarioLabel (scalar text)
%   - requirements (scalar struct, optional; default struct())
%       ExpectedSuccess defaults true. RequireDirectBlocked defaults false.
%**************************************************************************
% OUTPUTS
%   - validation (scalar struct)
%       Pass state, message, trajectory validation, and diagnostic checks.
%**************************************************************************
% UNITS
%   - Direct-route collision probes use degrees and seconds.
%**************************************************************************

%% Section 1: Resolve The Example Contract

if nargin < 3 || isempty(requirements)
    requirements = struct();
end
if ~isstruct(result) || ~isscalar(result) || ...
        ~isstruct(requirements) || ~isscalar(requirements)
    error("validateAzElExampleResult:InvalidInput", ...
        "result and requirements must be scalar structs.");
end
scenarioLabel = string(scenarioLabel);
if ~isscalar(scenarioLabel)
    error("validateAzElExampleResult:InvalidLabel", ...
        "scenarioLabel must be scalar text.");
end
expectedSuccess = fieldOrDefault(requirements, "ExpectedSuccess", true);
requireDirectBlocked = fieldOrDefault( ...
    requirements, "RequireDirectBlocked", false);
expectedSuccess = azElInternal.normalizeLogicalScalar( ...
    expectedSuccess, "ExpectedSuccess", ...
    "validateAzElExampleResult:InvalidExpectedSuccess");
requireDirectBlocked = azElInternal.normalizeLogicalScalar( ...
    requireDirectBlocked, "RequireDirectBlocked", ...
    "validateAzElExampleResult:InvalidRequireDirectBlocked");
requiredFields = {'Success', 'Message', 'TerminationReason', 'Inputs', ...
    'Options', 'Seeds', 'SeedSummaries', 'SearchDiagnostics'};
schemaIsStable = all(isfield(result, requiredFields));

%% Section 2: Validate Motion Or Expected Failure

trajectoryValidation = emptyTrajectoryValidation();
if schemaIsStable && result.Success
    trajectoryValidation = validateAzElTrajectory(result);
end
diagnosticsAreConsistent = schemaIsStable && ...
    numel(result.SeedSummaries) == numel(result.Seeds) && ...
    diagnosticCountsAreValid(result.SearchDiagnostics);
recognizedFailure = false;
if schemaIsStable && ~result.Success
    recognizedReasons = ["endpointBlocked", "dynamicEndpointInfeasible", ...
        "endpointOutsideWorkspace", "planningTimeLimit", ...
        "noValidatedSeed", "targetLeftAzElFrame"];
    recognizedFailure = any(result.TerminationReason == recognizedReasons) && ...
        strlength(string(result.Message)) > 0 && diagnosticsAreConsistent;
end

%% Section 3: Validate The Direct-Route Requirement

directRouteBlocked = false;
if schemaIsStable && requireDirectBlocked
    directRouteBlocked = directRouteHasCollision(result);
end
directRequirementPassed = ~requireDirectBlocked || directRouteBlocked;
if expectedSuccess
    passed = schemaIsStable && result.Success && ...
        trajectoryValidation.Passed && diagnosticsAreConsistent && ...
        directRequirementPassed;
else
    passed = schemaIsStable && ~result.Success && recognizedFailure && ...
        directRequirementPassed;
end
if passed
    message = scenarioLabel + " passed independent example validation.";
else
    message = scenarioLabel + " failed example validation. " + ...
        "Success=" + string(result.Success) + ...
        ", reason=" + string(result.TerminationReason) + ...
        ", diagnostics=" + string(diagnosticsAreConsistent) + ...
        ", directBlocked=" + string(directRouteBlocked) + ".";
end
validation = struct( ...
    "Passed", passed, ...
    "Message", message, ...
    "ExpectedSuccess", expectedSuccess, ...
    "SchemaIsStable", schemaIsStable, ...
    "DiagnosticsAreConsistent", diagnosticsAreConsistent, ...
    "RecognizedFailure", recognizedFailure, ...
    "DirectRouteBlocked", directRouteBlocked, ...
    "TrajectoryValidation", trajectoryValidation);
end

%% Section 4: Local Functions

function value = fieldOrDefault(record, fieldName, defaultValue)
% PURPOSE
%   - Read one optional validation requirement.
value = defaultValue;
if isfield(record, fieldName) && ~isempty(record.(fieldName))
    value = record.(fieldName);
end
end

function valid = diagnosticCountsAreValid(searchDiagnostics)
% PURPOSE
%   - Check bounded trace arrays and complete search counts.
valid = isstruct(searchDiagnostics) && isscalar(searchDiagnostics) && ...
    isfield(searchDiagnostics, "Grid") && ...
    isfield(searchDiagnostics, "TerminationReason");
if ~valid
    return;
end
gridRecord = searchDiagnostics.Grid;
countNames = ["NodeCount", "VisibilityEdgeCount", "ExpandedCount", ...
    "RejectedTransitionCount", "GeneratedSeedCount"];
for name = countNames
    if isfield(gridRecord, name)
        value = gridRecord.(name);
        valid = valid && isnumeric(value) && isscalar(value) && ...
            isfinite(value) && value >= 0;
    end
end
if isfield(gridRecord, "ExploredNodes_deg")
    explored_deg = gridRecord.ExploredNodes_deg;
    valid = valid && size(explored_deg, 2) == 2 && ...
        all(isfinite(explored_deg), "all");
end
end

function blocked = directRouteHasCollision(result)
% PURPOSE
%   - Probe the complete timed direct line independently of seed selection.
sampleCount = 401;
initialState = result.Inputs.initialState;
goalState = result.Inputs.goalState;
sampleTime_s = linspace( ...
    initialState.time_s, goalState.time_s, sampleCount).';
goalPosition_deg = goalPositionAtTime(goalState, goalState.time_s);
fraction = linspace(0, 1, sampleCount).';
position_deg = initialState.position_deg + fraction .* ...
    (goalPosition_deg - initialState.position_deg);
occupied = queryAzElTimeObstacle( ...
    result.Inputs.obstacles, position_deg(:, 1), ...
    position_deg(:, 2), sampleTime_s);
blocked = any(occupied);
end

function position_deg = goalPositionAtTime(goalState, time_s)
% PURPOSE
%   - Evaluate fixed or sampled goal geometry for direct-route validation.
if isfield(goalState, "targetTime_s") && ~isempty(goalState.targetTime_s)
    position_deg = interp1( ...
        goalState.targetTime_s, goalState.targetPosition_deg, ...
        time_s, goalState.InterpolationMethod);
else
    position_deg = goalState.position_deg;
end
end

function validation = emptyTrajectoryValidation()
% PURPOSE
%   - Provide an explicit empty value when no trajectory can be validated.
validation = struct( ...
    "Passed", false, ...
    "Message", "No successful trajectory was available.");
end
