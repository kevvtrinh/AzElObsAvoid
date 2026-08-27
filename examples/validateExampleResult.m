function validation = validateExampleResult( result, scenarioLabel, requirements)
%% Section 0: Header & Readme
% SYNTAX
%   validation = validateExampleResult(result, scenarioLabel)
%   validation = validateExampleResult( ...
%       result, scenarioLabel, requirements)
%**************************************************************************
% PURPOSE
%   - Independently validate one maintained example result.
%   - Validate stable diagnostics for expected planning failures.
%**************************************************************************
% INPUTS
%   - result (scalar planTrajectory result)
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

%% Section 1: Resolve The Example Requirements

% Requirements state what this example expects. ExpectedSuccess separates a
% valid no-path result from an unexpected planner failure. RequireDirectBlocked
% checks that an obstacle example does not accidentally have a clear direct line.

if nargin < 3 || isempty(requirements)
    requirements = struct();
end
if ~isstruct(result) || ~isscalar(result) || ~isstruct(requirements) || ~isscalar(requirements)
    error("validateExampleResult:InvalidInput", "result and requirements must be scalar structs.");
end
scenarioLabel = string(scenarioLabel);
if ~isscalar(scenarioLabel)
    error("validateExampleResult:InvalidLabel", "scenarioLabel must be scalar text.");
end
expectedSuccess = fieldOrDefault(requirements, "ExpectedSuccess", true);
requireDirectBlocked = fieldOrDefault( requirements, "RequireDirectBlocked", false);
expectedSuccess = obstacleAvoidance.input.normalizeLogicalScalar( ...
    expectedSuccess, "ExpectedSuccess", "validateExampleResult:InvalidExpectedSuccess");
requireDirectBlocked = obstacleAvoidance.input.normalizeLogicalScalar( ...
    requireDirectBlocked, "RequireDirectBlocked", "validateExampleResult:InvalidRequireDirectBlocked");
requiredFields = {'Success', 'Message', 'TerminationReason', 'Inputs', ...
    'Options', 'Seeds', 'SeedSummaries', 'SearchDiagnostics'};
schemaIsStable = all(isfield(result, requiredFields));

%% Section 2: Validate Motion Or Expected Failure

% A successful plan receives full trajectory validation. An expected failure
% receives diagnostic validation instead. Both paths return one validation
% record with a clear message for the example warning.

trajectoryValidation = createEmptyTrajectoryValidation();
if schemaIsStable && result.Success
    trajectoryValidation = obstacleAvoidance.validateTrajectory(result);
end
diagnosticsAreConsistent = schemaIsStable && ...
    numel(result.SeedSummaries) == numel(result.Seeds) && diagnosticCountsAreValid(result.SearchDiagnostics);
recognizedFailure = false;
if schemaIsStable && ~result.Success
    recognizedReasons = ["endpointBlocked", "dynamicEndpointInfeasible", ...
        "endpointOutsideWorkspace", "noValidatedSeed", "targetLeftAzElFrame"];
    recognizedFailure = any(result.TerminationReason == recognizedReasons) && ...
        strlength(string(result.Message)) > 0 && diagnosticsAreConsistent;
end

%% Section 3: Validate The Direct-Route Requirement

% Probe the timed straight line independently. This check confirms that planner
% success did not come from an unintentionally easy scenario setup.

directRouteBlocked = false;
if schemaIsStable && requireDirectBlocked
    directRouteBlocked = directRouteHasCollision(result);
end
directRequirementPassed = ~requireDirectBlocked || directRouteBlocked;
if expectedSuccess
    passed = schemaIsStable && result.Success && ...
        trajectoryValidation.Passed && diagnosticsAreConsistent && directRequirementPassed;
else
    passed = schemaIsStable && ~result.Success && recognizedFailure && directRequirementPassed;
end
if passed
    message = scenarioLabel + " passed independent example validation.";
else
    message = scenarioLabel + " failed example validation. " + ...
        "Success=" + logicalText(result.Success) + ...
        ", reason=" + string(result.TerminationReason) + ...
        ", diagnostics=" + string(diagnosticsAreConsistent) + ", directBlocked=" + string(directRouteBlocked) + ".";
end

function text = logicalText(value)
% Convert one logical status to the text true or false.
if logical(value)
    text = "true";
else
    text = "false";
end
end
validation = struct( ...
    "Passed", passed, ...
    "Message", message, ...
    "ExpectedSuccess", expectedSuccess, ...
    "SchemaIsStable", schemaIsStable, ...
    "DiagnosticsAreConsistent", diagnosticsAreConsistent, ...
    "RecognizedFailure", recognizedFailure, ...
    "DirectRouteBlocked", directRouteBlocked, "TrajectoryValidation", trajectoryValidation);
end


function value = fieldOrDefault(record, fieldName, defaultValue)
% Read one optional validation requirement. Use its default when it is absent.
value = defaultValue;
if isfield(record, fieldName) && ~isempty(record.(fieldName))
    value = record.(fieldName);
end
end

function valid = diagnosticCountsAreValid(searchDiagnostics)
% Check stored trace arrays and complete search counts. A trace can be shortened
% for display, but its total count must still describe the complete search.
valid = isstruct(searchDiagnostics) && isscalar(searchDiagnostics) && ...
    isfield(searchDiagnostics, "Grid") && isfield(searchDiagnostics, "TerminationReason");
if ~valid
    return;
end
gridRecord = searchDiagnostics.Grid;
countNames = ["NodeCount", "VisibilityEdgeCount", "ExpandedCount", "RejectedTransitionCount", "GeneratedSeedCount"];

% Require a finite nonnegative scalar for each available search count.
for name = countNames
    if isfield(gridRecord, name)
        value = gridRecord.(name);
        valid = valid && isnumeric(value) && isscalar(value) && isfinite(value) && value >= 0;
    end
end
if isfield(gridRecord, "ExploredNodes_deg")
    explored_deg = gridRecord.ExploredNodes_deg;
    valid = valid && size(explored_deg, 2) == 2 && all(isfinite(explored_deg), "all");
end
if isfield(gridRecord, "FrontierNodes_deg")
    frontier_deg = gridRecord.FrontierNodes_deg;
    valid = valid && size(frontier_deg, 2) == 2 && all(isfinite(frontier_deg), "all");
end
end

function blocked = directRouteHasCollision(result)
% Probe the complete timed direct line. Do not use the selected planner seed.
sampleCount = 401;
initialState = result.Inputs.initialState;
goalState = result.Inputs.goalState;
sampleTime_s = linspace( initialState.time_s, goalState.time_s, sampleCount).';
goalPosition_deg = obstacleAvoidance.input.goalPositionAtTime( ...
    goalState, goalState.time_s);
fraction = linspace(0, 1, sampleCount).';
position_deg = initialState.position_deg + fraction .* (goalPosition_deg - initialState.position_deg);
queryOptions = struct();
coarseIndex = unique(round(linspace(1, sampleCount, 41))).';
occupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
    result.Inputs.obstacles, position_deg(coarseIndex, 1), ...
    position_deg(coarseIndex, 2), sampleTime_s(coarseIndex), queryOptions);
blocked = any(occupied);
if blocked
    return;
end
remainingIndex = setdiff((1:sampleCount).', coarseIndex, "stable");
occupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
    result.Inputs.obstacles, position_deg(remainingIndex, 1), ...
    position_deg(remainingIndex, 2), sampleTime_s(remainingIndex), queryOptions);
blocked = any(occupied);
end

function validation = createEmptyTrajectoryValidation()
% Return an explicit empty value when no trajectory is available.
validation = struct( "Passed", false, "Message", "No successful trajectory was available.");
end
