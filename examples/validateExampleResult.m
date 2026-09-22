function validation = validateExampleResult(result, scenarioLabel, requirements)
%% Section 0: Header & Readme
% SYNTAX
%   validation = validateExampleResult(result, scenarioLabel)
%   validation = validateExampleResult(result, scenarioLabel, requirements)
%**************************************************************************
% PURPOSE
%   - Independently validate one maintained example result.
%   - Validate stable diagnostics for expected planning failures.
%**************************************************************************
% INPUTS
%   - result (scalar planner-result struct)
%       Result returned by the public planner.
%   - scenarioLabel (scalar text)
%       Human-readable scenario name used in the validation message.
%   - requirements (scalar struct, optional; default struct())
%       ExpectedSuccess defaults true. RequireDirectBlocked defaults false.
%**************************************************************************
% OUTPUTS
%   - validation (scalar struct)
%       Pass state, message, trajectory validation, and diagnostic checks.
%       Invalid input throws an error.
%**************************************************************************
% UNITS
%   - Direct-route collision probes use coordinate units and seconds.
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
expectedSuccess      = fieldOrDefault(requirements, "ExpectedSuccess", true);
requireDirectBlocked = fieldOrDefault(requirements, "RequireDirectBlocked", false);
expectedSuccess = obstacleAvoidance.input.normalizeLogicalScalar( ...
    expectedSuccess, "ExpectedSuccess", "validateExampleResult:InvalidExpectedSuccess");
requireDirectBlocked = obstacleAvoidance.input.normalizeLogicalScalar( ...
    requireDirectBlocked, "RequireDirectBlocked", ...
    "validateExampleResult:InvalidRequireDirectBlocked");
requiredFields       = {'Success', 'Message', 'TerminationReason', 'Inputs', ...
    'Options', 'Route_units'};
formatIsStable       = all(isfield(result, requiredFields));

%% Section 2: Validate Motion Or Expected Failure
% A successful plan receives full trajectory validation. An expected failure
% receives diagnostic validation instead. Both paths return one validation
% record with a clear message for the example warning.

trajectoryValidation = createEmptyTrajectoryValidation();
if formatIsStable && result.Success
    trajectoryValidation = obstacleAvoidance.validateTrajectory(result);
end
diagnosticsAreConsistent = formatIsStable && isfield(result, 'VisibilityGraph') && ...
    diagnosticCountsAreValid(result.VisibilityGraph);
recognizedFailure        = false;
if formatIsStable && ~result.Success
    recognizedReasons = ["endpointBlocked", "dynamicEndpointInfeasible", ...
        "endpointOutsideWorkspace", "noValidatedSeed", "targetLeftXYFrame", ...
        "invalidEndpoint", "noVisibilityRoute", "noOptimizedFeasibleIterate", ...
        "fixedArrivalInfeasible", "timeWindowInfeasible"];
    recognizedFailure = any(result.TerminationReason == recognizedReasons) && ...
        strlength(string(result.Message)) > 0 && diagnosticsAreConsistent;
end

%% Section 3: Validate The Direct-Route Requirement
% Probe the timed straight line independently. This check confirms that planner
% success did not come from an unintentionally easy scenario setup.

directRouteBlocked = false;
if formatIsStable && requireDirectBlocked
    directRouteBlocked = directRouteHasCollision(result);
end
directRequirementPassed = ~requireDirectBlocked || directRouteBlocked;
if expectedSuccess
    passed = formatIsStable && result.Success && trajectoryValidation.Passed && ...
        diagnosticsAreConsistent && directRequirementPassed;
else
    passed = formatIsStable && ~result.Success && recognizedFailure && directRequirementPassed;
end
if passed
    message = scenarioLabel + " passed independent example validation.";
else
    message = scenarioLabel + " failed example validation. " + ...
        "Success=" + string(logical(result.Success)) + ...
        ", reason=" + string(result.TerminationReason) + ...
        ", diagnostics=" + string(diagnosticsAreConsistent) + ...
        ", directBlocked=" + string(directRouteBlocked) + ".";
end
validation = struct( ...
    "Passed",                   passed, ...
    "Message",                  message, ...
    "ExpectedSuccess",          expectedSuccess, ...
    "FormatIsStable",           formatIsStable, ...
    "DiagnosticsAreConsistent", diagnosticsAreConsistent, ...
    "RecognizedFailure",        recognizedFailure, ...
    "DirectRouteBlocked",       directRouteBlocked, ...
    "TrajectoryValidation",     trajectoryValidation);
end

%% Section 4: Local Functions

function value = fieldOrDefault(record, fieldName, defaultValue)
    % Read one optional validation requirement. Use its default when it is absent.
    value = defaultValue;
    if isfield(record, fieldName) && ~isempty(record.(fieldName))
        value = record.(fieldName);
    end
end

function valid = diagnosticCountsAreValid(searchDiagnostics)
    % Check the complete search count the visibility graph reports.
    valid = isstruct(searchDiagnostics) && isscalar(searchDiagnostics);
    if ~valid
        return
    end
    % Require a finite nonnegative scalar for the expansion count the
    % visibility graph reports. No other count reaches this record.
    if isfield(searchDiagnostics, "ExpandedCount")
        expandedCount = searchDiagnostics.ExpandedCount;
        valid = valid && isnumeric(expandedCount) && isscalar(expandedCount) && ...
            isfinite(expandedCount) && expandedCount >= 0;
    end
end

function blocked = directRouteHasCollision(result)
    % Probe the complete timed direct line. Do not use the selected planner seed.
    sampleCount        = 401;
    initialState       = result.Inputs.initialState;
    goalState          = result.Inputs.goalState;
    sampleTime_s       = linspace(initialState.time_s, goalState.time_s, sampleCount).';
    goalPosition_units = goalState.position_units;
    fraction           = linspace(0, 1, sampleCount).';
    position_units     = initialState.position_units + ...
        fraction .* (goalPosition_units - initialState.position_units);
    queryOptions       = struct();
    coarseIndices      = unique(round(linspace(1, sampleCount, 41))).';
    occupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
        result.Inputs.obstacles, position_units(coarseIndices, 1), ...
        position_units(coarseIndices, 2), sampleTime_s(coarseIndices), queryOptions);
    blocked = any(occupied);
    if blocked
        return
    end
    remainingIndices = setdiff((1:sampleCount).', coarseIndices, "stable");
    occupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
        result.Inputs.obstacles, position_units(remainingIndices, 1), ...
        position_units(remainingIndices, 2), sampleTime_s(remainingIndices), queryOptions);
    blocked = any(occupied);
end

function validation = createEmptyTrajectoryValidation()
    % Return an explicit empty value when no trajectory is available.
    validation = struct();
    validation.Passed  = false;
    validation.Message = "No successful trajectory was available.";
end
