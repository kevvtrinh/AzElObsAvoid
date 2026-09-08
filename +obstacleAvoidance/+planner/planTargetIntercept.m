function [result, diagnosis] = planTargetIntercept(obstacles, initialState, goalState, limits, plannerOptions)
%% Section 0: Header & Readme
% SYNTAX: [result, diagnosis] = planTargetIntercept(obstacles, initialState, goalState, limits, plannerOptions)
% PURPOSE: Resolve target history through exact or bounded chronological fixed-time trials.
% INPUTS: The public planner request; targetMotion and terminal policies are in goalState.
% OUTPUTS: Selected fixed-time result, intercept policy, and complete search evidence.
% UNITS: Coordinate units, seconds, and their physical derivatives.

%% Section 1: Normalize States And Terminal Policies
initialState = obstacleAvoidance.input.normalizePlannerState(initialState, "initialState");
limits = obstacleAvoidance.input.normalizePlannerLimits(limits);
targetMotion = goalState.targetMotion;
goalState.position_units = [0 0];
terminalState = obstacleAvoidance.input.normalizePlannerState(goalState, "goalState");
options = struct();
options.InterceptMode = "earliest";
if plannerOptions.GoalTimeMode == "fixedArrival"
    options.InterceptMode = "specifiedTime";
end
options.SpecifiedInterceptTime_s = terminalState.time_s;
options.MaximumSearchDuration_s = terminalState.time_s - initialState.time_s;
options.PlannerOptions = plannerOptions;
options.TerminalVelocity_units_s = terminalState.velocity_units_s;
options.TerminalAcceleration_units_s2 = terminalState.acceleration_units_s2;
for fieldName = ["MatchTargetVelocity", "MatchTargetAcceleration"]
    value = false;
    if isfield(goalState, fieldName) && ~isempty(goalState.(fieldName))
        value = goalState.(fieldName);
    end
    options.(fieldName) = obstacleAvoidance.input.normalizeLogicalScalar(value, fieldName, "planner:InvalidLogicalOption");
end
validateattributes(options.MaximumSearchDuration_s, {'numeric'}, {'real', 'finite', 'scalar', 'positive'});

%% Section 2: Normalize The Sampled Target

if ~isstruct(targetMotion) || ~isscalar(targetMotion) || ~all(isfield(targetMotion, {'time_s', 'position_units'}))
    error("planner:InvalidTargetMotion", "targetMotion must contain time_s and position_units.");
end
validateattributes(targetMotion.time_s, {'numeric'}, {'real', 'finite', 'vector', 'increasing'});
targetMotion.time_s = double(targetMotion.time_s(:));
if numel(targetMotion.time_s) < 2
    error("planner:TargetHistoryTooShort", "targetMotion.time_s must contain at least two samples.");
end
validateattributes(targetMotion.position_units, {'numeric'}, {'real', 'finite', '2d', 'ncols', 2, 'nrows', numel(targetMotion.time_s)});
targetMotion.position_units = double(targetMotion.position_units);
if ~isfield(targetMotion, "InterpolationMethod") || isempty(targetMotion.InterpolationMethod)
    targetMotion.InterpolationMethod = "linear";
end
targetMotion.InterpolationMethod = string(targetMotion.InterpolationMethod);
if ~isscalar(targetMotion.InterpolationMethod) || ~any(targetMotion.InterpolationMethod == ["linear", "pchip"])
    error("planner:InvalidInterpolation", "InterpolationMethod must be 'linear' or 'pchip'.");
end

%% Section 3: Plan The Intercept

initialTime_s = double(initialState.time_s);
if options.InterceptMode == "specifiedTime"
    interceptTime_s = double(options.SpecifiedInterceptTime_s);
    validateattributes(interceptTime_s, {'numeric'}, {'real', 'finite', 'scalar', '>', initialTime_s});
    if interceptTime_s < targetMotion.time_s(1) || interceptTime_s > targetMotion.time_s(end)
        error("planner:InterceptOutsideHistory", "SpecifiedInterceptTime_s must be inside targetMotion.time_s.");
    end
    [result, diagnosis] = planAtTime(obstacles, initialState, targetMotion, limits, options, interceptTime_s, nargout > 1);
    search = searchRecord("specifiedFixedTime", 1, 0, 0, interceptTime_s, interceptTime_s, interceptTime_s, interceptTime_s, 0, struct());
else
    if options.MatchTargetVelocity || options.MatchTargetAcceleration || any(options.TerminalVelocity_units_s ~= 0) || any(options.TerminalAcceleration_units_s2 ~= 0)
        error("planner:UnsupportedMovingDerivative", "Earliest intercept supports explicit zero terminal velocity " + "and acceleration only.");
    end
    [result, search, diagnosis] = searchEarliest(obstacles, initialState, targetMotion, limits, options, nargout > 1);
end
if result.Success
    achievedTime_s     = result.time_s(end);
    achievedTarget_units = targetAtTime(targetMotion, achievedTime_s);
else
    achievedTime_s     = NaN;
    achievedTarget_units = [NaN NaN];
end
% Every trial overwrites this mode before invoking the public planner.
options.PlannerOptions.GoalTimeMode = "fixedArrival";
policies = ["zero", "target"];
result.Intercept = struct("Mode", options.InterceptMode, ...
    "Time_s", achievedTime_s, "TargetPosition_units", achievedTarget_units, ...
    "TerminalVelocityPolicy", policies(options.MatchTargetVelocity + 1), ...
    "TerminalAccelerationPolicy", ...
        policies(options.MatchTargetAcceleration + 1));
if ~options.MatchTargetVelocity && any(options.TerminalVelocity_units_s ~= 0)
    result.Intercept.TerminalVelocityPolicy = "specified";
end
if ~options.MatchTargetAcceleration && any(options.TerminalAcceleration_units_s2 ~= 0)
    result.Intercept.TerminalAccelerationPolicy = "specified";
end
if nargout > 1
    diagnosis.InterceptSearch  = search;
    diagnosis.InterceptOptions = rmfield(options, "PlannerOptions");
end
end

%% Section 4: Local Functions

function [result, search, diagnosis] = searchEarliest(obstacles, initialState, targetMotion, limits, options, includeDiagnosis)
    % Use the exact direct-intercept solver when eligible; otherwise search a time grid.
    tolerance_s = options.PlannerOptions.ArrivalTimeTolerance_s;
    searchStart_s = max(targetMotion.time_s(1), initialState.time_s + tolerance_s);
    searchEnd_s   = min(targetMotion.time_s(end), initialState.time_s + options.MaximumSearchDuration_s);
    if searchEnd_s <= searchStart_s
        error("planner:EmptySearchWindow", "The target history and MaximumSearchDuration_s do not overlap " + "after initialState.time_s.");
    end
    exactDiagnostics = struct();
    isDirectExact    = isempty(obstacles) && targetMotion.InterpolationMethod == "linear" && all(initialState.velocity_units_s == 0) && all(initialState.acceleration_units_s2 == 0);
    if isDirectExact
        [exactTime_s, exactDiagnostics] = obstacleAvoidance.planner.findEarliestLinearIntercept(initialState, targetMotion.time_s, targetMotion.position_units, limits, searchEnd_s);
        if isfinite(exactTime_s) && exactTime_s >= searchStart_s
            [trial, trialDiagnosis] = planAtTime(obstacles, initialState, targetMotion, limits, options, exactTime_s, includeDiagnosis);
            if trial.Success
                result    = trial;
                diagnosis = trialDiagnosis;
                search    = searchRecord("completePiecewisePolynomialDirect", 1, 0, 0, searchStart_s, searchEnd_s, exactTime_s, exactTime_s, 0, exactDiagnostics);
                return;
            end
        end
    end

    coarseIntervalCount = 16;
    coarseTime_s        = unique([linspace(searchStart_s, searchEnd_s, coarseIntervalCount + 1).'; targetMotion.time_s(targetMotion.time_s >= searchStart_s & targetMotion.time_s <= searchEnd_s)]);
    trialCount          = 0;
    selectedTime_s      = NaN;
    lowerTime_s         = searchStart_s;
    result              = [];
    for queryTime_s = coarseTime_s.'
        [trial, trialDiagnosis] = planAtTime(obstacles, initialState, targetMotion, limits, options, queryTime_s, includeDiagnosis);
        trialCount = trialCount + 1;
        result     = trial;
        diagnosis  = trialDiagnosis;
        if trial.Success
            selectedTime_s = queryTime_s;
            break;
        end
        lowerTime_s = queryTime_s;
    end
    initialUpperTime_s = selectedTime_s;
    refinementCount    = 0;
    while isfinite(selectedTime_s) && selectedTime_s - lowerTime_s > tolerance_s && refinementCount < 16
        queryTime_s = 0.5 * (lowerTime_s + selectedTime_s);
        [trial, trialDiagnosis] = planAtTime(obstacles, initialState, targetMotion, limits, options, queryTime_s, includeDiagnosis);
        trialCount      = trialCount + 1;
        refinementCount = refinementCount + 1;
        if trial.Success
            result         = trial;
            diagnosis      = trialDiagnosis;
            selectedTime_s = queryTime_s;
        else
            lowerTime_s = queryTime_s;
        end
    end
    search = searchRecord("boundedChronologicalFixedTime", trialCount, coarseIntervalCount, refinementCount, searchStart_s, searchEnd_s, initialUpperTime_s, lowerTime_s, tolerance_s, exactDiagnostics);
    search.MaximumCoarseStep_s = max(diff(coarseTime_s));
end

function [result, diagnosis] = planAtTime(obstacles, initialState, targetMotion, limits, options, interceptTime_s, includeDiagnosis)
    % Call the public planner for one fixed-time intercept.
    terminalPosition_units        = targetAtTime(targetMotion, interceptTime_s);
    terminalVelocity_units_s      = options.TerminalVelocity_units_s;
    terminalAcceleration_units_s2 = options.TerminalAcceleration_units_s2;
    if options.MatchTargetVelocity || options.MatchTargetAcceleration
        [targetVelocity_units_s, targetAcceleration_units_s2] = targetDerivatives(targetMotion.time_s, targetMotion.position_units, interceptTime_s, targetMotion.InterpolationMethod);
        if options.MatchTargetVelocity
            terminalVelocity_units_s = targetVelocity_units_s;
        end
        if options.MatchTargetAcceleration
            terminalAcceleration_units_s2 = targetAcceleration_units_s2;
        end
    end
    goalState = struct("time_s", interceptTime_s, ...
        "position_units", terminalPosition_units, ...
        "velocity_units_s", terminalVelocity_units_s, ...
        "acceleration_units_s2", terminalAcceleration_units_s2, ...
        "targetTime_s", targetMotion.time_s, ...
        "targetPosition_units", targetMotion.position_units, ...
        "InterpolationMethod", targetMotion.InterpolationMethod);
    plannerOptions = options.PlannerOptions;
    plannerOptions.GoalTimeMode = "fixedArrival";
    if includeDiagnosis
        [result, diagnosis] = planner(obstacles, initialState, goalState, limits, plannerOptions);
    else
        result    = planner(obstacles, initialState, goalState, limits, plannerOptions);
        diagnosis = struct();
    end
end

function position_units = targetAtTime(targetMotion, queryTime_s)
    % Evaluate the target without extrapolating its history.
    position_units = interp1(targetMotion.time_s, targetMotion.position_units, queryTime_s, targetMotion.InterpolationMethod);
end

function [velocity_units_s, acceleration_units_s2] = targetDerivatives(time_s, position_units, queryTime_s, method)
    % Estimate derivatives using the documented bounded centered/one-sided step.
    step_s              = max(1e-5, min(1e-2, min(diff(time_s)) / 100));
    lowerTime_s         = max(time_s(1), queryTime_s - step_s);
    upperTime_s         = min(time_s(end), queryTime_s + step_s);
    sample_units          = interp1(time_s, position_units, [lowerTime_s; queryTime_s; upperTime_s], method);
    leftDuration_s      = max(queryTime_s - lowerTime_s, eps);
    rightDuration_s     = max(upperTime_s - queryTime_s, eps);
    velocity_units_s      = (sample_units(3, :) - sample_units(1, :)) / (upperTime_s - lowerTime_s);
    leftVelocity_units_s  = (sample_units(2, :) - sample_units(1, :)) / leftDuration_s;
    rightVelocity_units_s = (sample_units(3, :) - sample_units(2, :)) / rightDuration_s;
    acceleration_units_s2 = 2 * (rightVelocity_units_s - leftVelocity_units_s) / (leftDuration_s + rightDuration_s);
end

function search = searchRecord(policy, trialCount, coarseCount, refinementCount, startTime_s, endTime_s, upperTime_s, lowerTime_s, tolerance_s, exactDiagnostics)
    % Assemble intercept-search diagnostics.
    if policy == "completePiecewisePolynomialDirect"
        optimalityStatus = "certifiedEarliest";
    elseif policy == "boundedChronologicalFixedTime"
        optimalityStatus = "resolutionBounded";
    else
        optimalityStatus = "notAnOptimization";
    end
    search = struct("Policy", policy, ...
        "OptimalityStatus", optimalityStatus, ...
        "MaximumCoarseStep_s", NaN, ...
        "TrialCount", trialCount, ...
        "CoarseIntervalCount", coarseCount, "RefinementCount", refinementCount, ...
        "SearchStartTime_s", startTime_s, "SearchEndTime_s", endTime_s, ...
        "InitialValidatedUpperTime_s", upperTime_s, ...
        "FinalLowerTime_s", lowerTime_s, ...
        "ArrivalTimeTolerance_s", tolerance_s, "ExactDirect", exactDiagnostics);
end
