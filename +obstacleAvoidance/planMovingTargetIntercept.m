function result = planMovingTargetIntercept(varargin)
%% Section 0: Header & Readme
% SYNTAX
%   options = obstacleAvoidance.planMovingTargetIntercept()
%   result = obstacleAvoidance.planMovingTargetIntercept( ...
%       initialState, targetMotion, limits, options)
%   result = obstacleAvoidance.planMovingTargetIntercept( ...
%       obstacles, initialState, targetMotion, limits, options)
%**************************************************************************
% PURPOSE
%   - Convert sampled target motion into fixed-time planner requests.
%   - Enumerate every algebraic event for an obstacle-free linear target;
%     otherwise retain a bounded chronological planner search.
%**************************************************************************
% INPUTS
%   - obstacles (canonical protected obstacle array, optional; default [])
%   - initialState (scalar state struct)
%   - targetMotion (scalar struct)
%       Increasing time_s and N-by-2 position_deg are required. Optional
%       InterpolationMethod is "linear" or "pchip".
%   - limits (scalar limits struct)
%   - options (scalar struct, optional; default struct())
%       InterceptMode is "earliest" or "specifiedTime". SpecifiedInterceptTime_s
%       is required for specifiedTime. MaximumSearchDuration_s defaults to 60.
%       MatchTargetVelocity and MatchTargetAcceleration default false.
%       PlannerOptions is a partial planTrajectory options struct.
%**************************************************************************
% OUTPUTS
%   - result (scalar planTrajectory result)
%       Adds a stable Intercept record containing policy and search evidence.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s and deg/s^2.
%**************************************************************************

%% Section 1: Resolve The Call And Options

defaults = struct("InterceptMode", "earliest", ...
    "SpecifiedInterceptTime_s", NaN, "MaximumSearchDuration_s", 60, ...
    "MatchTargetVelocity", false, "MatchTargetAcceleration", false, ...
    "PlannerOptions", struct());
if nargin == 0
    result = defaults;
    return;
elseif nargin == 4
    obstacles = [];
    initialState = varargin{1};
    targetMotion = varargin{2};
    limits = varargin{3};
    optionOverrides = varargin{4};
elseif nargin == 5
    obstacles = varargin{1};
    initialState = varargin{2};
    targetMotion = varargin{3};
    limits = varargin{4};
    optionOverrides = varargin{5};
else
    error("planMovingTargetIntercept:InvalidCall", ...
        "Use zero, four, or five inputs as documented.");
end
if isempty(optionOverrides)
    optionOverrides = struct();
end
if ~isstruct(optionOverrides) || ~isscalar(optionOverrides)
    error("planMovingTargetIntercept:InvalidOptions", ...
        "options must be a scalar struct.");
end
[options, unknownNames] = obstacleAvoidance.input.resolveOptions( ...
    defaults, optionOverrides);
if ~isempty(unknownNames)
    warning("planMovingTargetIntercept:UnknownOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", ...
        strjoin(unknownNames, ", "));
end
options.InterceptMode = string(options.InterceptMode);
if ~isscalar(options.InterceptMode) || ~any( ...
        options.InterceptMode == ["earliest", "specifiedTime"])
    error("planMovingTargetIntercept:InvalidMode", ...
        "InterceptMode must be 'earliest' or 'specifiedTime'.");
end
for fieldName = ["MatchTargetVelocity", "MatchTargetAcceleration"]
    options.(fieldName) = obstacleAvoidance.input.normalizeLogicalScalar( ...
        options.(fieldName), fieldName, ...
        "planMovingTargetIntercept:InvalidLogicalOption");
end
validateattributes(options.MaximumSearchDuration_s, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'positive'});
if ~isstruct(options.PlannerOptions) || ~isscalar(options.PlannerOptions)
    error("planMovingTargetIntercept:InvalidPlannerOptions", ...
        "PlannerOptions must be a scalar struct.");
end

%% Section 2: Normalize The Sampled Target

if ~isstruct(targetMotion) || ~isscalar(targetMotion) || ...
        ~all(isfield(targetMotion, {'time_s', 'position_deg'}))
    error("planMovingTargetIntercept:InvalidTargetMotion", ...
        "targetMotion must contain time_s and position_deg.");
end
validateattributes(targetMotion.time_s, {'numeric'}, ...
    {'real', 'finite', 'vector', 'increasing'});
targetMotion.time_s = double(targetMotion.time_s(:));
if numel(targetMotion.time_s) < 2
    error("planMovingTargetIntercept:TargetHistoryTooShort", ...
        "targetMotion.time_s must contain at least two samples.");
end
validateattributes(targetMotion.position_deg, {'numeric'}, ...
    {'real', 'finite', '2d', 'ncols', 2, ...
    'nrows', numel(targetMotion.time_s)});
targetMotion.position_deg = double(targetMotion.position_deg);
if ~isfield(targetMotion, "InterpolationMethod") || ...
        isempty(targetMotion.InterpolationMethod)
    targetMotion.InterpolationMethod = "linear";
end
targetMotion.InterpolationMethod = string(targetMotion.InterpolationMethod);
if ~isscalar(targetMotion.InterpolationMethod) || ~any( ...
        targetMotion.InterpolationMethod == ["linear", "pchip"])
    error("planMovingTargetIntercept:InvalidInterpolation", ...
        "InterpolationMethod must be 'linear' or 'pchip'.");
end

%% Section 3: Plan The Intercept

initialTime_s = double(initialState.time_s);
if options.InterceptMode == "specifiedTime"
    interceptTime_s = double(options.SpecifiedInterceptTime_s);
    validateattributes(interceptTime_s, {'numeric'}, ...
        {'real', 'finite', 'scalar', '>', initialTime_s});
    if interceptTime_s < targetMotion.time_s(1) || ...
            interceptTime_s > targetMotion.time_s(end)
        error("planMovingTargetIntercept:InterceptOutsideHistory", ...
            "SpecifiedInterceptTime_s must be inside targetMotion.time_s.");
    end
    [result, search] = planAtTime(obstacles, initialState, targetMotion, ...
        limits, options, interceptTime_s);
else
    if options.MatchTargetVelocity || options.MatchTargetAcceleration
        error("planMovingTargetIntercept:UnsupportedMovingDerivative", ...
            "Earliest intercept supports explicit zero terminal velocity " + ...
            "and acceleration only.");
    end
    [result, search] = searchEarliest( ...
        obstacles, initialState, targetMotion, limits, options);
end
if result.Success
    achievedTime_s = result.time_s(end);
    achievedTarget_deg = targetAtTime(targetMotion, achievedTime_s);
else
    achievedTime_s = NaN;
    achievedTarget_deg = [NaN NaN];
end
policies = ["zero", "target"];
result.Intercept = struct("Mode", options.InterceptMode, ...
    "Time_s", achievedTime_s, "TargetPosition_deg", achievedTarget_deg, ...
    "TerminalVelocityPolicy", policies(options.MatchTargetVelocity + 1), ...
    "TerminalAccelerationPolicy", ...
        policies(options.MatchTargetAcceleration + 1), ...
    "Search", search, "Options", options);
end

%% Section 4: Local Functions

function [result, search] = searchEarliest( ...
        obstacles, initialState, targetMotion, limits, options)
% Use the complete direct event kernel when applicable, else a bounded grid.
plannerDefaults = obstacleAvoidance.planTrajectory();
tolerance_s = plannerDefaults.ArrivalTimeTolerance_s;
if isfield(options.PlannerOptions, "ArrivalTimeTolerance_s")
    tolerance_s = options.PlannerOptions.ArrivalTimeTolerance_s;
end
searchStart_s = max(targetMotion.time_s(1), initialState.time_s + tolerance_s);
searchEnd_s = min(targetMotion.time_s(end), ...
    initialState.time_s + options.MaximumSearchDuration_s);
if searchEnd_s <= searchStart_s
    error("planMovingTargetIntercept:EmptySearchWindow", ...
        "The target history and MaximumSearchDuration_s do not overlap " + ...
        "after initialState.time_s.");
end
exactDiagnostics = struct();
isDirectExact = isempty(obstacles) && ...
    targetMotion.InterpolationMethod == "linear" && ...
    derivativeIsZero(initialState, "velocity_deg_s") && ...
    derivativeIsZero(initialState, "acceleration_deg_s2");
if isDirectExact
    [exactTime_s, exactDiagnostics] = ...
        obstacleAvoidance.planner.findEarliestLinearIntercept( ...
        initialState, targetMotion.time_s, targetMotion.position_deg, ...
        limits, searchEnd_s);
    if isfinite(exactTime_s) && exactTime_s >= searchStart_s
        [trial, ~] = planAtTime(obstacles, initialState, targetMotion, ...
            limits, options, exactTime_s);
        if trial.Success
            result = trial;
            search = searchRecord("completePiecewisePolynomialDirect", ...
                1, 0, 0, searchStart_s, searchEnd_s, exactTime_s, ...
                exactTime_s, 0, exactDiagnostics);
            return;
        end
    end
end

coarseIntervalCount = 16;
coarseTime_s = unique([linspace(searchStart_s, searchEnd_s, ...
    coarseIntervalCount + 1).'; targetMotion.time_s( ...
    targetMotion.time_s >= searchStart_s & targetMotion.time_s <= searchEnd_s)]);
trialCount = 0;
selectedTime_s = NaN;
lowerTime_s = searchStart_s;
result = [];
for queryTime_s = coarseTime_s.'
    [trial, ~] = planAtTime(obstacles, initialState, targetMotion, ...
        limits, options, queryTime_s);
    trialCount = trialCount + 1;
    result = trial;
    if trial.Success
        selectedTime_s = queryTime_s;
        break;
    end
    lowerTime_s = queryTime_s;
end
initialUpperTime_s = selectedTime_s;
refinementCount = 0;
while isfinite(selectedTime_s) && selectedTime_s - lowerTime_s > ...
        tolerance_s && refinementCount < 16
    queryTime_s = 0.5 * (lowerTime_s + selectedTime_s);
    [trial, ~] = planAtTime(obstacles, initialState, targetMotion, ...
        limits, options, queryTime_s);
    trialCount = trialCount + 1;
    refinementCount = refinementCount + 1;
    if trial.Success
        result = trial;
        selectedTime_s = queryTime_s;
    else
        lowerTime_s = queryTime_s;
    end
end
search = searchRecord("boundedChronologicalFixedTime", trialCount, ...
    coarseIntervalCount, refinementCount, searchStart_s, searchEnd_s, ...
    initialUpperTime_s, lowerTime_s, tolerance_s, exactDiagnostics);
end

function isZero = derivativeIsZero(state, fieldName)
% Recognize the exact rest state required by the algebraic kernel.
isZero = ~isfield(state, fieldName) || isempty(state.(fieldName)) || ...
    all(double(state.(fieldName)) == 0, "all");
end

function [result, search] = planAtTime( ...
        obstacles, initialState, targetMotion, limits, options, interceptTime_s)
% Solve one fixed-time intercept through the maintained public planner path.
terminalPosition_deg = targetAtTime(targetMotion, interceptTime_s);
terminalVelocity_deg_s = [0 0];
terminalAcceleration_deg_s2 = [0 0];
if options.MatchTargetVelocity || options.MatchTargetAcceleration
    [targetVelocity_deg_s, targetAcceleration_deg_s2] = ...
        targetDerivatives(targetMotion.time_s, targetMotion.position_deg, ...
        interceptTime_s, targetMotion.InterpolationMethod);
    if options.MatchTargetVelocity
        terminalVelocity_deg_s = targetVelocity_deg_s;
    end
    if options.MatchTargetAcceleration
        terminalAcceleration_deg_s2 = targetAcceleration_deg_s2;
    end
end
goalState = struct("time_s", interceptTime_s, ...
    "position_deg", terminalPosition_deg, ...
    "velocity_deg_s", terminalVelocity_deg_s, ...
    "acceleration_deg_s2", terminalAcceleration_deg_s2, ...
    "targetTime_s", targetMotion.time_s, ...
    "targetPosition_deg", targetMotion.position_deg, ...
    "InterpolationMethod", targetMotion.InterpolationMethod);
plannerOptions = options.PlannerOptions;
plannerOptions.GoalTimeMode = "fixedArrival";
result = obstacleAvoidance.planTrajectory( ...
    obstacles, initialState, goalState, limits, plannerOptions);
search = searchRecord("specifiedFixedTime", 1, 0, 0, ...
    interceptTime_s, interceptTime_s, interceptTime_s, ...
    interceptTime_s, 0, struct());
end

function position_deg = targetAtTime(targetMotion, queryTime_s)
% Evaluate the normalized target representation without extrapolation.
position_deg = interp1(targetMotion.time_s, targetMotion.position_deg, ...
    queryTime_s, targetMotion.InterpolationMethod);
end

function [velocity_deg_s, acceleration_deg_s2] = targetDerivatives( ...
        time_s, position_deg, queryTime_s, method)
% Estimate derivatives using the documented bounded centered/one-sided step.
step_s = max(1e-5, min(1e-2, min(diff(time_s)) / 100));
lowerTime_s = max(time_s(1), queryTime_s - step_s);
upperTime_s = min(time_s(end), queryTime_s + step_s);
sample_deg = interp1(time_s, position_deg, ...
    [lowerTime_s; queryTime_s; upperTime_s], method);
leftDuration_s = max(queryTime_s - lowerTime_s, eps);
rightDuration_s = max(upperTime_s - queryTime_s, eps);
velocity_deg_s = (sample_deg(3, :) - sample_deg(1, :)) / ...
    (upperTime_s - lowerTime_s);
leftVelocity_deg_s = (sample_deg(2, :) - sample_deg(1, :)) / leftDuration_s;
rightVelocity_deg_s = (sample_deg(3, :) - sample_deg(2, :)) / rightDuration_s;
acceleration_deg_s2 = 2 * (rightVelocity_deg_s - leftVelocity_deg_s) / ...
    (leftDuration_s + rightDuration_s);
end

function search = searchRecord(policy, trialCount, coarseCount, ...
        refinementCount, startTime_s, endTime_s, upperTime_s, ...
        lowerTime_s, tolerance_s, exactDiagnostics)
% Assemble one stable intercept-search diagnostic record.
search = struct("Policy", policy, "TrialCount", trialCount, ...
    "CoarseIntervalCount", coarseCount, "RefinementCount", refinementCount, ...
    "SearchStartTime_s", startTime_s, "SearchEndTime_s", endTime_s, ...
    "InitialValidatedUpperTime_s", upperTime_s, ...
    "FinalLowerTime_s", lowerTime_s, ...
    "ArrivalTimeTolerance_s", tolerance_s, "ExactDirect", exactDiagnostics);
end
