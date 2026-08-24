function result = planMovingTargetIntercept(varargin)
%% Section 0: Header & Readme
% SYNTAX
%   options = planAzElMovingTargetIntercept()
%   result = planAzElMovingTargetIntercept( ...
%       initialState, targetMotion, limits, options)
%   result = planAzElMovingTargetIntercept( ...
%       obstacles, initialState, targetMotion, limits, options)
%**************************************************************************
% PURPOSE
%   - Adapt a sampled moving target to planAzElMotion. Specified-time mode
%     makes one planner call; earliest mode makes a bounded chronological
%     search and then bisects only the first observed feasible time bracket.
%**************************************************************************
% INPUTS
%   - obstacles (canonical protected obstacle array, optional; default [])
%   - initialState (scalar state struct)
%   - targetMotion (scalar struct)
%       Increasing time_s, N-by-2 position_deg, and optional
%       InterpolationMethod equal to "linear" or "pchip".
%   - limits (scalar limits struct)
%   - options (scalar struct, optional; default struct())
%       InterceptMode is "earliest" or "specifiedTime".
%       SpecifiedInterceptTime_s is required for specifiedTime.
%       MaximumSearchDuration_s defaults to 60.
%       MatchTargetVelocity and MatchTargetAcceleration default false.
%       PlannerOptions is a partial planAzElMotion option struct.
%**************************************************************************
% OUTPUTS
%   - result (scalar planAzElMotion result)
%       Adds one compact Intercept record. No second planner is used.
%**************************************************************************
% UNITS
%   - Position is degrees, time is seconds, and target derivatives use
%     degrees per second and degrees per second squared.
%**************************************************************************

%% Section 1: Resolve Call Form And Options

% This wrapper owns intercept-policy options only. Planner-specific choices
% remain nested in PlannerOptions and are normalized by planAzElMotion.
defaults = struct( ...
    "InterceptMode", "earliest", ...
    "SpecifiedInterceptTime_s", NaN, ...
    "MaximumSearchDuration_s", 60, ...
    "MatchTargetVelocity", false, "MatchTargetAcceleration", false, "PlannerOptions", struct());
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
    error("planAzElMovingTargetIntercept:InvalidCall", "Use zero, four, or five inputs as documented.");
end
if isempty(optionOverrides)
    optionOverrides = struct();
end
if ~isstruct(optionOverrides) || ~isscalar(optionOverrides)
    error("planAzElMovingTargetIntercept:InvalidOptions", "options must be a scalar struct.");
end
[options, unknownNames] = azElInternal.resolveOptions( ...
    defaults, optionOverrides);
if ~isempty(unknownNames)
    warning("planAzElMovingTargetIntercept:UnknownOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", strjoin(unknownNames, ", "));
end
options.InterceptMode = string(options.InterceptMode);
if ~isscalar(options.InterceptMode) || ~any(options.InterceptMode == ["earliest", "specifiedTime"])
    error("planAzElMovingTargetIntercept:InvalidMode", "InterceptMode must be 'earliest' or 'specifiedTime'.");
end
logicalNames = ["MatchTargetVelocity", "MatchTargetAcceleration"];

% Normalize both target-derivative matching controls to scalar logical values.
for name = logicalNames
    options.(name) = azElInternal.normalizeLogicalScalar( ...
        options.(name), name, "planAzElMovingTargetIntercept:InvalidLogicalOption");
end
validateattributes(options.MaximumSearchDuration_s, {'numeric'}, {'real', 'finite', 'scalar', 'positive'});
if ~isstruct(options.PlannerOptions) || ~isscalar(options.PlannerOptions)
    error("planAzElMovingTargetIntercept:InvalidPlannerOptions", "PlannerOptions must be a scalar struct.");
end

%% Section 2: Normalize The Sampled Target

% Keep one time orientation and interpolation rule so target position and
% finite-difference terminal derivatives describe the same sampled history.
if ~isstruct(targetMotion) || ~isscalar(targetMotion) || ~all(isfield(targetMotion, {'time_s', 'position_deg'}))
    error("planAzElMovingTargetIntercept:InvalidTargetMotion", "targetMotion must contain time_s and position_deg.");
end
validateattributes(targetMotion.time_s, {'numeric'}, {'real', 'finite', 'vector', 'increasing'});
targetTime_s = double(targetMotion.time_s(:));
if numel(targetTime_s) < 2
    error("planAzElMovingTargetIntercept:TargetHistoryTooShort", ...
        "targetMotion.time_s must contain at least two samples.");
end
validateattributes(targetMotion.position_deg, {'numeric'}, ...
    {'real', 'finite', '2d', 'ncols', 2, 'nrows', numel(targetTime_s)});
targetPosition_deg = double(targetMotion.position_deg);
interpolationMethod = "linear";
if isfield(targetMotion, "InterpolationMethod") && ~isempty(targetMotion.InterpolationMethod)
    interpolationMethod = string(targetMotion.InterpolationMethod);
end
if ~isscalar(interpolationMethod) || ~any(interpolationMethod == ["linear", "pchip"])
    error("planAzElMovingTargetIntercept:InvalidInterpolation", "InterpolationMethod must be 'linear' or 'pchip'.");
end

%% Section 3: Adapt The Goal And Call The Single Planner

% Each trial becomes an ordinary fixed-arrival planner request. This avoids a
% second motion implementation with different collision or limit semantics.
initialTime_s = double(initialState.time_s);
if options.InterceptMode == "specifiedTime"
    interceptTime_s = double(options.SpecifiedInterceptTime_s);
    validateattributes(interceptTime_s, {'numeric'}, {'real', 'finite', 'scalar', '>', initialTime_s});
    if interceptTime_s < targetTime_s(1) || interceptTime_s > targetTime_s(end)
        error("planAzElMovingTargetIntercept:InterceptOutsideHistory", ...
            "SpecifiedInterceptTime_s must be inside targetMotion.time_s.");
    end
    [result, interceptSearch] = planAtInterceptTime( ...
        obstacles, initialState, targetTime_s, targetPosition_deg, ...
        interpolationMethod, limits, options, interceptTime_s);
else
    if options.MatchTargetVelocity || options.MatchTargetAcceleration
        error("planAzElMovingTargetIntercept:UnsupportedMovingDerivative", ...
            "Earliest intercept supports explicit zero terminal velocity " + "and acceleration only.");
    end
    [result, ~, interceptSearch] = searchEarliestIntercept( ...
        obstacles, initialState, targetTime_s, targetPosition_deg, interpolationMethod, limits, options);
end
if result.Success
    achievedTime_s = result.time_s(end);
    achievedTarget_deg = interp1( targetTime_s, targetPosition_deg, achievedTime_s, interpolationMethod);
else
    achievedTime_s = NaN;
    achievedTarget_deg = [NaN NaN];
end
result.Intercept = struct( ...
    "Mode", options.InterceptMode, ...
    "Time_s", achievedTime_s, ...
    "TargetPosition_deg", achievedTarget_deg, ...
    "TerminalVelocityPolicy", conditionalText( ...
    options.MatchTargetVelocity, "target", "zero"), ...
    "TerminalAccelerationPolicy", conditionalText( ...
    options.MatchTargetAcceleration, "target", "zero"), "Search", interceptSearch, "Options", options);
end


function [result, selectedTime_s, search] = searchEarliestIntercept( ...
        obstacles, initialState, targetTime_s, targetPosition_deg, interpolationMethod, limits, options)
% Find the earliest validated fixed-time intercept within a bounded, deterministic time grid and refine its first observed bracket.
plannerDefaults = azElPlannerMethods.corridor.plan();
arrivalTolerance_s = plannerDefaults.ArrivalTimeTolerance_s;
if isfield(options.PlannerOptions, "ArrivalTimeTolerance_s")
    arrivalTolerance_s = options.PlannerOptions.ArrivalTimeTolerance_s;
end
searchStart_s = max(targetTime_s(1), initialState.time_s + arrivalTolerance_s);
searchEnd_s = min(targetTime_s(end), initialState.time_s + options.MaximumSearchDuration_s);
if searchEnd_s <= searchStart_s
    error("planAzElMovingTargetIntercept:EmptySearchWindow", ...
        "The target history and MaximumSearchDuration_s do not overlap " + "after initialState.time_s.");
end
coarseIntervalCount = 16;
coarseTime_s = unique([ ...
    linspace(searchStart_s, searchEnd_s, coarseIntervalCount + 1).'; ...
    targetTime_s(targetTime_s >= searchStart_s & targetTime_s <= searchEnd_s)]);
trialCount = 0;
selectedTime_s = NaN;
result = [];
lowerTime_s = searchStart_s;

% Try each coarse intercept time chronologically until a feasible plan is found.
for timeIndex = 1:numel(coarseTime_s)
    queryTime_s = coarseTime_s(timeIndex);
    [trial, ~] = planAtInterceptTime( ...
        obstacles, initialState, targetTime_s, targetPosition_deg, interpolationMethod, limits, options, queryTime_s);
    trialCount = trialCount + 1;
    if trial.Success
        result = trial;
        selectedTime_s = queryTime_s;
        if timeIndex > 1
            lowerTime_s = coarseTime_s(timeIndex - 1);
        end
        break;
    end
    result = trial;
    lowerTime_s = queryTime_s;
end
initialUpperTime_s = selectedTime_s;
maximumRefinementCount = 16;
refinementCount = 0;

% Bisect the last infeasible/feasible time bracket to refine the earliest intercept.
while isfinite(selectedTime_s) && ...
        selectedTime_s - lowerTime_s > arrivalTolerance_s && refinementCount < maximumRefinementCount
    queryTime_s = 0.5 * (lowerTime_s + selectedTime_s);
    [trial, ~] = planAtInterceptTime( ...
        obstacles, initialState, targetTime_s, targetPosition_deg, interpolationMethod, limits, options, queryTime_s);
    trialCount = trialCount + 1;
    refinementCount = refinementCount + 1;
    if trial.Success
        result = trial;
        selectedTime_s = queryTime_s;
    else
        lowerTime_s = queryTime_s;
    end
end
search = struct( ...
    "Method", "boundedChronologicalFixedTime", ...
    "TrialCount", trialCount, ...
    "CoarseIntervalCount", coarseIntervalCount, ...
    "RefinementCount", refinementCount, ...
    "SearchStartTime_s", searchStart_s, ...
    "SearchEndTime_s", searchEnd_s, ...
    "InitialValidatedUpperTime_s", initialUpperTime_s, ...
    "FinalLowerTime_s", lowerTime_s, ...
    "ArrivalTimeTolerance_s", arrivalTolerance_s, ...
    "OptimalityStatement", ...
    "Earliest independently validated intercept in the first observed " + ...
    "bounded time bracket; global optimality is not claimed.");
end

function [result, search] = planAtInterceptTime( ...
        obstacles, initialState, targetTime_s, targetPosition_deg, ...
        interpolationMethod, limits, options, interceptTime_s)
% Solve one fixed-time intercept using the selected production planner.
terminalPosition_deg = interp1( targetTime_s, targetPosition_deg, interceptTime_s, interpolationMethod);
[terminalVelocity_deg_s, terminalAcceleration_deg_s2] = targetDerivatives(targetTime_s, targetPosition_deg, ...
    interceptTime_s, interpolationMethod);
if ~options.MatchTargetVelocity
    terminalVelocity_deg_s = [0 0];
end
if ~options.MatchTargetAcceleration
    terminalAcceleration_deg_s2 = [0 0];
end
goalState = struct( ...
    "time_s", interceptTime_s, ...
    "position_deg", terminalPosition_deg, ...
    "velocity_deg_s", terminalVelocity_deg_s, ...
    "acceleration_deg_s2", terminalAcceleration_deg_s2, ...
    "targetTime_s", targetTime_s, "targetPosition_deg", targetPosition_deg, "InterpolationMethod", interpolationMethod);
plannerOptions = options.PlannerOptions;
plannerOptions.GoalTimeMode = "fixedArrival";
result = azElPlannerMethods.corridor.plan( obstacles, initialState, goalState, limits, plannerOptions);
search = struct( ...
    "Method", "specifiedFixedTime", "TrialCount", 1, ...
    "CoarseIntervalCount", 0, "RefinementCount", 0, ...
    "SearchStartTime_s", interceptTime_s, ...
    "SearchEndTime_s", interceptTime_s, ...
    "InitialValidatedUpperTime_s", interceptTime_s, ...
    "FinalLowerTime_s", interceptTime_s, ...
    "ArrivalTimeTolerance_s", 0, "OptimalityStatement", "The requested intercept time was evaluated.");
end

function [velocity_deg_s, acceleration_deg_s2] = targetDerivatives( ...
        time_s, position_deg, queryTime_s, interpolationMethod)
% Estimate target derivatives from the same selected interpolation.
minimumStep_s = min(diff(time_s));
step_s = max(1e-5, min(1e-2, minimumStep_s / 100));
lowerTime_s = max(time_s(1), queryTime_s - step_s);
upperTime_s = min(time_s(end), queryTime_s + step_s);
centerPosition_deg = interp1( time_s, position_deg, queryTime_s, interpolationMethod);
lowerPosition_deg = interp1( time_s, position_deg, lowerTime_s, interpolationMethod);
upperPosition_deg = interp1( time_s, position_deg, upperTime_s, interpolationMethod);
velocity_deg_s = (upperPosition_deg - lowerPosition_deg) / (upperTime_s - lowerTime_s);
leftDuration_s = max(queryTime_s - lowerTime_s, eps);
rightDuration_s = max(upperTime_s - queryTime_s, eps);
leftVelocity_deg_s = (centerPosition_deg - lowerPosition_deg) / leftDuration_s;
rightVelocity_deg_s = (upperPosition_deg - centerPosition_deg) / rightDuration_s;
acceleration_deg_s2 = 2 * (rightVelocity_deg_s - leftVelocity_deg_s) / (leftDuration_s + rightDuration_s);
end

function value = conditionalText(condition, trueValue, falseValue)
% Select one explicit terminal-state policy label.
if condition
    value = trueValue;
else
    value = falseValue;
end
end
