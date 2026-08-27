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
%   - Convert sampled target motion into fixed-time HS3 planning requests.
%   - Specified-time mode makes one planner call.
%   - Earliest mode tests times in order. It then divides the first observed
%     failed-to-passed time interval into smaller intervals.
%**************************************************************************
% INPUTS
%   - obstacles (canonical protected obstacle array, optional; default [])
%   - initialState (scalar state struct)
%   - targetMotion (scalar struct)
%       time_s must increase. position_deg is N-by-2. The optional
%       InterpolationMethod equal to "linear" or "pchip".
%   - limits (scalar limits struct)
%   - options (scalar struct, optional; default struct())
%       InterceptMode is "earliest" or "specifiedTime".
%       SpecifiedInterceptTime_s is required for specifiedTime.
%       MaximumSearchDuration_s defaults to 60.
%       MatchTargetVelocity and MatchTargetAcceleration default false.
%       PlannerOptions is a partial planTrajectory option struct.
%**************************************************************************
% OUTPUTS
%   - result (scalar planTrajectory result)
%       The result adds one Intercept record. This function uses no other
%       motion planner.
%**************************************************************************
% UNITS
%   - Position is degrees, time is seconds, and target derivatives use
%     degrees per second and degrees per second squared.
%**************************************************************************

%% Section 1: Resolve Call Form And Options

% This function processes intercept options only. PlannerOptions contains the
% motion-planner options. planTrajectory validates those options. This
% separation lets each trial use the same motion and safety rules.
defaults = struct( ...
    "InterceptMode", "earliest", ...
    "SpecifiedInterceptTime_s", NaN, ...
    "MaximumSearchDuration_s", 60, ...
    "MatchTargetVelocity", false, "MatchTargetAcceleration", false, ...
    "PlannerOptions", struct());
if nargin == 0
    result = defaults;
    return;
elseif nargin == 4
    % The short call form defines an intercept problem with no obstacles.
    obstacles = [];
    initialState = varargin{1};
    targetMotion = varargin{2};
    limits = varargin{3};
    optionOverrides = varargin{4};
elseif nargin == 5
    % The full call form puts obstacles first. This matches planTrajectory.
    obstacles = varargin{1};
    initialState = varargin{2};
    targetMotion = varargin{3};
    limits = varargin{4};
    optionOverrides = varargin{5};
else
    error("planMovingTargetIntercept:InvalidCall", "Use zero, four, or five inputs as documented.");
end
if isempty(optionOverrides)
    optionOverrides = struct();
end
if ~isstruct(optionOverrides) || ~isscalar(optionOverrides)
    error("planMovingTargetIntercept:InvalidOptions", "options must be a scalar struct.");
end
[options, unknownNames] = obstacleAvoidance.input.resolveOptions( ...
    defaults, optionOverrides);
if ~isempty(unknownNames)
    warning("planMovingTargetIntercept:UnknownOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", strjoin(unknownNames, ", "));
end
options.InterceptMode = string(options.InterceptMode);
if ~isscalar(options.InterceptMode) || ~any(options.InterceptMode == ["earliest", "specifiedTime"])
    error("planMovingTargetIntercept:InvalidMode", "InterceptMode must be 'earliest' or 'specifiedTime'.");
end
logicalNames = ["MatchTargetVelocity", "MatchTargetAcceleration"];

% Convert both derivative-matching controls to scalar logical values.
for name = logicalNames
    options.(name) = obstacleAvoidance.input.normalizeLogicalScalar( ...
        options.(name), name, "planMovingTargetIntercept:InvalidLogicalOption");
end
validateattributes(options.MaximumSearchDuration_s, {'numeric'}, {'real', 'finite', 'scalar', 'positive'});
if ~isstruct(options.PlannerOptions) || ~isscalar(options.PlannerOptions)
    error("planMovingTargetIntercept:InvalidPlannerOptions", "PlannerOptions must be a scalar struct.");
end
%% Section 2: Normalize The Sampled Target

% Use one time-vector orientation and one interpolation method. Target position
% and calculated target derivatives then use the same sampled data. Convert
% time to a column. Each later interp1 call then returns one row with two
% coordinates.
if ~isstruct(targetMotion) || ~isscalar(targetMotion) || ~all(isfield(targetMotion, {'time_s', 'position_deg'}))
    error("planMovingTargetIntercept:InvalidTargetMotion", "targetMotion must contain time_s and position_deg.");
end
validateattributes(targetMotion.time_s, {'numeric'}, {'real', 'finite', 'vector', 'increasing'});
targetTime_s = double(targetMotion.time_s(:));
if numel(targetTime_s) < 2
    error("planMovingTargetIntercept:TargetHistoryTooShort", ...
        "targetMotion.time_s must contain at least two samples.");
end
validateattributes(targetMotion.position_deg, {'numeric'}, ...
    {'real', 'finite', '2d', 'ncols', 2, 'nrows', numel(targetTime_s)});
targetPosition_deg = double(targetMotion.position_deg);
interpolationMethod = "linear";
% Linear interpolation connects adjacent samples with straight lines. PCHIP
% gives a smooth curve and limits unwanted overshoot between samples.
if isfield(targetMotion, "InterpolationMethod") && ~isempty(targetMotion.InterpolationMethod)
    interpolationMethod = string(targetMotion.InterpolationMethod);
end
if ~isscalar(interpolationMethod) || ~any(interpolationMethod == ["linear", "pchip"])
    error("planMovingTargetIntercept:InvalidInterpolation", "InterpolationMethod must be 'linear' or 'pchip'.");
end

%% Section 3: Adapt The Goal And Call The Single Planner

% Each trial is a normal fixed-arrival planTrajectory request. All trials use
% the same collision checks and physical limits as other plans.
initialTime_s = double(initialState.time_s);
if options.InterceptMode == "specifiedTime"
    % A specified intercept time needs one planning attempt. Reject a time
    % outside the target data. Do not estimate target motion beyond supplied
    % samples.
    interceptTime_s = double(options.SpecifiedInterceptTime_s);
    validateattributes(interceptTime_s, {'numeric'}, {'real', 'finite', 'scalar', '>', initialTime_s});
    if interceptTime_s < targetTime_s(1) || interceptTime_s > targetTime_s(end)
        error("planMovingTargetIntercept:InterceptOutsideHistory", ...
            "SpecifiedInterceptTime_s must be inside targetMotion.time_s.");
    end
    [result, interceptSearch] = planAtInterceptTime( ...
        obstacles, initialState, targetTime_s, targetPosition_deg, ...
        interpolationMethod, limits, options, interceptTime_s);
else
    % Earliest mode changes arrival time during the search. Target derivatives
    % also change with time. Matching them would change two parts of the goal
    % at once. Therefore, earliest mode uses zero terminal velocity and
    % acceleration.
    if options.MatchTargetVelocity || options.MatchTargetAcceleration
        error("planMovingTargetIntercept:UnsupportedMovingDerivative", ...
            "Earliest intercept supports explicit zero terminal velocity " + "and acceleration only.");
    end
    [result, ~, interceptSearch] = searchEarliestIntercept( ...
        obstacles, initialState, targetTime_s, targetPosition_deg, ...
        interpolationMethod, limits, options);
end
if result.Success
    % Evaluate the target again at the returned final time. Store the target
    % state that the returned motion reached. Do not copy only the trial value.
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
        obstacles, initialState, targetTime_s, targetPosition_deg, ...
        interpolationMethod, limits, options)
% Find the first validated intercept on a finite time grid. Then refine the
% first observed interval between a failed trial and a passed trial.
plannerDefaults = obstacleAvoidance.planTrajectory();
% ArrivalTimeTolerance has two uses. It sets the earliest allowed time after
% the initial time. It also sets the final width of the refinement interval.
arrivalTolerance_s = plannerDefaults.ArrivalTimeTolerance_s;
if isfield(options.PlannerOptions, "ArrivalTimeTolerance_s")
    arrivalTolerance_s = options.PlannerOptions.ArrivalTimeTolerance_s;
end
searchStart_s = max(targetTime_s(1), initialState.time_s + arrivalTolerance_s);
searchEnd_s = min(targetTime_s(end), initialState.time_s + options.MaximumSearchDuration_s);
if searchEnd_s <= searchStart_s
    error("planMovingTargetIntercept:EmptySearchWindow", ...
        "The target history and MaximumSearchDuration_s do not overlap " + "after initialState.time_s.");
end
coarseIntervalCount = 16;
% Combine a uniform grid with all supplied target times. The uniform grid
% limits the number of planner calls. The original times include corners or
% rapid changes in the sampled target path.
coarseTime_s = unique([ ...
    linspace(searchStart_s, searchEnd_s, coarseIntervalCount + 1).'; ...
    targetTime_s(targetTime_s >= searchStart_s & targetTime_s <= searchEnd_s)]);
trialCount = 0;
selectedTime_s = NaN;
result = [];
lowerTime_s = searchStart_s;

% Test coarse intercept times from earliest to latest. Stop at the first passed
% plan. The previous trial failed and the current trial passed. These two times
% form the observed refinement interval. This search does not prove that all
% later times are feasible.
for timeIndex = 1:numel(coarseTime_s)
    queryTime_s = coarseTime_s(timeIndex);
    [trial, ~] = planAtInterceptTime( ...
        obstacles, initialState, targetTime_s, targetPosition_deg, ...
        interpolationMethod, limits, options, queryTime_s);
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

% Divide the failed-to-passed time interval into halves. A passed midpoint
% lowers the passed upper time. A failed midpoint raises the failed lower time.
% The iteration limit prevents an endless search near an unstable boundary.
while isfinite(selectedTime_s) && ...
        selectedTime_s - lowerTime_s > arrivalTolerance_s && refinementCount < maximumRefinementCount
    queryTime_s = 0.5 * (lowerTime_s + selectedTime_s);
    [trial, ~] = planAtInterceptTime( ...
        obstacles, initialState, targetTime_s, targetPosition_deg, ...
        interpolationMethod, limits, options, queryTime_s);
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
    "Policy", "boundedChronologicalFixedTime", ...
    "TrialCount", trialCount, ...
    "CoarseIntervalCount", coarseIntervalCount, ...
    "RefinementCount", refinementCount, ...
    "SearchStartTime_s", searchStart_s, ...
    "SearchEndTime_s", searchEnd_s, ...
    "InitialValidatedUpperTime_s", initialUpperTime_s, ...
    "FinalLowerTime_s", lowerTime_s, ...
    "ArrivalTimeTolerance_s", arrivalTolerance_s);
end

function [result, search] = planAtInterceptTime( ...
        obstacles, initialState, targetTime_s, targetPosition_deg, ...
        interpolationMethod, limits, options, interceptTime_s)
% Solve one fixed-time intercept with the public production planner.
% If one trial fails, inspect its termination reason before assuming that the
% target time is too short. Search, collision, or optimizer failure can also
% cause a failed trial.
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
result = obstacleAvoidance.planTrajectory( ...
    obstacles, initialState, goalState, limits, plannerOptions);
search = struct( ...
    "Policy", "specifiedFixedTime", "TrialCount", 1, ...
    "CoarseIntervalCount", 0, "RefinementCount", 0, ...
    "SearchStartTime_s", interceptTime_s, ...
    "SearchEndTime_s", interceptTime_s, ...
    "InitialValidatedUpperTime_s", interceptTime_s, ...
    "FinalLowerTime_s", interceptTime_s, ...
    "ArrivalTimeTolerance_s", 0);
end

function [velocity_deg_s, acceleration_deg_s2] = targetDerivatives( ...
        time_s, position_deg, queryTime_s, interpolationMethod)
% Estimate target derivatives with the selected interpolation method. Use a
% centered time difference when possible. Use one-sided differences at the
% ends of the target data.
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
% Return one clear label for the selected terminal-state rule.
if condition
    value = trueValue;
else
    value = falseValue;
end
end
