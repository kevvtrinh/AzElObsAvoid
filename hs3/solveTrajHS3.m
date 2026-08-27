function trajectory = solveTrajHS3( ...
        initialState, terminalState, limits, optionOverrides, pathConstraints)
%% Section 0: Header & Readme
% SYNTAX
%   options = solveTrajHS3()
%   trajectory = solveTrajHS3(initialState, terminalState, limits)
%   trajectory = solveTrajHS3(initialState, terminalState, limits, ...
%       optionOverrides)
%   trajectory = solveTrajHS3(initialState, terminalState, limits, ...
%       optionOverrides, pathConstraints)
%**************************************************************************
% PURPOSE
%   - Create a dimension-neutral fixed-time or earliest-arrival HS3 slew.
%**************************************************************************
% INPUTS
%   - initialState (scalar struct)
%       Fields time, position, velocity, and acceleration use 1-by-D rows.
%   - terminalState (scalar struct)
%       Position, velocity, and acceleration use 1-by-D rows; maximumTime is
%       required for earliest arrival and may supply fixed final time.
%   - limits (scalar struct)
%       Required maximumVelocity, maximumAcceleration, and maximumJerk are
%       positive scalar or 1-by-D values. Optional lower/upper fields support
%       asymmetric coordinate and derivative bounds.
%   - optionOverrides (scalar struct, optional; default struct())
%       Partial hs3Internal.defaultOptions overrides; empty fields use defaults.
%   - pathConstraints (scalar struct, optional; default empty)
%       Tau, optional TauEnd, and LowerBound are M-by-1; Normal is M-by-D.
%       A point row enforces Normal*position(Tau) >= LowerBound. TauEnd>Tau
%       enforces the same affine half-space over the complete interval.
%**************************************************************************
% OUTPUTS
%   - trajectory (scalar struct)
%       Zero inputs return defaults. Planning calls return one stable
%       success-or-failure record with motion, polynomial, validation, and
%       diagnostics. Invalid contracts throw identified errors.
%**************************************************************************
% UNITS
%   - Time and coordinate units are caller-defined and must be consistent.
%**************************************************************************

if nargin == 0
    trajectory = hs3Internal.defaultOptions();
    return;
end
if nargin < 3
    error("solveTrajHS3:NotEnoughInputs", ...
        "initialState, terminalState, and limits are required.");
end
if nargin < 4 || isempty(optionOverrides)
    optionOverrides = struct();
end
if nargin < 5 || isempty(pathConstraints)
    pathConstraints = struct();
end

%% Section 1: Validate Inputs And Apply Defaults

options = resolveOptions(optionOverrides);
[initialState, terminalState, limits, pathConstraints] = ...
    normalizeInputs(initialState, terminalState, limits, pathConstraints);
scaledPathStart = options.SegmentCount * pathConstraints.Tau;
pathSegmentIndex = min( ...
    options.SegmentCount, floor(scaledPathStart) + 1);
pathSegmentEndTau = pathSegmentIndex / options.SegmentCount;
pathIntervalTolerance = 32 * eps(max(1, options.SegmentCount));
if any(pathConstraints.TauEnd > ...
        pathSegmentEndTau + pathIntervalTolerance)
    error("solveTrajHS3:CrossSegmentPathInterval", ...
        "Each path-constraint interval must lie inside one HS3 segment.");
end
hs3Internal.subintervalHullMap(pathConstraints.Tau, pathConstraints.TauEnd, ...
    options.SegmentCount, 1);
dimensionCount = numel(initialState.position);
startTime = initialState.time;
maximumFinalTime = terminalState.maximumTime;
if options.TimeMode == "fixed"
    finalTime = options.FinalTime;
    if isempty(finalTime)
        finalTime = maximumFinalTime;
    end
    validateattributes(finalTime, {'numeric'}, ...
        {'real', 'finite', 'scalar', '>', startTime});
    minimumFinalTime = finalTime;
    maximumFinalTime = finalTime;
else
    validateattributes(maximumFinalTime, {'numeric'}, ...
        {'real', 'finite', 'scalar', '>', startTime});
    displacement = abs(terminalState.position - initialState.position);
    minimumDuration = max([ ...
        1e-3, displacement ./ limits.maximumVelocity]);
    minimumFinalTime = startTime + minimumDuration;
    if minimumFinalTime >= maximumFinalTime
        trajectory = emptyTrajectory( ...
            initialState, terminalState, limits, options, pathConstraints);
        trajectory.Message = ...
            "The time horizon is no longer than the velocity lower bound.";
        trajectory.TerminationReason = "infeasibleTimeHorizon";
        return;
    end
end

%% Section 2: Solve The Requested Time Mode

if options.TimeMode == "fixed"
    solverResult = hs3Internal.solveFixedTime( ...
        initialState, terminalState, limits, options, ...
        pathConstraints, finalTime);
else
    solverResult = hs3Internal.solveFreeTime( ...
        initialState, terminalState, limits, options, pathConstraints, ...
        minimumFinalTime, maximumFinalTime);
end

%% Section 3: Reconstruct And Validate The Motion

trajectory = emptyTrajectory( ...
    initialState, terminalState, limits, options, pathConstraints);
trajectory.Message = solverResult.Message;
trajectory.TerminationReason = solverResult.TerminationReason;
trajectory.Diagnostics = solverResult;
trajectory.FinalTime = solverResult.FinalTime;
trajectory.Duration = solverResult.FinalTime - startTime;
trajectory.MaximumConstraintViolation = ...
    solverResult.MaximumConstraintViolation;
if isempty(solverResult.Decision)
    return;
end
controlCount = 2 * options.SegmentCount + 1;
controlJerk = reshape( ...
    solverResult.Decision, controlCount, dimensionCount);
polynomial = hs3Internal.reconstructPolynomial( ...
    controlJerk, initialState, solverResult.FinalTime, options.SegmentCount);
uniformTime = (startTime:options.SampleTime:solverResult.FinalTime).';
sampleTime = unique([uniformTime; polynomial.SegmentStartTime; ...
    solverResult.FinalTime]);
[sampleTime, position, velocity, acceleration, jerk] = ...
    hs3Internal.evaluatePolynomial(polynomial, sampleTime);
trajectory.time = sampleTime;
trajectory.position = position;
trajectory.velocity = velocity;
trajectory.acceleration = acceleration;
trajectory.jerk = jerk;
trajectory.ControlJerk = controlJerk;
trajectory.Polynomial = polynomial;
trajectory.IntegratedSquaredJerk = hs3Internal.integratedSquaredJerk( ...
    solverResult.Decision, false, solverResult.FinalTime, ...
    options.SegmentCount, startTime, dimensionCount);
trajectory.Validation = hs3Internal.validate(trajectory);
trajectory.Success = solverResult.Success && trajectory.Validation.Passed;
if trajectory.Success
    trajectory.Message = "HS3 returned an independently valid trajectory.";
    trajectory.TerminationReason = "goalReached";
elseif solverResult.Success && ~trajectory.Validation.Passed
    trajectory.Message = trajectory.Validation.Message;
    trajectory.TerminationReason = "validationFailed";
end
end

%% Section 4: Local Functions

function options = resolveOptions(overrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = resolveOptions(overrides)
%**************************************************************************
% PURPOSE
%   - Merge, normalize, and validate standalone HS3 option overrides.
%**************************************************************************
% INPUTS
%   - overrides (scalar struct), partial option values.
%**************************************************************************
% OUTPUTS
%   - options (scalar struct), fully resolved engine options.
%**************************************************************************
% UNITS
%   - Time fields use caller-defined consistent time units.
%**************************************************************************
if ~isstruct(overrides) || ~isscalar(overrides)
    error("solveTrajHS3:InvalidOptions", ...
        "optionOverrides must be a scalar struct.");
end
options = hs3Internal.defaultOptions();
unknownNames = setdiff(string(fieldnames(overrides)), ...
    string(fieldnames(options)), "stable");
if ~isempty(unknownNames)
    warning("solveTrajHS3:UnknownOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", ...
        strjoin(unknownNames, ", "));
end
for fieldName = string(fieldnames(options)).'
    if isfield(overrides, fieldName) && ~isempty(overrides.(fieldName))
        options.(fieldName) = overrides.(fieldName);
    end
end
options.TimeMode = string(options.TimeMode);
if ~isscalar(options.TimeMode) || ...
        ~any(options.TimeMode == ["fixed", "earliestArrival"])
    error("solveTrajHS3:InvalidTimeMode", ...
        "options.TimeMode must be 'fixed' or 'earliestArrival'.");
end
if ~isempty(options.FinalTime)
    validateattributes(options.FinalTime, {'numeric'}, ...
        {'real', 'finite', 'scalar'});
end
for fieldName = ["SegmentCount", "MaximumIterations", ...
        "MaximumFunctionEvaluations"]
    validateattributes(options.(fieldName), {'numeric'}, ...
        {'real', 'finite', 'scalar', 'integer', 'positive'});
end
for fieldName = ["SampleTime", "MaximumSolveTime", ...
        "ArrivalTimeTolerance", "ConstraintTolerance", ...
        "OptimalityTolerance", "StepTolerance"]
    validateattributes(options.(fieldName), {'numeric'}, ...
        {'real', 'finite', 'scalar', 'positive'});
end
if ~(islogical(options.Verbose) && isscalar(options.Verbose)) && ...
        ~(isnumeric(options.Verbose) && isscalar(options.Verbose) && ...
        isfinite(options.Verbose) && any(options.Verbose == [0 1]))
    error("solveTrajHS3:InvalidVerbose", ...
        "options.Verbose must be a scalar logical or binary numeric value.");
end
options.Verbose = logical(options.Verbose);
end

function [initialState, terminalState, limits, pathConstraints] = ...
        normalizeInputs(initialState, terminalState, limits, pathConstraints)
%% Section 0: Header & Readme
% SYNTAX
%   [initialState, terminalState, limits, pathConstraints] = ...
%       normalizeInputs(initialState, terminalState, limits, pathConstraints)
%**************************************************************************
% PURPOSE
%   - Normalize one dimension-neutral HS3 boundary-value problem.
%**************************************************************************
% INPUTS
%   - initialState, terminalState, limits, pathConstraints (scalar structs)
%**************************************************************************
% OUTPUTS
%   - Normalized scalar structs with row-oriented coordinate values.
%**************************************************************************
% UNITS
%   - Values retain caller-defined consistent units.
%**************************************************************************
requiredInitial = ["time", "position", "velocity", "acceleration"];
requiredTerminal = [ ...
    "position", "velocity", "acceleration", "maximumTime"];
if ~isstruct(initialState) || ~isscalar(initialState) || ...
        ~all(isfield(initialState, requiredInitial))
    error("solveTrajHS3:InvalidInitialState", ...
        "initialState must be scalar with time, position, velocity, and acceleration.");
end
if ~isstruct(terminalState) || ~isscalar(terminalState) || ...
        ~all(isfield(terminalState, requiredTerminal))
    error("solveTrajHS3:InvalidTerminalState", ...
        "terminalState must be scalar with position, velocity, acceleration, and maximumTime.");
end
validateattributes(initialState.time, {'numeric'}, ...
    {'real', 'finite', 'scalar'});
initialState.time = double(initialState.time);
initialState = normalizeStateRows(initialState, "initialState");
terminalState = normalizeStateRows(terminalState, "terminalState");
dimensionCount = numel(initialState.position);
if numel(terminalState.position) ~= dimensionCount
    error("solveTrajHS3:DimensionMismatch", ...
        "Initial and terminal states must use the same dimension count.");
end
requiredLimits = [ ...
    "maximumVelocity", "maximumAcceleration", "maximumJerk"];
if ~isstruct(limits) || ~isscalar(limits) || ...
        ~all(isfield(limits, requiredLimits))
    error("solveTrajHS3:InvalidLimits", ...
        "limits must define maximumVelocity, maximumAcceleration, and maximumJerk.");
end
limits.maximumVelocity = expandLimit( ...
    limits.maximumVelocity, dimensionCount, "maximumVelocity", true);
limits.maximumAcceleration = expandLimit( ...
    limits.maximumAcceleration, dimensionCount, "maximumAcceleration", true);
limits.maximumJerk = expandLimit( ...
    limits.maximumJerk, dimensionCount, "maximumJerk", true);
limits = resolveBoundPair(limits, "position", ...
    -Inf(1, dimensionCount), Inf(1, dimensionCount), dimensionCount);
limits = resolveBoundPair(limits, "velocity", ...
    -limits.maximumVelocity, limits.maximumVelocity, dimensionCount);
limits = resolveBoundPair(limits, "acceleration", ...
    -limits.maximumAcceleration, limits.maximumAcceleration, dimensionCount);
limits = resolveBoundPair(limits, "jerk", ...
    -limits.maximumJerk, limits.maximumJerk, dimensionCount);
pathConstraints = normalizePathConstraints(pathConstraints, dimensionCount);
end

function state = normalizeStateRows(state, stateName)
%% Section 0: Header & Readme
% SYNTAX
%   state = normalizeStateRows(state, stateName)
%**************************************************************************
% PURPOSE
%   - Validate and row-normalize position and supported motion derivatives.
%**************************************************************************
% INPUTS
%   - state (scalar struct), boundary state.
%   - stateName (text scalar), diagnostic input name.
%**************************************************************************
% OUTPUTS
%   - state (scalar struct), finite row-oriented state.
%**************************************************************************
% UNITS
%   - Values retain caller-defined consistent units.
%**************************************************************************
for fieldName = ["position", "velocity", "acceleration"]
    value = state.(fieldName);
    validateattributes(value, {'numeric'}, ...
        {'real', 'finite', 'vector', 'nonempty'});
    state.(fieldName) = double(value(:).');
end
dimensionCount = numel(state.position);
if numel(state.velocity) ~= dimensionCount || ...
        numel(state.acceleration) ~= dimensionCount
    error("solveTrajHS3:StateDimensionMismatch", ...
        "%s position, velocity, and acceleration lengths must match.", ...
        stateName);
end
end

function value = expandLimit(value, dimensionCount, fieldName, isPositive)
%% Section 0: Header & Readme
% SYNTAX
%   value = expandLimit(value, dimensionCount, fieldName, isPositive)
%**************************************************************************
% PURPOSE
%   - Expand a scalar limit or validate one value per modeled coordinate.
%**************************************************************************
% INPUTS
%   - value (numeric scalar or vector), supplied limit.
%   - dimensionCount (positive integer scalar), modeled coordinates.
%   - fieldName (text scalar), diagnostic name.
%   - isPositive (logical scalar), requires strictly positive values.
%**************************************************************************
% OUTPUTS
%   - value (1-by-D double), normalized limit.
%**************************************************************************
% UNITS
%   - Values retain the supplied limit's physical units.
%**************************************************************************
validateattributes(value, {'numeric'}, {'real', 'vector', 'nonempty'});
value = double(value(:).');
if isscalar(value)
    value = repmat(value, 1, dimensionCount);
end
if numel(value) ~= dimensionCount || any(isnan(value)) || ...
        (isPositive && any(~isfinite(value) | value <= 0))
    error("solveTrajHS3:InvalidLimit", ...
        "%s must be positive finite scalar or 1-by-%d vector.", ...
        fieldName, dimensionCount);
end
end

function limits = resolveBoundPair( ...
        limits, prefix, defaultLower, defaultUpper, dimensionCount)
%% Section 0: Header & Readme
% SYNTAX
%   limits = resolveBoundPair(limits, prefix, defaultLower, ...
%       defaultUpper, dimensionCount)
%**************************************************************************
% PURPOSE
%   - Resolve optional lower and upper coordinate-wise bounds.
%**************************************************************************
% INPUTS
%   - limits (scalar struct), partial limit record.
%   - prefix (text scalar), bound field prefix.
%   - defaultLower, defaultUpper (1-by-D numeric), symmetric defaults.
%   - dimensionCount (positive integer scalar), modeled coordinates.
%**************************************************************************
% OUTPUTS
%   - limits (scalar struct), normalized lower and upper bound fields.
%**************************************************************************
% UNITS
%   - Bounds retain prefix-specific physical units.
%**************************************************************************
lowerName = prefix + "Lower";
upperName = prefix + "Upper";
lower = defaultLower;
upper = defaultUpper;
if isfield(limits, lowerName) && ~isempty(limits.(lowerName))
    lower = expandLimit(limits.(lowerName), ...
        dimensionCount, lowerName, false);
end
if isfield(limits, upperName) && ~isempty(limits.(upperName))
    upper = expandLimit(limits.(upperName), ...
        dimensionCount, upperName, false);
end
if any(lower >= upper)
    error("solveTrajHS3:InvalidBoundOrder", ...
        "%s must be strictly below %s in every coordinate.", ...
        lowerName, upperName);
end
limits.(lowerName) = lower;
limits.(upperName) = upper;
end

function pathConstraints = normalizePathConstraints( ...
        pathConstraints, dimensionCount)
%% Section 0: Header & Readme
% SYNTAX
%   pathConstraints = normalizePathConstraints( ...
%       pathConstraints, dimensionCount)
%**************************************************************************
% PURPOSE
%   - Normalize optional coordinate-space affine path constraints.
%**************************************************************************
% INPUTS
%   - pathConstraints (scalar struct), partial Tau/Normal/LowerBound record.
%   - dimensionCount (positive integer scalar), modeled coordinates.
%**************************************************************************
% OUTPUTS
%   - pathConstraints (scalar struct), dimensionally consistent rows.
%**************************************************************************
% UNITS
%   - Tau is dimensionless; normals and bounds use caller-defined units.
%**************************************************************************
if ~isstruct(pathConstraints) || ~isscalar(pathConstraints)
    error("solveTrajHS3:InvalidPathConstraints", ...
        "pathConstraints must be a scalar struct or empty.");
end
if isempty(fieldnames(pathConstraints))
    pathConstraints = struct( ...
        "Tau", zeros(0, 1), ...
        "TauEnd", zeros(0, 1), ...
        "Normal", zeros(0, dimensionCount), ...
        "LowerBound", zeros(0, 1));
    return;
end
requiredFields = ["Tau", "Normal", "LowerBound"];
if ~all(isfield(pathConstraints, requiredFields))
    error("solveTrajHS3:InvalidPathConstraints", ...
        "pathConstraints must define Tau, Normal, and LowerBound.");
end
pathConstraints.Tau = double(pathConstraints.Tau(:));
if ~isfield(pathConstraints, "TauEnd") || isempty(pathConstraints.TauEnd)
    pathConstraints.TauEnd = pathConstraints.Tau;
else
    pathConstraints.TauEnd = double(pathConstraints.TauEnd(:));
end
pathConstraints.Normal = double(pathConstraints.Normal);
pathConstraints.LowerBound = double(pathConstraints.LowerBound(:));
constraintCount = numel(pathConstraints.Tau);
if size(pathConstraints.Normal, 1) ~= constraintCount || ...
        size(pathConstraints.Normal, 2) ~= dimensionCount || ...
        numel(pathConstraints.TauEnd) ~= constraintCount || ...
        numel(pathConstraints.LowerBound) ~= constraintCount || ...
        any(~isfinite(pathConstraints.Tau)) || ...
        any(pathConstraints.Tau < 0 | pathConstraints.Tau > 1) || ...
        any(~isfinite(pathConstraints.TauEnd)) || ...
        any(pathConstraints.TauEnd < pathConstraints.Tau | ...
        pathConstraints.TauEnd > 1) || ...
        any(~isfinite(pathConstraints.Normal), "all") || ...
        any(~isfinite(pathConstraints.LowerBound))
    error("solveTrajHS3:InvalidPathConstraints", ...
        "Path rows require ordered Tau/TauEnd in [0,1], M-by-D Normal, " + ...
        "and M-by-1 LowerBound.");
end
end

function trajectory = emptyTrajectory( ...
        initialState, terminalState, limits, options, pathConstraints)
%% Section 0: Header & Readme
% SYNTAX
%   trajectory = emptyTrajectory(initialState, terminalState, limits, ...
%       options, pathConstraints)
%**************************************************************************
% PURPOSE
%   - Define one stable standalone HS3 success-or-failure schema.
%**************************************************************************
% INPUTS
%   - initialState, terminalState, limits, options, pathConstraints
%       Fully resolved engine inputs.
%**************************************************************************
% OUTPUTS
%   - trajectory (scalar struct), documented empty failure record.
%**************************************************************************
% UNITS
%   - Fields retain caller-defined consistent units.
%**************************************************************************
dimensionCount = numel(initialState.position);
trajectory = struct( ...
    "Success", false, ...
    "Message", "HS3 has not produced a trajectory.", ...
    "TerminationReason", "notRun", ...
    "Inputs", struct( ...
    "initialState", initialState, "terminalState", terminalState, ...
    "limits", limits, "pathConstraints", pathConstraints), ...
    "Options", options, ...
    "time", zeros(0, 1), ...
    "position", zeros(0, dimensionCount), ...
    "velocity", zeros(0, dimensionCount), ...
    "acceleration", zeros(0, dimensionCount), ...
    "jerk", zeros(0, dimensionCount), ...
    "ControlJerk", zeros(0, dimensionCount), ...
    "Polynomial", struct(), ...
    "FinalTime", NaN, ...
    "Duration", NaN, ...
    "IntegratedSquaredJerk", Inf, ...
    "MaximumConstraintViolation", Inf, ...
    "Validation", struct( ...
    "Passed", false, "Message", "No trajectory is available."), ...
    "Diagnostics", struct());
end
