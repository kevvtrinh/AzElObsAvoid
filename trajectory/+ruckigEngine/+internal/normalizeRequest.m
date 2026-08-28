function [initialState, terminalState, limits, pathConstraints] = ...
        normalizeRequest(initialState, terminalState, limits, pathConstraints)
%% Section 0: Header & Readme
% SYNTAX
%   [initialState, terminalState, limits, pathConstraints] = ...
%       ruckigEngine.internal.normalizeRequest( ...
%       initialState, terminalState, limits, pathConstraints)
%**************************************************************************
% PURPOSE
%   - Normalize state, limit, and path inputs for the Ruckig engine.
%**************************************************************************
% INPUTS
%   - initialState (scalar struct)
%       Fields time, position, velocity, and acceleration are required.
%   - terminalState (scalar struct)
%       Position, velocity, acceleration, and maximumTime are required.
%   - limits (scalar struct)
%       Positive maximum derivative limits are required; optional lower and
%       upper bounds may be asymmetric.
%   - pathConstraints (scalar struct or empty)
%       Tau/TauEnd, Normal, and LowerBound describe affine path rows.
%**************************************************************************
% OUTPUTS
%   - initialState, terminalState (scalar structs)
%       Motion vectors are normalized to 1-by-D double rows.
%   - limits (scalar struct)
%       Scalar maxima are expanded and every lower/upper pair is resolved.
%   - pathConstraints (scalar struct)
%       Empty input becomes a stable zero-row path-constraint record.
%**************************************************************************
% UNITS
%   - Units are caller-defined and must be consistent across derivatives.
%**************************************************************************

%% Section 1: Normalize States

requiredInitial = ["time", "position", "velocity", "acceleration"];
requiredTerminal = [ ...
    "position", "velocity", "acceleration", "maximumTime"];
if ~isstruct(initialState) || ~isscalar(initialState) || ...
        ~all(isfield(initialState, requiredInitial))
    error("ruckigEngine:InvalidInitialState", ...
        "initialState must define time, position, velocity, and acceleration.");
end
if ~isstruct(terminalState) || ~isscalar(terminalState) || ...
        ~all(isfield(terminalState, requiredTerminal))
    error("ruckigEngine:InvalidTerminalState", ...
        "terminalState must define position, velocity, acceleration, and maximumTime.");
end
validateattributes(initialState.time, {'numeric'}, ...
    {'real', 'finite', 'scalar'});
validateattributes(terminalState.maximumTime, {'numeric'}, ...
    {'real', 'finite', 'scalar', '>', initialState.time});
initialState.time = double(initialState.time);
terminalState.maximumTime = double(terminalState.maximumTime);
initialState = normalizeStateRows(initialState, "initialState");
terminalState = normalizeStateRows(terminalState, "terminalState");
dimensionCount = numel(initialState.position);
if numel(terminalState.position) ~= dimensionCount
    error("ruckigEngine:DimensionMismatch", ...
        "Initial and terminal states must use the same dimension count.");
end

%% Section 2: Normalize Limits

requiredLimits = [ ...
    "maximumVelocity", "maximumAcceleration", "maximumJerk"];
if ~isstruct(limits) || ~isscalar(limits) || ...
        ~all(isfield(limits, requiredLimits))
    error("ruckigEngine:InvalidLimits", ...
        "limits must define maximumVelocity, maximumAcceleration, and maximumJerk.");
end
limits.maximumVelocity = expandLimit( ...
    limits.maximumVelocity, dimensionCount, "maximumVelocity", true);
limits.maximumAcceleration = expandLimit( ...
    limits.maximumAcceleration, dimensionCount, ...
    "maximumAcceleration", true);
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

%% Section 3: Normalize Path Constraints

if nargin < 4 || isempty(pathConstraints)
    pathConstraints = struct();
end
pathConstraints = normalizePathConstraints(pathConstraints, dimensionCount);
end

%% Section 4: Local Functions

function state = normalizeStateRows(state, stateName)
% Validate one motion state and standardize every coordinate vector to a row.
for fieldName = ["position", "velocity", "acceleration"]
    value = state.(fieldName);
    validateattributes(value, {'numeric'}, ...
        {'real', 'finite', 'vector', 'nonempty'});
    state.(fieldName) = double(value(:).');
end
dimensionCount = numel(state.position);
if numel(state.velocity) ~= dimensionCount || ...
        numel(state.acceleration) ~= dimensionCount
    error("ruckigEngine:StateDimensionMismatch", ...
        "%s position, velocity, and acceleration lengths must match.", ...
        stateName);
end
end

function value = expandLimit(value, dimensionCount, fieldName, isPositive)
% Expand a scalar limit or validate one finite-or-infinite value per axis.
validateattributes(value, {'numeric'}, {'real', 'vector', 'nonempty'});
value = double(value(:).');
if isscalar(value)
    value = repmat(value, 1, dimensionCount);
end
hasInvalidValue = numel(value) ~= dimensionCount || any(isnan(value));
if isPositive
    hasInvalidValue = hasInvalidValue || any(~isfinite(value) | value <= 0);
end
if hasInvalidValue
    error("ruckigEngine:InvalidLimit", ...
        "%s must be a valid scalar or 1-by-%d vector.", ...
        fieldName, dimensionCount);
end
end

function limits = resolveBoundPair( ...
        limits, prefix, defaultLower, defaultUpper, dimensionCount)
% Resolve optional coordinate-wise lower and upper bounds without clipping.
lowerName = prefix + "Lower";
upperName = prefix + "Upper";
lower = defaultLower;
upper = defaultUpper;
if isfield(limits, lowerName) && ~isempty(limits.(lowerName))
    lower = expandLimit( ...
        limits.(lowerName), dimensionCount, lowerName, false);
end
if isfield(limits, upperName) && ~isempty(limits.(upperName))
    upper = expandLimit( ...
        limits.(upperName), dimensionCount, upperName, false);
end
if any(lower >= upper)
    error("ruckigEngine:InvalidBoundOrder", ...
        "%s must be strictly below %s in every coordinate.", ...
        lowerName, upperName);
end
limits.(lowerName) = lower;
limits.(upperName) = upper;
end

function pathConstraints = normalizePathConstraints( ...
        pathConstraints, dimensionCount)
% Publish one stable affine path structure for Ruckig validation.
if ~isstruct(pathConstraints) || ~isscalar(pathConstraints)
    error("ruckigEngine:InvalidPathConstraints", ...
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
    error("ruckigEngine:InvalidPathConstraints", ...
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
hasInvalidSize = size(pathConstraints.Normal, 1) ~= constraintCount || ...
    size(pathConstraints.Normal, 2) ~= dimensionCount || ...
    numel(pathConstraints.TauEnd) ~= constraintCount || ...
    numel(pathConstraints.LowerBound) ~= constraintCount;
hasInvalidValue = any(~isfinite(pathConstraints.Tau)) || ...
    any(pathConstraints.Tau < 0 | pathConstraints.Tau > 1) || ...
    any(~isfinite(pathConstraints.TauEnd)) || ...
    any(pathConstraints.TauEnd < pathConstraints.Tau | ...
    pathConstraints.TauEnd > 1) || ...
    any(~isfinite(pathConstraints.Normal), "all") || ...
    any(~isfinite(pathConstraints.LowerBound));
if hasInvalidSize || hasInvalidValue
    error("ruckigEngine:InvalidPathConstraints", ...
        "Path rows require ordered Tau/TauEnd in [0,1], M-by-D Normal, and M-by-1 LowerBound.");
end
end
