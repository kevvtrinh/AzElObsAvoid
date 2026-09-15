function request = createSolveRequest(seed, regions_units, coverage, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   request = bmtpEngine.createSolveRequest(seed, regions_units, coverage, ...
%       initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Validate BMTP inputs and select the polynomial representation.
%   - Collect shared geometry, timing, and solver controls.
%**************************************************************************
% INPUTS
%   - seed (scalar struct)
%       Ordered positions, route progress, and optional clock declarations.
%   - regions_units (column cell array)
%       Convex exclusion regions.
%   - coverage (scalar struct)
%       Authoritative geometry and time-coverage evidence.
%   - initialState (scalar struct)
%       Initial time, position, velocity, and acceleration.
%   - goalState (scalar struct)
%       Goal time, position, velocity, and acceleration.
%   - limits (scalar struct)
%       Workspace and per-axis derivative limits.
%   - options (scalar struct)
%       Resolved timing, sampling, and clearance controls.
%**************************************************************************
% OUTPUTS
%   - request (scalar struct)
%       Validated representation, geometry, timing, and solver controls;
%       invalid input throws an error.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Check The Engine Inputs
% Validate convex static regions before solving. The caller normalizes both
% endpoint states completely, so the engine checks the supplied derivatives
% and never substitutes a missing one.
for derivativeName = ["velocity_units_s", "acceleration_units_s2"]
    validateattributes(initialState.(derivativeName), {'numeric'}, {'real', 'finite', 'size', [1, 2]});
    validateattributes(goalState.(derivativeName), {'numeric'}, {'real', 'finite', 'size', [1, 2]});
end
validateKernelInputs(seed, regions_units, coverage, initialState, goalState, limits, options);

%% Section 2: Select The Polynomial Representation
% Choose the representation from the physical request, never from a
% diagnostic seed label. The seed's producer declares its own physical clock
% with typed logical fields. Every non-static timed proposal uses the same
% quintic C3 representation; its physical clock controls only the time mesh.

[degree, splitCount] = deal(5, 3);
if options.GoalTimeMode == "earliestArrival" && ~isfield(coverage, 'ActiveTimeInterval_s')
    degree = 8;
end
usesVariableClock = isfield(seed, 'UsesVariableClock') && seed.UsesVariableClock;
usesTimeScopedSolver = usesVariableClock || ...
    (isfield(seed, 'UsesTimeScopedSolver') && seed.UsesTimeScopedSolver);
motionHorizon_s = goalState.time_s - initialState.time_s;
if motionHorizon_s <= 0
    error("bmtpEngine:InvalidGoalTime", "goalState.time_s must be greater than initialState.time_s.");
end

%% Section 3: Prepare Shared Solver Controls
% Set shared tolerances and iteration limits for both conic solvers.

regionMinimum_units    = zeros(numel(regions_units), 2);
regionMaximum_units    = zeros(numel(regions_units), 2);
separatingLineGeometry = cell(numel(regions_units), 1);
for regionIndex = 1:numel(regions_units)
    regionMinimum_units(regionIndex, :) = min(regions_units{regionIndex}, [], 1);
    regionMaximum_units(regionIndex, :) = max(regions_units{regionIndex}, [], 1);
    endRegion_units = regions_units{regionIndex};
    if isfield(coverage, 'EndRegions_units')
        endRegion_units = coverage.EndRegions_units{regionIndex};
    end
    separatingLineGeometry{regionIndex} = createSeparatingLineGeometry( ...
        regions_units{regionIndex}, endRegion_units);
end
maximumTrajectoryIterations = 300;
trajectoryOptions = optimoptions("coneprog", ...
    "Display",             "none", ...
    "MaxIterations",       maximumTrajectoryIterations, ...
    "ConstraintTolerance",  1e-10, ...
    "OptimalityTolerance",  1e-9);
% The variable-clock formulation is tuned at the solver's default tolerances,
% and tightening them moved measured outcomes in both directions, so any
% change is a separate tuning decision.
timedTrajectoryOptions = optimoptions("coneprog", ...
    "Display",       "none", ...
    "MaxIterations", maximumTrajectoryIterations);
maximumAlternatingIterations = 35;
if isfield(seed, 'MaximumAlternatingIterations')
    validateattributes(seed.MaximumAlternatingIterations, {'numeric'}, ...
        {'real', 'finite', 'scalar', 'integer', 'positive', '<=', 35});
    maximumAlternatingIterations = double(seed.MaximumAlternatingIterations);
end
minimumMotionDuration_s = 0;
if isfield(coverage, 'MinimumMotionDuration_s')
    minimumMotionDuration_s = double(coverage.MinimumMotionDuration_s);
    validateattributes(minimumMotionDuration_s, {'numeric'}, ...
        {'real', 'finite', 'scalar', 'nonnegative', '<=', motionHorizon_s});
end
seedMotionDuration_s = 0;
if isfield(coverage, 'SeedMotionDuration_s')
    seedMotionDuration_s = double(coverage.SeedMotionDuration_s);
    validateattributes(seedMotionDuration_s, {'numeric'}, ...
        {'real', 'finite', 'scalar', 'nonnegative', '<=', motionHorizon_s});
end
requestIsRest = all([initialState.velocity_units_s, initialState.acceleration_units_s2, ...
    goalState.velocity_units_s, goalState.acceleration_units_s2] == 0);
request = struct( ...
    "Seed",                         seed, ...
    "Regions_units",                {regions_units}, ...
    "Coverage",                     coverage, ...
    "InitialState",                 initialState, ...
    "GoalState",                    goalState, ...
    "Limits",                       limits, ...
    "Options",                      options, ...
    "IsRest",                       requestIsRest, ...
    "Degree",                       degree, ...
    "SplitCount",                   splitCount, ...
    "MotionHorizon_s",              motionHorizon_s, ...
    "MinimumMotionDuration_s",      minimumMotionDuration_s, ...
    "SeedMotionDuration_s",         seedMotionDuration_s, ...
    "UsesVariableClock",            usesVariableClock, ...
    "UsesTimeScopedSolver",         usesTimeScopedSolver, ...
    "RegionMinimum_units",          regionMinimum_units, ...
    "RegionMaximum_units",          regionMaximum_units, ...
    "SeparatingLineGeometry",       {separatingLineGeometry}, ...
    "MaximumAlternatingIterations", maximumAlternatingIterations, ...
    "TrajectoryOptions",            trajectoryOptions, ...
    "TimedTrajectoryOptions",       timedTrajectoryOptions);
end

%% Section 4: Local Functions
function validateKernelInputs(seed, regions_units, coverage, initialState, goalState, limits, options)
    % Check engine-specific input restrictions.
    if ~isstruct(seed) || ~isscalar(seed) || ~all(isfield(seed, {'position_units', 'tau'}))
        error("bmtpEngine:InvalidSeed", "seed must be scalar and contain position_units and tau.");
    end
    tau         = double(seed.tau(:));
    route_units = double(seed.position_units);
    seedIsValid = size(route_units, 2) == 2 && ...
        size(route_units, 1) == numel(tau) && ...
        all(isfinite(route_units), "all") && numel(tau) >= 2 && ...
        all(isfinite(tau)) && all(diff(tau) > 0) && ...
        abs(tau(1)) <= 32 * eps && abs(tau(end) - 1) <= 32 * eps;
    if ~seedIsValid
        error("bmtpEngine:InvalidSeedTau", "seed.position_units must be finite N-by-2 and tau must increase 0 to 1.");
    end

    regionsAreValid = iscell(regions_units) && iscolumn(regions_units);
    for regionIndex = 1:numel(regions_units)
        region_units = regions_units{regionIndex};
        regionsAreValid = regionsAreValid && isnumeric(region_units) && ...
            size(region_units, 2) == 2 && ...
            size(region_units, 1) >= 3 && all(isfinite(region_units), "all");
    end

    coverageIsValid = isstruct(coverage) && isscalar(coverage) && ...
        isfield(coverage, "Passed") && islogical(coverage.Passed) && isscalar(coverage.Passed);
    if ~(regionsAreValid && coverageIsValid)
        error("bmtpEngine:InvalidExclusionRegions", ...
            "regions_units must be a column cell array of finite N-by-2 polygons " + ...
            "and coverage must contain scalar logical Passed.");
    end

    endpointDerivatives = [initialState.velocity_units_s, ...
        initialState.acceleration_units_s2, goalState.velocity_units_s, goalState.acceleration_units_s2];
    derivativeLimits = [limits.maxVelocity_units_s; limits.maxAcceleration_units_s2; ...
        limits.maxJerk_units_s3];
    requestIsSupported = all(isfinite(endpointDerivatives)) && ...
        any(string(options.GoalTimeMode) == ["fixedArrival", "earliestArrival"]) && ...
        options.SampleTime_s > 0 && isequal(size(derivativeLimits), [3, 2]) && ...
        all(isfinite(derivativeLimits), "all") && all(derivativeLimits > 0, "all");
    if ~requestIsSupported
        error("bmtpEngine:UnsupportedRequest", "The BMTP kernel requires a finite unwrapped full-state request.");
    end
end

function geometry = createSeparatingLineGeometry(first_units, last_units)
    % Obstacle-edge directions and supports do not change while alternating
    % the trajectory. Cache them once; curve-hull directions remain per trial.
    edges_units = diff([first_units; first_units(1, :)], 1, 1);
    if ~isequal(first_units, last_units)
        edges_units = [edges_units; diff([last_units; last_units(1, :)], 1, 1)];
    end
    edgeLengths_units = vecnorm(edges_units, 2, 2);
    edges_units       = edges_units(edgeLengths_units > 0, :);
    edgeLengths_units = edgeLengths_units(edgeLengths_units > 0);
    positiveNormals       = [-edges_units(:, 2), edges_units(:, 1)] ./ edgeLengths_units;
    firstProjection_units = first_units * positiveNormals.';
    lastProjection_units  = last_units * positiveNormals.';
    geometry = struct( ...
        'PositiveNormals',                  positiveNormals, ...
        'FirstPositiveSupport_units',       min(firstProjection_units, [], 1), ...
        'LastPositiveSupport_units',        min(lastProjection_units, [], 1), ...
        'FirstNegativeSupport_units',       -max(firstProjection_units, [], 1), ...
        'LastNegativeSupport_units',        -max(lastProjection_units, [], 1));
end
