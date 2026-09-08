function request = createSolveRequest(seed, regions_units, coverage, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   request = bmtpEngine.createSolveRequest( ...
%       seed, regions_units, coverage, initialState, goalState, limits, options)
%
% PURPOSE
%   - Check BMTP inputs and select the established polynomial representation.
%   - Collect horizon, region, objective, and numerical solver controls once.
%
% INPUTS
%   - seed (scalar route-seed struct)
%       Ordered positions and normalized route progress.
%   - regions_units (R-by-1 cell array)
%       Convex exclusion polygons.
%   - coverage (scalar struct)
%       Static region-coverage evidence from the caller.
%   - initialState, goalState, limits, options (scalar structs)
%       Dimension-neutral boundary request, limits, and resolved controls.
%
% OUTPUTS
%   - request (scalar struct)
%       Validated inputs, representation choice, horizon, region bounds,
%       objective rate, and numerical solver options.
%
% UNITS
%   - Position is coordinate units and time is seconds; derivatives use units/s,
%     units/s^2, and units/s^3.
%

%% Section 1: Check The Engine Inputs

% Validate convex static regions before solving.

validateKernelInputs(seed, regions_units, coverage, initialState, goalState, limits, options);

%% Section 2: Select The Polynomial Representation

% Fixed-time event meshes already resolve time. Earliest-arrival optimization
% uses four degree-eight subspans per visibility edge to resolve transients.

[degree, splitCount] = deal(8, 1);
if options.GoalTimeMode=="earliestArrival", splitCount = 4; end
motionHorizon_s = goalState.time_s - initialState.time_s;
if motionHorizon_s <= 0
    error("bmtpEngine:InvalidGoalTime", "goalState.time_s must be greater than initialState.time_s.");
end
%% Section 3: Prepare Shared Solver Controls

% Set shared tolerances and iteration limits for both conic solvers.

regionMinimum_units = zeros(numel(regions_units), 2);
regionMaximum_units = zeros(numel(regions_units), 2);
% Process each geometric region while constructing or checking the region topology.
for regionIndex = 1:numel(regions_units)
    regionMinimum_units(regionIndex, :) = min(regions_units{regionIndex}, [], 1);
    regionMaximum_units(regionIndex, :) = max(regions_units{regionIndex}, [], 1);
end
maximumTrajectoryIterations = 300;
trajectoryOptions           = optimoptions("coneprog", "Display", "none", "MaxIterations", maximumTrajectoryIterations, 'ConstraintTolerance',1e-10,'OptimalityTolerance',1e-9);
request                     = struct("Seed", seed, ...
    "Regions_units", {regions_units}, ...
    "Coverage", coverage, ...
    "InitialState", initialState, ...
    "GoalState", goalState, ...
    "Limits", limits, ...
    "Options", options, ...
    "Degree", degree, ...
    "SplitCount", splitCount, ...
    "MotionHorizon_s", motionHorizon_s, ...
    "RegionMinimum_units", regionMinimum_units, ...
    "RegionMaximum_units", regionMaximum_units, ...
    "TrajectoryOptions", trajectoryOptions);
end

%% Section 4: Local Functions

function validateKernelInputs(seed, regions_units, coverage, initialState, goalState, limits, options)
    % Check engine-specific input restrictions.
    if ~isstruct(seed) || ~isscalar(seed) || ~all(isfield(seed, {'position_units', 'tau'}))
        error("bmtpEngine:InvalidSeed", "seed must be scalar and contain position_units and tau.");
    end
    tau         = double(seed.tau(:));
    route_units   = double(seed.position_units);
    seedIsValid = size(route_units, 2) == 2 && size(route_units, 1) == numel(tau) && all(isfinite(route_units), "all") && numel(tau) >= 2 && all(isfinite(tau)) && all(diff(tau) > 0) && abs(tau(1)) <= 32 * eps && abs(tau(end) - 1) <= 32 * eps;
    if ~seedIsValid
        error("bmtpEngine:InvalidSeedTau", "seed.position_units must be finite N-by-2 and tau must increase 0 to 1.");
    end
    regionsAreValid = iscell(regions_units) && iscolumn(regions_units);
    % Process each geometric region while constructing or checking the region topology.
    for regionIndex = 1:numel(regions_units)
        region_units      = regions_units{regionIndex};
        regionsAreValid = regionsAreValid && isnumeric(region_units) && size(region_units, 2) == 2 && size(region_units, 1) >= 3 && all(isfinite(region_units), "all");
    end
    coverageIsValid = isstruct(coverage) && isscalar(coverage) && isfield(coverage, "Passed") && islogical(coverage.Passed) && isscalar(coverage.Passed);
    if ~(regionsAreValid && coverageIsValid)
        error("bmtpEngine:InvalidExclusionRegions", "regions_units must be a column cell array of finite N-by-2 polygons " + "and coverage must contain scalar logical Passed.");
    end
    endpointDerivative = [initialState.velocity_units_s, ...
        initialState.acceleration_units_s2, goalState.velocity_units_s, goalState.acceleration_units_s2];
    limitsMatrix       = [limits.maxVelocity_units_s; limits.maxAcceleration_units_s2; limits.maxJerk_units_s3];
    requestIsSupported = max(abs(endpointDerivative)) <= options.ConstraintTolerance && any(string(options.GoalTimeMode) == ["fixedArrival", "earliestArrival"]) && options.SampleTime_s > 0 && isequal(size(limitsMatrix), [3 2]) && all(isfinite(limitsMatrix), "all") && all(limitsMatrix > 0, "all") && (~isfield(goalState, "targetTime_s") || isempty(goalState.targetTime_s));
    if ~requestIsSupported
        error("bmtpEngine:UnsupportedRequest", "The BMTP kernel requires a finite unwrapped rest-to-rest request.");
    end
end
