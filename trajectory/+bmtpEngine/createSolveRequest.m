function request = createSolveRequest(seed, regions_units, coverage, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX: request = bmtpEngine.createSolveRequest( seed, regions_units, coverage, initialState,
%   goalState, limits, options)
% PURPOSE: Check BMTP inputs and select the established polynomial representation. Collect horizon,
%   region, objective, and numerical solver controls once.
% INPUTS: seed (scalar route-seed struct) Ordered positions and normalized route progress.
%   regions_units (R-by-1 cell array) Convex exclusion polygons. coverage (scalar struct) Static
%   region-coverage evidence from the caller. initialState, goalState, limits, options (scalar
%   structs) Dimension-neutral boundary request, limits, and resolved controls.
% OUTPUTS: request (scalar struct) Validated inputs, representation choice, horizon, region bounds,
%   objective rate, and numerical solver options.
% UNITS: Position is coordinate units and time is seconds; derivatives use units/s, units/s^2, and
%   units/s^3.

%% Section 1: Check The Engine Inputs
% Validate convex static regions before solving.

for name = ["velocity_units_s","acceleration_units_s2"]
    if ~isfield(initialState,name) || isempty(initialState.(name)), initialState.(name) = [0 0]; end
    if ~isfield(goalState,name) || isempty(goalState.(name)), goalState.(name) = [0 0]; end
    validateattributes(initialState.(name),{'numeric'},{'real','finite','size',[1 2]});
    validateattributes(goalState.(name),{'numeric'},{'real','finite','size',[1 2]});
end
validateKernelInputs(seed, regions_units, coverage, initialState, goalState, limits, options);

%% Section 2: Select The Polynomial Representation
% Start static guide edges with three quintic subspans to limit model size
% while retaining continuous-jerk steering freedom.
% Fixed-arrival clocks use their natural events and a minimum steering mesh.

[degree, splitCount] = deal(5, 3);
if isfield(seed,'Source') && string(seed.Source)=="timeExpandedVisibilityGraph"
    [degree,splitCount] = deal(8,2);
end
motionHorizon_s = goalState.time_s - initialState.time_s;
if motionHorizon_s <= 0
    error("bmtpEngine:InvalidGoalTime", "goalState.time_s must be greater than initialState.time_s.");
end
%% Section 3: Prepare Shared Solver Controls
% Set shared tolerances and iteration limits for both conic solvers.

regionMinimum_units = zeros(numel(regions_units), 2);
regionMaximum_units = zeros(numel(regions_units), 2);
separatingLineGeometry = cell(numel(regions_units),1);
for regionIndex = 1:numel(regions_units)
    regionMinimum_units(regionIndex, :) = min(regions_units{regionIndex}, [], 1);
    regionMaximum_units(regionIndex, :) = max(regions_units{regionIndex}, [], 1);
    endRegion_units = regions_units{regionIndex};
    if isfield(coverage,'EndRegions_units')
        endRegion_units = coverage.EndRegions_units{regionIndex};
    end
    separatingLineGeometry{regionIndex} = createSeparatingLineGeometry( ...
        regions_units{regionIndex},endRegion_units);
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
    "IsRest", all([initialState.velocity_units_s initialState.acceleration_units_s2 goalState.velocity_units_s goalState.acceleration_units_s2]==0), ...
    "Degree", degree, ...
    "SplitCount", splitCount, ...
    "MotionHorizon_s", motionHorizon_s, ...
    "RegionMinimum_units", regionMinimum_units, ...
    "RegionMaximum_units", regionMaximum_units, ...
    "SeparatingLineGeometry", {separatingLineGeometry}, ...
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
    requestIsSupported = all(isfinite(endpointDerivative)) && any(string(options.GoalTimeMode) == ["fixedArrival", "earliestArrival"]) && options.SampleTime_s > 0 && isequal(size(limitsMatrix), [3 2]) && all(isfinite(limitsMatrix), "all") && all(limitsMatrix > 0, "all") && (~isfield(goalState, "targetTime_s") || isempty(goalState.targetTime_s));
    if ~requestIsSupported
        error("bmtpEngine:UnsupportedRequest", "The BMTP kernel requires a finite unwrapped full-state request.");
    end
end

function geometry = createSeparatingLineGeometry(first_units,last_units)
    % Obstacle-edge directions and supports do not change while alternating
    % the trajectory. Cache them once; curve-hull directions remain per trial.
    edges_units = diff([first_units;first_units(1,:)],1,1);
    if ~isequal(first_units,last_units)
        edges_units = [edges_units;diff([last_units;last_units(1,:)],1,1)];
    end
    length_units = vecnorm(edges_units,2,2);
    edges_units = edges_units(length_units>0,:);
    length_units = length_units(length_units>0);
    positiveNormals = [-edges_units(:,2),edges_units(:,1)]./length_units;
    firstProjection_units = first_units*positiveNormals.';
    lastProjection_units = last_units*positiveNormals.';
    geometry = struct('PositiveNormals',positiveNormals, ...
        'FirstPositiveSupport_units',min(firstProjection_units,[],1), ...
        'LastPositiveSupport_units',min(lastProjection_units,[],1), ...
        'FirstNegativeSupport_units',-max(firstProjection_units,[],1), ...
        'LastNegativeSupport_units',-max(lastProjection_units,[],1));
end
