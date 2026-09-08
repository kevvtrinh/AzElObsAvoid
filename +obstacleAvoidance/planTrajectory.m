function result = planTrajectory(obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   result = obstacleAvoidance.planTrajectory()
%   result = obstacleAvoidance.planTrajectory( ...
%       obstacles, initialState, goalState, limits, options)
%
% PURPOSE
%   - Prepare static convex obstacles, build one exhaustive visibility graph,
%     and turn its shortest route into one BMTP trajectory.
%
% INPUTS
%   - obstacles: static polygon structs accepted by prepareObstacles.
%   - initialState, goalState: position_units and time_s; omitted endpoint
%     velocity and acceleration default to zero.
%   - limits: workspace intervals and per-axis velocity, acceleration, jerk.
%   - options: fixed-arrival BMTP sampling and validation tolerances.
%
% OUTPUTS
%   - result: stable success/failure record containing resolved inputs,
%     prepared geometry, visibility graph, BMTP diagnostics, and validation.
%
% UNITS
%   - Position is coordinate units; time is seconds; derivatives use units/s,
%     units/s^2, and units/s^3.

%% Section 1: Resolve The Independent Defaults

useIndependentDefaults = nargin == 0;
[defaultObstacles, defaultInitialState, defaultGoalState, defaultLimits, defaultOptions] = createDefaults();
if useIndependentDefaults
    obstacles = defaultObstacles;
elseif nargin < 1
    obstacles = [];
end
if nargin < 2 || isempty(initialState), initialState = defaultInitialState; end
if nargin < 3 || isempty(goalState), goalState = defaultGoalState; end
if nargin < 4 || isempty(limits), limits = defaultLimits; end
if nargin < 5 || isempty(options), options = struct(); end
initialState = normalizeState(initialState, defaultInitialState, "initialState");
goalState    = normalizeState(goalState, defaultGoalState, "goalState");
limits       = normalizeLimits(limits, defaultLimits);
options      = resolveOptions(options, defaultOptions);
if goalState.time_s <= initialState.time_s
    error("planTrajectory:InvalidTimeOrder", "goalState.time_s must be greater than initialState.time_s.");
end
if norm(goalState.position_units - initialState.position_units) <= options.ConstraintTolerance
    error("planTrajectory:CoincidentEndpoints", "Initial and goal positions must be distinct.");
end

%% Section 2: Prepare Obstacles And Visibility Route

totalTimer = tic;
preparedObstacles = obstacleAvoidance.obstacles.prepareObstacles(obstacles);
visibilityGraph = obstacleAvoidance.search.createVisibilityGraph(preparedObstacles, initialState.position_units, goalState.position_units, limits, options);
result = createEmptyResult(obstacles, preparedObstacles, initialState, goalState, limits, options, visibilityGraph);
if ~visibilityGraph.SourceFree || ~visibilityGraph.GoalFree
    result.Message = "An endpoint lies inside or on protected obstacle geometry.";
    result.TerminationReason = "invalidEndpoint";
    result.ElapsedTime_s = toc(totalTimer);
    result.Validation = obstacleAvoidance.validateTrajectory(result);
    return;
end
if ~visibilityGraph.IsConnected
    result.Message = "The static visibility graph contains no start-to-goal route.";
    result.TerminationReason = "noVisibilityRoute";
    result.ElapsedTime_s = toc(totalTimer);
    result.Validation = obstacleAvoidance.validateTrajectory(result);
    return;
end

%% Section 3: Solve The Visibility Route With BMTP

route_units = visibilityGraph.Route_units;
edgeLength_units = vecnorm(diff(route_units, 1, 1), 2, 2);
routeLength_units = sum(edgeLength_units);
seed = struct();
seed.position_units = route_units;
seed.tau = [0; cumsum(edgeLength_units)] / routeLength_units;
seed.Index = 1;
seed.Source = "visibilityGraph";
seed.ObstacleEnvelope_units = zeros(0, 2);
regions_units = reshape({preparedObstacles.ProtectedVertices_units}, [], 1);
coverage = struct("Passed", true, ...
    "ExactRegionCount", numel(regions_units), ...
    "SolverRegionCount", numel(regions_units), ...
    "AuthoritativeCoverageCheck", "independentPlaneVerification");
[candidate, solverDiagnostics] = bmtpEngine.solve(seed, regions_units, coverage, initialState, goalState, limits, options);
candidateFields = string(fieldnames(candidate));
for fieldName = reshape(candidateFields, 1, [])
    result.(fieldName) = candidate.(fieldName);
end
result.Route_units = route_units;
result.SolverDiagnostics = solverDiagnostics;
result.Validation = obstacleAvoidance.validateTrajectory(result);
if candidate.Success && ~result.Validation.Passed
    result.Success = false;
    result.Message = "BMTP returned motion that failed independent validation: " + result.Validation.Message;
    result.TerminationReason = "invalidMotion";
end
result.ElapsedTime_s = toc(totalTimer);
end

%% Section 4: Local Functions

function [obstacles, initialState, goalState, limits, options] = createDefaults()
    % Provide one independently runnable static detour request.
    obstacles = struct("Name", "center block", ...
        "Vertices_units", [-1 -1; 1 -1; 1 1; -1 1], ...
        "SafetyMargin_units", 0.25);
    initialState = struct("time_s", 0, "position_units", [-4 0], ...
        "velocity_units_s", [0 0], "acceleration_units_s2", [0 0]);
    goalState = struct("time_s", 12, "position_units", [4 0], ...
        "velocity_units_s", [0 0], "acceleration_units_s2", [0 0]);
    limits = struct("xInterval_units", [-6 6], "yInterval_units", [-4 4], ...
        "maxVelocity_units_s", [2 2], ...
        "maxAcceleration_units_s2", [2 2], "maxJerk_units_s3", [4 4]);
    options = struct("GoalTimeMode", "fixedArrival", ...
        "SampleTime_s", 0.05, "ConstraintTolerance", 1e-8, ...
        "CollisionClearanceTolerance_units", 1e-7, ...
        "ArrivalTimeTolerance_s", 1e-8);
end

function state = normalizeState(state, defaults, argumentName)
    % Resolve omitted rest-to-rest fields and reject unsupported state data.
    if ~isstruct(state) || ~isscalar(state)
        error("planTrajectory:InvalidState", "%s must be a scalar struct.", argumentName);
    end
    allowedFields = string(fieldnames(defaults));
    unknownFields = setdiff(string(fieldnames(state)), allowedFields);
    if ~isempty(unknownFields)
        error("planTrajectory:UnsupportedStateField", "%s contains unsupported fields: %s.", argumentName, strjoin(unknownFields, ", "));
    end
    for fieldName = reshape(allowedFields, 1, [])
        if ~isfield(state, fieldName) || isempty(state.(fieldName))
            state.(fieldName) = defaults.(fieldName);
        end
    end
    validateattributes(state.time_s, {'numeric'}, {'real', 'finite', 'scalar'});
    for fieldName = ["position_units", "velocity_units_s", "acceleration_units_s2"]
        value = double(state.(fieldName));
        if ~isnumeric(state.(fieldName)) || ~isequal(size(value), [1 2]) || any(~isfinite(value))
            error("planTrajectory:InvalidState", "%s.%s must be a finite 1-by-2 row.", argumentName, fieldName);
        end
        state.(fieldName) = value;
    end
    state.time_s = double(state.time_s);
end

function limits = normalizeLimits(limits, defaults)
    % Resolve the small fixed set of workspace and derivative limits.
    if ~isstruct(limits) || ~isscalar(limits)
        error("planTrajectory:InvalidLimits", "limits must be a scalar struct.");
    end
    names = string(fieldnames(defaults));
    unknownFields = setdiff(string(fieldnames(limits)), names);
    if ~isempty(unknownFields)
        error("planTrajectory:UnsupportedLimitField", "Unsupported limit fields: %s.", strjoin(unknownFields, ", "));
    end
    for fieldName = reshape(names, 1, [])
        if ~isfield(limits, fieldName) || isempty(limits.(fieldName))
            limits.(fieldName) = defaults.(fieldName);
        end
    end
    for fieldName = ["xInterval_units", "yInterval_units"]
        interval = double(limits.(fieldName));
        if ~isnumeric(limits.(fieldName)) || ~isequal(size(interval), [1 2]) || any(~isfinite(interval)) || interval(2) <= interval(1)
            error("planTrajectory:InvalidWorkspace", "%s must be a finite increasing 1-by-2 row.", fieldName);
        end
        limits.(fieldName) = interval;
    end
    for fieldName = ["maxVelocity_units_s", "maxAcceleration_units_s2", "maxJerk_units_s3"]
        value = double(limits.(fieldName));
        if isscalar(value), value = [value value]; end %#ok<AGROW>
        if ~isnumeric(limits.(fieldName)) || ~isequal(size(value), [1 2]) || any(~isfinite(value)) || any(value <= 0)
            error("planTrajectory:InvalidDerivativeLimit", "%s must be a positive scalar or finite 1-by-2 row.", fieldName);
        end
        limits.(fieldName) = value;
    end
end

function options = resolveOptions(options, defaults)
    % Resolve all BMTP controls in one place and warn once about unknown fields.
    if ~isstruct(options) || ~isscalar(options)
        error("planTrajectory:InvalidOptions", "options must be a scalar struct.");
    end
    knownFields = string(fieldnames(defaults));
    unknownFields = setdiff(string(fieldnames(options)), knownFields);
    if ~isempty(unknownFields)
        warning("planTrajectory:UnknownOptions", "Ignoring unknown option fields: %s.", strjoin(unknownFields, ", "));
    end
    supplied = options;
    options = defaults;
    for fieldName = reshape(intersect(string(fieldnames(supplied)), knownFields, "stable"), 1, [])
        if ~isempty(supplied.(fieldName))
            options.(fieldName) = supplied.(fieldName);
        end
    end
    options.GoalTimeMode = string(options.GoalTimeMode);
    if ~isscalar(options.GoalTimeMode) || options.GoalTimeMode ~= "fixedArrival"
        error("planTrajectory:UnsupportedGoalTimeMode", "The empty core supports GoalTimeMode='fixedArrival' only.");
    end
    for fieldName = ["SampleTime_s", "ConstraintTolerance", "CollisionClearanceTolerance_units", "ArrivalTimeTolerance_s"]
        validateattributes(options.(fieldName), {'numeric'}, {'real', 'finite', 'scalar', 'positive'});
        options.(fieldName) = double(options.(fieldName));
    end
end

function result = createEmptyResult(obstacles, preparedObstacles, initialState, goalState, limits, options, visibilityGraph)
    % Keep one result schema for expected search and solver failures.
    result = struct();
    result.Success = false;
    result.Message = "Planning has not completed.";
    result.TerminationReason = "notStarted";
    result.Inputs = struct("obstacles", obstacles, "initialState", initialState, ...
        "goalState", goalState, "limits", limits, "options", options);
    result.PreparedObstacles = preparedObstacles;
    result.Limits = limits;
    result.Options = options;
    result.VisibilityGraph = visibilityGraph;
    result.Route_units = zeros(0, 2);
    result.time_s = zeros(0, 1);
    result.position_units = zeros(0, 2);
    result.velocity_units_s = zeros(0, 2);
    result.acceleration_units_s2 = zeros(0, 2);
    result.jerk_units_s3 = zeros(0, 2);
    result.Polynomial = struct();
    result.PlaneCertificate = struct();
    result.SolverDiagnostics = struct();
    result.Validation = struct("Passed", false, "Message", "No motion is available.");
    result.ArrivalTime_s = NaN;
    result.TrajectoryDuration_s = NaN;
    result.ElapsedTime_s = 0;
end
