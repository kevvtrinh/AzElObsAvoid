function [result, diagnosis] = planner(obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   result = planner()
%   result = planner( ...
%       obstacles, initialState, goalState, limits, options)
%
% PURPOSE
%   - Prepare protected polygon histories, test the kinematic clock bound,
%     and construct independently certified BMTP motion. Search the exact
%     spatial visibility graph when a bound solution is unavailable.
%
% INPUTS
%   - obstacles: static polygon structs or canonical polygon histories.
%   - initialState, goalState: position_units and time_s; omitted endpoint
%     velocity and acceleration default to zero.
%     A goal may supply targetMotion (sampled time_s and N-by-2
%     position_units, with linear or pchip InterpolationMethod) instead.
%   - limits: workspace intervals and per-axis velocity, acceleration, jerk.
%   - options: arrival policy, BMTP sampling, and validation tolerances.
%
% OUTPUTS
%   - result: stable success/failure record containing resolved inputs,
%     prepared geometry, visibility graph, BMTP diagnostics, and validation.
%   - diagnosis: optional empty compatibility output; evidence is in result.
%
% UNITS
%   - Position is coordinate units; time is seconds; derivatives use units/s,
%     units/s^2, and units/s^3.

%% Section 1: Resolve The Independent Defaults

useIndependentDefaults = nargin == 0;
diagnosis = struct();
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

%% Section 2: Prepare Authoritative Geometry And Motion Coverage

totalTimer = tic;
earliestTarget = ~isempty(goalState.targetMotion) && options.GoalTimeMode=="earliestArrival";
interceptTime_s = goalState.time_s;
if earliestTarget
    interceptTime_s = obstacleAvoidance.input.findEarliestTargetTime(goalState.targetMotion,initialState,goalState.time_s,limits);
    if isfinite(interceptTime_s)
        goalState.position_units = obstacleAvoidance.input.targetPositionAtTime(goalState.targetMotion,interceptTime_s);
    end
end
preparedObstacles = obstacleAvoidance.obstacles.prepareObstacles(obstacles);
scene = obstacleAvoidance.obstacles.snapshot(preparedObstacles, initialState.time_s);
visibilityGraph = struct('NodePosition_units',zeros(0,2),'AcceptedNodeIndex',zeros(0,2), ...
    'AcceptedWeight_units',zeros(0,1),'RejectedNodeIndex',zeros(0,2), ...
    'RouteNodeIndex',zeros(1,0),'Route_units',zeros(0,2),'RouteLength_units',Inf, ...
    'SourceFree',false,'GoalFree',false,'IsConnected',false,'ExpandedCount',0, ...
    'GraphIsFullyEnumerated',false,'SearchKind',"notSearched");
result = createEmptyResult(obstacles, preparedObstacles, initialState, goalState, limits, options, visibilityGraph);
if earliestTarget && isnan(interceptTime_s)
    result.Message = "The target never enters the rest-to-rest reachable set within the supplied horizon.";
    result.TerminationReason = "targetUnreachable";
    result.ElapsedTime_s = toc(totalTimer);
    result.Validation = obstacleAvoidance.validateTrajectory(result);
    return;
end
isDynamic = ~isempty(preparedObstacles) && any(arrayfun(@(o) ~o.InternalPreparation.IsTimeInvariant,preparedObstacles));
regions_units = cell(0,1);
for k = 1:numel(scene), regions_units = [regions_units; scene(k).Regions_units]; end
if isDynamic
    cells = obstacleAvoidance.obstacles.createTimeCells(preparedObstacles,initialState.time_s,goalState.time_s);
    regions_units = cells.Regions_units;
end
coverage = struct("Passed", true, ...
    "ExactRegionCount", numel(regions_units), ...
    "SolverRegionCount", numel(regions_units), ...
    "AuthoritativeCoverageCheck", "independentPlaneVerification");
if isDynamic
    coverage.ActiveTimeInterval_s = cells.ActiveTimeInterval_s;
    coverage.EndRegions_units = cells.EndRegions_units;
    if options.GoalTimeMode == "fixedArrival", coverage.BreakTime_s = cells.BreakTime_s; end
end
motionGoalState = goalState;
motionGoalState.time_s = interceptTime_s;

%% Section 3: Test The Kinematic Bound Before Spatial Route Search

route_units = [initialState.position_units;goalState.position_units];
seed = struct('position_units',route_units,'tau',[0;1],'Index',1, ...
    'Source',"kinematicBound",'ObstacleEnvelope_units',zeros(0,2));
candidate = struct('Success',false);
boundAttempted = options.GoalTimeMode=="earliestArrival";
if boundAttempted
    [candidate,solverDiagnostics,clockGuide] = bmtpEngine.solve(seed,regions_units,coverage, ...
        initialState,motionGoalState,limits,options,"kinematicBound");
    result.SolverDiagnostics = solverDiagnostics;
    if candidate.Success
        result.VisibilityGraph.SearchKind = "analyticMotion";
        if isfield(clockGuide,'Route_units')
            clockGuide.SearchKind = "kinematicClockProjection";
            result.VisibilityGraph = clockGuide;
            route_units = clockGuide.Route_units;
        end
    end
end
if ~candidate.Success
    attemptedDiagnostics = {};
    if boundAttempted, attemptedDiagnostics{end+1} = solverDiagnostics; end
    delayedCandidate = struct('Success',false);
    if isDynamic && boundAttempted && ~earliestTarget
        delayedSeed = seed; delayedSeed.Source = "departureSchedule";
        [delayedCandidate,delayedDiagnostics] = bmtpEngine.solve(delayedSeed,regions_units,coverage, ...
            initialState,motionGoalState,limits,options,"delayedChord");
        attemptedDiagnostics{end+1} = delayedDiagnostics;
    end
    visibilityGraph = obstacleAvoidance.search.createVisibilityGraph(scene,initialState.position_units,goalState.position_units,limits,options);
    visibilityGraph.SearchKind = "initialSpatialSnapshot";
    result.VisibilityGraph = visibilityGraph;
    % Compare the scheduled chord with the speed bound for traversing the
    % spatial guide. A valid schedule also resolves disconnected snapshots.
    useDelayed = delayedCandidate.Success && (~visibilityGraph.IsConnected || ...
        delayedCandidate.TrajectoryDuration_s<=visibilityGraph.RouteLength_units/norm(limits.maxVelocity_units_s));
    if ~useDelayed && (~visibilityGraph.SourceFree || ~visibilityGraph.GoalFree)
        result.Message = "An endpoint lies inside or on protected obstacle geometry.";
        result.TerminationReason = "invalidEndpoint";
        result.ElapsedTime_s = toc(totalTimer);
        result.Validation = obstacleAvoidance.validateTrajectory(result);
        return;
    end
    if ~useDelayed && ~visibilityGraph.IsConnected
        result.Message = "The initial visibility graph contains no start-to-goal route.";
        result.TerminationReason = "noVisibilityRoute";
        result.ElapsedTime_s = toc(totalTimer);
        result.Validation = obstacleAvoidance.validateTrajectory(result);
        return;
    end
    if ~useDelayed
        route_units = visibilityGraph.Route_units;
        edgeLength_units = vecnorm(diff(route_units,1,1),2,2);
        seed.position_units = route_units;
        seed.tau = [0;cumsum(edgeLength_units)]/sum(edgeLength_units);
        seed.Source = "visibilityGraph";
        stage = "complete";
        if boundAttempted, stage = "route"; end
        [candidate,solverDiagnostics] = bmtpEngine.solve(seed,regions_units,coverage, ...
            initialState,motionGoalState,limits,options,stage);
        attemptedDiagnostics{end+1} = solverDiagnostics;
        useDelayed = delayedCandidate.Success && (~candidate.Success || ...
            delayedCandidate.TrajectoryDuration_s<=candidate.TrajectoryDuration_s);
    end
    if useDelayed
        candidate = delayedCandidate;
        solverDiagnostics = delayedDiagnostics;
        route_units = [initialState.position_units;goalState.position_units];
    end
    if boundAttempted
        solverDiagnostics.LowerBoundAttempt = result.SolverDiagnostics.LowerBoundAttempt;
        solverDiagnostics.TrajectorySocpCount = 0;
        solverDiagnostics.ConicSolver.CallCount = 0;
        solverDiagnostics.ConicSolver.TotalTime_s = 0;
        solverDiagnostics.ElapsedTime_s = 0;
        for k = 1:numel(attemptedDiagnostics)
            previous = attemptedDiagnostics{k};
            solverDiagnostics.TrajectorySocpCount = solverDiagnostics.TrajectorySocpCount+previous.TrajectorySocpCount;
            solverDiagnostics.ConicSolver.CallCount = solverDiagnostics.ConicSolver.CallCount+previous.ConicSolver.CallCount;
            solverDiagnostics.ConicSolver.TotalTime_s = solverDiagnostics.ConicSolver.TotalTime_s+previous.ConicSolver.TotalTime_s;
            solverDiagnostics.ElapsedTime_s = solverDiagnostics.ElapsedTime_s+previous.ElapsedTime_s;
        end
    end
end

%% Section 4: Independently Validate The Complete Returned Motion

if earliestTarget && ~candidate.Success
    candidate.Message = "Motion at the kinematic interception bound was not certified; later interception has not been searched.";
    candidate.TerminationReason = "earliestInterceptUncertified";
end
candidateFields = string(fieldnames(candidate));
for fieldName = reshape(candidateFields, 1, [])
    result.(fieldName) = candidate.(fieldName);
end
result.Route_units = route_units;
result.SolverDiagnostics = solverDiagnostics;
if ~isempty(goalState.targetMotion)
    result.Intercept = struct('Time_s',candidate.ArrivalTime_s, ...
        'TargetPosition_units',goalState.position_units,'TerminalVelocityPolicy',"zero");
end
result.Validation = obstacleAvoidance.validateTrajectory(result);
if candidate.Success && ~result.Validation.Passed
    result.Success = false;
    result.Message = "BMTP returned motion that failed independent validation: " + result.Validation.Message;
    result.TerminationReason = "invalidMotion";
end
result.ElapsedTime_s = toc(totalTimer);
end

%% Section 5: Local Functions

function [obstacles, initialState, goalState, limits, options] = createDefaults()
    % Provide one independently runnable static detour request.
    obstacles = struct("Name", "center block", ...
        "Vertices_units", [-1 -1; 1 -1; 1 1; -1 1], ...
        "SafetyMargin_units", 0.25);
    initialState = struct("time_s", 0, "position_units", [-4 0], ...
        "velocity_units_s", [0 0], "acceleration_units_s2", [0 0]);
    goalState = struct("time_s", 12, "position_units", [4 0], ...
        "velocity_units_s", [0 0], "acceleration_units_s2", [0 0], 'targetMotion', []);
    limits = struct("xInterval_units", [-180 180], "yInterval_units", [-90 90], ...
        "maxVelocity_units_s", [2 2], ...
        "maxAcceleration_units_s2", [2 2], "maxJerk_units_s3", [4 4]);
    options = struct("GoalTimeMode", "fixedArrival", ...
        "SampleTime_s", 0.05, "ConstraintTolerance", 1e-8, ...
        "CollisionClearanceTolerance_units", 1e-7, ...
        "ArrivalTimeTolerance_s", 1e-8, "WrapX", false, "WrapY", false);
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
    if isfield(state,'targetMotion') && ~isempty(state.targetMotion)
        state.position_units = obstacleAvoidance.input.targetPositionAtTime(state.targetMotion,state.time_s);
    end
    for fieldName = ["position_units", "velocity_units_s", "acceleration_units_s2"]
        value = double(state.(fieldName));
        if ~isnumeric(state.(fieldName)) || ~isreal(value) || ~isvector(value) || numel(value) ~= 2 || any(~isfinite(value))
            error("planTrajectory:InvalidState", "%s.%s must be a finite 1-by-2 row.", argumentName, fieldName);
        end
        state.(fieldName) = reshape(value,1,2);
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
        if ~isnumeric(limits.(fieldName)) || ~isreal(interval) || ~isvector(interval) || numel(interval) ~= 2 || any(~isfinite(interval)) || interval(2) <= interval(1)
            error("planTrajectory:InvalidWorkspace", "%s must be a finite increasing 1-by-2 row.", fieldName);
        end
        limits.(fieldName) = reshape(interval,1,2);
    end
    for fieldName = ["maxVelocity_units_s", "maxAcceleration_units_s2", "maxJerk_units_s3"]
        value = double(limits.(fieldName));
        if isscalar(value), value = [value value]; end %#ok<AGROW>
        if ~isnumeric(limits.(fieldName)) || ~isreal(value) || ~isvector(value) || numel(value) ~= 2 || any(~isfinite(value)) || any(value <= 0)
            error("planTrajectory:InvalidDerivativeLimit", "%s must be a positive scalar or finite 1-by-2 row.", fieldName);
        end
        limits.(fieldName) = reshape(value,1,2);
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
    if ~isscalar(options.GoalTimeMode) || ~any(options.GoalTimeMode == ["fixedArrival", "earliestArrival"])
        error("planner:UnsupportedGoalTimeMode", "GoalTimeMode must be fixedArrival or earliestArrival.");
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
    result.Inputs = struct("obstacles", {obstacles}, "initialState", initialState, ...
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
    result.Intercept = struct('Time_s',NaN,'TargetPosition_units',goalState.position_units, ...
        'TerminalVelocityPolicy',"zero");
    result.TrajectoryDuration_s = NaN;
    result.ElapsedTime_s = 0;
end
