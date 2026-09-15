function result = planner(obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   result = planner()
%   result = planner( ...
%       obstacles, initialState, goalState, limits, options)
%
% PURPOSE
%   - Prepare protected polygon histories and an exact visibility guide,
%     then construct independently certified C3 quintic BMTP motion.
%   - Fixed-arrival moving-obstacle requests try the initial exact spatial
%     guide, a distinct arrival-snapshot guide, then one time-expanded guide.
%     Every accepted motion passes independent validation.
%
% INPUTS
%   - obstacles: static polygon structs or canonical polygon histories.
%   - initialState, goalState: position_units and time_s; omitted endpoint
%     velocity and acceleration default to zero.
%     A goal may supply targetMotion (sampled time_s and N-by-2
%     position_units, with linear or pchip InterpolationMethod) instead.
%   - limits: workspace intervals; scalar derivative limits are combined
%     magnitudes allocated equally, and two-element vectors are per-axis.
%   - options: arrival policy, BMTP sampling, validation tolerances, WrapX/Y,
%     MatchTargetVelocity/Acceleration, TemporalResolution_s, MaxArrivalTrials.
%
% OUTPUTS
%   - result: stable success/failure record containing resolved inputs,
%     prepared geometry, visibility graph, BMTP diagnostics, and validation.
%
% UNITS
%   - Position is coordinate units; time is seconds; derivatives use units/s,
%     units/s^2, and units/s^3.

%% Section 1: Resolve The Independent Defaults

% Recursive user path setup can put archived benchmark packages ahead of this
% checkout's engine. Keep planning and validation bound to the same checkout.
plannerFolder = fileparts(mfilename('fullpath'));
engineFolder = fullfile(plannerFolder,'trajectory');
productionPath = [plannerFolder pathsep engineFolder];
if ~startsWith(path,[productionPath pathsep])
    addpath(productionPath,'-begin');
end

[defaultObstacles, defaultInitialState, defaultGoalState, defaultLimits, defaultOptions] = createDefaults();
if nargin == 0
    obstacles = defaultObstacles;
end
if nargin < 2 || isempty(initialState), initialState = defaultInitialState; end
if nargin < 3 || isempty(goalState), goalState = defaultGoalState; end
if nargin < 4 || isempty(limits), limits = defaultLimits; end
if nargin < 5 || isempty(options), options = struct(); end
suppliedLimits = limits;
suppliedGoalState = goalState;
initialState = normalizeState(initialState, defaultInitialState, "initialState");
goalState    = normalizeState(goalState, defaultGoalState, "goalState");
limits       = normalizeLimits(limits, defaultLimits);
options      = resolveOptions(options, defaultOptions);
requestedLimits = limits;
requestedGoalState = goalState;
if options.MatchTargetVelocity || options.MatchTargetAcceleration
    if isempty(goalState.targetMotion)
        error('planner:MissingTarget','Derivative matching requires goalState.targetMotion.');
    end
    derivativeNames = ["velocity_units_s","acceleration_units_s2"];
    matches = [options.MatchTargetVelocity,options.MatchTargetAcceleration];
    [~,targetVelocity,targetAcceleration] = obstacleAvoidance.input.targetPositionAtTime(goalState.targetMotion,goalState.time_s);
    derivatives = [targetVelocity;targetAcceleration];
    for k = find(matches)
        name = derivativeNames(k);
        if isfield(suppliedGoalState,name) && ~isempty(suppliedGoalState.(name)) && ...
                any(abs(suppliedGoalState.(name)-derivatives(k,:))>options.ConstraintTolerance)
            error('planner:ConflictingTargetDerivative','Explicit and matched target derivatives conflict.');
        end
        goalState.(name) = derivatives(k,:);
    end
end
if options.WrapX || options.WrapY
    if ~isempty(obstacles) || ~isempty(goalState.targetMotion)
        error('planner:UnsupportedPeriodicRequest','Wrapping supports obstacle-free fixed-position goals only.');
    end
    names = ["xInterval_units","yInterval_units"];
    for axis = find([options.WrapX options.WrapY])
        period = diff(limits.(names(axis)));
        goalState.position_units(axis) = goalState.position_units(axis)+period* ...
            floor((initialState.position_units(axis)-goalState.position_units(axis))/period+0.5);
        reach = limits.maxVelocity_units_s(axis)*(goalState.time_s-initialState.time_s);
        limits.(names(axis)) = initialState.position_units(axis)+[-reach reach];
    end
end
if goalState.time_s <= initialState.time_s
    error("planTrajectory:InvalidTimeOrder", "goalState.time_s must be greater than initialState.time_s.");
end
if norm(goalState.position_units - initialState.position_units) <= options.ConstraintTolerance
    error("planTrajectory:CoincidentEndpoints", "Initial and goal positions must be distinct.");
end

%% Section 2: Prepare Authoritative Geometry And Motion Coverage

totalTimer = tic;
earliestTarget = ~isempty(goalState.targetMotion) && options.GoalTimeMode=="earliestArrival";
preparedObstacles = obstacleAvoidance.obstacles.prepareObstacles(obstacles,[initialState.time_s,goalState.time_s],true);
isDynamic = ~isempty(preparedObstacles) && any(arrayfun(@(obstacle) ...
    ~obstacle.InternalPreparation.IsTimeInvariant || ...
    (numel(obstacle.time_s)>1 && (initialState.time_s<obstacle.time_s(1) || ...
    goalState.time_s>obstacle.time_s(end))),preparedObstacles));
visibilityGraph = struct('NodePosition_units',zeros(0,2),'AcceptedNodeIndex',zeros(0,2), ...
    'RejectedNodeIndex',zeros(0,2), ...
    'Route_units',zeros(0,2),'RouteLength_units',Inf, ...
    'IsConnected',false,'ExpandedCount',0, ...
    'GraphIsFullyEnumerated',false,'SearchKind',"notSearched");
result = createEmptyResult(obstacles, preparedObstacles, initialState, goalState, limits, options, visibilityGraph);
result.SuppliedLimits = suppliedLimits;
result.RequestedLimits = requestedLimits;
result.RequestedGoalState = requestedGoalState;
result.SuppliedGoalState = suppliedGoalState;
requestedInterval_s=[initialState.time_s,goalState.time_s];
% Swept corresponding cells have a declared conservative continuous model.
% Only intervals without correspondence or any certificate stop preparation.
unsupportedObstacleIndex = find(arrayfun(@(obstacle) any( ...
    obstacle.InternalPreparation.IntervalPrepared & ...
    obstacle.InternalPreparation.IntervalGeometryModel=="unsupportedContinuousDeformation" & ...
    obstacle.time_s(1:end-1)<requestedInterval_s(2) & ...
    obstacle.time_s(2:end)>requestedInterval_s(1)),preparedObstacles),1);
if ~isempty(unsupportedObstacleIndex)
    preparation = preparedObstacles(unsupportedObstacleIndex).InternalPreparation;
    obstacleTime_s=preparedObstacles(unsupportedObstacleIndex).time_s;
    unsupportedIntervalIndex = find(preparation.IntervalPrepared & ...
        preparation.IntervalGeometryModel=="unsupportedContinuousDeformation" & ...
        obstacleTime_s(1:end-1)<requestedInterval_s(2) & ...
        obstacleTime_s(2:end)>requestedInterval_s(1),1);
    intervalTime_s = preparedObstacles(unsupportedObstacleIndex).time_s( ...
        unsupportedIntervalIndex:unsupportedIntervalIndex+1);
    result.Message = sprintf(['Obstacle %d ("%s"), interval [%g, %g] s, has no ' ...
        'certified exact continuous interpolation.'],unsupportedObstacleIndex, ...
        string(preparedObstacles(unsupportedObstacleIndex).targetName),intervalTime_s(1),intervalTime_s(2));
    if isfield(preparation,'IntervalCertificationReason') && ...
            preparation.IntervalCertificationReason(unsupportedIntervalIndex)=="sweptEnvelopeExcludesProtectedSample"
        result.Message = result.Message+" The prescribed swept margin-square enclosure excludes authoritative protected sample area.";
    end
    result.TerminationReason = "unsupportedObstacleInterpolation";
    result.ElapsedTime_s = toc(totalTimer);
    return;
end
scene = obstacleAvoidance.obstacles.snapshot(preparedObstacles, initialState.time_s, ~isDynamic);
[endpointFeasible,result.Message,result.TerminationReason] = obstacleAvoidance.input.validatePlannerEndpoints( ...
    preparedObstacles,initialState,goalState,limits,options);
if ~endpointFeasible
    result.ElapsedTime_s = toc(totalTimer);
    return;
end
if isDynamic
    cells = obstacleAvoidance.obstacles.createTimeCells(preparedObstacles,initialState.time_s,goalState.time_s);
    regions_units = cells.Regions_units;
else
    regionCount = sum(arrayfun(@(obstacle) numel(obstacle.Regions_units),scene));
    regions_units = cell(regionCount,1);
    nextRegionIndex = 1;
    for obstacleIndex = 1:numel(scene)
        obstacleRegionCount = numel(scene(obstacleIndex).Regions_units);
        targetIndices = nextRegionIndex:nextRegionIndex+obstacleRegionCount-1;
        regions_units(targetIndices) = scene(obstacleIndex).Regions_units;
        nextRegionIndex = nextRegionIndex+obstacleRegionCount;
    end
end
coverage = struct("Passed", true, ...
    "ExactRegionCount", numel(regions_units));
if isDynamic
    coverage.ActiveTimeInterval_s = cells.ActiveTimeInterval_s;
    coverage.EndRegions_units = cells.EndRegions_units;
    if options.GoalTimeMode == "fixedArrival", coverage.BreakTime_s = cells.BreakTime_s; end
end
isRest = all([initialState.velocity_units_s,initialState.acceleration_units_s2, ...
    goalState.velocity_units_s,goalState.acceleration_units_s2]==0);
if options.GoalTimeMode=="earliestArrival" && ...
        (isDynamic || earliestTarget || ~isRest)
    % A certified zero-delay C3 chord attains the physical travel lower
    % bound and is globally earliest. A delayed chord is only an incumbent;
    % the single timed BMTP profile may still find an earlier homotopy.
    departureCandidate=struct('Success',false);
    departureDiagnostics=struct();
    departureRoute_units=[initialState.position_units;goalState.position_units];
    if isDynamic && ~earliestTarget && isRest
        departureSeed=struct('position_units',departureRoute_units, ...
            'tau',[0;1],'Source',"departureSchedule");
        [departureCandidate,departureDiagnostics]=bmtpEngine.solve( ...
            departureSeed,regions_units,coverage,initialState,goalState, ...
            limits,options);
        if departureCandidate.Success
            departureDelay_s=0;
            if isfield(departureDiagnostics,'DepartureSchedule')
                departureDelay_s= ...
                    departureDiagnostics.DepartureSchedule.DepartureDelay_s;
            end
            if departureDelay_s<=options.ArrivalTimeTolerance_s
                result=obstacleAvoidance.input.finalizeCandidate( ...
                    result,departureCandidate,departureRoute_units, ...
                    departureDiagnostics);
                result.VisibilityGraph.SearchKind="c3DepartureSchedule";
                result.ElapsedTime_s=toc(totalTimer);
                if result.Success,return;end
            end
        end
    end
    result.ElapsedTime_s=toc(totalTimer);
    [timedResult,timedAccepted]=obstacleAvoidance.input.tryTimedArrival(result);
    if timedAccepted
        if departureCandidate.Success && ...
                departureCandidate.ArrivalTime_s<=timedResult.ArrivalTime_s+ ...
                options.ArrivalTimeTolerance_s
            result=obstacleAvoidance.input.finalizeCandidate( ...
                result,departureCandidate,departureRoute_units, ...
                departureDiagnostics);
            result.VisibilityGraph.SearchKind="c3DepartureSchedule";
            result.ElapsedTime_s=toc(totalTimer);
        else
            result=timedResult;
        end
        return
    end
    if departureCandidate.Success
        result=obstacleAvoidance.input.finalizeCandidate( ...
            result,departureCandidate,departureRoute_units, ...
            departureDiagnostics);
        result.VisibilityGraph.SearchKind="c3DepartureSchedule";
        result.ElapsedTime_s=toc(totalTimer);
        if result.Success,return;end
    end
    result=timedResult;
    result.ElapsedTime_s=toc(totalTimer);
    result = obstacleAvoidance.input.searchArrivalTimes(result);
    return;
end
motionGoalState = goalState;

%% Section 3: Construct A Spatial Guide And Solve C3 Quintic Motion

route_units = [initialState.position_units;goalState.position_units];
fixedPositionDynamic = isDynamic && options.GoalTimeMode=="fixedArrival" && ...
    isempty(goalState.targetMotion);
initialSpatialAttempted = false;
initialSpatialRoute_units = zeros(0,2);
initialSpatialCandidate = struct();
initialSpatialDiagnostics = struct();
if fixedPositionDynamic
    initialVisibilityGraph = getVisibilityGraph(scene,initialState.position_units, ...
        goalState.position_units,limits,options,"initialSpatialSnapshot");
    if initialVisibilityGraph.IsConnected
        initialSpatialAttempted = true;
        initialSpatialRoute_units = initialVisibilityGraph.Route_units;
        initialEdgeLength_units = vecnorm(diff(initialSpatialRoute_units,1,1),2,2);
        initialSeed = struct('position_units',initialSpatialRoute_units, ...
            'tau',[0;cumsum(initialEdgeLength_units)]/sum(initialEdgeLength_units), ...
            'Source',"initialSpatialSnapshot", ...
            'MaximumAlternatingIterations',2);
        [initialSpatialCandidate,initialSpatialDiagnostics] = bmtpEngine.solve( ...
            initialSeed,regions_units,coverage,initialState,motionGoalState, ...
            limits,options);
        if initialSpatialCandidate.Success
            % A solver success that the public validator rejects is a defect
            % to diagnose upstream, not a reason to try another guide.
            result.VisibilityGraph = initialVisibilityGraph;
            result = obstacleAvoidance.input.finalizeCandidate( ...
                result,initialSpatialCandidate,initialSpatialRoute_units, ...
                initialSpatialDiagnostics);
            result.ElapsedTime_s = toc(totalTimer);
            return
        end
    end
end
futureGoalBlocked = isDynamic && ~fixedPositionDynamic && ...
    obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
    preparedObstacles,goalState.position_units(1),goalState.position_units(2),initialState.time_s);
if fixedPositionDynamic
    guideScene = obstacleAvoidance.obstacles.snapshot( ...
        preparedObstacles,motionGoalState.time_s,false);
    visibilityGraph = getVisibilityGraph(guideScene,initialState.position_units, ...
        goalState.position_units,limits,options,"arrivalSpatialSnapshot");
elseif futureGoalBlocked
    visibilityGraph.Route_units = route_units;
    visibilityGraph.RouteLength_units = norm(diff(route_units));
    visibilityGraph.SearchKind = "temporalDirectSeed";
else
    guideScene = scene;
    guideKind = "initialSpatialSnapshot";
    visibilityGraph = getVisibilityGraph(guideScene,initialState.position_units, ...
        goalState.position_units,limits,options,guideKind);
end
if isDynamic && ~visibilityGraph.IsConnected
    % A disconnected spatial guide cannot rule out a later temporal opening.
    % This chord is only a seed; all time-dependent exclusions remain active.
    visibilityGraph.Route_units = route_units;
    visibilityGraph.RouteLength_units = norm(diff(route_units));
    visibilityGraph.SearchKind = "temporalDirectSeed";
    futureGoalBlocked = true;
end
result.VisibilityGraph = visibilityGraph;
if initialSpatialAttempted
    result.VisibilityGraph.InitialSpatialSeedDiagnostics = initialSpatialDiagnostics;
end
if ~futureGoalBlocked && ~visibilityGraph.IsConnected
    result.Message = "The initial visibility graph contains no start-to-goal route.";
    result.TerminationReason = "noVisibilityRoute";
    result.ElapsedTime_s = toc(totalTimer);
    return;
end
route_units = visibilityGraph.Route_units;
edgeLength_units = vecnorm(diff(route_units,1,1),2,2);
seed = struct('position_units',route_units,'tau',[0;cumsum(edgeLength_units)]/sum(edgeLength_units), ...
    'Source',visibilityGraph.SearchKind);
if isDynamic && options.GoalTimeMode=="fixedArrival"
    % Give the exact spatial route its initial BMTP pass and one pass on the
    % resulting refined mesh. If neither pass certifies complete motion,
    % construct the exact timed route instead of repeatedly optimizing the
    % same failed homotopy.
    seed.MaximumAlternatingIterations=2;
end
sameFailedSpatialRoute = fixedPositionDynamic && initialSpatialAttempted && ...
    isequaln(route_units,initialSpatialRoute_units);
if sameFailedSpatialRoute
    candidate = initialSpatialCandidate;
    solverDiagnostics = initialSpatialDiagnostics;
else
    [candidate,solverDiagnostics] = bmtpEngine.solve(seed,regions_units,coverage, ...
        initialState,motionGoalState,limits,options);
end

result=obstacleAvoidance.input.finalizeCandidate( ...
    result,candidate,route_units,solverDiagnostics);
% Only solver-level infeasibility of the spatial guide admits the timed guide.
% A motion the public validator rejects terminates here as a defect.
spatialFailureCanUseTimedGuide = ~candidate.Success && ...
    candidate.OptimizerIterateUnavailable;
if fixedPositionDynamic && spatialFailureCanUseTimedGuide
    result.VisibilityGraph.SpatialSeedDiagnostics=solverDiagnostics;
    result.ElapsedTime_s=toc(totalTimer);
    [result,~]=obstacleAvoidance.input.tryTimedArrival(result);
    return
end
result.ElapsedTime_s = toc(totalTimer);
end

%% Section 4: Local Functions

function graph = getVisibilityGraph(scene,start_units,goal_units,limits,options,kind)
    % Reuse only a graph with exactly identical geometry and public inputs.
    persistent previousInput previousGraph
    input = struct('Scene',scene,'Start',start_units,'Goal',goal_units, ...
        'Limits',limits,'Options',options);
    if isequaln(input,previousInput)
        graph = previousGraph;
    else
        graph = obstacleAvoidance.search.createVisibilityGraph( ...
            scene,start_units,goal_units,limits,options);
        previousInput = input;
        previousGraph = graph;
    end
    graph.SearchKind = kind;
end

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
        "ArrivalTimeTolerance_s", 1e-8, "WrapX", false, "WrapY", false, ...
        "MatchTargetVelocity",false,"MatchTargetAcceleration",false, ...
        "TemporalResolution_s",0.5,"MaxArrivalTrials",100);
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
    physicalNames = ["maxVelocity_units_s", "maxAcceleration_units_s2", "maxJerk_units_s3"];
    sizes = arrayfun(@(name) numel(limits.(name)),physicalNames);
    if any(sizes~=sizes(1))
        error('planTrajectory:MixedLimitModes','Velocity, acceleration, and jerk limits must all be scalars or all be two-element vectors.');
    end
    for fieldName = physicalNames
        value = double(limits.(fieldName));
        if isscalar(value), value = [value value]/sqrt(2); end
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
    [options, unknownFields] = obstacleAvoidance.input.resolveOptions(defaults, options);
    if ~isempty(unknownFields)
        warning("planTrajectory:UnknownOptions", "Ignoring unknown option fields: %s.", strjoin(unknownFields, ", "));
    end
    options.GoalTimeMode = string(options.GoalTimeMode);
    if ~isscalar(options.GoalTimeMode) || ~any(options.GoalTimeMode == ["fixedArrival", "earliestArrival"])
        error("planner:UnsupportedGoalTimeMode", "GoalTimeMode must be fixedArrival or earliestArrival.");
    end
    for fieldName = ["SampleTime_s", "ConstraintTolerance", "CollisionClearanceTolerance_units", "ArrivalTimeTolerance_s","TemporalResolution_s"]
        validateattributes(options.(fieldName), {'numeric'}, {'real', 'finite', 'scalar', 'positive'});
        options.(fieldName) = double(options.(fieldName));
    end
    for name = ["WrapX","WrapY","MatchTargetVelocity","MatchTargetAcceleration"]
        options.(name) = obstacleAvoidance.input.normalizeLogicalScalar(options.(name),name,"planner:InvalidLogicalOption");
    end
    validateattributes(options.MaxArrivalTrials,{'numeric'},{'scalar','finite','integer','positive'});
end

function result = createEmptyResult(obstacles, preparedObstacles, initialState, goalState, limits, options, visibilityGraph)
    % Keep one result schema for expected search and solver failures.
    result = struct();
    result.Success = false;
    result.Message = "Planning has not completed.";
    result.TerminationReason = "notStarted";
    result.Inputs = struct("obstacles", {obstacles}, "initialState", initialState, ...
        "goalState", goalState);
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
