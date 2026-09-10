function [result, diagnosis] = planner(obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   result = planner()
%   result = planner( ...
%       obstacles, initialState, goalState, limits, options)
%
% PURPOSE
%   - Prepare protected polygon histories and an exact visibility guide,
%     then construct independently certified C3 quintic BMTP motion.
%   - Moving-obstacle guides use requested-window envelopes for initialization;
%     motion constraints retain the original time-dependent geometry.
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
%     FixedArrivalSearch: spatial (default) or timeExpanded for timed visibility.
%     PathLengthTimeAllowance_s (default 0.49, range [0,0.5)) permits static
%     monotone corridor refinement to spend arrival time for at least 1% shorter
%     motion. Set zero to shorten only at the earliest feasible corridor clock.
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

% Recursive user path setup can put archived benchmark packages ahead of this
% checkout's engine. Keep planning and validation bound to the same checkout.
plannerFolder = fileparts(mfilename('fullpath'));
engineFolder = fullfile(plannerFolder,'trajectory');
productionPath = [plannerFolder pathsep engineFolder];
if ~startsWith(path,[productionPath pathsep])
    addpath(productionPath,'-begin');
end

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
interceptTime_s = goalState.time_s;
preparedObstacles = obstacleAvoidance.obstacles.prepareObstacles(obstacles,[initialState.time_s,goalState.time_s]);
scene = obstacleAvoidance.obstacles.snapshot(preparedObstacles, initialState.time_s);
visibilityGraph = struct('NodePosition_units',zeros(0,2),'AcceptedNodeIndex',zeros(0,2), ...
    'AcceptedWeight_units',zeros(0,1),'RejectedNodeIndex',zeros(0,2), ...
    'RouteNodeIndex',zeros(1,0),'Route_units',zeros(0,2),'RouteLength_units',Inf, ...
    'SourceFree',false,'GoalFree',false,'IsConnected',false,'ExpandedCount',0, ...
    'GraphIsFullyEnumerated',false,'SearchKind',"notSearched");
result = createEmptyResult(obstacles, preparedObstacles, initialState, goalState, limits, options, visibilityGraph);
result.SuppliedLimits = suppliedLimits;
result.RequestedLimits = requestedLimits;
result.RequestedGoalState = requestedGoalState;
result.SuppliedGoalState = suppliedGoalState;
[endpointFeasible,result.Message,result.TerminationReason] = obstacleAvoidance.input.validatePlannerEndpoints( ...
    preparedObstacles,initialState,goalState,limits,options);
if ~endpointFeasible
    result.ElapsedTime_s = toc(totalTimer);
    return;
end
isDynamic = ~isempty(preparedObstacles) && any(arrayfun(@(o) ~o.InternalPreparation.IsTimeInvariant || ...
    (numel(o.time_s)>1 && (initialState.time_s<o.time_s(1) || goalState.time_s>o.time_s(end))),preparedObstacles));
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
else
    coverage.StaticScene = scene;
end
if options.GoalTimeMode=="fixedArrival" && options.FixedArrivalSearch=="timeExpanded"
    result.ElapsedTime_s = toc(totalTimer);
    [result,~] = obstacleAvoidance.input.tryTimedArrival(result);
    return;
end
if options.GoalTimeMode=="earliestArrival" && (isDynamic || earliestTarget)
    result.ElapsedTime_s = toc(totalTimer);
    [timedResult,timedAccepted] = obstacleAvoidance.input.tryTimedArrival(result);
    if timedAccepted
        result = timedResult;
        return;
    end
    % Preserve the delayed chord as an incumbent when the dense-history
    % fast path is inapplicable or does not certify a motion.
    isRest = all([initialState.velocity_units_s,initialState.acceleration_units_s2, ...
        goalState.velocity_units_s,goalState.acceleration_units_s2]==0);
    if isDynamic && ~earliestTarget && isRest
        route_units = [initialState.position_units;goalState.position_units];
        seed = struct('position_units',route_units,'tau',[0;1],'Source',"departureSchedule");
        [candidate,diagnostics] = bmtpEngine.solve(seed,regions_units,coverage,initialState,goalState,limits,options);
        if candidate.Success
            for name=reshape(string(fieldnames(candidate)),1,[]), result.(name)=candidate.(name); end
            result.Route_units=route_units; result.SolverDiagnostics=diagnostics;
            result.VisibilityGraph.SearchKind="c3DepartureSchedule";
            result.Validation=obstacleAvoidance.validateTrajectory(result);
            result.Success=result.Validation.Passed;
            result.ElapsedTime_s=toc(totalTimer);
            hasWait = isfield(diagnostics,'DepartureSchedule') && ...
                diagnostics.DepartureSchedule.DepartureDelay_s>options.ArrivalTimeTolerance_s;
            if result.Success && ~hasWait, return; end
        end
    end
    result = obstacleAvoidance.input.searchArrivalTimes(result);
    return;
end
motionGoalState = goalState;
motionGoalState.time_s = interceptTime_s;

%% Section 3: Construct A Spatial Guide And Solve C3 Quintic Motion

route_units = [initialState.position_units;goalState.position_units];
futureGoalBlocked = isDynamic && obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
    preparedObstacles,goalState.position_units(1),goalState.position_units(2),initialState.time_s);
if futureGoalBlocked
    visibilityGraph.Route_units = route_units;
    visibilityGraph.RouteLength_units = norm(diff(route_units));
    visibilityGraph.SearchKind = "temporalDirectSeed";
else
    guideScene = scene;
    guideKind = "initialSpatialSnapshot";
    if isDynamic
        % The sweep selects a spatial guide only. BMTP below retains the
        % original moving cells and can tighten the motion inside this hull.
        guideScene = struct('ProtectedShape',{});
        for k = 1:numel(preparedObstacles)
            indices = find(cells.SourceObstacleIndex==k);
            if isempty(indices), continue; end
            if preparedObstacles(k).InternalPreparation.IsTimeInvariant
                sampleIndex=find(preparedObstacles(k).InternalPreparation.SamplePrepared,1);
                guideScene(end+1).ProtectedShape=preparedObstacles(k).InternalPreparation.SampleShapes{sampleIndex}; %#ok<AGROW>
                continue;
            end
            vertices_units = [vertcat(cells.Regions_units{indices});vertcat(cells.EndRegions_units{indices})];
            hull = convhull(vertices_units(:,1),vertices_units(:,2));
            guideScene(end+1).ProtectedShape = polyshape(vertices_units(hull(1:end-1),:), ...
                'Simplify',false,'KeepCollinearPoints',true);
        end
        guideKind = "requestedWindowEnvelopeGuide";
    end
    % Chronological trials often share exactly the same spatial problem.
    % Compare all graph inputs directly so changed source geometry cannot
    % reuse stale visibility edges. Retain only the most recent graph.
    persistent previousVisibilityInput previousVisibilityGraph
    visibilityInput=struct('Scene',guideScene,'Start',initialState.position_units, ...
        'Goal',goalState.position_units,'Limits',limits,'Options',options);
    if isequaln(visibilityInput,previousVisibilityInput)
        visibilityGraph=previousVisibilityGraph;
    else
        visibilityGraph = obstacleAvoidance.search.createVisibilityGraph(guideScene,initialState.position_units,goalState.position_units,limits,options);
        previousVisibilityInput=visibilityInput; previousVisibilityGraph=visibilityGraph;
    end
    visibilityGraph.SearchKind = guideKind;
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
if ~futureGoalBlocked && ~visibilityGraph.IsConnected
    result.Message = "The initial visibility graph contains no start-to-goal route.";
    result.TerminationReason = "noVisibilityRoute";
    result.ElapsedTime_s = toc(totalTimer);
    return;
end
route_units = visibilityGraph.Route_units;
edgeLength_units = vecnorm(diff(route_units,1,1),2,2);
seed = struct('position_units',route_units,'tau',[0;cumsum(edgeLength_units)]/sum(edgeLength_units), ...
    'Index',1,'Source',visibilityGraph.SearchKind,'ObstacleEnvelope_units',zeros(0,2));
[candidate,solverDiagnostics] = bmtpEngine.solve(seed,regions_units,coverage, ...
    initialState,motionGoalState,limits,options);

%% Section 4: Independently Validate The Complete Returned Motion

candidateFields = string(fieldnames(candidate));
for fieldName = reshape(candidateFields, 1, [])
    result.(fieldName) = candidate.(fieldName);
end
result.Route_units = route_units;
result.SolverDiagnostics = solverDiagnostics;
if ~isempty(goalState.targetMotion)
    result.Intercept = struct('Time_s',candidate.ArrivalTime_s, ...
        'TargetPosition_units',goalState.position_units,'TerminalVelocityPolicy',"explicit", ...
        'TerminalAccelerationPolicy',"explicit");
    if all(goalState.velocity_units_s==0), result.Intercept.TerminalVelocityPolicy = "zero"; end
    if all(goalState.acceleration_units_s2==0), result.Intercept.TerminalAccelerationPolicy = "zero"; end
    if options.MatchTargetVelocity, result.Intercept.TerminalVelocityPolicy = "matched"; end
    if options.MatchTargetAcceleration, result.Intercept.TerminalAccelerationPolicy = "matched"; end
end
result.Validation = obstacleAvoidance.validateTrajectory(result);
if candidate.Success && ~result.Validation.Passed
    result.Success = false;
    result.Message = "BMTP returned motion that failed independent validation: " + result.Validation.Message;
    result.TerminationReason = "invalidMotion";
end
result.ElapsedTime_s = toc(totalTimer);
if ~result.Success && options.GoalTimeMode=="earliestArrival" && (isDynamic || earliestTarget)
    result = obstacleAvoidance.input.searchArrivalTimes(result);
end
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
    options = struct("GoalTimeMode", "fixedArrival", "FixedArrivalSearch", "spatial", ...
        "SampleTime_s", 0.05, "ConstraintTolerance", 1e-8, ...
        "CollisionClearanceTolerance_units", 1e-7, ...
        "ArrivalTimeTolerance_s", 1e-8, "WrapX", false, "WrapY", false, ...
        "MatchTargetVelocity",false,"MatchTargetAcceleration",false, ...
        "TemporalResolution_s",0.5,"MaxArrivalTrials",100,"PathLengthTimeAllowance_s",0.49);
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
        if isscalar(value), value = [value value]/sqrt(2); end %#ok<AGROW>
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
    options.FixedArrivalSearch = string(options.FixedArrivalSearch);
    if ~isscalar(options.FixedArrivalSearch) || ~any(options.FixedArrivalSearch==["spatial","timeExpanded"])
        error("planner:UnsupportedFixedArrivalSearch","FixedArrivalSearch must be spatial or timeExpanded.");
    end
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
    allowance_s = options.PathLengthTimeAllowance_s;
    if ~isnumeric(allowance_s) || ~isreal(allowance_s) || ~isscalar(allowance_s) || ...
            ~isfinite(allowance_s) || allowance_s<0 || allowance_s>=0.5
        error('planner:InvalidPathLengthTimeAllowance', ...
            'PathLengthTimeAllowance_s must be a finite scalar in [0,0.5).');
    end
    options.PathLengthTimeAllowance_s = double(allowance_s);
    validateattributes(options.MaxArrivalTrials,{'numeric'},{'scalar','finite','integer','positive'});
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
