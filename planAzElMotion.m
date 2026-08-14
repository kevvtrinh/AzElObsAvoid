function result = planAzElMotion( obstacles, initialState, goalState, limits, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = planAzElMotion()
%   result = planAzElMotion(obstacles, initialState, goalState, limits)
%   result = planAzElMotion( ...
%       obstacles, initialState, goalState, limits, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Plan one general collision-free azimuth/elevation motion through
%     adaptive visibility search and the requested spatial retimer.
%**************************************************************************
% INPUTS
%   - obstacles (canonical obstacle data, nested cell array, or [])
%       Static or time-varying protected polygon geometry.
%   - initialState, goalState (scalar structs)
%       time_s, position_deg, velocity_deg_s, and acceleration_deg_s2.
%   - limits (scalar struct)
%       Per-axis velocity, acceleration, and optional jerk limits.
%   - optionOverrides (scalar struct, optional; default struct())
%       Partial overrides of the zero-input defaults.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Stable plan, trajectory, validation, search, and display record.
%       Expected infeasibility returns Success=false; invalid input throws.
%**************************************************************************
% UNITS
%   - Angles use degrees and time uses seconds. Derivative suffixes state
%     deg/s, deg/s^2, and deg/s^3.
%**************************************************************************
defaults = plannerDefaults();
if nargin == 0
    result = defaults;
    return;
end
if nargin < 5 || isempty(optionOverrides)
    optionOverrides = struct();
end
planningTimer = tic;
options = resolveOptions(defaults, optionOverrides);
initialState = normalizeState(initialState, "initialState");
goalState = normalizeState(goalState, "goalState");
limits = normalizeLimits(limits);
if goalState.time_s <= initialState.time_s
    error("planAzElMotion:InvalidTimeWindow", ...
        "goalState.time_s must be greater than initialState.time_s.");
end
%% Section 1: Build Protected Geometry & Visibility Routes
protectedAzElData = combineAzElObstacles(obstacles);
obstacleField = buildAzElTimeObstacleField(protectedAzElData, struct( ...
    "MaximumVerticesPerRegion", options.MaximumVerticesPerRegion, "Verbose", options.Verbose));
[originalObstacleField, obstacleSafetyMargins_deg] = ...
    recoverOriginalAzElObstacleField(obstacleField);
startBlocked = queryAzElTimeObstacle(obstacleField, ...
    initialState.position_deg(1), initialState.position_deg(2), initialState.time_s);
goalBlocked = queryAzElTimeObstacle(obstacleField, ...
    goalState.position_deg(1), goalState.position_deg(2), goalState.time_s);
endpointBlocked = logical(startBlocked || goalBlocked);
candidateClearance_deg = max(0.05, 3 * min(options.TurnRadius_deg, 0.45));
viewOptions = struct( "FigureVisible", options.FigureVisible, ...
    "Title", options.Title, "ShowSweptSurfaces", options.ShowSweptSurfaces, ...
    "MaximumDisplayedSlicesPerObstacle", options.MaximumDisplayedSlicesPerObstacle, ...
    "UseParallel", options.UseParallel, "Verbose", options.Verbose, ...
    "ShowBoundaryCandidates", true, "ShowBoundaryConnections", true, ...
    "ShowVisibilityGraph", true, "CandidateClearance_deg", candidateClearance_deg, ...
    "CornerAngleThreshold_deg", options.CornerAngleThreshold_deg, ...
    "PolygonCandidateMode", options.PolygonCandidateMode, ...
    "ExtremeDirectionCount", options.ExtremeDirectionCount, "MaximumTangenciesPerReference", ...
        options.MaximumTangenciesPerReference, "BoundaryRouteReductionTolerance_deg", ...
        options.BoundaryRouteReductionTolerance_deg, ...
    "VisibilitySampleStep_deg", options.VisibilitySampleStep_deg);
spaceView = visualizeAzElTimeSpace( obstacleField, initialState, goalState, viewOptions);
[candidateRoutes_deg, snapshotTime_s, graphIndex, consolidation] = ...
    collectRoutes(spaceView.VisibilityGraphs, initialState, goalState, ...
    options.MaximumRetimedVisibilityRoutes);
candidateCount = numel(candidateRoutes_deg);
%% Section 2: Smooth, Retime & Check Every Candidate
candidateTimedPaths = cell(candidateCount, 1);
candidateSmoothPaths = cell(candidateCount, 1);
candidateBlocked = cell(candidateCount, 1);
retimed = false(candidateCount, 1);
collisionFree = false(candidateCount, 1);
arrivalTime_s = inf(candidateCount, 1);
attemptArrivalTime_s = inf(candidateCount, 1);
pathLength_deg = zeros(candidateCount, 1);
candidateMessage = strings(candidateCount, 1);
for candidateIndex = 1:candidateCount
    route_deg = candidateRoutes_deg{candidateIndex};
    pathLength_deg(candidateIndex) = sum(vecnorm(diff(route_deg, 1, 1), 2, 2));
    try
        if ~routeWithinAzimuthPolicy(route_deg, options)
            error("planAzElMotion:AzimuthPolicy", ...
                "Candidate violates the configured azimuth interval.");
        end
        smoothPath = smoothRoute(route_deg, obstacleField, ...
            snapshotTime_s(candidateIndex), options);
        timedPath = retimeSpatialPath(smoothPath, initialState, goalState, limits, options);
        blocked = false(0, 1);
        if timedPath.Success
            blocked = queryAzElTimedPathCollision(obstacleField, ...
                timedPath.time_s, timedPath.position_deg, struct( "TimePaddingSamples", ...
                    options.CollisionTimePaddingSamples, "BoundaryIsOccupied", false));
        end
    catch candidateError
        smoothPath = emptySmoothPath(route_deg);
        timedPath = emptyTimedPath( limits, options, string(candidateError.message));
        blocked = false(0, 1);
    end
    candidateTimedPaths{candidateIndex} = timedPath;
    candidateSmoothPaths{candidateIndex} = smoothPath;
    candidateBlocked{candidateIndex} = logical(blocked(:));
    retimed(candidateIndex) = timedPath.Success;
    candidateMessage(candidateIndex) = timedPath.Message;
    if timedPath.Success
        attemptArrivalTime_s(candidateIndex) = timedPath.GoalLineInterceptTime_s;
    end
    collisionFree(candidateIndex) = timedPath.Success && ~any(blocked);
    if collisionFree(candidateIndex)
        arrivalTime_s(candidateIndex) = timedPath.GoalLineInterceptTime_s;
    elseif timedPath.Success
        candidateMessage(candidateIndex) = ...
            "The complete timed trajectory intersects protected geometry.";
    end
end
directFraction = linspace(0, 1, 501).';
directPosition_deg = initialState.position_deg + directFraction .* ...
    (goalState.position_deg - initialState.position_deg);
directTime_s = initialState.time_s + directFraction .* (goalState.time_s - initialState.time_s);
directBlocked = queryAzElTimedPathCollision(obstacleField, ...
    directTime_s, directPosition_deg, struct( ...
    "TimePaddingSamples", options.CollisionTimePaddingSamples));
source = repmat("visibilityGraph", candidateCount, 1);
source(1) = "direct";
candidateDiagnostics = table((1:candidateCount).', source, ...
    snapshotTime_s, graphIndex, pathLength_deg, retimed, collisionFree, ...
    attemptArrivalTime_s, arrivalTime_s, candidateMessage, ...
    'VariableNames', {'Index','Source','SnapshotTime_s','GraphIndex', ...
    'PathLength_deg','Retimed','CollisionFree','AttemptArrivalTime_s', ...
    'ArrivalTime_s','Message'});
%% Section 3: Select & Independently Validate One Result
feasibleIndex = find(isfinite(arrivalTime_s) & ~endpointBlocked);
planningSucceeded = ~isempty(feasibleIndex);
if planningSucceeded
    ranking = [arrivalTime_s(feasibleIndex), pathLength_deg(feasibleIndex), feasibleIndex];
    [~, order] = sortrows(ranking, [1 2 3]);
    selectedCandidateIndex = feasibleIndex(order(1));
else
    selectedCandidateIndex = selectBestAttempt( ...
        retimed, attemptArrivalTime_s, pathLength_deg, graphIndex);
end
selectedRoute_deg = candidateRoutes_deg{selectedCandidateIndex};
timedSlopePath = candidateTimedPaths{selectedCandidateIndex};
smoothPath = candidateSmoothPaths{selectedCandidateIndex};
bestAttemptPosition_deg = selectedRoute_deg;
bestAttemptTime_s = repmat(snapshotTime_s(selectedCandidateIndex), ...
    size(selectedRoute_deg, 1), 1);
if ~isempty(timedSlopePath.time_s)
    bestAttemptPosition_deg = timedSlopePath.position_deg;
    bestAttemptTime_s = timedSlopePath.time_s;
end
bestAttemptBlocked = queryAzElTimedPathCollision(originalObstacleField, ...
    bestAttemptTime_s, bestAttemptPosition_deg, struct( ...
    "TimePaddingSamples", options.CollisionTimePaddingSamples));
bestAttemptProtectedBlocked = queryAzElTimedPathCollision(obstacleField, ...
    bestAttemptTime_s, bestAttemptPosition_deg, struct( ...
    "TimePaddingSamples", options.CollisionTimePaddingSamples, "BoundaryIsOccupied", false));
validation = validatePlan(planningSucceeded, endpointBlocked, ...
    timedSlopePath, bestAttemptProtectedBlocked, goalState, limits, options);
success = validation.Passed;
message = "Adaptive visibility and certified spatial retiming succeeded.";
terminationReason = "goalReached";
if endpointBlocked
    message = "The protected geometry contains the start or goal.";
    terminationReason = "endpointBlocked";
elseif ~planningSucceeded
    message = "No candidate satisfies collision and motion constraints.";
    terminationReason = "noFeasibleCandidate";
elseif ~success
    message = "Independent post-validation failed: " + validation.Message;
    terminationReason = "validationFailed";
end
%% Section 4: Display Returned Data & Assemble Stable Output
[timedSlopeHandle, failureCollisionHandles] = plotAttempt( ...
    spaceView, bestAttemptPosition_deg, bestAttemptTime_s, ...
    bestAttemptBlocked, bestAttemptProtectedBlocked, success, options);
animation = struct();
if success && options.ShowAnimation
    pause_s = options.AnimationPause_s;
    if options.FigureVisible == "off"
        pause_s = 0;
    end
    animation = animateAzElTimedSlopePath(timedSlopePath, ...
        obstacleField, struct("FigureVisible", options.FigureVisible, ...
        "FrameStride", options.AnimationFrameStride, ...
        "Pause_s", pause_s, "Title", options.Title + " animation"));
end
kinematicPlot = struct();
if ~isempty(timedSlopePath.time_s) && options.ShowKinematicPlot
    kinematicPlot = plotKinematics(timedSlopePath, limits, options);
end
selectedPath = emptyGraph();
if graphIndex(selectedCandidateIndex) > 0
    selectedPath = spaceView.VisibilityGraphs(graphIndex(selectedCandidateIndex));
end
elapsedPlanningTime_s = toc(planningTimer);
searchDiagnostics = struct( "VisibilityGraphCount", numel(spaceView.VisibilityGraphs), ...
    "SuccessfulVisibilityGraphCount", nnz([spaceView.VisibilityGraphs.Success]), ...
    "CandidateRouteCount", candidateCount, "FeasibleCandidateCount", nnz(collisionFree), ...
    "SelectedCandidateIndex", selectedCandidateIndex, ...
    "BestAttemptPosition_deg", bestAttemptPosition_deg, ...
    "BestAttemptTime_s", bestAttemptTime_s, "ElapsedPlanningTime_s", elapsedPlanningTime_s, ...
    "TerminationReason", terminationReason);
progressLog = makeProgressLog(success, message, selectedCandidateIndex);
if options.Verbose
    fprintf("[AzEl] %s Candidate %d; elapsed %.3f s.\n", ...
        message, selectedCandidateIndex, elapsedPlanningTime_s);
end
result = struct( "Success", success, "Message", message, ...
    "TerminationReason", terminationReason, ...
    "MotionType", "velocityCarrying", "Options", options, "azElData", protectedAzElData, ...
    "originalAzElData", originalObstacleField.SourceAzElData, ...
    "inflatedAzElData", protectedAzElData, "obstacleField", obstacleField, ...
    "originalObstacleField", originalObstacleField, ...
    "initialState", initialState, "goalState", goalState, "limits", limits, ...
    "obstacleSafetyMargins_deg", obstacleSafetyMargins_deg, ...
    "safetyMarginDiagnostics", spaceView.SafetyMarginDiagnostics, ...
    "directPosition_deg", directPosition_deg, "directTime_s", directTime_s, ...
    "directBlocked", logical(directBlocked(:)), ...
    "candidateRoutes_deg", {candidateRoutes_deg}, ...
    "candidateTimedPaths", {candidateTimedPaths}, ...
    "candidateSmoothPaths", {candidateSmoothPaths}, "candidateBlocked", {candidateBlocked}, ...
    "candidateDiagnostics", candidateDiagnostics, ...
    "selectedCandidateIndex", selectedCandidateIndex, ...
    "selectedRoute_deg", selectedRoute_deg, "selectedPath", selectedPath, ...
    "selectedVisibilityPath", selectedPath, ...
    "retimedVisibilityGraphIndices", consolidation.SelectedGraphIndices, ...
    "visibilityRouteConsolidation", consolidation, ...
    "ParallelExecution", spaceView.ParallelExecution, ...
    "smoothPath", smoothPath, "timedSlopePath", timedSlopePath, ...
    "goalLineInterceptTime_s", timedSlopePath.GoalLineInterceptTime_s, ...
    "timedSlopeHandle", timedSlopeHandle, ...
    "bestAttemptPosition_deg", bestAttemptPosition_deg, ...
    "bestAttemptTime_s", bestAttemptTime_s, ...
    "bestAttemptBlocked", logical(bestAttemptBlocked(:)), "bestAttemptProtectedBlocked", ...
        logical(bestAttemptProtectedBlocked(:)), ...
    "failureCollisionHandles", failureCollisionHandles, "usedDiagnosticRetiming", false, ...
    "diagnosticTimedPathAvailable", ~isempty(timedSlopePath.time_s), ...
    "spaceView", spaceView, "animation", animation, "kinematicPlot", kinematicPlot, ...
    "SearchDiagnostics", searchDiagnostics, "ElapsedPlanningTime_s", elapsedPlanningTime_s, ...
    "RandomSeed", [], "ProgressLog", progressLog, "Validation", validation);
end
%% Section 5: Local Functions
function options = plannerDefaults()
%% Section 0: Header & Readme
% Return the argument-independent public planner options.
options = struct( "MotionType", "velocityCarrying", ...
    "GoalTimeMode", "earliestArrival", "SampleTime_s", 0.05, ...
    "TurnRadius_deg", 1.0, "CollisionTimePaddingSamples", 1, ...
    "AllowAzimuthWrapping", false, "AzimuthInterval_deg", [-180 180], ...
    "MaximumVerticesPerRegion", Inf, "VisibilitySampleStep_deg", 0.10, ...
    "CornerAngleThreshold_deg", 15, "PolygonCandidateMode", "adaptive", ...
    "ExtremeDirectionCount", 16, "MaximumTangenciesPerReference", 2, ...
    "BoundaryRouteReductionTolerance_deg", 0.10, "FigureVisible", "on", ...
    "ShowAnimation", true, "ShowKinematicPlot", true, ...
    "AnimationFrameStride", 10, "AnimationPause_s", 0.001, ...
    "ShowSweptSurfaces", true, "MaximumDisplayedSlicesPerObstacle", 10, ...
    "MaximumRetimedVisibilityRoutes", 12, "UseParallel", false, ...
    "Verbose", false, "Title", "Azimuth/elevation motion plan");
end
function options = resolveOptions(defaults, overrides)
%% Section 0: Header & Readme
% Merge partial overrides, warn once for unknown fields, and validate.
if ~isstruct(overrides) || ~isscalar(overrides)
    error("planAzElMotion:InvalidOptions", "optionOverrides must be a scalar struct.");
end
legacyMarginNames = intersect(fieldnames(overrides), ...
    {'SafetyMarginDeg', 'RoundingClearance_deg'}, "stable");
if ~isempty(legacyMarginNames)
    error("planAzElMotion:SafetyMarginMoved", ...
        "Safety margins belong to obstacle data. Remove %s and pass " + ...
        "each margin to makeAzElObstacleData.", strjoin(string(legacyMarginNames), ", "));
end
options = defaults;
names = fieldnames(overrides);
unknown = strings(0, 1);
for index = 1:numel(names)
    name = names{index};
    if isfield(defaults, name)
        if ~isempty(overrides.(name))
            options.(name) = overrides.(name);
        end
    else
        unknown(end + 1, 1) = string(name); %#ok<AGROW>
    end
end
if ~isempty(unknown)
    warning("planAzElMotion:UnknownOptions", ...
        "Ignored unknown option fields: %s. No behavior changed.", strjoin(unknown, ", "));
end
motionType = lower(string(options.MotionType));
if ~isscalar(motionType) || motionType ~= "velocitycarrying"
    error("planAzElMotion:UnsupportedMotionType", ...
        "This feature branch supports MotionType=velocityCarrying only.");
end
options.MotionType = "velocityCarrying";
options.GoalTimeMode = lower(string(options.GoalTimeMode));
if ~isscalar(options.GoalTimeMode) || ...
        ~any(options.GoalTimeMode == ["earliestarrival" "fixedarrival"])
    error("planAzElMotion:InvalidGoalTimeMode", ...
        "GoalTimeMode must be earliestArrival or fixedArrival.");
end
options.FigureVisible = lower(string(options.FigureVisible));
if ~isscalar(options.FigureVisible) || ~any(options.FigureVisible == ["on" "off"])
    error("planAzElMotion:InvalidFigureVisible", "FigureVisible must be on or off.");
end
options.PolygonCandidateMode = lower(string(options.PolygonCandidateMode));
if ~isscalar(options.PolygonCandidateMode) || ~any(options.PolygonCandidateMode == ...
        ["adaptive" "allcorners" "extreme"])
    error("planAzElMotion:InvalidPolygonCandidateMode", ...
        "PolygonCandidateMode must be adaptive, allCorners, or extreme.");
end
logicalNames = ["AllowAzimuthWrapping" "ShowAnimation" ...
    "ShowKinematicPlot" "ShowSweptSurfaces" "Verbose"];
for index = 1:numel(logicalNames)
    name = logicalNames(index);
    value = options.(name);
    if ~(islogical(value) && isscalar(value)) && ~(isnumeric(value) && isscalar(value) && ...
            isfinite(value) && any(value == [0 1]))
        error("planAzElMotion:InvalidLogicalOption", ...
            "%s must be scalar logical or binary numeric.", name);
    end
    options.(name) = logical(value);
end
if (islogical(options.UseParallel) || isnumeric(options.UseParallel)) && ...
        isscalar(options.UseParallel)
    if isnumeric(options.UseParallel) && (~isfinite(options.UseParallel) || ...
            ~any(options.UseParallel == [0 1]))
        error("planAzElMotion:InvalidUseParallel", "Numeric UseParallel must be zero or one.");
    end
    if logical(options.UseParallel)
        options.UseParallel = "on";
    else
        options.UseParallel = "off";
    end
else
    options.UseParallel = lower(string(options.UseParallel));
end
if ~isscalar(options.UseParallel) || ~any(options.UseParallel == ["auto" "on" "off"])
    error("planAzElMotion:InvalidUseParallel", ...
        "UseParallel must be auto, on, off, or scalar logical.");
end
positiveNames = ["SampleTime_s" "TurnRadius_deg" ...
    "VisibilitySampleStep_deg" "ExtremeDirectionCount" "MaximumTangenciesPerReference" ...
    "BoundaryRouteReductionTolerance_deg" ...
    "AnimationFrameStride" "MaximumDisplayedSlicesPerObstacle" ...
    "MaximumRetimedVisibilityRoutes"];
for index = 1:numel(positiveNames)
    validateattributes(options.(positiveNames(index)), {'numeric'}, ...
        {'real','finite','scalar','positive'});
end
validateattributes(options.CollisionTimePaddingSamples, {'numeric'}, ...
    {'real','finite','scalar','integer','nonnegative'});
validateattributes(options.AnimationPause_s, {'numeric'}, ...
    {'real','finite','scalar','nonnegative'});
validateattributes(options.MaximumVerticesPerRegion, {'numeric'}, {'real','scalar','positive'});
validateattributes(options.CornerAngleThreshold_deg, {'numeric'}, ...
    {'real','finite','scalar','>=',0,'<=',180});
validateattributes(options.AzimuthInterval_deg, {'numeric'}, ...
    {'real','finite','vector','numel',2,'increasing'});
options.Title = string(options.Title);
if ~isscalar(options.Title)
    error("planAzElMotion:InvalidTitle", "Title must be scalar text.");
end
end
function state = normalizeState(state, label)
%% Section 0: Header & Readme
% Normalize one endpoint state to scalar time and 1-by-2 state vectors.
if ~isstruct(state) || ~isscalar(state) || ~all(isfield(state, ["time_s" "position_deg"]))
    error("planAzElMotion:InvalidState", "%s must contain time_s and position_deg.", label);
end
if ~isfield(state, "velocity_deg_s") || isempty(state.velocity_deg_s)
    state.velocity_deg_s = [0 0];
end
if ~isfield(state, "acceleration_deg_s2") || isempty(state.acceleration_deg_s2)
    state.acceleration_deg_s2 = [0 0];
end
validateattributes(state.time_s, {'numeric'}, {'real','finite','scalar'});
names = ["position_deg" "velocity_deg_s" "acceleration_deg_s2"];
for index = 1:numel(names)
    name = names(index);
    validateattributes(state.(name), {'numeric'}, {'real','finite','vector','numel',2});
    state.(name) = reshape(double(state.(name)), 1, 2);
end
state.time_s = double(state.time_s);
end
function limits = normalizeLimits(limits)
%% Section 0: Header & Readme
% Normalize physical limits to positive 1-by-2 vectors.
if ~isstruct(limits) || ~isscalar(limits) || ~isfield(limits, "maxVelocity_deg_s")
    error("planAzElMotion:InvalidLimits", "limits must contain maxVelocity_deg_s.");
end
names = ["maxVelocity_deg_s" "maxAcceleration_deg_s2" "maxJerk_deg_s3"];
defaults = {[], [Inf Inf], [Inf Inf]};
for index = 1:numel(names)
    name = names(index);
    if ~isfield(limits, name) || isempty(limits.(name))
        limits.(name) = defaults{index};
    end
    value = limits.(name);
    validateattributes(value, {'numeric'}, {'real','vector','nonempty','positive'});
    if any(isnan(value)) || ~(isscalar(value) || numel(value) == 2)
        error("planAzElMotion:InvalidLimits", ...
            "%s must be scalar or two-element and cannot contain NaN.", name);
    end
    value = reshape(double(value), 1, []);
    if isscalar(value)
        value = repmat(value, 1, 2);
    end
    limits.(name) = value;
end
end
function [routes, snapshotTime_s, graphIndex, diagnostics] = ...
        collectRoutes(graphs, initialState, goalState, maximumCount)
%% Section 0: Header & Readme
% Collect distinct visibility paths and retain cost/time representatives.
routes = {[initialState.position_deg; goalState.position_deg]};
snapshotTime_s = initialState.time_s;
graphIndex = 0;
successful = find([graphs.Success]);
distinct = zeros(0, 1);
for index = reshape(successful, 1, [])
    candidate = graphs(index).PathPosition_deg;
    isDuplicate = false;
    for routeIndex = 1:numel(routes)
        route = routes{routeIndex};
        if isequal(size(route), size(candidate)) && max(abs(route(:) - candidate(:))) <= 1e-9
            isDuplicate = true;
            break;
        end
    end
    if ~isDuplicate
        routes{end + 1, 1} = candidate; %#ok<AGROW>
        snapshotTime_s(end + 1, 1) = graphs(index).Time_s; %#ok<AGROW>
        graphIndex(end + 1, 1) = index; %#ok<AGROW>
        distinct(end + 1, 1) = index; %#ok<AGROW>
    end
end
if numel(routes) > maximumCount + 1
    cost = zeros(numel(routes) - 1, 1);
    for index = 2:numel(routes)
        cost(index - 1) = sum(vecnorm(diff(routes{index}), 2, 2));
    end
    [~, cheapest] = min(cost);
    retained = unique(round(linspace(1, numel(cost), maximumCount))).';
    retained = unique([cheapest; retained], "stable");
    if numel(retained) > maximumCount
        retained = retained(1:maximumCount);
    end
    keep = [1; retained + 1];
    routes = routes(keep);
    snapshotTime_s = snapshotTime_s(keep);
    graphIndex = graphIndex(keep);
end
diagnostics = struct( "SuccessfulGraphCount", numel(successful), ...
    "DistinctRouteCount", numel(distinct), "MaximumRetimedVisibilityRoutes", maximumCount, ...
    "SelectedRouteCount", numel(routes) - 1, ...
    "SelectedGraphIndices", graphIndex(graphIndex > 0));
end
function respects = routeWithinAzimuthPolicy(position_deg, options)
%% Section 0: Header & Readme
% Check the configured non-wrapping azimuth interval.
if options.AllowAzimuthWrapping
    respects = true;
    return;
end
azimuth_deg = position_deg(:, 1);
respects = all(azimuth_deg >= options.AzimuthInterval_deg(1) - 1e-9) && ...
    all(azimuth_deg <= options.AzimuthInterval_deg(2) + 1e-9) && ...
    all(abs(diff(azimuth_deg)) < 180);
end
function selectedIndex = selectBestAttempt( retimed, arrivalTime_s, pathLength_deg, graphIndex)
%% Section 0: Header & Readme
% Rank failed candidates by usable timing, path length, and directness.
rank = [~retimed(:), arrivalTime_s(:), pathLength_deg(:), ...
    graphIndex(:) > 0, (1:numel(retimed)).'];
[~, order] = sortrows(rank, [1 2 3 4 5]);
selectedIndex = order(1);
end
function smoothPath = smoothRoute( route_deg, obstacleField, collisionTime_s, options)
%% Section 0: Header & Readme
% Replace every resolvable polyline turn with the same symmetric G3 blend.
validateattributes(route_deg, {'numeric'}, {'real','finite','2d','ncols',2});
step_deg = diff(route_deg, 1, 1);
route_deg = route_deg([true; vecnorm(step_deg, 2, 2) > 1e-9], :);
if size(route_deg, 1) < 2
    error("planAzElMotion:ZeroLengthRoute", "A candidate route needs two distinct points.");
end
cornerCount = size(route_deg, 1) - 2;
cornerTemplate = struct( "PathPointIndex", 0, "Position_deg", zeros(1, 2), ...
    "DeflectionAngle_rad", 0, "AppliedRadius_deg", 0, "EntryPosition_deg", zeros(1, 2), ...
    "ExitPosition_deg", zeros(1, 2), "ControlPoints_deg", zeros(6, 2), ...
    "Smoothed", false, "MandatoryStop", false, "Reason", "");
corners = repmat(cornerTemplate, cornerCount, 1);
minimumRadius_deg = min(0.02, options.TurnRadius_deg);
for cornerIndex = 1:cornerCount
    pointIndex = cornerIndex + 1;
    corner = route_deg(pointIndex, :);
    incomingVector = corner - route_deg(pointIndex - 1, :);
    outgoingVector = route_deg(pointIndex + 1, :) - corner;
    incomingLength_deg = norm(incomingVector);
    outgoingLength_deg = norm(outgoingVector);
    incoming = incomingVector / incomingLength_deg;
    outgoing = outgoingVector / outgoingLength_deg;
    angle_rad = acos(min(1, max(-1, dot(incoming, outgoing))));
    turnCross = incoming(1) * outgoing(2) - incoming(2) * outgoing(1);
    diagnostic = cornerTemplate;
    diagnostic.PathPointIndex = pointIndex;
    diagnostic.Position_deg = corner;
    diagnostic.EntryPosition_deg = corner;
    diagnostic.ExitPosition_deg = corner;
    diagnostic.DeflectionAngle_rad = angle_rad;
    if angle_rad <= 1e-9
        diagnostic.Reason = "collinear";
        corners(cornerIndex) = diagnostic;
        continue;
    end
    if pi - angle_rad <= 1e-6 || abs(turnCross) <= 1e-12
        diagnostic.MandatoryStop = true;
        diagnostic.Reason = "unresolved reversal";
        corners(cornerIndex) = diagnostic;
        continue;
    end
    tangentScale = (384 / 125) * sin(angle_rad / 2) / cos(angle_rad / 2)^2;
    maximumRadius_deg = 0.45 * min(incomingLength_deg, outgoingLength_deg) / tangentScale;
    requestedRadius_deg = min(options.TurnRadius_deg, maximumRadius_deg);
    trialRadii_deg = requestedRadius_deg * 0.65 .^ (0:60);
    trialRadii_deg = trialRadii_deg( trialRadii_deg >= minimumRadius_deg);
    if requestedRadius_deg >= minimumRadius_deg && (isempty(trialRadii_deg) || ...
            trialRadii_deg(end) > minimumRadius_deg * (1 + eps))
        trialRadii_deg(end + 1) = minimumRadius_deg; %#ok<AGROW>
    end
    for radius_deg = trialRadii_deg
        trim_deg = radius_deg * tangentScale;
        controlPoints_deg = quinticControls( corner, trim_deg, incoming, outgoing);
        primitive = quinticLookup(controlPoints_deg);
        checkCount = max(21, ceil(primitive.Length_deg / 0.02) + 1);
        checkS_deg = linspace(0, primitive.Length_deg, checkCount).';
        checkPosition_deg = samplePrimitive(primitive, checkS_deg);
        blocked = queryAzElTimedPathCollision(obstacleField, ...
            collisionTime_s, checkPosition_deg, struct( ...
            "TimePaddingSamples", options.CollisionTimePaddingSamples, ...
            "BoundaryIsOccupied", false));
        if any(blocked)
            continue;
        end
        diagnostic.AppliedRadius_deg = radius_deg;
        diagnostic.EntryPosition_deg = controlPoints_deg(1, :);
        diagnostic.ExitPosition_deg = controlPoints_deg(end, :);
        diagnostic.ControlPoints_deg = controlPoints_deg;
        diagnostic.Smoothed = true;
        diagnostic.Reason = "collision-free G3 blend";
        break;
    end
    if ~diagnostic.Smoothed
        diagnostic.MandatoryStop = true;
        diagnostic.Reason = "no collision-free blend";
    end
    corners(cornerIndex) = diagnostic;
end
primitiveTemplate = struct( "Type", "", "StartPosition_deg", zeros(1, 2), ...
    "EndPosition_deg", zeros(1, 2), "Direction", zeros(1, 2), ...
    "Length_deg", 0, "StartArcLength_deg", 0, ...
    "EndArcLength_deg", 0, "ControlPoints_deg", zeros(6, 2), "ParameterGrid", zeros(0, 1), ...
    "ArcLengthGrid_deg", zeros(0, 1), "CornerPathPointIndex", 0);
primitives = repmat(primitiveTemplate, 0, 1);
mandatoryStopArcLength_deg = zeros(0, 1);
currentPosition_deg = route_deg(1, :);
currentArcLength_deg = 0;
for cornerIndex = 1:cornerCount
    corner = corners(cornerIndex);
    [primitives, currentArcLength_deg] = appendLine( ...
        primitives, primitiveTemplate, currentPosition_deg, ...
        corner.EntryPosition_deg, currentArcLength_deg);
    if corner.Smoothed
        primitive = quinticLookup(corner.ControlPoints_deg);
        primitive.StartArcLength_deg = currentArcLength_deg;
        primitive.EndArcLength_deg = currentArcLength_deg + primitive.Length_deg;
        primitive.CornerPathPointIndex = corner.PathPointIndex;
        primitives(end + 1, 1) = primitive; %#ok<AGROW>
        currentArcLength_deg = primitive.EndArcLength_deg;
        currentPosition_deg = corner.ExitPosition_deg;
    else
        currentPosition_deg = corner.Position_deg;
        if corner.MandatoryStop
            mandatoryStopArcLength_deg(end + 1, 1) = currentArcLength_deg; %#ok<AGROW>
        end
    end
end
[primitives, currentArcLength_deg] = appendLine( ...
    primitives, primitiveTemplate, currentPosition_deg, route_deg(end, :), ...
    currentArcLength_deg);
if isempty(primitives)
    error("planAzElMotion:EmptySmoothPath", "Smoothing produced no nonzero primitive.");
end
primitiveBoundaryS_deg = [primitives.EndArcLength_deg].';
sampleS_deg = unique([0; (0:0.05:currentArcLength_deg).'; ...
    primitiveBoundaryS_deg; mandatoryStopArcLength_deg; currentArcLength_deg]);
definition = struct( "Primitives", primitives, "TotalLength_deg", currentArcLength_deg);
samples = samplePath(definition, sampleS_deg);
mandatoryStop = false(size(sampleS_deg));
for stopIndex = 1:numel(mandatoryStopArcLength_deg)
    [~, index] = min(abs( sampleS_deg - mandatoryStopArcLength_deg(stopIndex)));
    mandatoryStop(index) = true;
end
smoothPath = struct( "Success", true, ...
    "Message", sprintf("Rounded %d corners; %d stops remain.", ...
        nnz([corners.Smoothed]), nnz([corners.MandatoryStop])), ...
    "OriginalPathPosition_deg", route_deg, ...
    "Primitives", primitives, "TotalLength_deg", currentArcLength_deg, ...
    "SampleArcLength_deg", sampleS_deg, ...
    "position_deg", samples.position_deg, "tangent", samples.tangent, ...
    "secondDerivative_deg_inv", samples.secondDerivative_deg_inv, ...
    "thirdDerivative_deg_inv2", samples.thirdDerivative_deg_inv2, ...
    "curvature_deg_inv", samples.curvature_deg_inv, ...
    "PrimitiveIndex", samples.PrimitiveIndex, "PrimitiveType", samples.PrimitiveType, ...
    "MandatoryStop", mandatoryStop, ...
    "MandatoryStopArcLength_deg", mandatoryStopArcLength_deg, "CornerDiagnostics", corners, ...
    "RoundedCornerCount", nnz([corners.Smoothed]), ...
    "MandatoryStopCount", nnz([corners.MandatoryStop]), ...
    "Options", struct("TurnRadius_deg", options.TurnRadius_deg));
end
function controlPoints_deg = quinticControls( corner_deg, trim_deg, incoming, outgoing)
%% Section 0: Header & Readme
% Construct the symmetric quintic with zero q'' and q''' at both joins.
controlPoints_deg = [ corner_deg - trim_deg * incoming; ...
    corner_deg - 0.5 * trim_deg * incoming; corner_deg; corner_deg; ...
    corner_deg + 0.5 * trim_deg * outgoing; corner_deg + trim_deg * outgoing];
end
function primitive = quinticLookup(controlPoints_deg)
%% Section 0: Header & Readme
% Build one monotone parameter-to-arc-length lookup.
controlLength_deg = sum(vecnorm(diff(controlPoints_deg), 2, 2));
parameterGrid = linspace(0, 1, max(100, ceil(controlLength_deg / 0.004)) + 1).';
[~, firstDerivative] = evaluateQuintic(controlPoints_deg, parameterGrid);
parameterSpeed_deg = vecnorm(firstDerivative, 2, 2);
if any(parameterSpeed_deg <= 1e-10)
    error("planAzElMotion:DegenerateQuintic", ...
        "A quintic blend has a zero parameter derivative.");
end
arcLengthGrid_deg = cumtrapz(parameterGrid, parameterSpeed_deg);
primitive = struct( "Type", "quintic", "StartPosition_deg", controlPoints_deg(1, :), ...
    "EndPosition_deg", controlPoints_deg(end, :), "Direction", zeros(1, 2), ...
    "Length_deg", arcLengthGrid_deg(end), "StartArcLength_deg", 0, "EndArcLength_deg", 0, ...
    "ControlPoints_deg", controlPoints_deg, "ParameterGrid", parameterGrid, ...
    "ArcLengthGrid_deg", arcLengthGrid_deg, "CornerPathPointIndex", 0);
end
function [position_deg, firstDerivative, secondDerivative, ...
        thirdDerivative] = evaluateQuintic(controlPoints_deg, parameter)
%% Section 0: Header & Readme
% Evaluate a quintic Bezier and its first three parameter derivatives.
parameter = double(parameter(:));
oneMinus = 1 - parameter;
position_deg = [oneMinus.^5, 5 * oneMinus.^4 .* parameter, 10 * oneMinus.^3 .* parameter.^2, ...
    10 * oneMinus.^2 .* parameter.^3, ...
    5 * oneMinus .* parameter.^4, parameter.^5] * controlPoints_deg;
firstDerivative = [oneMinus.^4, 4 * oneMinus.^3 .* parameter, ...
    6 * oneMinus.^2 .* parameter.^2, 4 * oneMinus .* parameter.^3, parameter.^4] * ...
    (5 * diff(controlPoints_deg, 1, 1));
secondDerivative = [oneMinus.^3, 3 * oneMinus.^2 .* parameter, ...
    3 * oneMinus .* parameter.^2, parameter.^3] * (20 * diff(controlPoints_deg, 2, 1));
thirdDerivative = [oneMinus.^2, 2 * oneMinus .* parameter, parameter.^2] * ...
    (60 * diff(controlPoints_deg, 3, 1));
end
function [primitives, endS_deg] = appendLine( ...
        primitives, template, start_deg, goal_deg, startS_deg)
%% Section 0: Header & Readme
% Append one nonzero straight primitive with exact arc metadata.
delta_deg = goal_deg - start_deg;
length_deg = norm(delta_deg);
endS_deg = startS_deg;
if length_deg <= 1e-9
    return;
end
primitive = template;
primitive.Type = "line";
primitive.StartPosition_deg = start_deg;
primitive.EndPosition_deg = goal_deg;
primitive.Direction = delta_deg / length_deg;
primitive.Length_deg = length_deg;
primitive.StartArcLength_deg = startS_deg;
endS_deg = startS_deg + length_deg;
primitive.EndArcLength_deg = endS_deg;
primitives(end + 1, 1) = primitive;
end
function position_deg = samplePrimitive(primitive, localS_deg)
%% Section 0: Header & Readme
% Sample one line or quintic primitive by local arc length.
if primitive.Type == "line"
    position_deg = primitive.StartPosition_deg + localS_deg(:) .* primitive.Direction;
else
    parameter = interp1(primitive.ArcLengthGrid_deg, ...
        primitive.ParameterGrid, localS_deg(:), "pchip");
    position_deg = evaluateQuintic( primitive.ControlPoints_deg, min(max(parameter, 0), 1));
end
end
function samples = samplePath(smoothPath, arcLength_deg)
%% Section 0: Header & Readme
% Evaluate exact position and first three arc derivatives on the path.
queryS_deg = double(arcLength_deg(:));
totalLength_deg = smoothPath.TotalLength_deg;
tolerance_deg = 1e-10 * max(1, totalLength_deg);
if any(queryS_deg < -tolerance_deg) || any(queryS_deg > totalLength_deg + tolerance_deg)
    error("planAzElMotion:ArcLengthOutsidePath", ...
        "Arc-length queries must remain on the smooth path; " + ...
        "observed [%.17g, %.17g] deg, path [0, %.17g] deg.", ...
        min(queryS_deg), max(queryS_deg), totalLength_deg);
end
queryS_deg = min(max(queryS_deg, 0), totalLength_deg);
count = numel(queryS_deg);
position_deg = zeros(count, 2);
tangent = zeros(count, 2);
second = zeros(count, 2);
third = zeros(count, 2);
primitiveIndex = zeros(count, 1);
primitiveType = strings(count, 1);
primitives = smoothPath.Primitives;
for index = 1:numel(primitives)
    primitive = primitives(index);
    belongs = queryS_deg >= primitive.StartArcLength_deg - tolerance_deg;
    if index < numel(primitives)
        belongs = belongs & queryS_deg < primitive.EndArcLength_deg - tolerance_deg;
    else
        belongs = belongs & queryS_deg <= primitive.EndArcLength_deg + tolerance_deg;
    end
    belongs = belongs & primitiveIndex == 0;
    if ~any(belongs)
        continue;
    end
    localS_deg = min(max(queryS_deg(belongs) - ...
        primitive.StartArcLength_deg, 0), primitive.Length_deg);
    if primitive.Type == "line"
        position_deg(belongs, :) = primitive.StartPosition_deg + ...
            localS_deg .* primitive.Direction;
        tangent(belongs, :) = repmat( primitive.Direction, nnz(belongs), 1);
    else
        parameter = interp1(primitive.ArcLengthGrid_deg, ...
            primitive.ParameterGrid, localS_deg, "pchip");
        [position, first, parameterSecond, parameterThird] = ...
            evaluateQuintic(primitive.ControlPoints_deg, min(max(parameter, 0), 1));
        speed = vecnorm(first, 2, 2);
        firstSecond = sum(first .* parameterSecond, 2);
        speedSquared = speed.^2;
        secondValue = parameterSecond ./ speedSquared - first .* firstSecond ./ speed.^4;
        thirdValue = parameterThird ./ speed.^3 - ...
            3 * parameterSecond .* firstSecond ./ speed.^5 - ...
            first .* (sum(parameterSecond.^2, 2) + ...
            sum(first .* parameterThird, 2)) ./ speed.^5 + ...
            4 * first .* firstSecond.^2 ./ speed.^7;
        position_deg(belongs, :) = position;
        tangent(belongs, :) = first ./ speed;
        second(belongs, :) = secondValue;
        third(belongs, :) = thirdValue;
    end
    primitiveIndex(belongs) = index;
    primitiveType(belongs) = primitive.Type;
end
if any(primitiveIndex == 0)
    error("planAzElMotion:PrimitiveCoverage", ...
        "Smooth primitives do not cover every requested arc length.");
end
samples = struct( "arcLength_deg", queryS_deg, "position_deg", position_deg, ...
    "tangent", tangent, "secondDerivative_deg_inv", second, ...
    "thirdDerivative_deg_inv2", third, "curvature_deg_inv", vecnorm(second, 2, 2), ...
    "PrimitiveIndex", primitiveIndex, "PrimitiveType", primitiveType);
end
function timedPath = retimeSpatialPath(smoothPath, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% Retime one fixed G3 path with certified spatial limits in either mode.
tolerance = 1e-9;
totalLength_deg = smoothPath.TotalLength_deg;
endpoint = samplePath(smoothPath, [0; totalLength_deg]);
initialSpeed_deg_s = boundarySpeed( ...
    initialState.velocity_deg_s, endpoint.tangent(1, :), tolerance);
goalSpeed_deg_s = boundarySpeed( goalState.velocity_deg_s, endpoint.tangent(2, :), tolerance);
requiredInitialAcceleration_deg_s2 = ...
    endpoint.secondDerivative_deg_inv(1, :) * initialSpeed_deg_s^2;
requiredGoalAcceleration_deg_s2 = endpoint.secondDerivative_deg_inv(2, :) * goalSpeed_deg_s^2;
if norm(initialState.acceleration_deg_s2 - ...
        requiredInitialAcceleration_deg_s2) > tolerance || ...
        norm(goalState.acceleration_deg_s2 - requiredGoalAcceleration_deg_s2) > tolerance
    timedPath = emptyTimedPath(limits, options, ...
        "Endpoint acceleration does not match the path curvature.");
    return;
end
jerkConstrained = any(isfinite(limits.maxJerk_deg_s3));
geometricPrimitiveCount = numel(smoothPath.Primitives);
runPrimitiveIndex = (1:geometricPrimitiveCount).';
boundaryS_deg = [0; [smoothPath.Primitives.EndArcLength_deg].'];
if ~jerkConstrained
    boundaryS_deg = 0;
    runPrimitiveIndex = zeros(0, 1);
    for primitiveIndex = 1:geometricPrimitiveCount
        primitive = smoothPath.Primitives(primitiveIndex);
        cellCount = max(2, ceil(primitive.Length_deg / 0.1));
        localS_deg = linspace(primitive.StartArcLength_deg, ...
            primitive.EndArcLength_deg, cellCount + 1).';
        boundaryS_deg = [boundaryS_deg; localS_deg(2:end)]; %#ok<AGROW>
        runPrimitiveIndex = [runPrimitiveIndex; ...
            repmat(primitiveIndex, cellCount, 1)]; %#ok<AGROW>
    end
end
runCount = numel(runPrimitiveIndex);
length_deg = diff(boundaryS_deg);
boundTemplate = derivativeBoundsTemplate();
bounds = repmat(boundTemplate, runCount, 1);
maximumSpeed_deg_s = zeros(runCount, 1);
maximumAcceleration_deg_s2 = zeros(runCount, 1);
maximumJerk_deg_s3 = zeros(runCount, 1);
effectiveLimits = limits;
unconstrainedAcceleration = ~isfinite(limits.maxAcceleration_deg_s2);
effectiveLimits.maxAcceleration_deg_s2(unconstrainedAcceleration) = ...
    100 * max(limits.maxVelocity_deg_s);
unconstrainedJerk = ~isfinite(limits.maxJerk_deg_s3);
if jerkConstrained
    effectiveLimits.maxJerk_deg_s3(unconstrainedJerk) = ...
        1000 * max(1, max(effectiveLimits.maxAcceleration_deg_s2));
end
for index = 1:runCount
    primitiveIndex = runPrimitiveIndex(index);
    bounds(index) = derivativeBounds( ...
        smoothPath.Primitives(primitiveIndex), boundaryS_deg(index), ...
        boundaryS_deg(index + 1), index);
    [maximumSpeed_deg_s(index), maximumAcceleration_deg_s2(index), ...
        maximumJerk_deg_s3(index)] = scalarLimits( bounds(index), effectiveLimits, tolerance);
end
mandatoryStopNode = false(runCount + 1, 1);
for stopS_deg = reshape(smoothPath.MandatoryStopArcLength_deg, 1, [])
    [distance_deg, index] = min(abs(boundaryS_deg - stopS_deg));
    if distance_deg > tolerance * max(1, totalLength_deg)
        error("planAzElMotion:MissingStopNode", ...
            "A mandatory stop does not coincide with a primitive join.");
    end
    mandatoryStopNode(index) = true;
end
[nodeSpeed_deg_s, feasible, failureMessage] = ...
    accelerationNodeSpeeds(length_deg, maximumSpeed_deg_s, bounds, ...
    limits.maxAcceleration_deg_s2, initialSpeed_deg_s, ...
    goalSpeed_deg_s, mandatoryStopNode, tolerance);
if jerkConstrained
    [nodeSpeed_deg_s, feasible, failureMessage] = ...
        reachableNodeSpeeds(length_deg, maximumSpeed_deg_s, ...
        maximumAcceleration_deg_s2, maximumJerk_deg_s3, ...
        initialSpeed_deg_s, goalSpeed_deg_s, mandatoryStopNode, tolerance);
end
if ~feasible
    timedPath = emptyTimedPath(limits, options, failureMessage);
    return;
end
profiles = repmat(profileTemplate(), runCount, 1);
runEndpoint = samplePath(smoothPath, boundaryS_deg);
for index = 1:runCount
    if jerkConstrained
        [profile, feasible, failureMessage] = minimumTimeProfile( ...
            length_deg(index), nodeSpeed_deg_s(index), ...
            nodeSpeed_deg_s(index + 1), maximumSpeed_deg_s(index), ...
            maximumAcceleration_deg_s2(index), maximumJerk_deg_s3(index), tolerance);
    else
        profile = profileTemplate();
        profile.Length_deg = length_deg(index);
        profile.StartSpeed_deg_s = nodeSpeed_deg_s(index);
        profile.EndSpeed_deg_s = nodeSpeed_deg_s(index + 1);
        profile.Duration_s = 2 * length_deg(index) / ...
            (profile.StartSpeed_deg_s + profile.EndSpeed_deg_s);
        profile.TangentialAcceleration_deg_s2 = ...
            (profile.EndSpeed_deg_s^2 - profile.StartSpeed_deg_s^2) / (2 * length_deg(index));
        profile.PeakSpeed_deg_s = max( profile.StartSpeed_deg_s, profile.EndSpeed_deg_s);
        profile.PeakAcceleration_deg_s2 = abs(profile.TangentialAcceleration_deg_s2);
        profile.PeakJerk_deg_s3 = NaN;
        profile.PhaseDuration_s(1) = profile.Duration_s;
        profile.PhaseStartTime_s(2:end) = profile.Duration_s;
        profile.PhaseStartPosition_deg(2:end) = profile.Length_deg;
        profile.PhaseStartSpeed_deg_s(1) = profile.StartSpeed_deg_s;
        profile.PhaseStartSpeed_deg_s(2:end) = profile.EndSpeed_deg_s;
        profile.PhaseStartAcceleration_deg_s2(1) = profile.TangentialAcceleration_deg_s2;
        feasible = isfinite(profile.Duration_s) && profile.Duration_s > 0;
        failureMessage = "A zero-speed spatial cell is infeasible.";
    end
    if ~feasible
        timedPath = emptyTimedPath(limits, options, ...
            "Primitive " + index + ": " + failureMessage);
        return;
    end
    primitive = smoothPath.Primitives(runPrimitiveIndex(index));
    profile.PrimitiveType = primitive.Type;
    profile.StartPosition_deg = runEndpoint.position_deg(index, :);
    profile.EndPosition_deg = runEndpoint.position_deg(index + 1, :);
    profile.StartArcLength_deg = boundaryS_deg(index);
    profile.EndArcLength_deg = boundaryS_deg(index + 1);
    profile.MaxSpeed_deg_s = maximumSpeed_deg_s(index);
    profile.MaxAcceleration_deg_s2 = maximumAcceleration_deg_s2(index);
    profile.MaxJerk_deg_s3 = maximumJerk_deg_s3(index);
    [profile.PeakVelocityByAxis_deg_s, profile.PeakAccelerationByAxis_deg_s2, ...
        profile.PeakJerkByAxis_deg_s3] = cartesianBounds(bounds(index), profile);
    profiles(index) = profile;
end
minimumMotionDuration_s = sum([profiles.Duration_s]);
minimumArrivalTime_s = initialState.time_s + minimumMotionDuration_s;
timeTolerance_s = tolerance * max(1, abs(goalState.time_s));
if minimumArrivalTime_s > goalState.time_s + timeTolerance_s
    timedPath = emptyTimedPath(limits, options, sprintf( ...
        "Earliest arrival %.9g s exceeds goal time %.9g s.", ...
        minimumArrivalTime_s, goalState.time_s));
    return;
end
waitDuration_s = 0;
if options.GoalTimeMode == "fixedarrival"
    waitDuration_s = max(0, goalState.time_s - minimumArrivalTime_s);
    if waitDuration_s > timeTolerance_s && (norm(initialState.velocity_deg_s) > tolerance || ...
            norm(initialState.acceleration_deg_s2) > tolerance)
        timedPath = emptyTimedPath(limits, options, ...
            "fixedArrival cannot hold a moving initial state.");
        return;
    end
end
motionStartTime_s = initialState.time_s + waitDuration_s;
startTime_s = motionStartTime_s + [0, cumsum([profiles(1:end - 1).Duration_s])];
for index = 1:runCount
    profiles(index).StartTime_s = startTime_s(index);
    profiles(index).EndTime_s = startTime_s(index) + profiles(index).Duration_s;
end
[time_s, sampleS_deg, scalarSpeed_deg_s, scalarAcceleration_deg_s2, ...
    scalarJerk_deg_s3] = sampleProfiles( ...
    profiles, initialState.time_s, waitDuration_s, options.SampleTime_s);
geometry = samplePath(smoothPath, sampleS_deg);
position_deg = geometry.position_deg;
velocity_deg_s = geometry.tangent .* scalarSpeed_deg_s;
acceleration_deg_s2 = geometry.tangent .* scalarAcceleration_deg_s2 + ...
    geometry.secondDerivative_deg_inv .* scalarSpeed_deg_s.^2;
jerk_deg_s3 = geometry.tangent .* scalarJerk_deg_s3 + ...
    3 * geometry.secondDerivative_deg_inv .* scalarSpeed_deg_s .* ...
    scalarAcceleration_deg_s2 + geometry.thirdDerivative_deg_inv2 .* scalarSpeed_deg_s.^3;
position_deg(1, :) = initialState.position_deg;
velocity_deg_s(1, :) = initialState.velocity_deg_s;
acceleration_deg_s2(1, :) = initialState.acceleration_deg_s2;
position_deg(end, :) = goalState.position_deg;
velocity_deg_s(end, :) = goalState.velocity_deg_s;
acceleration_deg_s2(end, :) = goalState.acceleration_deg_s2;
if jerkConstrained
    jerk_deg_s3(1, :) = [0 0];
    jerk_deg_s3(end, :) = [0 0];
else
    jerk_deg_s3(:) = NaN;
end
peakVelocity_deg_s = max(vertcat( profiles.PeakVelocityByAxis_deg_s), [], 1);
peakAcceleration_deg_s2 = max(vertcat( profiles.PeakAccelerationByAxis_deg_s2), [], 1);
peakJerk_deg_s3 = max(vertcat( profiles.PeakJerkByAxis_deg_s3), [], 1);
velocitySatisfied = all(peakVelocity_deg_s <= limits.maxVelocity_deg_s + tolerance);
accelerationSatisfied = all(peakAcceleration_deg_s2 <= ...
    limits.maxAcceleration_deg_s2 + tolerance);
jerkSatisfied = ~jerkConstrained || all(peakJerk_deg_s3 <= limits.maxJerk_deg_s3 + tolerance);
constraintsSatisfied = velocitySatisfied && accelerationSatisfied && jerkSatisfied;
joinSpeed_deg_s = nodeSpeed_deg_s(2:end - 1);
geometricJoin = ismember(boundaryS_deg(2:end - 1), ...
    [smoothPath.Primitives(1:end - 1).EndArcLength_deg].');
ordinaryJoin = geometricJoin & ~mandatoryStopNode(2:end - 1);
ordinaryJoinSpeed_deg_s = joinSpeed_deg_s(ordinaryJoin);
minimumJoinSpeed_deg_s = NaN;
if ~isempty(ordinaryJoinSpeed_deg_s)
    minimumJoinSpeed_deg_s = min(ordinaryJoinSpeed_deg_s);
end
diagnostics = struct( "PeakVelocity_deg_s", peakVelocity_deg_s, ...
    "PeakAcceleration_deg_s2", peakAcceleration_deg_s2, "PeakJerk_deg_s3", peakJerk_deg_s3, ...
    "VelocityMargin_deg_s", limits.maxVelocity_deg_s - peakVelocity_deg_s, ...
    "AccelerationMargin_deg_s2", limits.maxAcceleration_deg_s2 - peakAcceleration_deg_s2, ...
    "JerkMargin_deg_s3", limits.maxJerk_deg_s3 - peakJerk_deg_s3, ...
    "VelocitySatisfied", velocitySatisfied, "AccelerationSatisfied", accelerationSatisfied, ...
    "JerkSatisfied", jerkSatisfied, "JerkConstrained", jerkConstrained, ...
    "FiniteJerkCertified", jerkConstrained && constraintsSatisfied, ...
    "FiniteJerkNumericallyVerified", jerkConstrained && jerkSatisfied, ...
    "ContinuousJerkCertified", false, "G3JoinCount", nnz(ordinaryJoin), ...
    "MinimumG3JoinSpeed_deg_s", minimumJoinSpeed_deg_s, "VelocityCarriedAcrossG3Joins", ...
        isempty(ordinaryJoinSpeed_deg_s) || all(ordinaryJoinSpeed_deg_s > tolerance), ...
    "JoinContinuityOrder", "G3", "GeometryDerivativeBounds", bounds, ...
    "SpatiallyVaryingLimits", true, "SpatialRetimingCellCount", runCount, ...
    "ExecutedMotionProfileCount", runCount, "MandatoryStopCount", nnz(mandatoryStopNode), ...
    "MandatoryStopArcLength_deg", smoothPath.MandatoryStopArcLength_deg, ...
    "CurvatureDiscontinuityStopCount", nnz(mandatoryStopNode), ...
    "RoundedVelocityCarried", isempty(ordinaryJoinSpeed_deg_s) || ...
        all(ordinaryJoinSpeed_deg_s > tolerance), ...
    "MinimumArcSpeed_deg_s", minimumJoinSpeed_deg_s, "Satisfied", constraintsSatisfied);
curveNodeTime_s = [profiles.StartTime_s, profiles(end).EndTime_s].';
timedPath = struct( "Success", constraintsSatisfied, ...
    "Message", "Certified spatial retiming succeeded.", ...
    "time_s", time_s, "position_deg", position_deg, "velocity_deg_s", velocity_deg_s, ...
    "acceleration_deg_s2", acceleration_deg_s2, "jerk_deg_s3", jerk_deg_s3, ...
    "PathPosition_deg", smoothPath.position_deg, "WaypointTime_s", curveNodeTime_s, ...
    "MotionStartTime_s", motionStartTime_s, "WaitDuration_s", waitDuration_s, ...
    "MinimumMotionDuration_s", minimumMotionDuration_s, ...
    "GoalLineInterceptTime_s", time_s(end), "SegmentProfiles", profiles, "Limits", limits, ...
    "Options", struct("GoalTimeMode", options.GoalTimeMode, ...
        "SampleTime_s", options.SampleTime_s), ...
    "ConstraintDiagnostics", diagnostics, "SmoothPath", smoothPath, ...
    "CurveArcLength_deg", boundaryS_deg, "CurveNodeTime_s", curveNodeTime_s, ...
    "CurveSpeed_deg_s", nodeSpeed_deg_s, "CurveSpeedSquared_deg2_s2", nodeSpeed_deg_s.^2, ...
    "CurveTangentialAcceleration_deg_s2", nan(size(nodeSpeed_deg_s)), ...
    "CurveTangentialJerk_deg_s3", nan(size(nodeSpeed_deg_s)), ...
    "SampleArcLength_deg", sampleS_deg, "SampleSpeed_deg_s", scalarSpeed_deg_s, ...
    "SampleTangentialAcceleration_deg_s2", scalarAcceleration_deg_s2, ...
    "SampleTangentialJerk_deg_s3", scalarJerk_deg_s3, ...
    "CurvatureDiscontinuityStopCount", nnz(mandatoryStopNode), ...
    "RetimerType", "certifiedAnalyticSpatialJerk", "MotionType", "velocityCarrying");
if ~jerkConstrained
    timedPath.Message = "Certified acceleration-only spatial retiming succeeded.";
    timedPath.RetimerType = "certifiedSpatialAccelerationForwardBackward";
end
if ~constraintsSatisfied
    timedPath.Message = "Internal continuous constraint certification failed.";
end
end
function speed_deg_s = boundarySpeed(velocity_deg_s, tangent, tolerance)
%% Section 0: Header & Readme
% Project a boundary velocity onto the path and reject lateral motion.
speed_deg_s = dot(velocity_deg_s, tangent);
if speed_deg_s < -tolerance || norm(velocity_deg_s - speed_deg_s * tangent) > tolerance
    error("planAzElMotion:BoundaryVelocityMismatch", ...
        "Endpoint velocity must be nonnegative and tangent to the path.");
end
speed_deg_s = max(0, speed_deg_s);
end
function bounds = derivativeBoundsTemplate()
%% Section 0: Header & Readme
% Return one stable continuous derivative-certificate record.
bounds = struct( "RunIndex", 0, "StartArcLength_deg", 0, ...
    "EndArcLength_deg", 0, "TangentByAxis", zeros(1, 2), ...
    "SecondDerivativeByAxis_deg_inv", zeros(1, 2), ...
    "ThirdDerivativeByAxis_deg_inv2", zeros(1, 2), "CertifiedTangentByAxis", zeros(1, 2), ...
    "CertifiedSecondDerivativeByAxis_deg_inv", zeros(1, 2), ...
    "CertifiedThirdDerivativeByAxis_deg_inv2", zeros(1, 2), ...
    "NumericalTangentByAxis", zeros(1, 2), ...
    "NumericalSecondDerivativeByAxis_deg_inv", zeros(1, 2), ...
    "NumericalThirdDerivativeByAxis_deg_inv2", zeros(1, 2), ...
    "EnvelopeInflationFactor", 1, "SampleCount", 0, "CertificateSubdivisionCount", 0, ...
    "CertificateFallbackCount", 0, "CertificatePrimitiveCount", 1, ...
    "SampledBoundsWithinCertificate", true, "Method", "continuousAnalyticEnvelope");
end
function bounds = derivativeBounds(primitive, startS_deg, endS_deg, index)
%% Section 0: Header & Readme
% Certify the first three arc derivatives over a primitive subinterval.
localStartS_deg = max(0, startS_deg - primitive.StartArcLength_deg);
localEndS_deg = min(primitive.Length_deg, endS_deg - primitive.StartArcLength_deg);
if primitive.Type == "line"
    tangent = abs(primitive.Direction);
    second = [0 0];
    third = [0 0];
    subdivisionCount = 0;
    fallbackCount = 0;
    method = "exactLine";
else
    startLookupIndex = find(primitive.ArcLengthGrid_deg <= localStartS_deg, 1, "last");
    endLookupIndex = find(primitive.ArcLengthGrid_deg >= localEndS_deg, 1, "first");
    parameterInterval = primitive.ParameterGrid( [startLookupIndex, endLookupIndex]);
    certificate = azElInternal.certifyQuinticArcDerivatives( ...
        primitive.ControlPoints_deg, parameterInterval);
    tangent = min(1, (1 + 1e-12) * certificate.TangentByAxis);
    second = (1 + 1e-12) * certificate.SecondDerivativeByAxis_deg_inv;
    third = (1 + 1e-12) * certificate.ThirdDerivativeByAxis_deg_inv2;
    subdivisionCount = certificate.SubdivisionCount;
    fallbackCount = certificate.FallbackCount;
    method = certificate.Method;
end
localPrimitive = primitive;
localPrimitive.StartArcLength_deg = 0;
localPrimitive.EndArcLength_deg = primitive.Length_deg;
sample = samplePath(struct("Primitives", localPrimitive, ...
    "TotalLength_deg", primitive.Length_deg), linspace(localStartS_deg, localEndS_deg, 17).');
numericalTangent = max(abs(sample.tangent), [], 1);
numericalSecond = max(abs(sample.secondDerivative_deg_inv), [], 1);
numericalThird = max(abs(sample.thirdDerivative_deg_inv2), [], 1);
comparisonTolerance = 1e-10 * max(1, max([tangent second third]));
within = all(numericalTangent <= tangent + comparisonTolerance) && ...
    all(numericalSecond <= second + comparisonTolerance) && ...
    all(numericalThird <= third + comparisonTolerance);
if ~within
    error("planAzElMotion:CertificateViolation", ...
        "A derivative sample exceeded the continuous certificate.");
end
bounds = derivativeBoundsTemplate();
bounds.RunIndex = index;
bounds.StartArcLength_deg = startS_deg;
bounds.EndArcLength_deg = endS_deg;
bounds.TangentByAxis = tangent;
bounds.SecondDerivativeByAxis_deg_inv = second;
bounds.ThirdDerivativeByAxis_deg_inv2 = third;
bounds.CertifiedTangentByAxis = tangent;
bounds.CertifiedSecondDerivativeByAxis_deg_inv = second;
bounds.CertifiedThirdDerivativeByAxis_deg_inv2 = third;
bounds.NumericalTangentByAxis = numericalTangent;
bounds.NumericalSecondDerivativeByAxis_deg_inv = numericalSecond;
bounds.NumericalThirdDerivativeByAxis_deg_inv2 = numericalThird;
bounds.SampleCount = 17;
bounds.CertificateSubdivisionCount = subdivisionCount;
bounds.CertificateFallbackCount = fallbackCount;
bounds.SampledBoundsWithinCertificate = within;
bounds.Method = method;
end
function [maximumSpeed_deg_s, maximumAcceleration_deg_s2, ...
        maximumJerk_deg_s3] = scalarLimits(bounds, limits, tolerance)
%% Section 0: Header & Readme
% Derive conservative scalar budgets from coupled Cartesian constraints.
tangent = bounds.CertifiedTangentByAxis;
second = bounds.CertifiedSecondDerivativeByAxis_deg_inv;
third = bounds.CertifiedThirdDerivativeByAxis_deg_inv2;
maximumSpeed_deg_s = Inf;
maximumAcceleration_deg_s2 = Inf;
maximumJerk_deg_s3 = Inf;
hasCurvature = any(second > tolerance) || any(third > tolerance);
for axisIndex = 1:2
    if tangent(axisIndex) > tolerance
        maximumSpeed_deg_s = min(maximumSpeed_deg_s, ...
            limits.maxVelocity_deg_s(axisIndex) / tangent(axisIndex));
        accelerationFraction = 1;
        jerkFraction = 1;
        if hasCurvature && any(isfinite(limits.maxJerk_deg_s3))
            accelerationFraction = 0.55;
            jerkFraction = 1 / 3;
        end
        maximumAcceleration_deg_s2 = min( maximumAcceleration_deg_s2, accelerationFraction * ...
            limits.maxAcceleration_deg_s2(axisIndex) / tangent(axisIndex));
        maximumJerk_deg_s3 = min(maximumJerk_deg_s3, ...
            jerkFraction * limits.maxJerk_deg_s3(axisIndex) / tangent(axisIndex));
    end
    if hasCurvature && second(axisIndex) > tolerance
        curvatureFraction = 1;
        if any(isfinite(limits.maxJerk_deg_s3))
            curvatureFraction = 0.45;
        end
        maximumSpeed_deg_s = min(maximumSpeed_deg_s, sqrt( ...
            curvatureFraction * limits.maxAcceleration_deg_s2(axisIndex) / second(axisIndex)));
    end
    if hasCurvature && third(axisIndex) > tolerance
        maximumSpeed_deg_s = min(maximumSpeed_deg_s, nthroot( ...
            limits.maxJerk_deg_s3(axisIndex) / (3 * third(axisIndex)), 3));
    end
end
for axisIndex = 1:2
    if hasCurvature && second(axisIndex) > tolerance
        crossLimit_deg_s2 = limits.maxJerk_deg_s3(axisIndex) / ...
            (9 * second(axisIndex) * max(maximumSpeed_deg_s, eps));
        maximumAcceleration_deg_s2 = min( maximumAcceleration_deg_s2, crossLimit_deg_s2);
    end
end
if ~all(isfinite([maximumSpeed_deg_s, maximumAcceleration_deg_s2])) || ...
        isnan(maximumJerk_deg_s3) || any([maximumSpeed_deg_s, maximumAcceleration_deg_s2, ...
        maximumJerk_deg_s3] <= 0)
    error("planAzElMotion:InvalidScalarLimits", ...
        "The path produced unusable scalar retiming limits.");
end
end
function [speed_deg_s, feasible, message] = accelerationNodeSpeeds( ...
        length_deg, runSpeedCap_deg_s, bounds, accelerationLimit_deg_s2, ...
        initialSpeed_deg_s, goalSpeed_deg_s, mandatoryStopNode, tolerance)
%% Section 0: Header & Readme
% Carry maximum speed while reserving curvature-dependent acceleration.
runCount = numel(length_deg);
cap_deg_s = [runSpeedCap_deg_s(1); min(runSpeedCap_deg_s(1:end - 1), ...
    runSpeedCap_deg_s(2:end)); runSpeedCap_deg_s(end)];
cap_deg_s(mandatoryStopNode) = 0;
speedTolerance_deg_s = tolerance * max(1, max(runSpeedCap_deg_s));
if initialSpeed_deg_s > cap_deg_s(1) + speedTolerance_deg_s || ...
        goalSpeed_deg_s > cap_deg_s(end) + speedTolerance_deg_s
    speed_deg_s = zeros(runCount + 1, 1);
    feasible = false;
    message = "An endpoint speed exceeds its local path limit.";
    return;
end
speedSquared_deg2_s2 = cap_deg_s.^2;
speedSquared_deg2_s2([1 end]) = [initialSpeed_deg_s^2; goalSpeed_deg_s^2];
for passIndex = 1:max(8, 2 * (runCount + 1))
    previous = speedSquared_deg2_s2;
    for runIndex = 1:runCount
        speedSquared_deg2_s2(runIndex + 1) = min( speedSquared_deg2_s2(runIndex + 1), ...
            accelerationReachableSquared(speedSquared_deg2_s2(runIndex), ...
            cap_deg_s(runIndex + 1)^2, length_deg(runIndex), ...
            bounds(runIndex), accelerationLimit_deg_s2, tolerance));
    end
    for runIndex = runCount:-1:1
        speedSquared_deg2_s2(runIndex) = min( speedSquared_deg2_s2(runIndex), ...
            accelerationReachableSquared( ...
            speedSquared_deg2_s2(runIndex + 1), cap_deg_s(runIndex)^2, ...
            length_deg(runIndex), bounds(runIndex), accelerationLimit_deg_s2, tolerance));
    end
    speedSquared_deg2_s2(mandatoryStopNode) = 0;
    speedSquared_deg2_s2([1 end]) = [initialSpeed_deg_s^2; goalSpeed_deg_s^2];
    if max(abs(speedSquared_deg2_s2 - previous)) <= speedTolerance_deg_s^2
        break;
    end
end
feasible = true;
for runIndex = 1:runCount
    maximumSquaredSpeed = max(speedSquared_deg2_s2(runIndex:runIndex + 1));
    allowance_deg_s2 = scalarAccelerationAllowance(maximumSquaredSpeed, ...
        bounds(runIndex), accelerationLimit_deg_s2, tolerance);
    feasible = feasible && abs(diff( speedSquared_deg2_s2(runIndex:runIndex + 1))) <= ...
        2 * allowance_deg_s2 * length_deg(runIndex) + tolerance;
end
speed_deg_s = sqrt(max(0, speedSquared_deg2_s2));
message = "Acceleration boundary speeds are not mutually reachable.";
end
function reachableSquaredSpeed = accelerationReachableSquared( ...
        startSquaredSpeed, capSquaredSpeed, distance_deg, bounds, ...
        accelerationLimit_deg_s2, tolerance)
%% Section 0: Header & Readme
% Solve one monotone squared-speed reachability step by bisection.
if capSquaredSpeed <= startSquaredSpeed
    reachableSquaredSpeed = capSquaredSpeed;
    return;
end
lower = startSquaredSpeed;
upper = capSquaredSpeed;
for iteration = 1:60
    middle = 0.5 * (lower + upper);
    allowance_deg_s2 = scalarAccelerationAllowance( ...
        middle, bounds, accelerationLimit_deg_s2, tolerance);
    if middle - startSquaredSpeed <= 2 * allowance_deg_s2 * distance_deg
        lower = middle;
    else
        upper = middle;
    end
end
reachableSquaredSpeed = lower;
end
function acceleration_deg_s2 = scalarAccelerationAllowance( ...
        squaredSpeed_deg2_s2, bounds, accelerationLimit_deg_s2, tolerance)
%% Section 0: Header & Readme
% Intersect certified per-axis acceleration intervals at one path speed.
tangent = bounds.CertifiedTangentByAxis;
second = bounds.CertifiedSecondDerivativeByAxis_deg_inv;
acceleration_deg_s2 = Inf;
for axisIndex = 1:2
    if ~isfinite(accelerationLimit_deg_s2(axisIndex))
        continue;
    end
    remaining_deg_s2 = accelerationLimit_deg_s2(axisIndex) - ...
        second(axisIndex) * squaredSpeed_deg2_s2;
    if remaining_deg_s2 < -tolerance
        acceleration_deg_s2 = 0;
        return;
    elseif tangent(axisIndex) > tolerance
        acceleration_deg_s2 = min(acceleration_deg_s2, ...
            max(0, remaining_deg_s2) / tangent(axisIndex));
    end
end
end
function [nodeSpeed_deg_s, feasible, message] = reachableNodeSpeeds( ...
        length_deg, maximumSpeed_deg_s, maximumAcceleration_deg_s2, ...
        maximumJerk_deg_s3, initialSpeed_deg_s, goalSpeed_deg_s, mandatoryStopNode, tolerance)
%% Section 0: Header & Readme
% Carry the largest mutually reachable zero-acceleration boundary speeds.
count = numel(length_deg);
cap_deg_s = Inf(count + 1, 1);
cap_deg_s(1) = maximumSpeed_deg_s(1);
cap_deg_s(end) = maximumSpeed_deg_s(end);
for index = 2:count
    cap_deg_s(index) = min( maximumSpeed_deg_s(index - 1), maximumSpeed_deg_s(index));
end
cap_deg_s(mandatoryStopNode) = 0;
speedTolerance_deg_s = tolerance * max(1, max(maximumSpeed_deg_s));
if initialSpeed_deg_s > cap_deg_s(1) + speedTolerance_deg_s || ...
        goalSpeed_deg_s > cap_deg_s(end) + speedTolerance_deg_s
    nodeSpeed_deg_s = zeros(count + 1, 1);
    feasible = false;
    message = "An endpoint speed exceeds its local path limit.";
    return;
end
nodeSpeed_deg_s = cap_deg_s;
nodeSpeed_deg_s(1) = initialSpeed_deg_s;
nodeSpeed_deg_s(end) = goalSpeed_deg_s;
for passIndex = 1:max(8, 2 * (count + 1))
    previous = nodeSpeed_deg_s;
    for index = 1:count
        reachable = reachableSpeed(nodeSpeed_deg_s(index), ...
            length_deg(index), maximumSpeed_deg_s(index), maximumAcceleration_deg_s2(index), ...
            maximumJerk_deg_s3(index), tolerance);
        if index == count && goalSpeed_deg_s > reachable + speedTolerance_deg_s
            feasible = false;
            message = "The goal speed is not forward reachable.";
            return;
        elseif index < count
            nodeSpeed_deg_s(index + 1) = min( nodeSpeed_deg_s(index + 1), reachable);
        end
    end
    for index = count:-1:1
        reachable = reachableSpeed(nodeSpeed_deg_s(index + 1), ...
            length_deg(index), maximumSpeed_deg_s(index), maximumAcceleration_deg_s2(index), ...
            maximumJerk_deg_s3(index), tolerance);
        if index == 1 && initialSpeed_deg_s > reachable + speedTolerance_deg_s
            feasible = false;
            message = "The initial speed cannot brake to path limits.";
            return;
        elseif index > 1
            nodeSpeed_deg_s(index) = min(nodeSpeed_deg_s(index), reachable);
        end
    end
    nodeSpeed_deg_s(mandatoryStopNode) = 0;
    nodeSpeed_deg_s(1) = initialSpeed_deg_s;
    nodeSpeed_deg_s(end) = goalSpeed_deg_s;
    if max(abs(nodeSpeed_deg_s - previous)) <= speedTolerance_deg_s
        break;
    end
end
feasible = true;
message = "";
end
function speed_deg_s = reachableSpeed( boundarySpeed_deg_s, distance_deg, speedLimit_deg_s, ...
        accelerationLimit_deg_s2, jerkLimit_deg_s3, tolerance)
%% Section 0: Header & Readme
% Find the greatest zero-acceleration speed reachable through one run.
boundarySpeed_deg_s = min(max(boundarySpeed_deg_s, 0), speedLimit_deg_s);
if transitionDistance(boundarySpeed_deg_s, speedLimit_deg_s, ...
        accelerationLimit_deg_s2, jerkLimit_deg_s3) <= ...
        distance_deg + tolerance * max(1, distance_deg)
    speed_deg_s = speedLimit_deg_s;
    return;
end
lower = boundarySpeed_deg_s;
upper = speedLimit_deg_s;
for iteration = 1:70
    middle = 0.5 * (lower + upper);
    if transitionDistance(boundarySpeed_deg_s, middle, ...
            accelerationLimit_deg_s2, jerkLimit_deg_s3) <= distance_deg
        lower = middle;
    else
        upper = middle;
    end
end
speed_deg_s = lower;
end
function [profile, feasible, message] = minimumTimeProfile( ...
        length_deg, startSpeed_deg_s, endSpeed_deg_s, ...
        speedLimit_deg_s, accelerationLimit_deg_s2, jerkLimit_deg_s3, tolerance)
%% Section 0: Header & Readme
% Build the minimum-time seven-phase scalar S-curve for one path run.
profile = profileTemplate();
profile.Length_deg = length_deg;
profile.StartSpeed_deg_s = startSpeed_deg_s;
profile.EndSpeed_deg_s = endSpeed_deg_s;
if min([length_deg, speedLimit_deg_s, accelerationLimit_deg_s2, jerkLimit_deg_s3]) <= 0 || ...
        max(startSpeed_deg_s, endSpeed_deg_s) > speedLimit_deg_s + tolerance
    feasible = false;
    message = "Invalid scalar S-curve inputs.";
    return;
end
minimumDistance_deg = transitionDistance( startSpeed_deg_s, endSpeed_deg_s, ...
    accelerationLimit_deg_s2, jerkLimit_deg_s3);
if minimumDistance_deg > length_deg + tolerance * max(1, length_deg)
    feasible = false;
    message = "The run is too short for its boundary speeds.";
    return;
end
peakSpeed_deg_s = speedLimit_deg_s;
distanceAtLimit_deg = transitionDistance(startSpeed_deg_s, ...
    peakSpeed_deg_s, accelerationLimit_deg_s2, jerkLimit_deg_s3) + ...
    transitionDistance(peakSpeed_deg_s, endSpeed_deg_s, ...
    accelerationLimit_deg_s2, jerkLimit_deg_s3);
if distanceAtLimit_deg > length_deg
    lower = max(startSpeed_deg_s, endSpeed_deg_s);
    upper = speedLimit_deg_s;
    for iteration = 1:80
        middle = 0.5 * (lower + upper);
        distance = transitionDistance(startSpeed_deg_s, middle, ...
            accelerationLimit_deg_s2, jerkLimit_deg_s3) + ...
            transitionDistance(middle, endSpeed_deg_s, ...
            accelerationLimit_deg_s2, jerkLimit_deg_s3);
        if distance <= length_deg
            lower = middle;
        else
            upper = middle;
        end
    end
    peakSpeed_deg_s = lower;
end
[firstDuration_s, firstJerk_deg_s3] = transitionPhases( startSpeed_deg_s, peakSpeed_deg_s, ...
    accelerationLimit_deg_s2, jerkLimit_deg_s3);
[lastDuration_s, lastJerk_deg_s3] = transitionPhases( peakSpeed_deg_s, endSpeed_deg_s, ...
    accelerationLimit_deg_s2, jerkLimit_deg_s3);
transitionDistance_deg = transitionDistance(startSpeed_deg_s, ...
    peakSpeed_deg_s, accelerationLimit_deg_s2, jerkLimit_deg_s3) + ...
    transitionDistance(peakSpeed_deg_s, endSpeed_deg_s, ...
    accelerationLimit_deg_s2, jerkLimit_deg_s3);
cruiseDuration_s = max(0, length_deg - transitionDistance_deg) / max(peakSpeed_deg_s, eps);
phaseDuration_s = [firstDuration_s; cruiseDuration_s; lastDuration_s];
phaseJerk_deg_s3 = [firstJerk_deg_s3; 0; lastJerk_deg_s3];
phaseStartTime_s = [0; cumsum(phaseDuration_s(1:end - 1))];
phaseStartPosition_deg = zeros(7, 1);
phaseStartSpeed_deg_s = zeros(7, 1);
phaseStartAcceleration_deg_s2 = zeros(7, 1);
phaseStartSpeed_deg_s(1) = startSpeed_deg_s;
for index = 1:6
    duration_s = phaseDuration_s(index);
    jerk_deg_s3 = phaseJerk_deg_s3(index);
    phaseStartPosition_deg(index + 1) = phaseStartPosition_deg(index) + ...
        phaseStartSpeed_deg_s(index) * duration_s + ...
        0.5 * phaseStartAcceleration_deg_s2(index) * duration_s^2 + ...
        jerk_deg_s3 * duration_s^3 / 6;
    phaseStartSpeed_deg_s(index + 1) = phaseStartSpeed_deg_s(index) + ...
        phaseStartAcceleration_deg_s2(index) * duration_s + 0.5 * jerk_deg_s3 * duration_s^2;
    phaseStartAcceleration_deg_s2(index + 1) = ...
        phaseStartAcceleration_deg_s2(index) + jerk_deg_s3 * duration_s;
end
profile.Duration_s = sum(phaseDuration_s);
profile.PeakSpeed_deg_s = peakSpeed_deg_s;
profile.PeakAcceleration_deg_s2 = max(abs([ phaseStartAcceleration_deg_s2; ...
    phaseStartAcceleration_deg_s2 + phaseJerk_deg_s3 .* phaseDuration_s]));
profile.PeakJerk_deg_s3 = max(abs(phaseJerk_deg_s3));
profile.PhaseDuration_s = phaseDuration_s;
profile.PhaseJerk_deg_s3 = phaseJerk_deg_s3;
profile.PhaseStartTime_s = phaseStartTime_s;
profile.PhaseStartPosition_deg = phaseStartPosition_deg;
profile.PhaseStartSpeed_deg_s = phaseStartSpeed_deg_s;
profile.PhaseStartAcceleration_deg_s2 = phaseStartAcceleration_deg_s2;
profile.TangentialAcceleration_deg_s2 = 0;
feasible = true;
message = "";
end
function profile = profileTemplate()
%% Section 0: Header & Readme
% Return one stable scalar S-curve profile record.
profile = struct( "PrimitiveType", "", "StartPosition_deg", zeros(1, 2), ...
    "EndPosition_deg", zeros(1, 2), "Length_deg", 0, ...
    "StartArcLength_deg", 0, "EndArcLength_deg", 0, ...
    "StartSpeed_deg_s", 0, "EndSpeed_deg_s", 0, ...
    "Duration_s", 0, "StartTime_s", 0, "EndTime_s", 0, ...
    "MaxSpeed_deg_s", 0, "MaxAcceleration_deg_s2", 0, ...
    "MaxJerk_deg_s3", 0, "PeakSpeed_deg_s", 0, ...
    "PeakAcceleration_deg_s2", 0, "PeakJerk_deg_s3", 0, ...
    "PeakVelocityByAxis_deg_s", zeros(1, 2), "PeakAccelerationByAxis_deg_s2", zeros(1, 2), ...
    "PeakJerkByAxis_deg_s3", zeros(1, 2), "PhaseDuration_s", zeros(7, 1), ...
    "PhaseJerk_deg_s3", zeros(7, 1), "PhaseStartTime_s", zeros(7, 1), ...
    "PhaseStartPosition_deg", zeros(7, 1), "PhaseStartSpeed_deg_s", zeros(7, 1), ...
    "PhaseStartAcceleration_deg_s2", zeros(7, 1), "TangentialAcceleration_deg_s2", 0);
end
function distance_deg = transitionDistance( firstSpeed_deg_s, secondSpeed_deg_s, ...
        accelerationLimit_deg_s2, jerkLimit_deg_s3)
%% Section 0: Header & Readme
% Return the exact distance of a zero-acceleration S-curve transition.
[duration_s, ~] = transitionPhases(firstSpeed_deg_s, ...
    secondSpeed_deg_s, accelerationLimit_deg_s2, jerkLimit_deg_s3);
distance_deg = 0.5 * (firstSpeed_deg_s + secondSpeed_deg_s) * sum(duration_s);
end
function [duration_s, jerk_deg_s3] = transitionPhases( firstSpeed_deg_s, secondSpeed_deg_s, ...
        accelerationLimit_deg_s2, jerkLimit_deg_s3)
%% Section 0: Header & Readme
% Return three jerk phases for one monotone zero-acceleration transition.
velocityChange_deg_s = secondSpeed_deg_s - firstSpeed_deg_s;
direction = sign(velocityChange_deg_s);
changeMagnitude_deg_s = abs(velocityChange_deg_s);
if changeMagnitude_deg_s == 0
    duration_s = zeros(3, 1);
    jerk_deg_s3 = zeros(3, 1);
    return;
end
jerkRampDuration_s = min(sqrt(changeMagnitude_deg_s / jerkLimit_deg_s3), ...
    accelerationLimit_deg_s2 / jerkLimit_deg_s3);
constantAccelerationDuration_s = max(0, ...
    changeMagnitude_deg_s / (jerkLimit_deg_s3 * jerkRampDuration_s) - jerkRampDuration_s);
duration_s = [jerkRampDuration_s; constantAccelerationDuration_s; jerkRampDuration_s];
jerk_deg_s3 = direction * [jerkLimit_deg_s3; 0; -jerkLimit_deg_s3];
end
function [position_deg, speed_deg_s, acceleration_deg_s2, ...
        jerk_deg_s3] = sampleProfile(time_s, profile)
%% Section 0: Header & Readme
% Evaluate one analytic S-curve at local times.
time_s = min(max(double(time_s(:)), 0), profile.Duration_s);
count = numel(time_s);
position_deg = zeros(count, 1);
speed_deg_s = zeros(count, 1);
acceleration_deg_s2 = zeros(count, 1);
jerk_deg_s3 = zeros(count, 1);
for index = 1:7
    if index < 7
        belongs = time_s >= profile.PhaseStartTime_s(index) & ...
            time_s < profile.PhaseStartTime_s(index + 1);
    else
        belongs = time_s >= profile.PhaseStartTime_s(index);
    end
    localTime_s = time_s(belongs) - profile.PhaseStartTime_s(index);
    jerk = profile.PhaseJerk_deg_s3(index);
    position_deg(belongs) = profile.PhaseStartPosition_deg(index) + ...
        profile.PhaseStartSpeed_deg_s(index) .* localTime_s + ...
        0.5 * profile.PhaseStartAcceleration_deg_s2(index) .* ...
        localTime_s.^2 + jerk .* localTime_s.^3 / 6;
    speed_deg_s(belongs) = profile.PhaseStartSpeed_deg_s(index) + ...
        profile.PhaseStartAcceleration_deg_s2(index) .* localTime_s + ...
        0.5 * jerk .* localTime_s.^2;
    acceleration_deg_s2(belongs) = profile.PhaseStartAcceleration_deg_s2(index) + ...
        jerk .* localTime_s;
    jerk_deg_s3(belongs) = jerk;
end
if isinf(profile.MaxJerk_deg_s3)
    jerk_deg_s3(:) = NaN;
end
atEnd = time_s == profile.Duration_s;
position_deg(atEnd) = profile.Length_deg;
speed_deg_s(atEnd) = profile.EndSpeed_deg_s;
acceleration_deg_s2(atEnd) = 0;
end
function [time_s, arcLength_deg, speed_deg_s, acceleration_deg_s2, ...
        jerk_deg_s3] = sampleProfiles( profiles, initialTime_s, waitDuration_s, sampleTime_s)
%% Section 0: Header & Readme
% Sample all scalar profiles while preserving strict absolute timestamps.
time_s = zeros(0, 1);
arcLength_deg = zeros(0, 1);
speed_deg_s = zeros(0, 1);
acceleration_deg_s2 = zeros(0, 1);
jerk_deg_s3 = zeros(0, 1);
if waitDuration_s > 1e-12
    localTime_s = regularTimes(waitDuration_s, sampleTime_s, []);
    time_s = initialTime_s + localTime_s;
    arcLength_deg = zeros(size(time_s));
    speed_deg_s = zeros(size(time_s));
    acceleration_deg_s2 = zeros(size(time_s));
    jerk_deg_s3 = zeros(size(time_s));
end
for index = 1:numel(profiles)
    profile = profiles(index);
    phaseEnd_s = cumsum(profile.PhaseDuration_s);
    phaseStart_s = [0; phaseEnd_s(1:end - 1)];
    events_s = unique([phaseStart_s; phaseStart_s + 0.5 * profile.PhaseDuration_s; phaseEnd_s]);
    localTime_s = regularTimes( profile.Duration_s, sampleTime_s, events_s);
    [distance_deg, velocity_deg_s, accel_deg_s2, localJerk_deg_s3] = ...
        sampleProfile(localTime_s, profile);
    if any(distance_deg < -1e-10) || any(distance_deg > profile.Length_deg + 1e-10)
        error("planAzElMotion:ProfileDistanceOutsideRun", ...
            "Profile %d sampled [%.17g, %.17g] deg for length %.17g deg.", ...
            index, min(distance_deg), max(distance_deg), profile.Length_deg);
    end
    absoluteTime_s = profile.StartTime_s + localTime_s;
    previousTime_s = [];
    if ~isempty(time_s)
        previousTime_s = time_s(end);
    end
    keep = strictTimeMask(absoluteTime_s, previousTime_s);
    time_s = [time_s; absoluteTime_s(keep)]; %#ok<AGROW>
    arcLength_deg = [arcLength_deg; ...
        profile.StartArcLength_deg + distance_deg(keep)]; %#ok<AGROW>
    speed_deg_s = [speed_deg_s; velocity_deg_s(keep)]; %#ok<AGROW>
    acceleration_deg_s2 = [acceleration_deg_s2; accel_deg_s2(keep)]; %#ok<AGROW>
    jerk_deg_s3 = [jerk_deg_s3; localJerk_deg_s3(keep)]; %#ok<AGROW>
end
if numel(time_s) < 2 || any(diff(time_s) <= 0)
    error("planAzElMotion:NonIncreasingTime", ...
        "Profile assembly must produce strictly increasing time.");
end
end
function time_s = regularTimes(duration_s, sampleTime_s, events_s)
%% Section 0: Header & Readme
% Combine regular samples and exact events on one local duration.
time_s = unique([0; (0:sampleTime_s:duration_s).'; events_s(:); duration_s]);
time_s = time_s(time_s >= 0 & time_s <= duration_s);
end
function keep = strictTimeMask(time_s, previousTime_s)
%% Section 0: Header & Readme
% Keep the last state for collapsed timestamps and remove prior joins.
time_s = time_s(:);
keep = [diff(time_s) > 0; true];
if ~isempty(previousTime_s)
    keep = keep & time_s > previousTime_s;
end
end
function [velocityBound_deg_s, accelerationBound_deg_s2, ...
        jerkBound_deg_s3] = cartesianBounds(bounds, profile)
%% Section 0: Header & Readme
% Map scalar S-curve peaks through the certified path derivative envelope.
tangent = bounds.CertifiedTangentByAxis;
second = bounds.CertifiedSecondDerivativeByAxis_deg_inv;
third = bounds.CertifiedThirdDerivativeByAxis_deg_inv2;
velocityBound_deg_s = tangent * profile.PeakSpeed_deg_s;
accelerationBound_deg_s2 = tangent * profile.PeakAcceleration_deg_s2 + ...
    second * profile.PeakSpeed_deg_s^2;
jerkBound_deg_s3 = tangent * profile.PeakJerk_deg_s3 + ...
    3 * second * profile.PeakSpeed_deg_s * profile.PeakAcceleration_deg_s2 + ...
    third * profile.PeakSpeed_deg_s^3;
end
function smoothPath = emptySmoothPath(route_deg)
%% Section 0: Header & Readme
% Return the stable empty smooth-path schema for candidate failures.
smoothPath = struct( "Success", false, "Message", "No smooth path was produced.", ...
    "OriginalPathPosition_deg", route_deg, "Primitives", struct([]), "TotalLength_deg", 0, ...
    "SampleArcLength_deg", zeros(0, 1), "position_deg", zeros(0, 2), "tangent", zeros(0, 2), ...
    "secondDerivative_deg_inv", zeros(0, 2), "thirdDerivative_deg_inv2", zeros(0, 2), ...
    "curvature_deg_inv", zeros(0, 1), "PrimitiveIndex", zeros(0, 1), ...
    "PrimitiveType", strings(0, 1), "MandatoryStop", false(0, 1), ...
    "MandatoryStopArcLength_deg", zeros(0, 1), ...
    "CornerDiagnostics", struct([]), "RoundedCornerCount", 0, ...
    "MandatoryStopCount", 0, "Options", struct());
end
function timedPath = emptyTimedPath(limits, options, message)
%% Section 0: Header & Readme
% Return the stable timed-path failure schema without hiding diagnostics.
diagnostics = struct( "PeakVelocity_deg_s", [NaN NaN], ...
    "PeakAcceleration_deg_s2", [NaN NaN], "PeakJerk_deg_s3", [NaN NaN], ...
    "VelocityMargin_deg_s", [NaN NaN], "AccelerationMargin_deg_s2", [NaN NaN], ...
    "JerkMargin_deg_s3", [NaN NaN], ...
    "VelocitySatisfied", false, "AccelerationSatisfied", false, "JerkSatisfied", false, ...
    "JerkConstrained", any(isfinite(limits.maxJerk_deg_s3)), "FiniteJerkCertified", false, ...
    "FiniteJerkNumericallyVerified", false, "ContinuousJerkCertified", false, ...
    "G3JoinCount", 0, "MinimumG3JoinSpeed_deg_s", NaN, ...
    "VelocityCarriedAcrossG3Joins", false, "JoinContinuityOrder", "G3", ...
    "GeometryDerivativeBounds", repmat( derivativeBoundsTemplate(), 0, 1), ...
    "SpatiallyVaryingLimits", true, "SpatialRetimingCellCount", 0, ...
    "ExecutedMotionProfileCount", 0, "MandatoryStopCount", 0, ...
    "MandatoryStopArcLength_deg", zeros(0, 1), "CurvatureDiscontinuityStopCount", 0, ...
    "RoundedVelocityCarried", false, "MinimumArcSpeed_deg_s", NaN, "Satisfied", false);
timedPath = struct( "Success", false, "Message", string(message), ...
    "time_s", zeros(0, 1), "position_deg", zeros(0, 2), "velocity_deg_s", zeros(0, 2), ...
    "acceleration_deg_s2", zeros(0, 2), "jerk_deg_s3", zeros(0, 2), ...
    "PathPosition_deg", zeros(0, 2), "WaypointTime_s", zeros(0, 1), ...
    "MotionStartTime_s", NaN, "WaitDuration_s", NaN, "MinimumMotionDuration_s", NaN, ...
    "GoalLineInterceptTime_s", NaN, "SegmentProfiles", repmat(profileTemplate(), 0, 1), ...
    "Limits", limits, "Options", struct("GoalTimeMode", options.GoalTimeMode, ...
        "SampleTime_s", options.SampleTime_s), ...
    "ConstraintDiagnostics", diagnostics, "SmoothPath", struct(), ...
    "CurveArcLength_deg", zeros(0, 1), "CurveNodeTime_s", zeros(0, 1), ...
    "CurveSpeed_deg_s", zeros(0, 1), "CurveSpeedSquared_deg2_s2", zeros(0, 1), ...
    "CurveTangentialAcceleration_deg_s2", zeros(0, 1), ...
    "CurveTangentialJerk_deg_s3", zeros(0, 1), "SampleArcLength_deg", zeros(0, 1), ...
    "SampleSpeed_deg_s", zeros(0, 1), "SampleTangentialAcceleration_deg_s2", zeros(0, 1), ...
    "SampleTangentialJerk_deg_s3", zeros(0, 1), "CurvatureDiscontinuityStopCount", 0, ...
    "RetimerType", "certifiedAnalyticSpatialJerk", "MotionType", "velocityCarrying");
end
function validation = validatePlan(planningSucceeded, endpointBlocked, ...
        timedPath, blocked, goalState, limits, options)
%% Section 0: Header & Readme
% Independently check the complete returned state and its certified bounds.
hasMotion = timedPath.Success && numel(timedPath.time_s) >= 2;
finiteJerkAxis = isfinite(limits.maxJerk_deg_s3);
finiteJerkHistory = ~any(finiteJerkAxis) || ...
    all(isfinite(timedPath.jerk_deg_s3(:, finiteJerkAxis)), "all");
finiteState = hasMotion && all(isfinite(timedPath.time_s)) && ...
    all(isfinite(timedPath.position_deg(:))) && ...
    all(isfinite(timedPath.velocity_deg_s(:))) && ...
    all(isfinite(timedPath.acceleration_deg_s2(:))) && finiteJerkHistory;
strictTime = hasMotion && all(diff(timedPath.time_s) > 0);
endpointMatched = hasMotion && ...
    norm(timedPath.position_deg(end, :) - goalState.position_deg) <= 1e-7 && ...
    norm(timedPath.velocity_deg_s(end, :) - goalState.velocity_deg_s) <= 1e-7 && ...
    norm(timedPath.acceleration_deg_s2(end, :) - goalState.acceleration_deg_s2) <= 1e-7;
goalTimeSatisfied = hasMotion && timedPath.time_s(end) <= goalState.time_s + 1e-7;
if options.GoalTimeMode == "fixedarrival"
    goalTimeSatisfied = hasMotion && abs(timedPath.time_s(end) - goalState.time_s) <= 1e-7;
end
sampleVelocitySatisfied = hasMotion && all(max( abs(timedPath.velocity_deg_s), [], 1) <= ...
    limits.maxVelocity_deg_s + 1e-7);
sampleAccelerationSatisfied = hasMotion && all(max( ...
    abs(timedPath.acceleration_deg_s2), [], 1) <= limits.maxAcceleration_deg_s2 + 1e-7);
sampleJerkSatisfied = hasMotion && (~any(finiteJerkAxis) || ...
    all(max(abs(timedPath.jerk_deg_s3(:, finiteJerkAxis)), [], 1) <= ...
    limits.maxJerk_deg_s3(finiteJerkAxis) + 1e-7));
certificateSatisfied = hasMotion && timedPath.ConstraintDiagnostics.Satisfied;
collisionFree = hasMotion && ~any(blocked);
passed = planningSucceeded && ~endpointBlocked && finiteState && ...
    strictTime && endpointMatched && goalTimeSatisfied && ...
    sampleVelocitySatisfied && sampleAccelerationSatisfied && ...
    sampleJerkSatisfied && certificateSatisfied && collisionFree;
failedChecks = strings(0, 1);
checks = [planningSucceeded, ~endpointBlocked, finiteState, strictTime, ...
    endpointMatched, goalTimeSatisfied, sampleVelocitySatisfied, ...
    sampleAccelerationSatisfied, sampleJerkSatisfied, certificateSatisfied, collisionFree];
names = ["planningSucceeded" "endpointsClear" "finiteState" ...
    "strictTime" "endpointMatched" "goalTimeSatisfied" ...
    "sampleVelocitySatisfied" "sampleAccelerationSatisfied" ...
    "sampleJerkSatisfied" "certificateSatisfied" "collisionFree"];
for index = 1:numel(checks)
    if ~checks(index)
        failedChecks(end + 1, 1) = names(index); %#ok<AGROW>
    end
end
message = "All independent checks passed.";
if ~passed
    message = "Failed checks: " + strjoin(failedChecks, ", ");
end
validation = struct( "Passed", passed, "Message", message, ...
    "PlanningSucceeded", planningSucceeded, "EndpointBlocked", endpointBlocked, ...
    "EndpointClear", ~endpointBlocked, "ProtectedGeometryClear", collisionFree, ...
    "AzimuthPolicySatisfied", hasMotion && ...
        routeWithinAzimuthPolicy(timedPath.position_deg, options), ...
    "GoalReached", endpointMatched, "ArrivalWithinGoalTime", goalTimeSatisfied, ...
    "FiniteState", finiteState, "TimeStrictlyIncreasing", strictTime, ...
    "EndpointMatched", endpointMatched, "GoalTimeSatisfied", goalTimeSatisfied, ...
    "VelocitySatisfied", sampleVelocitySatisfied, ...
    "AccelerationSatisfied", sampleAccelerationSatisfied, ...
    "JerkSatisfied", sampleJerkSatisfied, "VelocityWithinLimits", sampleVelocitySatisfied, ...
    "AccelerationWithinLimits", sampleAccelerationSatisfied, ...
    "JerkWithinLimits", sampleJerkSatisfied, "CertificateSatisfied", certificateSatisfied, ...
    "CollisionFree", collisionFree);
end
function [pathHandle, collisionHandles] = plotAttempt( ...
        spaceView, position_deg, time_s, blocked, protectedBlocked, success, options)
%% Section 0: Header & Readme
% Draw only the returned selected or best-attempt trajectory.
pathHandle = gobjects(0, 1);
collisionHandles = gobjects(0, 1);
if ~isfield(spaceView, "Axes") || ~isgraphics(spaceView.Axes) || isempty(position_deg)
    return;
end
color = [0.05 0.70 0.82];
label = "Selected timed path";
if ~success
    color = [0.95 0.45 0.05];
    label = "Best failed attempt";
    title(spaceView.Axes, options.Title + " | PLANNING FAILED");
end
pathHandle = plot3(spaceView.Axes, position_deg(:, 1), ...
    position_deg(:, 2), time_s, "-", "Color", color, "LineWidth", 4, "DisplayName", label);
if ~success && any(blocked)
    collisionHandles(end + 1, 1) = scatter3(spaceView.Axes, ...
        position_deg(blocked, 1), position_deg(blocked, 2), ...
        time_s(blocked), 28, [0.75 0 0], "filled", "DisplayName", "Obstacle collision");
end
protectedOnly = protectedBlocked & ~blocked;
if ~success && any(protectedOnly)
    collisionHandles(end + 1, 1) = scatter3(spaceView.Axes, ...
        position_deg(protectedOnly, 1), position_deg(protectedOnly, 2), ...
        time_s(protectedOnly), 22, [0.85 0.10 0.75], "filled", ...
        "DisplayName", "Protected-geometry conflict");
end
drawnow;
end
function plotRecord = plotKinematics(timedPath, limits, options)
%% Section 0: Header & Readme
% Plot returned position, velocity, acceleration, and jerk histories.
figureHandle = figure("Color", "w", "Visible", options.FigureVisible, ...
    "Name", options.Title + " kinematics");
layout = tiledlayout(figureHandle, 4, 1, "TileSpacing", "compact", "Padding", "compact");
data = {timedPath.position_deg, timedPath.velocity_deg_s, ...
    timedPath.acceleration_deg_s2, timedPath.jerk_deg_s3};
labels = ["Position (deg)" "Velocity (deg/s)" "Acceleration (deg/s^2)" "Jerk (deg/s^3)"];
axesHandles = gobjects(4, 1);
lineHandles = gobjects(2, 4);
for index = 1:4
    axesHandles(index) = nexttile(layout);
    lineHandles(:, index) = plot(axesHandles(index), timedPath.time_s, data{index}, ...
        "LineWidth", 1.2);
    grid(axesHandles(index), "on");
    ylabel(axesHandles(index), labels(index));
end
xlabel(axesHandles(end), "Time (s)");
legend(axesHandles(1), ["Azimuth" "Elevation"], "Location", "best");
plotRecord = struct( "Figure", figureHandle, "Layout", layout, "Axes", axesHandles, ...
    "PositionLines", lineHandles(:, 1), "VelocityLines", lineHandles(:, 2), ...
    "AccelerationLines", lineHandles(:, 3), "JerkLines", lineHandles(:, 4), ...
    "LimitLines", gobjects(0, 1), "Diagnostics", timedPath.ConstraintDiagnostics, ...
    "Options", struct("FigureVisible", options.FigureVisible, ...
        "ShowLimits", false, "Title", options.Title), "Limits", limits);
end
function graph = emptyGraph()
%% Section 0: Header & Readme
% Return the stable empty selected-visibility-graph schema.
graph = struct( "Success", false, "Message", "No visibility graph selected.", ...
    "Time_s", NaN, "NodePosition_deg", zeros(0, 2), ...
    "PathPosition_deg", zeros(0, 2), "PathCost_deg", Inf);
end
function log = makeProgressLog(success, message, candidateIndex)
%% Section 0: Header & Readme
% Publish a compact deterministic progress record for the final outcome.
status = "failed";
event = "PlanningFailed";
if success
    status = "complete";
    event = "PlanningSucceeded";
end
log = struct( "Sequence", 1, "Stage", "complete", "Event", event, ...
    "Status", status, "Message", message, "ObstacleIndex", NaN, ...
    "CandidateIndex", candidateIndex, "Details", struct());
end
