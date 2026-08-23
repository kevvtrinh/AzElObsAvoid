function motion = solveCorridorQuintic(obstacles, initialState, goalState, limits, route_deg, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = azElPlannerMethods.corridor.internal.motion.solveCorridorQuintic()
%   motion = azElPlannerMethods.corridor.internal.motion.solveCorridorQuintic(obstacles, ...
%       initialState, goalState, limits, route_deg, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Convert one ordered route into a smooth quintic trajectory constrained
%     to protected free-space supports. Route cleanup and corridor fitting are
%     proposal steps; the returned Success flag is set only after independent
%     trajectory and optional corridor-certificate validation.
%**************************************************************************
% INPUTS
%   - obstacles (canonical or prepared struct array), protected geometry.
%   - initialState, goalState (scalar structs), endpoint motion request.
%   - limits (scalar struct), workspace and derivative limits.
%   - route_deg (N-by-2 array), ordered [azimuth elevation] seed route.
%   - optionOverrides (scalar struct), partial corridor solver controls.
%**************************************************************************
% OUTPUTS
%   - motion (scalar struct), stable candidate or diagnosable failure.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s powers.
%**************************************************************************

%% Section 1: Validate Inputs & Resolve Options

% Normalize raw callers and reuse prepared obstacle histories from the public
% planner. Re-preparing them here would repeat expensive polygon work and could
% make different stages disagree about the same obstacle interpolation.
diagnosticTimer = tic;
defaults = struct( ...
    "RouteVertexCount", Inf, ...
    "ClearanceTarget_deg", 0.02, ...
    "MaximumControlPointOffset_deg", 3, ...
    "RouteExpansionFraction", 0.02, ...
    "SpanLengthExponent", 1.05, ...
    "RouteTau", zeros(0, 1), ...
    "RouteSamplingResolution_deg", 0.005, ...
    "EnvelopePadding_deg", 1e-6, ...
    "ObstacleEnvelopeBoundary_deg", zeros(0, 2), ...
    "RequireStaticCorridorCertificate", true, ...
    "EnableExactTraversal", false, ...
    "SampleTime_s", 0.05, "GoalTimeMode", "earliestArrival", "AllowAzimuthWrapping", false);
% A zero-input call returns defaults without constructing any planner state.
if nargin == 0
    motion = defaults;
    return;
end
% Reject an incomplete solver request before normalizing obstacles or reducing the route.
if nargin < 5
    error("solveCorridorQuintic:MissingInputs", "obstacles, endpoint states, limits, and route_deg are required.");
end
if nargin < 6 || isempty(optionOverrides)
    optionOverrides = struct();
end
[options, unknownNames] = azElPlannerMethods.corridor.internal.resolveOptions( defaults, optionOverrides);
if ~isempty(unknownNames)
    warning("solveCorridorQuintic:UnknownOptions", ...
        "Ignoring unknown fields: %s. No behavior changed.", strjoin(unknownNames, ", "));
end
options = validateOptions(options, size(route_deg, 1));
hasPreparedObstacles = isstruct(obstacles) && isfield(obstacles, "InternalPreparation");
% Public-style calls may provide raw obstacles; planner-internal calls reuse prepared geometry.
if ~hasPreparedObstacles
    obstacles = azElPlannerMethods.corridor.combineObstacles(obstacles);
    obstacles = azElPlannerMethods.corridor.internal.obstacles.prepareDynamic(obstacles);
end
azElPlannerMethods.corridor.internal.motion.buildQuinticSpline( ...
    route_deg, initialState, goalState, limits, struct( "AllowAzimuthWrapping", options.AllowAzimuthWrapping));
inputRoute_deg = double(route_deg);
% Static certificates use the conservative time-independent envelope; dynamic trials validate in time instead.
if options.RequireStaticCorridorCertificate
    corridorObstacles = obstacles;
else
    corridorObstacles = obstacles([]);
end
plannerOptions = azElPlannerMethods.corridor.plan();
plannerOptions.GoalTimeMode = options.GoalTimeMode;
plannerOptions.AllowAzimuthWrapping = options.AllowAzimuthWrapping;
expandedRoute_deg = expandRouteClearance( ...
    route_deg, corridorObstacles, initialState.time_s, options.ClearanceTarget_deg, options.RouteExpansionFraction);
requestedVertexCount = min(size(route_deg, 1), options.RouteVertexCount);
reducedRoute_deg = sampledVisibilitySubsequence( ...
    expandedRoute_deg, corridorObstacles, requestedVertexCount, ...
    initialState.time_s, options.RouteSamplingResolution_deg);
% Route reduction can prove that no valid ordered subsequence remains.
if isempty(reducedRoute_deg)
    % Return a normal failed candidate rather than throwing: the input contract
    % is valid, but this finite route representation cannot support a corridor.
    motion = azElPlannerMethods.corridor.internal.motion.buildQuinticSpline( route_deg, initialState, goalState, limits);
    motion.Success = false;
    motion.Message = "No sampled-clear ordered route subsequence exists " + "at the requested vertex count.";
    motion.TerminationReason = "noProtectedSubsequence";
    motion.OriginalRoute_deg = inputRoute_deg;
    motion.ExpandedRoute_deg = expandedRoute_deg;
    motion.ReducedRoute_deg = zeros(0, 2);
    motion.SeedCorridorBoundary_deg = zeros(0, 2);
    emptyCorridor = struct( ...
        "SegmentIndex", 0, "RegionIndex", 0, "Normal", [0 0], "BoundaryOffset_deg", 0, "Clearance_deg", 0);
    motion.SeedCorridor = repmat(emptyCorridor, 0, 1);
    motion.OptimizerOptions = options;
    motion.OptimizerDiagnostics = emptyDiagnostics( size(route_deg, 1), requestedVertexCount, toc(diagnosticTimer));
    return;
end
route_deg = reducedRoute_deg;
edgeLength_deg = vecnorm(diff(route_deg), 2, 2);
if ~isempty(options.RouteTau) && size(route_deg, 1) == size(inputRoute_deg, 1)
    spanWeights = diff(options.RouteTau);
else
    spanWeights = edgeLength_deg .^ options.SpanLengthExponent;
    positiveLength_deg = edgeLength_deg(edgeLength_deg > 0);
    % A route made only of holds still needs positive numerical span weights.
    if isempty(positiveLength_deg)
        spanWeights(:) = 1;
    else
        spanWeights = max(spanWeights, 1e-6 * min(positiveLength_deg) ^ options.SpanLengthExponent);
    end
end
spanWeights = spanWeights / mean(spanWeights);
% Interior route vertices provide two decision coordinates each. Endpoint
% position/velocity/acceleration are enforced separately by the spline builder.
interiorCount = size(route_deg, 1) - 2;
decisionCount = 2 * interiorCount;

%% Section 2: Build Complete Protected-Obstacle Corridor Records

% Each route segment receives convex exterior half-planes. These are linear in
% spline control points, which allows a small quadratic solve while retaining
% a later exact certificate against the complete protected geometry.
queryOptions = azElPlannerMethods.corridor.queryTimeObstacle();
envelopePadding_deg = max( options.EnvelopePadding_deg, 1000 * queryOptions.ClearanceTolerance_deg);
seed = struct( ...
    "position_deg", route_deg, "tau", linspace(0, 1, size(route_deg, 1)).', "CorridorBoundary_deg", zeros(0, 2));
% Dynamic planning does not claim a static envelope certificate that it never constructed.
if ~options.RequireStaticCorridorCertificate
    seed.CorridorBoundary_deg = zeros(0, 2);
elseif isempty(options.ObstacleEnvelopeBoundary_deg)
    seed.CorridorBoundary_deg = azElPlannerMethods.corridor.internal.obstacles.buildEnvelopeBoundary( obstacles, envelopePadding_deg);
else
    seed.CorridorBoundary_deg = options.ObstacleEnvelopeBoundary_deg;
end
baseOptions = struct( ...
    "SpanWeights", spanWeights, ...
    "SampleTime_s", options.SampleTime_s, ...
    "GoalTimeMode", options.GoalTimeMode, "AllowAzimuthWrapping", options.AllowAzimuthWrapping);
baseMotion = azElPlannerMethods.corridor.internal.motion.buildQuinticSpline( route_deg, initialState, goalState, limits, baseOptions);
corridor = azElPlannerMethods.corridor.internal.validation.buildSeedCorridor( seed, baseMotion.Polynomial.SegmentCount);

% Apply the requested clearance consistently to every segment/region support record.
for corridorIndex = 1:numel(corridor)
    corridor(corridorIndex).Clearance_deg = options.ClearanceTarget_deg;
end
baseInequality_deg = azElPlannerMethods.corridor.internal.validation.seedCorridorInequality( baseMotion.Polynomial, corridor);

%% Section 3: Derive & Solve The Affine Corridor System

% The base spline supplies the constant term. Rebuilding it with unit interior
% offsets supplies columns of the affine map from decision variables to every
% corridor inequality.
inequalityMatrix = zeros(numel(baseInequality_deg), decisionCount);
if ~isempty(corridor)
    endpointDerivative_deg = [initialState.velocity_deg_s, ...
        initialState.acceleration_deg_s2, goalState.velocity_deg_s, goalState.acceleration_deg_s2];
    canUseDirectAffineBasis = max(abs(endpointDerivative_deg)) <= 1e-10;
    % Reuse the exact B-spline affine map when endpoint derivatives do not require refitting controls.
    if canUseDirectAffineBasis
        controlPointCount = size(baseMotion.ControlPoint_deg, 1);
        affineBasisPolynomial = azElPlannerMethods.corridor.internal.motion.convertBsplineToPolynomial( ...
            eye(controlPointCount), 5, initialState.time_s, baseMotion.SpanDuration_s);
        corridorSegmentIndex = [corridor.SegmentIndex].';
        corridorNormal = vertcat(corridor.Normal);
        coefficientCount = size( affineBasisPolynomial.positionPower_deg, 3);
    end

    % Perturb each interior coordinate once to form its corridor-inequality column.
    for decisionIndex = 1:decisionCount
        interiorIndex = mod(decisionIndex - 1, interiorCount) + 1;
        axisIndex = floor((decisionIndex - 1) / interiorCount) + 1;
        if canUseDirectAffineBasis
            controlPointIndex = interiorIndex + 3;
            selectedBasis_deg = affineBasisPolynomial.positionPower_deg( corridorSegmentIndex, controlPointIndex, :);
            basisPower_deg = reshape( selectedBasis_deg, numel(corridor), coefficientCount);
            projectionPower_deg = corridorNormal(:, axisIndex) .* basisPower_deg;
            projectionBernstein_deg = azElPlannerMethods.corridor.internal.powerToBernstein( projectionPower_deg.');
            inequalityMatrix(:, decisionIndex) = -projectionBernstein_deg(:);
        else
            decisionOffset_deg = zeros(interiorCount, 2);
            decisionOffset_deg(interiorIndex, axisIndex) = 1;
            basisOptions = baseOptions;
            basisOptions.ControlPointOffsets_deg = decisionOffset_deg;
            basisMotion = azElPlannerMethods.corridor.internal.motion.buildQuinticSpline( ...
                route_deg, initialState, goalState, limits, basisOptions);
            basisInequality_deg = azElPlannerMethods.corridor.internal.validation.seedCorridorInequality( basisMotion.Polynomial, corridor);
            inequalityMatrix(:, decisionIndex) = basisInequality_deg - baseInequality_deg;
        end
    end
end
maximumOffset_deg = options.MaximumControlPointOffset_deg;
quadraticMatrix = eye(decisionCount);
linearVector = zeros(decisionCount, 1);
quadraticOptions = optimoptions( "quadprog", "Display", "off", "Algorithm", "active-set");
solveTimer = tic;
% Routes with no movable interior controls bypass the quadratic program cleanly.
if decisionCount == 0
    decision_deg = zeros(0, 1);
    exitFlag = double(all(baseInequality_deg <= 0));
    solverOutput = struct( "iterations", 0, "message", "The zero-variable corridor was checked directly.");
else
    optimizationInequalityMatrix = inequalityMatrix;
    optimizationInequalityBound = -baseInequality_deg;
    initialDecision_deg = zeros(decisionCount, 1);
    linearExitFlag = 1;
    % Project an infeasible initial guess into the linear corridor before asking the QP to improve it.
    if any(optimizationInequalityMatrix * initialDecision_deg > optimizationInequalityBound)
        linearOptions = optimoptions("linprog", "Display", "off");
        [initialDecision_deg, ~, linearExitFlag] = linprog( ...
            zeros(decisionCount, 1), ...
            optimizationInequalityMatrix, optimizationInequalityBound, ...
            [], [], -maximumOffset_deg * ones(decisionCount, 1), ...
            maximumOffset_deg * ones(decisionCount, 1), linearOptions);
    end
    if linearExitFlag <= 0
        decision_deg = zeros(decisionCount, 1);
        exitFlag = linearExitFlag;
        solverOutput = struct( "iterations", 0, "message", "The affine corridor is infeasible.");
    else
        [decision_deg, ~, exitFlag, solverOutput] = quadprog( ...
            quadraticMatrix, linearVector, ...
            optimizationInequalityMatrix, optimizationInequalityBound, ...
            [], [], -maximumOffset_deg * ones(decisionCount, 1), ...
            maximumOffset_deg * ones(decisionCount, 1), initialDecision_deg, quadraticOptions);
    end
end
exactTraversalDiagnostics = struct( ...
    "Attempted", false, "Accepted", false, ...
    "InitialScale", NaN, "FinalScale", NaN, "LinearSolveCount", 0, "ActiveConstraintCount", 0);
canOptimizeExactTraversal = exitFlag > 0 && ~isempty(corridor) && ...
    canUseDirectAffineBasis && ...
    options.GoalTimeMode == "earliestArrival" && ...
    options.EnableExactTraversal && decisionCount * baseMotion.Polynomial.SegmentCount <= 100;
if canOptimizeExactTraversal
    [trialDecision_deg, exactTraversalDiagnostics] = azElPlannerMethods.corridor.internal.motion.optimizeExactTraversal( ...
        baseMotion, affineBasisPolynomial, decision_deg, ...
        inequalityMatrix, -baseInequality_deg, maximumOffset_deg, limits);
    % Adopt the exact traversal refinement only after its own feasibility checks accept it.
    if exactTraversalDiagnostics.Accepted
        decision_deg = trialDecision_deg;
    end
end
solveTime_s = toc(solveTimer);
% Retry the unreduced route when an explicitly requested reduction made the optimization infeasible.
if exitFlag <= 0 && isfinite(options.RouteVertexCount) && size(route_deg, 1) < size(inputRoute_deg, 1)
    fallbackOptions = options;
    fallbackOptions.RouteVertexCount = Inf;
    motion = azElPlannerMethods.corridor.internal.motion.solveCorridorQuintic( ...
        obstacles, initialState, goalState, limits, inputRoute_deg, fallbackOptions);
    motion.OptimizerOptions = options;
    motion.OptimizerDiagnostics.RequestedRouteVertexCount = requestedVertexCount;
    motion.OptimizerDiagnostics.RouteVertexCountExpanded = true;
    motion.OptimizerDiagnostics.CompressionFallbackUsed = true;
    motion.OptimizerDiagnostics.CompressedRouteVertexCount = size(route_deg, 1);
    motion.OptimizerDiagnostics.CompressedExitFlag = exitFlag;
    motion.OptimizerDiagnostics.CompressedSolveTime_s = solveTime_s;
    motion.OptimizerDiagnostics.TotalDiagnosticTime_s = toc(diagnosticTimer);
    return;
end

%% Section 4: Independently Validate The Candidate

% Solver feasibility is not planner success. Reconstruct the exact polynomial,
% validate the full timed motion, and—when requested—verify that its continuous
% span envelopes remain inside their certified free-space supports.
if isempty(decision_deg)
    candidateMotion = baseMotion;
else
    candidateOptions = baseOptions;
    candidateOptions.ControlPointOffsets_deg = [decision_deg(1:interiorCount), decision_deg(interiorCount + 1:end)];
    candidateOptions.SampleTime_s = options.SampleTime_s;
    candidateMotion = azElPlannerMethods.corridor.internal.motion.buildQuinticSpline( ...
        route_deg, initialState, goalState, limits, candidateOptions);
end
candidateMotion.SeedCorridorBoundary_deg = seed.CorridorBoundary_deg;
candidateMotion.SeedCorridor = corridor;
if isempty(corridorObstacles)
    corridorCertified = true;
    certifiedClearance_deg = Inf;
else
    [corridorCertified, certifiedClearance_deg] = azElPlannerMethods.corridor.internal.validation.certifySeedCorridor( ...
            candidateMotion, corridorObstacles, queryOptions.ClearanceTolerance_deg);
end
validation = azElPlannerMethods.corridor.validateTrajectory( candidateMotion, obstacles, initialState, goalState, limits, plannerOptions);
candidateInequality_deg = azElPlannerMethods.corridor.internal.validation.seedCorridorInequality( candidateMotion.Polynomial, corridor);
envelopeShape = polyshape( seed.CorridorBoundary_deg(:, 1), seed.CorridorBoundary_deg(:, 2), "Simplify", true);
convexEnvelopeRegions = azElPlannerMethods.corridor.internal.geometry.convexPolygonRegions(envelopeShape);
expectedRecordCount = candidateMotion.Polynomial.SegmentCount * numel(convexEnvelopeRegions);
if isempty(corridorObstacles)
    envelopeContainsObstacles = true;
else
    envelopeContainsObstacles = azElPlannerMethods.corridor.internal.validation.seedEnvelopeContainsObstacles( ...
            seed.CorridorBoundary_deg, obstacles, queryOptions.ClearanceTolerance_deg);
end
totalDiagnosticTime_s = toc(diagnosticTimer);
if isempty(decision_deg)
    maximumDecision_deg = NaN;
else
    maximumDecision_deg = max(abs(decision_deg));
end
optimizerDiagnostics = struct( ...
    "OriginalRouteVertexCount", size(expandedRoute_deg, 1), ...
    "RequestedRouteVertexCount", requestedVertexCount, ...
    "RouteVertexCount", size(route_deg, 1), ...
    "RouteVertexCountExpanded", ...
    size(route_deg, 1) > requestedVertexCount, ...
    "CompressionFallbackUsed", false, ...
    "CompressedRouteVertexCount", size(route_deg, 1), ...
    "CompressedExitFlag", NaN, ...
    "CompressedSolveTime_s", NaN, ...
    "DecisionCount", decisionCount, ...
    "SegmentCount", candidateMotion.Polynomial.SegmentCount, ...
    "ExpectedCorridorRecordCount", expectedRecordCount, ...
    "ActualCorridorRecordCount", numel(corridor), ...
    "EnvelopeRegionCount", numel(convexEnvelopeRegions), ...
    "EnvelopeContainsObstacles", envelopeContainsObstacles, ...
    "StaticCorridorCertificateApplicable", ...
    options.RequireStaticCorridorCertificate, ...
    "BaseMaximumInequality_deg", ...
    max([NaN; baseInequality_deg(:)], [], "omitmissing"), ...
    "ExitFlag", exitFlag, ...
    "SolverIterations", solverOutput.iterations, ...
    "SolveTime_s", solveTime_s, ...
    "ExactTraversalAttempted", exactTraversalDiagnostics.Attempted, ...
    "ExactTraversalAccepted", exactTraversalDiagnostics.Accepted, ...
    "ExactTraversalInitialScale", exactTraversalDiagnostics.InitialScale, ...
    "ExactTraversalFinalScale", exactTraversalDiagnostics.FinalScale, ...
    "ExactTraversalLinearSolveCount", ...
    exactTraversalDiagnostics.LinearSolveCount, ...
    "ExactTraversalActiveConstraintCount", ...
    exactTraversalDiagnostics.ActiveConstraintCount, ...
    "MaximumDecision_deg", maximumDecision_deg, ...
    "CandidateMaximumInequality_deg", ...
    max([NaN; candidateInequality_deg(:)], [], "omitmissing"), ...
    "CorridorCertified", corridorCertified, ...
    "CertifiedClearance_deg", certifiedClearance_deg, ...
    "ValidationPassed", validation.Passed, ...
    "ContinuousClearance_deg", validation.MinimumClearance_deg, ...
    "MotionDuration_s", candidateMotion.MotionDuration_s, ...
    "TotalDiagnosticTime_s", totalDiagnosticTime_s, "SolverMessage", string(solverOutput.message));
motion = candidateMotion;
motion.Validation = validation;
motion.Success = exitFlag > 0 && corridorCertified && validation.Passed;
motion.OriginalRoute_deg = inputRoute_deg;
motion.ExpandedRoute_deg = expandedRoute_deg;
motion.ReducedRoute_deg = route_deg;
motion.SeedCorridorBoundary_deg = seed.CorridorBoundary_deg;
motion.SeedCorridor = corridor;
motion.OptimizerOptions = options;
motion.OptimizerDiagnostics = optimizerDiagnostics;
% Report whether the independently validated motion, not merely the optimizer, succeeded.
if motion.Success
    motion.Message = "The corridor-constrained quintic prototype passed " + ...
        "full-span certification and maintained validation.";
    motion.TerminationReason = "corridorPrototypeValidated";
elseif exitFlag <= 0
    motion.Message = "The bounded affine corridor system is infeasible.";
    motion.TerminationReason = "corridorInfeasible";
elseif ~corridorCertified
    motion.Message = "The candidate failed full-span corridor certification.";
    motion.TerminationReason = "corridorCertificateFailed";
else
    motion.Message = "The corridor-feasible candidate failed maintained " + "trajectory validation.";
    motion.TerminationReason = "trajectoryValidationFailed";
end
end


function expandedRoute_deg = expandRouteClearance( ...
        route_deg, obstacles, queryTime_s, clearanceTarget_deg, routeExpansionFraction)
% Move low-clearance interior vertices outward with shape-relative reserve.
expandedRoute_deg = route_deg;

% Endpoints are fixed by the request, so only interior seed vertices may move.
for routeIndex = 2:size(route_deg, 1) - 1
    point_deg = route_deg(routeIndex, :);
    nearestDistance_deg = Inf;
    nearestBoundaryPoint_deg = [NaN NaN];
    nearestSpan_deg = 0;

    % Find the protected obstacle that most restricts this route vertex.
    for obstacleIndex = 1:numel(obstacles)
        shape = azElPlannerMethods.corridor.internal.obstacles.shapeAtTime( obstacles(obstacleIndex), queryTime_s);
        vertices_deg = shape.Vertices;
        vertices_deg = vertices_deg(all(isfinite(vertices_deg), 2), :);
        [distance_deg, boundaryPoint_deg] = azElPlannerMethods.corridor.internal.geometry.pointPolygonClearance(shape, point_deg);
        if distance_deg < nearestDistance_deg
            nearestDistance_deg = distance_deg;
            nearestBoundaryPoint_deg = boundaryPoint_deg;
            nearestSpan_deg = norm( max(vertices_deg, [], 1) - min(vertices_deg, [], 1));
        end
    end
    desiredClearance_deg = clearanceTarget_deg + routeExpansionFraction * nearestSpan_deg;
    outwardDirection = point_deg - nearestBoundaryPoint_deg;
    if nearestDistance_deg < desiredClearance_deg && norm(outwardDirection) > eps
        expandedRoute_deg(routeIndex, :) = point_deg + ...
            (desiredClearance_deg - nearestDistance_deg) * outwardDirection / norm(outwardDirection);
    end
end
end

function reducedRoute_deg = sampledVisibilitySubsequence( ...
        route_deg, obstacles, requestedVertexCount, queryTime_s, edgeSamplingResolution_deg)
% Select a sampled-clear ordered subsequence before exact certification.
routeVertexCount = size(route_deg, 1);
visibility = false(routeVertexCount);

% Batch-test every forward route edge that could participate in an ordered subsequence.
for startIndex = 1:routeVertexCount - 1
    endIndices = (startIndex + 1:routeVertexCount).';
    edgeCount = numel(endIndices);
    sampleCounts = zeros(edgeCount, 1);

    % Allocate enough samples to honor the requested geometric resolution on each edge.
    for edgeIndex = 1:edgeCount
        edgeDelta_deg = route_deg(endIndices(edgeIndex), :) - route_deg(startIndex, :);
        sampleCounts(edgeIndex) = max( 2, ceil(norm(edgeDelta_deg) / edgeSamplingResolution_deg) + 1);
    end
    sampleOffsets = [0; cumsum(sampleCounts)];
    edgeSamples_deg = zeros(sampleOffsets(end), 2);

    % Pack all candidate edges into one obstacle query for this start vertex.
    for edgeIndex = 1:edgeCount
        endIndex = endIndices(edgeIndex);
        edgeDelta_deg = route_deg(endIndex, :) - route_deg(startIndex, :);
        edgeFraction = linspace(0, 1, sampleCounts(edgeIndex)).';
        rows = sampleOffsets(edgeIndex) + 1:sampleOffsets(edgeIndex + 1);
        edgeSamples_deg(rows, :) = route_deg(startIndex, :) + edgeFraction .* edgeDelta_deg;
    end
    isOccupied = azElPlannerMethods.corridor.queryTimeObstacle( ...
        obstacles, edgeSamples_deg(:, 1), edgeSamples_deg(:, 2), queryTime_s * ones(size(edgeSamples_deg, 1), 1));
    isOccupied = logical(isOccupied(:));

    % Mark an edge visible only when every packed sample remains outside obstacles.
    for edgeIndex = 1:edgeCount
        rows = sampleOffsets(edgeIndex) + 1:sampleOffsets(edgeIndex + 1);
        visibility(startIndex, endIndices(edgeIndex)) = ~any(isOccupied(rows));
    end
end
cost_deg = Inf(routeVertexCount, routeVertexCount);
parentIndex = zeros(routeVertexCount, routeVertexCount);
cost_deg(1, 1) = 0;

% Dynamic programming tracks the shortest ordered route for every allowed vertex count.
for usedVertexCount = 2:routeVertexCount

    % Try each route vertex as the endpoint of the current subsequence length.
    for endIndex = 2:routeVertexCount
        predecessorIndex = find( ...
            visibility(1:endIndex - 1, endIndex) & isfinite(cost_deg(1:endIndex - 1, usedVertexCount - 1)));
        if isempty(predecessorIndex)
            continue;
        end
        edgeLength_deg = vecnorm( route_deg(endIndex, :) - route_deg(predecessorIndex, :), 2, 2);
        candidateCost_deg = cost_deg(predecessorIndex, usedVertexCount - 1) + edgeLength_deg;
        [cost_deg(endIndex, usedVertexCount), localIndex] = min(candidateCost_deg);
        parentIndex(endIndex, usedVertexCount) = predecessorIndex(localIndex);
    end
end
selectedVertexCount = find( isfinite(cost_deg(end, requestedVertexCount:end)), 1, "first");
if isempty(selectedVertexCount)
    reducedRoute_deg = zeros(0, 2);
    return;
end
selectedVertexCount = selectedVertexCount + requestedVertexCount - 1;
selectedIndex = zeros(selectedVertexCount, 1);
selectedIndex(end) = routeVertexCount;

% Follow parent pointers backward to recover the selected ordered subsequence.
for usedVertexCount = selectedVertexCount:-1:2
    selectedIndex(usedVertexCount - 1) = parentIndex( selectedIndex(usedVertexCount), usedVertexCount);
end
reducedRoute_deg = route_deg(selectedIndex, :);
end

function options = validateOptions(options, routeVertexCount)
% Validate resolved corridor controls once.
validateattributes(options.RouteVertexCount, {'numeric'}, {'real', 'scalar', 'positive'});
if isfinite(options.RouteVertexCount)
    validateattributes(options.RouteVertexCount, {'numeric'}, {'integer', '>=', 2, '<=', routeVertexCount});
end
positiveNames = [ ...
    "MaximumControlPointOffset_deg", "SpanLengthExponent", ...
    "RouteSamplingResolution_deg", "EnvelopePadding_deg", "SampleTime_s"];

% Validate every strictly positive scalar option through one consistent contract.
for optionName = positiveNames
    validateattributes(options.(optionName), {'numeric'}, {'real', 'finite', 'scalar', 'positive'});
end
nonnegativeNames = ["ClearanceTarget_deg", "RouteExpansionFraction"];

% Validate controls for which zero intentionally disables expansion or clearance reserve.
for optionName = nonnegativeNames
    validateattributes(options.(optionName), {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
end
boundary_deg = options.ObstacleEnvelopeBoundary_deg;
boundaryRowsAreValid = isempty(boundary_deg) || ...
    (isnumeric(boundary_deg) && ismatrix(boundary_deg) && ...
    size(boundary_deg, 2) == 2 && all(all(isfinite(boundary_deg), 2) | all(isnan(boundary_deg), 2)));
if ~boundaryRowsAreValid
    error("solveCorridorQuintic:InvalidEnvelopeBoundary", ...
        "ObstacleEnvelopeBoundary_deg must be finite or NaN-separated N-by-2.");
end
if ~isempty(options.RouteTau)
    validateattributes(options.RouteTau, {'numeric'}, ...
        {'real', 'finite', 'vector', 'increasing', 'numel', routeVertexCount});
    options.RouteTau = double(options.RouteTau(:));
end
options.RequireStaticCorridorCertificate = azElPlannerMethods.corridor.internal.normalizeLogicalScalar( ...
    options.RequireStaticCorridorCertificate, ...
    "RequireStaticCorridorCertificate", "solveCorridorQuintic:InvalidCertificateControl");
options.GoalTimeMode = string(options.GoalTimeMode);
if ~isscalar(options.GoalTimeMode) || ~any(options.GoalTimeMode == ["earliestArrival", "fixedArrival"])
    error("solveCorridorQuintic:InvalidGoalTimeMode", "GoalTimeMode must be earliestArrival or fixedArrival.");
end
options.AllowAzimuthWrapping = azElPlannerMethods.corridor.internal.normalizeLogicalScalar( ...
    options.AllowAzimuthWrapping, "AllowAzimuthWrapping", "solveCorridorQuintic:InvalidWrappingOption");
options.EnableExactTraversal = azElPlannerMethods.corridor.internal.normalizeLogicalScalar( ...
    options.EnableExactTraversal, "EnableExactTraversal", "solveCorridorQuintic:InvalidExactTraversalControl");
end

function diagnostics = emptyDiagnostics(originalRouteVertexCount, requestedRouteVertexCount, elapsedTime_s)
% Define stable diagnostics for an unavailable route subsequence.
diagnostics = struct( ...
    "OriginalRouteVertexCount", originalRouteVertexCount, ...
    "RequestedRouteVertexCount", requestedRouteVertexCount, ...
    "RouteVertexCount", requestedRouteVertexCount, ...
    "RouteVertexCountExpanded", false, ...
    "CompressionFallbackUsed", false, ...
    "CompressedRouteVertexCount", 0, ...
    "CompressedExitFlag", NaN, ...
    "CompressedSolveTime_s", NaN, ...
    "DecisionCount", 0, ...
    "SegmentCount", 0, ...
    "ExpectedCorridorRecordCount", 0, ...
    "ActualCorridorRecordCount", 0, ...
    "EnvelopeRegionCount", 0, ...
    "EnvelopeContainsObstacles", false, ...
    "StaticCorridorCertificateApplicable", false, ...
    "BaseMaximumInequality_deg", NaN, ...
    "ExitFlag", 0, ...
    "SolverIterations", 0, ...
    "SolveTime_s", 0, ...
    "MaximumDecision_deg", NaN, ...
    "CandidateMaximumInequality_deg", NaN, ...
    "CorridorCertified", false, ...
    "CertifiedClearance_deg", NaN, ...
    "ValidationPassed", false, ...
    "ContinuousClearance_deg", NaN, ...
    "MotionDuration_s", NaN, ...
    "ExactTraversalAttempted", false, ...
    "ExactTraversalAccepted", false, ...
    "ExactTraversalInitialScale", NaN, ...
    "ExactTraversalFinalScale", NaN, ...
    "ExactTraversalLinearSolveCount", 0, ...
    "ExactTraversalActiveConstraintCount", 0, ...
    "TotalDiagnosticTime_s", elapsedTime_s, "SolverMessage", "Route subsequence unavailable.");
end
