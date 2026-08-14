function search = buildAzElVisibilityRoutes( ...
        obstacleField, initialState, goalState, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   search = buildAzElVisibilityRoutes( ...
%       obstacleField, initialState, goalState)
%   search = buildAzElVisibilityRoutes( ...
%       obstacleField, initialState, goalState, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Build collision-checked visibility routes from a bounded candidate
%     set while retaining full protected polygons for collision validation.
%**************************************************************************
% INPUTS
%   - obstacleField (scalar packed obstacle field)
%       Output from buildAzElTimeObstacleField.
%   - initialState, goalState (scalar structs)
%       time_s and position_deg fields.
%   - optionOverrides (scalar struct, optional; default struct())
%       Internal search and reduction controls.
%**************************************************************************
% OUTPUTS
%   - search (scalar struct)
%       VisibilityGraphs and CandidateReductionDiagnostics.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds.
%**************************************************************************
if nargin < 4 || isempty(optionOverrides)
    optionOverrides = struct();
end
defaults = struct( ...
    "MaximumSnapshotsPerObstacle", 10, ...
    "CandidateClearance_deg", 0, ...
    "CornerAngleThreshold_deg", 15, ...
    "PolygonCandidateMode", "adaptive", ...
    "ExtremeDirectionCount", 16, ...
    "MaximumTangenciesPerReference", 2, ...
    "BoundaryRouteReductionTolerance_deg", 0.10, ...
    "VisibilitySampleStep_deg", 0.10, ...
    "Verbose", false);
if ~isstruct(optionOverrides) || ~isscalar(optionOverrides)
    error("buildAzElVisibilityRoutes:InvalidOptions", ...
        "optionOverrides must be a scalar struct.");
end
unknownFields = setdiff(fieldnames(optionOverrides), fieldnames(defaults), "stable");
if ~isempty(unknownFields)
    warning("buildAzElVisibilityRoutes:UnknownOptions", ...
        "Ignoring unknown fields: %s.", strjoin(string(unknownFields), ", "));
    optionOverrides = rmfield(optionOverrides, unknownFields);
end
options = defaults;
names = fieldnames(optionOverrides);
for index = 1:numel(names)
    if ~isempty(optionOverrides.(names{index}))
        options.(names{index}) = optionOverrides.(names{index});
    end
end
validateattributes(options.MaximumSnapshotsPerObstacle, {'numeric'}, ...
    {'scalar','integer','positive'});
validateattributes(options.CandidateClearance_deg, {'numeric'}, ...
    {'scalar','real','finite','nonnegative'});
validateattributes(options.ExtremeDirectionCount, {'numeric'}, ...
    {'scalar','integer','>=',4});
validateattributes(options.MaximumTangenciesPerReference, {'numeric'}, ...
    {'scalar','integer','nonnegative'});
validateattributes(options.BoundaryRouteReductionTolerance_deg, {'numeric'}, ...
    {'scalar','real','finite','nonnegative'});
validateattributes(options.VisibilitySampleStep_deg, {'numeric'}, ...
    {'scalar','real','finite','positive'});
options.Verbose = logical(options.Verbose);
fieldIsPacked = isstruct(obstacleField) && isscalar(obstacleField) && ...
    isfield(obstacleField, "Format") && ...
    string(obstacleField.Format) == "AzElTimeObstacleField";
if ~fieldIsPacked
    error("buildAzElVisibilityRoutes:InvalidObstacleField", ...
        "obstacleField must come from buildAzElTimeObstacleField.");
end
startPoint = normalizedStatePoint(initialState, "initialState");
goalPoint = normalizedStatePoint(goalState, "goalState");
candidatePoints = zeros(0, 3);
candidateTypes = strings(0, 1);
candidateObstacleIndex = zeros(0, 1);
candidateSampleIndex = zeros(0, 1);
candidateRegionIndex = zeros(0, 1);
candidateBoundaryGeometry = cell(0, 1);
diagnosticTemplate = struct( ...
    "ObstacleIndex", 0, "SampleIndex", 0, "RegionIndex", 0, ...
    "InputVertexCount", 0, "StartTangentCount", 0, ...
    "GoalTangentCount", 0, "ExtremePointCount", 0, ...
    "CornerCount", 0, "SelectedCandidateCount", 0, ...
    "GraphActiveCandidateCount", 0, "ReductionPercent", 0, ...
    "RelevanceReductionPercent", 0, "Mode", "");
candidateReductionDiagnostics = repmat(diagnosticTemplate, 0, 1);
for obstacleIndex = 1:obstacleField.ObstacleCount
    obstacle = obstacleField.Obstacles(obstacleIndex);
    availableSamples = find(all(isfinite(double(obstacle.BoundsDeg)), 2));
    if isempty(availableSamples)
        continue;
    end
    if numel(availableSamples) > options.MaximumSnapshotsPerObstacle
        positions = unique(round(linspace(1, numel(availableSamples), ...
            options.MaximumSnapshotsPerObstacle)));
        retainedSamples = availableSamples(positions);
    else
        retainedSamples = availableSamples;
    end
    for sampleIndex = reshape(retainedSamples, 1, [])
        sampleTime_s = double(obstacle.TimeSeconds(sampleIndex));
        regions = unpackSliceRegions(obstacle, sampleIndex);
        relevantRegion = routeRelevantRegionMask( ...
            regions, startPoint(1:2), goalPoint(1:2));
        for regionIndex = 1:numel(regions)
            diagnostic = diagnosticTemplate;
            diagnostic.ObstacleIndex = obstacleIndex;
            diagnostic.SampleIndex = sampleIndex;
            diagnostic.RegionIndex = regionIndex;
            diagnostic.InputVertexCount = size(regions{regionIndex}, 1);
            if ~relevantRegion(regionIndex)
                diagnostic.ReductionPercent = 100;
                diagnostic.Mode = "inaccessibleRegion";
                candidateReductionDiagnostics(end + 1, 1) = diagnostic; %#ok<AGROW>
                continue;
            end
            routingRegion_deg = candidateRoutingBoundary( ...
                regions{regionIndex}, options.CandidateClearance_deg);
            [points_deg, types, diagnostic] = boundaryCandidates( ...
                routingRegion_deg, startPoint(1:2), goalPoint(1:2), options);
            diagnostic.ObstacleIndex = obstacleIndex;
            diagnostic.SampleIndex = sampleIndex;
            diagnostic.RegionIndex = regionIndex;
            candidateReductionDiagnostics(end + 1, 1) = diagnostic; %#ok<AGROW>
            count = size(points_deg, 1);
            candidatePoints = [candidatePoints; ...
                points_deg, repmat(sampleTime_s, count, 1)]; %#ok<AGROW>
            candidateTypes = [candidateTypes; types]; %#ok<AGROW>
            candidateObstacleIndex = [candidateObstacleIndex; ...
                repmat(obstacleIndex, count, 1)]; %#ok<AGROW>
            candidateSampleIndex = [candidateSampleIndex; ...
                repmat(sampleIndex, count, 1)]; %#ok<AGROW>
            candidateRegionIndex = [candidateRegionIndex; ...
                repmat(regionIndex, count, 1)]; %#ok<AGROW>
            candidateBoundaryGeometry = [candidateBoundaryGeometry; ...
                repmat({routingRegion_deg}, count, 1)]; %#ok<AGROW>
        end
    end
end
visibilityGraphs = buildSnapshotVisibilityGraphs( ...
    obstacleField, candidatePoints, candidateTypes, ...
    candidateObstacleIndex, candidateSampleIndex, candidateRegionIndex, ...
    candidateBoundaryGeometry, startPoint(1:2), goalPoint(1:2), options);
for graphIndex = 1:numel(visibilityGraphs)
    activeCandidate = visibilityGraphs(graphIndex).NodeCandidateIndex(3:end);
    activeCandidate = activeCandidate( ...
        visibilityGraphs(graphIndex).CandidateActiveMask);
    for diagnosticIndex = 1:numel(candidateReductionDiagnostics)
        diagnostic = candidateReductionDiagnostics(diagnosticIndex);
        belongs = candidateObstacleIndex == diagnostic.ObstacleIndex & ...
            candidateSampleIndex == diagnostic.SampleIndex & ...
            candidateRegionIndex == diagnostic.RegionIndex;
        activeCount = nnz(ismember(find(belongs), activeCandidate));
        candidateReductionDiagnostics(diagnosticIndex). ...
            GraphActiveCandidateCount = activeCount;
        if diagnostic.SelectedCandidateCount > 0
            candidateReductionDiagnostics(diagnosticIndex). ...
                RelevanceReductionPercent = 100 * ...
                (1 - activeCount / diagnostic.SelectedCandidateCount);
        end
    end
end
search = struct( ...
    "VisibilityGraphs", visibilityGraphs, ...
    "CandidateReductionDiagnostics", candidateReductionDiagnostics, ...
    "CandidatePointsAzElTime", candidatePoints, ...
    "Options", options);
end

%% Section 1: Local Functions
function [candidatePoints_deg, candidateTypes, diagnostics] = ...
        boundaryCandidates(region_deg, startPosition_deg, ...
        goalPosition_deg, options)
%% Section 0: Header & Readme
% Generate a bounded graph-node set without simplifying collision geometry.
diagnostics = struct( ...
    "ObstacleIndex", 0, ...
    "SampleIndex", 0, ...
    "RegionIndex", 0, ...
    "InputVertexCount", size(region_deg, 1), ...
    "StartTangentCount", 0, ...
    "GoalTangentCount", 0, ...
    "ExtremePointCount", 0, ...
    "CornerCount", 0, ...
    "SelectedCandidateCount", 0, ...
    "GraphActiveCandidateCount", 0, ...
    "ReductionPercent", 0, ...
    "RelevanceReductionPercent", 0, ...
    "Mode", options.PolygonCandidateMode);
[isCircle, center_deg, radius_deg] = fittedCircle(region_deg);
if isCircle
    startTangents_deg = tangentContacts( ...
        startPosition_deg, center_deg, radius_deg);
    goalTangents_deg = tangentContacts( ...
        goalPosition_deg, center_deg, radius_deg);
    candidatePoints_deg = [startTangents_deg; goalTangents_deg];
    candidateTypes = [ ...
        repmat("startTangent", size(startTangents_deg, 1), 1); ...
        repmat("goalTangent", size(goalTangents_deg, 1), 1)];
    diagnostics.StartTangentCount = size(startTangents_deg, 1);
    diagnostics.GoalTangentCount = size(goalTangents_deg, 1);
    diagnostics.SelectedCandidateCount = size(candidatePoints_deg, 1);
    diagnostics.ReductionPercent = candidateReductionPercent( ...
        diagnostics.InputVertexCount, diagnostics.SelectedCandidateCount);
    diagnostics.Mode = "circleTangencies";
    return;
end

adaptiveCornerLimit = 2 * options.ExtremeDirectionCount;
useAllCorners = options.PolygonCandidateMode == "allCorners" || ...
    (options.PolygonCandidateMode == "adaptive" && ...
    size(region_deg, 1) <= adaptiveCornerLimit);
if useAllCorners
    startTangentIndex = polygonTangentVertexIndices( ...
        region_deg, startPosition_deg, Inf);
    goalTangentIndex = polygonTangentVertexIndices( ...
        region_deg, goalPosition_deg, Inf);
    cornerIndex = meaningfulCornerIndices( ...
        region_deg, options.CornerAngleThreshold_deg);
    extremeIndex = zeros(0, 1);
    if options.PolygonCandidateMode == "adaptive"
        diagnostics.Mode = "adaptiveAllCorners";
    end
else
    startTangentIndex = polygonTangentVertexIndices( ...
        region_deg, startPosition_deg, ...
        options.MaximumTangenciesPerReference);
    goalTangentIndex = polygonTangentVertexIndices( ...
        region_deg, goalPosition_deg, ...
        options.MaximumTangenciesPerReference);
    cornerIndex = zeros(0, 1);
    extremeIndex = directionalExtremeVertexIndices( ...
        region_deg, options.ExtremeDirectionCount);
    if options.PolygonCandidateMode == "adaptive"
        diagnostics.Mode = "adaptiveExtreme";
    end
end
candidatePoints_deg = [ ...
    region_deg(startTangentIndex, :); ...
    region_deg(goalTangentIndex, :); ...
    region_deg(extremeIndex, :); ...
    region_deg(cornerIndex, :)];
candidateTypes = [ ...
    repmat("startTangent", numel(startTangentIndex), 1); ...
    repmat("goalTangent", numel(goalTangentIndex), 1); ...
    repmat("extreme", numel(extremeIndex), 1); ...
    repmat("corner", numel(cornerIndex), 1)];
if ~isempty(candidatePoints_deg)
    roundedCandidate_deg = round(candidatePoints_deg * 1e10) / 1e10;
    [~, uniqueCandidateIndex] = unique( ...
        roundedCandidate_deg, "rows", "stable");
    candidatePoints_deg = candidatePoints_deg(uniqueCandidateIndex, :);
    candidateTypes = candidateTypes(uniqueCandidateIndex);
end
diagnostics.StartTangentCount = numel(startTangentIndex);
diagnostics.GoalTangentCount = numel(goalTangentIndex);
diagnostics.ExtremePointCount = numel(extremeIndex);
diagnostics.CornerCount = numel(cornerIndex);
diagnostics.SelectedCandidateCount = size(candidatePoints_deg, 1);
diagnostics.ReductionPercent = candidateReductionPercent( ...
    diagnostics.InputVertexCount, diagnostics.SelectedCandidateCount);
end

function cornerIndex = meaningfulCornerIndices(region_deg, threshold_deg)
%% Section 0: Header & Readme
% Preserve the legacy all-corners candidate policy as an explicit fallback.
vertexCount = size(region_deg, 1);
previousVertex_deg = region_deg([vertexCount 1:vertexCount - 1], :);
nextVertex_deg = region_deg([2:vertexCount 1], :);
incoming_deg = region_deg - previousVertex_deg;
outgoing_deg = nextVertex_deg - region_deg;
turnCross = incoming_deg(:, 1) .* outgoing_deg(:, 2) - ...
    incoming_deg(:, 2) .* outgoing_deg(:, 1);
turnDot = sum(incoming_deg .* outgoing_deg, 2);
turnAngle_deg = abs(rad2deg(atan2(turnCross, turnDot)));
cornerIndex = find(turnAngle_deg >= threshold_deg);
end

function extremeIndex = directionalExtremeVertexIndices( ...
        region_deg, directionCount)
%% Section 0: Header & Readme
% Keep full-resolution polygon support points in bounded directions.
center_deg = mean(region_deg, 1);
directionAngle_rad = (0:directionCount - 1).' .* ...
    (2 * pi / directionCount);
direction = [cos(directionAngle_rad), sin(directionAngle_rad)];
projection = (region_deg - center_deg) * direction.';
[~, supportIndex] = max(projection, [], 1);
extremeIndex = unique(supportIndex(:), "stable");
end

function reductionPercent = candidateReductionPercent( ...
        inputVertexCount, selectedCandidateCount)
%% Section 0: Header & Readme
if inputVertexCount <= 0
    reductionPercent = 0;
    return;
end
reductionPercent = 100 * max(0, ...
    1 - selectedCandidateCount / inputVertexCount);
end

function routingRegion_deg = candidateRoutingBoundary( ...
        protectedRegion_deg, candidateClearance_deg)
%% Section 0: Header & Readme
% Offset only the search boundary; protected collision geometry is unchanged.
if candidateClearance_deg <= 0
    routingRegion_deg = protectedRegion_deg;
    return;
end
protectedPolygon = polyshape( ...
    protectedRegion_deg(:, 1), protectedRegion_deg(:, 2), ...
    "Simplify", true, "KeepCollinearPoints", true);
routingPolygon = polybuffer( ...
    protectedPolygon, candidateClearance_deg, "JointType", "square");
[routingAzimuth_deg, routingElevation_deg] = boundary(routingPolygon);
routingRegion_deg = [ ...
    double(routingAzimuth_deg(:)), double(routingElevation_deg(:))];
finiteRows = all(isfinite(routingRegion_deg), 2);
if ~all(finiteRows)
    firstRegionChanges = diff([false; finiteRows; false]);
    firstRegionStart = find(firstRegionChanges == 1, 1, "first");
    firstRegionStop = find(firstRegionChanges == -1, 1, "first") - 1;
    routingRegion_deg = ...
        routingRegion_deg(firstRegionStart:firstRegionStop, :);
end
end

function tangentIndex = polygonTangentVertexIndices( ...
        region_deg, referencePosition_deg, maximumCount)
%% Section 0: Header & Readme
% Find sampled boundary vertices where bearing from a reference reverses.
% These local angular extrema approximate tangencies on rounded buffers and
% recover lip contacts that are no longer sharp polygon corners.
vertexCount = size(region_deg, 1);
if vertexCount < 3
    tangentIndex = zeros(0, 1);
    return;
end
bearing_rad = atan2( ...
    region_deg(:, 2) - referencePosition_deg(2), ...
    region_deg(:, 1) - referencePosition_deg(1));
nextBearing_rad = bearing_rad([2:end 1]);
bearingStep_rad = mod( ...
    nextBearing_rad - bearing_rad + pi, 2 * pi) - pi;
previousBearingStep_rad = bearingStep_rad([end 1:end - 1]);
isLocalMaximum = previousBearingStep_rad > 1e-10 & ...
    bearingStep_rad <= 1e-10;
isLocalMinimum = previousBearingStep_rad < -1e-10 & ...
    bearingStep_rad >= -1e-10;
tangentIndex = find(isLocalMaximum | isLocalMinimum);
if isempty(tangentIndex) || isinf(maximumCount) || ...
        numel(tangentIndex) <= maximumCount
    return;
end
if maximumCount == 0
    tangentIndex = zeros(0, 1);
    return;
end

% Strong bearing reversals represent the meaningful silhouette contacts;
% small reversals are normally boundary digitization noise. Retain both
% angular sides before filling any remaining slots by reversal strength.
reversalScore = abs( ...
    previousBearingStep_rad(tangentIndex) - ...
    bearingStep_rad(tangentIndex));
candidateIsMaximum = isLocalMaximum(tangentIndex);
selectedPosition = zeros(0, 1);
for tangentSide = [true false]
    sidePosition = find(candidateIsMaximum == tangentSide);
    if isempty(sidePosition) || numel(selectedPosition) >= maximumCount
        continue;
    end
    [~, strongestOnSide] = max(reversalScore(sidePosition));
    selectedPosition(end + 1, 1) = ...
        sidePosition(strongestOnSide); %#ok<AGROW>
end
remainingPosition = setdiff( ...
    (1:numel(tangentIndex)).', selectedPosition, "stable");
[~, scoreOrder] = sort(reversalScore(remainingPosition), "descend");
remainingCapacity = maximumCount - numel(selectedPosition);
selectedPosition = [selectedPosition; remainingPosition( ...
    scoreOrder(1:min(remainingCapacity, numel(scoreOrder))))];
tangentIndex = tangentIndex(selectedPosition);
end

function [isCircle, center_deg, radius_deg] = fittedCircle(region_deg)
%% Section 0: Header & Readme
% Fit a circle and reject rings whose radial residual is not circle-like.
isCircle = false;
center_deg = [NaN NaN];
radius_deg = NaN;
if size(region_deg, 1) < 8
    return;
end
x_deg = region_deg(:, 1);
y_deg = region_deg(:, 2);
fitMatrix = [2 * x_deg, 2 * y_deg, ones(size(x_deg))];
if rank(fitMatrix) < 3
    return;
end
fitValues = fitMatrix \ (x_deg.^2 + y_deg.^2);
center_deg = fitValues(1:2).';
radiusSquared_deg2 = fitValues(3) + sum(center_deg.^2);
if radiusSquared_deg2 <= 0
    return;
end
radius_deg = sqrt(radiusSquared_deg2);
radialDistance_deg = hypot( ...
    x_deg - center_deg(1), y_deg - center_deg(2));
maximumResidual_deg = max(abs(radialDistance_deg - radius_deg));
circleTolerance_deg = max(1e-4, 0.01 * radius_deg);
isCircle = maximumResidual_deg <= circleTolerance_deg;
end

function tangentPoints_deg = tangentContacts( ...
        referencePosition_deg, center_deg, radius_deg)
%% Section 0: Header & Readme
% Return the two exact contacts from an external point to a circle.
centerToReference_deg = referencePosition_deg - center_deg;
referenceDistance_deg = hypot( ...
    centerToReference_deg(1), centerToReference_deg(2));
if referenceDistance_deg <= radius_deg + 1e-12
    tangentPoints_deg = zeros(0, 2);
    return;
end
baseAngle_rad = atan2( ...
    centerToReference_deg(2), centerToReference_deg(1));
tangentOffset_rad = acos(radius_deg / referenceDistance_deg);
tangentAngles_rad = baseAngle_rad + ...
    [-tangentOffset_rad; tangentOffset_rad];
tangentPoints_deg = center_deg + radius_deg * [ ...
    cos(tangentAngles_rad), sin(tangentAngles_rad)];
end

function graphs = buildSnapshotVisibilityGraphs( ...
        obstacleField, candidatePoints, candidateTypes, ...
        candidateObstacleIndex, candidateSampleIndex, ...
        candidateRegionIndex, candidateBoundaryGeometry, ...
        startPosition_deg, goalPosition_deg, options)
%% Section 0: Header & Readme
% Build one all-obstacle graph at each retained obstacle-slice time.
graphs = repmat(emptyVisibilityGraph(), 0, 1);
if isempty(candidatePoints)
    return;
end
snapshotTimes_s = unique(candidatePoints(:, 3), "stable");
snapshotCount = numel(snapshotTimes_s);
graphs = repmat(emptyVisibilityGraph(), snapshotCount, 1);
for snapshotIndex = 1:snapshotCount
    snapshotTime_s = snapshotTimes_s(snapshotIndex);
    graphs(snapshotIndex) = buildOneSnapshotVisibilityGraph( ...
        obstacleField, candidatePoints, candidateTypes, ...
        candidateObstacleIndex, candidateSampleIndex, ...
        candidateRegionIndex, candidateBoundaryGeometry, ...
        startPosition_deg, goalPosition_deg, options, snapshotTime_s);
    if options.Verbose
        fprintf("[visibility graph] %d/%d at t=%.3f s: " + ...
            "%d active candidates, success=%d.\n", ...
            snapshotIndex, snapshotCount, snapshotTime_s, ...
            nnz(graphs(snapshotIndex).CandidateActiveMask), ...
            graphs(snapshotIndex).Success);
    end
end
end

function graph = buildOneSnapshotVisibilityGraph( ...
        obstacleField, candidatePoints, candidateTypes, ...
        candidateObstacleIndex, candidateSampleIndex, ...
        candidateRegionIndex, candidateBoundaryGeometry, ...
        startPosition_deg, goalPosition_deg, options, snapshotTime_s)
%% Section 0: Header & Readme
% Solve one graph for one retained obstacle snapshot.
candidateIsOnSnapshot = abs( ...
    candidatePoints(:, 3) - snapshotTime_s) <= 1e-10;
globalCandidateIndex = find(candidateIsOnSnapshot);
graph = buildVisibilityGraphAtTime( ...
    obstacleField, candidatePoints(candidateIsOnSnapshot, 1:2), ...
    candidateTypes(candidateIsOnSnapshot), ...
    candidateObstacleIndex(candidateIsOnSnapshot), ...
    candidateSampleIndex(candidateIsOnSnapshot), ...
    candidateRegionIndex(candidateIsOnSnapshot), ...
    candidateBoundaryGeometry(candidateIsOnSnapshot), ...
    globalCandidateIndex, startPosition_deg, goalPosition_deg, ...
    snapshotTime_s, options);
end

function graph = emptyVisibilityGraph()
%% Section 0: Header & Readme
% Define the stable public schema for one snapshot visibility graph.
graph = struct( ...
    "Success", false, ...
    "Message", "Visibility graph was not evaluated.", ...
    "Time_s", NaN, ...
    "NodePosition_deg", zeros(0, 2), ...
    "NodeType", strings(0, 1), ...
    "NodeCandidateIndex", zeros(0, 1), ...
    "NodeObstacleIndex", zeros(0, 1), ...
    "CandidateActiveMask", false(0, 1), ...
    "EdgeCost_deg", zeros(0, 0), ...
    "EdgeType", strings(0, 0), ...
    "EdgeRoute_deg", {cell(0, 0)}, ...
    "PathNodeIndex", zeros(0, 1), ...
    "PathCandidateIndex", zeros(0, 1), ...
    "PathObstacleIndex", zeros(0, 1), ...
    "PathPosition_deg", zeros(0, 2), ...
    "PathEdgeType", strings(0, 1), ...
    "PathCost_deg", Inf, ...
    "PathEdgeCount", 0, ...
    "PathVisibilityEdgeCount", 0, ...
    "PathBoundaryEdgeCount", 0, ...
    "GeneratedVisibilityEdgeCount", 0, ...
    "GeneratedBoundaryEdgeCount", 0, ...
    "BoundaryRouteInputVertexCount", 0, ...
    "BoundaryRouteRetainedVertexCount", 0, ...
    "BoundaryRouteVertexReductionPercent", 0);
end

function graph = buildVisibilityGraphAtTime( ...
        obstacleField, candidatePosition_deg, candidateTypes, ...
        candidateObstacleIndex, candidateSampleIndex, ...
        candidateRegionIndex, candidateBoundaryGeometry, ...
        globalCandidateIndex, ...
        startPosition_deg, goalPosition_deg, snapshotTime_s, options)
%% Section 0: Header & Readme
% Connect all mutually visible nodes and adjacent same-boundary candidates.
graph = emptyVisibilityGraph();
graph.Time_s = snapshotTime_s;
candidateCount = size(candidatePosition_deg, 1);
nodePosition_deg = [startPosition_deg; goalPosition_deg; ...
    candidatePosition_deg];
nodeCount = size(nodePosition_deg, 1);
nodeType = ["start"; "goal"; candidateTypes];
nodeCandidateIndex = [0; 0; globalCandidateIndex(:)];
nodeObstacleIndex = [0; 0; candidateObstacleIndex(:)];
edgeCost_deg = inf(nodeCount, nodeCount);
edgeType = strings(nodeCount, nodeCount);
edgeRoute_deg = cell(nodeCount, nodeCount);
generatedVisibilityEdgeCount = 0;
generatedBoundaryEdgeCount = 0;
boundaryRouteInputVertexCount = 0;
boundaryRouteRetainedVertexCount = 0;

% Phase 1 is intentionally sparse: connect only start/goal to candidates,
% plus the direct edge. Together with cyclic boundary edges below, this
% produces a feasible upper-bound route before any candidate-pair clique is
% attempted.
endpointPair = [1 2];
for endpointPairIndex = 1:size(endpointPair, 1)
    firstNodeIndex = endpointPair(endpointPairIndex, 1);
    secondNodeIndex = endpointPair(endpointPairIndex, 2);
    [edgeCost_deg, edgeType, edgeRoute_deg, edgeWasAdded] = ...
        addVisibilityEdgeIfClear(edgeCost_deg, edgeType, edgeRoute_deg, ...
        nodePosition_deg, firstNodeIndex, secondNodeIndex, ...
        obstacleField, snapshotTime_s, options);
    generatedVisibilityEdgeCount = generatedVisibilityEdgeCount + ...
        edgeWasAdded;
end
for candidateIndex = 1:candidateCount
    candidateNodeIndex = candidateIndex + 2;
    for endpointNodeIndex = 1:2
        [edgeCost_deg, edgeType, edgeRoute_deg, edgeWasAdded] = ...
            addVisibilityEdgeIfClear( ...
            edgeCost_deg, edgeType, edgeRoute_deg, nodePosition_deg, ...
            endpointNodeIndex, candidateNodeIndex, obstacleField, ...
            snapshotTime_s, options);
        generatedVisibilityEdgeCount = generatedVisibilityEdgeCount + ...
            edgeWasAdded;
    end
end

% Candidates on one ring are sorted by boundary arclength. Connecting only
% cyclic neighbors preserves both sides of every obstacle without creating
% an artificial chord through its filled interior.
candidateMetadata = [candidateObstacleIndex(:), ...
    candidateSampleIndex(:), candidateRegionIndex(:)];
boundaryGroups = unique(candidateMetadata, "rows", "stable");
for boundaryGroupIndex = 1:size(boundaryGroups, 1)
    boundaryGroup = boundaryGroups(boundaryGroupIndex, :);
    candidateIsOnBoundary = all( ...
        candidateMetadata == boundaryGroup, 2);
    localCandidateIndex = find(candidateIsOnBoundary);
    if numel(localCandidateIndex) < 2
        continue;
    end
    if isempty(candidateBoundaryGeometry{localCandidateIndex(1)})
        continue;
    end
    region_deg = candidateBoundaryGeometry{localCandidateIndex(1)};
    candidateArc_deg = candidateBoundaryArcPositions( ...
        region_deg, candidatePosition_deg(localCandidateIndex, :));
    [~, boundaryOrder] = sort(candidateArc_deg);
    orderedCandidateIndex = localCandidateIndex(boundaryOrder);
    for orderedIndex = 1:numel(orderedCandidateIndex)
        nextOrderedIndex = mod( ...
            orderedIndex, numel(orderedCandidateIndex)) + 1;
        firstCandidateIndex = orderedCandidateIndex(orderedIndex);
        secondCandidateIndex = orderedCandidateIndex(nextOrderedIndex);
        firstNodeIndex = firstCandidateIndex + 2;
        secondNodeIndex = secondCandidateIndex + 2;
        [rawBoundaryRoute_deg, ~, ~] = ...
            forwardBoundaryRoute(region_deg, ...
            candidatePosition_deg(firstCandidateIndex, :), ...
            candidatePosition_deg(secondCandidateIndex, :));
        [boundaryRoute_deg, routeIsValid] = ...
            reduceBoundaryRouteForGraph(rawBoundaryRoute_deg, ...
            obstacleField, snapshotTime_s, options);
        if ~routeIsValid
            continue;
        end
        boundaryRouteInputVertexCount = ...
            boundaryRouteInputVertexCount + size(rawBoundaryRoute_deg, 1);
        boundaryRouteRetainedVertexCount = ...
            boundaryRouteRetainedVertexCount + size(boundaryRoute_deg, 1);
        boundaryStep_deg = diff(boundaryRoute_deg, 1, 1);
        boundaryDistance_deg = sum(hypot( ...
            boundaryStep_deg(:, 1), boundaryStep_deg(:, 2)));
        if boundaryDistance_deg >= ...
                edgeCost_deg(firstNodeIndex, secondNodeIndex) - 1e-10
            continue;
        end
        edgeCost_deg(firstNodeIndex, secondNodeIndex) = ...
            boundaryDistance_deg;
        edgeCost_deg(secondNodeIndex, firstNodeIndex) = ...
            boundaryDistance_deg;
        edgeType(firstNodeIndex, secondNodeIndex) = "boundary";
        edgeType(secondNodeIndex, firstNodeIndex) = "boundary";
        edgeRoute_deg{firstNodeIndex, secondNodeIndex} = ...
            boundaryRoute_deg;
        edgeRoute_deg{secondNodeIndex, firstNodeIndex} = ...
            flipud(boundaryRoute_deg);
        generatedBoundaryEdgeCount = generatedBoundaryEdgeCount + 1;
    end
end

% A collision-free baseline gives a rigorous upper bound. Any candidate
% whose Euclidean start-plus-goal lower bound exceeds it cannot occur on a
% shorter route and is removed before candidate-pair visibility expansion.
[baselinePathNodeIndex, baselinePathCost_deg] = ...
    shortestVisibilityGraphPath(edgeCost_deg, 1, 2);
candidateActiveMask = true(candidateCount, 1);
if ~isempty(baselinePathNodeIndex) && isfinite(baselinePathCost_deg)
    startOffset_deg = candidatePosition_deg - startPosition_deg;
    goalOffset_deg = candidatePosition_deg - goalPosition_deg;
    candidateLowerBound_deg = hypot( ...
        startOffset_deg(:, 1), startOffset_deg(:, 2)) + hypot( ...
        goalOffset_deg(:, 1), goalOffset_deg(:, 2));
    costTolerance_deg = 1e-9 * max(1, baselinePathCost_deg);
    candidateActiveMask = candidateLowerBound_deg <= ...
        baselinePathCost_deg + costTolerance_deg;
    baselineCandidateNode = baselinePathNodeIndex( ...
        baselinePathNodeIndex > 2) - 2;
    candidateActiveMask(baselineCandidateNode) = true;
    inactiveNodeIndex = find(~candidateActiveMask) + 2;
    edgeCost_deg(inactiveNodeIndex, :) = Inf;
    edgeCost_deg(:, inactiveNodeIndex) = Inf;
    edgeType(inactiveNodeIndex, :) = "";
    edgeType(:, inactiveNodeIndex) = "";
    edgeRoute_deg(inactiveNodeIndex, :) = {[]};
    edgeRoute_deg(:, inactiveNodeIndex) = {[]};
end

% Phase 2 forms visibility edges only among candidates that can still beat
% the known upper bound. This turns the dense graph from O(all extremes^2)
% into O(route-relevant extremes^2) without a preferred-side heuristic.
activeCandidateIndex = find(candidateActiveMask);
for firstActiveIndex = 1:numel(activeCandidateIndex) - 1
    firstCandidateIndex = activeCandidateIndex(firstActiveIndex);
    firstNodeIndex = firstCandidateIndex + 2;
    for secondActiveIndex = firstActiveIndex + 1: ...
            numel(activeCandidateIndex)
        secondCandidateIndex = activeCandidateIndex(secondActiveIndex);
        secondNodeIndex = secondCandidateIndex + 2;
        [edgeCost_deg, edgeType, edgeRoute_deg, edgeWasAdded] = ...
            addVisibilityEdgeIfClear( ...
            edgeCost_deg, edgeType, edgeRoute_deg, nodePosition_deg, ...
            firstNodeIndex, secondNodeIndex, obstacleField, ...
            snapshotTime_s, options);
        generatedVisibilityEdgeCount = generatedVisibilityEdgeCount + ...
            edgeWasAdded;
    end
end
generatedVisibilityEdgeCount = nnz(triu(edgeType == "visibility", 1));
generatedBoundaryEdgeCount = nnz(triu(edgeType == "boundary", 1));

[pathNodeIndex, pathCost_deg] = shortestVisibilityGraphPath( ...
    edgeCost_deg, 1, 2);
graph.NodePosition_deg = nodePosition_deg;
graph.NodeType = nodeType;
graph.NodeCandidateIndex = nodeCandidateIndex;
graph.NodeObstacleIndex = nodeObstacleIndex;
graph.EdgeCost_deg = edgeCost_deg;
graph.EdgeType = edgeType;
graph.EdgeRoute_deg = edgeRoute_deg;
graph.GeneratedVisibilityEdgeCount = generatedVisibilityEdgeCount;
graph.GeneratedBoundaryEdgeCount = generatedBoundaryEdgeCount;
graph.CandidateActiveMask = candidateActiveMask;
graph.BoundaryRouteInputVertexCount = boundaryRouteInputVertexCount;
graph.BoundaryRouteRetainedVertexCount = ...
    boundaryRouteRetainedVertexCount;
if boundaryRouteInputVertexCount > 0
    graph.BoundaryRouteVertexReductionPercent = 100 * max(0, ...
        1 - boundaryRouteRetainedVertexCount / ...
        boundaryRouteInputVertexCount);
end
if isempty(pathNodeIndex)
    graph.Message = "No path connects start to goal in this visibility graph.";
    return;
end

pathEdgeCount = numel(pathNodeIndex) - 1;
pathEdgeType = strings(pathEdgeCount, 1);
pathPosition_deg = zeros(0, 2);
for pathEdgeIndex = 1:pathEdgeCount
    firstNodeIndex = pathNodeIndex(pathEdgeIndex);
    secondNodeIndex = pathNodeIndex(pathEdgeIndex + 1);
    pathEdgeType(pathEdgeIndex) = edgeType( ...
        firstNodeIndex, secondNodeIndex);
    edgePath_deg = edgeRoute_deg{firstNodeIndex, secondNodeIndex};
    if isempty(pathPosition_deg)
        pathPosition_deg = edgePath_deg;
    else
        pathPosition_deg = [pathPosition_deg; ...
            edgePath_deg(2:end, :)]; %#ok<AGROW>
    end
end
graph.Success = true;
graph.Message = ...
    "Global visibility-graph Dijkstra connected start to goal.";
graph.PathNodeIndex = pathNodeIndex;
graph.PathCandidateIndex = nodeCandidateIndex(pathNodeIndex);
graph.PathObstacleIndex = nodeObstacleIndex(pathNodeIndex);
graph.PathPosition_deg = removeConsecutiveDuplicatePoints( ...
    pathPosition_deg);
graph.PathEdgeType = pathEdgeType;
graph.PathCost_deg = pathCost_deg;
graph.PathEdgeCount = pathEdgeCount;
graph.PathVisibilityEdgeCount = nnz(pathEdgeType == "visibility");
graph.PathBoundaryEdgeCount = nnz(pathEdgeType == "boundary");
end

function [edgeCost_deg, edgeType, edgeRoute_deg, edgeWasAdded] = ...
        addVisibilityEdgeIfClear(edgeCost_deg, edgeType, edgeRoute_deg, ...
        nodePosition_deg, firstNodeIndex, secondNodeIndex, ...
        obstacleField, snapshotTime_s, options)
%% Section 0: Header & Readme
% Add one straight graph edge only when its open interior is collision-free.
edgeWasAdded = 0;
displacement_deg = nodePosition_deg(secondNodeIndex, :) - ...
    nodePosition_deg(firstNodeIndex, :);
distance_deg = hypot(displacement_deg(1), displacement_deg(2));
if distance_deg <= 1e-12 || distance_deg >= ...
        edgeCost_deg(firstNodeIndex, secondNodeIndex) - 1e-10 || ...
        ~lineVisibleAtTime(obstacleField, ...
        nodePosition_deg(firstNodeIndex, :), ...
        nodePosition_deg(secondNodeIndex, :), snapshotTime_s, options)
    return;
end
edgeCost_deg(firstNodeIndex, secondNodeIndex) = distance_deg;
edgeCost_deg(secondNodeIndex, firstNodeIndex) = distance_deg;
edgeType(firstNodeIndex, secondNodeIndex) = "visibility";
edgeType(secondNodeIndex, firstNodeIndex) = "visibility";
edgeRoute_deg{firstNodeIndex, secondNodeIndex} = [ ...
    nodePosition_deg(firstNodeIndex, :); ...
    nodePosition_deg(secondNodeIndex, :)];
edgeRoute_deg{secondNodeIndex, firstNodeIndex} = [ ...
    nodePosition_deg(secondNodeIndex, :); ...
    nodePosition_deg(firstNodeIndex, :)];
edgeWasAdded = 1;
end

function candidateArc_deg = candidateBoundaryArcPositions( ...
        region_deg, candidatePosition_deg)
%% Section 0: Header & Readme
% Project every graph candidate onto one cyclic boundary coordinate.
if size(region_deg, 1) > 1 && hypot( ...
        region_deg(end, 1) - region_deg(1, 1), ...
        region_deg(end, 2) - region_deg(1, 2)) <= 1e-10
    region_deg(end, :) = [];
end
closedRegion_deg = [region_deg; region_deg(1, :)];
boundaryStep_deg = diff(closedRegion_deg, 1, 1);
edgeLength_deg = hypot( ...
    boundaryStep_deg(:, 1), boundaryStep_deg(:, 2));
perimeter_deg = sum(edgeLength_deg);
vertexArc_deg = [0; cumsum(edgeLength_deg(1:end - 1))];
candidateArc_deg = zeros(size(candidatePosition_deg, 1), 1);
for candidateIndex = 1:size(candidatePosition_deg, 1)
    candidateArc_deg(candidateIndex) = boundaryArcPosition( ...
        region_deg, edgeLength_deg, vertexArc_deg, ...
        candidatePosition_deg(candidateIndex, :), perimeter_deg);
end
end

function [pathNodeIndex, pathCost] = shortestVisibilityGraphPath( ...
        edgeCost, startNodeIndex, goalNodeIndex)
%% Section 0: Header & Readme
% Run deterministic Dijkstra on the small dense snapshot graph.
nodeCount = size(edgeCost, 1);
costToReach = inf(nodeCount, 1);
hopCount = inf(nodeCount, 1);
parentNodeIndex = zeros(nodeCount, 1);
settled = false(nodeCount, 1);
costToReach(startNodeIndex) = 0;
hopCount(startNodeIndex) = 0;
for expansionIndex = 1:nodeCount
    unfinishedCost = costToReach;
    unfinishedCost(settled) = Inf;
    [currentCost, currentNodeIndex] = min(unfinishedCost);
    if ~isfinite(currentCost)
        break;
    end
    settled(currentNodeIndex) = true;
    if currentNodeIndex == goalNodeIndex
        break;
    end
    neighborNodeIndex = find(isfinite(edgeCost(currentNodeIndex, :)));
    for neighborIndex = reshape(neighborNodeIndex, 1, [])
        if settled(neighborIndex)
            continue;
        end
        trialCost = currentCost + edgeCost( ...
            currentNodeIndex, neighborIndex);
        trialHopCount = hopCount(currentNodeIndex) + 1;
        improvesCost = trialCost < costToReach(neighborIndex) - 1e-10;
        tiesCostWithFewerHops = abs( ...
            trialCost - costToReach(neighborIndex)) <= 1e-10 && ...
            trialHopCount < hopCount(neighborIndex);
        if improvesCost || tiesCostWithFewerHops
            costToReach(neighborIndex) = trialCost;
            hopCount(neighborIndex) = trialHopCount;
            parentNodeIndex(neighborIndex) = currentNodeIndex;
        end
    end
end
pathCost = costToReach(goalNodeIndex);
if ~isfinite(pathCost)
    pathNodeIndex = zeros(0, 1);
    return;
end
pathNodeIndex = goalNodeIndex;
while pathNodeIndex(1) ~= startNodeIndex
    parentIndex = parentNodeIndex(pathNodeIndex(1));
    if parentIndex == 0
        pathNodeIndex = zeros(0, 1);
        pathCost = Inf;
        return;
    end
    pathNodeIndex = [parentIndex; pathNodeIndex]; %#ok<AGROW>
end
end

function [reducedRoute_deg, success] = reduceBoundaryRouteForGraph( ...
        sourceRoute_deg, obstacleField, sampleTime_s, options)
%% Section 0: Header & Readme
% Reduce a boundary-following command polyline without changing collision
% geometry. RDP proposes a compact shape, blocked chords restore source
% detail, and a final farthest-visible pass removes redundant safe points.
sourceRoute_deg = removeConsecutiveDuplicatePoints(sourceRoute_deg);
sourceVertexCount = size(sourceRoute_deg, 1);
if sourceVertexCount <= 2 || ...
        options.BoundaryRouteReductionTolerance_deg <= 0
    reducedRoute_deg = sourceRoute_deg;
    success = routeSegmentsAreVisible( ...
        reducedRoute_deg, obstacleField, sampleTime_s, options);
    return;
end

retainedIndex = rdpPolylineIndices(sourceRoute_deg, ...
    options.BoundaryRouteReductionTolerance_deg);
retainedIndex = refineBlockedRouteSegments( ...
    sourceRoute_deg, retainedIndex, obstacleField, sampleTime_s, options);
proposedRoute_deg = sourceRoute_deg(retainedIndex, :);
if ~routeSegmentsAreVisible( ...
        proposedRoute_deg, obstacleField, sampleTime_s, options)
    reducedRoute_deg = sourceRoute_deg;
    success = routeSegmentsAreVisible( ...
        reducedRoute_deg, obstacleField, sampleTime_s, options);
    return;
end

% Greedily jump to the farthest collision-free retained point. This keeps
% true protrusions while removing points that do not constrain visibility.
reducedRoute_deg = proposedRoute_deg(1, :);
currentIndex = 1;
while currentIndex < size(proposedRoute_deg, 1)
    nextIndex = size(proposedRoute_deg, 1);
    while nextIndex > currentIndex + 1 && ~lineVisibleAtTime( ...
            obstacleField, proposedRoute_deg(currentIndex, :), ...
            proposedRoute_deg(nextIndex, :), sampleTime_s, options)
        nextIndex = nextIndex - 1;
    end
    reducedRoute_deg(end + 1, :) = ...
        proposedRoute_deg(nextIndex, :); %#ok<AGROW>
    currentIndex = nextIndex;
end
success = routeSegmentsAreVisible( ...
    reducedRoute_deg, obstacleField, sampleTime_s, options);
end

function retainedIndex = rdpPolylineIndices(position_deg, tolerance_deg)
%% Section 0: Header & Readme
% Iterative Ramer-Douglas-Peucker indices; endpoints are always retained.
vertexCount = size(position_deg, 1);
isRetained = false(vertexCount, 1);
isRetained([1 end]) = true;
segmentStack = [1 vertexCount];
while ~isempty(segmentStack)
    firstIndex = segmentStack(end, 1);
    finalIndex = segmentStack(end, 2);
    segmentStack(end, :) = [];
    if finalIndex <= firstIndex + 1
        continue;
    end
    interiorIndex = (firstIndex + 1:finalIndex - 1).';
    distance_deg = pointToSegmentDistance( ...
        position_deg(interiorIndex, :), position_deg(firstIndex, :), ...
        position_deg(finalIndex, :));
    [maximumDistance_deg, localMaximumIndex] = max(distance_deg);
    if maximumDistance_deg <= tolerance_deg
        continue;
    end
    splitIndex = interiorIndex(localMaximumIndex);
    isRetained(splitIndex) = true;
    segmentStack = [segmentStack; ...
        firstIndex splitIndex; splitIndex finalIndex]; %#ok<AGROW>
end
retainedIndex = find(isRetained);
end

function distance_deg = pointToSegmentDistance( ...
        point_deg, firstPosition_deg, secondPosition_deg)
%% Section 0: Header & Readme
segment_deg = secondPosition_deg - firstPosition_deg;
segmentLengthSquared_deg2 = sum(segment_deg.^2);
if segmentLengthSquared_deg2 <= eps
    offset_deg = point_deg - firstPosition_deg;
    distance_deg = hypot(offset_deg(:, 1), offset_deg(:, 2));
    return;
end
projection = (point_deg - firstPosition_deg) * segment_deg.' ./ ...
    segmentLengthSquared_deg2;
projection = max(0, min(1, projection));
closestPoint_deg = firstPosition_deg + projection .* segment_deg;
offset_deg = point_deg - closestPoint_deg;
distance_deg = hypot(offset_deg(:, 1), offset_deg(:, 2));
end

function retainedIndex = refineBlockedRouteSegments( ...
        sourceRoute_deg, retainedIndex, obstacleField, sampleTime_s, options)
%% Section 0: Header & Readme
% Restore source vertices only where an RDP chord penetrates an obstacle.
changed = true;
while changed
    changed = false;
    refinedIndex = retainedIndex(1);
    for segmentIndex = 1:numel(retainedIndex) - 1
        firstIndex = retainedIndex(segmentIndex);
        finalIndex = retainedIndex(segmentIndex + 1);
        isVisible = lineVisibleAtTime(obstacleField, ...
            sourceRoute_deg(firstIndex, :), ...
            sourceRoute_deg(finalIndex, :), sampleTime_s, options);
        if ~isVisible && finalIndex > firstIndex + 1
            interiorIndex = (firstIndex + 1:finalIndex - 1).';
            distance_deg = pointToSegmentDistance( ...
                sourceRoute_deg(interiorIndex, :), ...
                sourceRoute_deg(firstIndex, :), ...
                sourceRoute_deg(finalIndex, :));
            [~, localMaximumIndex] = max(distance_deg);
            splitIndex = interiorIndex(localMaximumIndex);
            if splitIndex <= firstIndex || splitIndex >= finalIndex
                splitIndex = floor((firstIndex + finalIndex) / 2);
            end
            refinedIndex(end + 1, 1) = splitIndex; %#ok<AGROW>
            changed = true;
        end
        refinedIndex(end + 1, 1) = finalIndex; %#ok<AGROW>
    end
    retainedIndex = unique(refinedIndex, "stable");
end
end

function visible = routeSegmentsAreVisible( ...
        route_deg, obstacleField, sampleTime_s, options)
%% Section 0: Header & Readme
visible = true;
for segmentIndex = 1:size(route_deg, 1) - 1
    if ~lineVisibleAtTime(obstacleField, ...
            route_deg(segmentIndex, :), ...
            route_deg(segmentIndex + 1, :), sampleTime_s, options)
        visible = false;
        return;
    end
end
end

function visible = lineVisibleAtTime( ...
        obstacleField, firstPosition_deg, secondPosition_deg, ...
        sampleTime_s, ~)
%% Section 0: Header & Readme
% Test one open segment by topology intervals rather than distance samples.
% Polygon occupancy is constant between consecutive boundary crossings, so
% one midpoint per interval is exact for the stored piecewise-linear field.
% Cost therefore scales with packed edges and actual crossings, not with
% segment length divided by VisibilitySampleStep_deg.
displacement_deg = secondPosition_deg - firstPosition_deg;
distance_deg = hypot(displacement_deg(1), displacement_deg(2));
if distance_deg <= 1e-12
    visible = true;
    return;
end
sampleFraction = visibilitySegmentIntervalMidpoints( ...
    obstacleField, firstPosition_deg, secondPosition_deg, sampleTime_s);
samplePosition_deg = firstPosition_deg + ...
    sampleFraction .* displacement_deg;
blocked = queryAzElTimeObstacle(obstacleField, ...
    samplePosition_deg(:, 1), samplePosition_deg(:, 2), ...
    repmat(sampleTime_s, numel(sampleFraction), 1), struct( ...
    "CollisionMode", "polygon", ...
    "TimePaddingSamples", 0, ...
    "BoundaryIsOccupied", false));
visible = ~any(blocked);
end

function intervalMidpoint = visibilitySegmentIntervalMidpoints( ...
        obstacleField, firstPosition_deg, secondPosition_deg, sampleTime_s)
%% Section 0: Header & Readme
% Find every parameter at which the segment crosses a packed slice edge.
breakpoint = [0; 1];
segmentMinimum_deg = min(firstPosition_deg, secondPosition_deg);
segmentMaximum_deg = max(firstPosition_deg, secondPosition_deg);
for obstacleIndex = 1:numel(obstacleField.Obstacles)
    obstacle = obstacleField.Obstacles(obstacleIndex);
    if obstacle.SampleCount < 1 || sampleTime_s < ...
            double(obstacle.TimeSeconds(1)) || sampleTime_s > ...
            double(obstacle.TimeSeconds(end))
        continue;
    end
    [~, sampleIndex] = min(abs( ...
        double(obstacle.TimeSeconds(:)) - sampleTime_s));
    bounds_deg = double(obstacle.BoundsDeg(sampleIndex, :));
    if any(~isfinite(bounds_deg)) || ...
            segmentMaximum_deg(1) < bounds_deg(1) || ...
            segmentMinimum_deg(1) > bounds_deg(2) || ...
            segmentMaximum_deg(2) < bounds_deg(3) || ...
            segmentMinimum_deg(2) > bounds_deg(4)
        continue;
    end
    firstEdgeIndex = double(obstacle.EdgeOffsets(sampleIndex));
    finalEdgeIndex = double(obstacle.EdgeOffsets(sampleIndex + 1)) - 1;
    if finalEdgeIndex < firstEdgeIndex
        continue;
    end
    edgeIndex = (firstEdgeIndex:finalEdgeIndex).';
    edgeStart_deg = [ ...
        double(obstacle.EdgeStartAzimuthDeg(edgeIndex)), ...
        double(obstacle.EdgeStartElevationDeg(edgeIndex))];
    edgeEnd_deg = [ ...
        double(obstacle.EdgeEndAzimuthDeg(edgeIndex)), ...
        double(obstacle.EdgeEndElevationDeg(edgeIndex))];
    edgeBreakpoint = segmentEdgeIntersectionParameters( ...
        firstPosition_deg, secondPosition_deg, ...
        edgeStart_deg, edgeEnd_deg);
    breakpoint = [breakpoint; edgeBreakpoint]; %#ok<AGROW>
end
breakpoint = sort(max(0, min(1, breakpoint)));
if numel(breakpoint) > 1
    scale = max(1, max(abs(breakpoint)));
    retain = [true; diff(breakpoint) > 1e-11 * scale];
    breakpoint = breakpoint(retain);
end
if breakpoint(1) > 0
    breakpoint = [0; breakpoint];
end
if breakpoint(end) < 1
    breakpoint(end + 1, 1) = 1;
end
intervalMidpoint = 0.5 .* (breakpoint(1:end - 1) + ...
    breakpoint(2:end));
if isempty(intervalMidpoint)
    intervalMidpoint = 0.5;
end
end

function intersectionParameter = segmentEdgeIntersectionParameters( ...
        firstPosition_deg, secondPosition_deg, ...
        edgeStart_deg, edgeEnd_deg)
%% Section 0: Header & Readme
% Return candidate-segment parameters for crossings and collinear overlaps.
segmentVector_deg = secondPosition_deg - firstPosition_deg;
edgeVector_deg = edgeEnd_deg - edgeStart_deg;
edgeOffset_deg = edgeStart_deg - firstPosition_deg;
denominator_deg2 = segmentVector_deg(1) .* edgeVector_deg(:, 2) - ...
    segmentVector_deg(2) .* edgeVector_deg(:, 1);
edgeLength_deg = hypot(edgeVector_deg(:, 1), edgeVector_deg(:, 2));
segmentLength_deg = hypot(segmentVector_deg(1), segmentVector_deg(2));
tolerance_deg2 = 1e-11 .* max(1, ...
    segmentLength_deg .* edgeLength_deg);
isNonparallel = abs(denominator_deg2) > tolerance_deg2;
intersectionParameter = zeros(0, 1);
if any(isNonparallel)
    numeratorT_deg2 = edgeOffset_deg(:, 1) .* edgeVector_deg(:, 2) - ...
        edgeOffset_deg(:, 2) .* edgeVector_deg(:, 1);
    numeratorU_deg2 = edgeOffset_deg(:, 1) .* segmentVector_deg(2) - ...
        edgeOffset_deg(:, 2) .* segmentVector_deg(1);
    parameterT = numeratorT_deg2(isNonparallel) ./ ...
        denominator_deg2(isNonparallel);
    parameterU = numeratorU_deg2(isNonparallel) ./ ...
        denominator_deg2(isNonparallel);
    parameterTolerance = 1e-10;
    isOnBothSegments = parameterT >= -parameterTolerance & ...
        parameterT <= 1 + parameterTolerance & ...
        parameterU >= -parameterTolerance & ...
        parameterU <= 1 + parameterTolerance;
    intersectionParameter = parameterT(isOnBothSegments);
end

isParallel = ~isNonparallel;
if any(isParallel)
    collinearCross_deg2 = ...
        edgeOffset_deg(:, 1) .* segmentVector_deg(2) - ...
        edgeOffset_deg(:, 2) .* segmentVector_deg(1);
    isCollinear = isParallel & ...
        abs(collinearCross_deg2) <= tolerance_deg2;
    if any(isCollinear)
        segmentLengthSquared_deg2 = sum(segmentVector_deg.^2);
        collinearStart = edgeOffset_deg(isCollinear, :) * ...
            segmentVector_deg.' ./ segmentLengthSquared_deg2;
        collinearEnd = (edgeEnd_deg(isCollinear, :) - ...
            firstPosition_deg) * segmentVector_deg.' ./ ...
            segmentLengthSquared_deg2;
        overlapStart = max(0, min(collinearStart, collinearEnd));
        overlapEnd = min(1, max(collinearStart, collinearEnd));
        hasOverlap = overlapEnd >= overlapStart - 1e-10;
        intersectionParameter = [intersectionParameter; ...
            overlapStart(hasOverlap); overlapEnd(hasOverlap)];
    end
end
intersectionParameter = max(0, min(1, intersectionParameter));
end

function [route_deg, edgeCount, routeDistance_deg] = ...
        forwardBoundaryRoute(region_deg, startCandidate_deg, ...
        goalCandidate_deg)
%% Section 0: Header & Readme
% Walk one polygon ring in its stored direction between two contacts.
if size(region_deg, 1) > 1 && hypot( ...
        region_deg(end, 1) - region_deg(1, 1), ...
        region_deg(end, 2) - region_deg(1, 2)) <= 1e-10
    region_deg(end, :) = [];
end
vertexCount = size(region_deg, 1);
if vertexCount < 2
    route_deg = [startCandidate_deg; goalCandidate_deg];
    edgeCount = 1;
    routeDistance_deg = hypot( ...
        goalCandidate_deg(1) - startCandidate_deg(1), ...
        goalCandidate_deg(2) - startCandidate_deg(2));
    return;
end

closedRegion_deg = [region_deg; region_deg(1, :)];
boundaryStep_deg = diff(closedRegion_deg, 1, 1);
boundaryEdgeLength_deg = hypot( ...
    boundaryStep_deg(:, 1), boundaryStep_deg(:, 2));
perimeter_deg = sum(boundaryEdgeLength_deg);
vertexArc_deg = [0; cumsum(boundaryEdgeLength_deg(1:end - 1))];
startArc_deg = boundaryArcPosition( ...
    region_deg, boundaryEdgeLength_deg, vertexArc_deg, ...
    startCandidate_deg, perimeter_deg);
goalArc_deg = boundaryArcPosition( ...
    region_deg, boundaryEdgeLength_deg, vertexArc_deg, ...
    goalCandidate_deg, perimeter_deg);
forwardArc_deg = mod(goalArc_deg - startArc_deg, perimeter_deg);

relativeVertexArc_deg = mod(vertexArc_deg - startArc_deg, perimeter_deg);
arcTolerance_deg = max(1e-10, 1e-10 * perimeter_deg);
interiorVertex = relativeVertexArc_deg > arcTolerance_deg & ...
    relativeVertexArc_deg < forwardArc_deg - arcTolerance_deg;
interiorVertexIndex = find(interiorVertex);
[~, vertexOrder] = sort(relativeVertexArc_deg(interiorVertexIndex));
interiorVertexIndex = interiorVertexIndex(vertexOrder);
route_deg = [startCandidate_deg; ...
    region_deg(interiorVertexIndex, :); goalCandidate_deg];
route_deg = removeConsecutiveDuplicatePoints(route_deg);
edgeCount = max(0, size(route_deg, 1) - 1);
routeStep_deg = diff(route_deg, 1, 1);
routeDistance_deg = sum(hypot( ...
    routeStep_deg(:, 1), routeStep_deg(:, 2)));
end

function arcPosition_deg = boundaryArcPosition( ...
        region_deg, edgeLength_deg, vertexArc_deg, point_deg, ...
        perimeter_deg)
%% Section 0: Header & Readme
% Project a boundary candidate to the closest polygon edge and arclength.
nextRegion_deg = region_deg([2:end 1], :);
edgeVector_deg = nextRegion_deg - region_deg;
edgeLengthSquared_deg2 = sum(edgeVector_deg.^2, 2);
pointOffset_deg = point_deg - region_deg;
projectionFraction = sum(pointOffset_deg .* edgeVector_deg, 2) ./ ...
    max(edgeLengthSquared_deg2, eps);
projectionFraction = min(max(projectionFraction, 0), 1);
projectedPoint_deg = region_deg + projectionFraction .* edgeVector_deg;
projectionError_deg2 = sum((projectedPoint_deg - point_deg).^2, 2);
[~, closestEdgeIndex] = min(projectionError_deg2);
arcPosition_deg = vertexArc_deg(closestEdgeIndex) + ...
    projectionFraction(closestEdgeIndex) * edgeLength_deg(closestEdgeIndex);
arcPosition_deg = mod(arcPosition_deg, perimeter_deg);
end

function route_deg = removeConsecutiveDuplicatePoints(route_deg)
%% Section 0: Header & Readme
% Retain geometric turns while removing zero-length augmented edges.
if size(route_deg, 1) < 2
    return;
end
routeStep_deg = diff(route_deg, 1, 1);
keepPoint = [true; hypot( ...
    routeStep_deg(:, 1), routeStep_deg(:, 2)) > 1e-10];
route_deg = route_deg(keepPoint, :);
end

function point = normalizedStatePoint(state, label)
%% Section 0: Header & Readme
% Convert a boundary state to [azimuth elevation time].
hasRequiredFields = isstruct(state) && isscalar(state) && ...
    all(isfield(state, ["time_s" "position_deg"]));
if ~hasRequiredFields
    error("buildAzElVisibilityRoutes:InvalidState", ...
        "%s must contain time_s and position_deg.", label);
end
validateattributes(state.time_s, {'numeric'}, ...
    {'scalar', 'real', 'finite'});
validateattributes(state.position_deg, {'numeric'}, ...
    {'vector', 'numel', 2, 'real', 'finite'});
position_deg = reshape(double(state.position_deg), 1, 2);
point = [position_deg, double(state.time_s)];
end

function relevantRegion = routeRelevantRegionMask( ...
        regions, startPosition_deg, goalPosition_deg)
%% Section 0: Header & Readme
% Exclude enclosed free-space components that neither endpoint can occupy.
% Ring nesting is recomputed independently for every time slice, so moving
% or deforming topology never inherits classifications from another slice.
regionCount = numel(regions);
relevantRegion = true(regionCount, 1);
if regionCount < 2
    return;
end
regionArea_deg2 = zeros(regionCount, 1);
for regionIndex = 1:regionCount
    region = regions{regionIndex};
    regionArea_deg2(regionIndex) = abs(polyarea( ...
        region(:, 1), region(:, 2)));
end
parentRegionIndex = zeros(regionCount, 1);
for regionIndex = 1:regionCount
    region = regions{regionIndex};
    testPoint_deg = region(1, :);
    containingRegion = zeros(0, 1);
    for possibleParentIndex = 1:regionCount
        if possibleParentIndex == regionIndex || ...
                regionArea_deg2(possibleParentIndex) <= ...
                regionArea_deg2(regionIndex)
            continue;
        end
        possibleParent = regions{possibleParentIndex};
        [isInside, isOn] = inpolygon(testPoint_deg(1), ...
            testPoint_deg(2), possibleParent(:, 1), possibleParent(:, 2));
        if isInside || isOn
            containingRegion(end + 1, 1) = ...
                possibleParentIndex; %#ok<AGROW>
        end
    end
    if ~isempty(containingRegion)
        [~, smallestContainer] = min( ...
            regionArea_deg2(containingRegion));
        parentRegionIndex(regionIndex) = ...
            containingRegion(smallestContainer);
    end
end
nestingDepth = zeros(regionCount, 1);
for regionIndex = 1:regionCount
    ancestorIndex = parentRegionIndex(regionIndex);
    visitedCount = 0;
    while ancestorIndex > 0 && visitedCount < regionCount
        nestingDepth(regionIndex) = nestingDepth(regionIndex) + 1;
        ancestorIndex = parentRegionIndex(ancestorIndex);
        visitedCount = visitedCount + 1;
    end
end
for regionIndex = 1:regionCount
    if nestingDepth(regionIndex) == 0
        continue;
    end
    freeSpaceAncestorIndex = regionIndex;
    while nestingDepth(freeSpaceAncestorIndex) > 1
        freeSpaceAncestorIndex = ...
            parentRegionIndex(freeSpaceAncestorIndex);
    end
    freeSpaceAncestor = regions{freeSpaceAncestorIndex};
    startInside = inpolygon(startPosition_deg(1), startPosition_deg(2), ...
        freeSpaceAncestor(:, 1), freeSpaceAncestor(:, 2));
    goalInside = inpolygon(goalPosition_deg(1), goalPosition_deg(2), ...
        freeSpaceAncestor(:, 1), freeSpaceAncestor(:, 2));
    relevantRegion(regionIndex) = startInside || goalInside;
end
end

function regions = unpackSliceRegions(obstacle, sampleIndex)
%% Section 0: Header & Readme
% Recover independent finite polygon rings from one packed slice.
firstVertex = double(obstacle.SliceOffsets(sampleIndex));
finalVertex = double(obstacle.SliceOffsets(sampleIndex + 1) - 1);
if finalVertex < firstVertex
    regions = cell(0, 1);
    return;
end
azimuth_deg = double(obstacle.AzimuthDeg(firstVertex:finalVertex));
elevation_deg = double(obstacle.ElevationDeg(firstVertex:finalVertex));
isFiniteVertex = isfinite(azimuth_deg) & isfinite(elevation_deg);
regionChanges = diff([false; isFiniteVertex; false]);
regionStarts = find(regionChanges == 1);
regionStops = find(regionChanges == -1) - 1;
regions = cell(numel(regionStarts), 1);
for regionIndex = 1:numel(regionStarts)
    rows = regionStarts(regionIndex):regionStops(regionIndex);
    regions{regionIndex} = [azimuth_deg(rows), elevation_deg(rows)];
end
end
