function selection = selectAzElVisibilitySnapshots( ...
        obstacleField, initialState, goalState, options)
%% Section 0: Header & Readme
% SYNTAX
%   selection = azElInternal.selectAzElVisibilitySnapshots( ...
%       obstacleField, initialState, goalState, options)
%**************************************************************************
% PURPOSE
%   - Select informative obstacle snapshots and a bounded, traceable pool
%     of boundary candidates for visibility-graph construction.
%**************************************************************************
% INPUTS
%   - obstacleField (scalar packed obstacle field)
%       Complete protected static or time-varying obstacle history.
%   - initialState, goalState (scalar structs)
%       Normalized request states containing position_deg.
%   - options (scalar struct)
%       Resolved snapshot-event and candidate-reduction controls.
%**************************************************************************
% OUTPUTS
%   - selection (scalar struct)
%       Retained times, boundary candidates, provenance, and diagnostics.
%**************************************************************************
% UNITS
%   - Candidate position is degrees and snapshot time is seconds.
%**************************************************************************

%% Section 1: Select Snapshots & Collect Candidates

startPoint = normalizedStatePoint(initialState, "initialState");
goalPoint = normalizedStatePoint(goalState, "goalState");
candidatePoints = zeros(0, 3);
candidateTypes = strings(0, 1);
candidateObstacleIndex = zeros(0, 1);
candidateSampleIndex = zeros(0, 1);
candidateRegionIndex = zeros(0, 1);
candidateBoundaryGeometry = cell(0, 1);
retainedSnapshotTimes_s = zeros(0, 1);
snapshotDiagnosticTemplate = struct( ...
    "ObstacleIndex", 0, "InputSnapshotCount", 0, ...
    "RetainedSnapshotCount", 0, "WasReduced", false, ...
    "RetainedSampleIndex", zeros(0, 1), ...
    "RequiredSampleIndex", zeros(0, 1), ...
    "EventSampleIndex", zeros(0, 1), ...
    "EventDiagnostics", struct());
snapshotDiagnostics = repmat( ...
    snapshotDiagnosticTemplate, obstacleField.ObstacleCount, 1);
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
    snapshotDiagnostics(obstacleIndex).ObstacleIndex = obstacleIndex;
    snapshotDiagnostics(obstacleIndex).InputSnapshotCount = ...
        numel(availableSamples);
    if isempty(availableSamples)
        continue;
    end
    if isfinite(options.MaximumSnapshotsPerObstacle) && ...
            numel(availableSamples) > ...
            options.MaximumSnapshotsPerObstacle
        positions = unique(round(linspace(1, numel(availableSamples), ...
            options.MaximumSnapshotsPerObstacle)));
        retainedSamples = availableSamples(positions);
    else
        retainedSamples = availableSamples;
    end
    [eventSamples, eventDiagnostics] = detectSnapshotEvents( ...
        obstacle, availableSamples, options);
    retainedSamples = unique([retainedSamples(:); eventSamples]);
    requiredSamples = zeros(0, 1);
    obstacleTime_s = double(obstacle.TimeSeconds(:));
    requestedTime_s = options.RequiredSnapshotTimes_s;
    requestedTime_s = requestedTime_s( ...
        requestedTime_s >= obstacleTime_s(availableSamples(1)) & ...
        requestedTime_s <= obstacleTime_s(availableSamples(end)));
    for requestedTimeIndex = 1:numel(requestedTime_s)
        [~, nearestAvailableIndex] = min(abs( ...
            obstacleTime_s(availableSamples) - ...
            requestedTime_s(requestedTimeIndex)));
        requiredSamples(end + 1, 1) = ...
            availableSamples(nearestAvailableIndex); %#ok<AGROW>
    end
    requiredSamples = unique(requiredSamples);
    retainedSamples = unique([retainedSamples(:); requiredSamples]);
    snapshotDiagnostics(obstacleIndex).RetainedSnapshotCount = ...
        numel(retainedSamples);
    snapshotDiagnostics(obstacleIndex).WasReduced = ...
        numel(retainedSamples) < numel(availableSamples);
    snapshotDiagnostics(obstacleIndex).RetainedSampleIndex = ...
        retainedSamples(:);
    snapshotDiagnostics(obstacleIndex).RequiredSampleIndex = ...
        requiredSamples(:);
    snapshotDiagnostics(obstacleIndex).EventSampleIndex = ...
        eventSamples(:);
    snapshotDiagnostics(obstacleIndex).EventDiagnostics = ...
        eventDiagnostics;
    if options.Verbose
        maximumRotation_deg = max( ...
            eventDiagnostics.MaximumEdgeRotation_deg, [], "omitnan");
        maximumMotion_deg = max( ...
            eventDiagnostics.MaximumBoundaryMotion_deg, [], "omitnan");
        fprintf("[visibility events] obstacle %d: %d event slices, " + ...
            "max edge rotation %.3f deg, max boundary motion %.3f deg.\n", ...
            obstacleIndex, numel(eventSamples), ...
            maximumRotation_deg, maximumMotion_deg);
    end
    retainedSnapshotTimes_s = [retainedSnapshotTimes_s; ...
        double(obstacle.TimeSeconds(retainedSamples))]; %#ok<AGROW>
    if snapshotDiagnostics(obstacleIndex).WasReduced
        warning("buildAzElVisibilityRoutes:SnapshotReduction", ...
            "Obstacle '%s' retained %d of %d source snapshots for route " + ...
            "generation. Continuous collision validation still uses the " + ...
            "complete history, but a route available only at an omitted " + ...
            "snapshot can be missed. Set the standalone search option " + ...
            "MaximumSnapshotsPerObstacle=Inf, or planner option " + ...
            "MaximumVisibilitySnapshotsPerObstacle=Inf, to disable this " + ...
            "explicit runtime tradeoff.", ...
            obstacle.Name, numel(retainedSamples), numel(availableSamples));
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


%% Section 2: Assemble Selection

selection = struct( ...
    "StartPoint", startPoint, ...
    "GoalPoint", goalPoint, ...
    "CandidatePointsAzElTime", candidatePoints, ...
    "CandidateTypes", candidateTypes, ...
    "CandidateObstacleIndex", candidateObstacleIndex, ...
    "CandidateSampleIndex", candidateSampleIndex, ...
    "CandidateRegionIndex", candidateRegionIndex, ...
    "CandidateBoundaryGeometry", {candidateBoundaryGeometry}, ...
    "RetainedSnapshotTimes_s", unique(retainedSnapshotTimes_s), ...
    "SnapshotDiagnostics", snapshotDiagnostics, ...
    "CandidateReductionDiagnostics", candidateReductionDiagnostics);
end

%% Section 3: Local Functions

function [eventSampleIndex, diagnostics] = detectSnapshotEvents( ...
        obstacle, availableSampleIndex, options)
%% Section 0: Header & Readme
% SYNTAX
%   [eventSampleIndex, diagnostics] = detectSnapshotEvents( ...
%       obstacle, availableSampleIndex, options)
%**************************************************************************
% PURPOSE
%   - Scan every available source slice using cheap structural and boundary
%     motion descriptors before selecting expensive visibility graphs.
%**************************************************************************
% INPUTS
%   - obstacle (scalar packed-obstacle struct)
%   - availableSampleIndex (N-by-1 positive integer vector)
%   - options (resolved visibility-search options)
%**************************************************************************
% OUTPUTS
%   - eventSampleIndex (M-by-1 positive integer vector)
%       Both sides of every detected change interval.
%   - diagnostics (scalar struct)
%       Per-slice counts and per-interval rotation/motion measurements.
%**************************************************************************
% UNITS
%   - Edge rotation is degrees and midpoint motion is degrees.
%**************************************************************************
sampleCount = numel(availableSampleIndex);
vertexCount = zeros(sampleCount, 1);
edgeCount = zeros(sampleCount, 1);
boundaryComponentCount = zeros(sampleCount, 1);
islandCount = zeros(sampleCount, 1);
probeEdgeAngle_deg = nan(sampleCount, ...
    options.SnapshotProbeEdgeCount);
boundaryCenter_deg = nan(sampleCount, 2);
maximumEdgeRotation_deg = nan(sampleCount, 1);
maximumBoundaryMotion_deg = nan(sampleCount, 1);
topologyChanged = false(sampleCount, 1);
countThresholdCrossed = false(sampleCount, 1);
edgeCorrespondenceChanged = false(sampleCount, 1);
rotationThresholdCrossed = false(sampleCount, 1);
motionThresholdCrossed = false(sampleCount, 1);
eventSampleIndex = zeros(0, 1);

% --- Measure Every Available Source Slice -------------------------------
% These descriptors are deliberately cheaper than constructing a
% visibility graph. They identify intervals worth refining without making
% the planner solve the full graph at every source timestamp.
for availableIndex = 1:sampleCount
    sampleIndex = availableSampleIndex(availableIndex);
    vertexCount(availableIndex) = double( ...
        obstacle.SliceOffsets(sampleIndex + 1) - ...
        obstacle.SliceOffsets(sampleIndex));
    edgeCount(availableIndex) = double( ...
        obstacle.EdgeOffsets(sampleIndex + 1) - ...
        obstacle.EdgeOffsets(sampleIndex));
    if edgeCount(availableIndex) == 0
        continue;
    end
    edgeRows = double(obstacle.EdgeOffsets(sampleIndex)): ...
        double(obstacle.EdgeOffsets(sampleIndex + 1) - 1);
    edgeStart_deg = [obstacle.EdgeStartAzimuthDeg(edgeRows), ...
        obstacle.EdgeStartElevationDeg(edgeRows)];
    edgeEnd_deg = [obstacle.EdgeEndAzimuthDeg(edgeRows), ...
        obstacle.EdgeEndElevationDeg(edgeRows)];
    startsNewBoundary = [true; hypot( ...
        edgeStart_deg(2:end, 1) - edgeEnd_deg(1:end - 1, 1), ...
        edgeStart_deg(2:end, 2) - edgeEnd_deg(1:end - 1, 2)) > 1e-10];
    boundaryComponentCount(availableIndex) = nnz(startsNewBoundary);

    boundaryStartIndex = find(startsNewBoundary);
    boundaryEndIndex = [boundaryStartIndex(2:end) - 1; ...
        size(edgeStart_deg, 1)];
    signedArea_deg2 = zeros(numel(boundaryStartIndex), 1);
    for boundaryIndex = 1:numel(boundaryStartIndex)
        boundaryRows = boundaryStartIndex(boundaryIndex): ...
            boundaryEndIndex(boundaryIndex);
        signedArea_deg2(boundaryIndex) = 0.5 * sum( ...
            edgeStart_deg(boundaryRows, 1) .* ...
            edgeEnd_deg(boundaryRows, 2) - ...
            edgeEnd_deg(boundaryRows, 1) .* ...
            edgeStart_deg(boundaryRows, 2));
    end
    [largestBoundaryArea_deg2, largestBoundaryIndex] = max( ...
        abs(signedArea_deg2));
    areaTolerance_deg2 = 1e-12 * max(1, largestBoundaryArea_deg2);
    if largestBoundaryArea_deg2 > areaTolerance_deg2
        outerBoundarySign = sign(signedArea_deg2(largestBoundaryIndex));
        islandCount(availableIndex) = nnz( ...
            signedArea_deg2 .* outerBoundarySign > areaTolerance_deg2);
    else
        islandCount(availableIndex) = ...
            boundaryComponentCount(availableIndex);
    end

    sampleBounds_deg = double(obstacle.BoundsDeg(sampleIndex, :));
    boundaryCenter_deg(availableIndex, :) = 0.5 .* [ ...
        sampleBounds_deg(1) + sampleBounds_deg(2), ...
        sampleBounds_deg(3) + sampleBounds_deg(4)];
    edgeMidpoint_deg = 0.5 .* (edgeStart_deg + edgeEnd_deg);
    centeredMidpoint_deg = edgeMidpoint_deg - ...
        boundaryCenter_deg(availableIndex, :);
    edgeVector_deg = edgeEnd_deg - edgeStart_deg;
    edgeHasLength = hypot(edgeVector_deg(:, 1), ...
        edgeVector_deg(:, 2)) > 1e-12;
    probeDirection_deg = (0:options.SnapshotProbeEdgeCount - 1) .* ...
        (360 / options.SnapshotProbeEdgeCount);
    for probeIndex = 1:options.SnapshotProbeEdgeCount
        probeDirection = [cosd(probeDirection_deg(probeIndex)), ...
            sind(probeDirection_deg(probeIndex))];
        supportValue_deg = centeredMidpoint_deg * probeDirection.';
        supportValue_deg(~edgeHasLength) = -Inf;
        [~, probeEdgeIndex] = max(supportValue_deg);
        if isfinite(supportValue_deg(probeEdgeIndex))
            % Edge orientation is undirected. Mapping it into [0, 180)
            % prevents a ring-order reversal from looking like rotation.
            probeEdgeAngle_deg(availableIndex, probeIndex) = mod( ...
                atan2d(edgeVector_deg(probeEdgeIndex, 2), ...
                edgeVector_deg(probeEdgeIndex, 1)), 180);
        end
    end
end

% --- Detect Changes Between Adjacent Source Slices ----------------------
% An interval becomes an event when topology, size, orientation, or bulk
% motion changes enough to alter the useful visibility connections.
if options.DetectSnapshotEvents
    for availableIndex = 2:sampleCount
        previousSampleIndex = availableSampleIndex(availableIndex - 1);
        sampleIndex = availableSampleIndex(availableIndex);
        vertexCountChange = abs(vertexCount(availableIndex) - ...
            vertexCount(availableIndex - 1)) / max( ...
            1, vertexCount(availableIndex - 1));
        edgeCountChange = abs(edgeCount(availableIndex) - ...
            edgeCount(availableIndex - 1)) / max( ...
            1, edgeCount(availableIndex - 1));
        countThresholdCrossed(availableIndex) = max( ...
            vertexCountChange, edgeCountChange) > ...
            options.SnapshotCountChangeThreshold;
        topologyChanged(availableIndex) = ...
            boundaryComponentCount(availableIndex) ~= ...
            boundaryComponentCount(availableIndex - 1) || ...
            islandCount(availableIndex) ~= ...
            islandCount(availableIndex - 1);
        edgeCorrespondenceChanged(availableIndex) = any( ...
            ~obstacle.TopologyMatchesNext( ...
            previousSampleIndex:sampleIndex - 1));

        if edgeCount(availableIndex) > 0 && ...
                edgeCount(availableIndex - 1) > 0
            rotation_deg = mod( ...
                probeEdgeAngle_deg(availableIndex, :) - ...
                probeEdgeAngle_deg(availableIndex - 1, :) + 90, 180) - 90;
            maximumEdgeRotation_deg(availableIndex) = ...
                max(abs(rotation_deg), [], "omitnan");
            centerStep_deg = boundaryCenter_deg(availableIndex, :) - ...
                boundaryCenter_deg(availableIndex - 1, :);
            maximumBoundaryMotion_deg(availableIndex) = hypot( ...
                centerStep_deg(1), centerStep_deg(2));
            rotationThresholdCrossed(availableIndex) = ...
                maximumEdgeRotation_deg(availableIndex) > ...
                options.SnapshotEdgeRotationThreshold_deg;
            motionThresholdCrossed(availableIndex) = ...
                maximumBoundaryMotion_deg(availableIndex) > ...
                options.SnapshotBoundaryMotionThreshold_deg;
        end
        if topologyChanged(availableIndex) || ...
                countThresholdCrossed(availableIndex) || ...
                rotationThresholdCrossed(availableIndex) || ...
                motionThresholdCrossed(availableIndex)
            eventSampleIndex = [eventSampleIndex; ...
                previousSampleIndex; sampleIndex]; %#ok<AGROW>
        end
    end
end

% --- Publish Event Samples And Their Evidence ---------------------------
eventSampleIndex = unique(eventSampleIndex);
diagnostics = struct( ...
    "SampleIndex", availableSampleIndex(:), ...
    "VertexCount", vertexCount, ...
    "EdgeCount", edgeCount, ...
    "BoundaryComponentCount", boundaryComponentCount, ...
    "IslandCount", islandCount, ...
    "ProbeEdgeAngle_deg", probeEdgeAngle_deg, ...
    "MaximumEdgeRotation_deg", maximumEdgeRotation_deg, ...
    "MaximumBoundaryMotion_deg", maximumBoundaryMotion_deg, ...
    "TopologyChanged", topologyChanged, ...
    "CountThresholdCrossed", countThresholdCrossed, ...
    "EdgeCorrespondenceChanged", edgeCorrespondenceChanged, ...
    "RotationThresholdCrossed", rotationThresholdCrossed, ...
    "MotionThresholdCrossed", motionThresholdCrossed);
end

function [candidatePoints_deg, candidateTypes, diagnostics] = ...
        boundaryCandidates(region_deg, startPosition_deg, ...
        goalPosition_deg, options)
%% Section 0: Header & Readme
% SYNTAX
%   [candidatePoints_deg, candidateTypes, diagnostics] = ...
%       boundaryCandidates(region_deg, startPosition_deg, ...
%       goalPosition_deg, options)
%**************************************************************************
% PURPOSE
%   - Generate bounded graph nodes while preserving collision geometry.
%**************************************************************************
% INPUTS
%   - region_deg (N-by-2 numeric matrix)
%       Closed obstacle-ring vertices in [azimuth elevation] order.
%   - startPosition_deg, goalPosition_deg (1-by-2 numeric rows)
%       Route endpoints used to identify tangent contacts.
%   - options (scalar struct)
%       Candidate-selection controls resolved by the public function.
%**************************************************************************
% OUTPUTS
%   - candidatePoints_deg (M-by-2 numeric matrix)
%       Selected boundary points.
%   - candidateTypes (M-by-1 string array)
%       Provenance label for each point.
%   - diagnostics (scalar struct)
%       Candidate counts and reduction metadata.
%**************************************************************************
% UNITS
%   - Positions and angular thresholds are degrees.
%**************************************************************************
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
    diagnostics.ReductionPercent = 100 * max(0, 1 - ...
        diagnostics.SelectedCandidateCount / diagnostics.InputVertexCount);
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

    % Retain only turns large enough to change useful visibility. This
    % works on the original ring; collision geometry is never reduced.
    vertexCount = size(region_deg, 1);
    previousVertex_deg = region_deg([vertexCount 1:vertexCount - 1], :);
    nextVertex_deg = region_deg([2:vertexCount 1], :);
    incoming_deg = region_deg - previousVertex_deg;
    outgoing_deg = nextVertex_deg - region_deg;
    turnCross = incoming_deg(:, 1) .* outgoing_deg(:, 2) - ...
        incoming_deg(:, 2) .* outgoing_deg(:, 1);
    turnDot = sum(incoming_deg .* outgoing_deg, 2);
    turnAngle_deg = abs(rad2deg(atan2(turnCross, turnDot)));
    cornerIndex = find( ...
        turnAngle_deg >= options.CornerAngleThreshold_deg);
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

    % Support points in uniform directions bound the graph size while
    % retaining the extremal contacts most likely to open a short route.
    center_deg = mean(region_deg, 1);
    directionAngle_rad = (0:options.ExtremeDirectionCount - 1).' .* ...
        (2 * pi / options.ExtremeDirectionCount);
    direction = [cos(directionAngle_rad), sin(directionAngle_rad)];
    projection = (region_deg - center_deg) * direction.';
    [~, supportIndex] = max(projection, [], 1);
    extremeIndex = unique(supportIndex(:), "stable");
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
diagnostics.ReductionPercent = 100 * max(0, 1 - ...
    diagnostics.SelectedCandidateCount / diagnostics.InputVertexCount);
end

function routingRegion_deg = candidateRoutingBoundary( ...
        protectedRegion_deg, candidateClearance_deg)
%% Section 0: Header & Readme
% SYNTAX
%   routingRegion_deg = candidateRoutingBoundary( ...
%       protectedRegion_deg, candidateClearance_deg)
%**************************************************************************
% PURPOSE
%   - Offset only graph candidates while retaining protected collision data.
%**************************************************************************
% INPUTS
%   - protectedRegion_deg (N-by-2 numeric matrix)
%       Protected obstacle boundary.
%   - candidateClearance_deg (nonnegative numeric scalar)
%       Additional routing offset.
%**************************************************************************
% OUTPUTS
%   - routingRegion_deg (M-by-2 numeric matrix)
%       First finite ring of the optional offset boundary.
%**************************************************************************
% UNITS
%   - Positions and clearance are degrees.
%**************************************************************************
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
% SYNTAX
%   tangentIndex = polygonTangentVertexIndices( ...
%       region_deg, referencePosition_deg, maximumCount)
%**************************************************************************
% PURPOSE
%   - Approximate tangent contacts from reversals in boundary bearing.
%**************************************************************************
% INPUTS
%   - region_deg (N-by-2 numeric matrix)
%       Cyclic polygon vertices.
%   - referencePosition_deg (1-by-2 numeric row)
%       External position used for bearing measurements.
%   - maximumCount (nonnegative integer scalar or Inf)
%       Maximum contacts retained.
%**************************************************************************
% OUTPUTS
%   - tangentIndex (M-by-1 numeric vector)
%       Selected indices into region_deg.
%**************************************************************************
% UNITS
%   - Positions are degrees; bearings are evaluated in radians.
%**************************************************************************
% Local angular extrema recover rounded lip contacts that are not corners.
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
% SYNTAX
%   [isCircle, center_deg, radius_deg] = fittedCircle(region_deg)
%**************************************************************************
% PURPOSE
%   - Fit a circle and reject rings with excessive radial residual.
%**************************************************************************
% INPUTS
%   - region_deg (N-by-2 numeric matrix)
%       Polygon-ring vertices.
%**************************************************************************
% OUTPUTS
%   - isCircle (logical scalar)
%   - center_deg (1-by-2 numeric row)
%   - radius_deg (numeric scalar)
%**************************************************************************
% UNITS
%   - Center, radius, and residuals are degrees.
%**************************************************************************
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
% SYNTAX
%   tangentPoints_deg = tangentContacts( ...
%       referencePosition_deg, center_deg, radius_deg)
%**************************************************************************
% PURPOSE
%   - Return exact tangent contacts from an external point to a circle.
%**************************************************************************
% INPUTS
%   - referencePosition_deg, center_deg (1-by-2 numeric rows)
%   - radius_deg (positive numeric scalar)
%**************************************************************************
% OUTPUTS
%   - tangentPoints_deg (zero-by-2 or two-by-2 numeric matrix)
%**************************************************************************
% UNITS
%   - Positions and radius are degrees; internal angles are radians.
%**************************************************************************
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

function point = normalizedStatePoint(state, label)
%% Section 0: Header & Readme
% SYNTAX
%   point = normalizedStatePoint(state, label)
%**************************************************************************
% PURPOSE
%   - Validate and normalize a state to [azimuth elevation time].
%**************************************************************************
% INPUTS
%   - state (scalar struct)
%       Requires scalar time_s and two-element position_deg.
%   - label (scalar text)
%       Input name used in diagnostics.
%**************************************************************************
% OUTPUTS
%   - point (1-by-3 numeric row)
%**************************************************************************
% UNITS
%   - Azimuth/elevation are degrees and time is seconds.
%**************************************************************************
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
% SYNTAX
%   relevantRegion = routeRelevantRegionMask( ...
%       regions, startPosition_deg, goalPosition_deg)
%**************************************************************************
% PURPOSE
%   - Exclude nested components that neither route endpoint can occupy.
%**************************************************************************
% INPUTS
%   - regions (N-by-1 cell array)
%       Independent polygon rings for one time slice.
%   - startPosition_deg, goalPosition_deg (1-by-2 numeric rows)
%**************************************************************************
% OUTPUTS
%   - relevantRegion (N-by-1 logical vector)
%**************************************************************************
% UNITS
%   - Positions and polygon coordinates are degrees.
%**************************************************************************
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
% SYNTAX
%   regions = unpackSliceRegions(obstacle, sampleIndex)
%**************************************************************************
% PURPOSE
%   - Recover independent finite polygon rings from one packed slice.
%**************************************************************************
% INPUTS
%   - obstacle (scalar packed-obstacle struct)
%   - sampleIndex (positive integer scalar)
%**************************************************************************
% OUTPUTS
%   - regions (N-by-1 cell array)
%       Each cell contains one M-by-2 [azimuth elevation] ring.
%**************************************************************************
% UNITS
%   - Polygon coordinates are degrees.
%**************************************************************************
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

