function [obstacle, history] = createContiguousUSObstacle(time_s, safetyMargin_units, options)
%% Section 0: Header & Readme
% SYNTAX
%   [obstacle, history] = createContiguousUSObstacle( ...
%       time_s, safetyMargin_units)
%   [obstacle, history] = createContiguousUSObstacle( ...
%       time_s, safetyMargin_units, options)
%**************************************************************************
% PURPOSE
%   - Load and union the Mapping Toolbox contiguous-U.S. outline.
%   - Construct either a static outline history or the maintained extreme
%     growth, deformation, translation, and half-turn history.
%   - Keep source loading, deformation code, validation, and safety
%     protection out of example scripts.
%**************************************************************************
% INPUTS
%   - time_s (nonempty increasing numeric vector)
%       Sample times for the requested obstacle history.
%   - safetyMargin_units (nonnegative scalar)
%       Euclidean protection margin applied by obstacle construction.
%   - options (scalar struct, optional)
%       .MotionMode is static or movingDeforming (default static).
%       .Verbose is logical (default false).
%       .MaximumOutlineVertices caps the source outline vertex count by a
%        deterministic Douglas-Peucker reduction (default Inf, no reduction).
%        The reduced outline is the supplied obstacle; the planner treats it
%        exactly and applies no further simplification.
%**************************************************************************
% OUTPUTS
%   - obstacle (scalar obstacle struct)
%       Canonical protected moving obstacle.
%   - history (scalar struct)
%       Generic slice history plus source outline metadata. Invalid input
%       throws an error.
%**************************************************************************
% UNITS
%   - Longitude/latitude are treated as x/y coordinate units; time_s
%     is seconds and safetyMargin_units is coordinate units.
%**************************************************************************

%% Section 1: Validate Inputs And Apply Defaults

% Check the time history, margin, and motion mode before map data is loaded.
% Static mode repeats one outline. Moving mode applies a known transform at each
% time so the generic moving-obstacle function can validate every slice.

if nargin < 3 || isempty(options)
    options = struct();
end
if ~isstruct(options) || ~isscalar(options)
    error("createContiguousUSObstacle:InvalidOptions", "options must be a scalar struct.");
end
defaultOptions                        = struct();
defaultOptions.MotionMode             = "static";
defaultOptions.Verbose                = false;
defaultOptions.MaximumOutlineVertices = Inf;
[resolvedOptions, unknownOptionFields] = obstacleAvoidance.input.resolveOptions(defaultOptions, options);
if ~isempty(unknownOptionFields)
    warning("createContiguousUSObstacle:UnknownOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", ...
        strjoin(unknownOptionFields, ", "));
end
motionMode = lower(string(resolvedOptions.MotionMode));
if ~isscalar(motionMode) || ~any(motionMode == ["static" "movingdeforming"])
    error("createContiguousUSObstacle:InvalidMotionMode", "MotionMode must be static or movingDeforming.");
end
verbose = obstacleAvoidance.input.normalizeLogicalScalar( ...
    resolvedOptions.Verbose, "Verbose", ...
    "createContiguousUSObstacle:InvalidVerbose");
resolvedOptions.Verbose = verbose;
maximumOutlineVertices  = double(resolvedOptions.MaximumOutlineVertices);
if ~isscalar(maximumOutlineVertices) || ~isreal(maximumOutlineVertices) || ...
        isnan(maximumOutlineVertices) || maximumOutlineVertices < 3 || ...
        (isfinite(maximumOutlineVertices) && ...
        maximumOutlineVertices ~= floor(maximumOutlineVertices))
    error("createContiguousUSObstacle:InvalidMaximumOutlineVertices", ...
        "MaximumOutlineVertices must be Inf or an integer of at least 3.");
end
resolvedOptions.MaximumOutlineVertices = maximumOutlineVertices;

%% Section 2: Load One Dense Exterior Boundary

% Read state boundaries from Mapping Toolbox data. Keep the contiguous mainland
% states and combine them into one polygon. The largest exterior ring removes
% islands and holes that are not part of this example.

boundaryFile = which("usastatehi.shp");
if isempty(boundaryFile)
    error("createContiguousUSObstacle:MappingToolboxRequired", ...
        "Mapping Toolbox file usastatehi.shp was not found.");
end
if verbose
    fprintf("[U.S. obstacle] loading and unioning mainland boundaries...\n");
end
stateBoundary = shaperead(boundaryFile, "UseGeoCoords", true);
stateNames    = string({stateBoundary.Name});
stateBoundary = stateBoundary(~ismember(stateNames, ["Alaska" "Hawaii"]));
if isempty(stateBoundary)
    error("createContiguousUSObstacle:NoMainlandStates", "No contiguous-U.S. state boundaries were found.");
end
mainlandUS = polyshape(stateBoundary(1).Lon, stateBoundary(1).Lat, ...
    "Simplify", false, "KeepCollinearPoints", true);

% Join each remaining state polygon to the mainland polygon.
for stateIndex = 2:numel(stateBoundary)
    statePolygon = polyshape( ...
        stateBoundary(stateIndex).Lon, stateBoundary(stateIndex).Lat, ...
        "Simplify", false, "KeepCollinearPoints", true);
    mainlandUS   = union(mainlandUS, statePolygon);
    if verbose && (mod(stateIndex, 10) == 0 || stateIndex == numel(stateBoundary))
        fprintf("[U.S. obstacle] state union %d/%d complete.\n", stateIndex, numel(stateBoundary));
    end
end
[allLongitude_units, allLatitude_units]   = boundary(mainlandUS);
[baseLongitude_units, baseLatitude_units] = largestFiniteRing(allLongitude_units, allLatitude_units);
fullOutlineVertexCount = numel(baseLongitude_units);
if isfinite(maximumOutlineVertices) && fullOutlineVertexCount > maximumOutlineVertices
    reduced_units      = reduceClosedRing( ...
        [baseLongitude_units, baseLatitude_units], maximumOutlineVertices);
    baseLongitude_units = reduced_units(:, 1);
    baseLatitude_units  = reduced_units(:, 2);
    if verbose
        fprintf("[U.S. obstacle] outline reduced from %d to %d vertices.\n", ...
            fullOutlineVertexCount, numel(baseLongitude_units));
    end
end

%% Section 3: Delegate All Slice Work To The Generic Constructor

% The generic constructor calls the transform for each requested time. It owns
% slice validation, history metrics, and safety-margin protection.

time_s             = double(time_s(:));
missionStartTime_s = time_s(1);
missionDuration_s  = time_s(end) - missionStartTime_s;
baseCenter_units   = [mean(baseLongitude_units), mean(baseLatitude_units)];
basePosition_units = [baseLongitude_units, baseLatitude_units];
localRange_units   = max(basePosition_units - baseCenter_units, [], 1) - ...
    min(basePosition_units - baseCenter_units, [], 1);
sliceTransform = @(sourcePosition_units, sampleTime_s, sampleIndex) ...
    transformUSSlice(sourcePosition_units, sampleTime_s, sampleIndex, ...
    motionMode, missionStartTime_s, missionDuration_s, baseCenter_units, ...
    localRange_units);
[obstacle, history] = obstacleAvoidance.obstacles.createMovingObstacle( ...
    "Growing and rotating contiguous United States", time_s, ...
    baseLongitude_units, baseLatitude_units, sliceTransform, ...
    safetyMargin_units, struct("Verbose", verbose));
profile = extremeUSProfile(time_s, missionStartTime_s, missionDuration_s);
history.sourceOutlineLatLon_units = [baseLatitude_units, baseLongitude_units];
history.sourceOutlineVertexCount  = numel(baseLongitude_units);
history.fullOutlineVertexCount    = fullOutlineVertexCount;
history.scaleFactor               = profile.ScaleFactor(:);
history.rotation_deg              = profile.Rotation_deg(:);
history.translation_units         = profile.Translation_units;
history.deformationWeight         = profile.DeformationWeight(:);
history.ExampleOptions            = resolvedOptions;
end

%% Section 4: Local Functions

function transformed_units = transformUSSlice(sourcePosition_units, sampleTime_s, ~, ...
        motionMode, missionStartTime_s, missionDuration_s, baseCenter_units, ...
        localRange_units)
    % Return one U.S. slice. The generic constructor owns the time loop.
    if motionMode == "static" || missionDuration_s <= 0
        transformed_units = sourcePosition_units;
        return
    end
    missionProgress   = (sampleTime_s - missionStartTime_s) / missionDuration_s;
    profile           = extremeUSProfile(sampleTime_s, missionStartTime_s, missionDuration_s);
    phase_rad         = 2 * pi * missionProgress;
    baseLocal_units     = sourcePosition_units - baseCenter_units;
    deformedLocal_units = baseLocal_units;
    deformedLocal_units(:, 1) = deformedLocal_units(:, 1) + ...
        profile.DeformationWeight * 0.80 * ...
        sin(2 * pi * baseLocal_units(:, 2) / localRange_units(2) + phase_rad);
    deformedLocal_units(:, 2) = deformedLocal_units(:, 2) + ...
        profile.DeformationWeight * 0.55 * ...
        sin(2 * pi * baseLocal_units(:, 1) / localRange_units(1) - 0.7 * phase_rad);
    rotation_rad   = deg2rad(profile.Rotation_deg);
    rotationMatrix = [cos(rotation_rad) -sin(rotation_rad); sin(rotation_rad) cos(rotation_rad)];
    transformed_units = profile.ScaleFactor * deformedLocal_units * ...
        rotationMatrix.' + baseCenter_units + profile.Translation_units;
end

function profile = extremeUSProfile(sampleTime_s, missionStartTime_s, missionDuration_s)
    % Create one smooth time profile for geometry and diagnostics.
    if missionDuration_s <= 0
        missionProgress = zeros(size(sampleTime_s));
    else
        missionProgress = (sampleTime_s - missionStartTime_s) / missionDuration_s;
    end
    missionProgress = min(max(missionProgress, 0), 1);
    smoothProgress  = 10 * missionProgress.^3 - 15 * missionProgress.^4 + 6 * missionProgress.^5;
    profile         = struct("ScaleFactor", 0.08 + 1.27 * smoothProgress, ...
        "Rotation_deg", 180 * smoothProgress, ...
        "Translation_units", [ ...
            2.5 * sin(2 * pi * missionProgress(:)), ...
            1.5 * sin(pi * missionProgress(:))], ...
        "DeformationWeight", sin(pi * missionProgress));
end

function reduced_units = reduceClosedRing(ring_units, maximumVertexCount)
    % Reduce a closed ring with Douglas-Peucker and a bisection tolerance.
    % Split at the vertex farthest from the first so both open polylines keep
    % their endpoints. Uncross every fitting candidate at source vertices.
    % The final candidate must remain one simple ring within the vertex cap.
    vertexCount         = size(ring_units, 1);
    [~, anchorIndex]    = max(vecnorm(ring_units - ring_units(1, :), 2, 2));
    lowerTolerance_units = 0;
    upperTolerance_units = max(max(ring_units, [], 1) - min(ring_units, [], 1));
    keptIndex            = zeros(0, 1);
    for iteration = 1:64
        tolerance_units = (lowerTolerance_units + upperTolerance_units) / 2;
        keep                = false(vertexCount, 1);
        keep(1:anchorIndex) = douglasPeuckerKeep( ...
            ring_units(1:anchorIndex, :), tolerance_units);
        secondSegmentIndex = [anchorIndex:vertexCount, 1];
        keepSecond         = douglasPeuckerKeep( ...
            ring_units(secondSegmentIndex, :), tolerance_units);
        keep(anchorIndex:vertexCount) = keep(anchorIndex:vertexCount) | keepSecond(1:end - 1);
        candidateIndex = find(keep);
        if numel(candidateIndex) <= maximumVertexCount
            candidateIndex = uncrossReducedRing(ring_units, candidateIndex);
        end
        if numel(candidateIndex) > maximumVertexCount
            lowerTolerance_units = tolerance_units;
        else
            upperTolerance_units = tolerance_units;
            keptIndex = candidateIndex;
        end
    end
    if numel(keptIndex) < 3
        error("createContiguousUSObstacle:OutlineReductionFailed", ...
            "The outline could not be reduced to %d vertices.", maximumVertexCount);
    end
    reduced_units = ring_units(keptIndex, :);
    checkShape = polyshape(reduced_units(:, 1), reduced_units(:, 2), ...
        "Simplify", true, "KeepCollinearPoints", true);
    if checkShape.NumRegions ~= 1 || checkShape.NumHoles ~= 0 || ...
            size(checkShape.Vertices, 1) ~= size(reduced_units, 1) || ...
            ~all(ismember(checkShape.Vertices, reduced_units, "rows"))
        error("createContiguousUSObstacle:OutlineReductionFolded", "The reduced outline is not one simple ring.");
    end
end

function keptIndex = uncrossReducedRing(ring_units, keptIndex)
    % Split crossing reduced edges at the source vertex farthest from each
    % chord until no proper crossing remains. keptIndex is ascending and
    % begins at 1; its last reduced edge wraps to the first source vertex.
    % The simple source ring guarantees an available split and termination.
    vertexCount = size(ring_units, 1);
    while true
        crossingPairs = properEdgeCrossings(ring_units(keptIndex, :));
        if isempty(crossingPairs)
            return
        end
        addedIndex = zeros(0, 1);
        for edgeIndex = unique(crossingPairs(:)).'
            firstSourceIndex = keptIndex(edgeIndex);
            if edgeIndex < numel(keptIndex)
                lastSourceIndex = keptIndex(edgeIndex + 1);
            else
                lastSourceIndex = vertexCount + 1;
            end
            interiorIndex = (firstSourceIndex + 1:lastSourceIndex - 1).';
            if isempty(interiorIndex)
                continue
            end
            chordEnd_units    = ring_units(mod(lastSourceIndex - 1, vertexCount) + 1, :);
            chord_units       = chordEnd_units - ring_units(firstSourceIndex, :);
            offset_units      = ring_units(interiorIndex, :) - ring_units(firstSourceIndex, :);
            chordLength_units = norm(chord_units);
            if chordLength_units > 0
                distance_units = abs(offset_units(:, 1) * chord_units(2) - ...
                    offset_units(:, 2) * chord_units(1)) / chordLength_units;
            else
                distance_units = vecnorm(offset_units, 2, 2);
            end
            [~, farthestOffsetIndex] = max(distance_units);
            addedIndex(end + 1, 1) = interiorIndex(farthestOffsetIndex); %#ok<AGROW>
        end
        if isempty(addedIndex)
            error("createContiguousUSObstacle:OutlineReductionFolded", ...
                "The reduced outline is not one simple ring.");
        end
        keptIndex = sort([keptIndex; addedIndex]);
    end
end

function crossingPairs = properEdgeCrossings(ring_units)
    % Index pairs (i, j), i < j, of non-adjacent ring edges that cross at
    % one interior point of both (strict orientation test on both sides).
    edgeCount    = size(ring_units, 1);
    start_units  = ring_units;
    finish_units = ring_units([2:edgeCount, 1], :);
    [firstEdgeIndex, secondEdgeIndex] = find(triu(true(edgeCount), 2));
    edgesAreAdjacent = firstEdgeIndex == 1 & secondEdgeIndex == edgeCount;
    firstEdgeIndex(edgesAreAdjacent)  = [];
    secondEdgeIndex(edgesAreAdjacent) = [];
    orientation = @(firstPoint, secondPoint, thirdPoint) ...
        (secondPoint(:, 1) - firstPoint(:, 1)) .* ...
        (thirdPoint(:, 2) - firstPoint(:, 2)) - ...
        (secondPoint(:, 2) - firstPoint(:, 2)) .* ...
        (thirdPoint(:, 1) - firstPoint(:, 1));
    firstStart_units  = start_units(firstEdgeIndex, :);
    firstFinish_units = finish_units(firstEdgeIndex, :);
    secondStart_units  = start_units(secondEdgeIndex, :);
    secondFinish_units = finish_units(secondEdgeIndex, :);
    edgesCross = orientation(firstStart_units, firstFinish_units, secondStart_units) .* ...
        orientation(firstStart_units, firstFinish_units, secondFinish_units) < 0 & ...
        orientation(secondStart_units, secondFinish_units, firstStart_units) .* ...
        orientation(secondStart_units, secondFinish_units, firstFinish_units) < 0;
    crossingPairs = [firstEdgeIndex(edgesCross), secondEdgeIndex(edgesCross)];
end

function keep = douglasPeuckerKeep(points_units, tolerance_units)
    % Iterative Douglas-Peucker on an open polyline: retain endpoints and every
    % vertex whose distance from the current chord exceeds the tolerance.
    pointCount            = size(points_units, 1);
    keep                  = false(pointCount, 1);
    keep([1, pointCount]) = true;
    stack                 = [1, pointCount];
    while ~isempty(stack)
        firstIndex    = stack(end, 1);
        lastIndex     = stack(end, 2);
        stack(end, :) = [];
        if lastIndex - firstIndex < 2
            continue
        end
        chord_units       = points_units(lastIndex, :) - points_units(firstIndex, :);
        interiorIndex     = (firstIndex + 1:lastIndex - 1).';
        offset_units      = points_units(interiorIndex, :) - points_units(firstIndex, :);
        chordLength_units = norm(chord_units);
        if chordLength_units > 0
            distance_units = abs(offset_units(:, 1) * chord_units(2) - ...
                offset_units(:, 2) * chord_units(1)) / chordLength_units;
        else
            distance_units = vecnorm(offset_units, 2, 2);
        end
        [farthest_units, farthestOffsetIndex] = max(distance_units);
        if farthest_units > tolerance_units
            splitIndex      = interiorIndex(farthestOffsetIndex);
            keep(splitIndex) = true;
            stack = [stack; firstIndex, splitIndex; splitIndex, lastIndex]; %#ok<AGROW>
        end
    end
end

function [largestX, largestY] = largestFiniteRing(x, y)
    % Keep the largest finite boundary ring. Discard holes and islands.
    x          = double(x(:));
    y          = double(y(:));
    finiteRows = isfinite(x) & isfinite(y);
    changes    = diff([false; finiteRows; false]);
    ringStart  = find(changes == 1);
    ringStop   = find(changes == -1) - 1;
    if isempty(ringStart)
        error("createContiguousUSObstacle:EmptyOutline", ...
            "The state union did not produce a finite exterior boundary.");
    end
    ringArea = zeros(numel(ringStart), 1);

    % Measure each finite boundary ring. Keep the largest mainland outline.
    for ringIndex = 1:numel(ringStart)
        rows = ringStart(ringIndex):ringStop(ringIndex);
        ringArea(ringIndex) = abs(polyarea(x(rows), y(rows)));
    end
    [~, largestRingIndex] = max(ringArea);
    rows     = ringStart(largestRingIndex):ringStop(largestRingIndex);
    largestX = x(rows);
    largestY = y(rows);
    if largestX(1) == largestX(end) && largestY(1) == largestY(end)
        largestX(end) = [];
        largestY(end) = [];
    end
end
