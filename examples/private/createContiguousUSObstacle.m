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
%   - safetyMargin_units (nonnegative scalar)
%   - options (scalar struct, optional)
%       .MotionMode is static or movingDeforming (default static).
%       .Verbose is logical (default false).
%       .MaximumOutlineVertices caps the source outline vertex count by a
%        deterministic Douglas-Peucker reduction (default Inf, no reduction).
%        The reduced outline is the supplied obstacle; the planner treats it
%        exactly and applies no further simplification.
%**************************************************************************
% OUTPUTS
%   - obstacle (canonical protected moving obstacle)
%   - history (generic slice history plus source outline metadata)
%**************************************************************************
% UNITS
%   - Longitude/latitude are treated as x/y coordinate units; time_s
%     is seconds and safetyMargin_units is coordinate units.
%**************************************************************************

%% Section 1: Validate Inputs & Apply Defaults

% Check the time history, margin, and motion mode before map data is loaded.
% Static mode repeats one outline. Moving mode applies a known transform at each
% time so the generic moving-obstacle function can validate every slice.

if nargin < 3 || isempty(options)
    options = struct();
end
if ~isstruct(options) || ~isscalar(options)
    error("createContiguousUSObstacle:InvalidOptions", "options must be a scalar struct.");
end
defaultOptions = struct();
defaultOptions.MotionMode = "static";
defaultOptions.Verbose    = false;
defaultOptions.MaximumOutlineVertices = Inf;
[resolvedOptions, unknownOptionFields] = obstacleAvoidance.input.resolveOptions(defaultOptions, options);
if ~isempty(unknownOptionFields)
    warning("createContiguousUSObstacle:UnknownOptions", "Ignoring unknown option fields: %s. No behavior changed.", strjoin(unknownOptionFields, ", "));
end
motionMode = lower(string(resolvedOptions.MotionMode));
if ~isscalar(motionMode) || ~any(motionMode == ["static" "movingdeforming"])
    error("createContiguousUSObstacle:InvalidMotionMode", "MotionMode must be static or movingDeforming.");
end
verbose = obstacleAvoidance.input.normalizeLogicalScalar(resolvedOptions.Verbose, "Verbose", "createContiguousUSObstacle:InvalidVerbose");
resolvedOptions.Verbose = verbose;
maximumOutlineVertices = double(resolvedOptions.MaximumOutlineVertices);
if ~isscalar(maximumOutlineVertices) || ~isreal(maximumOutlineVertices) || isnan(maximumOutlineVertices) || maximumOutlineVertices < 3 || ...
        (isfinite(maximumOutlineVertices) && maximumOutlineVertices ~= floor(maximumOutlineVertices))
    error("createContiguousUSObstacle:InvalidMaximumOutlineVertices", "MaximumOutlineVertices must be Inf or an integer of at least 3.");
end
resolvedOptions.MaximumOutlineVertices = maximumOutlineVertices;

%% Section 2: Load One Dense Exterior Boundary

% Read state boundaries from Mapping Toolbox data. Keep the contiguous mainland
% states and combine them into one polygon. The largest exterior ring removes
% islands and holes that are not part of this example.

boundaryFile = which("usastatehi.shp");
if isempty(boundaryFile)
    error("createContiguousUSObstacle:MappingToolboxRequired", "Mapping Toolbox file usastatehi.shp was not found.");
end
if verbose
    fprintf("[U.S. obstacle] loading and unioning mainland boundaries...\n");
end
stateBoundary = shaperead(boundaryFile, "UseGeoCoords", true);
stateName     = string({stateBoundary.Name});
stateBoundary = stateBoundary(~ismember(stateName, ["Alaska" "Hawaii"]));
if isempty(stateBoundary)
    error("createContiguousUSObstacle:NoMainlandStates", "No contiguous-U.S. state boundaries were found.");
end
mainlandUS = polyshape(stateBoundary(1).Lon, stateBoundary(1).Lat, "Simplify", false, "KeepCollinearPoints", true);

% Join each remaining state polygon to the mainland polygon.
for stateIndex = 2:numel(stateBoundary)
    statePolygon = polyshape(stateBoundary(stateIndex).Lon, stateBoundary(stateIndex).Lat, "Simplify", false, "KeepCollinearPoints", true);
    mainlandUS   = union(mainlandUS, statePolygon);
    if verbose && (mod(stateIndex, 10) == 0 || stateIndex == numel(stateBoundary))
        fprintf("[U.S. obstacle] state union %d/%d complete.\n", stateIndex, numel(stateBoundary));
    end
end
[allLongitude_units, allLatitude_units]   = boundary(mainlandUS);
[baseLongitude_units, baseLatitude_units] = largestFiniteRing(allLongitude_units, allLatitude_units);
fullOutlineVertexCount = numel(baseLongitude_units);
if isfinite(maximumOutlineVertices) && fullOutlineVertexCount > maximumOutlineVertices
    reduced_units = reduceClosedRing([baseLongitude_units, baseLatitude_units], maximumOutlineVertices);
    baseLongitude_units = reduced_units(:, 1);
    baseLatitude_units  = reduced_units(:, 2);
    if verbose
        fprintf("[U.S. obstacle] outline reduced from %d to %d vertices.\n", fullOutlineVertexCount, numel(baseLongitude_units));
    end
end

%% Section 3: Delegate All Slice Work To The Generic Constructor

% The generic constructor calls the transform for each requested time. It owns
% slice validation, history metrics, and safety-margin protection.

time_s             = double(time_s(:));
missionStartTime_s = time_s(1);
missionDuration_s  = time_s(end) - missionStartTime_s;
baseCenter_units     = [mean(baseLongitude_units), mean(baseLatitude_units)];
basePosition_units   = [baseLongitude_units, baseLatitude_units];
localRange_units     = max(basePosition_units - baseCenter_units, [], 1) - min(basePosition_units - baseCenter_units, [], 1);
sliceTransform     = @(sourcePosition_units, sampleTime_s, sampleIndex) transformUSSlice(sourcePosition_units, sampleTime_s, sampleIndex, motionMode, missionStartTime_s, missionDuration_s, baseCenter_units, localRange_units);
[obstacle, history] = obstacleAvoidance.obstacles.createMovingObstacle("Growing and rotating contiguous United States", time_s, baseLongitude_units, baseLatitude_units, sliceTransform, safetyMargin_units, struct("Verbose", verbose));
profile = extremeUSProfile(time_s, missionStartTime_s, missionDuration_s);
history.motionMode               = motionMode;
history.sourceFile               = string(boundaryFile);
history.sourceOutlineLatLon_units  = [baseLatitude_units, baseLongitude_units];
history.sourceOutlineVertexCount = numel(baseLongitude_units);
history.fullOutlineVertexCount   = fullOutlineVertexCount;
history.scaleFactor              = profile.ScaleFactor(:);
history.rotation_deg             = profile.Rotation_deg(:);
history.translation_units          = profile.Translation_units;
history.deformationWeight        = profile.DeformationWeight(:);
history.ExampleOptions           = resolvedOptions;
end


function transformed_units = transformUSSlice(sourcePosition_units, sampleTime_s, ~, motionMode, missionStartTime_s, missionDuration_s, baseCenter_units, localRange_units)
    % Return one U.S. slice. The generic constructor owns the time loop.
    if motionMode == "static" || missionDuration_s <= 0
        transformed_units = sourcePosition_units;
        return;
    end
    missionProgress   = (sampleTime_s - missionStartTime_s) / missionDuration_s;
    profile           = extremeUSProfile(sampleTime_s, missionStartTime_s, missionDuration_s);
    phase_rad         = 2 * pi * missionProgress;
    baseLocal_units     = sourcePosition_units - baseCenter_units;
    deformedLocal_units = baseLocal_units;
    deformedLocal_units(:, 1) = deformedLocal_units(:, 1) + profile.DeformationWeight * 0.80 * sin(2 * pi * baseLocal_units(:, 2) / localRange_units(2) + phase_rad);
    deformedLocal_units(:, 2) = deformedLocal_units(:, 2) + profile.DeformationWeight * 0.55 * sin(2 * pi * baseLocal_units(:, 1) / localRange_units(1) - 0.7 * phase_rad);
    rotation_rad    = deg2rad(profile.Rotation_deg);
    rotationMatrix  = [cos(rotation_rad) -sin(rotation_rad); sin(rotation_rad) cos(rotation_rad)];
    transformed_units = profile.ScaleFactor * deformedLocal_units * rotationMatrix.' + baseCenter_units + profile.Translation_units;
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
    % Reduce a simple closed ring to at most maximumVertexCount vertices with
    % the Douglas-Peucker rule, choosing the smallest distance tolerance that
    % meets the cap by bisection. The ring is split at its first vertex and
    % the vertex farthest from it so both halves are open polylines whose
    % endpoints are always retained. Douglas-Peucker alone can fold an
    % outline (a chord across a bay can cross the far shore), so every
    % candidate that fits the cap is uncrossed by splitting each crossing
    % chord at the source vertex farthest from it; the uncrossed candidate
    % must still fit the cap. The result must be one simple ring; anything
    % else is an error, not a repair.
    vertexCount = size(ring_units, 1);
    [~, anchorIndex] = max(vecnorm(ring_units - ring_units(1, :), 2, 2));
    lowerTolerance_units = 0;
    upperTolerance_units = max(max(ring_units, [], 1) - min(ring_units, [], 1));
    keptIndex = zeros(0, 1);
    for iteration = 1:64
        tolerance_units = (lowerTolerance_units + upperTolerance_units) / 2;
        keep = false(vertexCount, 1);
        keep(1:anchorIndex) = douglasPeuckerKeep(ring_units(1:anchorIndex, :), tolerance_units);
        keepSecond = douglasPeuckerKeep(ring_units([anchorIndex:vertexCount, 1], :), tolerance_units);
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
        error("createContiguousUSObstacle:OutlineReductionFailed", "The outline could not be reduced to %d vertices.", maximumVertexCount);
    end
    reduced_units = ring_units(keptIndex, :);
    checkShape = polyshape(reduced_units(:, 1), reduced_units(:, 2), "Simplify", true, "KeepCollinearPoints", true);
    if checkShape.NumRegions ~= 1 || checkShape.NumHoles ~= 0 || size(checkShape.Vertices, 1) ~= size(reduced_units, 1) || ...
            ~all(ismember(checkShape.Vertices, reduced_units, "rows"))
        error("createContiguousUSObstacle:OutlineReductionFolded", "The reduced outline is not one simple ring.");
    end
end

function keptIndex = uncrossReducedRing(ring_units, keptIndex)
    % Split every reduced edge that properly crosses another reduced edge at
    % the source vertex farthest from its chord, until no proper crossing
    % remains. keptIndex is an ascending list of source indices starting at
    % 1, so reduced edge k runs over source vertices keptIndex(k) to
    % keptIndex(k+1), and the last edge wraps to vertex 1. The source ring is
    % simple, so at least one edge of every crossing pair has interior source
    % vertices, and the loop ends at the source ring at worst.
    vertexCount = size(ring_units, 1);
    while true
        crossing = properEdgeCrossings(ring_units(keptIndex, :));
        if isempty(crossing)
            return;
        end
        added = zeros(0, 1);
        for edge = unique(crossing(:)).'
            firstSource = keptIndex(edge);
            if edge < numel(keptIndex)
                lastSource = keptIndex(edge + 1);
            else
                lastSource = vertexCount + 1;
            end
            interior = (firstSource + 1:lastSource - 1).';
            if isempty(interior)
                continue;
            end
            chordEnd_units = ring_units(mod(lastSource - 1, vertexCount) + 1, :);
            chord_units    = chordEnd_units - ring_units(firstSource, :);
            offset_units   = ring_units(interior, :) - ring_units(firstSource, :);
            chordLength_units = norm(chord_units);
            if chordLength_units > 0
                distance_units = abs(offset_units(:, 1) * chord_units(2) - offset_units(:, 2) * chord_units(1)) / chordLength_units;
            else
                distance_units = vecnorm(offset_units, 2, 2);
            end
            [~, farthestOffset] = max(distance_units);
            added(end + 1, 1) = interior(farthestOffset); %#ok<AGROW>
        end
        if isempty(added)
            error("createContiguousUSObstacle:OutlineReductionFolded", "The reduced outline is not one simple ring.");
        end
        keptIndex = sort([keptIndex; added]);
    end
end

function pairs = properEdgeCrossings(ring_units)
    % Index pairs (i, j), i < j, of non-adjacent ring edges that cross at
    % one interior point of both (strict orientation test on both sides).
    edgeCount = size(ring_units, 1);
    start_units  = ring_units;
    finish_units = ring_units([2:edgeCount, 1], :);
    [i, j] = find(triu(true(edgeCount), 2));
    adjacent = i == 1 & j == edgeCount;
    i(adjacent) = []; j(adjacent) = [];
    orientation = @(p, q, r) (q(:, 1) - p(:, 1)) .* (r(:, 2) - p(:, 2)) - (q(:, 2) - p(:, 2)) .* (r(:, 1) - p(:, 1));
    a = start_units(i, :); b = finish_units(i, :); c = start_units(j, :); d = finish_units(j, :);
    crosses = orientation(a, b, c) .* orientation(a, b, d) < 0 & orientation(c, d, a) .* orientation(c, d, b) < 0;
    pairs = [i(crosses), j(crosses)];
end

function keep = douglasPeuckerKeep(points_units, tolerance_units)
    % Iterative Douglas-Peucker on an open polyline: retain endpoints and every
    % vertex whose distance from the current chord exceeds the tolerance.
    count = size(points_units, 1);
    keep  = false(count, 1);
    keep([1, count]) = true;
    stack = [1, count];
    while ~isempty(stack)
        first = stack(end, 1); last = stack(end, 2); stack(end, :) = [];
        if last - first < 2, continue; end
        chord_units = points_units(last, :) - points_units(first, :);
        interior = (first + 1:last - 1).';
        offset_units = points_units(interior, :) - points_units(first, :);
        chordLength_units = norm(chord_units);
        if chordLength_units > 0
            distance_units = abs(offset_units(:, 1) * chord_units(2) - offset_units(:, 2) * chord_units(1)) / chordLength_units;
        else
            distance_units = vecnorm(offset_units, 2, 2);
        end
        [farthest_units, farthestOffset] = max(distance_units);
        if farthest_units > tolerance_units
            split = interior(farthestOffset);
            keep(split) = true;
            stack = [stack; first, split; split, last]; %#ok<AGROW>
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
        error("createContiguousUSObstacle:EmptyOutline", "The state union did not produce a finite exterior boundary.");
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
