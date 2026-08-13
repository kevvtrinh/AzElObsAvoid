function obstacleField = buildAzElTimeObstacleField( ...
        azElData, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = buildAzElTimeObstacleField()
%   obstacleField = buildAzElTimeObstacleField(azElData)
%   obstacleField = buildAzElTimeObstacleField(azElData, options)
%**************************************************************************
% PURPOSE
%   - Pack validated moving azimuth/elevation polygons for repeated,
%     allocation-light collision queries.
%**************************************************************************
% INPUTS
%   - azElData (struct array, cell array, or [])
%       Canonical obstacle samples. Nested cells and struct arrays are
%       flattened in caller order. [] creates a valid zero-obstacle field.
%       NaN vertex rows separate polygon regions.
%   - optionOverrides (scalar struct)
%       .MaximumVerticesPerRegion (positive integer or Inf)
%           Boundary cap per region. Finite lossy caps are rejected because
%           collision geometry must conservatively contain the source.
%       .ReferenceTime (datetime scalar)
%           UTC epoch corresponding to time_s == 0.
%       .Verbose (logical scalar)
%           Print completed per-slice packing progress (default false).
%**************************************************************************
% OUTPUTS
%   - obstacleField (scalar struct)
%       Packed obstacle records, resolved options, and storage diagnostics.
%       A zero-argument call returns the fully populated options structure.
%**************************************************************************
% UNITS
%   - Input boundary fields az_deg and el_deg are degrees.
%   - Input time_s and packed TimeSeconds values are seconds.

%% Section 1: Validate Inputs & Apply Defaults
defaultOptions = defaultAzElTimeObstacleFieldOptions();
if nargin == 0
    obstacleField = defaultOptions;
    return;
end
if nargin < 2 || isempty(optionOverrides)
    optionOverrides = struct();
end
if ~isstruct(optionOverrides) || ~isscalar(optionOverrides)
    error("buildAzElTimeObstacleField:InvalidOptions", ...
        "options must be a scalar struct.");
end
unknownOptionFields = setdiff( ...
    fieldnames(optionOverrides), fieldnames(defaultOptions), "stable");
if ~isempty(unknownOptionFields)
    warning("buildAzElTimeObstacleField:UnknownOptions", ...
        "Ignoring unknown option fields: %s.", ...
        strjoin(string(unknownOptionFields), ", "));
    optionOverrides = rmfield(optionOverrides, unknownOptionFields);
end
resolvedOptions = defaultOptions;
providedOptionFields = fieldnames(optionOverrides);
for optionIndex = 1:numel(providedOptionFields)
    optionName = providedOptionFields{optionIndex};
    if ~isempty(optionOverrides.(optionName))
        resolvedOptions.(optionName) = optionOverrides.(optionName);
    end
end
validateattributes(resolvedOptions.MaximumVerticesPerRegion, {'numeric'}, ...
    {'scalar', 'real', 'positive'});
if isfinite(resolvedOptions.MaximumVerticesPerRegion)
    validateattributes(resolvedOptions.MaximumVerticesPerRegion, {'numeric'}, ...
        {'integer', '>=', 4});
end
maximumVerticesPerRegion = resolvedOptions.MaximumVerticesPerRegion;
validateattributes(resolvedOptions.Verbose, ...
    {'logical','numeric'}, {'scalar'});
resolvedOptions.Verbose = logical(resolvedOptions.Verbose);
if isfinite(maximumVerticesPerRegion)
    error("buildAzElTimeObstacleField:NonconservativeVertexCap", ...
        "Finite MaximumVerticesPerRegion values can shrink protected " + ...
        "geometry. Use Inf so packed collision geometry remains exact.");
end

% The reference epoch is the one input still checked here, because
% queryAzElTimeObstacle reads it from the obstacle field on every collision
% query and a wrong-typed value there fails far from this file.
if isempty(resolvedOptions.ReferenceTime)
    referenceTime = datetime(1970, 1, 1, 0, 0, 0, "TimeZone", "UTC");
else
    referenceTime = resolvedOptions.ReferenceTime;
    if ~isdatetime(referenceTime) || ~isscalar(referenceTime) || ...
            isnat(referenceTime)
        error("buildAzElTimeObstacleField:InvalidTime", ...
            "ReferenceTime must be a finite datetime scalar.");
    end
    referenceTime.TimeZone = "UTC";
end

% Separation in degrees below which two vertices count as the same point.
% Used both to spot an explicitly closed ring and to drop zero-length edges,
% so the two always agree on whether first and last vertices coincide.
tolerance_deg = 1e-12;

%% Section 2: Normalize & Pack Obstacles
canonicalObstacles = combineAzElObstacles(azElData);

% combineAzElObstacles owns recursive flattening and returns a canonical
% column array, so packing never needs a second compatibility branch.
% The shared empty template fixes field order so struct-array assignment in
% the loop cannot fail.
packedObstacles = repmat( ...
    emptyPackedObstacle(), numel(canonicalObstacles), 1);
for obstacleIndex = 1:numel(canonicalObstacles)
    obstacleData = canonicalObstacles(obstacleIndex);
    sliceTime_s = double(obstacleData.time_s(:));
    sliceCount = numel(sliceTime_s);
    if resolvedOptions.Verbose
        fprintf("[az/el pack] obstacle %d/%d '%s': %d slices.\n", ...
            obstacleIndex, numel(canonicalObstacles), ...
            obstacleData.targetName, sliceCount);
    end
    reducedRegionCount = 0;
    droppedRegionCount = 0;

    % One buffer entry per time slice, filled by the slice loop below and
    % concatenated afterwards. Buffering costs one transient copy but means
    % the packed arrays and the offset tables are both derived from the same
    % data, so they cannot describe different geometry.
    %
    % DIAGNOSING A BAD SLICE: sliceVertexCounts(sliceIndex) == 0 means the
    % slice contributed nothing, either because no paired finite vertices
    % arrived or every ring was too small. It intentionally becomes an empty
    % CSR range and therefore blocks no query at that sample.
    sliceAzimuthBuffer_deg = cell(sliceCount, 1);
    sliceElevationBuffer_deg = cell(sliceCount, 1);
    sliceEdgeBuffer_deg = cell(sliceCount, 1);
    sliceVertexCounts = zeros(sliceCount, 1);   % vertex rows written per slice
    sliceEdgeCounts = zeros(sliceCount, 1);     % edge rows written per slice
    sliceBounds_deg = nan(sliceCount, 4);

    for sliceIndex = 1:sliceCount
        inputAzimuth_deg = double(obstacleData.az_deg{sliceIndex}(:));
        inputElevation_deg = double(obstacleData.el_deg{sliceIndex}(:));
        % Tolerate mismatched az/el lengths by taking the common prefix.
        usableCount = min( ...
            numel(inputAzimuth_deg), numel(inputElevation_deg));
        inputAzimuth_deg = inputAzimuth_deg(1:usableCount);
        inputElevation_deg = inputElevation_deg(1:usableCount);
        % A vertex counts only if BOTH coordinates are finite. NaN rows are
        % the ring separators, so this mask is also the ring structure.
        isRealVertex = ...
            isfinite(inputAzimuth_deg) & isfinite(inputElevation_deg);

        % Broad-phase reject box, used by queryAzElTimeObstacle to skip the
        % expensive point-in-polygon test. Taken from the raw finite vertices
        % BEFORE any reduction, so it always encloses the full-resolution
        % boundary and can never reject a point the polygon would contain.
        % A slice with nothing usable (target below horizon, no data) keeps
        % its all-NaN box, which compares false and rejects everything.
        if any(isRealVertex)
            sliceBounds_deg(sliceIndex, :) = [ ...
                min(inputAzimuth_deg(isRealVertex)), ...
                max(inputAzimuth_deg(isRealVertex)), ...
                min(inputElevation_deg(isRealVertex)), ...
                max(inputElevation_deg(isRealVertex))];
        end

        % Locate each maximal run of real vertices; one run is one ring. The
        % false sentinels at both ends make a run that touches either end
        % produce a transition like any other, so no special casing is
        % needed. +1 marks the first vertex of a run, -1 the first vertex
        % after it, hence the -1 on ringLastVertex.
        runTransitions = diff([false; isRealVertex; false]);
        ringFirstVertex = find(runTransitions == 1);
        ringLastVertex = find(runTransitions == -1) - 1;

        % Each ring is reduced and turned into edges on its own. Nothing is
        % ever computed across ring boundaries, which is what stops a phantom
        % edge from bridging two disconnected regions of the same slice.
        keptRingAzimuth_deg = cell(numel(ringFirstVertex), 1);
        keptRingElevation_deg = cell(numel(ringFirstVertex), 1);
        keptRingEdges_deg = cell(numel(ringFirstVertex), 1);
        keptRingCount = 0;        % rings that survived to be packed
        keptVertexTotal = 0;      % their combined vertex count
        keptEdgeTotal = 0;        % their combined edge count

        for ringIndex = 1:numel(ringFirstVertex)
            firstRingVertex = ringFirstVertex(ringIndex);
            lastRingVertex = ringLastVertex(ringIndex);
            ringVertexRows = firstRingVertex:lastRingVertex;
            if numel(ringVertexRows) < 3
                droppedRegionCount = droppedRegionCount + 1;
                continue;  % fewer than 3 vertices cannot form a polygon
            end
            ringAzimuth_deg = inputAzimuth_deg(ringVertexRows);
            ringElevation_deg = inputElevation_deg(ringVertexRows);

            % Cap the ring by uniform index subsampling. This is a storage
            % and performance control, NOT a geometric simplifier: it does
            % not preserve area or shape, it just thins vertices evenly. A
            % ring that arrives explicitly closed stays explicitly closed.
            inputRingSize = numel(ringAzimuth_deg);
            hasFiniteVertexCap = isfinite(maximumVerticesPerRegion);
            exceedsVertexCap = inputRingSize > maximumVerticesPerRegion;
            shouldReduceRing = hasFiniteVertexCap && exceedsVertexCap;
            if shouldReduceRing
                reducedRegionCount = reducedRegionCount + 1;
                % "Explicitly closed" means the last vertex repeats the
                % first. Such a ring must keep that property, or the closing
                % edge below would be computed from the wrong pair.
                isExplicitlyClosed = inputRingSize > 3 && ...
                    hypot( ...
                    ringAzimuth_deg(1) - ringAzimuth_deg(end), ...
                    ringElevation_deg(1) - ringElevation_deg(end)) <= ...
                    tolerance_deg;
                if isExplicitlyClosed
                    % Thin the unique vertices only -- range stops one short
                    % of the duplicate closer -- then re-append the first
                    % kept vertex. That is one fewer unique vertex than the
                    % cap plus one repeat, so the cap is respected exactly.
                    keptVertexIndex = round(linspace( ...
                        1, inputRingSize - 1, ...
                        maximumVerticesPerRegion - 1));
                    ringAzimuth_deg = [ ...
                        ringAzimuth_deg(keptVertexIndex); ...
                        ringAzimuth_deg(keptVertexIndex(1))];
                    ringElevation_deg = [ ...
                        ringElevation_deg(keptVertexIndex); ...
                        ringElevation_deg(keptVertexIndex(1))];
                else
                    keptVertexIndex = round(linspace( ...
                        1, inputRingSize, maximumVerticesPerRegion));
                    ringAzimuth_deg = ringAzimuth_deg(keptVertexIndex);
                    ringElevation_deg = ...
                        ringElevation_deg(keptVertexIndex);
                end
                % round(linspace(...)) can repeat an index when the cap is
                % close to the ring size. Harmless: repeats become
                % zero-length edges and are dropped just below.
            end
            if numel(ringAzimuth_deg) < 3
                droppedRegionCount = droppedRegionCount + 1;
                continue;  % ring degenerated during reduction; drop it
            end

            % Build the closed edge list. circshift(-1) pairs vertex i with
            % i+1 and wraps the last back to the first, so the closing edge
            % is produced without appending anything. Zero-length edges are
            % then dropped, which quietly absorbs both explicit closing
            % vertices and any repeated points. Edges come straight off this
            % ring rather than from a rescan of the reassembled slice, so
            % the vertex list and the edge list cannot disagree.
            edgeEndAzimuth_deg = circshift(ringAzimuth_deg, -1);
            edgeEndElevation_deg = circshift(ringElevation_deg, -1);
            edgeHasLength = hypot( ...
                edgeEndAzimuth_deg - ringAzimuth_deg, ...
                edgeEndElevation_deg - ringElevation_deg) > tolerance_deg;

            keptRingCount = keptRingCount + 1;
            keptRingAzimuth_deg{keptRingCount} = ringAzimuth_deg;
            keptRingElevation_deg{keptRingCount} = ringElevation_deg;
            keptRingEdges_deg{keptRingCount} = [ ...
                ringAzimuth_deg(edgeHasLength), ...
                ringElevation_deg(edgeHasLength), ...
                edgeEndAzimuth_deg(edgeHasLength), ...
                edgeEndElevation_deg(edgeHasLength)];
            keptVertexTotal = keptVertexTotal + numel(ringAzimuth_deg);
            keptEdgeTotal = keptEdgeTotal + nnz(edgeHasLength);
        end

        if keptRingCount == 0
            % Nothing survived. Empty parts keep the concatenation below
            % type-correct; the two count entries stay 0.
            sliceAzimuthBuffer_deg{sliceIndex} = zeros(0, 1);
            sliceElevationBuffer_deg{sliceIndex} = zeros(0, 1);
            sliceEdgeBuffer_deg{sliceIndex} = zeros(0, 4);
            if resolvedOptions.Verbose
                fprintf( ...
                    "[az/el pack] slice %d/%d at t=%.3f s: " + ...
                    "empty protected geometry.\n", ...
                    sliceIndex, sliceCount, sliceTime_s(sliceIndex));
            end
            continue;
        end

        % Reassemble the slice in the same NaN-separated form the input
        % used, so a packed slice reads back like the data it came from.
        % The array is prefilled with NaN and the cursor advances by
        % ringSize+1, skipping one slot between rings -- that skipped slot
        % IS the separator, so no separator is ever written explicitly.
        % Length is vertices + (rings - 1) separators.
        sliceVertexAzimuth_deg = ...
            nan(keptVertexTotal + keptRingCount - 1, 1);
        sliceVertexElevation_deg = sliceVertexAzimuth_deg;
        nextRingWriteRow = 1;
        for ringIndex = 1:keptRingCount
            ringSize = numel(keptRingAzimuth_deg{ringIndex});
            lastRingWriteRow = nextRingWriteRow + ringSize - 1;
            ringWriteRows = nextRingWriteRow:lastRingWriteRow;
            sliceVertexAzimuth_deg(ringWriteRows) = keptRingAzimuth_deg{ringIndex};
            sliceVertexElevation_deg(ringWriteRows) = keptRingElevation_deg{ringIndex};
            nextRingWriteRow = nextRingWriteRow + ringSize + 1;
        end

        sliceAzimuthBuffer_deg{sliceIndex} = sliceVertexAzimuth_deg;
        sliceElevationBuffer_deg{sliceIndex} = sliceVertexElevation_deg;
        sliceEdgeBuffer_deg{sliceIndex} = ...
            vertcat(keptRingEdges_deg{1:keptRingCount});
        % Counts are taken from what was actually written, never predicted,
        % which is what keeps the offset tables below honest.
        sliceVertexCounts(sliceIndex) = numel(sliceVertexAzimuth_deg);
        sliceEdgeCounts(sliceIndex) = keptEdgeTotal;
        if resolvedOptions.Verbose
            fprintf( ...
                "[az/el pack] slice %d/%d at t=%.3f s: " + ...
                "%d vertices, %d rings, %d edges.\n", ...
                sliceIndex, sliceCount, sliceTime_s(sliceIndex), ...
                sliceVertexCounts(sliceIndex), keptRingCount, ...
                sliceEdgeCounts(sliceIndex));
        end
    end

    % CSR-style offset tables. Slice i owns rows
    % sliceVertexOffsets(i) .. sliceVertexOffsets(i+1)-1 of the packed
    % arrays; an empty slice leaves equal adjacent offsets and therefore a
    % zero-width range.
    % Offsets are 1-based to index MATLAB arrays directly.
    sliceVertexOffsets = ones(sliceCount + 1, 1, "uint64");
    cumulativeVertexCounts = cumsum(uint64(sliceVertexCounts));
    sliceVertexOffsets(2:end) = uint64(1) + cumulativeVertexCounts;
    sliceEdgeOffsets = ones(sliceCount + 1, 1, "uint64");
    cumulativeEdgeCounts = cumsum(uint64(sliceEdgeCounts));
    sliceEdgeOffsets(2:end) = uint64(1) + cumulativeEdgeCounts;

    % Vertices support reconstruction and plotting; explicit edges support
    % the vectorized point-in-polygon and safety-margin narrow phases. The
    % zero-slice branch exists because vertcat of an empty cell yields [],
    % which would be the wrong type and shape for the packed fields.
    if sliceCount == 0
        packedAzimuth_deg = zeros(0, 1);
        packedElevation_deg = zeros(0, 1);
        packedEdges_deg = zeros(0, 4);
    else
        packedAzimuth_deg = vertcat(sliceAzimuthBuffer_deg{:});
        packedElevation_deg = vertcat(sliceElevationBuffer_deg{:});
        packedEdges_deg = vertcat(sliceEdgeBuffer_deg{:});
    end

    % Uniform-grid metadata: when the time step is constant, a query time
    % maps to a slice index arithmetically instead of via search. The
    % relative tolerance absorbs floating-point noise across long missions
    % and the 1e-9 s floor covers steps near zero. NaN is the signal to fall
    % back to search-based lookup.
    timeSamplesAreUniform = true;
    timeStep_s = NaN;
    if sliceCount >= 2
        adjacentTimeSteps_s = diff(sliceTime_s);
        timeStep_s = median(adjacentTimeSteps_s);
        timeStepTolerance_s = max(1e-9, abs(timeStep_s) * 1e-9);
        timeSamplesAreUniform = all( ...
            abs(adjacentTimeSteps_s - timeStep_s) <= ...
            timeStepTolerance_s);
        if ~timeSamplesAreUniform
            timeStep_s = NaN;
        end
    end

    packedObstacle = emptyPackedObstacle();
    packedObstacle.Name = string(obstacleData.targetName);
    packedObstacle.SafetyMarginDeg = ...
        double(obstacleData.safetyMargin_deg);
    packedObstacle.TimeSeconds = sliceTime_s;
    packedObstacle.IsUniformTime = timeSamplesAreUniform;
    packedObstacle.TimeStepSeconds = timeStep_s;
    packedObstacle.SliceOffsets = sliceVertexOffsets;
    packedObstacle.AzimuthDeg = packedAzimuth_deg;
    packedObstacle.ElevationDeg = packedElevation_deg;
    packedObstacle.EdgeOffsets = sliceEdgeOffsets;
    packedObstacle.EdgeStartAzimuthDeg = packedEdges_deg(:, 1);
    packedObstacle.EdgeStartElevationDeg = packedEdges_deg(:, 2);
    packedObstacle.EdgeEndAzimuthDeg = packedEdges_deg(:, 3);
    packedObstacle.EdgeEndElevationDeg = packedEdges_deg(:, 4);
    packedObstacle.BoundsDeg = sliceBounds_deg;
    packedObstacle.SampleCount = sliceCount;
    packedObstacle.PackedVertexCount = numel(packedAzimuth_deg);
    packedObstacle.PackedEdgeCount = size(packedEdges_deg, 1);
    % Reported footprint: geometry, time, and uint64 offsets all use eight
    % bytes per value so collision boundaries retain source precision.
    doubleStorageBytes = 8 * (numel(packedObstacle.TimeSeconds) + ...
        numel(packedObstacle.SliceOffsets) + ...
        numel(packedObstacle.EdgeOffsets) + ...
        numel(packedObstacle.AzimuthDeg) + ...
        numel(packedObstacle.ElevationDeg) + ...
        numel(packedObstacle.EdgeStartAzimuthDeg) + ...
        numel(packedObstacle.EdgeStartElevationDeg) + ...
        numel(packedObstacle.EdgeEndAzimuthDeg) + ...
        numel(packedObstacle.EdgeEndElevationDeg) + ...
        numel(packedObstacle.BoundsDeg));
    packedObstacle.EstimatedStorageBytes = doubleStorageBytes;
    packedObstacles(obstacleIndex) = packedObstacle;
    if reducedRegionCount > 0
        warning("buildAzElTimeObstacleField:RegionsReduced", ...
            "Obstacle %d (%s) uniformly reduced %d polygon regions; " + ...
            "the packed collision boundary is an approximation.", ...
            obstacleIndex, packedObstacle.Name, reducedRegionCount);
    end
    if droppedRegionCount > 0
        warning("buildAzElTimeObstacleField:RegionsDropped", ...
            "Obstacle %d (%s) dropped %d regions with fewer than three " + ...
            "usable vertices; those regions cannot block queries.", ...
            obstacleIndex, packedObstacle.Name, droppedRegionCount);
    end
end

%% Section 3: Assemble The Output
% Format and Version allow planners and queries to reuse packed input.
obstacleField = struct();
obstacleField.Format = "AzElTimeObstacleField";
obstacleField.Version = 4;
obstacleField.ReferenceTime = referenceTime;
obstacleField.Obstacles = packedObstacles;
obstacleField.ObstacleCount = numel(packedObstacles);
obstacleField.Options = resolvedOptions;
% Retain canonical source geometry so plotting can recover the original
% uninflated boundary without asking callers for a second obstacle field.
obstacleField.SourceAzElData = canonicalObstacles;
obstacleField.SafetyMarginsDeg = reshape( ...
    [canonicalObstacles.safetyMargin_deg], [], 1);
packedStorageBytes = [packedObstacles.EstimatedStorageBytes];
obstacleField.EstimatedStorageBytes = sum(packedStorageBytes);
end

%% Section 4: Local Functions
function packedObstacle = emptyPackedObstacle()
%% Section 0: Header & Readme
% SYNTAX
%   packedObstacle = emptyPackedObstacle()
%**************************************************************************
% PURPOSE
%   - Define the single source of truth for one packed obstacle record.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - packedObstacle (scalar struct)
%       Empty packed obstacle record used for preallocation.
%**************************************************************************
% UNITS
%   - Unit-bearing fields identify seconds or degrees in their names.
%
% Serves both as the preallocation template (fixing field order for
% struct-array assignment) and as documentation of every field. Offset
% tables begin as the single sentinel value 1: an empty CSR table.
packedObstacle = struct( ...
    "Name", "", ...                   % obstacle display name
    "SafetyMarginDeg", 0, ...          % construction-time margin (deg)
    "TimeSeconds", zeros(0, 1), ...   % slice sample times (s)
    "IsUniformTime", false, ...       % constant time step?
    "TimeStepSeconds", NaN, ...       % step if uniform, else NaN
    "SliceOffsets", ones(1, 1, "uint64"), ...  % vertex CSR offsets (n+1)
    "AzimuthDeg", zeros(0, 1), ...             % packed vertex azimuths
    "ElevationDeg", zeros(0, 1), ...           % packed vertex elevations
    "EdgeOffsets", ones(1, 1, "uint64"), ...   % edge CSR offsets (n+1)
    "EdgeStartAzimuthDeg", zeros(0, 1), ...    % edge start vertices
    "EdgeStartElevationDeg", zeros(0, 1), ...
    "EdgeEndAzimuthDeg", zeros(0, 1), ...      % edge end vertices
    "EdgeEndElevationDeg", zeros(0, 1), ...
    "BoundsDeg", zeros(0, 4), ...       % [azMin azMax elMin elMax] rows
    "SampleCount", 0, ...             % number of time slices
    "PackedVertexCount", 0, ...       % rows in AzimuthDeg/ElevationDeg
    "PackedEdgeCount", 0, ...         % rows in the edge arrays
    "EstimatedStorageBytes", 0);      % reported packed footprint
end

function options = defaultAzElTimeObstacleFieldOptions()
%% Section 0: Header & Readme
% SYNTAX
%   options = defaultAzElTimeObstacleFieldOptions()
%**************************************************************************
% PURPOSE
%   - Keep obstacle-field defaults in one source of truth.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - options (scalar struct)
%       Fully populated public options structure.
%**************************************************************************
% UNITS
%   - MaximumVerticesPerRegion is dimensionless.
%   - ReferenceTime is a UTC datetime.
options = struct( ...
    "MaximumVerticesPerRegion", Inf, ...
    "ReferenceTime", [], ...
    "Verbose", false);
end
