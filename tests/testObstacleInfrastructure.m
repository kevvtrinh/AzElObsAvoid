function tests = testObstacleInfrastructure
%% Section 0: Header & Readme
% SYNTAX
%   tests = testObstacleInfrastructure
%**************************************************************************
% PURPOSE
%   - Freeze canonical obstacle combination, normalization, and query
%     behavior while duplicated method implementations are consolidated.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest function test array)
%**************************************************************************
% UNITS
%   - Boundary coordinates are coordinate units and time is seconds.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    % Prepare canonical obstacle fixtures. These tests check normalization, history
    % interpolation, boundary order, and occupancy queries. Investigate obstacle
    % preparation before planner search when one of these checks fails.
    % Add the repository root for path-based test execution.
    repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
    addpath(repositoryRoot);
    addpath(fullfile(repositoryRoot, "trajectory"));
    testCase.TestData.RepositoryRoot = repositoryRoot;
end

function testCombinePreservesCanonicalOrderAndFormat(testCase)
    % Verify every supported container form has one canonical interpretation.
    firstObstacle  = rectangleObstacle("first", [0; 4], [-2 0 -1 1]);
    secondObstacle = rectangleObstacle("second", [0; 4], [1 3 -2 2]);
    nestedInputs   = {[], {firstObstacle, {[], secondObstacle}}};

    rootResult = obstacleAvoidance.obstacles.combineObstacles(nestedInputs);
    verifyEqual(testCase, size(rootResult), [2 1]);
    verifyEqual(testCase, [rootResult.targetName].', ["first"; "second"]);

    rootEmpty = obstacleAvoidance.obstacles.combineObstacles();
    verifyEqual(testCase, obstacleAvoidance.obstacles.combineObstacles([]), rootEmpty);
    verifyEqual(testCase, fieldnames(rootEmpty), fieldnames(rootResult));
end

function testUnifiedOwnerNormalizesCanonicalValues(testCase)
    % Verify the unified owner establishes stable values, shapes, and fields.
    inputData  = normalizationFixture();
    rootResult = obstacleAvoidance.obstacles.createObstacle(inputData);
    verifyEqual(testCase, rootResult.time_s, [0; 2]);
    verifyEqual(testCase, rootResult.status, ["visible"; "visible"]);
    verifyEqual(testCase, size(rootResult.x_units), [2 1]);
    verifyEqual(testCase, rootResult.safetyMargin_units, 0.25);
end

function testNormalizeDiagnosticsAreEquivalent(testCase)
    % Verify malformed inputs preserve the established public error identifiers.
    base  = normalizationFixture();
    cases = cell(0, 2);
    cases(end + 1, :) = {@() struct(), ...
        "createObstacle:InvalidInput"};
    cases(end + 1, :) = {@() setField(base, "targetName", ""), ...
        "createObstacle:InvalidTargetName"};
    cases(end + 1, :) = {@() setField(base, "time_s", [0 0]), ...
        "createObstacle:InvalidTime"};
    cases(end + 1, :) = {@() setField(base, "x_units", {[0 1 1 0]}), ...
        "createObstacle:InvalidBoundary"};
    cases(end + 1, :) = {@() rmfield(base, "originalY_units"), ...
        "createObstacle:IncompleteOriginalBoundary"};
    cases(end + 1, :) = {@() setField(base, "status", ["a" "b" "c"]), ...
        "createObstacle:StatusSizeMismatch"};
    cases(end + 1, :) = {@() setBoundaryPair(base, [0 1 NaN 2], ...
        [0 1 0 2]), ...
        "createObstacle:UnpairedNonfiniteBoundary"};
    cases(end + 1, :) = {@() setBoundaryPair(base, 0, 0), ...
        "createObstacle:BoundaryRingTooShort"};

    % Exercise each case covered by this regression.
    for caseIndex = 1:size(cases, 1)
        inputData          = cases{caseIndex, 1}();
        expectedIdentifier = cases{caseIndex, 2};
        actualIdentifier   = captureErrorIdentifier(@obstacleAvoidance.obstacles.createObstacle, inputData);
        verifyEqual(testCase, actualIdentifier, expectedIdentifier);
    end
end

function testTwoVertexWarningIsEquivalent(testCase)
    % Verify degenerate-region removal emits one stable warning.
    inputData = normalizationFixture();
    inputData.x_units{1} = [0; 1; NaN; -2; 2; 2; -2];
    inputData.y_units{1} = [0; 1; NaN; -1; -1; 1; 1];
    inputData.originalX_units = inputData.x_units;
    inputData.originalY_units = inputData.y_units;
    lastwarn("");
    output = obstacleAvoidance.obstacles.createObstacle(inputData);
    [~, warningIdentifier] = lastwarn();
    verifyEqual(testCase, string(warningIdentifier), "createObstacle:RemovedTwoVertexRegions");
    verifyEqual(testCase, numel(output.x_units{1}), 4);
end

function testSharedQueryPreservesGeometryAndCompatibility(testCase)
    % Verify shared moving, multi-ring, broadcast, and diagnostic query behavior.
    movingObstacle = movingMultiRingObstacle();
    staticObstacle = rectangleObstacle("static", [0; 4], [-0.5 0.5 -0.5 0.5]);
    obstacles      = obstacleAvoidance.obstacles.combineObstacles(movingObstacle, staticObstacle);
    x_units    = [-3 0 3; -2 NaN 2];
    y_units  = [0 0 0; 1 0 -1];
    time_s         = [0 0 0; 2 2 4];
    options        = struct();
    options.BoundaryIsOccupied     = false;
    options.ClearanceTolerance_units = 1e-10;

    [sharedOccupied, sharedBlocker, sharedDetails] = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(obstacles, x_units, y_units, time_s, options);
    verifyClass(testCase, sharedBlocker, "uint32");
    verifyEqual(testCase, size(sharedOccupied), size(x_units));
    verifyEqual(testCase, size(sharedDetails.MinimumClearance_units), size(x_units));

    fastOccupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(obstacles, x_units, y_units, time_s, options);
    verifyEqual(testCase, fastOccupied, sharedOccupied);

    hs3Options = options;
    [hs3Occupied, hs3Blocker, hs3Details] = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(obstacles, x_units, y_units, time_s, hs3Options);
    verifyEqual(testCase, hs3Occupied, sharedOccupied);
    verifyEqual(testCase, hs3Blocker, sharedBlocker);
    verifyEqual(testCase, hs3Details.MinimumClearance_units, sharedDetails.MinimumClearance_units, "AbsTol", 1e-12);
    verifyFalse(testCase, isfield(hs3Details.Options, "PlannerMethod"));

    referenceTime   = datetime(2026, 1, 1, "TimeZone", "UTC");
    datetimeOptions = options;
    datetimeOptions.ReferenceTime = referenceTime;
    datetimeOccupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(obstacles, x_units, y_units, referenceTime + seconds(time_s), datetimeOptions);
    verifyEqual(testCase, datetimeOccupied, sharedOccupied);
end

function testBatchedMultiRingOccupancyMatchesPointwiseQueries(testCase)
    % Verify batched multi-ring clearance preserves pointwise boundary decisions.
    obstacle      = movingMultiRingObstacle();
    x_units   = [-3.5 -2.5 -1.5 -0.5 0.5 1.5 2.5 3.5 4.5];
    y_units = zeros(size(x_units));
    queryTime_s   = 1;

    % Exercise each boundary is occupied covered by this regression.
    for boundaryIsOccupied = [false true]
        options = struct("BoundaryIsOccupied", boundaryIsOccupied, ...
            "ClearanceTolerance_units", 1e-10);
        batchedOccupied   = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(obstacle, x_units, y_units, queryTime_s, options);
        pointwiseOccupied = false(size(x_units));
        % Exercise each point covered by this regression.
        for pointIndex = 1:numel(x_units)
            pointwiseOccupied(pointIndex) = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(obstacle, x_units(pointIndex), y_units(pointIndex), queryTime_s, options);
        end
        verifyEqual(testCase, batchedOccupied, pointwiseOccupied);

        [detailedOccupied, ~, details] = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(obstacle, x_units, y_units, queryTime_s, options);
        verifyEqual(testCase, batchedOccupied, detailedOccupied);
        verifyTrue(testCase, all(isfinite(details.MinimumClearance_units)));
    end
end

function testShapeQueryReportsOrderedBoundaryProperties(testCase)
    % Compare the lightweight ordered-boundary record with polyshape evidence.
    convexObstacle         = rectangleObstacle("convex", [0; 4], [-2 2 -1 1]);
    preparedConvexObstacle = obstacleAvoidance.obstacles.prepareObstacles(convexObstacle);
    verifyTrue(testCase, preparedConvexObstacle.InternalPreparation.IsTimeInvariant);
    [convexShape, convexGeometry] = obstacleAvoidance.obstacles.shapeAtTime(preparedConvexObstacle, 2);
    verifyTrue(testCase, convexGeometry.HasOrderedSingleRegion);
    verifyTrue(testCase, convexGeometry.IsConvex);
    vertices_units         = [convexGeometry.x_units, convexGeometry.y_units];
    edgeDelta_units        = vertices_units(2, :) - vertices_units(1, :);
    leftNormal           = [-edgeDelta_units(2), edgeDelta_units(1)] / norm(edgeDelta_units);
    probe_units            = 0.5 * sum(vertices_units(1:2, :), 1) + 1e-6 * leftNormal;
    referenceOutwardSign = 1 - 2 * isinterior(convexShape, probe_units(1), probe_units(2));
    verifyEqual(testCase, convexGeometry.OutwardSign, referenceOutwardSign);
    [~, repeatedGeometry] = obstacleAvoidance.obstacles.shapeAtTime(preparedConvexObstacle, 3, true);
    verifyEqual(testCase, repeatedGeometry, convexGeometry);

    concaveX_units   = [0; 2; 2; 1; 1; 0];
    concaveY_units = [0; 0; 1; 1; 2; 2];
    concaveObstacle      = obstacleAvoidance.obstacles.createObstacle("concave", [0; 4], concaveX_units, concaveY_units, 0);
    [~, concaveGeometry] = obstacleAvoidance.obstacles.shapeAtTime(concaveObstacle, 2, true);
    verifyTrue(testCase, concaveGeometry.HasOrderedSingleRegion);
    verifyFalse(testCase, concaveGeometry.IsConvex);
    multiRegionObstacle         = movingMultiRingObstacle();
    preparedMultiRegionObstacle = obstacleAvoidance.obstacles.prepareObstacles(multiRegionObstacle);
    verifyFalse(testCase, preparedMultiRegionObstacle.InternalPreparation.IsTimeInvariant);
    [~, multiRegionGeometry] = obstacleAvoidance.obstacles.shapeAtTime(preparedMultiRegionObstacle, 1, true);
    verifyTrue(testCase, multiRegionGeometry.HasOrderedSingleRegion);
    verifyTrue(testCase, multiRegionGeometry.IsConvex);
    verifyFalse(testCase, isnan(multiRegionGeometry.OutwardSign));
    verifyFalse(testCase, multiRegionGeometry.TopologyIsInterpolated);
    verifyEqual(testCase, multiRegionGeometry.GeometryModel, "conservativeEndpointConvexHull");
end

function testShapeQueryPreservesOutputTypesAcrossControlPaths(testCase)
    % Verify deferred allocation retains inactive, interpolated, and union outputs.
    movingObstacle = movingMultiRingObstacle();
    [inactiveShape, inactiveGeometry] = obstacleAvoidance.obstacles.shapeAtTime(movingObstacle, -1, false);
    verifyClass(testCase, inactiveShape, "polyshape");
    verifyEmpty(testCase, inactiveShape.Vertices);
    verifyFalse(testCase, inactiveGeometry.Active);
    verifyEqual(testCase, inactiveGeometry.LowerSampleIndex, 0);
    verifyEqual(testCase, inactiveGeometry.UpperSampleIndex, 0);

    [geometryOnlyShape, geometryOnlyRecord] = obstacleAvoidance.obstacles.shapeAtTime(movingObstacle, 5, true);
    verifyEqual(testCase, geometryOnlyShape, []);
    verifyEqual(testCase, geometryOnlyRecord, inactiveGeometry);

    [interpolatedShape, interpolatedGeometry] = obstacleAvoidance.obstacles.shapeAtTime(movingObstacle, 1, false);
    verifyClass(testCase, interpolatedShape, "polyshape");
    verifyNotEmpty(testCase, interpolatedShape.Vertices);
    verifyTrue(testCase, interpolatedGeometry.Active);
    verifyFalse(testCase, interpolatedGeometry.TopologyIsInterpolated);
    verifyEqual(testCase, interpolatedGeometry.GeometryModel, "conservativeEndpointConvexHull");

    singleSliceObstacle = obstacleAvoidance.obstacles.createObstacle("single slice", 0, [-1; 1; 1; -1], [-1; -1; 1; 1], 0);
    [singleSliceShape, singleSliceGeometry] = obstacleAvoidance.obstacles.shapeAtTime(singleSliceObstacle, 100, false);
    verifyClass(testCase, singleSliceShape, "polyshape");
    verifyTrue(testCase, singleSliceGeometry.Active);
    verifyEqual(testCase, singleSliceGeometry.LowerSampleIndex, 1);
    verifyEqual(testCase, singleSliceGeometry.UpperSampleIndex, 1);

    topologyObstacle = topologyChangingObstacle();
    [unionShape, unionGeometry]                 = obstacleAvoidance.obstacles.shapeAtTime(topologyObstacle, 1, false);
    [geometryOnlyUnionShape, geometryOnlyUnion] = obstacleAvoidance.obstacles.shapeAtTime(topologyObstacle, 1, true);
    verifyClass(testCase, geometryOnlyUnionShape, "polyshape");
    verifyEqual(testCase, geometryOnlyUnionShape.Vertices, unionShape.Vertices, "AbsTol", 0);
    verifyEqual(testCase, geometryOnlyUnion, unionGeometry);
    verifyFalse(testCase, unionGeometry.TopologyIsInterpolated);
end

function testPreparationCachesGeometryAndRejectsStaleSource(testCase)
    % Rebuild cached shapes after any canonical public source field changes.
    obstacle    = rectangleObstacle("cache source", [0; 4], [-2 2 -1 1]);
    prepared    = obstacleAvoidance.obstacles.prepareObstacles(obstacle);
    preparation = prepared.InternalPreparation;
    verifyEqual(testCase, preparation.PreparationVersion, 1);
    verifySize(testCase, preparation.SampleBounds_units, [2 4]);
    verifySize(testCase, preparation.IntervalBounds_units, [1 4]);
    verifyEqual(testCase, size(preparation.SampleEdgeStart_units{1}, 1), 4);
    verifyEqual(testCase, preparation.SampleEdgeStart_units{1}, preparation.SampleEdgeEnd_units{1}([4 1 2 3], :), "AbsTol", 0);

    mutated = prepared;
    % Exercise each sample covered by this regression.
    for sampleIndex = 1:numel(mutated.x_units)
        mutated.x_units{sampleIndex} = mutated.x_units{sampleIndex} + 5;
    end
    [shape, geometry] = obstacleAvoidance.obstacles.shapeAtTime(mutated, 2);
    verifyTrue(testCase, geometry.Active);
    verifyEqual(testCase, min(shape.Vertices(:, 1)), 3, "AbsTol", 1e-12);
    verifyFalse(testCase, obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(mutated, 0, 0, 2));
    [~, projection] = obstacleAvoidance.obstacles.createStationaryObstacleEnclosures(obstacleAvoidance.obstacles.prepareObstacles(mutated), 0, 4);
    verifyEqual(testCase, min(projection.Records.Boundary_units(:, 1)), 3, "AbsTol", 1e-12);
    reprepared = obstacleAvoidance.obstacles.prepareObstacles(mutated);
    verifyNotEqual(testCase, reprepared.InternalPreparation.SourceSnapshot, preparation.SourceSnapshot);
    verifyEqual(testCase, reprepared.InternalPreparation.SampleBounds_units(:, 1), [3; 3], "AbsTol", 1e-12);
end

function testPreparedConvexClearanceMatchesPolyshapePath(testCase)
    % Verify cached clockwise and counterclockwise edges preserve signed clearance.
    vertices_units = [-2 -1; 2 -1; 2 1; -2 1];
    shape        = polyshape(vertices_units, "Simplify", false);
    points_units   = [-3 0; 0 0; 2 0; 3 0; 0 1 + 1e-13];
    referenceClearance_units = obstacleAvoidance.geometry.pointPolygonClearance(shape, points_units);
    vertexOrientations_units = {vertices_units, flipud(vertices_units)};
    % Exercise each orientation covered by this regression.
    for orientationIndex = 1:numel(vertexOrientations_units)
        orderedVertices_units = vertexOrientations_units{orientationIndex};
        nextVertices_units    = circshift(orderedVertices_units, -1, 1);
        signedDoubleArea    = sum(orderedVertices_units(:, 1) .* nextVertices_units(:, 2) - orderedVertices_units(:, 2) .* nextVertices_units(:, 1));
        geometry = struct("EdgeStart_units", orderedVertices_units, ...
            "EdgeEnd_units", nextVertices_units, ...
            "HasOrderedSingleRegion", true, "IsConvex", true, ...
            "OutwardSign", -sign(signedDoubleArea));
        cachedClearance_units = obstacleAvoidance.geometry.pointPolygonClearance(shape, points_units, geometry);
        verifyEqual(testCase, cachedClearance_units, referenceClearance_units, "AbsTol", 1e-12);
    end
end

function testStaticHorizonClassifiesSpanAndGeometry(testCase)
    % Verify static, partial-span, moving, and empty-history classifications.
    staticObstacle = rectangleObstacle("static horizon", [0; 4], [-2 2 -1 1]);
    preparedStatic = obstacleAvoidance.obstacles.prepareObstacles(staticObstacle);
    [obstaclesRemainStatic, occupiedShape] = obstacleAvoidance.obstacles.queryStaticHorizon(preparedStatic, 0, 4);
    verifyTrue(testCase, obstaclesRemainStatic);
    verifyEqual(testCase, area(occupiedShape), area(preparedStatic.InternalPreparation.StaticShape), "AbsTol", 1e-12);

    [coversLongHorizon, unsupportedShape] = obstacleAvoidance.obstacles.queryStaticHorizon(preparedStatic, -1, 4);
    verifyFalse(testCase, coversLongHorizon);
    verifyEmpty(testCase, unsupportedShape.Vertices);

    movingObstacle = staticObstacle;
    movingObstacle.x_units{2} = movingObstacle.x_units{2} + 1;
    preparedMoving = obstacleAvoidance.obstacles.prepareObstacles(movingObstacle);
    verifyFalse(testCase, obstacleAvoidance.obstacles.queryStaticHorizon(preparedMoving, 0, 4));

    emptyTimeObstacle = preparedStatic;
    emptyTimeObstacle.time_s = zeros(0, 1);
    [supportsEmptyTime, emptyTimeShape] = obstacleAvoidance.obstacles.queryStaticHorizon(emptyTimeObstacle, 0, 4);
    verifyFalse(testCase, supportsEmptyTime);
    verifyEmpty(testCase, emptyTimeShape.Vertices);
end

function obstacle = normalizationFixture()
    % Construct one raw moving, multi-ring record requiring normalization.
    xSlices_units = {[-3 -1 -1 -3 NaN 1 3 3 1], ...
        [-2 0 0 -2 NaN 2 4 4 2]};
    ySlices_units = {[-1 -1 1 1 NaN -1 -1 1 1], ...
        [-1 -1 1 1 NaN -1 -1 1 1]};
    obstacle = struct("targetName", "normalization fixture", ...
        "time_s", [0 2], ...
        "x_units", {xSlices_units}, ...
        "y_units", {ySlices_units}, ...
        "originalX_units", {xSlices_units}, ...
        "originalY_units", {ySlices_units}, ...
        "safetyMargin_units", 0.25, ...
        "status", "visible");
end

function obstacle = movingMultiRingObstacle()
    % Construct two translating protected regions in one canonical obstacle.
    inputData = normalizationFixture();
    inputData.targetName       = "moving multi-ring";
    inputData.safetyMargin_units = 0;
    obstacle = obstacleAvoidance.obstacles.createObstacle(inputData);
end

function obstacle = topologyChangingObstacle()
    % Construct one protected region that splits into two rings between samples.
    closed_units = [-2 -1; 2 -1; 2 1; -2 1];
    left_units   = [-2 -1; -0.5 -1; -0.5 1; -2 1];
    right_units  = [0.5 -1; 2 -1; 2 1; 0.5 1];
    open_units   = [left_units; NaN NaN; right_units];
    obstacle   = obstacleAvoidance.obstacles.createObstacle("opening", [0; 2], {closed_units(:, 1); open_units(:, 1)}, {closed_units(:, 2); open_units(:, 2)}, 0);
end

function obstacle = rawObstacle(time_s, positionBySlice_units)
    % Construct the minimum protected history consumed by preparation queries.
    xBySlice_units   = cell(numel(positionBySlice_units), 1);
    yBySlice_units = cell(numel(positionBySlice_units), 1);
    % Exercise each sample covered by this regression.
    for sampleIndex = 1:numel(positionBySlice_units)
        position_units = positionBySlice_units{sampleIndex};
        xBySlice_units{sampleIndex} = position_units(:, 1);
        yBySlice_units{sampleIndex} = position_units(:, 2);
    end
    obstacle = struct("time_s", time_s(:), ...
        "x_units", {xBySlice_units}, ...
        "y_units", {yBySlice_units});
end

function obstacle = rectangleObstacle(name, time_s, bounds_units)
    % Construct one canonical static rectangle.
    x_units   = bounds_units([1 2 2 1]).';
    y_units = bounds_units([3 3 4 4]).';
    obstacle      = obstacleAvoidance.obstacles.createObstacle(name, time_s, x_units, y_units, 0);
end

function value = setField(value, fieldName, fieldValue)
    % Return a fixture with one field replaced.
    value.(fieldName) = fieldValue;
end

function value = setBoundaryPair(value, x_units, y_units)
    % Replace the first protected slice with a requested malformed boundary.
    value.x_units{1} = x_units;
    value.y_units{1} = y_units;
end

function identifier = captureErrorIdentifier(normalizer, inputData)
    % Return the identifier produced for one invalid normalization request.
    identifier = "";
    try
        normalizer(inputData);
    catch exception
        identifier = string(exception.identifier);
    end
end
