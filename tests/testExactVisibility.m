function tests = testExactVisibility
% Compare exact reduced visibility with exhaustive graphs and boundary contacts.
% Run with runtests('tests/testExactVisibility.m').
tests = functiontests(localfunctions);
end

function setupOnce(~)
    rootFolder = fileparts(fileparts(mfilename('fullpath')));
    addpath(rootFolder, fullfile(rootFolder, 'trajectory'), fullfile(rootFolder, 'examples'));
end

function testProposalVisibilityBoundaryAndInteriorRejection(testCase)
    runCases(testCase, { ...
        @proposalVisibilityBoundaryAndInteriorRejection});
end

function testExactReducedGraphContracts(testCase)
    previousRandomState = rng;
    restoreRandomState = onCleanup(@() rng(previousRandomState));
    coreLimits = struct('xInterval_units', [-6, 6], 'yInterval_units', [-4, 4], ...
        'maxVelocity_units_s', [2, 2], 'maxAcceleration_units_s2', [2, 2], ...
        'maxJerk_units_s3', [4, 4]);
    limits = struct('xInterval_units', [-12, 12], 'yInterval_units', [-10, 10]);
    options = struct('ConstraintTolerance', 1e-8);
    uShape = polyshape([-4, 5; -2, 5; -2, -2; 2, -2; 2, 5; 4, 5; 4, -4; -4, -4]);
    ring = subtract(polyshape([-4, -4; 4, -4; 4, 4; -4, 4]), ...
        polyshape([-2, -2; 2, -2; 2, 2; -2, 2]));
    runCases(testCase, { ...
        @(testCase) holeAndDisconnectedRegions(testCase, coreLimits, options); ...
        @(testCase) visibilityMatchesExhaustiveReference(testCase, limits, options); ...
        @(testCase) reducedGraphMatchesExhaustiveReferenceOnConcaveShapes(testCase, limits, options); ...
        @(testCase) batchedContactsHolesAndConcavities(testCase, limits, options, uShape, ring); ...
        @(testCase) reflectedAndTranslatedConcavities(testCase, uShape, ring)});
end

function proposalVisibilityBoundaryAndInteriorRejection(testCase)
    shape = polyshape([0, 1, 1, 0], [0, 0, 1, 1]);
    [edgeStart, edgeEnd] = obstacleAvoidance.geometry.boundaryToEdges(shape, 1e-12);
    first = [-2, -1; -1, 0.5; -3, 0.5; -1, 1; -1, 2; 0, 0.5; 0.25, 0.25];
    second = [-1, -1; 2, 0.5; 0.25, 0.5; 2, 1; 2, 2; -1, 0.5; 0.75, 0.75];
    expected = [true; false; false; false; true; false; false];
    verifyEqual(testCase, obstacleAvoidance.search.checkVisibilitySegments( ...
        first, second, shape, edgeStart, edgeEnd), expected);
    % No midpoint queries survive when every segment hits a boundary.
    blocked = [2, 3, 4, 6];
    verifyEqual(testCase, obstacleAvoidance.search.checkVisibilitySegments( ...
        first(blocked, :), second(blocked, :), shape, edgeStart, edgeEnd), false(4, 1));
    verifyEqual(testCase, obstacleAvoidance.search.checkVisibilitySegments( ...
        first, second, polyshape(), zeros(0, 2), zeros(0, 2)), true(size(expected)));
    shape = polyshape([0, 4, 4, 0, NaN, 1, 1, 3, 3], [0, 0, 4, 4, NaN, 1, 3, 3, 1]);
    [edgeStart, edgeEnd] = obstacleAvoidance.geometry.boundaryToEdges(shape, 1e-12);
    first = [1.25, 2; 1.25, 2; 0.25, 0.25; -2, 5; 1, 1];
    second = [2.75, 2; 3.5, 2; 0.75, 0.75; 6, 5; 3, 1];
    verifyEqual(testCase, obstacleAvoidance.search.checkVisibilitySegments( ...
        first, second, shape, edgeStart, edgeEnd), [true; false; false; true; false]);
end

function holeAndDisconnectedRegions(testCase, limits, options)
    outer = polyshape([-2 -2; 2 -2; 2 2; -2 2]);
    inner = polyshape([-1 -1; 1 -1; 1 1; -1 1]);
    shape = subtract(outer, inner);
    scene = struct('ProtectedShape', shape);
    vertexVisibility = obstacleAvoidance.search.createVertexVisibility(scene, limits, options);
    graph = obstacleAvoidance.search.createVisibilityGraph(vertexVisibility, [-0.5 0], [0.5 0]);
    verifyEqual(testCase, graph.RouteLength_units, 1, 'AbsTol', 1e-12);
    graph = obstacleAvoidance.search.createVisibilityGraph(vertexVisibility, [0 0], [4 0]);
    verifyFalse(testCase, graph.IsConnected);
end

function visibilityMatchesExhaustiveReference(testCase, limits, options)
    rng(73);
    for k = 1:30
        scene = struct('ProtectedShape', {}, 'ProtectedVertices_units', {});
        for j = 1:mod(k, 6) + 1
            points_units = rand(9, 2) * 1.4 + [-6 + 2 * j, -2 + rand * 4];
            hull = convhull(points_units(:, 1), points_units(:, 2));
            shape = polyshape(points_units(hull(1:end - 1), :));
            scene(j) = struct('ProtectedShape', shape, 'ProtectedVertices_units', shape.Vertices);
        end
        initial_units = [-9, rand * 2 - 1]; goal_units = [9, rand * 2 - 1];
        reference = createVisibilityGraphBaseline(scene, initial_units, goal_units, limits, options);
        vertexVisibility = obstacleAvoidance.search.createVertexVisibility(scene, limits, options);
        actual = obstacleAvoidance.search.createVisibilityGraph(vertexVisibility, initial_units, goal_units);
        verifyEqual(testCase, actual.IsConnected, reference.IsConnected);
        verifyEqual(testCase, actual.RouteLength_units, reference.RouteLength_units, 'AbsTol', 1e-8);
        verifyTrue(testCase, actual.GraphIsFullyEnumerated);
        % The reduced graph keeps a subset of the exhaustive connections.
        verifyLessThanOrEqual(testCase, size(actual.AcceptedNodeIndex, 1), size(reference.AcceptedNodeIndex, 1));
    end
end

function reducedGraphMatchesExhaustiveReferenceOnConcaveShapes(testCase, limits, options)
    % Random star-shaped (concave) obstacles, spaced so they never overlap,
    % against the exhaustive single-ring reference graph.
    rng(91);
    warningState = warning('off', 'MATLAB:polyshape:repairedBySimplify');
    restoreWarning = onCleanup(@() warning(warningState));
    for k = 1:40
        scene = struct('ProtectedShape', {}, 'ProtectedVertices_units', {});
        for j = 1:mod(k, 4) + 1
            % The reference reads one closed ring per obstacle, so redraw any
            % polygon that polyshape repaired into several regions.
            shape = polyshape();
            while shape.NumRegions ~= 1 || shape.NumHoles ~= 0
                angles = sort(rand(8, 1) * 2 * pi);
                radii = 0.5 + rand(8, 1) * 0.9;
                center = [-6 + 3 * j, rand * 6 - 3];
                shape = polyshape(center + [radii.*cos(angles), radii.*sin(angles)]);
            end
            scene(j) = struct('ProtectedShape', shape, 'ProtectedVertices_units', shape.Vertices);
        end
        initial_units = [-9, rand * 2 - 1]; goal_units = [9, rand * 2 - 1];
        reference = createVisibilityGraphBaseline(scene, initial_units, goal_units, limits, options);
        vertexVisibility = obstacleAvoidance.search.createVertexVisibility(scene, limits, options);
        actual = obstacleAvoidance.search.createVisibilityGraph(vertexVisibility, initial_units, goal_units);
        verifyEqual(testCase, actual.IsConnected, reference.IsConnected);
        verifyEqual(testCase, actual.RouteLength_units, reference.RouteLength_units, 'AbsTol', 1e-8);
        verifyLessThanOrEqual(testCase, size(actual.NodePosition_units, 1), size(reference.NodePosition_units, 1));
        verifyLessThanOrEqual(testCase, size(actual.AcceptedNodeIndex, 1), size(reference.AcceptedNodeIndex, 1));
    end
end

function batchedContactsHolesAndConcavities(testCase, limits, options, uShape, ring)
    islands = union(polyshape([-3, -1; -1, -1; -1, 1; -3, 1]), polyshape([1, -1; 3, -1; 3, 1; 1, 1]));
    touching = union(polyshape([-3, -3; 0, -3; 0, 0; -3, 0]), polyshape([0, 0; 3, 0; 3, 3; 0, 3]));
    shapes = {uShape, ring, ring, islands, touching};
    starts = [0, 0; 0, 0; -7, 0; -7, 1; -7, 0];
    goals = [0, -7; 1, 1; 7, 0; 7, 1; 7, 0];
    % Analytic boundary routes: U opening and outer corners; ring exterior;
    % and straight tangent routes along the remaining component boundaries.
    expectedLength_units = [16 + sqrt(29); sqrt(2); 18; 14; 14];
    for k = 1:numel(shapes)
        scene = struct('ProtectedShape', shapes{k}, 'ProtectedVertices_units', shapes{k}.Vertices);
        vertexVisibility = obstacleAvoidance.search.createVertexVisibility(scene, limits, options);
        actual = obstacleAvoidance.search.createVisibilityGraph(vertexVisibility, starts(k, :), goals(k, :));
        verifyTrue(testCase, actual.IsConnected);
        verifyEqual(testCase, actual.RouteLength_units, expectedLength_units(k), 'AbsTol', 1e-8);
    end
end

function reflectedAndTranslatedConcavities(testCase, uShape, ring)
    % The occupied side must come from filled geometry, including hole rings.
    shapes = {uShape, ring}; starts = [0, 0; -7, 0]; goals = [0, -7; 7, 0]; lengths = [16 + sqrt(29), 18];
    transforms = cat(3, eye(2), [-1, 0; 0, 1], [0, -1; 1, 0], [0, 1; 1, 0]);
    for translation = [0, 128]
        offset = [translation, -2 * translation];
        limits = struct('xInterval_units', [-12, 12] + offset(1), 'yInterval_units', [-12, 12] + offset(2));
        for transform = 1:size(transforms, 3)
            rotation = transforms(:, :, transform);
            for k = 1:numel(shapes)
                vertices = shapes{k}.Vertices * rotation + offset;
                shape = polyshape(vertices(:, 1), vertices(:, 2));
                scene = struct('ProtectedShape', shape);
                vertexVisibility = obstacleAvoidance.search.createVertexVisibility(scene, limits, struct('ConstraintTolerance', 1e-8));
                graph = obstacleAvoidance.search.createVisibilityGraph(vertexVisibility, starts(k, :) * rotation + offset, ...
                    goals(k, :) * rotation + offset);
                verifyTrue(testCase, graph.IsConnected);
                verifyEqual(testCase, graph.RouteLength_units, lengths(k), 'AbsTol', 1e-8);
            end
        end
    end
end
