function tests = testStaticPlanningProjection
%% Section 0: Header & Readme
% SYNTAX
%   tests = testStaticPlanningProjection
%**************************************************************************
% PURPOSE
%   - Verify conservative moving-history projection and one globally smooth
%     BMTP detour against authoritative moving geometry.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (MATLAB function-test array)
%       Focused projection and end-to-end validation cases.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(~)
    % Add the repository and independent trajectory packages for direct tests.
    repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
    addpath(repositoryRoot);
    addpath(fullfile(repositoryRoot, "trajectory"));
end

function testMovingHistoryProjectionIsStaticAndContainsSamples(testCase)
    % Enclose every protected history vertex in one documented convex surrogate.
    obstacle = createMovingRectangle();
    [projectionObstacles, projection] = obstacleAvoidance.obstacles.createStationaryObstacleEnclosures(obstacleAvoidance.obstacles.prepareObstacles(obstacle), 0, 20);

    verifyTrue(testCase, obstacleAvoidance.obstacles.queryStaticHorizon(obstacleAvoidance.obstacles.prepareObstacles(projectionObstacles), 0, 20));
    verifyEqual(testCase, projection.Records.Method, "conservativeProtectedHistoryConvexHull");
    boundary_units = projection.Records.Boundary_units;
    % Exercise each sample covered by this regression.
    for sampleIndex = 1:numel(obstacle.time_s)
        vertices_units = [obstacle.x_units{sampleIndex}, ...
            obstacle.y_units{sampleIndex}];
        [inside, onBoundary] = inpolygon(vertices_units(:, 1), vertices_units(:, 2), boundary_units(:, 1), boundary_units(:, 2));
        verifyTrue(testCase, all(inside | onBoundary));
    end
end

function testProjectedBmtpDetourPassesMovingValidation(testCase)
    % Solve one continuous multi-waypoint motion around a moving rectangle.
    obstacle = createMovingRectangle();
    [planningObstacles, ~] = obstacleAvoidance.obstacles.createStationaryObstacleEnclosures(obstacleAvoidance.obstacles.prepareObstacles(obstacle), 0, 20);
    initialState = struct();
    initialState.time_s       = 0;
    initialState.position_units = [-5 0];
    goalState = struct();
    goalState.time_s       = 20;
    goalState.position_units = [5 0];
    limits = struct();
    limits.maxVelocity_units_s      = [2 2];
    limits.maxAcceleration_units_s2 = [1 1];
    limits.maxJerk_units_s3         = [2 2];
    limits.xInterval_units    = [-6 6];
    limits.yInterval_units  = [-4 4];
    options = obstacleAvoidance.input.resolvePlannerOptions(struct("GoalTimeMode", "fixedArrival", "MaximumSeedCount", 3, "SampleTime_s", 0.05));
    [obstacle, initialState, goalState, limits] = obstacleAvoidance.input.normalizePlannerRequest(obstacle, initialState, goalState, limits, options);
    seed = obstacleAvoidance.search.createEmptyPathGuess();
    seed.Index        = 1;
    seed.Source       = "projectionTest";
    seed.position_units = [-5 0; -2 -2; 2 -2; 5 0];
    edgeLength_units = vecnorm(diff(seed.position_units), 2, 2);
    seed.tau                  = [0; cumsum(edgeLength_units)] / sum(edgeLength_units);
    seed.EstimatedDuration_s  = 20;
    seed.Length_units           = sum(edgeLength_units);
    seed.ObstacleEnvelope_units = projectionBoundary(planningObstacles);

    [candidate, ~] = obstacleAvoidance.planner.solveStaticBmtpTrajectory(seed, obstacleAvoidance.planner.prepareStaticSolverGeometry(obstacleAvoidance.obstacles.prepareObstacles(planningObstacles), initialState.time_s, goalState.time_s), initialState, goalState, limits, options);
    validation = obstacleAvoidance.validateTrajectory(candidate, obstacle, initialState, goalState, limits, options);

    verifyTrue(testCase, candidate.Success, candidate.Message);
    verifyTrue(testCase, validation.Passed, validation.Message);
    verifyTrue(testCase, validation.CollisionFree);
    verifyLessThanOrEqual(testCase, candidate.MotionLength_units, 1.1 * seed.Length_units, "Travel refinement made a broad excursion outside its route topology.");
    verifyLessThanOrEqual(testCase, max(candidate.position_units(:, 2)), 0.1, "The lower detour seed was unnecessarily smoothed over the obstacle.");
    verifyGreaterThan(testCase, max(vecnorm(candidate.velocity_units_s, 2, 2)), 0);
    interiorTime_s = initialState.time_s + seed.tau(2:end - 1) * candidate.TrajectoryDuration_s;
    [~, ~, interiorVelocity_units_s] = bmtpEngine.evaluatePolynomial(candidate.Polynomial, interiorTime_s);
    verifyGreaterThan(testCase, min(vecnorm(interiorVelocity_units_s, 2, 2)), 1e-3, "A multi-segment BMTP route was incorrectly forced to rest internally.");
end

function obstacle = createMovingRectangle()
    % Create a translating protected rectangle that blocks the direct route.
    time_s    = [0; 20];
    lower_units = [-1 -1; 1 -1; 1 1; -1 1];
    upper_units = lower_units + [0 1];
    obstacle  = obstacleAvoidance.obstacles.createObstacle("moving rectangle", time_s, {lower_units(:, 1); upper_units(:, 1)}, {lower_units(:, 2); upper_units(:, 2)}, 0.1);
end

function boundary_units = projectionBoundary(obstacles)
    % Return the complete static projection union boundary for seed provenance.
    obstacles = obstacleAvoidance.obstacles.prepareObstacles(obstacles);
    [isStatic, shape] = obstacleAvoidance.obstacles.queryStaticHorizon(obstacles, 0, 20);
    assert(isStatic, "Projection must be static over the test horizon.");
    boundary_units = shape.Vertices;
end
