function tests = testPlannerAuditFixes
%% Section 0: Header & Readme
% SYNTAX
%   tests = testPlannerAuditFixes
% PURPOSE
%   Check preserved diagnostics, certificate dimensions, and optional outputs.
% INPUTS
%   None.
% OUTPUTS
%   MATLAB function tests.
% UNITS
%   Position is degrees; time is seconds.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root, fullfile(root, 'trajectory'));
    initial = struct();
    initial.time_s       = 0;
    initial.position_deg = [-3 0];
    goal = struct();
    goal.time_s       = 10;
    goal.position_deg = [3 0];
    limits = struct();
    limits.maxVelocity_deg_s      = [2 2];
    limits.maxAcceleration_deg_s2 = [1 1];
    limits.maxJerk_deg_s3         = [2 2];
    limits.azimuthInterval_deg    = [-5 5];
    limits.elevationInterval_deg  = [-5 5];
    options = obstacleAvoidance.planTrajectory();
    [~, initial, goal, limits] = obstacleAvoidance.input.normalizePlannerRequest([], initial, goal, limits, options);
    testCase.TestData = struct('Initial', initial, 'Goal', goal, 'Limits', limits, 'Options', options);
end

function testCorridorClearanceIsScalarAcrossDegreesAndRegions(testCase)
    upper = [0 2;1 2;1 3;0 3];
    lower = [0 -3;1 -3;1 -2;0 -2];
    % Exercise each region covered by this regression.
    for regionCount = 1:2
        boundary = upper;
        if regionCount == 2
            boundary = [boundary; NaN NaN; lower];
        end
        obstacle = obstacleAvoidance.obstacles.createObstacle('upper', 0, upper(:, 1), upper(:, 2), 0);
        if regionCount == 2
            lowerObstacle = obstacleAvoidance.obstacles.createObstacle('lower', 0, lower(:, 1), lower(:, 2), 0);
            obstacle      = obstacleAvoidance.obstacles.combineObstacles(obstacle, lowerObstacle);
        end
        obstacle = obstacleAvoidance.obstacles.prepareObstacles(obstacle);
        regions  = obstacleAvoidance.geometry.convexPolygonRegions(polyshape(boundary(:, 1), boundary(:, 2)));
        corridor = repmat(struct('SegmentIndex', 1, 'RegionIndex', 0, 'Normal', [0 0], 'Clearance_deg', 0.5, 'BoundaryOffset_deg', -2), numel(regions), 1);
        % Exercise each k covered by this regression.
        for k = 1:numel(regions)
            corridor(k).RegionIndex = k;
            corridor(k).Normal = [0 -sign(mean(regions(k).Vertices(:, 2)))];
        end
        % Exercise each degree covered by this regression.
        for degree = [1 3]
            power = zeros(1, 2, degree+1);
            power(1, 1, 2) = 1;
            trajectory = struct('Polynomial', struct('SegmentCount', 1, 'positionPower_deg', power), ...
                'SeedCorridorBoundary_deg', boundary, 'SeedCorridor', corridor);
            [certified, clearance] = obstacleAvoidance.validation.certifySeedCorridor(trajectory, obstacle, 1e-7);
            verifyTrue(testCase, certified);
            verifySize(testCase, clearance, [1 1]);
            verifyEqual(testCase, clearance, 2, 'AbsTol', 1e-12);
        end
    end
end

function testOccupancyOutputsAgreeForStaticAndMovingHistories(testCase)
    % Exercise each moving covered by this regression.
    for moving = [false true]
        times = 0;
        az    = [-1;1;1;-1];
        el    = [-1;-1;1;1];
        if moving
            times = [0; 1];
            az    = {az, az + 0.5};
            el    = {el, el};
        end
        first     = obstacleAvoidance.obstacles.createObstacle('first', times, az, el, 0);
        second    = obstacleAvoidance.obstacles.createObstacle('second', 0, [-0.5;2;2;-0.5], [-1;-1;1;1], 0);
        obstacles = obstacleAvoidance.obstacles.combineObstacles(first, second);
        x         = [NaN -4 0 0.75 1.75 4];
        t         = 0.5;
        one       = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(obstacles, x, 0, t);
        [two, indexTwo]              = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(obstacles, x, 0, t);
        [three, indexThree, details] = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(obstacles, x, 0, t);
        verifyEqual(testCase, one, two);
        verifyEqual(testCase, two, three);
        verifyEqual(testCase, indexTwo, indexThree);
        verifyEqual(testCase, indexTwo(3), uint32(1));
        verifyEqual(testCase, indexTwo(5), uint32(2));
        verifySize(testCase, details.MinimumClearance_deg, size(x));
    end
end

function testSelectionReportsTheExecutedRanking(testCase)
    s = testCase.TestData;
    [result, diagnosis] = obstacleAvoidance.planTrajectory([], s.Initial, s.Goal, s.Limits);
    verifyTrue(testCase, result.Success);
    verifyEqual(testCase, diagnosis.Selection.ColumnNames, ["ArrivalTime_s", "MotionLength_deg", "CandidateIndex"]);
    verifyEqual(testCase, diagnosis.Selection.Values(1, 1), result.ArrivalTime_s);
    verifyFalse(testCase, isfield(diagnosis.Selection, 'UtilizationTieBreak'));
end

function testPartialRoutesAndVisibilityAttemptsSurviveOutputAssembly(testCase)
    s        = testCase.TestData;
    obstacle = obstacleAvoidance.obstacles.createObstacle('box', 0, [-1;1;1;-1], [-1;-1;1;1], 0);
    planningContext = obstacleAvoidance.obstacles.createPlanningContext(obstacle, s.Initial, s.Goal);
    planningContext = obstacleAvoidance.search.addRouteSearchGeometry(planningContext, s.Initial, s.Goal, s.Options);
    routeSearchGeometry = planningContext.routeSearchGeometry;
    graph    = obstacleAvoidance.search.createVisibilityGraph(s.Limits, planningContext);
    routes   = obstacleAvoidance.search.searchRoutes(s.Initial, s.Goal, s.Limits, s.Options, planningContext, graph);
    routes.TimedSearchAttempted = true;
    routes.TimedSearchRecord    = struct('LayerTimes_s', [0;10], 'CandidateLayerCount', 2, 'NodeCount', 2, ...
        'WaitEdgeCount', 1, 'MotionEdgeCount', 1, 'ExpandedCount', 2, 'RejectedTransitionCount', 0, ...
        'ExploredNodes_deg', [-3 0;0 -2], 'FrontierNodes_deg', [0 -2], ...
        'BestPartialRoute_deg', [-3 0;0 -2], 'SelectedGoalLayerIndex', [], 'ReachableGoalLayerCount', 0);
    routes.SpatialSearchRecord.BestPartialRoute_deg = [-3 0;-1 0];
    guesses = obstacleAvoidance.search.createPathGuesses(s.Initial, s.Goal, s.Limits, s.Options, routes, routeSearchGeometry.shape.Vertices);
    search  = obstacleAvoidance.search.createSearchDiagnostics(planningContext, graph, routes, guesses);
    verifyEqual(testCase, search.TimedBestPartialRoute_deg, [-3 0;0 -2]);
    verifyEqual(testCase, search.SpatialBestPartialRoute_deg, [-3 0;-1 0]);
    verifyEqual(testCase, search.BestPartialRoute_deg, search.TimedBestPartialRoute_deg);
    [record, ~] = obstacleAvoidance.planner.initializePlanningRecord(obstacle, s.Initial, s.Goal, s.Limits, s.Options, obstacleAvoidance.validateTrajectory());
    record.SearchDiagnostics.GraphSearch = search;record.Seeds = guesses;
    empty = obstacleAvoidance.planner.tryDirectAndFixedTimeMotions();
    record.SearchDiagnostics.DirectAttempt       = empty.DirectAttempt;
    record.SearchDiagnostics.FixedClockExcursion = empty.ExcursionDiagnostics;
    record.SearchDiagnostics.SelectionPolicy     = struct();
    [~, diagnosis] = obstacleAvoidance.planner.createPublicOutputs(record, true);
    verifyTrue(testCase, any(diagnosis.VisibilityAttempts.Field=="EdgeRejectionReasons"));
    verifyTrue(testCase, any(diagnosis.VisibilityAttempts.Field=="GraphComponents"));
end

function testRequestedPolynomialOutputsMatchFullEvaluation(testCase)
    % Fewer requested derivatives must leave the requested values unchanged.
    s = testCase.TestData;
    [result, ~] = obstacleAvoidance.planTrajectory([], s.Initial, s.Goal, s.Limits);
    polynomial = result.Polynomial;
    % Exercise each times covered by this regression.
    for times = {result.time_s, zeros(0, 1), NaN}
        t = times{1};
        [allTime, p, v, a, j]           = bmtpEngine.evaluatePolynomial(polynomial, t);
        [twoTime, twoP]                 = bmtpEngine.evaluatePolynomial(polynomial, t);
        [threeTime, threeP, threeV]     = bmtpEngine.evaluatePolynomial(polynomial, t);
        [fourTime, fourP, fourV, fourA] = bmtpEngine.evaluatePolynomial(polynomial, t);
        verifyEqual(testCase, bmtpEngine.evaluatePolynomial(polynomial, t), allTime);
        verifyEqual(testCase, {twoTime, twoP}, {allTime, p});
        verifyEqual(testCase, {threeTime, threeP, threeV}, {allTime, p, v});
        verifyEqual(testCase, {fourTime, fourP, fourV, fourA}, {allTime, p, v, a});
        verifySize(testCase, j, [numel(t), 2]);
    end
end

function testUnsupportedDirectGuessIsNotCalledMultiWaypoint(testCase)
    s     = testCase.TestData;
    guess = obstacleAvoidance.search.createPathGuesses(s.Initial, s.Goal, s.Limits, s.Options, struct(), []);
    [~, template] = obstacleAvoidance.planner.initializePlanningRecord([], s.Initial, s.Goal, s.Limits, s.Options, obstacleAvoidance.validateTrajectory());
    context = struct('SummaryTemplate', template, 'EnclosureGeometry', struct(), 'Enclosure', struct());
    [candidate, ~, details] = obstacleAvoidance.planner.solveDynamicPathGuess([], s.Initial, s.Goal, s.Limits, s.Options, guess, obstacleAvoidance.planner.createStageTiming(), context);
    verifyEqual(testCase, candidate.TerminationReason, "unsupportedDynamicDirectGuess");
    verifyEqual(testCase, details.FirstUnsupportedFeature, "directGuessWithoutWaitSchedule");
    verifyFalse(testCase, details.FallbackAttempted);
    verifyTrue(testCase, all(isfield(candidate, {'ArrivalTime_s', 'TrajectoryDuration_s'})));
    verifyFalse(testCase, any(isfield(candidate, {'FinalTime_s', 'MotionDuration_s'})));
end
