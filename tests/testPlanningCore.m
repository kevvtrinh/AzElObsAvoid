function tests = testPlanningCore
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testPlanningCore.m')
% PURPOSE: Exercise public inputs, direct/detour motion, no-path results,
%          independent rejection, and exact visibility on multiple rings.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Function-based test results.
% UNITS: Coordinate units and seconds.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root, fullfile(root,'trajectory'));
    testCase.TestData.Initial = struct('time_s',0,'position_units',[-4 0]);
    testCase.TestData.Goal = struct('time_s',12,'position_units',[4 0]);
    testCase.TestData.Limits = struct('xInterval_units',[-6 6],'yInterval_units',[-4 4], ...
        'maxVelocity_units_s',[2 2],'maxAcceleration_units_s2',[2 2],'maxJerk_units_s3',[4 4]);
    testCase.TestData.Options = struct('GoalTimeMode','fixedArrival');
end

function testProposalVisibilityBoundaryAndInteriorRejection(testCase)
    shape = polyshape([0,1,1,0],[0,0,1,1]);
    [edgeStart,edgeEnd] = obstacleAvoidance.geometry.boundaryToEdges(shape,1e-12);
    first = [-2,-1;-1,0.5;-3,0.5;-1,1;-1,2;0,0.5;0.25,0.25];
    second = [-1,-1;2,0.5;0.25,0.5;2,1;2,2;-1,0.5;0.75,0.75];
    expected = [true;false;false;false;true;false;false];
    verifyEqual(testCase,obstacleAvoidance.search.checkVisibilitySegments( ...
        first,second,shape,edgeStart,edgeEnd),expected);
    % No midpoint queries survive when every segment hits a boundary.
    blocked = [2,3,4,6];
    verifyEqual(testCase,obstacleAvoidance.search.checkVisibilitySegments( ...
        first(blocked,:),second(blocked,:),shape,edgeStart,edgeEnd),false(4,1));
    verifyEqual(testCase,obstacleAvoidance.search.checkVisibilitySegments( ...
        first,second,polyshape(),zeros(0,2),zeros(0,2)),true(size(expected)));
    shape = polyshape([0,4,4,0,NaN,1,1,3,3],[0,0,4,4,NaN,1,3,3,1]);
    [edgeStart,edgeEnd] = obstacleAvoidance.geometry.boundaryToEdges(shape,1e-12);
    first = [1.25,2;1.25,2;0.25,0.25;-2,5;1,1];
    second = [2.75,2;3.5,2;0.75,0.75;6,5;3,1];
    verifyEqual(testCase,obstacleAvoidance.search.checkVisibilitySegments( ...
        first,second,shape,edgeStart,edgeEnd),[true;false;false;true;false]);
end

function testDirect(testCase)
    r = planner([],testCase.TestData.Initial,testCase.TestData.Goal,testCase.TestData.Limits,testCase.TestData.Options);
    verifyTrue(testCase,r.Success,r.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
    verifyEqual(testCase,r.ArrivalTime_s,12,'AbsTol',1e-8);
    verifyEqual(testCase,r.Diagnostics.VisibilityGraph.SearchKind,"initialSpatialSnapshot");
    verifyTrue(testCase,r.Diagnostics.VisibilityGraph.GraphIsFullyEnumerated);
end

function testTimeToleranceIsIndependentOfConstraintTolerance(testCase)
    initial = struct('time_s', 0, 'position_units', [-1, 0]);
    goal    = struct('time_s', 10, 'position_units', [1, 0]);
    limits  = struct( ...
        'xInterval_units',          [-5, 5], ...
        'yInterval_units',          [-5, 5], ...
        'maxVelocity_units_s',      [10, 10], ...
        'maxAcceleration_units_s2', [10, 10], ...
        'maxJerk_units_s3',         [10, 10]);
    looseTolerance      = 1e-3;
    tightTolerance      = 1e-8;
    crossedTolerances  = [looseTolerance, tightTolerance; tightTolerance, looseTolerance];
    expectedAcceptance = [false; true];
    clockOffset_s       = 1e-4;
    controlPoint_units  = zeros(1, 6, 2);
    controlPoint_units(1, :, 1) = [-1, -1, -1, 1, 1, 1];

    for settingIndex = 1:size(crossedTolerances, 1)
        options = struct( ...
            'GoalTimeMode',          'fixedArrival', ...
            'ConstraintTolerance',   crossedTolerances(settingIndex, 1), ...
            'ArrivalTimeTolerance_s', crossedTolerances(settingIndex, 2));
        baseResult = planner([], initial, goal, limits, options);
        assertTrue(testCase, baseResult.Success, baseResult.Message);
        assertTrue(testCase, obstacleAvoidance.validateTrajectory(baseResult).Passed);

        seed = struct( ...
            'position_units', [initial.position_units; goal.position_units], ...
            'tau',            [0; 1]);
        request = bmtpEngine.pipeline.createSolveRequest(seed, ...
            struct('regions_units', {cell(0, 1)}, ...
            'coverage', struct('Passed', true, 'ExactRegionCount', 0)), ...
            struct('initialState', baseResult.Inputs.initialState, ...
            'goalState', baseResult.Inputs.goalState, ...
            'limits', baseResult.Diagnostics.Limits, ...
            'options', baseResult.Options));
        preparedMotion = bmtpEngine.pipeline.prepareFinalMotion(request, controlPoint_units, ...
            request.MotionHorizon_s + clockOffset_s);
        output = createProvenOutput(baseResult, request, preparedMotion);
        validation = obstacleAvoidance.validateTrajectory(output);

        verifyEqual(testCase, preparedMotion.Success, expectedAcceptance(settingIndex));
        verifyEqual(testCase, validation.Passed, expectedAcceptance(settingIndex));
    end

    for constraintTolerance = [tightTolerance, looseTolerance]
        options = struct( ...
            'GoalTimeMode',          'fixedArrival', ...
            'ConstraintTolerance',   constraintTolerance, ...
            'ArrivalTimeTolerance_s', tightTolerance);
        result = planner([], initial, goal, limits, options);
        assertTrue(testCase, result.Success, result.Message);
        result.Inputs.goalState.time_s = result.Diagnostics.Polynomial.FinalTime_s + clockOffset_s;
        validation = obstacleAvoidance.validateTrajectory(result);
        verifyFalse(testCase, validation.Passed);
        verifyFalse(testCase, validation.EndpointStatesMatched);

        if constraintTolerance == looseTolerance
            altered = result;
            altered.Diagnostics.Polynomial.FinalTime_s = altered.Diagnostics.Polynomial.FinalTime_s + clockOffset_s;
            validation = obstacleAvoidance.validateTrajectory(altered);
            verifyFalse(testCase, validation.SegmentTimingConsistent);

            altered = result;
            altered.Diagnostics.Polynomial.SegmentStartTime_s(1) = ...
                altered.Diagnostics.Polynomial.SegmentStartTime_s(1) + clockOffset_s;
            validation = obstacleAvoidance.validateTrajectory(altered);
            verifyFalse(testCase, validation.SegmentTimingConsistent);
            verifyFalse(testCase, validation.EndpointStatesMatched);

            altered           = result;
            altered.time_s(1) = altered.time_s(1) + clockOffset_s;
            validation = obstacleAvoidance.validateTrajectory(altered);
            verifyFalse(testCase, validation.SampledHistoriesMatched);

            altered             = result;
            altered.time_s(end) = altered.time_s(end) - clockOffset_s;
            validation = obstacleAvoidance.validateTrajectory(altered);
            verifyFalse(testCase, validation.SampledHistoriesMatched);

            altered = result;
            altered.Inputs.initialState.time_s = ...
                altered.Inputs.initialState.time_s + clockOffset_s;
            validation = obstacleAvoidance.validateTrajectory(altered);
            verifyFalse(testCase, validation.SeparationProofValid);
        end
    end
end

function testC3ChordDropsRoundoffZeroPhases(testCase)
    % A regime-boundary hold can evaluate to a positive 1e-16-second
    % remnant. It must not become the smoothing kernel and erase the chord.
    limits = struct('maxVelocity_units_s',[10,10], ...
        'maxAcceleration_units_s2',[10,10], ...
        'maxJerk_units_s3',[10,10]);
    start_units = [-5,0];
    goal_units = [0.201,2.999];
    [controls_units,durations_s,powers_units] = ...
        bmtpEngine.motion.createC3Chord(start_units,goal_units,limits);
    verifyGreaterThan(testCase,min(durations_s),1e-6);
    verifyEqual(testCase,squeeze(controls_units(1,1,:)).', ...
        start_units,'AbsTol',1e-12);
    verifyEqual(testCase,squeeze(controls_units(end,end,:)).', ...
        goal_units,'AbsTol',1e-10);
    verifyEqual(testCase,sum(squeeze(powers_units(end,:,:)),2).', ...
        goal_units,'AbsTol',1e-10);
end

function testJerkChordPreservesShortPhysicalRampsBesideLongCruise(testCase)
    limits = struct('maxVelocity_units_s',[1e-9,1e-9], ...
        'maxAcceleration_units_s2',[1e5,1e5], ...
        'maxJerk_units_s3',[1e18,1e18]);
    [controls_units,durations_s] = bmtpEngine.motion.createJerkLimitedChord( ...
        [0,0],[1,0],limits,3);
    verifyGreaterThan(testCase,numel(durations_s),1);
    verifyGreaterThan(testCase,min(durations_s),0);
    verifyEqual(testCase,squeeze(controls_units(1,1,:)).',[0,0], ...
        'AbsTol',1e-14);
    verifyEqual(testCase,squeeze(controls_units(end,end,:)).',[1,0], ...
        'AbsTol',1e-12);
end

function testDetourAndTampering(testCase)
    options = planner();
    verifyEqual(testCase, options.GoalTimeMode, "fixedArrival");
    verifyFalse(testCase, isfield(options, "Success"));
    obstacle = struct("Name", "center block", ...
        "Vertices_units", [-1 -1; 1 -1; 1 1; -1 1], ...
        "SafetyMargin_units", 0.25);
    initial = struct("time_s", 0, "position_units", [-4 0]);
    goal = struct("time_s", 12, "position_units", [4 0]);
    limits = struct("xInterval_units", [-180 180], "yInterval_units", [-90 90], ...
        "maxVelocity_units_s", [2 2], "maxAcceleration_units_s2", [2 2], ...
        "maxJerk_units_s3", [4 4]);
    r = planner(obstacle, initial, goal, limits, options);
    verifyTrue(testCase,r.Success,r.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
    verifyEqual(testCase,r.Diagnostics.VisibilityGraph.SearchKind,"initialSpatialSnapshot");
    verifyTrue(testCase,r.Diagnostics.VisibilityGraph.GraphIsFullyEnumerated);
    altered = r; altered.position_units(2,1) = altered.position_units(2,1)+0.1;
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(altered).Passed);
    altered = r; altered.Diagnostics.Polynomial.jerkPower_units_s3(1,1,1) = 1e4;
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(altered).Passed);
    altered = r; altered.Diagnostics.SeparationProof.Regions_units = {};
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(altered).Passed);
end

function testNoPath(testCase)
    obstacle = struct('Vertices_units',[-1 -5;1 -5;1 5;-1 5]);
    r = planner(obstacle,testCase.TestData.Initial,testCase.TestData.Goal,testCase.TestData.Limits,testCase.TestData.Options);
    verifyFalse(testCase,r.Success);
    verifyEqual(testCase,r.TerminationReason,"noVisibilityRoute");
    verifyEmpty(testCase,r.time_s);
    earliest = planner(obstacle,testCase.TestData.Initial,testCase.TestData.Goal, ...
        testCase.TestData.Limits,struct('GoalTimeMode','earliestArrival'));
    verifyFalse(testCase,earliest.Success);
    verifyEqual(testCase,earliest.TerminationReason,"noVisibilityRoute");
    verifyEqual(testCase,earliest.Diagnostics.Attempts.FailureKind,"noSpatialRoute");
end

function testMovingCellFringeBlocksTerminalReachability(testCase)
    % The moving-cell enclosure hulls carried triangles with margin squares, so it
    % reaches past every protected sample. A fixed goal that is free of the
    % samples but inside that fringe must still be proven unreachable before
    % any planning stage runs.
    lower = [-1,-1;1,-1;1,1;-1,1];
    upper = [-1,-1;1,-1;0.5,1;-1,1];
    obstacle = obstacleAvoidance.obstacles.createObstacle('swept fringe',[0;10], ...
        {lower(:,1);upper(:,1)},{lower(:,2);upper(:,2)},1, ...
        struct('vertexCorrespondence','sourceIndex'));
    prepared = obstacleAvoidance.obstacles.prepareObstacles(obstacle,[0,10],true);
    goal = struct('time_s',10,'position_units',[2.3,0]);
    verifyTrue(testCase,all(cellfun(@max,prepared.x_units) < goal.position_units(1)));
    verifyTrue(testCase,prepared.InternalPreparation.IntervalUsesMovingCells(1));
    initial = struct('time_s',0,'position_units',[-5,0]);
    limits = struct('xInterval_units',[-8,8],'yInterval_units',[-8,8], ...
        'maxVelocity_units_s',[3,3],'maxAcceleration_units_s2',[2,2],'maxJerk_units_s3',[4,4]);
    result = planner(obstacle,initial,goal,limits,struct('GoalTimeMode','fixedArrival'));
    verifyFalse(testCase,result.Success);
    verifyEqual(testCase,result.TerminationReason,"terminalReachabilityBlocked");
end

function testInvalidInputs(testCase)
    initial = testCase.TestData.Initial; initial.position_units = [NaN 0];
    verifyError(testCase,@() planner([],initial,testCase.TestData.Goal), 'planTrajectory:InvalidState');
    goal = testCase.TestData.Goal; goal.time_s = -1;
    verifyError(testCase,@() planner([],testCase.TestData.Initial,goal), 'planTrajectory:InvalidTimeOrder');
end

function testMarginOnce(testCase)
    obstacle = obstacleAvoidance.obstacles.createObstacle('box',0,[-1;1;1;-1],[-1;-1;1;1],0.2);
    rebuilt = obstacleAvoidance.obstacles.createObstacle(obstacle,0.2);
    verifyEqual(testCase,rebuilt.x_units,obstacle.x_units);
    verifyEqual(testCase,rebuilt.originalX_units,obstacle.originalX_units);
end

function testHoleAndDisconnectedRegions(testCase)
    outer = polyshape([-2 -2;2 -2;2 2;-2 2]);
    inner = polyshape([-1 -1;1 -1;1 1;-1 1]);
    shape = subtract(outer,inner);
    scene = struct('ProtectedShape',shape);
    opts = struct('ConstraintTolerance',1e-8);
    vertexVisibility = obstacleAvoidance.search.createVertexVisibility(scene,testCase.TestData.Limits,opts);
    graph = obstacleAvoidance.search.createVisibilityGraph(vertexVisibility,[-0.5 0],[0.5 0]);
    verifyEqual(testCase,graph.RouteLength_units,1,'AbsTol',1e-12);
    graph = obstacleAvoidance.search.createVisibilityGraph(vertexVisibility,[0 0],[4 0]);
    verifyFalse(testCase,graph.IsConnected);
end

function testVisibilityMatchesExhaustiveReference(testCase)
    rng(73);
    limits = struct('xInterval_units',[-12 12],'yInterval_units',[-10 10]);
    options = struct('ConstraintTolerance',1e-8);
    for k = 1:30
        scene = struct('ProtectedShape',{},'ProtectedVertices_units',{});
        for j = 1:mod(k,6)+1
            points_units = rand(9,2)*1.4 + [-6+2*j,-2+rand*4];
            hull = convhull(points_units(:,1),points_units(:,2));
            shape = polyshape(points_units(hull(1:end-1),:));
            scene(j) = struct('ProtectedShape',shape,'ProtectedVertices_units',shape.Vertices);
        end
        initial_units = [-9,rand*2-1]; goal_units = [9,rand*2-1];
        reference = createVisibilityGraphBaseline(scene,initial_units,goal_units,limits,options);
        vertexVisibility = obstacleAvoidance.search.createVertexVisibility(scene,limits,options);
        actual = obstacleAvoidance.search.createVisibilityGraph(vertexVisibility,initial_units,goal_units);
        verifyEqual(testCase,actual.IsConnected,reference.IsConnected);
        verifyEqual(testCase,actual.RouteLength_units,reference.RouteLength_units,'AbsTol',1e-8);
        verifyTrue(testCase,actual.GraphIsFullyEnumerated);
        % The reduced graph keeps a subset of the exhaustive connections.
        verifyLessThanOrEqual(testCase,size(actual.AcceptedNodeIndex,1),size(reference.AcceptedNodeIndex,1));
    end
end

function testReducedGraphMatchesExhaustiveReferenceOnConcaveShapes(testCase)
    % Random star-shaped (concave) obstacles, spaced so they never overlap,
    % against the exhaustive single-ring reference graph.
    rng(91);
    warningState = warning('off','MATLAB:polyshape:repairedBySimplify');
    restoreWarning = onCleanup(@() warning(warningState)); %#ok<NASGU>
    limits = struct('xInterval_units',[-12 12],'yInterval_units',[-10 10]);
    options = struct('ConstraintTolerance',1e-8);
    for k = 1:40
        scene = struct('ProtectedShape',{},'ProtectedVertices_units',{});
        for j = 1:mod(k,4)+1
            % The reference reads one closed ring per obstacle, so redraw any
            % polygon that polyshape repaired into several regions.
            shape = polyshape();
            while shape.NumRegions ~= 1 || shape.NumHoles ~= 0
                angles = sort(rand(8,1)*2*pi);
                radii  = 0.5+rand(8,1)*0.9;
                center = [-6+3*j,rand*6-3];
                shape  = polyshape(center+[radii.*cos(angles),radii.*sin(angles)]);
            end
            scene(j) = struct('ProtectedShape',shape,'ProtectedVertices_units',shape.Vertices);
        end
        initial_units = [-9,rand*2-1]; goal_units = [9,rand*2-1];
        reference = createVisibilityGraphBaseline(scene,initial_units,goal_units,limits,options);
        vertexVisibility = obstacleAvoidance.search.createVertexVisibility(scene,limits,options);
        actual = obstacleAvoidance.search.createVisibilityGraph(vertexVisibility,initial_units,goal_units);
        verifyEqual(testCase,actual.IsConnected,reference.IsConnected);
        verifyEqual(testCase,actual.RouteLength_units,reference.RouteLength_units,'AbsTol',1e-8);
        verifyLessThanOrEqual(testCase,size(actual.NodePosition_units,1),size(reference.NodePosition_units,1));
        verifyLessThanOrEqual(testCase,size(actual.AcceptedNodeIndex,1),size(reference.AcceptedNodeIndex,1));
    end
end

function testBatchedContactsHolesAndConcavities(testCase)
    limits = struct('xInterval_units',[-12 12],'yInterval_units',[-10 10]);
    options = struct('ConstraintTolerance',1e-8);
    uShape = polyshape([-4,5;-2,5;-2,-2;2,-2;2,5;4,5;4,-4;-4,-4]);
    ring = subtract(polyshape([-4,-4;4,-4;4,4;-4,4]),polyshape([-2,-2;2,-2;2,2;-2,2]));
    islands = union(polyshape([-3,-1;-1,-1;-1,1;-3,1]),polyshape([1,-1;3,-1;3,1;1,1]));
    touching = union(polyshape([-3,-3;0,-3;0,0;-3,0]),polyshape([0,0;3,0;3,3;0,3]));
    shapes = {uShape,ring,ring,islands,touching};
    starts = [0,0;0,0;-7,0;-7,1;-7,0];
    goals = [0,-7;1,1;7,0;7,1;7,0];
    % Analytic boundary routes: U opening and outer corners; ring exterior;
    % and straight tangent routes along the remaining component boundaries.
    expectedLength_units = [16+sqrt(29);sqrt(2);18;14;14];
    for k = 1:numel(shapes)
        scene = struct('ProtectedShape',shapes{k},'ProtectedVertices_units',shapes{k}.Vertices);
        vertexVisibility = obstacleAvoidance.search.createVertexVisibility(scene,limits,options);
        actual = obstacleAvoidance.search.createVisibilityGraph(vertexVisibility,starts(k,:),goals(k,:));
        verifyTrue(testCase,actual.IsConnected);
        verifyEqual(testCase,actual.RouteLength_units,expectedLength_units(k),'AbsTol',1e-8);
    end
end

function testReflectedAndTranslatedConcavities(testCase)
    % The occupied side must come from filled geometry, including hole rings.
    uShape=polyshape([-4,5;-2,5;-2,-2;2,-2;2,5;4,5;4,-4;-4,-4]);
    ring=subtract(polyshape([-4,-4;4,-4;4,4;-4,4]),polyshape([-2,-2;2,-2;2,2;-2,2]));
    shapes={uShape,ring}; starts=[0,0;-7,0]; goals=[0,-7;7,0]; lengths=[16+sqrt(29),18];
    transforms=cat(3,eye(2),[-1,0;0,1],[0,-1;1,0],[0,1;1,0]);
    for translation=[0,128]
        offset=[translation,-2*translation];
        limits=struct('xInterval_units',[-12,12]+offset(1),'yInterval_units',[-12,12]+offset(2));
        for transform=1:size(transforms,3)
            rotation=transforms(:,:,transform);
            for k=1:numel(shapes)
                vertices=shapes{k}.Vertices*rotation+offset;
                shape=polyshape(vertices(:,1),vertices(:,2));
                scene=struct('ProtectedShape',shape);
                vertexVisibility=obstacleAvoidance.search.createVertexVisibility(scene,limits,struct('ConstraintTolerance',1e-8));
                graph=obstacleAvoidance.search.createVisibilityGraph(vertexVisibility,starts(k,:)*rotation+offset, ...
                    goals(k,:)*rotation+offset);
                verifyTrue(testCase,graph.IsConnected);
                verifyEqual(testCase,graph.RouteLength_units,lengths(k),'AbsTol',1e-8);
            end
        end
    end
end

function output = createProvenOutput(baseResult, request, preparedMotion)
    % Build an adversarial validator fixture without stale planner decisions.
    roundoffReserve_units = baseResult.Diagnostics.SeparationProof.RoundoffReserve_units;
    target_units = baseResult.Diagnostics.SeparationProof.RequiredGap_units - roundoffReserve_units;
    motionOutput = bmtpEngine.pipeline.createMotionOutput(struct(), request, preparedMotion);
    output = baseResult;
    output.time_s                = motionOutput.time_s;
    output.position_units        = motionOutput.position_units;
    output.velocity_units_s      = motionOutput.velocity_units_s;
    output.acceleration_units_s2 = motionOutput.acceleration_units_s2;
    output.jerk_units_s3         = motionOutput.jerk_units_s3;
    output.ArrivalTime_s         = motionOutput.ArrivalTime_s;
    output.MotionLength_units    = motionOutput.MotionLength_units;
    output.Diagnostics.Polynomial                      = motionOutput.Polynomial;
    output.Diagnostics.TrajectoryDuration_s            = motionOutput.TrajectoryDuration_s;
    output.Diagnostics.IntegratedSquaredJerk_units2_s5 = motionOutput.IntegratedSquaredJerk_units2_s5;
    output.Diagnostics.MaximumConstraintViolation      = motionOutput.MaximumConstraintViolation;
    output.Diagnostics.SeparationProof = bmtpEngine.validation.checkFinalMotion( ...
        request, preparedMotion, roundoffReserve_units, target_units);
    output.Diagnostics.Validation = struct("Passed", false, ...
        "Message", "Synthetic motion has not been validated.");
end
