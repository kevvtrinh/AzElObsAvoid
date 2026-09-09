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

function testDirect(testCase)
    r = planner([],testCase.TestData.Initial,testCase.TestData.Goal,testCase.TestData.Limits,testCase.TestData.Options);
    verifyTrue(testCase,r.Success,r.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
    verifyEqual(testCase,r.ArrivalTime_s,12,'AbsTol',1e-8);
end

function testDetourAndTampering(testCase)
    r = planner();
    verifyTrue(testCase,r.Success,r.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
    altered = r; altered.position_units(2,1) = altered.position_units(2,1)+0.1;
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(altered).Passed);
    altered = r; altered.Polynomial.jerkPower_units_s3(1,1,1) = 1e4;
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(altered).Passed);
    altered = r; altered.PlaneCertificate.Regions_units = {};
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(altered).Passed);
end

function testNoPath(testCase)
    obstacle = struct('Vertices_units',[-1 -5;1 -5;1 5;-1 5]);
    r = planner(obstacle,testCase.TestData.Initial,testCase.TestData.Goal,testCase.TestData.Limits,testCase.TestData.Options);
    verifyFalse(testCase,r.Success);
    verifyEqual(testCase,r.TerminationReason,"noVisibilityRoute");
    verifyEmpty(testCase,r.time_s);
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
    graph = obstacleAvoidance.search.createVisibilityGraph(scene,[-0.5 0],[0.5 0],testCase.TestData.Limits,opts);
    verifyEqual(testCase,graph.RouteLength_units,1,'AbsTol',1e-12);
    graph = obstacleAvoidance.search.createVisibilityGraph(scene,[0 0],[4 0],testCase.TestData.Limits,opts);
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
        actual = obstacleAvoidance.search.createVisibilityGraph(scene,initial_units,goal_units,limits,options);
        verifyEqual(testCase,actual.IsConnected,reference.IsConnected);
        verifyEqual(testCase,actual.RouteLength_units,reference.RouteLength_units,'AbsTol',1e-8);
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
        actual = obstacleAvoidance.search.createVisibilityGraph(scene,starts(k,:),goals(k,:),limits,options);
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
                graph=obstacleAvoidance.search.createVisibilityGraph(scene,starts(k,:)*rotation+offset, ...
                    goals(k,:)*rotation+offset,limits,struct('ConstraintTolerance',1e-8));
                verifyTrue(testCase,graph.IsConnected);
                verifyEqual(testCase,graph.RouteLength_units,lengths(k),'AbsTol',1e-8);
            end
        end
    end
end
