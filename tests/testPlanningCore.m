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
