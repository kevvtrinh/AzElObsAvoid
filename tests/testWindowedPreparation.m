function tests = testWindowedPreparation
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testWindowedPreparation.m')
% PURPOSE: Verify partial preparation preserves source geometry and time coverage.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Function-based tests.
% UNITS: Coordinate units and seconds.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'));
    time_s = (0:10).';
    ring = [0 0;2 0;2 1;1 0.5;0 1];
    x = arrayfun(@(t)ring(:,1)+t,time_s,'UniformOutput',false);
    y = repmat({ring(:,2)},size(time_s));
    testCase.TestData.Source = obstacleAvoidance.obstacles.createObstacle('translated concavity',time_s,x,y,0);
end

function testWindowAndOverlap(testCase)
    source = testCase.TestData.Source;
    first = obstacleAvoidance.obstacles.prepareObstacles(source,[2,4]);
    verifyEqual(testCase,find(first.InternalPreparation.SamplePrepared),(3:5).');
    verifyEqual(testCase,find(first.InternalPreparation.IntervalPrepared),(3:4).');
    second = obstacleAvoidance.obstacles.prepareObstacles(first,[3.5,5.5]);
    verifyEqual(testCase,find(second.InternalPreparation.SamplePrepared),(3:7).');
    verifyEqual(testCase,find(second.InternalPreparation.IntervalPrepared),(3:6).');
    verifyEqual(testCase,second.x_units,source.x_units);
    verifyEqual(testCase,second.time_s,source.time_s);
    verifyEqual(testCase,second.InternalPreparation.SampleShapes{4},first.InternalPreparation.SampleShapes{4});
end

function testFullAndWindowGeometryAgree(testCase)
    source = testCase.TestData.Source;
    full = obstacleAvoidance.obstacles.prepareObstacles(source);
    partial = obstacleAvoidance.obstacles.prepareObstacles(source,[2.5,5.5]);
    for time_s = [2.5,3,4.25,5.5]
        expected = obstacleAvoidance.obstacles.preparedShapeAtTime(full,time_s);
        actual = obstacleAvoidance.obstacles.preparedShapeAtTime(partial,time_s);
        verifyEqual(testCase,actual.Vertices,expected.Vertices);
    end
    verifyEqual(testCase,obstacleAvoidance.obstacles.createTimeCells(partial,2.5,5.5), ...
        obstacleAvoidance.obstacles.createTimeCells(full,2.5,5.5));
end

function testPointQueriesAndInvalidation(testCase)
    source = testCase.TestData.Source;
    partial = obstacleAvoidance.obstacles.prepareObstacles(source,[2,4]);
    verifyError(testCase,@()obstacleAvoidance.obstacles.preparedShapeAtTime(partial,7.5), ...
        'preparedShapeAtTime:UnpreparedTime');
    expected = obstacleAvoidance.obstacles.shapeAtTime(source,7.5);
    actual = obstacleAvoidance.obstacles.shapeAtTime(partial,7.5);
    verifyEqual(testCase,actual.Vertices,expected.Vertices);
    partial.x_units{4} = partial.x_units{4}+0.2;
    changed = obstacleAvoidance.obstacles.prepareObstacles(partial,[3,3]);
    verifyEqual(testCase,find(changed.InternalPreparation.SamplePrepared),4);
    verifyFalse(testCase,any(changed.InternalPreparation.IntervalPrepared));
    original = obstacleAvoidance.obstacles.shapeAtTime(source,3);
    verifyNotEqual(testCase,changed.InternalPreparation.SampleShapes{4}.Vertices,original.Vertices);
end

function testEmptyEventsAndDisconnectedShapes(testCase)
    x = [0;1;1;0;NaN;3;4;4;3]; y = [0;0;1;1;NaN;0;0;1;1];
    empty = zeros(0,1);
    source = obstacleAvoidance.obstacles.createObstacle('appearance',[0;1;2;3;4], ...
        {empty;x;x+1;x+1;empty},{empty;y;y;y;empty},0);
    full = obstacleAvoidance.obstacles.prepareObstacles(source);
    partial = obstacleAvoidance.obstacles.prepareObstacles(source,[0.5,3.5]);
    verifyEqual(testCase,obstacleAvoidance.obstacles.createTimeCells(partial,0.5,3.5), ...
        obstacleAvoidance.obstacles.createTimeCells(full,0.5,3.5));
    for time_s = [0,0.5,1,1.5,3,3.5,4]
        expected = obstacleAvoidance.obstacles.shapeAtTime(full,time_s);
        actual = obstacleAvoidance.obstacles.shapeAtTime(partial,time_s);
        verifyEqual(testCase,actual.Vertices,expected.Vertices);
    end
end

function testStaticAndInactiveHistory(testCase)
    source = testCase.TestData.Source;
    inactive = obstacleAvoidance.obstacles.prepareObstacles(source,[20,21]);
    verifyFalse(testCase,any(inactive.InternalPreparation.SamplePrepared));
    shape = obstacleAvoidance.obstacles.preparedShapeAtTime(inactive,20);
    verifyEmpty(testCase,shape.Vertices);
    static = obstacleAvoidance.obstacles.createObstacle('static',0,[0;1;1;0],[0;0;1;1],0);
    static = obstacleAvoidance.obstacles.prepareObstacles(static,[20,21]);
    verifyTrue(testCase,static.InternalPreparation.SamplePrepared);
    verifyEqual(testCase,area(obstacleAvoidance.obstacles.preparedShapeAtTime(static,20)),1);
end

function testInvalidRange(testCase)
    verifyError(testCase,@()obstacleAvoidance.obstacles.prepareObstacles(testCase.TestData.Source,[3,2]), ...
        'prepareObstacles:InvalidTimeRange');
end

function testInactiveSourceBeforeMovingDetour(testCase)
    x = [-0.5;0.5;0.5;-0.5]; y = [-0.5;-0.5;0.5;0.5];
    moving = obstacleAvoidance.obstacles.createObstacle('crossing',[0;6;12], ...
        {x;x;x},{y-0.3;y;y+0.3},0.1);
    inactive = obstacleAvoidance.obstacles.createObstacle('inactive',[20;21],{x;x},{y;y},0);
    initial = struct('time_s',0,'position_units',[-4,0]);
    goal = struct('time_s',12,'position_units',[4,1]);
    reference = planner(moving,initial,goal);
    actual = planner([inactive;moving],initial,goal);
    verifyTrue(testCase,reference.Success,reference.Message);
    verifyTrue(testCase,actual.Success,actual.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(actual).Passed);
    verifyEqual(testCase,actual.Route_units,reference.Route_units);
    verifyEqual(testCase,actual.Polynomial,reference.Polynomial);
end
