function tests = testDepartureSchedule
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testDepartureSchedule.m')
% PURPOSE: Verify continuous departure scheduling, symmetry, and rejection.
% INPUTS: MATLAB unit test framework and unchanged waiting examples.
% OUTPUTS: Function-based tests.
% UNITS: Coordinate units and seconds.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'),fullfile(root,'examples'));
    testCase.TestData.Barrier = exampleMovingBarrierWait(struct('PlotOutputs',false,'Verbose',false));
    testCase.TestData.Opening = exampleOpeningUShapedObstacle(struct('PlotOutputs',false,'Verbose',false));
end

function testWaitingBenchmarkQuality(testCase)
    results = {testCase.TestData.Barrier,testCase.TestData.Opening};
    references_s = [10.0903015136719,13.617522354126];
    for k = 1:2
        r = results{k};
        verifyTrue(testCase,r.Success,r.Message);
        verifyTrue(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
        verifyLessThanOrEqual(testCase,r.TrajectoryDuration_s,references_s(k));
        verifyEqual(testCase,r.MotionLength_units,10,'AbsTol',1e-8);
        verifyGreaterThan(testCase,r.SolverDiagnostics.DepartureSchedule.DepartureDelay_s,2);
        verifyTrue(testCase,any(all(r.Polynomial.positionPower_units(:,:,2:end)==0,[2,3])));
        verifyEqual(testCase,r.SolverDiagnostics.TrajectorySocpCount,0);
    end
end

function testShiftedClockAndExchangedAxes(testCase)
    r = testCase.TestData.Barrier; source = r.Inputs.obstacles;
    obstacle = obstacleAvoidance.obstacles.createObstacle('exchanged barrier',source.time_s+7, ...
        source.originalY_units,source.originalX_units,source.safetyMargin_units);
    initial = r.Inputs.initialState; initial.position_units = fliplr(initial.position_units); initial.time_s = initial.time_s+7;
    goal = r.Inputs.goalState; goal.position_units = fliplr(goal.position_units); goal.time_s = goal.time_s+7;
    limits = r.Limits; limits.xInterval_units = r.Limits.yInterval_units; limits.yInterval_units = r.Limits.xInterval_units;
    changed = planner(obstacle,initial,goal,limits,r.Options);
    verifyTrue(testCase,changed.Success,changed.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(changed).Passed);
    verifyEqual(testCase,changed.ArrivalTime_s,r.ArrivalTime_s+7,'AbsTol',1e-8);
    verifyEqual(testCase,changed.MotionLength_units,10,'AbsTol',1e-8);
end

function testInsufficientDepartureWindow(testCase)
    r = testCase.TestData.Barrier;
    goal = r.Inputs.goalState; goal.time_s = 10.08;
    rejected = planner(r.Inputs.obstacles,r.Inputs.initialState,goal,r.Limits,r.Options);
    verifyFalse(testCase,rejected.Success);
    verifyEmpty(testCase,rejected.time_s);
end

function testWaitingPointMustRemainFree(testCase)
    r = testCase.TestData.Barrier;
    box = [-5.1,-0.1;-4.9,-0.1;-4.9,0.1;-5.1,0.1];
    incoming = obstacleAvoidance.obstacles.createObstacle('occupies waiting point',[0;1;12], ...
        {box(:,1);box(:,1);box(:,1)},{box(:,2)+8;box(:,2);box(:,2)},0);
    rejected = planner([r.Inputs.obstacles;incoming],r.Inputs.initialState,r.Inputs.goalState,r.Limits,r.Options);
    verifyFalse(testCase,rejected.Success);
    verifyEmpty(testCase,rejected.time_s);
end

function testStationaryIntervalsPreserveNonconvexGeometry(testCase)
    r = testCase.TestData.Opening;
    cells = obstacleAvoidance.obstacles.createTimeCells(r.PreparedObstacles,0,120);
    for time_s = [3,30]
        represented = polyshape();
        active = find(cells.ActiveTimeInterval_s(:,1)<time_s & cells.ActiveTimeInterval_s(:,2)>time_s);
        for k = reshape(active,1,[])
            represented = union(represented,polyshape(cells.Regions_units{k}));
        end
        scene = obstacleAvoidance.obstacles.snapshot(r.PreparedObstacles,time_s);
        verifyLessThan(testCase,area(xor(represented,scene.ProtectedShape)),1e-10);
        verifyFalse(testCase,isinterior(represented,0,0));
        verifyEqual(testCase,isinterior(represented,0,-5),time_s<7);
    end
end

function testUnknownCorrespondenceRetainsIntervalUnion(testCase)
    small = [0,0;1,0;0,1]; large = [-2,-2;3,-2;3,3;-2,3];
    source = obstacleAvoidance.obstacles.createObstacle('topology change',[0;10], ...
        {small(:,1);large(:,1)},{small(:,2);large(:,2)},0);
    prepared = obstacleAvoidance.obstacles.prepareObstacles(source);
    verifyFalse(testCase,prepared.InternalPreparation.MatchingTopology(1));
    cells = obstacleAvoidance.obstacles.createTimeCells(prepared,0,10);
    represented = polyshape();
    for k = 1:numel(cells.Regions_units)
        represented = union(represented,polyshape(cells.Regions_units{k}));
    end
    verifyLessThan(testCase,area(xor(represented,prepared.InternalPreparation.IntervalUnionShapes{1})),1e-10);
    verifyTrue(testCase,isinterior(represented,2,2));
end
