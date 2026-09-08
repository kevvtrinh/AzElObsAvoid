function tests = testFullEndpointStates
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testFullEndpointStates.m')
% PURPOSE: Exercise general endpoint equations, transformations, and validation.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Independent behavioral checks.
% UNITS: Coordinate units and seconds.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'));
    testCase.TestData.Initial = struct('time_s',0,'position_units',[-4,0], ...
        'velocity_units_s',[-0.2,0.1],'acceleration_units_s2',[0.04,-0.02]);
    testCase.TestData.Goal = struct('time_s',12,'position_units',[4,1], ...
        'velocity_units_s',[0.3,-0.1],'acceleration_units_s2',[-0.03,0.02]);
end

function testDirectBoundaryCombinations(testCase)
    for k = 1:6
        initial = testCase.TestData.Initial; goal = testCase.TestData.Goal;
        if k==1, initial.velocity_units_s=[]; initial.acceleration_units_s2=[]; end
        if k==2, goal.velocity_units_s=[]; goal.acceleration_units_s2=[]; end
        if k==4, initial.velocity_units_s=[0,0]; goal.velocity_units_s=[0,0]; end
        if k==5, initial.acceleration_units_s2=[0,0]; goal.acceleration_units_s2=[0,0]; end
        if k==6, initial.velocity_units_s=[0,0]; goal.acceleration_units_s2=[0,0]; end
        r = planner([],initial,goal);
        verifyTrue(testCase,r.Success,r.Message);
        verifyTrue(testCase,r.Validation.Passed);
        verifyEqual(testCase,r.ArrivalTime_s,12,'AbsTol',1e-8);
    end
end

function testAxisAndClockTranslation(testCase)
    initial = testCase.TestData.Initial; goal = testCase.TestData.Goal;
    limits = struct('maxVelocity_units_s',[2,1.5],'maxAcceleration_units_s2',[1,0.8],'maxJerk_units_s3',[3,2]);
    a = planner([],initial,goal,limits);
    for name = ["position_units","velocity_units_s","acceleration_units_s2"]
        initial.(name) = initial.(name)([2,1]); goal.(name) = goal.(name)([2,1]);
    end
    initial.position_units = initial.position_units+[7,-3]; goal.position_units = goal.position_units+[7,-3];
    initial.time_s=31; goal.time_s=43;
    for name = string(fieldnames(limits)).', limits.(name)=limits.(name)([2,1]); end
    b = planner([],initial,goal,limits);
    verifyTrue(testCase,a.Success && b.Success);
    verifyEqual(testCase,a.MotionLength_units,b.MotionLength_units,'AbsTol',1e-8);
    verifyEqual(testCase,b.velocity_units_s,a.velocity_units_s(:,[2,1]),'AbsTol',1e-8);
end

function testDetourHasMoreThanTwoGuideSegments(testCase)
    obstacle = struct('Vertices_units',[-1,-1;1,-1;1,1;-1,1],'SafetyMargin_units',0.2);
    r = planner(obstacle,testCase.TestData.Initial,testCase.TestData.Goal);
    verifyTrue(testCase,r.Success,r.Message);
    verifyGreaterThan(testCase,size(r.Route_units,1),3);
    verifyTrue(testCase,r.Validation.InterSegmentContinuous);
    verifyTrue(testCase,r.Validation.EndpointStatesMatched);
end

function testEarliestPhysicalClock(testCase)
    r = planner([],testCase.TestData.Initial,testCase.TestData.Goal,[],struct('GoalTimeMode','earliestArrival'));
    verifyTrue(testCase,r.Success,r.Message);
    verifyTrue(testCase,r.Validation.Passed);
    verifyLessThan(testCase,r.TrajectoryDuration_s,12);
    verifyEqual(testCase,r.SolverDiagnostics.DilationScale,1);
end

function testMovingCellsAndFutureEndpoint(testCase)
    x = [-0.5;0.5;0.5;-0.5]; y = [-0.5;-0.5;0.5;0.5];
    obstacle = obstacleAvoidance.obstacles.createObstacle('departing goal',[0;12],{x+4;x+4},{y+1;y+6},0.1);
    r = planner(obstacle,testCase.TestData.Initial,testCase.TestData.Goal);
    verifyTrue(testCase,r.Success,r.Message);
    verifyTrue(testCase,r.Validation.Passed);
end

function testMetadataAndSourceTampering(testCase)
    r = planner([],testCase.TestData.Initial,testCase.TestData.Goal);
    verifyTrue(testCase,r.Success,r.Message);
    altered=r; altered.Polynomial.TerminalState.velocity_units_s=[0,0];
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(altered).Passed);
    altered=r; altered.Inputs.goalState.acceleration_units_s2=[0,0];
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(altered).Passed);
    altered=r; altered.ArrivalTime_s=13;
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(altered).Passed);
    altered=r; altered.time_s(2)=altered.time_s(1);
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(altered).Passed);
end

function testPhysicalFailure(testCase)
    initial=testCase.TestData.Initial; initial.velocity_units_s=[3,0];
    r=planner([],initial,testCase.TestData.Goal);
    verifyFalse(testCase,r.Success); verifyEqual(testCase,r.TerminationReason,"dynamicEndpointInfeasible");
    initial=testCase.TestData.Initial; initial.position_units=[1000,0];
    r=planner([],initial,testCase.TestData.Goal);
    verifyFalse(testCase,r.Success); verifyEqual(testCase,r.TerminationReason,"endpointOutsideWorkspace");
end

function testEarliestDetourAndMovingDetour(testCase)
    initial=testCase.TestData.Initial; goal=testCase.TestData.Goal;
    box=struct('Vertices_units',[-1,-1;1,-1;1,1;-1,1],'SafetyMargin_units',0.2);
    r=planner(box,initial,goal,[],struct('GoalTimeMode','earliestArrival'));
    verifyTrue(testCase,r.Success,r.Message);
    verifyTrue(testCase,r.Validation.Passed);
    verifyLessThan(testCase,r.TrajectoryDuration_s,12);
    x=[-0.5;0.5;0.5;-0.5]; y=[-0.5;-0.5;0.5;0.5];
    moving=obstacleAvoidance.obstacles.createObstacle('moving detour',[0;6;12],{x;x;x},{y-0.3;y;y+0.3},0.1);
    r=planner(moving,initial,goal);
    verifyTrue(testCase,r.Success,r.Message);
    verifyTrue(testCase,r.Validation.CollisionFree);
    verifyGreaterThan(testCase,r.MotionLength_units,norm(goal.position_units-initial.position_units));
end

function testTightWindowHasHonestNumericalFailure(testCase)
    initial=struct('time_s',0,'position_units',[0,0],'velocity_units_s',[0.4,0]);
    goal=struct('time_s',3,'position_units',[4,0],'velocity_units_s',[0.2,0]);
    r=planner([],initial,goal);
    verifyFalse(testCase,r.Success);
    verifyEqual(testCase,r.TerminationReason,"noOptimizedFeasibleIterate");
    verifyGreaterThan(testCase,r.SolverDiagnostics.TrajectorySocpCount,0);
end
