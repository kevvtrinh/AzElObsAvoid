function tests = testArrivalSearchRegressions
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testArrivalSearchRegressions.m')
% PURPOSE: Exercise late feasible arrivals, selection against waiting motions, and proven skips of
%   fixed-arrival trials that cannot beat a certified wait incumbent.
% INPUTS: MATLAB unit test framework and deterministic public planner inputs.
% OUTPUTS: Independent validation and arrival-search regression checks.
% UNITS: Coordinate units, seconds, and physical derivatives.
tests = functiontests(localfunctions);
end

function setupOnce(~)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'),fullfile(root,'examples'));
end

function testCircleDetourBeatsWaiting(testCase)
    result = exampleMovingCircleNoWrap(struct('PlotOutputs',false,'Verbose',false));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyLessThanOrEqual(testCase,result.ArrivalTime_s,9);
    verifyLessThan(testCase,result.ArrivalTime_s,result.TemporalSearch.IncumbentArrival_s);
    verifyFalse(testCase,result.TemporalSearch.RetainedIncumbent);
end

function testLongRequestUsesBudgetAfterPhysicalBound(testCase)
    box = [-1,-0.5;1,-0.5;1,0.5;-1,0.5];
    wall = obstacleAvoidance.obstacles.createObstacle('static crossing',[0;180],box(:,1),box(:,2));
    remote = obstacleAvoidance.obstacles.createObstacle('remote moving box',[0;180], ...
        {box(:,1)+20;box(:,1)+30},{box(:,2)+60;box(:,2)+60});
    initial = struct('time_s',0,'position_units',[-60,0]);
    goal = struct('time_s',180,'position_units',[60,0]);
    limits = struct('maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[0.75,0.75], ...
        'maxJerk_units_s3',[2.5,2.5]);
    result = planner([wall;remote],initial,goal,limits,struct('GoalTimeMode','earliestArrival','MaxArrivalTrials',40));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyGreaterThan(testCase,result.TemporalSearch.NecessaryArrivalBound_s,60);
    verifyGreaterThanOrEqual(testCase,result.TemporalSearch.TrialTime_s, ...
        result.TemporalSearch.NecessaryArrivalBound_s-result.Options.ArrivalTimeTolerance_s);
    verifyLessThanOrEqual(testCase,numel(result.TemporalSearch.TrialTime_s),40);
    verifyLessThanOrEqual(testCase,result.ArrivalTime_s,82.5);
end

function testDisconnectedSnapshotReturnsValidatedWaitWithoutTrials(testCase)
    result = exampleMovingBarrierWait(struct('PlotOutputs',false,'Verbose',false,'MaxArrivalTrials',1));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.VisibilityGraph.SearchKind,"c3DepartureSchedule");
    verifyFalse(testCase,isfield(result,'TemporalSearch'));
    verifyGreaterThan(testCase,result.SolverDiagnostics.DepartureSchedule.DepartureDelay_s,0);
    verifyEqual(testCase,result.SolverDiagnostics.DepartureSchedule.InitialRouteTimeBound_s,Inf);
    verifyFalse(testCase,isfield(result,'FixedArrivalTrialTime_s'));
end

function testInitialRouteBoundReturnsValidatedWaitWithoutTrials(testCase)
    result = exampleOpeningUShapedObstacle(struct('PlotOutputs',false,'Verbose',false));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyFalse(testCase,isfield(result,'TemporalSearch'));
    routeBound_s=result.SolverDiagnostics.DepartureSchedule.InitialRouteTimeBound_s;
    verifyGreaterThanOrEqual(testCase,routeBound_s,result.TrajectoryDuration_s);
    verifyGreaterThan(testCase,result.SolverDiagnostics.DepartureSchedule.DepartureDelay_s,0);
end

function testChallengedDelayedChordIncumbentIsRetained(testCase)
    box=[-0.6,-1.5;0.6,-1.5;0.6,1.5;-0.6,1.5];
    obstacleTime_s=[0;5.5;6.5;15];
    obstacle=obstacleAvoidance.obstacles.createObstacle('moving box', ...
        obstacleTime_s,{box(:,1);box(:,1);box(:,1);box(:,1)}, ...
        {box(:,2);box(:,2);box(:,2)+4;box(:,2)+4},0.1);
    initial=struct('time_s',0,'position_units',[-5,0]);
    goal=struct('time_s',15,'position_units',[5,0]);
    limits=struct('xInterval_units',[-6,6], ...
        'yInterval_units',[-4,4],'maxVelocity_units_s',[2,2], ...
        'maxAcceleration_units_s2',[0.5,0.5], ...
        'maxJerk_units_s3',[2.5,2.5]);
    options=struct('GoalTimeMode','earliestArrival', ...
        'TemporalResolution_s',0.5,'MaxArrivalTrials',1);
    result=planner(obstacle,initial,goal,limits,options);
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.VisibilityGraph.SearchKind,"c3DepartureSchedule");
    verifyTrue(testCase,isfield(result,'TemporalSearch'));
    verifyTrue(testCase,result.TemporalSearch.RetainedIncumbent);
    verifyEqual(testCase,numel(result.TemporalSearch.TrialTime_s),1);
    verifyNotEqual(testCase,result.TemporalSearch.TrialTerminationReason, ...
        "goalReached");
    verifyEqual(testCase,result.ArrivalTime_s, ...
        result.TemporalSearch.IncumbentArrival_s,'AbsTol',1e-10);
    verifyFalse(testCase,isfield(result,'FixedArrivalTrialTime_s'));
end
