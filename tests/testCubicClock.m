function tests = testCubicClock
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testCubicClock.m')
% PURPOSE: Check variable-clock detours, coordinate changes, and infeasibility.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Independent physical and collision checks of returned cubic motion.
% UNITS: Coordinate units and seconds.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'),fullfile(root,'examples'));
    testCase.TestData.Vertices_units = [-8,7;-5,7;-5,-4;5,-4;5,7;8,7;8,-7;-8,-7];
end

function testStaticDetourHasBoundedJerkJumps(testCase)
    r = exampleStaticUShapedObstacle(struct('PlotOutputs',false,'Verbose',false));
    verifyTrue(testCase,r.Success,r.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
    verifyEqual(testCase,r.SolverDiagnostics.Identifier,"cubicJerkClock");
    highPowers = r.Polynomial.positionPower_units(:,:,5:end);
    verifyEqual(testCase,highPowers,zeros(size(highPowers)));
    jerkJump_units_s3 = diff(r.Polynomial.jerkPower_units_s3(:,:,1),1,1);
    verifyGreaterThan(testCase,max(abs(jerkJump_units_s3),[],'all'),0.1);
    verifyLessThan(testCase,r.TrajectoryDuration_s,21);
end

function testTranslatedSwappedAxes(testCase)
    shift_units = [3,-2];
    vertices_units = testCase.TestData.Vertices_units(:,[2,1])+shift_units;
    obstacle = obstacleAvoidance.obstacles.createObstacle('transformed U',[0;120], ...
        vertices_units(:,1),vertices_units(:,2),0.2);
    initial = struct('time_s',0,'position_units',shift_units);
    goal = struct('time_s',120,'position_units',[-10,0]+shift_units);
    limits = struct('xInterval_units',[-20,20],'yInterval_units',[-20,20], ...
        'maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[0.75,0.75], ...
        'maxJerk_units_s3',[2.5,2.5]);
    r = planner(obstacle,initial,goal,limits,struct('GoalTimeMode','earliestArrival'));
    verifyTrue(testCase,r.Success,r.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
end

function testInfeasibleDetourReturnsStableFailure(testCase)
    vertices_units = testCase.TestData.Vertices_units;
    obstacle = obstacleAvoidance.obstacles.createObstacle('U',[0;15], ...
        vertices_units(:,1),vertices_units(:,2),0.2);
    initial = struct('time_s',0,'position_units',[0,0]);
    goal = struct('time_s',15,'position_units',[0,-10]);
    limits = struct('maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[0.75,0.75], ...
        'maxJerk_units_s3',[2.5,2.5]);
    r = planner(obstacle,initial,goal,limits,struct('GoalTimeMode','earliestArrival'));
    verifyFalse(testCase,r.Success);
    verifyNotEmpty(testCase,r.Message);
    verifyNotEmpty(testCase,r.TerminationReason);
    verifyEmpty(testCase,r.time_s);
end
