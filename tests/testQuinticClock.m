function tests = testQuinticClock
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testQuinticClock.m')
% PURPOSE: Check variable-clock detours, coordinate changes, and infeasibility.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Independent physical and collision checks of returned quintic motion.
% UNITS: Coordinate units and seconds.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'),fullfile(root,'examples'));
    testCase.TestData.Vertices_units = [-8,7;-5,7;-5,-4;5,-4;5,7;8,7;8,-7;-8,-7];
end

function testStaticDetourHasContinuousJerk(testCase)
    r = exampleStaticUShapedObstacle(struct('PlotOutputs',false,'Verbose',false));
    verifyTrue(testCase,r.Success,r.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
    verifyEqual(testCase,r.SolverDiagnostics.Identifier,"quinticJerkClock");
    verifyEqual(testCase,r.Polynomial.Degree,5);
    jerk=r.Polynomial.jerkPower_units_s3;
    verifyLessThanOrEqual(testCase,max(abs(sum(jerk(1:end-1,:,:),3)-jerk(2:end,:,1)),[],'all'),1e-8);
    verifyLessThanOrEqual(testCase,r.TrajectoryDuration_s,r.Inputs.goalState.time_s);

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
