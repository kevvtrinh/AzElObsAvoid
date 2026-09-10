function tests = testTimeScopedPlanes
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testTimeScopedPlanes.m')
% PURPOSE: Verify constraints act on their physical time intervals, and
%          moving detours preserve nonzero physical endpoint states.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Behavioral constraint, collision, and endpoint regressions.
% UNITS: Coordinate units and seconds.
tests = functiontests(localfunctions);
end

function setupOnce(~)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'));
end

function testDifferentTimeWindowsDoNotConflict(testCase)
    initial = struct('time_s',0,'position_units',[-1,0],'velocity_units_s',[0,0],'acceleration_units_s2',[0,0]);
    goal = initial; goal.time_s = 6; goal.position_units = [1,0];
    limits = struct('xInterval_units',[-2,2],'yInterval_units',[-2,2], ...
        'maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[5,5],'maxJerk_units_s3',[10,10]);
    planes = repmat(struct('Active',true,'Normal',[1,0;1,0],'Offset_units',[0.5,0.5], ...
        'TimeFraction',[0,0.1]),1,2);
    planes(2).Normal = -planes(1).Normal; planes(2).TimeFraction = [0.9,1];
    options = optimoptions('coneprog','Display','none','ConstraintTolerance',1e-10,'OptimalityTolerance',1e-9);
    [controls,durations,flag,output] = bmtpEngine.solveTrajectoryStep(1,5,initial,goal,limits,planes,1e-8,6,options,1,true);
    verifyTrue(testCase,flag>0 || flag==-7);
    verifyEqual(testCase,durations,6);
    verifyLessThan(testCase,output.MaximumClearanceSlack_units,1e-6);
    first = bmtpEngine.restrictBezier(squeeze(controls),[0,0.1]);
    last = bmtpEngine.restrictBezier(squeeze(controls),[0.9,1]);
    verifyLessThan(testCase,max(first(:,1)),-0.5);
    verifyGreaterThan(testCase,min(last(:,1)),0.5);
    % The same two exclusions overlap in time and are genuinely incompatible.
    planes(1).TimeFraction = [0,1]; planes(2).TimeFraction = [0,1];
    [~,~,flag,output] = bmtpEngine.solveTrajectoryStep(1,5,initial,goal,limits,planes,1e-8,6,options,1,true);
    verifyTrue(testCase,flag<=0 || output.MaximumClearanceSlack_units>0.4);
end

function testMovingDetourWithNonzeroEndpointVelocity(testCase)
    times_s = (0:0.25:20)';
    box = [-0.5,-0.7;0.5,-0.7;0.5,0.7;-0.5,0.7];
    source = obstacleAvoidance.obstacles.createObstacle('moving rectangle',times_s, ...
        repmat({box(:,1)},numel(times_s),1), ...
        arrayfun(@(t)box(:,2)-2+0.2*t,times_s,'UniformOutput',false),0.1);
    initial = struct('time_s',0,'position_units',[-4,0],'velocity_units_s',[0.15,0]);
    goal = struct('time_s',20,'position_units',[4,0],'velocity_units_s',[0.1,0]);
    result = planner(source,initial,goal,[],struct('GoalTimeMode','fixedArrival'));
    assertTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.ArrivalTime_s,20,'AbsTol',1e-8);
    verifyEqual(testCase,result.velocity_units_s([1,end],:),[0.15,0;0.1,0],'AbsTol',1e-8);
    verifyLessThan(testCase,result.Polynomial.SegmentCount,80);
    verifyEqual(testCase,result.PlaneCertificate.SolverRegionCount,80);
    verifyGreaterThan(testCase,result.SolverDiagnostics.TrajectorySocpCount,0);
end
