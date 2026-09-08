function tests = testSharedFeatures
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testSharedFeatures.m')
% PURPOSE: Verify shared scalar limits, target derivatives, wrapping, and time trials.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Behavioral tests and tampering rejection.
% UNITS: Coordinate units and seconds.
tests=functiontests(localfunctions);
end
function setupOnce(testCase)
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'),fullfile(root,'examples'));
end
function testScalarAllocation(testCase)
    initial=struct('time_s',0,'position_units',[0,0]); goal=struct('time_s',12,'position_units',[4,2]);
    scalar=struct('maxVelocity_units_s',2,'maxAcceleration_units_s2',2,'maxJerk_units_s3',4);
    vector=scalar;
    for name=string(fieldnames(vector)).', vector.(name)=repmat(vector.(name)/sqrt(2),1,2); end
    a=planner([],initial,goal,scalar); b=planner([],initial,goal,vector);
    verifyTrue(testCase,a.Success && b.Success);
    verifyEqual(testCase,a.position_units,b.position_units);
    verifyEqual(testCase,a.Limits,b.Limits);
    scalar.maxJerk_units_s3=[4,4];
    verifyError(testCase,@() planner([],initial,goal,scalar),'planTrajectory:MixedLimitModes');
    altered=a; altered.SuppliedLimits.maxVelocity_units_s=3;
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(altered).Passed);
end
function testTargetMatchingAndConflict(testCase)
    target=struct('time_s',[0;12],'position_units',[2,1;4.4,2.2],'InterpolationMethod','linear');
    goal=struct('time_s',12,'targetMotion',target);
    initial=struct('time_s',0,'position_units',[-4,0],'velocity_units_s',[0.1,0]);
    options=struct('MatchTargetVelocity',true,'MatchTargetAcceleration',true);
    limits=struct('maxVelocity_units_s',2,'maxAcceleration_units_s2',2,'maxJerk_units_s3',4);
    r=planner([],initial,goal,limits,options);
    verifyTrue(testCase,r.Success,r.Message);
    verifyEqual(testCase,r.velocity_units_s(end,:),[0.2,0.1],'AbsTol',1e-8);
    changed=r; changed.Inputs.goalState.targetMotion.position_units(1,1)=3;
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(changed).Passed);
    goal.velocity_units_s=[0,0];
    verifyError(testCase,@() planner([],initial,goal,limits,options),'planner:ConflictingTargetDerivative');
end
function testPchipAnalyticDerivatives(testCase)
    target=struct('time_s',[0;4;8;12],'position_units',[0,0;1,0.4;3,1;4,2],'InterpolationMethod','pchip');
    [~,v,a]=obstacleAvoidance.input.targetPositionAtTime(target,6);
    pp=pchip(target.time_s,target.position_units(:,1)); c=pp.coefs(2,:); h=2;
    verifyEqual(testCase,v(1),3*c(1)*h^2+2*c(2)*h+c(3),'AbsTol',1e-12);
    verifyEqual(testCase,a(1),6*c(1)*h+2*c(2),'AbsTol',1e-12);
end
function testPeriodicAxes(testCase)
    initial=struct('time_s',0,'position_units',[179,89],'velocity_units_s',[0.1,0.1]);
    goal=struct('time_s',6,'position_units',[-179,-89],'velocity_units_s',[0.2,0.1]);
    r=planner([],initial,goal,[],struct('WrapX',true,'WrapY',true));
    verifyTrue(testCase,r.Success,r.Message);
    verifyEqual(testCase,r.position_units(end,:),[181,91],'AbsTol',1e-8);
    verifyTrue(testCase,r.Validation.Passed);
    verifyError(testCase,@() planner(struct('Vertices_units',[0,0;1,0;0,1]),initial,goal,[],struct('WrapX',true)), ...
        'planner:UnsupportedPeriodicRequest');
end
function testTemporalMatchingAndBudget(testCase)
    target=struct('time_s',[0;12],'position_units',[2,0;3.2,0],'InterpolationMethod','linear');
    initial=struct('time_s',0,'position_units',[0,0],'velocity_units_s',[0.2,0]);
    goal=struct('time_s',12,'targetMotion',target);
    options=struct('GoalTimeMode','earliestArrival','MatchTargetVelocity',true,'TemporalResolution_s',1,'MaxArrivalTrials',12);
    r=planner([],initial,goal,[],options);
    verifyTrue(testCase,r.Success,r.Message);
    verifyTrue(testCase,r.Validation.Passed);
    verifyFalse(testCase,r.TemporalSearch.GlobalEarliestProven);
    verifyEqual(testCase,r.velocity_units_s(1,:),[0.2,0],'AbsTol',1e-8);
    options.MaxArrivalTrials=1; options.TemporalResolution_s=0.1;
    r=planner([],initial,goal,[],options);
    verifyFalse(testCase,r.Success); verifyEqual(testCase,r.TerminationReason,"arrivalSearchExhausted");
    verifyEqual(testCase,numel(r.TemporalSearch.TrialTime_s),1);
end

function testExplicitTargetDerivativesAndScalarEquivalence(testCase)
    target=struct('time_s',[0;12],'position_units',[2,1;4.4,2.2]);
    goal=struct('time_s',12,'targetMotion',target,'velocity_units_s',[-0.1,0.2],'acceleration_units_s2',[0.03,0]);
    initial=struct('time_s',0,'position_units',[-4,0]);
    limits=struct('maxVelocity_units_s',2,'maxAcceleration_units_s2',2,'maxJerk_units_s3',4);
    a=planner([],initial,goal,limits);
    b=planner([],initial,goal,a.RequestedLimits);
    verifyTrue(testCase,a.Success && b.Success);
    verifyEqual(testCase,a.position_units,b.position_units);
    verifyEqual(testCase,a.velocity_units_s(end,:),goal.velocity_units_s,'AbsTol',1e-8);
    verifyEqual(testCase,a.acceleration_units_s2(end,:),goal.acceleration_units_s2,'AbsTol',1e-8);
end

function testUndefinedLinearTargetCorner(testCase)
    target=struct('time_s',[0;6;12],'position_units',[0,0;1,0;3,0]);
    goal=struct('time_s',6,'targetMotion',target);
    initial=struct('time_s',0,'position_units',[-4,0]);
    verifyError(testCase,@() planner([],initial,goal,[],struct('MatchTargetVelocity',true)), ...
        'planner:UndefinedTargetDerivative');
end

function testDisabledWrapAxisAndPlot(testCase)
    initial=struct('time_s',0,'position_units',[179,0]);
    goal=struct('time_s',6,'position_units',[-179,0]);
    r=planner([],initial,goal,struct('yInterval_units',[-1,1]),struct('WrapX',true));
    verifyTrue(testCase,r.Success,r.Message);
    verifyEqual(testCase,r.Limits.yInterval_units,[-1,1]);
    handles=obstacleAvoidance.plotting.plotTrajectory(r,struct('FigureVisible','off'));
    cleanup=onCleanup(@() close(ancestor(handles.Axes,'figure')));
    verifyEqual(testCase,xlim(handles.Axes),[-180,180]);
    verifyEqual(testCase,ylim(handles.Axes),[-1,1]);
    verifyTrue(testCase,any(isnan(handles.Trajectory.XData)));
end

function testEarliestMovingObstacleWithNonzeroStart(testCase)
    x=[-0.5;0.5;0.5;-0.5];y=[-0.5;-0.5;0.5;0.5];
    obstacle=obstacleAvoidance.obstacles.createObstacle('departing goal',[0;12],{x+4;x+4},{y;y+6},0.1);
    initial=struct('time_s',0,'position_units',[-4,0],'velocity_units_s',[0.2,0]);
    goal=struct('time_s',12,'position_units',[4,0],'velocity_units_s',[0.1,0]);
    r=planner(obstacle,initial,goal,[],struct('GoalTimeMode','earliestArrival','TemporalResolution_s',1,'MaxArrivalTrials',12));
    verifyTrue(testCase,r.Success,r.Message);
    verifyTrue(testCase,r.Validation.Passed);
    verifyTrue(testCase,isfield(r,'TemporalSearch'));
    verifyEqual(testCase,r.velocity_units_s(1,:),[0.2,0],'AbsTol',1e-8);
end

function testExampleForwardsTargetMatching(testCase)
    r=exampleInterceptMovingTargetAtSetTime(struct('PlotOutputs',false,'Verbose',false, ...
        'MatchTargetVelocity',true,'MatchTargetAcceleration',true));
    verifyTrue(testCase,r.Success,r.Message);
    verifyEqual(testCase,r.Intercept.TerminalVelocityPolicy,"matched");
    verifyEqual(testCase,r.Intercept.TerminalAccelerationPolicy,"matched");
    [~,v,a]=obstacleAvoidance.input.targetPositionAtTime(r.Inputs.goalState.targetMotion,r.ArrivalTime_s);
    verifyEqual(testCase,r.velocity_units_s(end,:),v,'AbsTol',1e-8);
    verifyEqual(testCase,r.acceleration_units_s2(end,:),a,'AbsTol',1e-8);
end
