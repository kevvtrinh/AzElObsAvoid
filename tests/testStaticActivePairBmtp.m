function tests = testStaticActivePairBmtp
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testStaticActivePairBmtp.m')
% PURPOSE: Regress collision-driven static BMTP on structurally different concave and multi-obstacle routes.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Independent validation and solver-representation checks.
% UNITS: Coordinate units and seconds.
tests=functiontests(localfunctions);
end

function setupOnce(~)
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'));
end

function testConcaveCavityEscape(testCase)
    angle_rad=linspace(pi/3,5*pi/3,10).';
    vertices_units=[8*cos(angle_rad),8*sin(angle_rad); ...
        4*cos(flipud(angle_rad)),4*sin(flipud(angle_rad))];
    obstacle=obstacleAvoidance.obstacles.createObstacle( ...
        'C cavity',[0;260],vertices_units(:,1),vertices_units(:,2),0.1);
    limits=struct('xInterval_units',[-23,23],'yInterval_units',[-23,23], ...
        'maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[0.8,0.8], ...
        'maxJerk_units_s3',[3,3]);
    result=planner(obstacle,state([0,0],0),state([-13,0],260),limits,options());
    verifyValidatedStaticBmtp(testCase,result);
end

function testSeparatedSlalomBarriers(testCase)
    center_units=[-6,-2.5;0,2.5;6,-2.5];
    obstacleList=cell(3,1);
    for obstacleIndex=1:3
        vertices_units=center_units(obstacleIndex,:)+[-0.7,-3;0.7,-3;0.7,3;-0.7,3];
        obstacleList{obstacleIndex}=obstacleAvoidance.obstacles.createObstacle( ...
            "barrier "+obstacleIndex,[0;110],vertices_units(:,1),vertices_units(:,2),0.1);
    end
    obstacles=obstacleAvoidance.obstacles.combineObstacles(obstacleList{:});
    limits=struct('xInterval_units',[-17,17],'yInterval_units',[-10,10], ...
        'maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[1,1], ...
        'maxJerk_units_s3',[2,2]);
    result=planner(obstacles,state([-13,0],0),state([13,0],110),limits,options());
    verifyValidatedStaticBmtp(testCase,result);
    verifyTrue(testCase,result.SolverDiagnostics.TravelRefinementAccepted);
end

function testAffinePlaneBlockConsolidation(testCase)
    limits=struct('xInterval_units',[-100,100],'yInterval_units',[-100,100]);
    source=plane([1,0;0.5,0.5],[-2,-2]);
    target=plane(0.5*source.Normal,[-1.1,-1.1]);
    reduced=bmtpEngine.removeRedundantPlanes([source,target],limits,0.1,true);
    verifyTrue(testCase,reduced(1).Active);
    verifyFalse(testCase,reduced(2).Active);
end

function testAffinePlaneNeedsOneEndpointWeight(testCase)
    limits=struct('xInterval_units',[-10,10],'yInterval_units',[-10,10]);
    source=plane([1,0;1,0],[0,0]);
    differentEndpointScale=plane([0.5,0;0.75,0],[0,0]);
    reduced=bmtpEngine.removeRedundantPlanes( ...
        [source,differentEndpointScale],limits,0,true);
    verifyTrue(testCase,all([reduced.Active]));
end

function verifyValidatedStaticBmtp(testCase,result)
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.SolverDiagnostics.Identifier,"bmtpStaticDegree8");
    verifyGreaterThan(testCase,result.SolverDiagnostics.TaggedPairCount,0);
    verifyGreaterThanOrEqual(testCase, ...
        result.SolverDiagnostics.TransientPlaneRemovalCount,0);
end

function value=state(position_units,time_s)
    value=struct('time_s',time_s,'position_units',position_units);
end

function value=options()
    value=struct('GoalTimeMode','earliestArrival','SampleTime_s',0.1);
end

function value=plane(normal,offset_units)
    value=struct('Active',true,'Verified',true,'ExitFlag',1, ...
        'Normal',normal,'Offset_units',offset_units,'SignedGap_units',1);
end
