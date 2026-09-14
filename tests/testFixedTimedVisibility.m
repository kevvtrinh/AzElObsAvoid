function tests = testFixedTimedVisibility
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testFixedTimedVisibility.m')
% PURPOSE: Verify unified fixed-arrival seed selection and physical validation.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Fixed-clock motion, unsupported request, and expected failure checks.
% UNITS: Coordinate units and seconds.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'),fullfile(root,'examples'));
    testCase.TestData.Initial=struct('time_s',0,'position_units',[0,0]);
    testCase.TestData.Goal=struct('time_s',10,'position_units',[5,0]);
    testCase.TestData.Limits=struct('xInterval_units',[-1,6],'yInterval_units',[-2,2], ...
        'maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[2,2],'maxJerk_units_s3',[4,4]);
    testCase.TestData.Options=struct('GoalTimeMode','fixedArrival');
end

function testPrescribedArrivalAndContinuousMotion(testCase)
    data=testCase.TestData;
    result=planner([],data.Initial,data.Goal,data.Limits,data.Options);
    assertTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.ArrivalTime_s,10,'AbsTol',1e-8);
    verifyEqual(testCase,result.VisibilityGraph.SearchKind,"initialSpatialSnapshot");
    verifyTrue(testCase,result.VisibilityGraph.GraphIsFullyEnumerated);
    verifyLessThan(testCase,size(result.Route_units,1),9);
end

function testNoSilentSpatialFallback(testCase)
    data=testCase.TestData;
    wall=struct('Vertices_units',[2,-3;3,-3;3,3;2,3]);
    result=planner(wall,data.Initial,data.Goal,data.Limits,data.Options);
    verifyFalse(testCase,result.Success);
    verifyEqual(testCase,result.TerminationReason,"noVisibilityRoute");
    verifyEmpty(testCase,result.time_s);
end

function testFixedTimedSearchRetainsBoundaryVelocity(testCase)
    data=testCase.TestData;
    data.Initial.velocity_units_s=[0.1,0];
    result=planner([],data.Initial,data.Goal,data.Limits,data.Options);
    assertTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,result.Validation.Passed);
    verifyEqual(testCase,result.velocity_units_s([1,end],:),[0.1,0;0,0], ...
        'AbsTol',1e-8);
end

function testSavedDetourUsesPrescribedDeadline(testCase)
    root=fileparts(mfilename('fullpath'));
    request=jsondecode(fileread(fullfile(root,'fixtures','savedMovingDetour.json')));
    sources=cell(numel(request.obstacles),1);
    for k=1:numel(sources)
        source=request.obstacles(k); frames=source.keyframes;
        sources{k}=obstacleAvoidance.obstacles.createObstacle(source.name,[frames.time_s].', ...
            arrayfun(@(f)f.vertices_units(:,1),frames,'UniformOutput',false), ...
            arrayfun(@(f)f.vertices_units(:,2),frames,'UniformOutput',false),source.safetyMargin_units);
    end
    options=request.options;
    options.GoalTimeMode='fixedArrival';
    result=planner(obstacleAvoidance.obstacles.combineObstacles(sources), ...
        request.initialState,request.goalState,request.limits,options);
    assertTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.ArrivalTime_s,180,'AbsTol',1e-8);
    % The unified spatial result stays within one percent of the former
    % explicitly selected timed motion (229.400575729 units).
    verifyLessThan(testCase,result.MotionLength_units,232);
    verifyEqual(testCase,result.SeedSource,"initialSpatialSnapshot");
    verifyGreaterThan(testCase,result.SolverDiagnostics.OptimizerSpanCount,16);
end

function testMovingCrossingRetainsCertifiedEndpointJerk(testCase)
    missionEndTime_s=12;
    time_s=linspace(0,missionEndTime_s,5).';
    halfSize_units=[0.73318128921311621,0.95945964014883089];
    initialAngle_rad=-0.46430464622215828;
    safetyMargin_units=0.14507508417445217;
    centerX_units=1.7787935948757139;
    center_units=[repmat(centerX_units,5,1),linspace(-3.8,3.8,5).'];
    angle_rad=initialAngle_rad+linspace(0,pi/3,5).';
    local_units=[-1,-1;1,-1;1,1;-1,1].*halfSize_units;
    xByTime_units=cell(5,1); yByTime_units=cell(5,1);
    for sampleIndex=1:5
        rotation=[cos(angle_rad(sampleIndex)),-sin(angle_rad(sampleIndex)); ...
            sin(angle_rad(sampleIndex)),cos(angle_rad(sampleIndex))];
        boundary_units=local_units*rotation.'+center_units(sampleIndex,:);
        xByTime_units{sampleIndex}=boundary_units(:,1);
        yByTime_units{sampleIndex}=boundary_units(:,2);
    end
    obstacle=obstacleAvoidance.obstacles.createObstacle('moving crossing regression', ...
        time_s,xByTime_units,yByTime_units,safetyMargin_units);
    initial=struct('time_s',0,'position_units',[-6,0], ...
        'velocity_units_s',[0,0],'acceleration_units_s2',[0,0]);
    goal=struct('time_s',missionEndTime_s,'position_units',[6,0], ...
        'velocity_units_s',[0,0],'acceleration_units_s2',[0,0]);
    limits=struct('xInterval_units',[-9,9],'yInterval_units',[-6,6], ...
        'maxVelocity_units_s',[3,3],'maxAcceleration_units_s2',[2,2], ...
        'maxJerk_units_s3',[4,4]);
    options=struct('GoalTimeMode','fixedArrival', ...
        'SampleTime_s',0.05, ...
        'TemporalResolution_s',0.75);
    result=planner(obstacle,initial,goal,limits,options);
    assertTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.ArrivalTime_s,missionEndTime_s,'AbsTol',1e-8);
    verifyLessThanOrEqual(testCase,max(abs(result.jerk_units_s3),[],1), ...
        limits.maxJerk_units_s3+result.Options.ConstraintTolerance);
end

function testArrivalSnapshotAvoidsTimedFallback(testCase)
    scenario=createRandomAzimuthScenario(26,true);
    result=planner(scenario.Obstacles,scenario.InitialState, ...
        scenario.GoalState,scenario.Limits,scenario.Options);
    assertTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.SeedSource,"arrivalSpatialSnapshot");
    verifyEqual(testCase,result.VisibilityGraph.SearchKind, ...
        "arrivalSpatialSnapshot");
    verifyFalse(testCase,isfield(result.VisibilityGraph,'SpatialSeedDiagnostics'));
    verifyTrue(testCase,isfield(result.VisibilityGraph,'InitialSpatialSeedDiagnostics'));
    verifyFalse(testCase,result.VisibilityGraph.InitialSpatialSeedDiagnostics.Accepted);
    verifyEqual(testCase, ...
        result.VisibilityGraph.InitialSpatialSeedDiagnostics.IterationCount,2);
    verifyGreaterThan(testCase,size(result.Route_units,1),2);
end
