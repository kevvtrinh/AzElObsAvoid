function tests = testFixedTimedVisibility
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testFixedTimedVisibility.m')
% PURPOSE: Verify explicit fixed-arrival timed search and physical validation.
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
    testCase.TestData.Options=struct('GoalTimeMode','fixedArrival','FixedArrivalSearch','timeExpanded');
end

function testPrescribedArrivalAndContinuousMotion(testCase)
    data=testCase.TestData;
    result=planner([],data.Initial,data.Goal,data.Limits,data.Options);
    assertTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.ArrivalTime_s,10,'AbsTol',1e-8);
    verifyEqual(testCase,result.VisibilityGraph.SearchKind,"timeExpandedVisibilityGraph");
    verifyFalse(testCase,result.VisibilityGraph.GraphIsFullyEnumerated);
    verifyLessThan(testCase,size(result.Route_units,1),9);
end

function testNoSilentSpatialFallback(testCase)
    data=testCase.TestData;
    wall=struct('Vertices_units',[2,-3;3,-3;3,3;2,3]);
    result=planner(wall,data.Initial,data.Goal,data.Limits,data.Options);
    verifyFalse(testCase,result.Success);
    verifyEqual(testCase,result.TerminationReason,"noTimedRoute");
    verifyEmpty(testCase,result.time_s);
end

function testUnsupportedBoundaryStateIsExplicit(testCase)
    data=testCase.TestData;
    data.Initial.velocity_units_s=[0.1,0];
    result=planner([],data.Initial,data.Goal,data.Limits,data.Options);
    verifyFalse(testCase,result.Success);
    verifyEqual(testCase,result.TerminationReason,"unsupportedTimedRequest");
    verifyEmpty(testCase,result.time_s);
end

function testInvalidSearchChoice(testCase)
    data=testCase.TestData; data.Options.FixedArrivalSearch='unknown';
    verifyError(testCase,@()planner([],data.Initial,data.Goal,data.Limits,data.Options), ...
        'planner:UnsupportedFixedArrivalSearch');
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
    options.GoalTimeMode='fixedArrival'; options.FixedArrivalSearch='timeExpanded';
    result=planner(obstacleAvoidance.obstacles.combineObstacles(sources), ...
        request.initialState,request.goalState,request.limits,options);
    assertTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.ArrivalTime_s,180,'AbsTol',1e-8);
    verifyEqual(testCase,result.VisibilityGraph.RouteTime_s([1,end]),[0;180]);
    verifyLessThan(testCase,result.MotionLength_units,230);
end
