function tests = testC3ProfileLibrary
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testC3ProfileLibrary.m')
% PURPOSE: Check portable profiles, current-scene repair, strict validation and recovery.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Behavioral checks on unseen geometry and malformed library inputs.
% UNITS: Coordinate units and seconds.
tests=functiontests(localfunctions);
end

function setupOnce(testCase)
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'),fullfile(root,'examples'));
    [obstacles,initial,goal,limits]=scene([1.1,1.05],0.4);
    training=planner(obstacles,initial,goal,limits,struct('GoalTimeMode','earliestArrival'));
    assert(training.Success,training.Message);
    testCase.TestData.Training=training;
    testCase.TestData.Library=bmtpEngine.buildC3ProfileLibrary({training});
    [obstacles,initial,goal,limits]=scene([1,1],0);
    testCase.TestData.Inputs={obstacles,initial,goal,limits};
    testCase.TestData.Reference=planner(obstacles,initial,goal,limits,struct('GoalTimeMode','earliestArrival'));
    assert(testCase.TestData.Reference.Success);
end

function testNormalizedDataAndFileRoundTrip(testCase)
    library=testCase.TestData.Library;
    entry=library.Entries{1};
    verifyEqual(testCase,entry.Route([1,end],:),[0,0;1,0],'AbsTol',1e-12);
    verifyEqual(testCase,sum(entry.SegmentTime),1,'AbsTol',1e-12);
    verifyEqual(testCase,sum(entry.PhaseTime),1,'AbsTol',1e-12);
    verifyEqual(testCase,sum(entry.CompactPhaseTime),1,'AbsTol',1e-12);
    verifyLessThan(testCase,numel(entry.CompactPhaseTime),numel(entry.PhaseTime));
    verifyEqual(testCase,numel(entry.PhaseTime),testCase.TestData.Training.SolverDiagnostics.OptimizerSpanCount);
    filename=[tempname,'.mat'];
    cleanup=onCleanup(@() delete(filename)); %#ok<NASGU>
    saved=bmtpEngine.buildC3ProfileLibrary({testCase.TestData.Training},filename);
    verifyEqual(testCase,bmtpEngine.loadC3ProfileLibrary(filename),saved);
end

function testRepairOnUnseenDetour(testCase)
    options=profileOptions(testCase);
    options.C3ProfileMaxArrival_s=testCase.TestData.Reference.ArrivalTime_s+0.49;
    result=planner(testCase.TestData.Inputs{:},options);
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyTrue(testCase,result.SolverDiagnostics.ProfileLibrary.Accepted);
    verifyFalse(testCase,result.SolverDiagnostics.ProfileLibrary.FallbackUsed);
    verifyLessThanOrEqual(testCase,result.ArrivalTime_s,options.C3ProfileMaxArrival_s);
    verifyLessThan(testCase,result.MotionLength_units,testCase.TestData.Reference.MotionLength_units);
    verifyGreaterThan(testCase,result.SolverDiagnostics.NonlinearSolver.funcCount,0);
    verifyLessThanOrEqual(testCase,result.SolverDiagnostics.NonlinearSolver.iterations,40);
    verifyLessThan(testCase,result.SolverDiagnostics.OptimizerSpanCount,testCase.TestData.Reference.SolverDiagnostics.OptimizerSpanCount);
    verifyLessThan(testCase,sum(totalJerkVariation(result)),sum(totalJerkVariation(testCase.TestData.Reference)));
end

function testArrivalCapRetriesOrdinarySolver(testCase)
    options=profileOptions(testCase);
    options.C3ProfileMaxArrival_s=testCase.TestData.Reference.ArrivalTime_s-1;
    result=planner(testCase.TestData.Inputs{:},options);
    record=result.SolverDiagnostics.ProfileLibrary;
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,result.Validation.Passed);
    verifyEqual(testCase,record.AttemptReason,"profileArrivalCapExceeded");
    verifyTrue(testCase,record.FallbackUsed);
    verifyFalse(testCase,record.Accepted);
    verifyGreaterThan(testCase,record.FallbackTime_s,0);
    verifyEqual(testCase,result.ArrivalTime_s,testCase.TestData.Reference.ArrivalTime_s,'AbsTol',1e-6);
end

function testWarmStartRetainsTimeOptimization(testCase)
    options=profileOptions(testCase);
    options.C3ProfileMode='warmStart';
    result=planner(testCase.TestData.Inputs{:},options);
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,result.Validation.Passed);
    verifyTrue(testCase,result.SolverDiagnostics.ProfileLibrary.Attempted);
    verifyGreaterThan(testCase,result.SolverDiagnostics.NonlinearSolver.funcCount,0);
end

function testRepairWithExplicitTimeAllowances(testCase)
    for allowance_s=[0,0.3]
        options=profileOptions(testCase);
        options.PathLengthTimeAllowance_s=allowance_s;
        result=planner(testCase.TestData.Inputs{:},options);
        verifyTrue(testCase,result.Success,result.Message);
        verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
        verifyTrue(testCase,result.SolverDiagnostics.ProfileLibrary.Accepted);
        verifyGreaterThanOrEqual(testCase,result.SolverDiagnostics.ProfileExtraTime_s,0);
        verifyLessThanOrEqual(testCase,result.SolverDiagnostics.ProfileExtraTime_s,allowance_s);
        usedAllowance_s=result.SolverDiagnostics.ProfileExtraTime_s+ ...
            result.SolverDiagnostics.NonlinearSolver.JerkVariationPenalty_s;
        verifyLessThanOrEqual(testCase,usedAllowance_s,allowance_s+1e-8);
    end
end

function testLengthCapRetriesOrdinarySolver(testCase)
    options=profileOptions(testCase);
    options.C3ProfileMaxLength_units=1;
    result=planner(testCase.TestData.Inputs{:},options);
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,result.Validation.Passed);
    verifyEqual(testCase,result.SolverDiagnostics.ProfileLibrary.AttemptReason,"profileLengthCapExceeded");
    verifyTrue(testCase,result.SolverDiagnostics.ProfileLibrary.FallbackUsed);
    verifyFalse(testCase,result.SolverDiagnostics.ProfileLibrary.Accepted);
    verifyEqual(testCase,result.MotionLength_units,testCase.TestData.Reference.MotionLength_units,'AbsTol',1e-6);
end

function testScaledTranslatedSwappedAxes(testCase)
    scale=1.25; shift=[13,-8];
    [~,~,~,limits,vertices]=scene([1,1],0);
    vertices=scale*vertices(:,[2,1])+shift;
    obstacles=obstacleAvoidance.obstacles.createObstacle('transformed',[0;120],vertices(:,1),vertices(:,2),scale*0.2);
    initial=struct('time_s',0,'position_units',shift);
    goal=struct('time_s',120,'position_units',scale*[-10,0]+shift);
    limits.maxVelocity_units_s=scale*limits.maxVelocity_units_s;
    limits.maxAcceleration_units_s2=scale*limits.maxAcceleration_units_s2;
    limits.maxJerk_units_s3=scale*limits.maxJerk_units_s3;
    result=planner(obstacles,initial,goal,limits,profileOptions(testCase));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyTrue(testCase,result.SolverDiagnostics.ProfileLibrary.Matched);
    verifyTrue(testCase,result.SolverDiagnostics.ProfileLibrary.Attempted);
end

function testEmptyLibraryMissPreservesOrdinaryMotion(testCase)
    options=profileOptions(testCase);
    options.C3ProfileLibrary.Entries={};
    result=planner(testCase.TestData.Inputs{:},options);
    verifyTrue(testCase,result.Success,result.Message);
    verifyFalse(testCase,result.SolverDiagnostics.ProfileLibrary.Matched);
    verifyTrue(testCase,result.SolverDiagnostics.ProfileLibrary.FallbackUsed);
    verifyEqual(testCase,result.ArrivalTime_s,testCase.TestData.Reference.ArrivalTime_s,'AbsTol',1e-6);
end

function testDirectAndFixedArrivalUseExistingSolver(testCase)
    options=profileOptions(testCase);
    result=planner([],struct('position_units',[0,0],'time_s',0),struct('position_units',[4,0],'time_s',12),[],options);
    verifyTrue(testCase,result.Success,result.Message);
    verifyEqual(testCase,result.SolverDiagnostics.ProfileLibrary.Reason,"ordinarySolverPreferred");
    options.GoalTimeMode='fixedArrival';
    result=planner([],struct('position_units',[0,0],'time_s',0),struct('position_units',[4,0],'time_s',12),[],options);
    verifyTrue(testCase,result.Success,result.Message);
    verifyEqual(testCase,result.ArrivalTime_s,12,'AbsTol',1e-8);
    verifyFalse(testCase,result.SolverDiagnostics.ProfileLibrary.Attempted);
end

function testRejectMalformedLibrary(testCase)
    library=testCase.TestData.Library;
    bad=library; bad.SchemaVersion=99;
    verifyError(testCase,@() bmtpEngine.loadC3ProfileLibrary(bad),'bmtpEngine:InvalidProfileLibrary');
    bad=library; bad.Entries{1}.SegmentTime(1)=-1;
    verifyError(testCase,@() bmtpEngine.loadC3ProfileLibrary(bad),'bmtpEngine:InvalidProfileLibrary');
    bad=library; bad.Entries{1}.PositionPower(2,1,1)=bad.Entries{1}.PositionPower(2,1,1)+0.01;
    verifyError(testCase,@() bmtpEngine.loadC3ProfileLibrary(bad),'bmtpEngine:InvalidProfileLibrary');
    bad=library; bad.Entries{1}.LimitSignature(1)=NaN;
    verifyError(testCase,@() bmtpEngine.loadC3ProfileLibrary(bad),'bmtpEngine:InvalidProfileLibrary');
end

function testRejectUnvalidatedTraining(testCase)
    bad=testCase.TestData.Training;
    bad.Success=false;
    verifyError(testCase,@() bmtpEngine.buildC3ProfileLibrary({bad}),'bmtpEngine:InvalidProfileTraining');
    bad=testCase.TestData.Training;
    bad.Polynomial.positionPower_units(2,1,1)=bad.Polynomial.positionPower_units(2,1,1)+1;
    verifyError(testCase,@() bmtpEngine.buildC3ProfileLibrary({bad}),'bmtpEngine:InvalidProfileTraining');
end

function testRejectInvalidOptions(testCase)
    options=profileOptions(testCase); options.C3ProfileMode='unchecked';
    verifyError(testCase,@() planner(testCase.TestData.Inputs{:},options),'planner:InvalidProfileMode');
end

function options=profileOptions(testCase)
    options=struct('GoalTimeMode','earliestArrival','C3ProfileLibrary',testCase.TestData.Library,'C3ProfileMode','repair');
end

function [obstacles,initial,goal,limits,vertices]=scene(scales,depth)
    vertices=[-8,7;-5,7;-5,-4;5,-4;5,7;8,7;8,-7;-8,-7].*scales;
    obstacles=obstacleAvoidance.obstacles.createObstacle('profile fixture',[0;120],vertices(:,1),vertices(:,2),0.2);
    initial=struct('time_s',0,'position_units',[depth,depth/2]);
    goal=struct('time_s',120,'position_units',[0,-10*scales(2)]);
    limits=struct('maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[0.75,0.75],'maxJerk_units_s3',[2.5,2.5]);
end
