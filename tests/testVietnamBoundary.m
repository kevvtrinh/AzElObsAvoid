function tests = testVietnamBoundary
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testVietnamBoundary.m')
% PURPOSE: Preserve the extracted boundary and its declared interpolation model.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Source fidelity and interpolated-history regression results.
% UNITS: Degrees and seconds.
tests = functiontests(localfunctions);
end

function setupOnce(~)
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'examples'),fullfile(root,'trajectory'));
end

function testSourceAndInterpolation(testCase)
    [obstacle,initial,goal,limits]=createVietnamBoundaryScenario();
    verifyEqual(testCase,obstacle.time_s,(2770:0.25:3000).');
    verifyEqual(testCase,cellfun(@numel,obstacle.x_units),275*ones(921,1));
    verifyEqual(testCase,cellfun(@numel,obstacle.y_units),275*ones(921,1));
    verifyEqual(testCase,obstacle.x_units{1}([1,end]),[71.2030949368775;71.1630137217925]);
    verifyEqual(testCase,obstacle.y_units{end}([1,end]),[44.5387196138512;44.6152394585970]);
    verifyEqual(testCase,obstacle.x_units,obstacle.originalX_units);
    verifyEqual(testCase,obstacle.y_units,obstacle.originalY_units);
    for index=[2,281,560,562,741,920]
        if index<=561, endpoints=[1,561]; else, endpoints=[561,921]; end
        fraction=(index-endpoints(1))/diff(endpoints);
        for coordinate=["x_units","y_units"]
            values=obstacle.(coordinate);
            expected=(1-fraction)*values{endpoints(1)}+fraction*values{endpoints(2)};
            verifyEqual(testCase,values{index},expected,'AbsTol',3e-14);
        end
    end
    verifyEqual(testCase,initial.position_units,[80,0]);
    verifyEqual(testCase,goal.position_units,[0,80]);
    verifyEqual(testCase,goal.time_s-initial.time_s,230);
    verifyEqual(testCase,limits.maxVelocity_units_s,[2,2]);
end

function testPreparationRetainsSourceAndCanonicalizesTwoSpans(testCase)
    [obstacle,initial,goal,~]=createVietnamBoundaryScenario();
    prepared=obstacleAvoidance.obstacles.prepareObstacles(obstacle,[initial.time_s,goal.time_s]);
    preparation=prepared.InternalPreparation;
    verifyEqual(testCase,preparation.MergedSpanTime_s,[2770,2910;2910,3000]);
    verifyEqual(testCase,preparation.MergedIntervalCount,918);
    verifyTrue(testCase,all(preparation.IntervalPrepared));
    verifyEqual(testCase,prepared.x_units,obstacle.x_units);
    verifyEqual(testCase,prepared.y_units,obstacle.y_units);
    verifyEqual(testCase,obstacle.NormalizationDiagnostics.RemovedZigzagVertexCountBySample,5*ones(921,2));
end

function testExactDeformationIsNeverReplacedByAConvexHull(testCase)
    [obstacle,initial,goal,limits]=createVietnamBoundaryScenario();
    prepared=obstacleAvoidance.obstacles.prepareObstacles(obstacle,[initial.time_s,goal.time_s]);
    models=prepared.InternalPreparation.IntervalGeometryModel;
    verifyEqual(testCase,models,repmat("linearCorrespondingConvexPartition",920,1));
    verifyFalse(testCase,any(contains(models,"ConvexHull",'IgnoreCase',true)));
    cells=obstacleAvoidance.obstacles.createTimeCells(prepared,2770,3000);
    verifyEqual(testCase,unique(cells.ActiveTimeInterval_s,'rows'),[2770,2910;2910,3000]);
    result=planner(prepared,initial,goal,limits,struct('GoalTimeMode','fixedArrival','WrapX',false));
    verifyTrue(testCase,result.Success,result.Message);
    verifyEqual(testCase,result.ArrivalTime_s,3000);
    verifyLessThan(testCase,abs(result.MotionLength_units/113.153-1),0.01);
    validation=obstacleAvoidance.validateTrajectory(result);
    verifyTrue(testCase,validation.Passed,validation.Message);
end
