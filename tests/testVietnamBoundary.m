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
    verifyEqual(testCase,cellfun(@numel,obstacle.x_units),280*ones(921,1));
    verifyEqual(testCase,cellfun(@numel,obstacle.y_units),280*ones(921,1));
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

function testCompleteMotionCarriesVelocityAcrossJoins(testCase)
    result=exampleVietnamBoundarySlew(struct('PlotOutputs',false,'Verbose',false));
    verifyTrue(testCase,result.Success);
    validation=obstacleAvoidance.validateTrajectory(result);
    verifyTrue(testCase,validation.Passed);
    verifyEqual(testCase,result.ArrivalTime_s,3000,'AbsTol',1e-8);
    verifyLessThan(testCase,result.MotionLength_units,113.147);
    verifyTrue(testCase,result.VisibilityGraph.GraphIsFullyEnumerated);
    polynomial=result.Polynomial;
    verifyGreaterThan(testCase,polynomial.SegmentCount,1);
    speed_deg_s=vecnorm(polynomial.velocityPower_units_s(2:end,:,1),2,2);
    verifyGreaterThan(testCase,min(speed_deg_s),1e-6);
    for field=["positionPower_units","velocityPower_units_s", ...
            "accelerationPower_units_s2","jerkPower_units_s3"]
        values=polynomial.(field);
        verifyEqual(testCase,sum(values(1:end-1,:,:),3), ...
            values(2:end,:,1),'AbsTol',result.Options.ConstraintTolerance);
    end
    models=result.PreparedObstacles.InternalPreparation.IntervalGeometryModel;
    verifyEqual(testCase,models,repmat("conservativeEndpointConvexHull",920,1));
end
