function tests = testQuinticNonrestDetour
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testQuinticNonrestDetour.m')
% PURPOSE: Exercise local quintic states on a detour with nonzero boundary jets.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Independent endpoint, C3, collision, and physical-limit checks.
% UNITS: Coordinate units and seconds.
tests=functiontests(localfunctions);
end

function testRotatedAnisotropicDetourAndAxisSwap(testCase)
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'));
    angle_rad=0.23;
    rotation=[cos(angle_rad),-sin(angle_rad);sin(angle_rad),cos(angle_rad)];
    shift_units=[4,-3];
    vertices_units=[-1.5,-2;1.5,-2;1.5,2;-1.5,2]*rotation.'+shift_units;
    initial=struct('time_s',3.7,'position_units',[-6,-0.6]*rotation.'+shift_units, ...
        'velocity_units_s',[0.15,-0.05],'acceleration_units_s2',[0.01,0.02]);
    goal=struct('time_s',123.7,'position_units',[6,0.8]*rotation.'+shift_units, ...
        'velocity_units_s',[0.1,0.05],'acceleration_units_s2',[-0.02,0.01]);
    limits=struct('maxVelocity_units_s',[1.9,2.1], ...
        'maxAcceleration_units_s2',[0.72,0.83],'maxJerk_units_s3',[2.4,2.8]);
    for variant=1:2
        if variant==2
            vertices_units=vertices_units(:,[2,1]);
            for name=["position_units","velocity_units_s","acceleration_units_s2"]
                initial.(name)=initial.(name)([2,1]); goal.(name)=goal.(name)([2,1]);
            end
            for name=string(fieldnames(limits)).', limits.(name)=limits.(name)([2,1]); end
        end
        obstacle=struct('Vertices_units',vertices_units,'SafetyMargin_units',0.15);
        result=planner(obstacle,initial,goal,limits,struct('GoalTimeMode','earliestArrival'));
        verifyTrue(testCase,result.Success,result.Message);
        verifyEqual(testCase,result.SolverDiagnostics.Identifier,"quinticJerkClock");
        validation=obstacleAvoidance.validateTrajectory(result);
        verifyTrue(testCase,validation.Passed,validation.Message);
        verifyTrue(testCase,validation.EndpointStatesMatched);
        verifyTrue(testCase,validation.InterSegmentContinuous);
        verifyLessThan(testCase,result.ArrivalTime_s,goal.time_s);
        verifyEqual(testCase,result.Polynomial.Degree,5);
    end
end
