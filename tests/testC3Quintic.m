function tests = testC3Quintic
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testC3Quintic.m')
% PURPOSE: Require quintic spans and independently reject jerk discontinuities.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Behavioral continuity and endpoint checks.
% UNITS: Coordinate units and seconds.
tests=functiontests(localfunctions);
end
function setupOnce(testCase)
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'));
end
function testQuinticDirectAndDetour(testCase)
    initial=struct('time_s',0,'position_units',[-4,0]);
    goal=struct('time_s',12,'position_units',[4,0]);
    motions={planner([],initial,goal),planner()};
    for k=1:numel(motions)
        r=motions{k}; verifyTrue(testCase,r.Success,r.Message);
        verifyEqual(testCase,r.Polynomial.Degree,5);
        verifyEqual(testCase,size(r.Polynomial.positionPower_units,3),6);
        p=r.Polynomial.jerkPower_units_s3;
        verifyEqual(testCase,sum(p(1:end-1,:,:),3),p(2:end,:,1),'AbsTol',1e-8);
        verifyTrue(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
    end
end
function testJerkJumpWithC2AndMatchedEndpointsRejected(testCase)
    r=planner([],struct('time_s',0,'position_units',[0,0]),struct('time_s',12,'position_units',[4,0]));
    p=r.Polynomial;
    p.positionPower_units(1,1,:)=p.positionPower_units(1,1,:)+reshape(1e-4*[0,0,0,10,-15,6],1,1,6);
    p.positionPower_units(2,1,:)=p.positionPower_units(2,1,:)+reshape(1e-4*[1,0,0,-10,15,-6],1,1,6);
    p.velocityPower_units_s=p.positionPower_units(:,:,2:end).*reshape(1:5,1,1,[])./p.SegmentDuration_s;
    p.accelerationPower_units_s2=p.velocityPower_units_s(:,:,2:end).*reshape(1:4,1,1,[])./p.SegmentDuration_s;
    p.jerkPower_units_s3=p.accelerationPower_units_s2(:,:,2:end).*reshape(1:3,1,1,[])./p.SegmentDuration_s;
    arrays={p.positionPower_units,p.velocityPower_units_s,p.accelerationPower_units_s2};
    for k=1:3, verifyEqual(testCase,sum(arrays{k}(1,:,:),3),arrays{k}(2,:,1),'AbsTol',1e-8); end
    r.Polynomial=p;
    [~,r.position_units,r.velocity_units_s,r.acceleration_units_s2,r.jerk_units_s3]=bmtpEngine.evaluatePolynomial(p,r.time_s);
    checked=obstacleAvoidance.validateTrajectory(r);
    verifyTrue(testCase,checked.EndpointStatesMatched);
    verifyTrue(testCase,checked.DynamicsConsistent);
    verifyTrue(testCase,checked.SampledHistoriesMatched);
    verifyFalse(testCase,checked.InterSegmentContinuous);
    verifyFalse(testCase,checked.Passed);
end

function testWrongOrMissingDegreeRejected(testCase)
    r=planner([],struct('time_s',0,'position_units',[0,0]),struct('time_s',12,'position_units',[4,0]));
    r.Polynomial.Degree=8;
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
    r.Polynomial=rmfield(r.Polynomial,'Degree');
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
end

function testSmoothedWaitingChordPreservesStatesAndBounds(testCase)
    displacements=[0.1,0;4,2;-3,6];
    limits=struct('maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[1,1],'maxJerk_units_s3',[2,2]);
    for k=1:size(displacements,1)
        [controls,times,powers]=bmtpEngine.createC3Chord([7,-3],[7,-3]+displacements(k,:),limits);
        p=bmtpEngine.createPowerPolynomial(controls,times,0,powers);
        verifyEqual(testCase,p.TerminalState.position_units,[7,-3]+displacements(k,:),'AbsTol',1e-8);
        verifyEqual(testCase,p.TerminalState.velocity_units_s,[0,0],'AbsTol',1e-8);
        verifyEqual(testCase,p.TerminalState.acceleration_units_s2,[0,0],'AbsTol',1e-8);
        verifyEqual(testCase,p.jerkPower_units_s3(1,:,1),[0,0],'AbsTol',1e-8);
        verifyEqual(testCase,sum(p.jerkPower_units_s3(end,:,:),3),[0,0],'AbsTol',1e-8);
        arrays={p.velocityPower_units_s,p.accelerationPower_units_s2,p.jerkPower_units_s3};
        bounds={limits.maxVelocity_units_s,limits.maxAcceleration_units_s2,limits.maxJerk_units_s3};
        for order=1:3
            a=arrays{order};
            verifyEqual(testCase,sum(a(1:end-1,:,:),3),a(2:end,:,1),'AbsTol',1e-8);
            for span=1:numel(times)
                for axis=1:2
                    verifyTrue(testCase,obstacleAvoidance.validation.certifyPolynomialRange( ...
                        reshape(a(span,axis,:),[],1),-bounds{order}(axis),bounds{order}(axis),1e-8));
                end
            end
        end
    end
end
