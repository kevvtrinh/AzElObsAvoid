function tests = testSharedMotionCore
% Shared mathematics must preserve both engines' event laws and evaluations.
tests = functiontests(localfunctions);
end

function setupOnce(~)
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'),fullfile(root,'tests'));
end

function testEvaluatorMatchesIndependentPriorImplementation(testCase)
    rng(391);
    for degree=[3 5 8]
        polynomial=struct();
        polynomial.positionPower_units=randn(7,4,degree+1);
        polynomial.velocityPower_units_s=randn(7,4,degree);
        polynomial.accelerationPower_units_s2=randn(7,4,degree-1);
        polynomial.jerkPower_units_s3=randn(7,4,degree-2);
        polynomial.SegmentDuration_s=0.1+rand(7,1);
        polynomial.SegmentStartTime_s=13+[0;cumsum(polynomial.SegmentDuration_s(1:end-1))];
        times=linspace(12,14+sum(polynomial.SegmentDuration_s),501);
        for explicit=[false true]
            segments=[];
            if explicit,segments=4;end
            [a{1:5}]=evaluatePolynomialReference(polynomial,times,segments);
            [b{1:5}]=motionCore.evaluatePolynomial(polynomial,times,segments);
            verifyEqual(testCase,b,a);
        end
    end
end

function testRestLawPreservesBothTimingPolicies(testCase)
    rng(4902);
    initial=struct('time',3,'position',0,'velocity',0,'acceleration',0);
    terminal=struct('position',1,'velocity',0,'acceleration',0);
    for k=1:24
        values=0.2+5*rand(3,1);
        limits=struct('maximumVelocity',values(1),'maximumAcceleration',values(2),'maximumJerk',values(3));
        minimum=createRestToRestJerkProfileReference(initial,terminal,limits,[]);
        for mode=["earliestArrival","fixed"]
            time=[];
            if mode=="fixed",time=minimum.FinalTime+2;end
            reference=createRestToRestJerkProfileReference(initial,terminal,limits,time);
            start=struct('time',3,'position',[0 0],'velocity',[0 0],'acceleration',[0 0]);
            goal=struct('position',[1 0.5],'velocity',[0 0],'acceleration',[0 0],'maximumTime',minimum.FinalTime+2);
            bounds=struct('maximumVelocity',values(1)*[1 0.5], ...
                'maximumAcceleration',values(2)*[1 0.5],'maximumJerk',values(3)*[1 0.5]);
            actual=ruckigEngine.solve(start,goal,bounds,struct('TimeMode',mode,'SampleTime',0.05));
            verifyTrue(testCase,actual.Success,actual.Message);
            verifyEqual(testCase,actual.FinalTime,reference.FinalTime,'AbsTol',1e-10);
            t=linspace(3,reference.FinalTime,127);
            [~,p,v,a,j]=motionCore.evaluatePolynomial(reference.Polynomial,t);
            [~,p2,v2,a2,j2]=motionCore.evaluatePolynomial(actual.Polynomial,t);
            verifyEqual(testCase,[p2 v2 a2],[p*[1 0.5] v*[1 0.5] a*[1 0.5]],'AbsTol',1e-10);
            % Compare jerk away from event boundaries, where its jump is intentional.
            interior=all(abs(t(:)-reference.Polynomial.SegmentStartTime_s.')>1e-9,2);
            verifyEqual(testCase,j2(interior,:),j(interior)*[1 0.5],'AbsTol',1e-10);
        end
    end
end

function testConstantAccelerationUsesExactSharedIntegration(testCase)
    state=struct('time_s',11,'position_units',[4 -2],'velocity_units_s',[1 0], ...
        'acceleration_units_s2',[0 0]);
    duration=[0.5;0.75;0.25];
    controls=[1 2;-2 1;0 -1];
    polynomial=motionCore.createJerkPolynomial(state,[0;cumsum(duration)],zeros(3,2),controls);
    p=state.position_units;v=state.velocity_units_s;
    for k=1:3
        p=p+v*duration(k)+controls(k,:)*duration(k)^2/2;
        v=v+controls(k,:)*duration(k);
        [~,actualP,actualV,actualA]=motionCore.evaluatePolynomial(polynomial,11+sum(duration(1:k)),k);
        verifyEqual(testCase,actualP,p,'AbsTol',1e-12);
        verifyEqual(testCase,actualV,v,'AbsTol',1e-12);
        verifyEqual(testCase,actualA,controls(k,:));
    end
end

function testRangeCheckerPreservesBothIndependentReferences(testCase)
    rng(88761);
    for k=1:240
        degree=mod(k,10);
        scale=10^(mod(k,13)-6);
        powers=scale*randn(degree+1,1);
        limits=[-scale,scale; -Inf,scale; -scale,Inf; 0,0; -Inf,Inf];
        for j=1:size(limits,1)
            lo=limits(j,1); hi=limits(j,2); tolerance=1e-7*scale;
            expected=certifyPolynomialRangeReference(powers,lo,hi,tolerance);
            actual=motionCore.checkPolynomialRange(powers,lo,hi,tolerance);
            [verdict,minimum,maximum]=motionCore.checkPolynomialRange(powers,lo,hi,tolerance);
            [oldVerdict,oldMinimum,oldMaximum]=checkPolynomialRangeReference(powers,lo,hi,tolerance);
            verifyEqual(testCase,actual,expected);
            verifyEqual(testCase,verdict,expected);
            verifyEqual(testCase,verdict,oldVerdict);
            verifyEqual(testCase,[minimum maximum],[oldMinimum oldMaximum],'AbsTol',64*eps(max(1,max(abs(powers)))));
        end
    end
end
