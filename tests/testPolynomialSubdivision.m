function tests = testPolynomialSubdivision
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testPolynomialSubdivision.m')
% PURPOSE: Check that certificate subdivision preserves the exported motion
%   after endpoint/continuity repair, including unequal spans and nonmidpoint cuts.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Independent validation and physical p/v/a/jerk preservation checks.
% UNITS: Coordinate units and seconds.
tests=functiontests(localfunctions);
end

function testPreparedCurveSurvivesSelectiveSubdivision(testCase)
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'));
    initial=struct('position_units',[-2,1],'time_s',11);
    goal=struct('position_units',[2,-1],'time_s',15);
    limits=struct('xInterval_units',[-5,5],'yInterval_units',[-5,5], ...
        'maxVelocity_units_s',[10,10],'maxAcceleration_units_s2',[20,20], ...
        'maxJerk_units_s3',[100,100]);
    base=planner([],initial,goal,limits,struct('GoalTimeMode','fixedArrival'));
    seed=struct('position_units',[initial.position_units;goal.position_units], ...
        'tau',[0;1],'Source',"visibilityGraph");
    request=bmtpEngine.createSolveRequest(seed,cell(0,1),struct('Passed',true), ...
        base.Inputs.initialState,base.Inputs.goalState,base.Limits,base.Options);
    durations_s=[1.3;0.7;2]; breaks=[0;cumsum(durations_s)]/4;
    [~,~,reserve_units]=bmtpEngine.createCoordinateTolerances(base.Route_units, ...
        limits.xInterval_units,limits.yInterval_units);
    target_units=(1+2^20*eps)*request.Options.CollisionClearanceTolerance_units+reserve_units;
    for degree=[5,8]
        fraction=zeros(degree+1,1); coefficients=[10,-15,6];
        for k=0:degree
            for j=3:min(k,5)
                fraction(k+1)=fraction(k+1)+coefficients(j-2)*nchoosek(k,j)/nchoosek(degree,j);
            end
        end
        whole=initial.position_units+fraction.*(goal.position_units-initial.position_units);
        controls_units=zeros(3,degree+1,2);
        for span=1:3
            controls_units(span,:,:)=bmtpEngine.restrictBezier(whole,breaks(span:span+1).');
        end
        % The quintic export repairs this residual. Later certification must
        % preserve that repaired curve instead of projecting these controls again.
        if degree==5, controls_units(2,1,1)=controls_units(2,1,1)+1e-6; end
        source=bmtpEngine.prepareFinalMotion(request,controls_units,durations_s);
        sourcePolynomial=bmtpEngine.createPowerPolynomial(source.ControlPoint_units, ...
            source.SegmentTime_s,0,source.PrescribedPower_units);
        original=bmtpEngine.createMotionOutput(base,request,source);
        original.PlaneCertificate=bmtpEngine.checkFinalMotion(request,[],source,reserve_units,target_units);
        assertTrue(testCase,obstacleAvoidance.validateTrajectory(original).Passed);
        changed=bmtpEngine.prepareFinalMotion(request,source.CertifiedControlPoint_units, ...
            source.SegmentTime_s,sourcePolynomial.positionPower_units, ...
            repelem([true;false;true],2),repelem([0.31;0.5;0.73],2));
        changedPolynomial=bmtpEngine.createPowerPolynomial(changed.ControlPoint_units, ...
            changed.SegmentTime_s,0,changed.PrescribedPower_units);
        output=bmtpEngine.createMotionOutput(base,request,changed);
        output.PlaneCertificate=bmtpEngine.checkFinalMotion(request,[],changed,reserve_units,target_units);
        verifyTrue(testCase,obstacleAvoidance.validateTrajectory(output).Passed);
        sampleTime_s=linspace(initial.time_s,goal.time_s,401).';
        [~,p,v,a,j]=bmtpEngine.evaluatePolynomial(sourcePolynomial,sampleTime_s);
        [~,splitP,splitV,splitA,splitJ]=bmtpEngine.evaluatePolynomial(changedPolynomial,sampleTime_s);
        verifyEqual(testCase,[splitP,splitV,splitA,splitJ],[p,v,a,j],'AbsTol',1e-9);
    end
end
