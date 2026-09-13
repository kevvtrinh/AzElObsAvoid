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

function testCompletePolynomialEdgeAdapterIsLossless(testCase)
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'));
    for degree=[5,8]
        parameter=(0:degree)/degree;
        duration_s=[1.25;2.75];
        splitFraction=duration_s(1)/sum(duration_s);
        wholeControl_units=[parameter.',(parameter.^2).'];
        controlPoint_units=zeros(2,degree+1,2);
        controlPoint_units(1,:,:)=bmtpEngine.restrictBezier( ...
            wholeControl_units,[0,splitFraction]);
        controlPoint_units(2,:,:)=bmtpEngine.restrictBezier( ...
            wholeControl_units,[splitFraction,1]);
        polynomial=bmtpEngine.createPowerPolynomial( ...
            controlPoint_units,duration_s,7);
        edges=bmtpEngine.createPolynomialEdges(polynomial);
        verifyEqual(testCase,numel(edges),2);
        verifyEqual(testCase,[edges.StartTime_s].', ...
            polynomial.SegmentStartTime_s);
        verifyEqual(testCase,[edges.SegmentDuration_s].',duration_s);
        restoredPower=zeros(size(polynomial.positionPower_units));
        restoredControls=zeros(size(controlPoint_units));
        for edgeIndex=1:numel(edges)
            restoredPower(edgeIndex,:,:)=edges(edgeIndex).PositionPower_units;
            restoredControls(edgeIndex,:,:)=edges(edgeIndex).ControlPoint_units;
        end
        restored=bmtpEngine.createPowerPolynomial(restoredControls, ...
            duration_s,7,restoredPower);
        for name=["positionPower_units","velocityPower_units_s", ...
                "accelerationPower_units_s2","jerkPower_units_s3"]
            verifyEqual(testCase,restored.(name),polynomial.(name));
        end
        verifyEqual(testCase,restored.SegmentStartTime_s, ...
            polynomial.SegmentStartTime_s);
        verifyEqual(testCase,restored.FinalTime_s,polynomial.FinalTime_s);
        verifyEqual(testCase,edges(1).EndJet,edges(2).StartJet, ...
            'AbsTol',1e-12);
    end
    initial=struct('time_s',3,'position_units',[-2,0]);
    goal=struct('time_s',8,'position_units',[2,1]);
    limits=struct('xInterval_units',[-5,5],'yInterval_units',[-5,5], ...
        'maxVelocity_units_s',[10,10], ...
        'maxAcceleration_units_s2',[20,20], ...
        'maxJerk_units_s3',[100,100]);
    base=planner([],initial,goal,limits,struct('GoalTimeMode','fixedArrival'));
    edges=bmtpEngine.createPolynomialEdges(base.Polynomial);
    seed=struct('position_units',[base.Inputs.initialState.position_units; ...
        base.Inputs.goalState.position_units],'tau',[0;1], ...
        'Source',"completeMotion",'PolynomialEdges',edges);
    request=bmtpEngine.createSolveRequest(seed,cell(0,1),struct('Passed',true), ...
        base.Inputs.initialState,base.Inputs.goalState,base.Limits,base.Options);
    warmStart=bmtpEngine.createWarmStart(request);
    verifyEqual(testCase,warmStart.SegmentTime_s, ...
        base.Polynomial.SegmentDuration_s);
    verifyEqual(testCase,warmStart.PrescribedPower_units, ...
        base.Polynomial.positionPower_units);
    alteredEdges=edges;
    alteredEdges(2).StartJet(2,1)=alteredEdges(2).StartJet(2,1)+1;
    request.Seed.PolynomialEdges=alteredEdges;
    verifyError(testCase,@()bmtpEngine.createWarmStart(request), ...
        'bmtpEngine:InvalidPolynomialEdges');
end
