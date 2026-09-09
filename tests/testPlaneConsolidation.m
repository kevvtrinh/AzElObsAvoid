function tests = testPlaneConsolidation
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testPlaneConsolidation.m')
% PURPOSE: Independently verify redundant affine-plane removal and length bounds.
% INPUTS: MATLAB unit test framework and Optimization Toolbox.
% OUTPUTS: Linear-program implication checks and complete planner regressions.
% UNITS: Coordinate units and seconds.
tests = functiontests(localfunctions);
end

function setupOnce(~)
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'),fullfile(root,'examples'));
end

function testAffineImplicationAgainstIndependentLinearPrograms(testCase)
    angle=(0:7)'*pi/4;
    normals=[cos(angle),sin(angle)];
    distances=repmat([1;3;1;3;1;3;1;3],1,2);
    distances(2,:)=[3,0.9];
    planes=makePlanes(normals,-distances);
    limits=struct('xInterval_units',[-4,4],'yInterval_units',[-4,4]);
    reserve_units=2e-8;
    reduced=bmtpEngine.removeRedundantPlanes(planes,limits,reserve_units);
    retained=[reduced.Active];
    verifyLessThan(testCase,nnz(retained),numel(planes));
    verifyTrue(testCase,retained(2),'A plane that cuts the corridor at its second endpoint must remain.');
    settings=optimoptions('linprog','Display','none','ConstraintTolerance',1e-10);
    for tau=linspace(0,1,9)
        offsets_units=-(1-tau)*distances(:,1)-tau*distances(:,2);
        for index=find(~retained)
            [point_units,~,exitFlag]=linprog(-normals(index,:).',normals(retained,:), ...
                -offsets_units(retained)-reserve_units,[],[],[-4;-4],[4;4],settings);
            verifyGreaterThan(testCase,exitFlag,0);
            violation_units=normals(index,:)*point_units+offsets_units(index)+reserve_units;
            verifyLessThanOrEqual(testCase,violation_units,1e-9);
        end
    end
end

function testActualCutAndLargeCoordinatesAreRetained(testCase)
    normals=[1,0;0,1;1/sqrt(2),1/sqrt(2)];
    distances=[1;1;sqrt(2)-1e-4];
    planes=makePlanes(normals,-repmat(distances,1,2));
    limits=struct('xInterval_units',[-1e9,1e9],'yInterval_units',[-1e9,1e9]);
    reserve_units=2e-8;
    reduced=bmtpEngine.removeRedundantPlanes(planes,limits,reserve_units);
    verifyTrue(testCase,reduced(3).Active);
    witness_units=[1,1]-2*reserve_units;
    verifyGreaterThan(testCase,normals(3,:)*witness_units.'-distances(3)+reserve_units,1e-5);
end

function testGeometricBoundSkipsOnlyUnattainableTimeTrade(testCase)
    display=struct('PlotOutputs',false,'Verbose',false);
    allowed=exampleTwoOpposingUVisibilityGraph(display);
    display.PathLengthTimeAllowance_s=0;
    strict=exampleTwoOpposingUVisibilityGraph(display);
    verifyTrue(testCase,allowed.Success,allowed.Message);
    verifyTrue(testCase,strict.Success,strict.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(allowed).Passed);
    refinement=allowed.SolverDiagnostics.PathLengthRefinement;
    verifyTrue(testCase,refinement.TimeTradeSkippedByLengthBound);
    verifyGreaterThanOrEqual(testCase,refinement.GeometricLowerBound_units,0.99*allowed.MotionLength_units);
    verifyEqual(testCase,strict.Options.PathLengthTimeAllowance_s,0);
    verifyEqual(testCase,allowed.ArrivalTime_s,strict.ArrivalTime_s,'AbsTol',1e-8);
    verifyEqual(testCase,allowed.MotionLength_units,strict.MotionLength_units,'AbsTol',1e-8);
end

function planes=makePlanes(normals,offsets_units)
    empty=struct('Active',true,'Verified',true,'ExitFlag',1,'Normal',zeros(2), ...
        'Offset_units',[0,0],'SignedGap_units',1);
    planes=repmat(empty,1,size(normals,1));
    for index=1:numel(planes)
        planes(index).Normal=repmat(normals(index,:),2,1);
        planes(index).Offset_units=offsets_units(index,:);
    end
end
