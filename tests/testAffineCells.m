function tests = testAffineCells
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testAffineCells.m')
% PURPOSE: Prove moving exclusion cells without discarding physical time.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Function-based tests.
% UNITS: Coordinate units and seconds.
tests = functiontests(localfunctions);
end

function setupOnce(~)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'));
end

function testMovingPlaneProvesTranslation(testCase)
    first = [1.5,-0.5;2.5,-0.5;2.5,0.5;1.5,0.5];
    vertices = cat(3,first,first+[10,0]);
    controls = [(0:8)'*10/8,zeros(9,1)];
    plane = bmtpEngine.separation.solveSeparatingLine(controls,vertices,1e-6,1e-8);
    verifyTrue(testCase,plane.Verified);
    verifyTrue(testCase,bmtpEngine.separation.verifySeparatingLine(plane,controls,vertices,1e-8,1e-6).Verified);
    verifyFalse(testCase,bmtpEngine.separation.verifySeparatingLine(plane,controls,[first;first+[10,0]],1e-8,1e-6).Verified);
end

function testStaticPlaneMatchesStationaryAffineRepresentation(testCase)
    vertices = [1.5,-0.5;2.5,-0.5;2.5,0.5;1.5,0.5];
    controls = [linspace(0,0.5,9).',zeros(9,1)];
    plane = struct('Normal',[1,0;0.75,0.5],'Offset_units',[-0.75,-0.75]);
    stationary = cat(3,vertices,vertices);
    checked = bmtpEngine.separation.verifySeparatingLine(plane,controls,vertices,1e-8,1e-6);
    verifyTrue(testCase,checked.Verified);
    verifyEqual(testCase,checked,bmtpEngine.separation.verifySeparatingLine(plane,controls,stationary,1e-8,1e-6));
    controls(end,:) = [3,0];
    verifyFalse(testCase,bmtpEngine.separation.verifySeparatingLine(plane,controls,vertices,1e-8,1e-6).Verified);
    verifyFalse(testCase,bmtpEngine.separation.verifySeparatingLine(plane,controls,stationary,1e-8,1e-6).Verified);
end

function testCachedMovingGeometryMatchesUncachedSearch(testCase)
    angle = linspace(0,2*pi,18).';
    first = [3.1*cos(angle(1:end-1)),1.7*sin(angle(1:end-1))]+[-1.2,0.8];
    last = first.*[0.72,1.31]+[4.6,-2.4];
    parameter = linspace(0,1,9).';
    controls = [-5+12*parameter,2.8*sin(pi*parameter)-1.1*parameter];
    geometry = createTestGeometry(first,last);
    vertices = cat(3,first,last);
    uncached = bmtpEngine.separation.solveSeparatingLine(controls,vertices,1e-5,1e-8);
    cached = bmtpEngine.separation.solveSeparatingLine(controls,vertices,1e-5,1e-8,geometry);
    verifyEqual(testCase,cached.Verified,uncached.Verified);
    verifyEqual(testCase,cached.Normal,uncached.Normal,'AbsTol',64*eps);
    verifyEqual(testCase,cached.Offset_units,uncached.Offset_units,'AbsTol',64*eps);
    verifyEqual(testCase,cached.SignedGap_units,uncached.SignedGap_units,'AbsTol',256*eps);
end

function testInteriorObstaclePlaneViolationRejected(testCase)
    first = [0.9,-0.1;1.1,-0.1;1.1,0.1;0.9,0.1];
    vertices = cat(3,first,first-[2,0]);
    plane = struct('Normal',[1,0;-1,0],'Offset_units',[-0.2,-0.2]);
    verifyGreaterThan(testCase,min(first(:,1)-0.2),0.1);
    checked = bmtpEngine.separation.verifySeparatingLine(plane,zeros(9,2),vertices,1e-8,0.1);
    verifyFalse(testCase,checked.Verified);
    verifyLessThan(testCase,checked.SignedGap_units,0);
end

function testAlteredEndpointCoverageRejected(testCase)
    x = [-0.5;0.5;0.5;-0.5]; y = [-0.5;-0.5;0.5;0.5];
    obstacle = obstacleAvoidance.obstacles.createObstacle('box',[0;12],{x;x},{y+2;y+3},0.1);
    r = planner(obstacle,struct('time_s',0,'position_units',[-4,0]), ...
        struct('time_s',12,'position_units',[4,0]),struct(),struct());
    verifyTrue(testCase,r.Success,r.Message);
    altered = r;
    altered.Diagnostics.SeparationProof.Coverage.EndRegions_units{1}(:,2) = altered.Diagnostics.SeparationProof.Coverage.EndRegions_units{1}(:,2)+1;
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(altered).Passed);
    altered = r;
    altered.Diagnostics.SeparationProof.Coverage = rmfield(altered.Diagnostics.SeparationProof.Coverage,'EndRegions_units');
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(altered).Passed);
end

function testPlaneSearchUsesProvenProductHull(testCase)
    controls = [ones(9,1),zeros(9,1)]; controls(5,2) = 2;
    vertices = [0.5,1.3;10,1.3;10,3;0.5,3];
    % The original control hull penetrates more in y than x. Its degree-nine
    % product hull is separated in y, so choosing by the original hull fails.
    plane = bmtpEngine.separation.solveSeparatingLine(controls,vertices,1e-6,1e-8);
    verifyTrue(testCase,plane.Verified);
    verifyGreaterThan(testCase,plane.SignedGap_units,0.18);
    verifyTrue(testCase,bmtpEngine.separation.verifySeparatingLine(plane,controls,vertices,1e-8,1e-6).Verified);
end

function testFinalProofRechecksNeighborDirections(testCase)
    box = [-0.5,-0.5;0.5,-0.5;0.5,0.5;-0.5,0.5];
    request = struct('Regions_units',{{box}},'Coverage',struct('Passed',true), ...
        'InitialState',struct('time_s',0),'IsRest',true);
    points = [-2,0;-2,0;2,0;0,0];
    prepared = struct('ControlPoint_units',repmat(reshape(points,4,1,2),1,9,1), ...
        'SegmentTime_s',ones(4,1),'FinalTime_s',4);
    proof = bmtpEngine.validation.checkFinalMotion(request,prepared,1e-8,1e-6);
    verifyFalse(testCase,proof.Passed);
    verifyEqual(testCase,proof.VerifiedPairCount,3);
    verifyEqual(testCase,proof.ReusedPairCount,1);
    verifyTrue(testCase,proof.Planes(3).Verified);
    verifyFalse(testCase,proof.Planes(4).Verified);
    for k = 1:3
        checked = bmtpEngine.separation.verifySeparatingLine(proof.Planes(k), ...
            squeeze(prepared.ControlPoint_units(k,:,:)),box,1e-8,1e-6);
        verifyTrue(testCase,checked.Verified);
    end
end

function geometry = createTestGeometry(first,last)
    edges = [diff([first;first(1,:)],1,1);diff([last;last(1,:)],1,1)];
    lengths = vecnorm(edges,2,2);
    edges = edges(lengths>0,:);
    lengths = lengths(lengths>0);
    normals = [-edges(:,2),edges(:,1)]./lengths;
    firstProjection = first*normals.';
    lastProjection = last*normals.';
    geometry = struct('PositiveNormals',normals, ...
        'FirstPositiveSupport_units',min(firstProjection,[],1), ...
        'LastPositiveSupport_units',min(lastProjection,[],1), ...
        'FirstNegativeSupport_units',-max(firstProjection,[],1), ...
        'LastNegativeSupport_units',-max(lastProjection,[],1));
end
