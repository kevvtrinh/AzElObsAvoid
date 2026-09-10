function tests = testAffineCells
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testAffineCells.m')
% PURPOSE: Certify moving exclusion cells without discarding physical time.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Function-based tests.
% UNITS: Coordinate units and seconds.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'));
end

function testMovingPlaneCertifiesTranslation(testCase)
    first = [1.5,-0.5;2.5,-0.5;2.5,0.5;1.5,0.5];
    vertices = cat(3,first,first+[10,0]);
    controls = [(0:8)'*10/8,zeros(9,1)];
    plane = bmtpEngine.solveSeparatingLine(controls,vertices,1e-6,1e-8);
    verifyTrue(testCase,plane.Verified);
    verifyTrue(testCase,bmtpEngine.verifySeparatingLine(plane,controls,vertices,1e-8,1e-6).Verified);
    verifyFalse(testCase,bmtpEngine.verifySeparatingLine(plane,controls,[first;first+[10,0]],1e-8,1e-6).Verified);
end

function testClippedSourceInterval(testCase)
    first = [-0.5,-0.5;0.5,-0.5;0.5,0.5;-0.5,0.5];
    obstacle = obstacleAvoidance.obstacles.createObstacle('translation',[0;10],{first(:,1);first(:,1)+10},{first(:,2);first(:,2)},0);
    prepared = obstacleAvoidance.obstacles.prepareObstacles(obstacle);
    cells = obstacleAvoidance.obstacles.createTimeCells(prepared,3,8);
    verifyEqual(testCase,cells.ActiveTimeInterval_s,[3,8]);
    verifyEqual(testCase,cells.EndRegions_units{1}-cells.Regions_units{1},repmat([5,0],4,1),'AbsTol',1e-12);
    restricted = bmtpEngine.regionOnInterval(cells.Regions_units{1},cells,1,[4,6]);
    verifyEqual(testCase,restricted(:,:,1),cells.Regions_units{1}+[1,0],'AbsTol',1e-12);
    verifyEqual(testCase,restricted(:,:,2),cells.Regions_units{1}+[3,0],'AbsTol',1e-12);
end

function testFullSourceIntervalPreservesStoredEndpoints(testCase)
    first = [1,0;2,0;2,1;1,1];
    last = [1e-16,-2;1,-2;1,-1;1e-16,-1];
    coverage = struct('Passed',true,'EndRegions_units',{{last}}, ...
        'ActiveTimeInterval_s',[3,8]);
    restricted = bmtpEngine.regionOnInterval(first,coverage,1,[3,8]);
    verifyTrue(testCase,isequal(restricted(:,:,1),first));
    verifyTrue(testCase,isequal(restricted(:,:,2),last));
end

function testCachedMovingGeometryMatchesUncachedSearch(testCase)
    angle = linspace(0,2*pi,18).';
    first = [3.1*cos(angle(1:end-1)),1.7*sin(angle(1:end-1))]+[-1.2,0.8];
    last = first.*[0.72,1.31]+[4.6,-2.4];
    parameter = linspace(0,1,9).';
    controls = [-5+12*parameter,2.8*sin(pi*parameter)-1.1*parameter];
    geometry = createTestGeometry(first,last);
    vertices = cat(3,first,last);
    uncached = bmtpEngine.solveSeparatingLine(controls,vertices,1e-5,1e-8);
    cached = bmtpEngine.solveSeparatingLine(controls,vertices,1e-5,1e-8,geometry);
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
    checked = bmtpEngine.verifySeparatingLine(plane,zeros(9,2),vertices,1e-8,0.1);
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
    altered.PlaneCertificate.Coverage.EndRegions_units{1}(:,2) = altered.PlaneCertificate.Coverage.EndRegions_units{1}(:,2)+1;
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(altered).Passed);
    altered = r;
    altered.PlaneCertificate.Coverage = rmfield(altered.PlaneCertificate.Coverage,'EndRegions_units');
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(altered).Passed);
end

function testPlaneSearchUsesCertifiedProductHull(testCase)
    controls = [ones(9,1),zeros(9,1)]; controls(5,2) = 2;
    vertices = [0.5,1.3;10,1.3;10,3;0.5,3];
    % The original control hull penetrates more in y than x. Its degree-nine
    % product hull is separated in y, so choosing by the original hull fails.
    plane = bmtpEngine.solveSeparatingLine(controls,vertices,1e-6,1e-8);
    verifyTrue(testCase,plane.Verified);
    verifyGreaterThan(testCase,plane.SignedGap_units,0.18);
    verifyTrue(testCase,bmtpEngine.verifySeparatingLine(plane,controls,vertices,1e-8,1e-6).Verified);
end

function testFinalCertificateRechecksNeighborDirections(testCase)
    box = [-0.5,-0.5;0.5,-0.5;0.5,0.5;-0.5,0.5];
    request = struct('Regions_units',{{box}},'Coverage',struct('Passed',true), ...
        'InitialState',struct('time_s',0),'IsRest',true);
    points = [-2,0;-2,0;2,0;0,0];
    prepared = struct('CertifiedControlPoint_units',repmat(reshape(points,4,1,2),1,9,1), ...
        'SegmentTime_s',ones(4,1));
    certificate = bmtpEngine.checkFinalMotion(request,struct(),prepared,1e-8,1e-6);
    verifyFalse(testCase,certificate.Passed);
    verifyEqual(testCase,certificate.VerifiedPairCount,3);
    verifyEqual(testCase,certificate.ReusedPairCount,1);
    verifyTrue(testCase,certificate.Planes(3).Verified);
    verifyFalse(testCase,certificate.Planes(4).Verified);
    for k = 1:3
        checked = bmtpEngine.verifySeparatingLine(certificate.Planes(k), ...
            squeeze(prepared.CertifiedControlPoint_units(k,:,:)),box,1e-8,1e-6);
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
