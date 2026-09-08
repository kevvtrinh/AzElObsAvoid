function tests = testConvexPreparation
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testConvexPreparation.m')
% PURPOSE: Verify exact convex partitioning on concavity, holes, and components.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Function-based tests.
% UNITS: Coordinate units and squared coordinate units.
tests = functiontests(localfunctions);
end

function testBatchedStaticOccupancy(testCase)
    root = fileparts(fileparts(mfilename('fullpath'))); addpath(root);
    a = obstacleAvoidance.obstacles.createObstacle('box',[0;2],[-1;1;1;-1],[-1;-1;1;1],0);
    b = obstacleAvoidance.obstacles.createObstacle('overlap',[0;3],[-0.5;1.5;1.5;-0.5],[-1;-1;1;1],0);
    x = [0 0 0 0 0 1]; y = zeros(size(x)); time = [-1 0 1 2 3 1];
    [occupied,index] = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime([a;b],x,y,time);
    verifyEqual(testCase,occupied,logical([0 1 1 1 1 1]));
    verifyEqual(testCase,index,uint32([0 1 1 1 2 1]));
    occupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(a,x,y,time,struct('BoundaryIsOccupied',false));
    verifyEqual(testCase,occupied,logical([0 1 1 1 0 0]));
end

function testExactUnionAndDisjointInteriors(testCase)
    root = fileparts(fileparts(mfilename('fullpath'))); addpath(root);
    rng(248);
    shapes = {polyshape([-4 4;-2 4;-2 -2;2 -2;2 4;4 4;4 -4;-4 -4]), ...
        subtract(polyshape([-4 -4;4 -4;4 4;-4 4]),polyshape([-1 -1;1 -1;1 1;-1 1]))};
    for k = 1:20
        angle = (0:19)'*pi/10; radius = 1+rand(20,1);
        shapes{end+1} = polyshape(radius.*cos(angle),radius.*sin(angle)); %#ok<AGROW>
    end
    shapes{end+1} = union(shapes{1},translate(shapes{2},[12 0]));
    for k = 1:numel(shapes)
        pieces = obstacleAvoidance.geometry.convexRegions(shapes{k});
        reconstructed = polyshape(); areaSum = 0;
        for j = 1:numel(pieces)
            p = polyshape(pieces{j},'Simplify',false);
            edges = diff([pieces{j};pieces{j}(1,:)]); next = circshift(edges,-1);
            cross = edges(:,1).*next(:,2)-edges(:,2).*next(:,1);
            verifyTrue(testCase,all(cross>=0) || all(cross<=0));
            reconstructed = union(reconstructed,p); areaSum = areaSum+area(p);
        end
        verifyLessThanOrEqual(testCase,area(xor(reconstructed,shapes{k})),1e-12);
        verifyEqual(testCase,areaSum,area(shapes{k}),'AbsTol',1e-12);
        verifyLessThanOrEqual(testCase,numel(pieces),size(triangulation(shapes{k}).ConnectivityList,1));
    end
end
