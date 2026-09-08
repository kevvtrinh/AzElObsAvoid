function tests = testBatchedContainment
%% Section 0: Header & Readme
% SYNTAX: tests = testBatchedContainment
% PURPOSE: Compare batched ray parity with the original polyshape predicate.
% INPUTS: None.
% OUTPUTS: Deterministic MATLAB function tests.
% UNITS: Coordinate units.
tests = functiontests(localfunctions);
end

function setupOnce(~)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'));
end

function testRandomRingsHolesAndOffsets(testCase)
    previous = rng(8301); cleanup = onCleanup(@() rng(previous));
    square = polyshape([0 4 4 0],[0 0 4 4]);
    hole = subtract(square,polyshape([1 3 3 1],[1 1 3 3]));
    island = union(hole,polyshape([1.5 2.5 2.5 1.5],[1.5 1.5 2.5 2.5]));
    disconnected = union(island,translate(square,[6 0]));
    shapes = {polyshape(),square,hole,island,disconnected};
    for count = [13 120 1200]
        angle = (0:count-1).' * 2*pi/count;
        radius = 3 + rand(count,1);
        shapes{end+1} = polyshape(radius.*cos(angle),radius.*sin(angle));
    end
    for shapeIndex = 1:numel(shapes)
        for offset = [0 1e6 1e9]
            shape = translate(shapes{shapeIndex},[offset -offset]);
            points = 14*rand(1000,2)-5+[offset -offset];
            [a,b] = obstacleAvoidance.geometry.boundaryToEdges(shape,0);
            if ~isempty(a)
                middle = 0.5*(a+b);
                points = [points; a; middle; middle+[1e-8 0]; middle-[1e-8 0]; middle+[0 1e-8]; middle-[0 1e-8]];
            end
            [actual,nearest,edge] = obstacleAvoidance.geometry.pointPolygonClearance(shape,points);
            [expected,referenceNearest,referenceEdge] = pointPolygonClearanceReference(shape,points);
            verifyEqual(testCase,actual,expected);
            verifyEqual(testCase,nearest,referenceNearest);
            verifyEqual(testCase,edge,referenceEdge);
        end
    end
end

function testPreparedConvexAndConcaveQueries(testCase)
    for vertexCount = [4 17 257]
        angle = (0:vertexCount-1).' * 2*pi/vertexCount;
        for amplitude = [0 0.7]
            radius = 3+amplitude*cos(5*angle);
            obstacle = obstacleAvoidance.obstacles.createObstacle("parity",[2;6],radius.*cos(angle),radius.*sin(angle),0);
            obstacle = obstacleAvoidance.obstacles.prepareObstacles(obstacle);
            [shape,geometry] = obstacleAvoidance.obstacles.preparedShapeAtTime(obstacle,4);
            [x,y] = meshgrid(linspace(-4,4,71));
            points = [x(:),y(:);shape.Vertices];
            [actual,nearest,edge] = obstacleAvoidance.geometry.pointPolygonClearance(shape,points,geometry);
            [expected,referenceNearest,referenceEdge] = pointPolygonClearanceReference(shape,points,geometry);
            verifyEqual(testCase,actual,expected);
            verifyEqual(testCase,nearest,referenceNearest);
            verifyEqual(testCase,edge,referenceEdge);
        end
    end
end
