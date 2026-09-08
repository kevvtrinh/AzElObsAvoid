function tests = testBatchedVisibility
% Preserve complete predicate and graph-edge evidence while bounding memory.
tests = functiontests(localfunctions);
end

function setupOnce(~)
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'));
end

function testScalarReferenceAtContactsHolesAndOffsets(testCase)
    previous=rng(30192,'twister'); cleanup=onCleanup(@() rng(previous));
    square=polyshape([0 4 4 0],[0 0 4 4]);
    hole=subtract(square,polyshape([1 3 3 1],[1 1 3 3]));
    shapes={polyshape(),square,hole,union(hole,translate(square,[6 0]))};
    for count=[12 120 1200]
        angle=(0:count-1).'*2*pi/count; radius=1+.2*rand(count,1);
        shapes{end+1}=polyshape(radius.*cos(angle),radius.*sin(angle));
    end
    for shapeIndex=1:numel(shapes)
        for offset=[0 1e6 1e9]
            shape=translate(shapes{shapeIndex},[offset -offset]);
            [a,b]=obstacleAvoidance.geometry.boundaryToEdges(shape,0);
            first=14*rand(1100,2)-5+[offset -offset];
            second=14*rand(1100,2)-5+[offset -offset];
            if ~isempty(a)
                first=[first;a;(a+b)/2;a;a+[1e-8 0]];
                second=[second;b;(a+b)/2;a+[.1 .2];a-[1e-8 0]];
            end
            expected=checkVisibilitySegmentsReference(first,second,shape,a,b);
            actual=obstacleAvoidance.search.checkVisibilitySegments(first,second,shape,a,b);
            verifyEqual(testCase,actual,expected);
        end
    end
end

function testGraphCostsAndRecordedEdgesUseTheSamePredicate(testCase)
    shape=polyshape([-.5 .5 .5 -.5],[-1 -1 1 1]);
    [a,b]=obstacleAvoidance.geometry.boundaryToEdges(shape,0);
    attempt=obstacleAvoidance.search.createVisibilityAttempt(shape,[-3 0],[3 0],struct('xInterval_units',[-4 4],'yInterval_units',[-3 3]),1e-3,0,1e6);
    nodes=attempt.Nodes.Positions_units; pairs=attempt.FinalCandidatePairs;
    first=nodes(pairs(:,1),:); second=nodes(pairs(:,2),:);
    expected=checkVisibilitySegmentsReference(first,second,shape,a,b);
    verifyEqual(testCase,attempt.AcceptedEdges_units,[first(expected,:) second(expected,:)]);
    verifyEqual(testCase,attempt.RejectedEdges_units,[first(~expected,:) second(~expected,:)]);
    verifyEqual(testCase,attempt.VisibilityEdgeCount,nnz(expected));
    verifyEqual(testCase,attempt.RejectedTransitionCount,nnz(~expected));
    costs=attempt.Cost_units(sub2ind(size(attempt.Cost_units),pairs(:,1),pairs(:,2)));
    verifyEqual(testCase,isfinite(costs),expected);
    verifyEqual(testCase,costs(expected),vecnorm(second(expected,:)-first(expected,:),2,2));
end
