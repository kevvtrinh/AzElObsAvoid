function tests = testBoundedCorrespondence
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testBoundedCorrespondence.m')
% PURPOSE: Compare bounded alignment with exhaustive reference geometry.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Alignment, interpolation, and cyclic-order regression results.
% UNITS: Coordinate units and seconds.
tests = functiontests(localfunctions);
end

function setupOnce(~)
    root=fileparts(fileparts(mfilename('fullpath'))); addpath(root,fullfile(root,'trajectory'));
end

function testTranslationAcrossStartsOrientationsAndScales(testCase)
    for count=[7,31,140,220]
        theta=(0:count-1)'*2*pi/count;
        radius=2+0.4*cos(3*theta)+0.2*sin(5*theta);
        ring=radius.*[cos(theta),sin(theta)];
        for scale=[1e-4,1,1e4]
            lower=scale*ring+[120,-70]; upper=lower+scale*[0.25,-0.1];
            for reverse=[false,true]
                reordered=circshift(upper,floor(count/3));
                if reverse, reordered=flipud(reordered); end
                prepared=preparePair(lower,reordered);
                verifyTrue(testCase,prepared.InternalPreparation.MatchingTopology);
                actual=lower+[prepared.InternalPreparation.DeltaX_units{1},prepared.InternalPreparation.DeltaY_units{1}];
                verifyEqual(testCase,actual,upper,'AbsTol',8*eps(max(abs(upper),[],'all')));
            end
        end
    end
end

function testDeformingConvexAgainstExhaustiveReference(testCase)
    for count=[9,32,140,220]
        theta=(0:count-1)'*2*pi/count;
        lower=[2*cos(theta),sin(theta)];
        upper=[2.05*cos(theta+0.015),0.97*sin(theta+0.015)]+[0.2,-0.1];
        upper=flipud(circshift(upper,floor(count/3)));
        expected=exhaustiveAlignment(lower,upper);
        prepared=preparePair(lower,upper);
        verifyTrue(testCase,prepared.InternalPreparation.MatchingTopology);
        actual=lower+[prepared.InternalPreparation.DeltaX_units{1},prepared.InternalPreparation.DeltaY_units{1}];
        verifyEqual(testCase,actual,expected,'AbsTol',1e-14);
        expectedShape=polyshape((lower+expected)/2,'Simplify',false);
        actualShape=obstacleAvoidance.obstacles.preparedShapeAtTime(prepared,0.5);
        verifyLessThan(testCase,area(xor(actualShape,expectedShape)),1e-12);
    end
end

function testSymmetricTiesAreStartingIndexInvariant(testCase)
    theta=(0:7)'*pi/4;
    lower=[cos(theta),sin(theta)]; upper=[cos(theta+pi/8),sin(theta+pi/8)];
    reference=preparePair(lower,upper);
    referenceShape=obstacleAvoidance.obstacles.preparedShapeAtTime(reference,0.3);
    for shift=0:7
        for reverse=[false,true]
            changed=circshift(upper,shift); if reverse, changed=flipud(changed); end
            changedLower=circshift(lower,2*shift);
            if mod(shift,2)==0, changedLower=flipud(changedLower); end
            prepared=preparePair(changedLower,changed);
            actual=obstacleAvoidance.obstacles.preparedShapeAtTime(prepared,0.3);
            verifyLessThan(testCase,area(xor(referenceShape,actual)),1e-12);
        end
    end
end

function testUnsupportedConcaveDeformationRemainsExplicit(testCase)
    lower=[0,0;2,0;2,2;1,0.5;0,2]; upper=lower; upper(4,:)=[0.5,1];
    prepared=preparePair(lower,circshift(upper,2));
    verifyFalse(testCase,prepared.InternalPreparation.MatchingTopology);
    verifyTrue(testCase,startsWith(prepared.InternalPreparation.IntervalGeometryModel,"conservative"));
    shape=obstacleAvoidance.obstacles.preparedShapeAtTime(prepared,0.5);
    verifyLessThan(testCase,area(subtract(polyshape(lower),shape)),1e-12);
    verifyLessThan(testCase,area(subtract(polyshape(upper),shape)),1e-12);
end

function prepared=preparePair(lower,upper)
    source=obstacleAvoidance.obstacles.createObstacle('generic',[0;1], ...
        {lower(:,1);upper(:,1)},{lower(:,2);upper(:,2)},0);
    prepared=obstacleAvoidance.obstacles.prepareObstacles(source);
end

function best=exhaustiveAlignment(lower,upper)
    best=[]; bestCost=Inf;
    for reverse=[false,true]
        oriented=upper; if reverse, oriented=flipud(oriented); end
        for shift=0:size(lower,1)-1
            candidate=circshift(oriented,shift);
            cost=sum((candidate-lower).^2,'all');
            if cost<bestCost, bestCost=cost; best=candidate; end
        end
    end
end
