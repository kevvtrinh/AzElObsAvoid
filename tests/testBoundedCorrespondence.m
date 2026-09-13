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

function testConcaveDeformationUsesExactMovingPartition(testCase)
    lower=[0,0;2,0;2,2;1,0.5;0,2]; upper=lower; upper(4,:)=[0.5,1];
    prepared=preparePair(lower,circshift(upper,2));
    verifyTrue(testCase,prepared.InternalPreparation.MatchingTopology);
    verifyEqual(testCase,prepared.InternalPreparation.IntervalGeometryModel, ...
        "linearCorrespondingConvexPartition");
    cells=obstacleAvoidance.obstacles.createTimeCells(prepared,0,1);
    verifyGreaterThan(testCase,numel(cells.Regions_units),1);
    for tau=[0,0.25,0.5,0.75,1]
        partitionShape=polyshape();
        for regionIndex=1:numel(cells.Regions_units)
            region=cells.Regions_units{regionIndex}+tau* ...
                (cells.EndRegions_units{regionIndex}-cells.Regions_units{regionIndex});
            partitionShape=union(partitionShape,polyshape(region,'Simplify',false));
        end
        exactBoundary=obstacleAvoidance.obstacles.preparedShapeAtTime(prepared,tau);
        verifyLessThan(testCase,area(xor(partitionShape,exactBoundary)),1e-12);
        verifyFalse(testCase,isinterior(partitionShape,1,1.75));
    end
end

function testContainmentClassificationMatchesBooleanReference(testCase)
    % Unequal ring lengths bypass correspondence. Cover shrinking, growing,
    % equal-area shifted, disconnected and holed shapes, in both orders.
    lower=[0,0;4,0;4,1;1,1;1,4;0,4];
    candidates={0.8*lower+[0.1,0.1],1.3*lower-[0.1,0.1], ...
        lower+[0.1,0.2],lower+[8,0], ...
        [lower;NaN,NaN;8,0;9,0;9,1;8,1], ...
        [-1,-1;6,-1;6,6;-1,6;NaN,NaN;2,2;2,3;3,3;3,2]};
    for index=1:numel(candidates)
        upper=candidates{index};
        upper=[upper(1,:);mean(upper(1:2,:),1);upper(2:end,:)];
        for reversed=[false,true]
            first=lower; last=upper;
            if reversed, first=upper; last=lower; end
            prepared=preparePair(first,last);
            firstShape=polyshape(first,'Simplify',false);
            lastShape=polyshape(last,'Simplify',false);
            tolerance=512*eps(max([1,area(firstShape),area(lastShape)]));
            equivalent=area(xor(firstShape,lastShape))<=tolerance;
            if equivalent, expected="staticEquivalentSamples";
            else, expected="unsupportedContinuousDeformation"; end
            verifyEqual(testCase,prepared.InternalPreparation.IntervalGeometryModel,expected);
        end
    end
end

function testUnsupportedGeometryReturnsStablePlannerOutcome(testCase)
    lower=[0,0;2,0;2,2;0,2];
    upper=[0,0;2,0;2,2;1,3;0,2];
    obstacle=obstacleAvoidance.obstacles.createObstacle('changing count',[0;1], ...
        {lower(:,1);upper(:,1)},{lower(:,2);upper(:,2)},0);
    initial=struct('time_s',0,'position_units',[-2,0], ...
        'velocity_units_s',[0,0],'acceleration_units_s2',[0,0]);
    goal=struct('time_s',1,'position_units',[4,0], ...
        'velocity_units_s',[0,0],'acceleration_units_s2',[0,0]);
    limits=struct('xInterval_units',[-5,5],'yInterval_units',[-5,5], ...
        'maxVelocity_units_s',[10,10],'maxAcceleration_units_s2',[10,10], ...
        'maxJerk_units_s3',[20,20]);
    result=planner(obstacle,initial,goal,limits,struct('GoalTimeMode','fixedArrival'));
    verifyFalse(testCase,result.Success);
    verifyEqual(testCase,result.TerminationReason,"unsupportedObstacleInterpolation");
end

function testCachedUnsupportedFutureDoesNotRejectEarlierHorizon(testCase)
    first=[10,10;12,10;12,12;10,12];
    second=first+[0.2,0];
    third=[10.4,10;12.4,10;12.4,12;11.4,13;10.4,12];
    source=obstacleAvoidance.obstacles.createObstacle('later unsupported',[0;1;2], ...
        {first(:,1);second(:,1);third(:,1)}, ...
        {first(:,2);second(:,2);third(:,2)},0);
    prepared=obstacleAvoidance.obstacles.prepareObstacles(source,[0,2]);
    initial=struct('time_s',0,'position_units',[-2,0]);
    goal=struct('time_s',0.5,'position_units',[-1,0]);
    limits=struct('xInterval_units',[-5,15],'yInterval_units',[-5,15], ...
        'maxVelocity_units_s',[10,10],'maxAcceleration_units_s2',[100,100], ...
        'maxJerk_units_s3',[1000,1000]);
    options=struct('GoalTimeMode','fixedArrival','FixedArrivalSearch','spatial');
    early=planner(prepared,initial,goal,limits,options);
    verifyNotEqual(testCase,early.TerminationReason,"unsupportedObstacleInterpolation");
    goal.time_s=1.5;
    overlapping=planner(prepared,initial,goal,limits,options);
    verifyEqual(testCase,overlapping.TerminationReason,"unsupportedObstacleInterpolation");
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
