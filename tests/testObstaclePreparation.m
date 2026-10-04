function tests = testObstaclePreparation
% Check canonical queries, exact partitions, repairs, and interval models.
% Run with runtests('tests/testObstaclePreparation.m').
tests = functiontests(localfunctions);
end

function setupOnce(~)
    rootFolder = fileparts(fileparts(mfilename('fullpath')));
    addpath(rootFolder, fullfile(rootFolder, 'trajectory'), fullfile(rootFolder, 'examples'));
end

function testCanonicalPreparationAndQueryContracts(testCase)
    box_units = [-0.5, -0.5; 0.5, -0.5; 0.5, 0.5; -0.5, 0.5];
    runCases(testCase, { ...
        @emptyCanonicalSchemaMatchesNonemptyRecord; ...
        @(testCase) clippedSourceInterval(testCase, box_units); ...
        @(testCase) completePreparationReuseAndSourceChanges(testCase, box_units); ...
        @exactSampleIdentityCacheRefresh; ...
        @(testCase) geometryQueriesAndSourceRefresh(testCase, box_units); ...
        @(testCase) occupancyBoundaryAndBlockingContract(testCase, box_units); ...
        @boundaryOnlyGeometryPreservesPreparedModel; @marginOnce});
end

function testCorrespondenceAndExactPartitionContracts(testCase)
    runCases(testCase, { ...
        @translationAcrossStartsOrientationsAndScales; @consecutiveTranslationsReuseExactPartition; ...
        @deformingConvexAgainstExhaustiveReference; @symmetricTiesAreStartingIndexInvariant; ...
        @concaveDeformationUsesExactMovingPartition; @separatedCollinearEdgesRemainValidDuringNonaffineMotion; ...
        @redundantAffineKeyframesUseIdenticalCellsAndMotion});
end

function testRingRepairContracts(testCase)
    runCases(testCase, { ...
        @properCrossingZigzagHasDeclaredRepair; @smallInteriorFoldKeepsTheSameVertices; ...
        @smallSeamFoldRemovesTheCyclicComplement; @polygonClosureRoundoffKeepsTheLargerLoop; ...
        @closingCopyRemovalRemainsExact; @repairedSourceIndexCountChangeReportsEndpointHulls});
end

function testConservativeIntervalModelContracts(testCase)
    runCases(testCase, { ...
        @containmentClassificationMatchesBooleanReference; @changingVertexCountPreservesTimingAndPlansValidMotion; ...
        @cachedEndpointHullPlansEarlierAndOverlappingHorizons; @movingCellsContainUnprovableCorrespondingRing; ...
        @appearingObstacleBlocksStraightMotion; @ringSplitOccupiesFragmentsAndGap; ...
        @twoDeformingRingsAndAddedArea; @largeNonoverlappingJumpCoversCrossSampleSegments; ...
        @swappedRingIdentitiesCoverCrossSampleSegments; @degenerateEndpointRemainsUnsupported});
end

function testSourceIndexMarginAndProvenanceContracts(testCase)
    [lower_units, upper_units] = thinNotchFixture();
    runCases(testCase, { ...
        @subEpsilonVelocityDriftIsNotMergedAway; ...
        @(testCase) movingCellsDoNotBecomeAnExactTranslationPartition(testCase, lower_units, upper_units); ...
        @rootTwoMarginSquaresContainSquareJoinProtection; @movingObstacleDeclaresSourceIndexCorrespondence; ...
        @(testCase) reportedModelMutationDoesNotSelectBehavior(testCase, lower_units, upper_units)});
end

function emptyCanonicalSchemaMatchesNonemptyRecord(testCase)
    % Keep the obstacle-free canonical schema equal to populated records.
    vertices_units = [0, 0; 1, 0; 1, 1; 0, 1];
    populated = obstacleAvoidance.obstacles.createObstacle( ...
        'schema reference', 0, vertices_units(:, 1), vertices_units(:, 2));
    empty = obstacleAvoidance.obstacles.combineObstacles();
    verifyEmpty(testCase, empty);
    verifyEqual(testCase, fieldnames(empty), fieldnames(populated));
end

function clippedSourceInterval(testCase, box_units)
    first = box_units;
    obstacle = obstacleAvoidance.obstacles.createObstacle('translation', [0; 10], ...
        {first(:, 1); first(:, 1) + 10}, {first(:, 2); first(:, 2)}, 0);
    prepared = obstacleAvoidance.obstacles.prepareObstacles(obstacle);
    cells = obstacleAvoidance.obstacles.createTimeCells(prepared,3,8);
    verifyEqual(testCase,cells.ActiveTimeInterval_s,[3,8]);
    verifyEqual(testCase,cells.EndRegions_units{1}-cells.Regions_units{1},repmat([5,0],4,1),'AbsTol',1e-12);
    restricted = bmtpEngine.separation.regionOnInterval(cells.Regions_units{1},cells,1,[4,6]);
    verifyEqual(testCase,restricted(:,:,1),cells.Regions_units{1}+[1,0],'AbsTol',1e-12);
    verifyEqual(testCase,restricted(:,:,2),cells.Regions_units{1}+[3,0],'AbsTol',1e-12);
    % Return stored endpoints exactly, even when interpolation would cancel digits.
    first = [1,0;2,0;2,1;1,1];
    last = [1e-16,-2;1,-2;1,-1;1e-16,-1];
    coverage = struct('Passed',true,'EndRegions_units',{{last}}, ...
        'ActiveTimeInterval_s',[3,8]);
    restricted = bmtpEngine.separation.regionOnInterval(first,coverage,1,[3,8]);
    verifyTrue(testCase,isequal(restricted(:,:,1),first));
    verifyTrue(testCase,isequal(restricted(:,:,2),last));
end

function completePreparationReuseAndSourceChanges(testCase, box_units)
    box = box_units;
    source = obstacleAvoidance.obstacles.createObstacle('translation',[0;5;10], ...
        {box(:,1);box(:,1)+1;box(:,1)+2},repmat({box(:,2)},3,1),0);
    partial = obstacleAvoidance.obstacles.prepareObstacles(source,[0,2]);
    % A touched redundant span is proven in full, independent of query window.
    verifyTrue(testCase,partial.InternalPreparation.SamplePrepared(end));
    verifyEqual(testCase, partial.InternalPreparation.SpanStartSampleIndex, [1; 1]);
    verifyEqual(testCase, partial.InternalPreparation.SpanEndSampleIndex, [3; 3]);
    changedVelocity=source;
    changedVelocity.x_units{end}=changedVelocity.x_units{end}+1;
    changedVelocity.originalX_units{end}=changedVelocity.originalX_units{end}+1;
    scoped=obstacleAvoidance.obstacles.prepareObstacles(changedVelocity,[0,2]);
    verifyFalse(testCase,scoped.InternalPreparation.SamplePrepared(end));
    complete = obstacleAvoidance.obstacles.prepareObstacles(partial,[0,10]);
    verifyTrue(testCase,all(complete.InternalPreparation.SamplePrepared));
    verifyTrue(testCase,all(complete.InternalPreparation.IntervalPrepared));
    reused = obstacleAvoidance.obstacles.prepareObstacles(complete,[3,7]);
    verifyTrue(testCase,isequaln(reused,complete));
    verifyTrue(testCase,obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(reused,1.6,0,8));
    % A complete cache must still be invalidated when supplied data changes.
    changed = complete;
    changed.x_units = cellfun(@(x)x+10,changed.x_units,'UniformOutput',false);
    changed.originalX_units = cellfun(@(x)x+10,changed.originalX_units,'UniformOutput',false);
    [occupied,blocking] = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(changed,[1.6,11.6],[0,0],8);
    verifyEqual(testCase,occupied,[false,true]);
    verifyEqual(testCase,blocking,uint32([0,1]));
end

function exactSampleIdentityCacheRefresh(testCase)
    box = [-1,-1;1,-1;1,1;-1,1];
    source = obstacleAvoidance.obstacles.createObstacle('static', [0; 5; 10], ...
        repmat({box(:, 1)}, 3, 1), repmat({box(:, 2)}, 3, 1), 0);
    prepared = obstacleAvoidance.obstacles.prepareObstacles(source);
    verifyTrue(testCase,prepared.InternalPreparation.SamplesExactlyEqual);
    legacy = prepared;
    legacy.InternalPreparation = rmfield(legacy.InternalPreparation,'SamplesExactlyEqual');
    legacy.InternalPreparation.PreparationVersion = 4;
    migrated = obstacleAvoidance.obstacles.prepareObstacles(legacy);
    verifyTrue(testCase,migrated.InternalPreparation.SamplesExactlyEqual);
    verifyGreaterThan(testCase,migrated.InternalPreparation.PreparationVersion,4);
    changed = prepared;
    changed.x_units{end} = changed.x_units{end}+10;
    changed.originalX_units{end} = changed.originalX_units{end}+10;
    rebuilt = obstacleAvoidance.obstacles.prepareObstacles(changed);
    verifyFalse(testCase,rebuilt.InternalPreparation.SamplesExactlyEqual);
    verifyEqual(testCase, obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
        changed, [0, 10], [0, 0], 10), [false, true]);
    % Equivalent geometry with a different starting vertex is not numeric identity.
    rotated = circshift(box,1,1);
    source = obstacleAvoidance.obstacles.createObstacle('reordered', [0; 2], ...
        {box(:, 1); rotated(:, 1)}, {box(:, 2); rotated(:, 2)}, 0);
    prepared = obstacleAvoidance.obstacles.prepareObstacles(source);
    verifyTrue(testCase,prepared.InternalPreparation.IsTimeInvariant);
    verifyFalse(testCase,prepared.InternalPreparation.SamplesExactlyEqual);
end

function geometryQueriesAndSourceRefresh(testCase, box_units)
    box = box_units;
    source = obstacleAvoidance.obstacles.createObstacle('moving',[0;2], {box(:,1);box(:,1)+2},{box(:,2);box(:,2)},0);
    prepared = obstacleAvoidance.obstacles.prepareObstacles(source);
    verifyEqual(testCase, obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
        prepared, [0, 1, 2], [0, 0, 0], [0, 1, 2]), [true, true, true]);
    % Repeated times must still evaluate each supplied point.
    verifyEqual(testCase,obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
        prepared,[2,0,1,2],[0,0,0,0],[2,0,1,2]),[true,true,true,true]);
    % Check new points using the requested boundary policy.
    verifyEqual(testCase,obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
        prepared,[1.5,3],[0,0],1,struct('BoundaryIsOccupied',false)),[false,false]);
    changed = prepared;
    changed.x_units = cellfun(@(x)x+10,changed.x_units,'UniformOutput',false);
    changed.originalX_units = cellfun(@(x)x+10,changed.originalX_units,'UniformOutput',false);
    verifyEqual(testCase,obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(changed,[0,10],[0,0],0),[false,true]);
    verifyFalse(testCase,obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(prepared,0.5,0,-1));
    % Changing the active interval must refresh an inactive boundary too.
    prepared.time_s = [-2;2];
    verifyTrue(testCase,obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(prepared,0.5,0,-1));
end

function occupancyBoundaryAndBlockingContract(testCase, box_units)
    box = box_units;
    fixed = obstacleAvoidance.obstacles.createObstacle('fixed',0,box(:,1),box(:,2),0);
    moving = obstacleAvoidance.obstacles.createObstacle('moving',[0;2], {box(:,1)+3;box(:,1)+5},{box(:,2);box(:,2)},0);
    obstacles = obstacleAvoidance.obstacles.combineObstacles({fixed,moving,moving});
    preparedObstacles = obstacleAvoidance.obstacles.prepareObstacles(obstacles);
    x_units = repmat([0,0.5,1,3,5],2,1);
    y_units = zeros(size(x_units));
    time_s = repmat([0;2],1,5);
    for boundaryOccupied = [false,true]
        expected = logical([1,boundaryOccupied,0,1,0;1,boundaryOccupied,0,0,1]);
        expectedBlocking = uint32([1,boundaryOccupied,0,2,0;1,boundaryOccupied,0,0,2]);
        [occupied,blocking] = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
            obstacles,x_units,y_units,time_s,struct('BoundaryIsOccupied',boundaryOccupied));
        verifyEqual(testCase,occupied,expected);
        verifyEqual(testCase,blocking,expectedBlocking);
        for repeat = 1:2
            [preparedOccupied,preparedBlocking] = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
                preparedObstacles,x_units,y_units,time_s,struct('BoundaryIsOccupied',boundaryOccupied));
            verifyEqual(testCase,preparedOccupied,occupied);
            verifyEqual(testCase,preparedBlocking,blocking);
        end
    end
    verifyEqual(testCase, obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
        obstacles, [3, 5], [0, 0], [-1, 3]), [false, false]);
    verifyEqual(testCase, obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
        obstacles, zeros(0, 2), zeros(0, 2), 0), false(0, 2));
    verifyError(testCase,@()obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
        obstacles,[0,1],[0,0],[0;1]),'queryObstacleOccupancyAtTime:SizeMismatch');
end

function boundaryOnlyGeometryPreservesPreparedModel(testCase)
    box = [-2,-2;2,-2;2,2;-2,2];
    concave = [0,0;3,0;3,1;1,1;1,3;0,3];
    hole = [box;NaN,NaN;-1,-1;-1,1;1,1;1,-1];
    for boundary = {box,concave,hole}
        vertices = boundary{1};
        obstacle = obstacleAvoidance.obstacles.createObstacle('boundary',[0;2], ...
            {vertices(:,1);vertices(:,1)+2},{vertices(:,2);vertices(:,2)+1},0);
        obstacle = obstacleAvoidance.obstacles.prepareObstacles(obstacle);
        for time_s = [-1,0,0.75,2,3]
            [shape,shapeDetails] = obstacleAvoidance.obstacles.preparedShapeAtTime(obstacle,time_s);
            verifyEqual(testCase,obstacleAvoidance.obstacles.preparedShapeAtTime(obstacle,time_s),shape);
            [~,boundaryOnly] = obstacleAvoidance.obstacles.preparedShapeAtTime(obstacle,time_s,true);
            verifyEqual(testCase,boundaryOnly,shapeDetails);
        end
    end
end

function marginOnce(testCase)
    obstacle = obstacleAvoidance.obstacles.createObstacle('box',0,[-1;1;1;-1],[-1;-1;1;1],0.2);
    rebuilt = obstacleAvoidance.obstacles.createObstacle(obstacle,0.2);
    verifyEqual(testCase,rebuilt.x_units,obstacle.x_units);
    verifyEqual(testCase,rebuilt.originalX_units,obstacle.originalX_units);
end

function translationAcrossStartsOrientationsAndScales(testCase)
    for count=[7,31,140,220]
        theta=(0:count-1)'*2*pi/count;
        radius=2+0.4*cos(3*theta)+0.2*sin(5*theta);
        ring=radius.*[cos(theta),sin(theta)];
        for scale=[1e-4,1,1e4]
            lower=scale*ring+[120,-70]; upper=lower+scale*[0.25,-0.1];
            for reverse=[false,true]
                reordered=circshift(upper,floor(count/3));
                if reverse
                    reordered = flipud(reordered);
                end
                prepared=preparePair(lower,reordered);
                verifyTrue(testCase,prepared.InternalPreparation.MatchingTopology);
                actual = lower + [prepared.InternalPreparation.DeltaX_units{1}, ...
                    prepared.InternalPreparation.DeltaY_units{1}];
                verifyEqual(testCase,actual,upper,'AbsTol',8*eps(max(abs(upper),[],'all')));
            end
        end
    end
end

function consecutiveTranslationsReuseExactPartition(testCase)
    base=[0,0;2,0;2,2;1,0.5;0,2];
    frames={base,base+[1,0],base+[2,0.5],base+[3,1]};
    deformed=base+[4,1.5];
    deformed(4,:)=deformed(4,:)+[-0.25,0.2];
    frames{5}=deformed;
    obstacle=obstacleAvoidance.obstacles.createObstacle('translation chain',(0:4).', ...
        cellfun(@(v)v(:,1),frames,'UniformOutput',false), cellfun(@(v)v(:,2),frames,'UniformOutput',false),0);
    prepared=obstacleAvoidance.obstacles.prepareObstacles(obstacle,[0,4]);
    preparation=prepared.InternalPreparation;
    for intervalIndex = 2:3
        verifyEqual(testCase, preparation.IntervalStartRegions_units{intervalIndex}, ...
            preparation.IntervalEndRegions_units{intervalIndex - 1});
    end
    for intervalIndex=1:4
        startShape=unionRegions(preparation.IntervalStartRegions_units{intervalIndex});
        endShape=unionRegions(preparation.IntervalEndRegions_units{intervalIndex});
        verifyLessThan(testCase,area(xor(startShape,polyshape(frames{intervalIndex}, 'Simplify',false))),1e-12);
        verifyLessThan(testCase,area(xor(endShape,polyshape(frames{intervalIndex+1}, 'Simplify',false))),1e-12);
    end
end

function deformingConvexAgainstExhaustiveReference(testCase)
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

function symmetricTiesAreStartingIndexInvariant(testCase)
    theta=(0:7)'*pi/4;
    lower=[cos(theta),sin(theta)]; upper=[cos(theta+pi/8),sin(theta+pi/8)];
    reference=preparePair(lower,upper);
    referenceShape=obstacleAvoidance.obstacles.preparedShapeAtTime(reference,0.3);
    for shift=0:7
        for reverse=[false,true]
            changed = circshift(upper, shift);
            if reverse
                changed = flipud(changed);
            end
            changedLower=circshift(lower,2*shift);
            if mod(shift, 2) == 0
                changedLower = flipud(changedLower);
            end
            prepared=preparePair(changedLower,changed);
            actual=obstacleAvoidance.obstacles.preparedShapeAtTime(prepared,0.3);
            verifyLessThan(testCase,area(xor(referenceShape,actual)),1e-12);
        end
    end
end

function concaveDeformationUsesExactMovingPartition(testCase)
    lower=[0,0;2,0;2,2;1,0.5;0,2]; upper=lower; upper(4,:)=[0.5,1];
    prepared=preparePair(lower,circshift(upper,2));
    verifyTrue(testCase,prepared.InternalPreparation.MatchingTopology);
    verifyTrue(testCase,prepared.InternalPreparation.IntervalHasExactPartition);
    verifyFalse(testCase,prepared.InternalPreparation.IntervalUsesMovingCells);
    verifyFalse(testCase,prepared.InternalPreparation.IntervalIsUnsupported);
    cells=obstacleAvoidance.obstacles.createTimeCells(prepared,0,1);
    verifyGreaterThan(testCase,numel(cells.Regions_units),1);
    for tau=[0,0.25,0.5,0.75,1]
        partitionShape=polyshape();
        for regionIndex=1:numel(cells.Regions_units)
            region = cells.Regions_units{regionIndex} + ...
                tau * (cells.EndRegions_units{regionIndex} - cells.Regions_units{regionIndex});
            partitionShape=union(partitionShape,polyshape(region,'Simplify',false));
        end
        exactBoundary=obstacleAvoidance.obstacles.preparedShapeAtTime(prepared,tau);
        verifyLessThan(testCase,area(xor(partitionShape,exactBoundary)),1e-12);
        verifyFalse(testCase,isinterior(partitionShape,1,1.75));
    end
end

function separatedCollinearEdgesRemainValidDuringNonaffineMotion(testCase)
    % Disjoint horizontal ledges are not a self-intersection, even when their
    % orientation polynomial is identically zero throughout a deformation.
    lower=[0,0;6,0;6,4;5,4;5,1;4,1;4,4;3,4;3,1;2,1;2,4;0,4];
    upper=lower;
    upper([5,6],2)=1.2;
    upper([9,10],2)=0.8;
    for scale=[1e-3,1,1e3]
        prepared=preparePair(scale*lower,scale*upper);
        verifyTrue(testCase,prepared.InternalPreparation.MatchingTopology);
        cells=obstacleAvoidance.obstacles.createTimeCells(prepared,0,1);
        for tau=[0,0.25,0.5,0.75,1]
            regions=cell(size(cells.Regions_units));
            for index=1:numel(regions)
                regions{index}=(1-tau)*cells.Regions_units{index}+ tau*cells.EndRegions_units{index};
            end
            expected=polyshape(scale*((1-tau)*lower+tau*upper),'Simplify',false);
            verifyLessThan(testCase,area(xor(unionRegions(regions),expected)), 1e-10*max(1,scale^2));
        end
    end
end

function redundantAffineKeyframesUseIdenticalCellsAndMotion(testCase)
    ring=[10,10;13,10;13,13;11,11;10,13];
    times_s=linspace(0,16,17).';
    frames=arrayfun(@(t)ring+[t/8,-t/16],times_s,'UniformOutput',false);
    dense=obstacleAvoidance.obstacles.createObstacle('affine',times_s, ...
        cellfun(@(v)v(:,1),frames,'UniformOutput',false),cellfun(@(v)v(:,2),frames,'UniformOutput',false));
    sparse = obstacleAvoidance.obstacles.createObstacle('affine', times_s([1, end]), ...
        dense.x_units([1, end]), dense.y_units([1, end]));
    prepared=obstacleAvoidance.obstacles.prepareObstacles(dense);
    verifyEqual(testCase,prepared.time_s,times_s);
    verifyEqual(testCase,obstacleAvoidance.obstacles.createTimeCells(prepared,0,16), ...
        obstacleAvoidance.obstacles.createTimeCells(sparse,0,16));
    % Query-window coverage must not manufacture different canonical spans.
    partial=obstacleAvoidance.obstacles.prepareObstacles(dense,[3,5]);
    verifyEqual(testCase,obstacleAvoidance.obstacles.createTimeCells(partial,3,5), ...
        obstacleAvoidance.obstacles.createTimeCells(sparse,3,5));
    initial=struct('time_s',0,'position_units',[0,0]);
    goal=struct('time_s',16,'position_units',[4,2]);
    limits=struct('xInterval_units',[-5,20],'yInterval_units',[-5,20], ...
        'maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[2,2],'maxJerk_units_s3',[4,4]);
    denseResult=planner(dense,initial,goal,limits,struct('GoalTimeMode','fixedArrival'));
    sparseResult=planner(sparse,initial,goal,limits,struct('GoalTimeMode','fixedArrival'));
    verifyTrue(testCase,denseResult.Success);
    verifyTrue(testCase,sparseResult.Success);
    verifyEqual(testCase,denseResult.ArrivalTime_s,sparseResult.ArrivalTime_s);
    verifyEqual(testCase,denseResult.MotionLength_units,sparseResult.MotionLength_units);
end

function properCrossingZigzagHasDeclaredRepair(testCase)
    ring=[0,0;2,2;0,2;2,0;4,0;4,4;-1,4;-1,0];
    obstacle = obstacleAvoidance.obstacles.createObstacle('fold', [0; 1], ...
        {ring(:, 1); ring(:, 1) + 0.25}, {ring(:, 2); ring(:, 2)}, 0);
    verifyEqual(testCase,cellfun(@numel,obstacle.x_units),[6;6]);
    expected = ring([1,4:8],:);
    for sampleIndex = 1:2
        expectedSample = expected + [(sampleIndex - 1) * 0.25,0];
        verifyEqual(testCase,[obstacle.x_units{sampleIndex},obstacle.y_units{sampleIndex}],expectedSample);
        verifyEqual(testCase, ...
            [obstacle.originalX_units{sampleIndex}, obstacle.originalY_units{sampleIndex}], expectedSample);
    end
    rebuilt=obstacleAvoidance.obstacles.createObstacle(obstacle);
    verifyEqual(testCase,rebuilt,obstacle);
    % A four-vertex bow tie leaves fewer than three vertices: remove the run.
    bow=obstacleAvoidance.obstacles.createObstacle('empty fold',0,[0;2;0;2],[0;2;2;0]);
    verifyEmpty(testCase,bow.x_units{1});
    verifyEmpty(testCase,bow.y_units{1});
end

function smallInteriorFoldKeepsTheSameVertices(testCase)
    % Crossing edges 3 and 5 remove vertices 4 and 5; keep the six others.
    ring_units = [-1, 4; -1, 0; 0, 0; 2, 2; 0, 2; 2, 0; 4, 0; 4, 4];
    expected_units = ring_units([1:3, 6:8], :);
    verifyCrossingRepair(testCase, ring_units, expected_units, 2);
end

function smallSeamFoldRemovesTheCyclicComplement(testCase)
    % Crossing edges 1 and 7 remove two seam vertices, preserving the main boundary.
    ring_units = [0, 2; 2, 0; 4, 0; 4, 4; -1, 4; -1, 0; 0, 0; 2, 2];
    expected_units = ring_units(2:7, :);
    verifyCrossingRepair(testCase, ring_units, expected_units, 2);
end

function polygonClosureRoundoffKeepsTheLargerLoop(testCase)
    % Near closure, edges 1 and 6 cross; keep the five-vertex larger loop.
    ring_units = [1, 1; 3, 1; 4, 2; 3, 3; 1, 3; 0, 2; 1 + 4 * eps(1), 1 - 2 * eps(1)];
    expected_units = ring_units(2:6, :);
    verifyCrossingRepair(testCase, ring_units, expected_units, 2);
end

function closingCopyRemovalRemainsExact(testCase)
    % Keep the two-ulp closing point; remove only the exactly equal copy.
    square_units = [1, 1; 3, 1; 3, 3; 1, 3];
    for closureOffset_units = [0, 2 * eps(1)]
        ring_units = [square_units; 1, 1 + closureOffset_units];
        obstacle = obstacleAvoidance.obstacles.createObstacle( ...
            'exact closure', 0, ring_units(:, 1), ring_units(:, 2), 0);
        expected_units = ring_units;
        duplicateCount = double(closureOffset_units == 0);
        if duplicateCount > 0
            expected_units(end, :) = [];
        end
        verifyEqual(testCase, [obstacle.x_units{1}, obstacle.y_units{1}], expected_units);
        verifyEqual(testCase, [obstacle.originalX_units{1}, obstacle.originalY_units{1}], expected_units);
    end
end

function repairedSourceIndexCountChangeReportsEndpointHulls(testCase)
    % Repair changes the middle count; unknown matches require an endpoint hull.
    folded_units = [0, 2; 2, 0; 4, 0; 4, 4; -1, 4; -1, 0; 0, 0; 2, 2];
    neighbor_units = [-1, 0; 0, 0; 2, 0; 4, 0; 4, 2; 4, 4; -1, 4];
    frames_units = {neighbor_units; folded_units + [0.1, 0]; neighbor_units + [0.2, 0]};
    obstacle = obstacleAvoidance.obstacles.createObstacle('repaired source indices', [0; 1; 2], ...
        cellfun(@(frame) frame(:, 1), frames_units, 'UniformOutput', false), ...
        cellfun(@(frame) frame(:, 2), frames_units, 'UniformOutput', false), ...
        0, struct('vertexCorrespondence', 'sourceIndex'));
    verifyEqual(testCase, cellfun(@numel, obstacle.x_units), [7; 6; 7]);
    verifyEqual(testCase, [obstacle.x_units{2}, obstacle.y_units{2}], folded_units(2:7, :) + [0.1, 0]);
    verifyEqual(testCase, [obstacle.originalX_units{2}, obstacle.originalY_units{2}], folded_units(2:7, :) + [0.1, 0]);
    prepared = obstacleAvoidance.obstacles.prepareObstacles(obstacle);
    preparation = prepared.InternalPreparation;
    verifyTrue(testCase, prepared.UsesSourceIndex);
    verifyEqual(testCase, prepared.vertexCorrespondence, "sourceIndex");
    verifyEqual(testCase, preparation.IntervalUsesEndpointHull, true(2, 1));
    verifyEqual(testCase, preparation.MatchingTopology, false(2, 1));
    verifyEqual(testCase, preparation.IntervalHasExactPartition, false(2, 1));
    verifyEqual(testCase, preparation.IntervalUsesMovingCells, false(2, 1));
    verifyEqual(testCase, preparation.IntervalIsStationary, false(2, 1));
    verifyEqual(testCase, preparation.IntervalIsUnsupported, false(2, 1));
    for intervalIndex = 1:2
        cells = obstacleAvoidance.obstacles.createTimeCells(prepared, intervalIndex - 1, intervalIndex);
        verifyEqual(testCase, cells.Regions_units, cells.EndRegions_units);
        enclosure = unionRegions(cells.Regions_units);
        for sampleIndex = intervalIndex:intervalIndex + 1
            verifyEqual(testCase, area(subtract(preparation.SampleShapes{sampleIndex}, enclosure)), 0);
        end
        lower_units = [obstacle.x_units{intervalIndex}, obstacle.y_units{intervalIndex}];
        upper_units = [obstacle.x_units{intervalIndex + 1}, obstacle.y_units{intervalIndex + 1}];
        for vertexIndex = 1:size(lower_units, 1)
            midpoint_units = (lower_units(vertexIndex, :) + upper_units) / 2;
            verifyTrue(testCase, all(obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
                prepared, midpoint_units(:, 1), midpoint_units(:, 2), intervalIndex - 0.5)));
        end
    end
end

function containmentClassificationMatchesBooleanReference(testCase)
    % Unequal counts bypass matching; check both orders, including holes and splits.
    lower = [0, 0; 4, 0; 4, 1; 1, 1; 1, 4; 0, 4];
    candidates = {0.8 * lower + [0.1, 0.1], 1.3 * lower - [0.1, 0.1], lower + [0.1, 0.2], lower + [8, 0], ...
        [lower; NaN, NaN; 8, 0; 9, 0; 9, 1; 8, 1], [-1, -1; 6, -1; 6, 6; -1, 6; NaN, NaN; 2, 2; 2, 3; 3, 3; 3, 2]};
    for index = 1:numel(candidates)
        upper = candidates{index};
        upper = [upper(1, :); mean(upper(1:2, :), 1); upper(2:end, :)];
        for reversed = [false, true]
            first = lower;
            last  = upper;
            if reversed
                first = upper;
                last  = lower;
            end
            prepared   = preparePair(first, last);
            firstShape = polyshape(first, 'Simplify', false);
            lastShape  = polyshape(last, 'Simplify', false);
            tolerance  = 512 * eps(max([1, area(firstShape), area(lastShape)]));
            equivalent = area(xor(firstShape, lastShape)) <= tolerance;
            preparation = prepared.InternalPreparation;
            verifyEqual(testCase, preparation.IntervalIsStationary, equivalent);
            verifyEqual(testCase, preparation.IntervalUsesEndpointHull, ~equivalent);
            verifyFalse(testCase, preparation.IntervalIsUnsupported);
            verifyFalse(testCase, preparation.MatchingTopology);
            verifyFalse(testCase, preparation.IntervalHasExactPartition);
            verifyFalse(testCase, preparation.IntervalUsesMovingCells);
        end
    end
end

function changingVertexCountPreservesTimingAndPlansValidMotion(testCase)
    lower = [0, 0; 2, 0; 2, 2; 0, 2];
    upper = [0, 0; 2, 0; 2, 2; 1, 3; 0, 2];
    obstacle = obstacleAvoidance.obstacles.createObstacle('changing count', [0; 1], ...
        {lower(:, 1); upper(:, 1)}, {lower(:, 2); upper(:, 2)}, 0);
    initial = struct('time_s', 0, 'position_units', [-2, 0], ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    goal = struct('time_s', 1, 'position_units', [4, 0], ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    limits = struct( ...
        'xInterval_units', [-5, 5], ...
        'yInterval_units', [-5, 5], ...
        'maxVelocity_units_s', [10, 10], ...
        'maxAcceleration_units_s2', [10, 10], ...
        'maxJerk_units_s3', [20, 20]);
    options = struct('GoalTimeMode', 'fixedArrival');
    % At rest, acceleration alone limits travel to a*T^2/4 in one second.
    verifyLessThan(testCase, limits.maxAcceleration_units_s2(1) * goal.time_s^2 / 4, ...
        goal.position_units(1) - initial.position_units(1));
    tooShort = planner(obstacle, initial, goal, limits, options);
    verifyFalse(testCase, tooShort.Success);
    verifyEqual(testCase, tooShort.TerminationReason, "timeWindowInfeasible");

    % Ten seconds permits a detour with the same geometry and physical limits.
    goal.time_s        = 10;
    obstacle.time_s(2) = goal.time_s;
    result     = planner(obstacle, initial, goal, limits, options);
    validation = obstacleAvoidance.validateTrajectory(result);
    verifyTrue(testCase, result.Success, result.Message);
    verifyEqual(testCase, result.TerminationReason, "goalReached");
    verifyTrue(testCase, validation.Passed, validation.Message);
    verifyTrue(testCase, result.Diagnostics.PreparedObstacles.InternalPreparation.IntervalUsesEndpointHull);
end

function cachedEndpointHullPlansEarlierAndOverlappingHorizons(testCase)
    first  = [10, 10; 12, 10; 12, 12; 10, 12];
    second = first + [0.2, 0];
    third  = [10.4, 10; 12.4, 10; 12.4, 12; 11.4, 13; 10.4, 12];
    source = obstacleAvoidance.obstacles.createObstacle('later endpoint hull', [0; 1; 2], ...
        {first(:, 1); second(:, 1); third(:, 1)}, {first(:, 2); second(:, 2); third(:, 2)}, 0);
    prepared = obstacleAvoidance.obstacles.prepareObstacles(source, [0, 2]);
    initial  = struct('time_s', 0, 'position_units', [-2, 0]);
    goal     = struct('time_s', 0.5, 'position_units', [-1, 0]);
    limits = struct( ...
        'xInterval_units', [-5, 15], ...
        'yInterval_units', [-5, 15], ...
        'maxVelocity_units_s', [10, 10], ...
        'maxAcceleration_units_s2', [100, 100], ...
        'maxJerk_units_s3', [1000, 1000]);
    options = struct('GoalTimeMode', 'fixedArrival');
    early   = planner(prepared, initial, goal, limits, options);
    verifyNotEqual(testCase, early.TerminationReason, "unsupportedObstacleInterpolation");
    verifyTrue(testCase, early.Success, early.Message);
    earlyValidation = obstacleAvoidance.validateTrajectory(early);
    verifyTrue(testCase, earlyValidation.Passed, earlyValidation.Message);
    goal.time_s = 1.5;
    overlapping = planner(prepared, initial, goal, limits, options);
    verifyTrue(testCase, overlapping.Success, overlapping.Message);
    verifyEqual(testCase, overlapping.TerminationReason, "goalReached");
    overlapValidation = obstacleAvoidance.validateTrajectory(overlapping);
    verifyTrue(testCase, overlapValidation.Passed, overlapValidation.Message);
end

function movingCellsContainUnprovableCorrespondingRing(testCase)
    [lower, upper] = thinNotchFixture();
    for margin_units=[0,0.1]
        source=obstacleAvoidance.obstacles.createObstacle('thin notch',[0;1], ...
            {lower(:,1);upper(:,1)},{lower(:,2);upper(:,2)},margin_units);
        prepared=obstacleAvoidance.obstacles.prepareObstacles(source);
        preparation=prepared.InternalPreparation;
        if margin_units==0
            verifyFalse(testCase,preparation.MatchingTopology);
            verifyTrue(testCase,preparation.IntervalUsesMovingCells);
            verifyFalse(testCase,preparation.IntervalIsUnsupported);
            cells=obstacleAvoidance.obstacles.createTimeCells(prepared,0,1);
            verifyEqual(testCase,cells.Regions_units,cells.EndRegions_units);
            enclosure=unionRegions(cells.Regions_units);
        else
            % Protection may remove the notch; check cell-level margins separately.
            [supported,enclosure]=obstacleAvoidance.obstacles.createMovingCells(lower,upper,margin_units);
            verifyTrue(testCase,supported);
        end
        aligned=obstacleAvoidance.obstacles.alignCorrespondingRing(lower,upper);
        for tau=[0,0.25,0.5,0.75,1]
            polygon=polyshape((1-tau)*lower+tau*aligned,'Simplify',false);
            if margin_units > 0
                polygon = polybuffer(polygon, margin_units, 'JointType', 'square');
            end
            verifyLessThan(testCase,area(subtract(polygon,enclosure)),1e-11);
            queried=obstacleAvoidance.obstacles.preparedShapeAtTime(prepared,tau);
            if tau>0 && tau<1 && margin_units==0
                verifyEqual(testCase,area(xor(queried,enclosure)),0);
            elseif tau==0 || tau==1
                verifyEqual(testCase,area(xor(queried,preparation.SampleShapes{1+tau})),0);
            end
        end
    end
end

function appearingObstacleBlocksStraightMotion(testCase)
    square_units = [-1, -1; 1, -1; 1, 1; -1, 1];
    obstacle = obstacleAvoidance.obstacles.createObstacle( ...
        'appearing square', [0; 10], {zeros(0, 1); square_units(:, 1)}, {zeros(0, 1); square_units(:, 2)}, 0);
    prepared    = obstacleAvoidance.obstacles.prepareObstacles(obstacle);
    preparation = prepared.InternalPreparation;

    verifyEndpointHullModel(testCase, preparation);
    verifyTrue(testCase, obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(prepared, 0, 0, 5));

    [initialState, goalState, limits, options] = planningRequest([-3, 0], [3, 0], 0, 10, [-5, 5], [-5, 5]);
    result     = planner(obstacle, initialState, goalState, limits, options);
    validation = obstacleAvoidance.validateTrajectory(result);

    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, validation.Passed, validation.Message);
    verifyGreaterThan(testCase, max(abs(result.position_units(:, 2))), 1);
end

function ringSplitOccupiesFragmentsAndGap(testCase)
    leftLower_units  = [-2, -1; -1, -1; -1, 1; -2, 1];
    leftUpper_units  = leftLower_units + [0.1, 0];
    rightUpper_units = [1, -1; 2, -1; 2, 1; 1, 1];
    upper_units = [leftUpper_units; NaN, NaN; rightUpper_units];
    obstacle = obstacleAvoidance.obstacles.createObstacle('separated fragments', [0; 10], ...
        {leftLower_units(:, 1); upper_units(:, 1)}, {leftLower_units(:, 2); upper_units(:, 2)}, 0);
    prepared    = obstacleAvoidance.obstacles.prepareObstacles(obstacle);
    preparation = prepared.InternalPreparation;

    verifyEndpointHullModel(testCase, preparation);
    verifyEqual(testCase, numel(preparation.IntervalStartRegions_units{1}), 1);
    verifyEqual(testCase, obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
        prepared, [-1.5, 1.5, 0], [0, 0, 0], 5), [true, true, true]);

    [initialState, goalState, limits, options] = planningRequest([0, -3], [0, 3], 0, 10, [-4, 4], [-4, 4]);
    result     = planner(obstacle, initialState, goalState, limits, options);
    validation = obstacleAvoidance.validateTrajectory(result);

    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, validation.Passed, validation.Message);
    % The hull spans x=[-2,2], so a valid crossing must detour past an end.
    verifyGreaterThan(testCase, max(abs(result.position_units(:, 1))), 2);
    endpointUnion = union(preparation.SampleShapes{1}, preparation.SampleShapes{2});
    addedArea_units2 = area(subtract(preparation.IntervalUnionShapes{1}, endpointUnion));
    verifyGreaterThan(testCase, addedArea_units2, 0);
end

function twoDeformingRingsAndAddedArea(testCase)
    lowerLeft_units  = [-4, -1; -2, -1; -2, 1; -4, 1];
    lowerRight_units = [2, -1; 4, -1; 4, 1; 2, 1];
    upperLeft_units  = [-4.2, -0.7; -2.1, -1.3; -1.8, 0.8; -3.8, 1.2];
    upperRight_units = [2.1, -1.2; 4.3, -0.8; 3.8, 1.3; 1.9, 0.7];
    lower_units = [lowerLeft_units; NaN, NaN; lowerRight_units];
    upper_units = [upperLeft_units; NaN, NaN; upperRight_units];
    obstacle = obstacleAvoidance.obstacles.createObstacle('two deforming rings', [0; 4], ...
        {lower_units(:, 1); upper_units(:, 1)}, {lower_units(:, 2); upper_units(:, 2)}, 0);
    prepared    = obstacleAvoidance.obstacles.prepareObstacles(obstacle);
    preparation = prepared.InternalPreparation;

    verifyEndpointHullModel(testCase, preparation);
    regions_units = preparation.IntervalStartRegions_units{1};
    verifyEqual(testCase, numel(regions_units), 1);
    verifyTrue(testCase, groupIsCovered(regions_units, [lowerLeft_units; upperLeft_units]));
    verifyTrue(testCase, groupIsCovered(regions_units, [lowerRight_units; upperRight_units]));

    endpointUnion = union(preparation.SampleShapes{1}, preparation.SampleShapes{2});
    addedArea_units2 = area(subtract(preparation.IntervalUnionShapes{1}, endpointUnion));
    verifyTrue(testCase, isfinite(addedArea_units2));
    verifyGreaterThanOrEqual(testCase, addedArea_units2, 0);
    cells = obstacleAvoidance.obstacles.createTimeCells(prepared, 0, 4);
    verifyEqual(testCase, cells.Regions_units, cells.EndRegions_units);
end

function largeNonoverlappingJumpCoversCrossSampleSegments(testCase)
    square_units = [-1, -1; 1, -1; 1, 1; -1, 1];
    lower_units  = [square_units + [-10, 0]; NaN, NaN; square_units + [-5, 0]];
    upper_units  = [square_units + [5, 0]; NaN, NaN; square_units + [12, 0]];
    verifyCrossSampleSegments(testCase, lower_units, upper_units);
end

function swappedRingIdentitiesCoverCrossSampleSegments(testCase)
    square_units = [-1, -1; 1, -1; 1, 1; -1, 1];
    lower_units  = [square_units + [-5, 0]; NaN, NaN; square_units + [5, 0]];
    % Supplied ring identities swap sides. The offset keeps the samples
    % geometrically non-equivalent so this exercises the enclosure model.
    upper_units  = [square_units + [5, 0.5]; NaN, NaN; square_units + [-5, 0.5]];
    verifyCrossSampleSegments(testCase, lower_units, upper_units);
end

function degenerateEndpointRemainsUnsupported(testCase)
    lower_units = [0, 0; 1, 0; 2, 0];
    upper_units = [0, 0; 2, 0; 2, 2; 0, 2];
    obstacle = obstacleAvoidance.obstacles.createObstacle('degenerate endpoint', [0; 1], ...
        {lower_units(:, 1); upper_units(:, 1)}, {lower_units(:, 2); upper_units(:, 2)}, 0);
    prepared = obstacleAvoidance.obstacles.prepareObstacles(obstacle);
    verifyTrue(testCase, prepared.InternalPreparation.IntervalIsUnsupported);
    verifyFalse(testCase, prepared.InternalPreparation.IntervalUsesEndpointHull);
end

function subEpsilonVelocityDriftIsNotMergedAway(testCase)
    % A 1e-14 units/s velocity change accumulates to a 0.005-unit middle-sample
    % error if merged over 1e12 s. Preserve the supplied sample.
    square_units = [0 0; 1 0; 1 1; 0 1];
    time_s       = [0; 1e12; 2e12];
    xByTime      = {square_units(:, 1); square_units(:, 1); square_units(:, 1) + 0.01};
    yByTime      = repmat({square_units(:, 2)}, 3, 1);
    obstacle = obstacleAvoidance.obstacles.createObstacle('drifting square', time_s, xByTime, yByTime, 0);
    prepared = obstacleAvoidance.obstacles.prepareObstacles(obstacle, [0 2e12]);
    verifyEqual(testCase, prepared.InternalPreparation.SpanEndSampleIndex, [2; 3]);

    cells  = obstacleAvoidance.obstacles.createTimeCells(prepared, 1e12, 2e12);
    region = cells.Regions_units{1};
    verifyTrue(testCase, inpolygon(0.002, 0.5, region(:, 1), region(:, 2)));

    affineX  = {square_units(:, 1); square_units(:, 1) + 0.005; square_units(:, 1) + 0.01};
    affine   = obstacleAvoidance.obstacles.createObstacle('affine square', time_s, affineX, yByTime, 0);
    prepared = obstacleAvoidance.obstacles.prepareObstacles(affine, [0 2e12]);
    verifyEqual(testCase, prepared.InternalPreparation.SpanEndSampleIndex, [3; 3]);
end

function movingCellsDoNotBecomeAnExactTranslationPartition(testCase, lower_units, upper_units)
    first = lower_units;
    second = upper_units;
    third=second+[1,0];
    source=obstacleAvoidance.obstacles.createObstacle('mixed models',[0;1;2], ...
        {first(:,1);second(:,1);third(:,1)},{first(:,2);second(:,2);third(:,2)},0);
    prepared=obstacleAvoidance.obstacles.prepareObstacles(source);
    preparation = prepared.InternalPreparation;
    verifyEqual(testCase, preparation.IntervalUsesMovingCells, [true; false]);
    verifyEqual(testCase, preparation.MatchingTopology, [false; true]);
    verifyEqual(testCase, preparation.IntervalHasExactPartition, [false; true]);
    actual=unionRegions(prepared.InternalPreparation.IntervalStartRegions_units{2});
    verifyLessThan(testCase,area(xor(actual,polyshape(second,'Simplify',false))),1e-12);
    endShape = unionRegions(preparation.IntervalEndRegions_units{2});
    verifyLessThan(testCase, area(xor(endShape, polyshape(third, 'Simplify', false))), 1e-12);
end

function rootTwoMarginSquaresContainSquareJoinProtection(testCase)
    % Square joins reach d*sqrt(2) from the source. This rotated square proves
    % half-width d fails while d*sqrt(2) contains the protected shape.
    angle_rad=pi/8+(0:3)'*pi/2;
    vertices_units=[cos(angle_rad),sin(angle_rad)];
    protected=polybuffer(polyshape(vertices_units),0.1,'JointType','square');
    for halfWidth_units=[0.1,0.1*sqrt(2)]
        corners_units=halfWidth_units*[-1,-1;-1,1;1,-1;1,1];
        expanded_units=reshape(permute(vertices_units+permute(corners_units,[3,2,1]),[1,3,2]),[],2);
        hull=convhull(expanded_units);
        enclosure=polyshape(expanded_units(hull,:));
        uncovered_units2=area(subtract(protected,enclosure));
        if halfWidth_units<0.1*sqrt(2)
            verifyGreaterThan(testCase,uncovered_units2,0.0006);
        else
            verifyLessThanOrEqual(testCase,uncovered_units2,1e-12);
        end
    end
    % Endpoint containment also holds when this protected ring lacks an affine proof.
    lower=[0,0;4,0;4,4;2,4;2,4-1e-13;1,4;0,4];
    upper=lower; upper(5,1)=2.2; upper(2,1)=4.2;
    source=obstacleAvoidance.obstacles.createObstacle('square-join certificate',[0;1], ...
        {lower(:,1);upper(:,1)},{lower(:,2);upper(:,2)},0.1);
    prepared=obstacleAvoidance.obstacles.prepareObstacles(source);
    preparation=prepared.InternalPreparation;
    verifyFalse(testCase,preparation.IntervalIsUnsupported);
    if preparation.IntervalUsesMovingCells
        enclosure = preparation.IntervalUnionShapes{1};
        uncoveredArea_units2 = cellfun(@(shape) area(subtract(shape, enclosure)), preparation.SampleShapes);
        verifyLessThanOrEqual(testCase,max(uncoveredArea_units2), 4096*eps(max(1,area(preparation.SampleShapes{1}))));
        enclosure=unionRegions(preparation.IntervalStartRegions_units{1});
        for tau=[0,0.5,1]
            polygon=polybuffer(polyshape((1-tau)*lower+tau*upper,'Simplify',false),0.1,'JointType','square');
            verifyLessThan(testCase,area(subtract(polygon,enclosure)),1e-11);
        end
    end
    verifyEqual(testCase,prepared.x_units,source.x_units);
    verifyEqual(testCase,prepared.y_units,source.y_units);
end

function movingObstacleDeclaresSourceIndexCorrespondence(testCase)
    % A cyclic shift can undo this six-degree rotation during matching. The
    % constructor declares source indices, so cells must contain their interpolation.
    angle_rad=(0:359).'*(pi/180);
    radius_units=1+0.1*cos(3*angle_rad);
    source_units=[radius_units.*cos(angle_rad),radius_units.*sin(angle_rad)];
    rotate=@(position_units,time_s,~) position_units*[cosd(6*time_s),sind(6*time_s);-sind(6*time_s),cosd(6*time_s)];
    obstacle=obstacleAvoidance.obstacles.createMovingObstacle('rotating lobe',[0;1;2], ...
        source_units(:,1),source_units(:,2),rotate,0.05);
    verifyEqual(testCase,string(obstacle.vertexCorrespondence),"sourceIndex");
    verifyTrue(testCase,obstacle.UsesSourceIndex);
    generic=obstacleAvoidance.obstacles.createObstacle('generic copy',obstacle.time_s, ...
        obstacle.originalX_units,obstacle.originalY_units,0.05);
    verifyEqual(testCase,string(generic.vertexCorrespondence),"circularCorrelation");
    lower=[obstacle.originalX_units{1},obstacle.originalY_units{1}];
    upper=[obstacle.originalX_units{2},obstacle.originalY_units{2}];
    verifyNotEqual(testCase,obstacleAvoidance.obstacles.alignCorrespondingRing(lower,upper),upper);
    prepared=obstacleAvoidance.obstacles.prepareObstacles(obstacle,[0,1]);
    preparation=prepared.InternalPreparation;
    verifyTrue(testCase,prepared.UsesSourceIndex);
    verifyTrue(testCase,preparation.IntervalPrepared(1));
    % Buffered rings lose index order; build the enclosure from original rings.
    verifyTrue(testCase,preparation.IntervalUsesMovingCells(1));
    enclosure=obstacleAvoidance.obstacles.preparedShapeAtTime(prepared,0.5);
    for tau=[0.25,0.5,0.75]
        polygon=polybuffer(polyshape((1-tau)*lower+tau*upper,'Simplify',false),0.05,'JointType','square');
        verifyLessThan(testCase,area(subtract(polygon,enclosure)),1e-10);
    end
end

function reportedModelMutationDoesNotSelectBehavior(testCase, lower_units, upper_units)
    % Prove that retained provenance labels cannot select geometry behavior.
    source = obstacleAvoidance.obstacles.createObstacle('model mutation', [0; 1], ...
        {lower_units(:, 1); upper_units(:, 1)}, {lower_units(:, 2); upper_units(:, 2)}, 0);
    prepared = obstacleAvoidance.obstacles.prepareObstacles(source);
    verifyTrue(testCase, prepared.InternalPreparation.IntervalUsesMovingCells);

    mutated = prepared;
    mutated.vertexCorrespondence = "nonsense";

    referenceCells = obstacleAvoidance.obstacles.createTimeCells(prepared, 0, 1);
    mutatedCells   = obstacleAvoidance.obstacles.createTimeCells(mutated, 0, 1);
    verifyEqual(testCase, mutatedCells, referenceCells);

    [referenceShape, referenceGeometry] = obstacleAvoidance.obstacles.preparedShapeAtTime(prepared, 0.5);
    [mutatedShape, mutatedGeometry] = obstacleAvoidance.obstacles.preparedShapeAtTime(mutated, 0.5);
    verifyEqual(testCase, area(xor(mutatedShape, referenceShape)), 0);
    verifyEqual(testCase, mutatedGeometry, referenceGeometry);

    nodes_units = [-1, 2; 5, 2; -1, 5; 5, 5];
    edgeCost_units = hypot(nodes_units(:, 1) - nodes_units(:, 1).', nodes_units(:, 2) - nodes_units(:, 2).');
    initialState  = struct('time_s', 0, 'position_units', nodes_units(1, :), ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    goalState     = struct('time_s', 1, 'position_units', nodes_units(2, :), ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    % Keep acceleration and jerk ramps below the search clock precision.
    limits        = struct('maxVelocity_units_s', [20, 20], ...
        'maxAcceleration_units_s2', [1e30, 1e30], 'maxJerk_units_s3', [1e60, 1e60]);
    options       = struct('GoalTimeMode', "fixedArrival");
    sampleTimes_s = (0:0.1:1).';
    [referenceRoute_units, referenceRouteTime_s, referenceRecord] = ...
        obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodes_units, edgeCost_units, prepared, initialState, goalState, limits, sampleTimes_s, options);
    [mutatedRoute_units, mutatedRouteTime_s, mutatedRecord] = obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodes_units, edgeCost_units, mutated, initialState, goalState, limits, sampleTimes_s, options);
    verifyEqual(testCase, mutatedRoute_units, referenceRoute_units);
    verifyEqual(testCase, mutatedRouteTime_s, referenceRouteTime_s);
    verifyEqual(testCase, mutatedRecord, referenceRecord);
    verifyGreaterThan(testCase, referenceRecord.RejectedTransitionCount, 0);
    verifyTrue(testCase, any(all(referenceRoute_units == nodes_units(3, :), 2)));
    verifyTrue(testCase, any(all(referenceRoute_units == nodes_units(4, :), 2)));
end

function verifyCrossingRepair(testCase, ring_units, expected_units, removedVertexCount)
    % Verify the exact retained vertices and removed count in both histories.
    obstacle = obstacleAvoidance.obstacles.createObstacle('crossing fold', 0, ring_units(:, 1), ring_units(:, 2), 0);
    verifyEqual(testCase, [obstacle.x_units{1}, obstacle.y_units{1}], expected_units);
    verifyEqual(testCase, [obstacle.originalX_units{1}, obstacle.originalY_units{1}], expected_units);
    verifyEqual(testCase, size(ring_units, 1) - numel(obstacle.x_units{1}), removedVertexCount);
    verifyEqual(testCase, size(ring_units, 1) - numel(obstacle.originalX_units{1}), removedVertexCount);
    rebuilt = obstacleAvoidance.obstacles.createObstacle(obstacle);
    verifyEqual(testCase, rebuilt, obstacle);
end

function prepared=preparePair(lower,upper)
    source = obstacleAvoidance.obstacles.createObstacle('generic', [0; 1], ...
        {lower(:, 1); upper(:, 1)}, {lower(:, 2); upper(:, 2)}, 0);
    prepared=obstacleAvoidance.obstacles.prepareObstacles(source);
end

function [lower_units, upper_units] = thinNotchFixture()
    % Return the shared unprovable corresponding-ring regression fixture.
    lower_units = [0, 0; 4, 0; 4, 4; 2, 4; 2, 4 - 1e-13; 1, 4; 0, 4];
    upper_units            = lower_units;
    upper_units(5, 1)      = 2.2;
    upper_units([2, 3], 1) = 4.2;
end

function shape=unionRegions(regions_units)
    shape=polyshape();
    for regionIndex=1:numel(regions_units)
        shape=union(shape,polyshape(regions_units{regionIndex},'Simplify',false));
    end
end

function best=exhaustiveAlignment(lower,upper)
    best=[]; bestCost=Inf;
    for reverse=[false,true]
        oriented = upper;
        if reverse
            oriented = flipud(oriented);
        end
        for shift=0:size(lower,1)-1
            candidate=circshift(oriented,shift);
            cost=sum((candidate-lower).^2,'all');
            if cost < bestCost
                bestCost = cost;
                best = candidate;
            end
        end
    end
end

function verifyCrossSampleSegments(testCase, lower_units, upper_units)
    % Every lower-to-upper pair is admissible without declared ring identity.
    obstacle = obstacleAvoidance.obstacles.createObstacle('unknown ring identity', [0; 4], ...
        {lower_units(:, 1); upper_units(:, 1)}, {lower_units(:, 2); upper_units(:, 2)}, 0);
    prepared    = obstacleAvoidance.obstacles.prepareObstacles(obstacle);
    preparation = prepared.InternalPreparation;
    verifyEndpointHullModel(testCase, preparation);
    verifyEqual(testCase, numel(preparation.IntervalStartRegions_units{1}), 1);
    lowerFinite_units = lower_units(all(isfinite(lower_units), 2), :);
    upperFinite_units = upper_units(all(isfinite(upper_units), 2), :);
    for lowerIndex = 1:size(lowerFinite_units, 1)
        midpoint_units = (lowerFinite_units(lowerIndex, :) + upperFinite_units) / 2;
        verifyTrue(testCase, all(obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
            prepared, midpoint_units(:, 1), midpoint_units(:, 2), 2)));
        verifyTrue(testCase, groupIsCovered(preparation.IntervalStartRegions_units{1}, midpoint_units));
    end
    cells = obstacleAvoidance.obstacles.createTimeCells(prepared, 0, 4);
    verifyEqual(testCase, cells.Regions_units, preparation.IntervalStartRegions_units{1});
    verifyEqual(testCase, cells.Regions_units, cells.EndRegions_units);
end

function verifyEndpointHullModel(testCase, preparation)
    % Verify the typed interval classification without using its report name.
    verifyTrue(testCase, preparation.IntervalUsesEndpointHull(1));
    verifyFalse(testCase, preparation.MatchingTopology(1));
    verifyFalse(testCase, preparation.IntervalHasExactPartition(1));
    verifyFalse(testCase, preparation.IntervalUsesMovingCells(1));
    verifyFalse(testCase, preparation.IntervalIsStationary(1));
    verifyFalse(testCase, preparation.IntervalIsUnsupported(1));
end

function covered = groupIsCovered(regions_units, vertices_units)
    % All queried vertices must fit wholly inside the returned convex hull.
    covered = false;
    for regionIndex = 1:numel(regions_units)
        region_units = regions_units{regionIndex};
        [inside, onBoundary] = inpolygon( ...
            vertices_units(:, 1), vertices_units(:, 2), region_units(:, 1), region_units(:, 2));
        if all(inside | onBoundary)
            covered = true;
            return;
        end
    end
end

function [initialState, goalState, limits, options] = planningRequest( ...
        start_units, goal_units, initialTime_s, goalTime_s, xInterval_units, yInterval_units)
    % Build one deterministic fixed-arrival request for focused regressions.
    initialState = struct('time_s', initialTime_s, 'position_units', start_units);
    goalState    = struct('time_s', goalTime_s, 'position_units', goal_units);
    limits = struct( ...
        'xInterval_units', xInterval_units, ...
        'yInterval_units', yInterval_units, ...
        'maxVelocity_units_s', [2, 2], ...
        'maxAcceleration_units_s2', [2, 2], ...
        'maxJerk_units_s3', [4, 4]);
    options = struct('GoalTimeMode', 'fixedArrival');
end
