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

function testConsecutiveTranslationsReuseExactPartition(testCase)
    base=[0,0;2,0;2,2;1,0.5;0,2];
    frames={base,base+[1,0],base+[2,0.5],base+[3,1]};
    deformed=base+[4,1.5];
    deformed(4,:)=deformed(4,:)+[-0.25,0.2];
    frames{5}=deformed;
    obstacle=obstacleAvoidance.obstacles.createObstacle('translation chain',(0:4).', ...
        cellfun(@(v)v(:,1),frames,'UniformOutput',false), ...
        cellfun(@(v)v(:,2),frames,'UniformOutput',false),0);
    prepared=obstacleAvoidance.obstacles.prepareObstacles(obstacle,[0,4]);
    preparation=prepared.InternalPreparation;
    verifyEqual(testCase,preparation.IntervalPartitionReused, ...
        [false;true;true;false]);
    for intervalIndex=1:4
        startShape=unionRegions(preparation.IntervalStartRegions_units{intervalIndex});
        endShape=unionRegions(preparation.IntervalEndRegions_units{intervalIndex});
        verifyLessThan(testCase,area(xor(startShape,polyshape(frames{intervalIndex}, ...
            'Simplify',false))),1e-12);
        verifyLessThan(testCase,area(xor(endShape,polyshape(frames{intervalIndex+1}, ...
            'Simplify',false))),1e-12);
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
    verifyTrue(testCase,prepared.InternalPreparation.IntervalHasExactPartition);
    verifyFalse(testCase,prepared.InternalPreparation.IntervalUsesMovingCells);
    verifyFalse(testCase,prepared.InternalPreparation.IntervalIsUnsupported);
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

function testSeparatedCollinearEdgesRemainValidDuringNonaffineMotion(testCase)
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
                regions{index}=(1-tau)*cells.Regions_units{index}+ ...
                    tau*cells.EndRegions_units{index};
            end
            expected=polyshape(scale*((1-tau)*lower+tau*upper),'Simplify',false);
            verifyLessThan(testCase,area(xor(unionRegions(regions),expected)), ...
                1e-10*max(1,scale^2));
        end
    end
end

function testContainmentClassificationMatchesBooleanReference(testCase)
    % Unequal ring lengths bypass correspondence. Cover shrinking, growing,
    % equal-area shifted, disconnected and holed shapes, in both orders.
    lower = [0, 0; 4, 0; 4, 1; 1, 1; 1, 4; 0, 4];
    candidates = {0.8 * lower + [0.1, 0.1], 1.3 * lower - [0.1, 0.1], ...
        lower + [0.1, 0.2], lower + [8, 0], ...
        [lower; NaN, NaN; 8, 0; 9, 0; 9, 1; 8, 1], ...
        [-1, -1; 6, -1; 6, 6; -1, 6; NaN, NaN; 2, 2; 2, 3; 3, 3; 3, 2]};
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
            if equivalent
                expected = "staticEquivalentSamples";
            else
                expected = "endpointConvexHull";
            end
            preparation = prepared.InternalPreparation;
            verifyEqual(testCase, preparation.IntervalGeometryModel, expected);
            verifyEqual(testCase, preparation.IntervalIsStationary, equivalent);
            verifyEqual(testCase, preparation.IntervalUsesEndpointHull, ~equivalent);
            verifyFalse(testCase, preparation.IntervalIsUnsupported);
            verifyFalse(testCase, preparation.MatchingTopology);
            verifyFalse(testCase, preparation.IntervalHasExactPartition);
            verifyFalse(testCase, preparation.IntervalUsesMovingCells);
        end
    end
end

function testChangingVertexCountPreservesTimingAndPlansValidMotion(testCase)
    lower = [0, 0; 2, 0; 2, 2; 0, 2];
    upper = [0, 0; 2, 0; 2, 2; 1, 3; 0, 2];
    obstacle = obstacleAvoidance.obstacles.createObstacle('changing count', [0; 1], ...
        {lower(:, 1); upper(:, 1)}, {lower(:, 2); upper(:, 2)}, 0);
    initial = struct('time_s', 0, 'position_units', [-2, 0], ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    goal = struct('time_s', 1, 'position_units', [4, 0], ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    limits = struct( ...
        'xInterval_units',          [-5, 5], ...
        'yInterval_units',          [-5, 5], ...
        'maxVelocity_units_s',      [10, 10], ...
        'maxAcceleration_units_s2', [10, 10], ...
        'maxJerk_units_s3',         [20, 20]);
    options = struct('GoalTimeMode', 'fixedArrival');
    % The original one-second request is kinematically infeasible even with
    % no obstacle: rest-to-rest acceleration alone bounds travel by a*T^2/4.
    verifyLessThan(testCase, limits.maxAcceleration_units_s2(1) * goal.time_s^2 / 4, ...
        goal.position_units(1) - initial.position_units(1));
    tooShort = planner(obstacle, initial, goal, limits, options);
    verifyFalse(testCase, tooShort.Success);
    verifyEqual(testCase, tooShort.TerminationReason, "timeWindowInfeasible");

    % Stretch the fixture to ten seconds while keeping its endpoint geometry
    % and physical limits, so the hull is active throughout a feasible detour.
    goal.time_s        = 10;
    obstacle.time_s(2) = goal.time_s;
    result     = planner(obstacle, initial, goal, limits, options);
    validation = obstacleAvoidance.validateTrajectory(result);
    verifyTrue(testCase, result.Success, result.Message);
    verifyEqual(testCase, result.TerminationReason, "goalReached");
    verifyTrue(testCase, validation.Passed, validation.Message);
    verifyTrue(testCase, result.PreparedObstacles.InternalPreparation.IntervalUsesEndpointHull);
end

function testCachedEndpointHullPlansEarlierAndOverlappingHorizons(testCase)
    first  = [10, 10; 12, 10; 12, 12; 10, 12];
    second = first + [0.2, 0];
    third  = [10.4, 10; 12.4, 10; 12.4, 12; 11.4, 13; 10.4, 12];
    source = obstacleAvoidance.obstacles.createObstacle('later endpoint hull', [0; 1; 2], ...
        {first(:, 1); second(:, 1); third(:, 1)}, ...
        {first(:, 2); second(:, 2); third(:, 2)}, 0);
    prepared = obstacleAvoidance.obstacles.prepareObstacles(source, [0, 2]);
    initial  = struct('time_s', 0, 'position_units', [-2, 0]);
    goal     = struct('time_s', 0.5, 'position_units', [-1, 0]);
    limits = struct( ...
        'xInterval_units',          [-5, 15], ...
        'yInterval_units',          [-5, 15], ...
        'maxVelocity_units_s',      [10, 10], ...
        'maxAcceleration_units_s2', [100, 100], ...
        'maxJerk_units_s3',         [1000, 1000]);
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

function testProperCrossingZigzagHasDeclaredRepair(testCase)
    ring=[0,0;2,2;0,2;2,0;4,0;4,4;-1,4;-1,0];
    obstacle=obstacleAvoidance.obstacles.createObstacle('fold',[0;1], ...
        {ring(:,1);ring(:,1)+0.25},{ring(:,2);ring(:,2)},0);
    verifyEqual(testCase,cellfun(@numel,obstacle.x_units),[6;6]);
    diagnostics=obstacle.NormalizationDiagnostics;
    verifyTrue(testCase,any(diagnostics.Reasons=="selfCrossingZigzagRemoved"));
    verifyEqual(testCase,diagnostics.RemovedZigzagVertexCountBySample,2*ones(2,2));
    verifyEqual(testCase,diagnostics.AffectedSampleIndex,[1;2]);
    verifyEqual(testCase,diagnostics.AffectedSampleTime_s,[0;1]);
    rebuilt=obstacleAvoidance.obstacles.createObstacle(obstacle);
    verifyEqual(testCase,rebuilt.NormalizationDiagnostics,diagnostics);
    % A four-vertex bow tie leaves fewer than three vertices: remove the run.
    bow=obstacleAvoidance.obstacles.createObstacle('empty fold',0,[0;2;0;2],[0;2;2;0]);
    verifyEmpty(testCase,bow.x_units{1});
    verifyEqual(testCase,bow.NormalizationDiagnostics.RemovedRegionCount,[1,1]);
end

function testMovingCellsContainUnprovableCorrespondingRing(testCase)
    [lower, upper] = thinNotchFixture();
    for margin_units=[0,0.1]
        source=obstacleAvoidance.obstacles.createObstacle('thin notch',[0;1], ...
            {lower(:,1);upper(:,1)},{lower(:,2);upper(:,2)},margin_units);
        prepared=obstacleAvoidance.obstacles.prepareObstacles(source);
        preparation=prepared.InternalPreparation;
        if margin_units==0
            verifyEqual(testCase,preparation.IntervalGeometryModel,"movingConvexCells");
            verifyFalse(testCase,preparation.MatchingTopology);
            verifyTrue(testCase,preparation.IntervalUsesMovingCells);
            verifyFalse(testCase,preparation.IntervalIsUnsupported);
            cells=obstacleAvoidance.obstacles.createTimeCells(prepared,0,1);
            verifyEqual(testCase,cells.Regions_units,cells.EndRegions_units);
            enclosure=unionRegions(cells.Regions_units);
        else
            % Protection can itself remove a thin notch and admit an exact
            % model. Exercise the given cell-level margin independently.
            [supported,enclosure]=obstacleAvoidance.obstacles.createMovingCells(lower,upper,margin_units);
            verifyTrue(testCase,supported);
        end
        aligned=obstacleAvoidance.obstacles.alignCorrespondingRing(lower,upper);
        for tau=[0,0.25,0.5,0.75,1]
            polygon=polyshape((1-tau)*lower+tau*aligned,'Simplify',false);
            if margin_units>0, polygon=polybuffer(polygon,margin_units,'JointType','square'); end
            verifyLessThan(testCase,area(subtract(polygon,enclosure)),1e-11);
            [queried,geometry]=obstacleAvoidance.obstacles.preparedShapeAtTime(prepared,tau);
            if tau>0 && tau<1 && margin_units==0
                verifyEqual(testCase,area(xor(queried,enclosure)),0);
                verifyFalse(testCase,geometry.TopologyIsInterpolated);
                verifyEqual(testCase,geometry.VertexSpeedBound_units_s,0);
            elseif tau==0 || tau==1
                verifyEqual(testCase,area(xor(queried,preparation.SampleShapes{1+tau})),0);
            end
        end
    end
end

function testMovingCellsDoNotBecomeAnExactTranslationPartition(testCase)
    [first, second] = thinNotchFixture();
    third=second+[1,0];
    source=obstacleAvoidance.obstacles.createObstacle('mixed models',[0;1;2], ...
        {first(:,1);second(:,1);third(:,1)},{first(:,2);second(:,2);third(:,2)},0);
    prepared=obstacleAvoidance.obstacles.prepareObstacles(source);
    verifyEqual(testCase,prepared.InternalPreparation.IntervalGeometryModel, ...
        ["movingConvexCells";"linearCorrespondingConvexPartition"]);
    verifyFalse(testCase,prepared.InternalPreparation.IntervalPartitionReused(2));
    actual=unionRegions(prepared.InternalPreparation.IntervalStartRegions_units{2});
    verifyLessThan(testCase,area(xor(actual,polyshape(second,'Simplify',false))),1e-12);
end

function testRootTwoMarginSquaresContainSquareJoinProtection(testCase)
    % A square-join buffer of distance d reaches at most d*sqrt(2) from the
    % source, so an axis-aligned square of half-width d is not enough (the
    % rotated square below proves it) while half-width d*sqrt(2) is.
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
    % The proven endpoint containment therefore passes for a protected
    % corresponding ring that has no exact affine proof.
    lower=[0,0;4,0;4,4;2,4;2,4-1e-13;1,4;0,4];
    upper=lower; upper(5,1)=2.2; upper(2,1)=4.2;
    source=obstacleAvoidance.obstacles.createObstacle('square-join certificate',[0;1], ...
        {lower(:,1);upper(:,1)},{lower(:,2);upper(:,2)},0.1);
    prepared=obstacleAvoidance.obstacles.prepareObstacles(source);
    preparation=prepared.InternalPreparation;
    verifyNotEqual(testCase,preparation.IntervalGeometryModel,"unsupportedContinuousDeformation");
    verifyNotEqual(testCase,preparation.IntervalProofReason,"movingCellsExcludeProtectedSample");
    if preparation.IntervalGeometryModel=="movingConvexCells"
        verifyLessThanOrEqual(testCase,max(preparation.IntervalMovingCellUncoveredProtectedArea_units2), ...
            4096*eps(max(1,area(preparation.SampleShapes{1}))));
        enclosure=unionRegions(preparation.IntervalStartRegions_units{1});
        for tau=[0,0.5,1]
            polygon=polybuffer(polyshape((1-tau)*lower+tau*upper,'Simplify',false),0.1,'JointType','square');
            verifyLessThan(testCase,area(subtract(polygon,enclosure)),1e-11);
        end
    end
    verifyEqual(testCase,prepared.x_units,source.x_units);
    verifyEqual(testCase,prepared.y_units,source.y_units);
end

function testMovingObstacleDeclaresSourceIndexCorrespondence(testCase)
    % A dense near-circular ring rotated by six degrees per sample is
    % ambiguous to circular correlation (a cyclic index shift undoes the
    % rotation), but the moving-obstacle constructor transformed one source
    % ring, so it declares index correspondence and the moving cells must
    % contain the index-interpolated polygon, not the correlated one.
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
    % Buffered protected rings carry no index order, so the only faithful
    % model here is the moving-cell enclosure built from the original rings.
    verifyEqual(testCase,preparation.IntervalGeometryModel(1),"movingConvexCells");
    [enclosure,geometry]=obstacleAvoidance.obstacles.preparedShapeAtTime(prepared,0.5);
    for tau=[0.25,0.5,0.75]
        polygon=polybuffer(polyshape((1-tau)*lower+tau*upper,'Simplify',false),0.05,'JointType','square');
        verifyLessThan(testCase,area(subtract(polygon,enclosure)),1e-10);
    end
    verifyFalse(testCase,geometry.TopologyIsInterpolated && ...
        preparation.IntervalGeometryModel(1)=="movingConvexCells");
end

function testReportedModelMutationDoesNotSelectBehavior(testCase)
    % Prove that retained provenance labels cannot select geometry behavior.
    [lower_units, upper_units] = thinNotchFixture();
    source = obstacleAvoidance.obstacles.createObstacle( ...
        'model mutation', [0; 1], ...
        {lower_units(:, 1); upper_units(:, 1)}, ...
        {lower_units(:, 2); upper_units(:, 2)}, 0);
    prepared = obstacleAvoidance.obstacles.prepareObstacles(source);
    verifyTrue(testCase, prepared.InternalPreparation.IntervalUsesMovingCells);

    mutated = prepared;
    mutated.vertexCorrespondence = "nonsense";
    mutated.InternalPreparation.IntervalGeometryModel(:) = "nonsense";
    mutated.InternalPreparation.IntervalProofReason(:) = "nonsense";

    referenceCells = obstacleAvoidance.obstacles.createTimeCells(prepared, 0, 1);
    mutatedCells   = obstacleAvoidance.obstacles.createTimeCells(mutated, 0, 1);
    verifyEqual(testCase, mutatedCells, referenceCells);

    [referenceShape, referenceGeometry] = ...
        obstacleAvoidance.obstacles.preparedShapeAtTime(prepared, 0.5);
    [mutatedShape, mutatedGeometry] = ...
        obstacleAvoidance.obstacles.preparedShapeAtTime(mutated, 0.5);
    verifyEqual(testCase, area(xor(mutatedShape, referenceShape)), 0);
    verifyEqual(testCase, mutatedGeometry, referenceGeometry);

    nodes_units = [
        -1, 2
         5, 2
        -1, 5
         5, 5
    ];
    edgeCost_units = hypot( ...
        nodes_units(:, 1) - nodes_units(:, 1).', ...
        nodes_units(:, 2) - nodes_units(:, 2).');
    initialState  = struct('time_s', 0, 'position_units', nodes_units(1, :));
    goalState     = struct('time_s', 1, 'position_units', nodes_units(2, :));
    limits        = struct('maxVelocity_units_s', [20, 20]);
    options       = struct('GoalTimeMode', "fixedArrival");
    sampleTimes_s = (0:0.1:1).';
    [referenceRoute_units, referenceRouteTime_s, referenceRecord] = ...
        obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodes_units, edgeCost_units, prepared, initialState, goalState, ...
        limits, sampleTimes_s, options);
    [mutatedRoute_units, mutatedRouteTime_s, mutatedRecord] = ...
        obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodes_units, edgeCost_units, mutated, initialState, goalState, ...
        limits, sampleTimes_s, options);
    verifyEqual(testCase, mutatedRoute_units, referenceRoute_units);
    verifyEqual(testCase, mutatedRouteTime_s, referenceRouteTime_s);
    verifyEqual(testCase, mutatedRecord, referenceRecord);
    verifyGreaterThan(testCase, referenceRecord.RejectedTransitionCount, 0);
    verifyTrue(testCase, any(all(referenceRoute_units == nodes_units(3, :), 2)));
    verifyTrue(testCase, any(all(referenceRoute_units == nodes_units(4, :), 2)));
end

function testRedundantAffineKeyframesUseIdenticalCellsAndMotion(testCase)
    ring=[10,10;13,10;13,13;11,11;10,13];
    times_s=linspace(0,16,17).';
    frames=arrayfun(@(t)ring+[t/8,-t/16],times_s,'UniformOutput',false);
    dense=obstacleAvoidance.obstacles.createObstacle('affine',times_s, ...
        cellfun(@(v)v(:,1),frames,'UniformOutput',false),cellfun(@(v)v(:,2),frames,'UniformOutput',false));
    sparse=obstacleAvoidance.obstacles.createObstacle('affine',times_s([1,end]), ...
        dense.x_units([1,end]),dense.y_units([1,end]));
    prepared=obstacleAvoidance.obstacles.prepareObstacles(dense);
    verifyEqual(testCase,prepared.InternalPreparation.MergedSpanTime_s,[0,16]);
    verifyEqual(testCase,prepared.InternalPreparation.MergedIntervalCount,15);
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

function prepared=preparePair(lower,upper)
    source=obstacleAvoidance.obstacles.createObstacle('generic',[0;1], ...
        {lower(:,1);upper(:,1)},{lower(:,2);upper(:,2)},0);
    prepared=obstacleAvoidance.obstacles.prepareObstacles(source);
end

function [lower_units, upper_units] = thinNotchFixture()
    % Return the shared unprovable corresponding-ring regression fixture.
    lower_units = [
        0, 0
        4, 0
        4, 4
        2, 4
        2, 4 - 1e-13
        1, 4
        0, 4
    ];
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
        oriented=upper; if reverse, oriented=flipud(oriented); end
        for shift=0:size(lower,1)-1
            candidate=circshift(oriented,shift);
            cost=sum((candidate-lower).^2,'all');
            if cost<bestCost, bestCost=cost; best=candidate; end
        end
    end
end
