function tests = testObstacleFragments
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testObstacleFragments.m')
% PURPOSE: Preserve occupied regions and timestamps when zero-area fragments
%          appear in a polygon history.
% INPUTS: MATLAB unit test framework and the public obstacle constructor.
% OUTPUTS: Fragment, input rejection, original/protected, and margin checks.
% UNITS: Coordinate units and seconds.
tests=functiontests(localfunctions);
end

function setupOnce(~)
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'));
end

function testSingletonAtReportedSampleRetainsOtherRegion(testCase)
    times=(0:2664).';
    x=repmat({[0;2;2;0]},numel(times),1);
    y=repmat({[0;0;2;2]},numel(times),1);
    x{2664}=[x{2664};NaN;5]; y{2664}=[y{2664};NaN;5];
    obstacle=obstacleAvoidance.obstacles.createObstacle('fragment history',times,x,y);
    verifyEqual(testCase,obstacle.time_s,times);
    verifyEqual(testCase,obstacle.x_units{2664},[0;2;2;0]);
    verifyEqual(testCase,obstacle.y_units{2664},[0;0;2;2]);
    verifyEqual(testCase,obstacle.originalX_units,obstacle.x_units);
    verifyEqual(testCase,obstacle.originalY_units,obstacle.y_units);
    verifyEqual(testCase,obstacle.NormalizationDiagnostics.RemovedRegionCount,[1,1]);
    verifyEqual(testCase,obstacle.NormalizationDiagnostics.AffectedSampleIndex,2664);
    verifyEqual(testCase,obstacle.NormalizationDiagnostics.AffectedSampleTime_s,times(2664));
end

function testSingletonOnlySampleKeepsItsTime(testCase)
    times=[2;3;4]; x={[-1;1;1;-1];0;[-1;1;1;-1]};
    y={[-1;-1;1;1];0;[-1;-1;1;1]};
    obstacle=obstacleAvoidance.obstacles.createObstacle('contact event',times,x,y);
    verifyEqual(testCase,obstacle.time_s,times);
    verifySize(testCase,obstacle.x_units,[3,1]);
    verifyEmpty(testCase,obstacle.x_units{2});
    verifyEmpty(testCase,obstacle.y_units{2});
    verifyEqual(testCase,obstacle.x_units([1,3]),x([1,3]));
end

function testMixedFragmentsPreserveThinPositiveArea(testCase)
    width=eps(1);
    ring=[0,1;2,1;2,1+width;0,1+width];
    obstacle=obstacleAvoidance.obstacles.createObstacle('thin region',0, ...
        [8;NaN;ring(:,1);NaN;10;11],[8;NaN;ring(:,2);NaN;10;11]);
    verifyEqual(testCase,obstacle.x_units{1},ring(:,1));
    verifyEqual(testCase,obstacle.y_units{1},ring(:,2));
end

function testFragmentsDoNotChangeAbsoluteMargin(testCase)
    reference=obstacleAvoidance.obstacles.createObstacle('reference',0,[0;2;2;0],[0;0;2;2],0.2);
    actual=obstacleAvoidance.obstacles.createObstacle('fragment',0,[0;2;2;0;NaN;8],[0;0;2;2;NaN;8],0.2);
    rebuilt=obstacleAvoidance.obstacles.createObstacle(actual,0.2);
    verifyEqual(testCase,actual.x_units,reference.x_units);
    verifyEqual(testCase,actual.y_units,reference.y_units);
    verifyEqual(testCase,rebuilt.x_units,reference.x_units);
    verifyEqual(testCase,rebuilt.y_units,reference.y_units);
    verifyEqual(testCase,actual.originalX_units,reference.originalX_units);
    verifyEqual(testCase,actual.originalY_units,reference.originalY_units);
end

function testMalformedSeparatorsStillFail(testCase)
    verifyError(testCase,@()obstacleAvoidance.obstacles.createObstacle( ...
        'unpaired separator',0,[0;1;NaN],[0;1;2]),'createObstacle:UnpairedNonfiniteBoundary');
end

function testClosuresDuplicatesAndAlternatingTwoPoints(testCase)
    x=[0;0;2;2;0;0;0;NaN;4;5;4;5;4];
    y=[0;0;0;2;2;0;0;NaN;4;5;4;5;4];
    obstacle=obstacleAvoidance.obstacles.createObstacle('redundant copies',0,x,y);
    verifyEqual(testCase,obstacle.x_units{1},[0;2;2;0]);
    verifyEqual(testCase,obstacle.y_units{1},[0;0;2;2]);
    verifyEqual(testCase,obstacle.NormalizationDiagnostics.RemovedRegionCount,[1,1]);
    rebuilt=obstacleAvoidance.obstacles.createObstacle(obstacle);
    verifyEqual(testCase,rebuilt,obstacle);
end

function testPreparedEmptyEventPreservesNeighborCoverage(testCase)
    source=obstacleAvoidance.obstacles.createObstacle('visible neighbors',[2;3;4], ...
        {[-1;1;1;-1];0;[-1;1;1;-1]},{[-1;-1;1;1];0;[-1;-1;1;1]});
    prepared=obstacleAvoidance.obstacles.prepareObstacles(source);
    occupied=obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(prepared, ...
        zeros(5,1),zeros(5,1),[1;2.5;3;3.5;5]);
    verifyEqual(testCase,occupied,[false;true;false;true;false]);
end
