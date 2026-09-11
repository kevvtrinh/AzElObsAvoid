function tests = testStaticCorridorConsolidation
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testStaticCorridorConsolidation.m')
% PURPOSE: Check that consecutive static corridor cells merge only when
%   their convex superset remains exactly separated from protected geometry.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Verified safe-merge and unsafe-merge rejection checks.
% UNITS: Coordinate units.
tests=functiontests(localfunctions);
end

function testRepeatedSafeCellsMergeWithoutRemovingTheirFeasibleSet(testCase)
    [region_units,geometry,limits,emptyPlane]=createFixture();
    sourcePlanes=repmat(emptyPlane,4,1);
    sourcePlanes(:)=createActivePlane([0,1],0.2);
    [planes,diagnostics]=bmtpEngine.consolidateStaticCorridor(sourcePlanes, ...
        {region_units},{geometry},limits,0.1,0.01);
    verifyTrue(testCase,diagnostics.CompleteVerifiedCover);
    verifyTrue(testCase,diagnostics.Applied);
    verifyEqual(testCase,diagnostics.GroupSpanIndex,[1,4]);
    verifyEqual(testCase,diagnostics.GroupCount,1);
    verifyEqual(testCase,diagnostics.RetainedSpanPlaneCount,4);
    verifyTrue(testCase,all([planes.Active]));
    verifyTrue(testCase,all([planes.Verified]));
end

function testUnsafeCornerSpanningMergeIsRejected(testCase)
    [region_units,geometry,limits,emptyPlane]=createFixture();
    sourcePlanes=repmat(emptyPlane,3,1);
    sourcePlanes(1)=createActivePlane([1,0],0.2);
    sourcePlanes(2)=createActivePlane([0,1],0.2);
    sourcePlanes(3)=createActivePlane([-1,0],2.2);
    [planes,diagnostics]=bmtpEngine.consolidateStaticCorridor(sourcePlanes, ...
        {region_units},{geometry},limits,0.1,0.01);
    verifyTrue(testCase,diagnostics.CompleteVerifiedCover);
    verifyFalse(testCase,diagnostics.Applied);
    verifyEqual(testCase,diagnostics.GroupSpanIndex,[1,1;2,2;3,3]);
    verifyEqual(testCase,diagnostics.GroupCount,3);
    verifyEqual(testCase,diagnostics.RetainedSpanPlaneCount,3);
    verifyTrue(testCase,all([planes.Active]));
    verifyTrue(testCase,all([planes.Verified]));
end

function [region_units,geometry,limits,emptyPlane]=createFixture()
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'));
    region_units=[0,0;2,0;2,2;0,2];
    edges_units=diff([region_units;region_units(1,:)],1,1);
    edgeLength_units=vecnorm(edges_units,2,2);
    geometry=struct('PositiveNormals',[-edges_units(:,2),edges_units(:,1)]./edgeLength_units);
    limits=struct('xInterval_units',[-5,5],'yInterval_units',[-5,5]);
    emptyPlane=struct('Active',false,'Verified',false,'ExitFlag',-2, ...
        'Normal',zeros(2,2),'Offset_units',zeros(1,2),'SignedGap_units',NaN);
end

function plane=createActivePlane(normal,offset_units)
    plane=struct('Active',true,'Verified',true,'ExitFlag',1, ...
        'Normal',repmat(normal,2,1),'Offset_units',repmat(offset_units,1,2), ...
        'SignedGap_units',0.2);
end
