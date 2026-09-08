function tests = testStaticPlaneBatch
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testStaticPlaneBatch.m')
% PURPOSE: Compare batched certificates with the original scalar verifier.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Equivalence checks for passing, failing, and near-clearance planes.
% UNITS: Coordinate units.
tests = functiontests(localfunctions);
end

function setupOnce(~)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'));
end

function testScalarEquivalence(testCase)
    rng(21); regionCount = 300;
    template = struct('Active',true,'Verified',false,'ExitFlag',1,'Normal',zeros(2), ...
        'Offset_units',zeros(1,2),'SignedGap_units',NaN);
    for translation_units = [0,128]
        controls_units = randn(9,2)+translation_units;
        regions_units = cell(regionCount,1); planes = repmat(template,1,regionCount);
        for k = 1:regionCount
            angle = rand*2*pi; center_units = 5*[cos(angle),sin(angle)];
            theta = (0:4).'*2*pi/5;
            regions_units{k} = center_units+[cos(theta),sin(theta)]+translation_units;
            normals = [center_units;center_units+randn(1,2)*0.2];
            normals = normals./vecnorm(normals,2,2);
            if mod(k,7)==0, normals = normals*1.01; end
            planes(k).Normal = normals;
            planes(k).Offset_units = randn(1,2)-sum(normals,2).'*translation_units;
        end
        scalar = planes;
        for k = 1:regionCount
            scalar(k) = bmtpEngine.verifySeparatingLine(scalar(k),controls_units,regions_units{k},1e-8,1.1e-7);
        end
        batch = bmtpEngine.verifyStaticSeparatingLines(planes,controls_units,regions_units,1e-8,1.1e-7);
        verifyEqual(testCase,[batch.Verified],[scalar.Verified]);
        verifyEqual(testCase,[batch.Offset_units],[scalar.Offset_units],'AbsTol',256*eps(max(1,translation_units)));
        verifyEqual(testCase,[batch.SignedGap_units],[scalar.SignedGap_units],'AbsTol',256*eps(max(1,translation_units)));
    end
end

function testNearClearanceAndNonunitNormals(testCase)
    controls_units = zeros(9,2);
    target_units = 1.2e-7; reserve_units = 2e-8;
    gaps_units = target_units+reserve_units+(-4:4)*eps;
    template = struct('Active',true,'Verified',false,'ExitFlag',1,'Normal',eye(2), ...
        'Offset_units',zeros(1,2),'SignedGap_units',NaN);
    regions_units = cell(numel(gaps_units),1); planes = repmat(template,1,numel(gaps_units));
    for k = 1:numel(gaps_units)
        regions_units{k} = [gaps_units(k),-1;1,-1;1,1;gaps_units(k),1];
        planes(k).Normal = [1,0;1,0];
    end
    scalar = planes;
    for k = 1:numel(planes)
        scalar(k) = bmtpEngine.verifySeparatingLine(scalar(k),controls_units,regions_units{k},reserve_units,target_units);
    end
    batch = bmtpEngine.verifyStaticSeparatingLines(planes,controls_units,regions_units,reserve_units,target_units);
    verifyEqual(testCase,[batch.Verified],[scalar.Verified]);
    verifyEqual(testCase,[batch.Offset_units],[scalar.Offset_units]);
    verifyEqual(testCase,[batch.SignedGap_units],[scalar.SignedGap_units]);
    single = bmtpEngine.verifyStaticSeparatingLines(planes(1),controls_units,regions_units(1),reserve_units,target_units);
    verifyEqual(testCase,single,scalar(1));
end
