function tests = testMovingPlaneBatch
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testMovingPlaneBatch.m')
% PURPOSE: Match batched affine moving-plane verification to scalar checks.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Equivalent offsets, gaps, and decisions for varied convex cells.
% UNITS: Coordinate units.
tests = functiontests(localfunctions);
end

function setupOnce(~)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root, fullfile(root, 'trajectory'));
end

function testBatchMatchesScalarVerification(testCase)
    controlPoint_units = [-3, 0; -2, 0.2; -1, 0.1; 0, -0.1; 1, -0.2; 2, 0];
    firstRegions_units = { ...
        [4, -1; 5, -1; 4.5, 1]; ...
        [-1, 2; 1, 2; 1, 3; -1, 3]; ...
        [-0.5, -0.5; 0.5, -0.5; 0.7, 0; 0, 0.7; -0.7, 0]};
    shifts_units = {[0.4, 0.2]; [-0.2, 0.3]; [0.1, -0.1]};
    lastRegions_units = cellfun(@plus, firstRegions_units, shifts_units, ...
        'UniformOutput', false);
    reserve_units = 1e-8;
    target_units  = 1e-7;
    planes = repmat(bmtpEngine.createEmptyPlane(), 1, numel(firstRegions_units));
    scalarPlanes = planes;
    for regionIndex = 1:numel(firstRegions_units)
        vertices_units = cat(3, ...
            firstRegions_units{regionIndex}, lastRegions_units{regionIndex});
        planes(regionIndex) = bmtpEngine.solveSeparatingLine( ...
            controlPoint_units, vertices_units, target_units, reserve_units);
        scalarPlanes(regionIndex) = bmtpEngine.verifySeparatingLine( ...
            planes(regionIndex), controlPoint_units, vertices_units, ...
            reserve_units, target_units);
    end
    batchPlanes = bmtpEngine.verifyMovingSeparatingLines( ...
        planes, controlPoint_units, firstRegions_units, lastRegions_units, ...
        reserve_units, target_units);

    verifyEqual(testCase, [batchPlanes.Verified], [scalarPlanes.Verified]);
    verifyEqual(testCase, [batchPlanes.Offset_units], ...
        [scalarPlanes.Offset_units], 'AbsTol', 1e-12);
    verifyEqual(testCase, [batchPlanes.SignedGap_units], ...
        [scalarPlanes.SignedGap_units], 'AbsTol', 1e-12);
end

function testMalformedRegionIsRejected(testCase)
    plane = bmtpEngine.createEmptyPlane();
    verifyError(testCase, @() bmtpEngine.verifyMovingSeparatingLines( ...
        plane, zeros(6, 2), {zeros(0, 2)}, {zeros(0, 2)}, 1e-8, 1e-7), ...
        'bmtpEngine:InvalidMovingPlaneBatch');
end
