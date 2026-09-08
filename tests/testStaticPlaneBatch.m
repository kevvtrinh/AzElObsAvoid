function tests = testStaticPlaneBatch
%% Section 0: Header & Readme
% SYNTAX: tests = testStaticPlaneBatch
% PURPOSE: Compare bounded static plane verification with its scalar authority.
% INPUTS: None.
% OUTPUTS: MATLAB regression tests.
% UNITS: Coordinate units.
tests = functiontests(localfunctions);
end

function setupOnce(~)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root, fullfile(root, 'trajectory'));
end

function testScalarAgreementAcrossScalesDegreesAndBatchBoundaries(testCase)
    previous = rng(8401); cleanup = onCleanup(@() rng(previous));
    for degree = [3 5 8]
        for translation = [0 1e6 1e9]
            controls = rand(degree + 1, 2) + [translation -translation];
            planes = repmat(emptyPlane(), 1, 513);
            regions = cell(1, numel(planes));
            for index = 1:numel(planes)
                count = 3 + mod(index, 9);
                angle = (0:count - 1).' * 2 * pi / count;
                regions{index} = [cos(angle) sin(angle)] + 6 * rand(1, 2) + [translation -translation];
                normal = randn(2, 2);
                normal = normal ./ vecnorm(normal, 2, 2);
                if mod(index, 2) == 0, normal(2, :) = normal(1, :); end
                planes(index).Normal = normal;
                planes(index).Offset_units = randn(1, 2);
            end
            compareScalar(testCase, planes, controls, regions, 1e-6, 2e-6);
        end
    end
end

function testNearEveryAcceptanceBoundaryAndLargeRegion(testCase)
    controls = [0 0; 0 1; 0 2; 0 3];
    for translation = [0 1e6 1e9]
        scale = max(1, translation);
        reserve = 2 ^ 20 * eps * scale;
        target = 2 * reserve;
        perturbation = [-64 -2 -1 0 1 2 64] * eps(scale);
        regions = cell(1, numel(perturbation));
        planes = repmat(emptyPlane(), 1, numel(regions));
        for index = 1:numel(regions)
            gap = target + reserve + perturbation(index);
            regions{index} = [gap -1; gap+1 -1; gap+1 4; gap 4] + translation;
            planes(index).Normal = [1 0; 1 0];
        end
        compareScalar(testCase, planes, controls + translation, regions, reserve, target);
    end
    angle = (0:65536).' * 2 * pi / 65537;
    largeRegion = [cos(angle) + 4 sin(angle)];
    compareScalar(testCase, emptyPlane(), controls, {largeRegion}, 1e-6, 2e-6);
    verifyEmpty(testCase, bmtpEngine.verifyStaticSeparatingLines(repmat(emptyPlane(), 1, 0), controls, {}, 1e-6, 2e-6));
    verifyError(testCase, @() bmtpEngine.verifyStaticSeparatingLines(emptyPlane(), controls, {}, 1e-6, 2e-6), 'bmtpEngine:InvalidPlaneBatch');
end

function compareScalar(testCase, planes, controls, regions, reserve, target)
    expected = planes;
    for index = 1:numel(planes)
        expected(index) = bmtpEngine.verifySeparatingLine(planes(index), controls, regions{index}, reserve, target);
    end
    actual = bmtpEngine.verifyStaticSeparatingLines(planes, controls, regions, reserve, target);
    verifyEqual(testCase, [actual.Verified], [expected.Verified]);
    verifyEqual(testCase, [actual.Active], [expected.Active]);
    verifyEqual(testCase, [actual.Normal], [expected.Normal]);
    vertices = vertcat(regions{:});
    tolerance = 8 * eps(max(1, max(abs([controls(:); vertices(:)]))));
    verifyEqual(testCase, [actual.Offset_units], [expected.Offset_units], 'AbsTol', tolerance);
    verifyEqual(testCase, [actual.SignedGap_units], [expected.SignedGap_units], 'AbsTol', tolerance);
end

function plane = emptyPlane()
    plane = struct('Active', true, 'Verified', false, 'ExitFlag', NaN, ...
        'Normal', [1 0; 1 0], 'Offset_units', [0 0], 'SignedGap_units', NaN);
end
