function tests = testTimedMeshResolution
%% Section 0: Header & Readme
% SYNTAX
%   results = runtests('tests/testTimedMeshResolution.m')
%**************************************************************************
% PURPOSE
%   - Preserve timed guide waits without coupling motion spans to cell count.
%**************************************************************************
% INPUTS
%   - MATLAB function-based unit test framework.
%**************************************************************************
% OUTPUTS
%   - Guide-knot preservation, clock-scaling, and span-count regressions.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Register Tests
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root, fullfile(root, 'trajectory'));
    testCase.TestData.Root = root;
end

function testWaitKnotsScaleWithArrivalClock(testCase)
    request = createWaitRequest(1);
    firstWarm = bmtpEngine.createWarmStart(request);
    for duration_s = [6, 9]
        request.SeedMotionDuration_s = duration_s;
        warm = bmtpEngine.createWarmStart(request);
        meshTau = [0; cumsum(warm.SegmentTime_s)] / duration_s;
        tolerance = 64 * eps;
        for knotIndex = 1:numel(request.Seed.tau)
            [error, meshIndex] = min(abs(meshTau - request.Seed.tau(knotIndex)));
            verifyLessThanOrEqual(testCase, error, tolerance);
            if meshIndex <= warm.SegmentCount
                position_units = reshape(warm.ControlPoint_units(meshIndex, 1, :), 1, 2);
            else
                position_units = reshape(warm.ControlPoint_units(end, end, :), 1, 2);
            end
            verifyEqual(testCase, position_units, request.Seed.position_units(knotIndex, :), ...
                'AbsTol', tolerance);
        end
        waitSpans = meshTau(1:end - 1) >= request.Seed.tau(2) - tolerance & ...
            meshTau(2:end) <= request.Seed.tau(3) + tolerance;
        verifyTrue(testCase, any(waitSpans));
        verifyEqual(testCase, warm.ControlPoint_units(waitSpans, 1, :), ...
            warm.ControlPoint_units(waitSpans, end, :), 'AbsTol', tolerance);
        verifyEqual(testCase, sum(warm.SegmentTime_s(waitSpans)), ...
            duration_s * (request.Seed.tau(3) - request.Seed.tau(2)), ...
            'AbsTol', 64 * eps(duration_s));
        verifyEqual(testCase, warm.ControlPoint_units, firstWarm.ControlPoint_units);
    end
end

function testMotionSpanCountIgnoresCellCount(testCase)
    sparseRequest = createWaitRequest(1);
    denseRequest  = createWaitRequest(920);
    sparseWarm = bmtpEngine.createWarmStart(sparseRequest);
    denseWarm  = bmtpEngine.createWarmStart(denseRequest);
    guideSegmentCount = size(denseRequest.Seed.position_units, 1) - 1;
    expectedSegmentCount = max(20, guideSegmentCount * denseRequest.SplitCount);
    verifyEqual(testCase, denseWarm.SegmentCount, expectedSegmentCount);
    verifyEqual(testCase, denseWarm.ControlPoint_units, sparseWarm.ControlPoint_units);
    verifyEqual(testCase, denseWarm.SegmentTime_s, sparseWarm.SegmentTime_s);
    verifyGreaterThan(testCase, nnz(denseWarm.RegionActiveBySegment), ...
        nnz(sparseWarm.RegionActiveBySegment));
end

function request = createWaitRequest(cellCount)
    % The same stationary remote obstacle has either one cell or many cells.
    % The guide has eleven edges (a wait at its second knot, then nine
    % straight edges) so that the guide-derived span count exceeds the
    % twenty-span floor and the assertion pins the guide rule, not the floor.
    initial = struct('time_s', 0, 'position_units', [-1, 0]);
    goal    = struct('time_s', 10, 'position_units', [1, 0.5]);
    normalized = planner([], initial, goal, [], struct('GoalTimeMode', "earliestArrival"));
    tailTau    = linspace(0.4, 1, 9).';
    tail_units = [-0.4, 0.28] + (tailTau - 0.4) / 0.6 .* ([1, 0.5] - [-0.4, 0.28]);
    seed = struct( ...
        'position_units',    [-1, 0; -0.5, 0.25; -0.5, 0.25; tail_units], ...
        'tau',               [0; 0.2; 0.35; tailTau], ...
        'UsesVariableClock', true);
    breaks_s = linspace(0, 10, cellCount + 1).';
    regions_units = repmat({[3, 3; 4, 3; 4, 4; 3, 4]}, cellCount, 1);
    coverage = struct( ...
        'Passed',                  true, ...
        'MinimumMotionDuration_s', 4, ...
        'SeedMotionDuration_s',    6, ...
        'ActiveTimeInterval_s',    [breaks_s(1:end - 1), breaks_s(2:end)], ...
        'BreakTime_s',             breaks_s);
    request = bmtpEngine.createSolveRequest(seed, regions_units, coverage, ...
        normalized.Inputs.initialState, normalized.Inputs.goalState, ...
        normalized.Limits, normalized.Options);
end
