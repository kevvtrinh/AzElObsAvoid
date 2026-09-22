function tests = testParallelPreparation
%% Section 0: Header & Readme
% SYNTAX
%   results = runtests('tests/testParallelPreparation.m')
%**************************************************************************
% PURPOSE
%   - Compare complete serial and parallel preparation records.
%**************************************************************************
% INPUTS
%   - MATLAB function-based test framework.
%**************************************************************************
% OUTPUTS
%   - tests (function-based test array)
%       Exact equality includes all preparation and normalization fields.
%**************************************************************************
% UNITS
%   - Geometry uses coordinate units; time uses seconds.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root, fullfile(root, 'trajectory'));
    testCase.TestData.Root = root;
end

function testDenseWindowsAndWholeHistory(testCase)
    % The owner-provided history remains optional, ignored benchmark input.
    sourcePath = fullfile(testCase.TestData.Root, 'tmp', 'azel', 'azel_history.mat');
    assumeTrue(testCase, isfile(sourcePath), 'Owner-provided az/el history is unavailable.');
    requireIdlePool(testCase);
    loaded = load(sourcePath, 'obstacles');
    windows_s = [300, 530; 915, 1145; loaded.obstacles.time_s(1), loaded.obstacles.time_s(end)];
    for windowIndex = 1:size(windows_s, 1)
        fprintf('Comparing az/el preparation window [%g, %g] s.\n', windows_s(windowIndex, :));
        compareBuilds(testCase, loaded.obstacles, windows_s(windowIndex, :), false, false);
    end
end

function testTranslationReuseAndNonzeroMargin(testCase)
    requireIdlePool(testCase);
    for margin_units = [0, 0.125]
        source = createHistory(margin_units, false);
        prepared = compareBuilds(testCase, source, [0, 64], false, false);
        verifyTrue(testCase, any(prepared.InternalPreparation.IntervalPartitionReused));
        verifyEqual(testCase, prepared.safetyMargin_units, margin_units);
    end
end

function testRingCountChangesAndCacheExtension(testCase)
    requireIdlePool(testCase);
    source = createHistory(0, true);
    prepared = compareBuilds(testCase, source, [0, 40], false, false);
    verifyTrue(testCase, any(prepared.InternalPreparation.IntervalUsesEndpointHull));
    compareBuilds(testCase, prepared, [0, 64], false, false);
end

function testSmallHistoryAndMergedSpans(testCase)
    requireIdlePool(testCase);
    source = createHistory(0, false);
    compareBuilds(testCase, source, [0, 2], false, false);
    source = createHistory(0.125, false);
    source.time_s = (0:64).' .^ 2;
    prepared = compareBuilds(testCase, source, [0, 64 ^ 2], false, false);
    verifyGreaterThan(testCase, prepared.InternalPreparation.MergedIntervalCount, 0);
    source = createMergedSpanHistory();
    prepared = compareBuilds(testCase, source, [0, source.time_s(end)], false, true);
    verifyGreaterThan(testCase, prepared.InternalPreparation.MergedIntervalCount, 0);
end

function testEarlyStopPreservesSampleCoverage(testCase)
    requireIdlePool(testCase);
    source = createHistory(0, true);
    for sampleIndex = 17:18
        source.x_units{sampleIndex} = [0; 1; 2] * sampleIndex;
        source.y_units{sampleIndex} = [0; 0; 0];
        source.originalX_units{sampleIndex} = source.x_units{sampleIndex};
        source.originalY_units{sampleIndex} = source.y_units{sampleIndex};
    end
    prepared = compareBuilds(testCase, source, [0, 64], true, false);
    verifyTrue(testCase, any(prepared.InternalPreparation.IntervalIsUnsupported));
    verifyFalse(testCase, prepared.InternalPreparation.SamplePrepared(end));
end

function testWorkerErrorsKeepSerialIdentifierAndMessage(testCase)
    requireIdlePool(testCase);
    source = createHistory(0, true);
    prepared = obstacleAvoidance.obstacles.prepareObstacles(source);
    previous = prepared.InternalPreparation;
    previous.IntervalPrepared(:) = false;
    previous.SamplePrepared(:) = false;
    % Malformed internal input reaches the sample worker. Public construction
    % normally rejects this before preparation; exercise error transport here.
    source.y_units{9}(end) = [];
    pool = backgroundPool;
    blocker = parfeval(pool, @pause, 0, 600);
    blockerCleanup = onCleanup(@() cancel(blocker));
    serialError = capturePreparationError(source, previous);
    clear blockerCleanup;
    wait(blocker);
    parallelError = capturePreparationError(source, previous);
    verifyNotEmpty(testCase, serialError);
    verifyEqual(testCase, parallelError, serialError);
end

function errorDetails = capturePreparationError(source, previous)
    errorDetails = strings(0, 1);
    try
        obstacleAvoidance.obstacles.prepareOneObstacle(source, ...
            previous.PreparationVersion, previous.SourceSnapshot, [0, 64], previous, false);
    catch exception
        errorDetails = [string(exception.identifier); string(exception.message)];
    end
end

function prepared = compareBuilds(testCase, source, timeRange_s, stopAtUnsupported, expectDispatch)
    % One pending background job exercises the production busy-pool guard;
    % no option, label, or replacement implementation selects the reference.
    % expectDispatch profiles the idle-pool build, so a history that quietly
    % stayed serial cannot pass as a parallel comparison.
    pool = backgroundPool;
    blocker = parfeval(pool, @pause, 0, 600);
    blockerCleanup = onCleanup(@() cancel(blocker));
    assert(pool.Busy, 'testParallelPreparation:PoolNotBusy', 'The serial guard was not engaged.');
    serial = obstacleAvoidance.obstacles.prepareObstacles(source, timeRange_s, stopAtUnsupported);
    clear blockerCleanup;
    wait(blocker);
    if expectDispatch
        profile('on');
    end
    prepared = obstacleAvoidance.obstacles.prepareObstacles(source, timeRange_s, stopAtUnsupported);
    if expectDispatch
        profile('off');
        verifyGreaterThan(testCase, countDispatchHits(profile('info')), 0, ...
            'The parallel build never reached a parfeval dispatch line.');
    end
    verifyTrue(testCase, isequaln(serial, prepared), ...
        'Complete returned obstacle records must be bit-identical.');
end

function hitCount = countDispatchHits(profileInfo)
    % Count executions of the parfeval dispatch lines in the prepared source.
    sourcePath    = which('obstacleAvoidance.obstacles.prepareOneObstacle');
    dispatchLines = find(contains(readlines(sourcePath), 'parfeval('));
    hitCount      = 0;
    for entryIndex = 1:numel(profileInfo.FunctionTable)
        entry = profileInfo.FunctionTable(entryIndex);
        if ~endsWith(entry.FileName, 'prepareOneObstacle.m')
            continue;
        end
        executedLines = entry.ExecutedLines;
        dispatched    = ismember(executedLines(:, 1), dispatchLines);
        hitCount      = hitCount + sum(executedLines(dispatched, 2));
    end
end

function requireIdlePool(testCase)
    assumeEqual(testCase, exist('backgroundPool', 'builtin'), 5);
    pool = backgroundPool;
    assumeGreaterThan(testCase, pool.NumWorkers, 1);
    assumeFalse(testCase, pool.Busy);
end

function source = createHistory(margin_units, changeRingCount)
    % Changing translation velocity prevents merged spans and requires reuse.
    ring_units = [0, 0; 4, 0; 4, 1; 1, 1; 1, 4; 0, 4];
    time_s  = (0:64).';
    x_units = cell(size(time_s));
    y_units = cell(size(time_s));
    for sampleIndex = 1:numel(time_s)
        points_units = ring_units + [time_s(sampleIndex) ^ 2 / 4096, 0];
        if changeRingCount && mod(sampleIndex, 3) == 0
            points_units = [points_units; NaN, NaN; points_units + [6, 0]]; %#ok<AGROW>
        elseif margin_units == 0 && ~changeRingCount
            points_units = circshift(points_units, mod(sampleIndex, size(ring_units, 1)), 1);
        end
        x_units{sampleIndex} = points_units(:, 1);
        y_units{sampleIndex} = points_units(:, 2);
    end
    source = obstacleAvoidance.obstacles.createObstacle( ...
        'parallel preparation regression', time_s, x_units, y_units, margin_units);
end

function source = createMergedSpanHistory()
    % Accelerating translation isolates the first intervals, so more than the
    % 32-interval dispatch threshold reaches the workers; the constant-velocity
    % tail then merges its intervals into one span within the same history.
    ring_units        = [0, 0; 4, 0; 4, 1; 1, 1; 1, 4; 0, 4];
    isolatedEndIndex  = 49;
    time_s            = (0:68).';
    offsets_units     = zeros(size(time_s));
    x_units           = cell(size(time_s));
    y_units           = cell(size(time_s));
    for sampleIndex = 1:numel(time_s)
        if sampleIndex <= isolatedEndIndex
            offsets_units(sampleIndex) = (sampleIndex - 1) ^ 2 / 4096;
        else
            offsets_units(sampleIndex) = offsets_units(isolatedEndIndex) + ...
                (sampleIndex - isolatedEndIndex) / 64;
        end
        points_units = ring_units + [offsets_units(sampleIndex), 0];
        x_units{sampleIndex} = points_units(:, 1);
        y_units{sampleIndex} = points_units(:, 2);
    end
    source = obstacleAvoidance.obstacles.createObstacle( ...
        'merged span dispatch regression', time_s, x_units, y_units, 0);
end
