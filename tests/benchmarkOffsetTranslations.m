function results = benchmarkOffsetTranslations
%% Section 0: Header & Readme
% SYNTAX: results = benchmarkOffsetTranslations
% PURPOSE: Measure scalar and batched offset construction on identical inputs.
% INPUTS: None; three knot counts use fixed clocks, limits, and offsets.
% OUTPUTS: Three matched repetitions per method after separate warm-up calls.
% UNITS: Seconds and coordinate units.

%% Section 1: Prepare One Shared Analytic Motion
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root, fullfile(root, 'trajectory'));
initialState = struct('time_s', 0, 'position_units', [0 0], ...
    'velocity_units_s', [0 0], 'acceleration_units_s2', [0 0]);
goalState = initialState;
goalState.time_s = 20;
goalState.position_units = [10 2];
limits = struct('maxVelocity_units_s', [2 2], 'maxAcceleration_units_s2', [1 1], 'maxJerk_units_s3', [3 3]);
options = struct('GoalTimeMode', 'fixedArrival', 'SampleTime_s', 0.1);
base = bmtpEngine.createDirectMotion(initialState, goalState, limits, options);
assert(base.Success);
results = struct([]);
constructors = {@createOffsetSplineMotionReference, @bmtpEngine.createOffsetSplineMotion};

%% Section 2: Warm Both Implementations And Alternate Matched Timings
for knotCount = [3 9 25]
    times_s = linspace(0, 20, knotCount).';
    offsets_units = 0.2 * sin(linspace(0, 2 * pi, knotCount).');
    offsets_units([1 end]) = 0;
    args = {base, times_s, offsets_units, 2, initialState, 0.1, 'benchmark'};
    reference = constructors{1}(args{:});
    candidate = constructors{2}(args{:});
    assert(isequaln(reference, candidate), 'Scalar and batched motion records differ.');
    elapsed_s = zeros(3, 2);
    for repetition = 1:3
        order = 1:2;
        if mod(repetition, 2) == 0, order = fliplr(order); end
        for methodIndex = order
            constructor = constructors{methodIndex};
            timer = tic;
            for invocation = 1:30
                constructor(args{:});
            end
            elapsed_s(repetition, methodIndex) = toc(timer) / 30;
        end
    end
    medians_s = median(elapsed_s, 1);
    results = [results; struct('KnotCount', knotCount, 'SpanCount', candidate.Polynomial.SegmentCount, ...
        'Runs_s', elapsed_s, 'ScalarMedian_s', medians_s(1), 'BatchMedian_s', medians_s(2), ...
        'Speedup', medians_s(1) / medians_s(2))]; %#ok<AGROW>
    fprintf('OFFSET_TRANSLATION knots=%d spans=%d scalar_s=%.6g batch_s=%.6g speedup=%.4g\n', ...
        knotCount, candidate.Polynomial.SegmentCount, medians_s(1), medians_s(2), results(end).Speedup);
end
end
