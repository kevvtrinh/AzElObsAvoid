function report = benchmarkFrozenRequests(casesPath, outputPath, repetitionCount, measurement)
%% Section 0: Header & Readme
% SYNTAX: report = benchmarkFrozenRequests(casesPath, outputPath, repetitionCount)
% PURPOSE: Measure identical physical requests with an explicit warm-up policy.
% INPUTS: MAT file containing cases, output MAT path, and at least three repetitions.
%   Optional measurement fields: WarmupCount, WarmupOutputCount (0 or 2),
%   SaveFormat ('-v7' or '-v7.3'), and PlannerFunction (two-output handle).
% OUTPUTS: Individual timings, results, fresh validation, and adaptive arc lengths.
% UNITS: Seconds, coordinate units, and physical derivatives in those units.

%% Section 1: Read Frozen Physical Inputs
if nargin < 3, repetitionCount = 3; end
if nargin < 4, measurement = struct(); end
defaults = struct('WarmupCount',1,'WarmupOutputCount',0,'SaveFormat','-v7.3','PlannerFunction',@planner);
for name = string(fieldnames(defaults)).'
    if ~isfield(measurement,name), measurement.(name) = defaults.(name); end
end
validateattributes(measurement.WarmupCount, {'numeric'}, {'scalar','integer','>=',1});
assert(ismember(measurement.WarmupOutputCount,[0 2]));
assert(ismember(string(measurement.SaveFormat),["-v7","-v7.3"]));
assert(isa(measurement.PlannerFunction,'function_handle'));
validateattributes(repetitionCount, {'numeric'}, {'scalar','integer','>=',3});
data = load(casesPath, 'cases');
cases = data.cases;
report = struct();
report.MatlabVersion = version;
report.PlannerPath = which(func2str(measurement.PlannerFunction));
report.Toolboxes = ver;
report.WarmupPolicy = sprintf('%d untimed solves with %d outputs per case; %d timed solves with two outputs',measurement.WarmupCount,measurement.WarmupOutputCount,repetitionCount);
report.CaseNames = string({cases.Name}).';
report.CasesPath = string(casesPath);
report.Results = cell(numel(cases), repetitionCount);
report.Diagnoses = cell(numel(cases), repetitionCount);
report.Validation = cell(numel(cases), repetitionCount);
report.ElapsedTime_s = NaN(numel(cases), repetitionCount);
report.Length_units = NaN(numel(cases), repetitionCount);

%% Section 2: Warm And Measure Each Case Without Display Work
for caseIndex = 1:numel(cases)
    request = cases(caseIndex).Inputs;
    options = cases(caseIndex).Options;
    for warmupIndex = 1:measurement.WarmupCount
        if measurement.WarmupOutputCount == 2
            [~, ~] = measurement.PlannerFunction(request.obstacles, request.initialState, request.goalState, request.limits, options);
        else
            measurement.PlannerFunction(request.obstacles, request.initialState, request.goalState, request.limits, options);
        end
    end
    for repeatIndex = 1:repetitionCount
        timer = tic;
        [result, diagnosis] = measurement.PlannerFunction(request.obstacles, request.initialState, request.goalState, request.limits, options);
        report.ElapsedTime_s(caseIndex, repeatIndex) = toc(timer);
        % Discard internal caches so validation rebuilds authoritative geometry.
        sourceObstacles = request.obstacles;
        if isfield(sourceObstacles, 'InternalPreparation')
            sourceObstacles = rmfield(sourceObstacles, 'InternalPreparation');
        end
        check = obstacleAvoidance.validateTrajectory(result, sourceObstacles, request.initialState, request.goalState, request.limits, options);
        report.Results{caseIndex, repeatIndex} = result;
        report.Diagnoses{caseIndex, repeatIndex} = diagnosis;
        report.Validation{caseIndex, repeatIndex} = check;
        polyline_units = NaN;
        if result.Success
            report.Length_units(caseIndex, repeatIndex) = polynomialArcLength(result.Polynomial);
            polyline_units = sum(vecnorm(diff(result.Route_units), 2, 2));
        end
        fprintf('FROZEN name=%s repeat=%d success=%d valid=%d duration_s=%.12g polyline_units=%.12g arc_units=%.12g collision=%d kinematic=%d certificate=%d reason=%s runtime_s=%.6f\n', ...
            cases(caseIndex).Name, repeatIndex, result.Success, check.Passed, result.TrajectoryDuration_s, polyline_units, ...
            report.Length_units(caseIndex, repeatIndex), check.CollisionFree, ...
            check.VelocityWithinLimits && check.AccelerationWithinLimits && check.JerkWithinLimits, ...
            check.CollisionResolved && check.PolynomialFormatValid && check.DynamicsConsistent && check.InitialStateMatched && check.TerminalStateMatched, ...
            result.TerminationReason, report.ElapsedTime_s(caseIndex, repeatIndex));
        assert(result.Success == cases(caseIndex).ExpectedSuccess, 'Frozen request outcome changed.');
        assert(~result.Success || check.Passed, 'Successful motion failed independent validation.');
    end
end
report.MedianTime_s = median(report.ElapsedTime_s, 2);
save(outputPath, 'report', measurement.SaveFormat);
roundTrip = load(outputPath, 'report');
assert(isequaln(report, roundTrip.report), 'Capture round-trip changed report data.');
end

function length_units = polynomialArcLength(polynomial)
    % Integrate physical speed in each normalized polynomial span, independently
    % of the display samples and of any quadrature used by an optimizer.
    length_units = 0;
    for spanIndex = 1:polynomial.SegmentCount
        coefficients = reshape(polynomial.positionPower_units(spanIndex, :, :), 2, []);
        derivative = coefficients(:, 2:end) .* (1:size(coefficients, 2)-1);
        speed = @(tau) hypot(polyval(fliplr(derivative(1, :)), tau), polyval(fliplr(derivative(2, :)), tau));
        length_units = length_units + integral(speed, 0, 1, 'AbsTol', 1e-10, 'RelTol', 1e-10);
    end
end
