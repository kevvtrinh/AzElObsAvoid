function report = benchmarkFrozenRequests(casesPath, outputPath, repetitionCount)
%% Section 0: Header & Readme
% SYNTAX: report = benchmarkFrozenRequests(casesPath, outputPath, repetitionCount)
% PURPOSE: Measure identical physical requests with one untimed warm-up per case.
% INPUTS: MAT file containing cases, output MAT path, and at least three repetitions.
% OUTPUTS: Individual timings, results, fresh validation, and adaptive arc lengths.
% UNITS: Seconds, coordinate units, and physical derivatives in those units.

%% Section 1: Read Frozen Physical Inputs
if nargin < 3, repetitionCount = 3; end
validateattributes(repetitionCount, {'numeric'}, {'scalar','integer','>=',3});
data = load(casesPath, 'cases');
cases = data.cases;
report = struct();
report.MatlabVersion = version;
report.PlannerPath = which('obstacleAvoidance.planTrajectory');
report.Toolboxes = ver;
report.WarmupPolicy = "One complete untimed solve per physical case before three timed solves";
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
    obstacleAvoidance.planTrajectory(request.obstacles, request.initialState, request.goalState, request.limits, options);
    for repeatIndex = 1:repetitionCount
        timer = tic;
        [result, diagnosis] = obstacleAvoidance.planTrajectory(request.obstacles, request.initialState, request.goalState, request.limits, options);
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
save(outputPath, 'report', '-v7.3');
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
