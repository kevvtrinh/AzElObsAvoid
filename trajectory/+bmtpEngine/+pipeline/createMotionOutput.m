function candidate = createMotionOutput(candidate, request, preparedMotion)
%% Section 0: Header & Readme
% SYNTAX
%   candidate = bmtpEngine.pipeline.createMotionOutput(candidate, request, preparedMotion)
%**************************************************************************
% PURPOSE
%   - Export a prepared BMTP curve through the stable candidate fields.
%**************************************************************************
% INPUTS
%   - candidate (scalar struct)
%       Empty candidate record whose stable fields are filled in.
%   - request (scalar struct)
%       Checked request supplying the initial clock and sample time.
%   - preparedMotion (scalar struct)
%       Final prepared control net and its motion certificate.
%**************************************************************************
% OUTPUTS
%   - candidate (scalar struct)
%       Polynomial, sampled histories, and motion measures.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Convert The Prepared Net To A Sampled Polynomial
polynomial = bmtpEngine.motion.createPowerPolynomial(preparedMotion.ControlPoint_units, ...
    preparedMotion.SegmentTime_s, request.InitialState.time_s, preparedMotion.PrescribedPower_units);
sampledMotion = samplePolynomial(polynomial, request.Options.SampleTime_s);

%% Section 2: Measure Arrival, Path Length, And Jerk Cost
candidate.ArrivalTime_s        = polynomial.FinalTime_s;
candidate.TrajectoryDuration_s = polynomial.FinalTime_s - request.InitialState.time_s;
candidate.MotionLength_units   = 0;
for segmentIndex = 1:polynomial.SegmentCount
    velocityPower_units_s = squeeze(polynomial.velocityPower_units_s(segmentIndex, :, :));
    speedAtTauHandle = @(tau) hypot(polyval(fliplr(velocityPower_units_s(1, :)), tau), ...
        polyval(fliplr(velocityPower_units_s(2, :)), tau));
    candidate.MotionLength_units = candidate.MotionLength_units + ...
        polynomial.SegmentDuration_s(segmentIndex) * ...
        integral(speedAtTauHandle, 0, 1, 'AbsTol', 1e-11, 'RelTol', 1e-11);
end
candidate.IntegratedSquaredJerk_units2_s5 = integratedSquaredJerk(polynomial);
candidate.MaximumConstraintViolation      = preparedMotion.MotionCertificate.MaximumViolation;

%% Section 3: Transfer The Sampled Histories And The Polynomial
for fieldName = ["time_s", "position_units", "velocity_units_s", ...
        "acceleration_units_s2", "jerk_units_s3"]
    candidate.(fieldName) = sampledMotion.(fieldName);
end
candidate.Polynomial = polynomial;
end

%% Section 4: Local Functions
function sampled = samplePolynomial(polynomial, sampleTime_s)
    % Sample the output polynomial on a uniform grid that retains every knot.
    initialTime_s = polynomial.SegmentStartTime_s(1);
    duration_s    = polynomial.FinalTime_s - initialTime_s;
    % Distinct relative knots can round to the same absolute time after a
    % clock shift. Deduplicate in the exported coordinate, retaining endpoints.
    sampleTimes_s = unique([initialTime_s + (0:sampleTime_s:duration_s).'; ...
        polynomial.SegmentStartTime_s; polynomial.FinalTime_s]);
    [time_s, position_units, velocity_units_s, acceleration_units_s2, jerk_units_s3] = ...
        bmtpEngine.motion.evaluatePolynomial(polynomial, sampleTimes_s);
    sampled = struct( ...
        "time_s",                time_s, ...
        "position_units",        position_units, ...
        "velocity_units_s",      velocity_units_s, ...
        "acceleration_units_s2", acceleration_units_s2, ...
        "jerk_units_s3",         jerk_units_s3);
end

function cost_units2_s5 = integratedSquaredJerk(polynomial)
    % Integrate squared physical jerk exactly over every polynomial segment.
    jerkCoefficients_units_s3 = permute(polynomial.jerkPower_units_s3, [3, 1, 2]);
    powerOrder                 = (1:size(jerkCoefficients_units_s3, 1)).';
    gramMatrix                 = 1 ./ (powerOrder + powerOrder.' - 1);
    segmentCosts_units2_s6     = sum(jerkCoefficients_units_s3 .* ...
        pagemtimes(gramMatrix, jerkCoefficients_units_s3), 1);
    cost_units2_s5 = sum(polynomial.SegmentDuration_s(:) .* ...
        reshape(segmentCosts_units2_s6, polynomial.SegmentCount, 2), "all");
end
