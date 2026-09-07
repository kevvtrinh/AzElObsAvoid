function candidate = createMotionOutput(candidate, request, preparedMotion)
%% Section 0: Header & Readme
% SYNTAX
%   candidate = bmtpEngine.createMotionOutput( ...
%       candidate, request, preparedMotion)
%
% PURPOSE
%   - Convert the prepared BMTP curve into the stable polynomial and sampled
%     motion fields consumed outside the engine.
%
% INPUTS
%   - candidate (scalar struct)
%       Stable empty candidate record to populate.
%   - request, preparedMotion (scalar structs)
%       Checked request and final prepared control net.
%
% OUTPUTS
%   - candidate (scalar struct)
%       Candidate with polynomial, sampled histories, and motion measures.
%
% UNITS
%   - Position is coordinate units and time is seconds; derivatives use units/s,
%     units/s^2, and units/s^3.
%

%% Section 1: Create The Stable Motion Record

polynomial = bmtpEngine.createPowerPolynomial(preparedMotion.ControlPoint_units, preparedMotion.SegmentTime_s, request.InitialState.time_s);
sampled    = samplePolynomial(polynomial, request.Options.SampleTime_s);
candidate.ArrivalTime_s                 = polynomial.FinalTime_s;
candidate.TrajectoryDuration_s          = polynomial.FinalTime_s - request.InitialState.time_s;
candidate.MotionLength_units              = sum(vecnorm(diff(sampled.position_units, 1, 1), 2, 2));
candidate.IntegratedSquaredJerk_units2_s5 = integratedSquaredJerk(polynomial);
candidate.MaximumConstraintViolation    = preparedMotion.MotionCertificate.MaximumViolation;
% Apply the required validation or transfer to each field name.
for fieldName = ["time_s", "position_units", "velocity_units_s", ...
        "acceleration_units_s2", "jerk_units_s3"]
    candidate.(fieldName) = sampled.(fieldName);
end
candidate.Polynomial = polynomial;
end

%% Section 2: Local Functions

function sampled = samplePolynomial(polynomial, sampleTime_s)
    % Sample the output polynomial.
    initialTime_s  = polynomial.SegmentStartTime_s(1);
    duration_s     = polynomial.FinalTime_s - initialTime_s;
    segmentTime_s  = polynomial.SegmentDuration_s(1);
    relativeTime_s = unique([(0:sampleTime_s:duration_s).'; (0:polynomial.SegmentCount).' * segmentTime_s; duration_s]);
    [time_s, position_units, velocity_units_s, acceleration_units_s2, jerk_units_s3] = bmtpEngine.evaluatePolynomial(polynomial, initialTime_s + relativeTime_s);
    sampled = struct("time_s", time_s, "position_units", position_units, ...
        "velocity_units_s", velocity_units_s, ...
        "acceleration_units_s2", acceleration_units_s2, ...
        "jerk_units_s3", jerk_units_s3);
end

function cost_units2_s5 = integratedSquaredJerk(polynomial)
    % Integrate squared physical jerk exactly over every polynomial segment.
    coefficients = permute(polynomial.jerkPower_units_s3, [3 1 2]);
    order        = (1:size(coefficients, 1)).';
    gram         = 1 ./ (order + order.' - 1);
    segmentCosts = sum(coefficients .* pagemtimes(gram, coefficients), 1);
    cost_units2_s5 = sum(polynomial.SegmentDuration_s(:) .* reshape(segmentCosts, polynomial.SegmentCount, 2), "all");
end
