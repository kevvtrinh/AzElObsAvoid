function [candidate, diagnostics] = solveClockCorridor(seed, geometry, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX: [candidate, diagnostics] = bmtpEngine.solveClockCorridor(seed, geometry, initialState, goalState, limits, options)
% PURPOSE: Generate a monotone static corridor proposal on an analytic clock.
% INPUTS: Normalized request, spatial seed, and exact prepared static regions.
% OUTPUTS: Motion proposal and explicit eligibility or solve evidence.
%   Acceptance requires the public independent trajectory validator.
% UNITS: Coordinate units and seconds, with physical derivatives.

%% Section 1: Check The Reduced Formulation's Eligibility
candidate = bmtpEngine.createMotionRecord(struct(), initialState, [], [], options.SampleTime_s, seed.Source);
candidate.SeedIndex = seed.Index;
diagnostics = struct('Identifier', "monotoneStaticCorridor", 'Attempted', false, ...
    'Accepted', false, 'TerminationReason', "inapplicableClockCorridor");
% This replacement targets attainment of the independent-axis lower bound.
% Prescribed arrivals retain the broader solver: fixing one stretched clock
% did not replace work there in the matched performance experiment.
if options.GoalTimeMode ~= "earliestArrival"
    diagnostics.TerminationReason = "prescribedArrivalUsesGeneralSolver";
    candidate.TerminationReason = diagnostics.TerminationReason;
    return;
end
endpointDerivatives = [initialState.velocity_units_s, initialState.acceleration_units_s2, ...
    goalState.velocity_units_s, goalState.acceleration_units_s2];
if any(endpointDerivatives ~= 0) || any(~isfinite(limits.maxJerk_units_s3)) || isempty(geometry.ExactRegions_units) || ...
        (isfield(goalState, 'targetTime_s') && ~isempty(goalState.targetTime_s))
    candidate.TerminationReason = diagnostics.TerminationReason;
    return;
end
base = bmtpEngine.createDirectMotion(initialState, goalState, limits, options);
if ~base.Success
    candidate.TerminationReason = base.TerminationReason;
    return;
end
[~, axisIndex] = max(base.MinimumAxisDuration_s);
if abs(diff(base.MinimumAxisDuration_s)) <= 64 * eps(max(1, max(base.MinimumAxisDuration_s)))
    candidate.TerminationReason = "tiedPhysicalClocks";
    diagnostics.TerminationReason = candidate.TerminationReason;
    return;
end
direction = sign(goalState.position_units(axisIndex) - initialState.position_units(axisIndex));
if direction == 0 || any(direction * diff(seed.position_units(:, axisIndex)) <= 0)
    candidate.TerminationReason = "nonMonotoneClockGuide";
    diagnostics.TerminationReason = candidate.TerminationReason;
    return;
end

%% Section 2: Refine The Analytic Clock At The Visibility Guide Vertices
guideCoordinate_units = seed.position_units(2:end - 1, axisIndex);
lower_s = repmat(initialState.time_s, size(guideCoordinate_units));
upper_s = repmat(base.ArrivalTime_s, size(guideCoordinate_units));
for iteration = 1:48
    middle_s = (lower_s + upper_s) / 2;
    if isempty(middle_s), break; end
    [~, position_units] = bmtpEngine.evaluatePolynomial(base.Polynomial, middle_s);
    before = direction * position_units(:, axisIndex) < direction * guideCoordinate_units;
    lower_s(before) = middle_s(before);
    upper_s(~before) = middle_s(~before);
end
breaks_s = sort([base.Polynomial.SegmentStartTime_s; base.ArrivalTime_s; (lower_s + upper_s) / 2]);
breaks_s = breaks_s([true; diff(breaks_s) > 64 * eps(max(1, max(abs(breaks_s))))]);
durations_s = diff(breaks_s);
segmentCount = numel(durations_s);
[~, position_units, velocity_units_s, acceleration_units_s2, jerk_units_s3] = ...
    bmtpEngine.evaluatePolynomial(base.Polynomial, breaks_s(1:end - 1));
fixedPower_units = zeros(segmentCount, 2, 9);
fixedPower_units(:, :, 1) = position_units;
fixedPower_units(:, :, 2) = velocity_units_s .* durations_s;
fixedPower_units(:, :, 3) = acceleration_units_s2 .* durations_s .^ 2 / 2;
fixedPower_units(:, :, 4) = jerk_units_s3 .* durations_s .^ 3 / 6;
warmStart = struct('SegmentCount', segmentCount, 'SegmentTime_s', durations_s, ...
    'AxisMinimumTime_s', base.MinimumAxisDuration_s, 'FixedPower_units', fixedPower_units, ...
    'ClockGuide', struct('Route_units', seed.position_units));
request = bmtpEngine.createSolveRequest(seed, geometry.ExactRegions_units, geometry.ExactCoverage, initialState, goalState, limits, options);
[~, ~, reserve_units] = bmtpEngine.createCoordinateTolerances(seed.position_units, geometry.ExactRegions_units, ...
    limits.xInterval_units, limits.yInterval_units);
target_units = (1 + 2 ^ 20 * eps) * options.CollisionClearanceTolerance_units + reserve_units;
diagnostics.Attempted = true;
[motion, diagnostics] = bmtpEngine.solveStaticCorridor(request, warmStart, diagnostics, target_units, reserve_units);
if ~motion.Success
    candidate.TerminationReason = diagnostics.TerminationReason;
    return;
end

%% Section 3: Export And Certify The Complete Polynomial
preparedMotion = bmtpEngine.prepareFinalMotion(request, motion.ControlPoint_units, motion.SegmentTime_s, motion.PositionPower_units);
if ~preparedMotion.Success
    candidate.TerminationReason = preparedMotion.TerminationReason;
    return;
end
polynomial = bmtpEngine.createPowerPolynomial(preparedMotion.ControlPoint_units, preparedMotion.SegmentTime_s, initialState.time_s, preparedMotion.PrescribedPower_units);
candidate = bmtpEngine.createMotionRecord(candidate, initialState, polynomial, [], options.SampleTime_s, seed.Source);
candidate.PlaneCertificate = bmtpEngine.checkFinalMotion(request, ...
    struct('RegionActiveBySegment', true(segmentCount, numel(request.Regions_units))), ...
    preparedMotion, reserve_units, target_units);
candidate.Success = candidate.PlaneCertificate.Passed;
candidate.MinimumAxisDuration_s = base.MinimumAxisDuration_s;
diagnostics.Accepted = candidate.Success;
diagnostics.Coverage = geometry.ExactCoverage;
candidate.OptimizerFeasible = true;
candidate.MaximumConstraintViolation = preparedMotion.MotionCertificate.MaximumViolation;
candidate.TerminationReason = "clockCorridorProposal";
diagnostics.TerminationReason = candidate.TerminationReason;
candidate.Message = "The monotone static corridor returned a motion proposal requiring independent validation.";
candidate.SolverDiagnostics = diagnostics;
end
