function summary = createCandidateSummary( ...
        candidate, checkResult, diagnostics, elapsedTime_s, template, ...
        limits, options, initialTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   summary = obstacleAvoidance.planner.createCandidateSummary( ...
%       candidate, checkResult, diagnostics, elapsedTime_s, template, ...
%       limits, options, initialTime_s)
%**************************************************************************
% PURPOSE
%   - Copy solve and authoritative-check evidence into a stable candidate row.
%   - Calculate objective and utilization fields only for passing motions.
%**************************************************************************
% INPUTS
%   - candidate (scalar motion struct)
%       Motion returned by a production engine or explicit backup method.
%   - checkResult (scalar validation struct)
%       Authoritative result from obstacleAvoidance.validateTrajectory.
%   - diagnostics (scalar struct)
%       Complete solver and fallback details for this candidate.
%   - elapsedTime_s (nonnegative finite scalar)
%       Wall-clock motion-solving duration excluding validation.
%   - template (scalar candidate-summary struct)
%       Stable empty summary returned by createEmptyResult.
%   - limits (scalar struct)
%       Physical limits used to normalize peak motion measures.
%   - options (resolved scalar struct)
%       Goal-time policy and declared travel-time tradeoff.
%   - initialTime_s (finite scalar)
%       Physical start time used for elapsed-arrival cost.
%**************************************************************************
% OUTPUTS
%   - summary (scalar candidate-summary struct)
%       Stable selection and diagnostic evidence for one attempted seed.
%**************************************************************************
% UNITS
%   - Time is seconds; position and length are degrees; derivative units are
%     deg/s, deg/s^2, and deg/s^3.
%**************************************************************************

%% Section 1: Copy Candidate And Check Evidence

summary = template;
sourceNames = ["SeedIndex", "SeedSource", "OptimizerFeasible", ...
    "FinalTime_s", "MotionDuration_s", "MotionLength_deg", ...
    "IntegratedSquaredJerk_deg2_s5", "MaximumConstraintViolation", ...
    "TerminationReason"];
targetNames = ["SeedIndex", "SeedSource", "OptimizerFeasible", ...
    "ArrivalTime_s", "MotionDuration_s", "MotionLength_deg", ...
    "IntegratedSquaredJerk_deg2_s5", "MaximumConstraintViolation", ...
    "TerminationReason"];
for fieldIndex = 1:numel(sourceNames)
    summary.(targetNames(fieldIndex)) = ...
        candidate.(sourceNames(fieldIndex));
end
summary.ValidationPassed = checkResult.Passed;
summary.CollisionFree = checkResult.CollisionFree;
summary.CollisionResolved = checkResult.CollisionResolved;
summary.MinimumClearance_deg = checkResult.MinimumClearance_deg;
summary.UnresolvedIntervalCount = checkResult.UnresolvedIntervalCount;
summary.SeedPlanningElapsedTime_s = elapsedTime_s;
summary.Message = strtrim( ...
    string(candidate.Message) + " " + checkResult.Message);
summary.SolverDiagnostics = diagnostics;

%% Section 2: Calculate Passing-Candidate Measures

% Selection metrics are meaningful only after the full check passes. A failed
% motion retains its diagnostic evidence and receives the established failure
% reason without being eligible for ranking.

if checkResult.Passed
    normalizedPeaks = [checkResult.PeakVelocity_deg_s ./ ...
        limits.maxVelocity_deg_s, ...
        checkResult.PeakAcceleration_deg_s2 ./ ...
        limits.maxAcceleration_deg_s2, ...
        checkResult.PeakJerk_deg_s3 ./ limits.maxJerk_deg_s3];
    summary.KinematicUtilization = mean(normalizedPeaks);
    summary.TravelTimeTradeoffCost_deg = candidate.MotionLength_deg + ...
        options.MinimumTravelSavingsRate_deg_s * ...
        (candidate.FinalTime_s - initialTime_s);
end
if ~checkResult.Passed && ~isempty(candidate.time_s)
    summary.TerminationReason = "independentValidationFailed";
end
end
