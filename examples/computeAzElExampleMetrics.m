function metrics = computeAzElExampleMetrics(result)
%% Section 0: Header & Readme
% SYNTAX
%   metrics = computeAzElExampleMetrics(result)
%**************************************************************************
% PURPOSE
%   - Compute the uniform chat-report metrics from one returned result.
%**************************************************************************
% INPUTS
%   - result (scalar planAzElMotion result)
%**************************************************************************
% OUTPUTS
%   - metrics (scalar struct)
%       Polyline length, motion length, duration policy, constraints, and seed.
%**************************************************************************
% UNITS
%   - Length is degrees and duration is seconds.
%**************************************************************************

%% Section 1: Compute Success Or Failure Metrics

goalTimeMode = string(result.Options.GoalTimeMode);
if goalTimeMode == "earliestArrival"
    durationInterpretation = "earliestValidatedDuration";
elseif goalTimeMode == "fixedArrival"
    durationInterpretation = "fixedArrivalDuration";
else
    error("computeAzElExampleMetrics:InvalidGoalTimeMode", ...
        "result.Options.GoalTimeMode must be earliestArrival or " + "fixedArrival.");
end
if result.Success
    selectedPolylineLength_deg = result.Seeds( result.SelectedSeedIndex).Length_deg;
    smoothedPathLength_deg = sum(vecnorm( diff(result.position_deg, 1, 1), 2, 2));
    motionDuration_s = result.time_s(end) - result.time_s(1);
    collisionFree = result.Validation.CollisionFree;
    kinematicCertificatePassed = result.Validation.VelocityWithinLimits && ...
        result.Validation.AccelerationWithinLimits && ...
        result.Validation.JerkWithinLimits && result.Validation.DynamicsConsistent;
    maximumAbsoluteVelocity_deg_s = max(abs(result.velocity_deg_s), [], "all");
    maximumAbsoluteAcceleration_deg_s2 = max(abs(result.acceleration_deg_s2), [], "all");
    maximumAbsoluteJerk_deg_s3 = max(abs(result.jerk_deg_s3), [], "all");
else
    selectedPolylineLength_deg = NaN;
    smoothedPathLength_deg = NaN;
    motionDuration_s = NaN;
    collisionFree = NaN;
    kinematicCertificatePassed = NaN;
    maximumAbsoluteVelocity_deg_s = NaN;
    maximumAbsoluteAcceleration_deg_s2 = NaN;
    maximumAbsoluteJerk_deg_s3 = NaN;
end
metrics = struct( ...
    "JerkConstraintEnabled", true, ...
    "GoalTimeMode", goalTimeMode, ...
    "SelectedPolylineLength_deg", selectedPolylineLength_deg, ...
    "SmoothedPathLength_deg", smoothedPathLength_deg, ...
    "MotionDuration_s", motionDuration_s, ...
    "MotionDurationInterpretation", durationInterpretation, ...
    "CollisionFree", collisionFree, ...
    "KinematicCertificatePassed", kinematicCertificatePassed, ...
    "MaximumAbsoluteVelocity_deg_s", maximumAbsoluteVelocity_deg_s, ...
    "MaximumAbsoluteAcceleration_deg_s2", ...
    maximumAbsoluteAcceleration_deg_s2, ...
    "MaximumAbsoluteJerk_deg_s3", maximumAbsoluteJerk_deg_s3, ...
    "SelectedSeedIndex", result.SelectedSeedIndex, ...
    "SeedCount", numel(result.Seeds), "TerminationReason", result.TerminationReason);
end
