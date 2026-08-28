function mesh = createHybridActivityMesh( ...
        candidate, ruckigProfile, limits, maximumSegmentCount)
%% Section 0: Header & Readme
% SYNTAX
%   mesh = obstacleAvoidance.planner.createHybridActivityMesh( ...
%       candidate, ruckigProfile, limits, maximumSegmentCount)
%**************************************************************************
% PURPOSE
%   - Create one bounded nonuniform HS3 refinement when a coarse motion has
%     Ruckig-scale endpoint switching and a long velocity-limited interior.
%**************************************************************************
% INPUTS
%   - candidate (scalar planner-candidate struct)
%       Internal validated candidate produced by solveRouteCandidate.
%   - ruckigProfile (scalar pass-through profile struct)
%       Internal stable engine result whose segment durations provide the
%       switching-time scale. It is evidence, not an accepted obstacle path.
%   - limits (scalar planner-limits struct)
%       Normalized positive two-axis velocity, acceleration, and jerk limits.
%   - maximumSegmentCount (positive integer scalar)
%       Resolved planner upper work bound.
%**************************************************************************
% OUTPUTS
%   - mesh (scalar struct)
%       Stable decision record containing Applied, Reason, activity ratios,
%       SegmentCount, and SegmentBreakTau.
%**************************************************************************
% UNITS
%   - Polynomial time is seconds. Derivatives use deg/s, deg/s^2, and
%     deg/s^3. SegmentBreakTau and activity ratios are dimensionless.
%**************************************************************************

%% Section 1: Read Validated Planner State

% This is an internal policy helper with one production caller. The public
% planner already normalizes these records, and solveRouteCandidate owns the
% polynomial invariant. Revalidating them here duplicated boundary ownership.
maximumVelocity_deg_s = reshape( ...
    double(limits.maxVelocity_deg_s), 1, []);
maximumAcceleration_deg_s2 = reshape( ...
    double(limits.maxAcceleration_deg_s2), 1, []);
maximumJerk_deg_s3 = reshape(double(limits.maxJerk_deg_s3), 1, []);

polynomial = candidate.Polynomial;
segmentCount = double(polynomial.SegmentCount);
segmentStartTime_s = double(polynomial.SegmentStartTime_s(:));
segmentDuration_s = double(polynomial.SegmentDuration_s(:));
if isscalar(segmentDuration_s)
    segmentDuration_s = repmat(segmentDuration_s, segmentCount, 1);
end
totalDuration_s = polynomial.FinalTime_s - segmentStartTime_s(1);
segmentBreakTau = [segmentStartTime_s; polynomial.FinalTime_s];
segmentBreakTau = (segmentBreakTau - segmentStartTime_s(1)) / ...
    totalDuration_s;

%% Section 2: Measure Coarse Hs3 Activity

activitySampleCount = 9;
velocityUtilization = zeros(segmentCount, 1);
accelerationUtilization = zeros(segmentCount, 1);
jerkUtilization = zeros(segmentCount, 1);
for segmentIndex = 1:segmentCount
    evaluationTime_s = linspace( ...
        segmentStartTime_s(segmentIndex), ...
        segmentStartTime_s(segmentIndex) + ...
        segmentDuration_s(segmentIndex), activitySampleCount).';
    [~, ~, velocity_deg_s, acceleration_deg_s2, jerk_deg_s3] = ...
        obstacleAvoidance.planner.evaluatePlannerPolynomial( ...
        polynomial, evaluationTime_s, segmentIndex);
    velocityUtilization(segmentIndex) = max( ...
        abs(velocity_deg_s) ./ maximumVelocity_deg_s, [], "all");
    accelerationUtilization(segmentIndex) = max( ...
        abs(acceleration_deg_s2) ./ maximumAcceleration_deg_s2, [], "all");
    jerkUtilization(segmentIndex) = max( ...
        abs(jerk_deg_s3) ./ maximumJerk_deg_s3, [], "all");
end

% The product distinguishes a true endpoint switching layer from an isolated
% jerk pulse with ample acceleration reserve. A factor-of-two dominance is a
% scale-free separation from every interior interval. The 75% reserve matches
% the planner's established derivative-slack decision.
activityScore = accelerationUtilization .* jerkUtilization;
endpointScore = activityScore([1 end]);
interiorMaximumScore = max(activityScore(2:end - 1));
endpointDominance = min(endpointScore) / max(interiorMaximumScore, eps);
derivativeActivityThreshold = 0.75;
hasEndpointDynamics = endpointDominance >= 2 && ...
    all(accelerationUtilization([1 end]) >= ...
    derivativeActivityThreshold) && ...
    all(jerkUtilization([1 end]) >= derivativeActivityThreshold / 2);
velocityCruiseThreshold = 0.95;
cruiseFraction = mean(velocityUtilization(2:end - 1) >= ...
    velocityCruiseThreshold);

%% Section 3: Resolve Ruckig Switching Resolution

ruckigSegmentDuration_s = double( ...
    ruckigProfile.Polynomial.SegmentDuration_s(:));
ruckigDuration_s = ruckigProfile.FinalTime_s - ruckigProfile.time_s(1);
hasValidSwitchingScale = isfinite(ruckigDuration_s) && ...
    ruckigDuration_s > 0 && ~isempty(ruckigSegmentDuration_s) && ...
    all(isfinite(ruckigSegmentDuration_s)) && ...
    all(ruckigSegmentDuration_s > 0);
minimumSwitchIntervalTau = NaN;
if hasValidSwitchingScale
    minimumSwitchIntervalTau = min(ruckigSegmentDuration_s) / ...
        ruckigDuration_s;
end

mesh = struct( ...
    "Applied", false, ...
    "Reason", "activityNotSeparated", ...
    "BaseSegmentCount", segmentCount, ...
    "SegmentCount", segmentCount, ...
    "SegmentBreakTau", segmentBreakTau, ...
    "MinimumRuckigSwitchIntervalTau", minimumSwitchIntervalTau, ...
    "VelocityUtilization", velocityUtilization, ...
    "AccelerationUtilization", accelerationUtilization, ...
    "JerkUtilization", jerkUtilization, ...
    "EndpointDominance", endpointDominance, ...
    "VelocityCruiseFraction", cruiseFraction);
if ~hasEndpointDynamics || cruiseFraction < 0.5
    return;
end
if ~hasValidSwitchingScale
    mesh.Reason = "missingRuckigSwitchingScale";
    return;
end
if maximumSegmentCount <= segmentCount
    mesh.Reason = "segmentLimitReached";
    return;
end

%% Section 4: Create A Bounded Symmetric Refinement

endpointWidthTau = max([diff(segmentBreakTau(1:2)), ...
    diff(segmentBreakTau(end - 1:end))]);
% The smallest whole count whose local width does not exceed the shortest
% Ruckig switching phase is the discrete knee. A denser guard subdivision
% was valid in the bounded comparison but bought only a few milliseconds.
requestedSubdivisionCount = ...
    ceil(endpointWidthTau / minimumSwitchIntervalTau);
availableSubdivisionCount = ...
    floor((maximumSegmentCount - segmentCount) / 2) + 1;
subdivisionCount = min(requestedSubdivisionCount, ...
    availableSubdivisionCount);
if subdivisionCount < 2
    mesh.Reason = "segmentLimitReached";
    return;
end

startInteriorTau = linspace(segmentBreakTau(1), ...
    segmentBreakTau(2), subdivisionCount + 1).';
endInteriorTau = linspace(segmentBreakTau(end - 1), ...
    segmentBreakTau(end), subdivisionCount + 1).';
refinedBreakTau = sort([segmentBreakTau; ...
    startInteriorTau(2:end - 1); endInteriorTau(2:end - 1)]);
mesh.Applied = true;
mesh.Reason = "endpointSwitchingResolved";
mesh.SegmentCount = numel(refinedBreakTau) - 1;
mesh.SegmentBreakTau = refinedBreakTau;
end
