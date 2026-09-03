function [candidate, diagnostics] = createDirectWaitMotion( ...
        seed, initialState, goalState, limits, options, ...
        waitOverride_s, directMotionDuration_s)
%% Section 0: Header & Readme
% SYNTAX
%   [candidate, diagnostics] = ...
%       obstacleAvoidance.planner.createDirectWaitMotion( ...
%       seed, initialState, goalState, limits, options, ...
%       waitOverride_s, directMotionDuration_s)
%**************************************************************************
% PURPOSE
%   - Realize a timed dwell followed by one exact direct motion.
%   - Reject unsupported timed multi-waypoint topology with explicit details.
%**************************************************************************
% INPUTS
%   - seed (scalar route-seed struct)
%       Timed route with positions, normalized times, source, and index.
%   - initialState, goalState (scalar structs)
%       Normalized endpoint motion states.
%   - limits (scalar struct)
%       Independent-axis physical limits.
%   - options (resolved scalar struct)
%       Sampling and explicit fallback policy.
%   - waitOverride_s (empty or nonnegative scalar)
%       Optional refined dwell duration; [] uses the seed timing.
%   - directMotionDuration_s (empty or nonnegative scalar)
%       Direct-motion duration paired with a supplied wait override.
%**************************************************************************
% OUTPUTS
%   - candidate (scalar motion struct)
%       Delayed exact motion or stable failure record.
%   - diagnostics (scalar struct)
%       Wait, unsupported-feature, fallback-policy, and timing details.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds.
%**************************************************************************

%% Section 1: Check The Timed Seed Form

% The compact construction can represent only an initial dwell followed by a
% direct move. Identify unsupported multi-waypoint topology here so the caller
% can apply only the explicitly selected backup policy.

timer = tic;
candidate = bmtpEngine.createMotionRecord( ...
    struct(), initialState, [], [], options.SampleTime_s, seed.Source);
candidate.SeedIndex = seed.Index;
diagnostics = struct("Accepted", false, "ElapsedTime_s", 0, ...
    "TerminationReason", "unsupportedTimedMultiWaypointRoute", ...
    "WaitTime_s", NaN, "InitialWaitTime_s", NaN, ...
    "FinalWaitTime_s", NaN, "RefinementCount", 0, ...
    "InfeasibleLowerWaitTime_s", NaN, ...
    "EstimatedDuration_s", seed.EstimatedDuration_s, ...
    "HorizonRetryAttempted", false, ...
    "TimingRepairAttempted", false, ...
    "ExactMinimumDirectDuration_s", NaN, ...
    "InitialTimingTerminationReason", "", ...
    "SeedIndex", seed.Index, "SeedSource", string(seed.Source), ...
    "WaypointPosition_deg", seed.position_deg, "Tau", seed.tau, ...
    "ContainsWait", hasRepeatedWaypoint(seed.position_deg), ...
    "FirstUnsupportedTransitionIndex", 0, ...
    "FirstUnsupportedFeature", "", ...
    "OriginalTerminationReason", "", ...
    "FallbackPolicy", options.UnsupportedTimedTopologyPolicy, ...
    "FallbackAvailable", true, "FallbackAttempted", false, ...
    "FallbackMethod", "", "FallbackOutcome", "notApplicable", ...
    "AllInteriorWaypointsConstrainedToRest", false);
if string(seed.Source) ~= "directWait"
    diagnostics.FirstUnsupportedTransitionIndex = 1;
    diagnostics.FirstUnsupportedFeature = "multiWaypointTimedRoute";
    diagnostics.OriginalTerminationReason = diagnostics.TerminationReason;
    diagnostics.FallbackOutcome = "fallbackDisabledByPolicy";
    candidate.Message = ...
        "The compact dynamic kernel currently requires a direct-wait seed.";
    candidate.TerminationReason = diagnostics.TerminationReason;
    candidate.SolverDiagnostics = diagnostics;
    diagnostics.ElapsedTime_s = toc(timer);
    return;
end

coordinateScale_deg = bmtpEngine.createCoordinateTolerances( ...
    seed.position_deg, initialState.position_deg, goalState.position_deg);
positionTolerance_deg = 256 * eps(coordinateScale_deg);
isInitialPosition = vecnorm( ...
    seed.position_deg - initialState.position_deg, 2, 2) <= ...
    positionTolerance_deg;
firstMotionIndex = find(~isInitialPosition, 1, "first");
isDirectWait = ~isempty(firstMotionIndex) && firstMotionIndex > 1 && ...
    all(vecnorm(seed.position_deg(firstMotionIndex:end, :) - ...
    goalState.position_deg, 2, 2) <= positionTolerance_deg);
if ~isDirectWait
    diagnostics.TerminationReason = "invalidDirectWaitSeed";
    diagnostics.FirstUnsupportedTransitionIndex = firstMotionIndex;
    diagnostics.FirstUnsupportedFeature = "nonDirectMotionAfterWait";
    diagnostics.OriginalTerminationReason = diagnostics.TerminationReason;
    diagnostics.FallbackOutcome = "fallbackDisabledByPolicy";
    candidate.Message = ...
        "The timed seed is not a dwell followed by a direct edge.";
    candidate.TerminationReason = diagnostics.TerminationReason;
    candidate.SolverDiagnostics = diagnostics;
    diagnostics.ElapsedTime_s = toc(timer);
    return;
end

%% Section 2: Create The Delayed Direct Motion

% Resolve the dwell and direct interval in physical time, then let the shared
% direct engine create the motion segment under a fixed-arrival request.

duration_s = double(seed.EstimatedDuration_s);
waitTime_s = duration_s * double(seed.tau(firstMotionIndex - 1));
if ~isempty(waitOverride_s)
    waitTime_s = waitOverride_s;
    duration_s = waitTime_s + directMotionDuration_s;
end
delayedInitialState = initialState;
delayedInitialState.time_s = initialState.time_s + waitTime_s;
delayedGoalState = goalState;
delayedGoalState.time_s = initialState.time_s + duration_s;
fixedOptions = options;
fixedOptions.GoalTimeMode = "fixedArrival";
direct = bmtpEngine.createDirectMotion( ...
    delayedInitialState, delayedGoalState, limits, fixedOptions);
canRetryAtHorizon = isempty(waitOverride_s) && ...
    options.GoalTimeMode ~= "fixedArrival" && ...
    duration_s < goalState.time_s - initialState.time_s - ...
    options.ArrivalTimeTolerance_s;
if ~direct.Success && canRetryAtHorizon
    diagnostics.HorizonRetryAttempted = true;
    diagnostics.InitialTimingTerminationReason = direct.TerminationReason;
    duration_s = goalState.time_s - initialState.time_s;
    delayedGoalState.time_s = initialState.time_s + duration_s;
    direct = bmtpEngine.createDirectMotion( ...
        delayedInitialState, delayedGoalState, limits, fixedOptions);
end
if ~direct.Success && isempty(waitOverride_s) && waitTime_s > 0
    % Search-layer timing is only a proposal. Recover the largest dwell that
    % leaves the exact direct kernel enough motion time, so an optimistic
    % transition estimate cannot turn an ample request horizon into failure.
    diagnostics.TimingRepairAttempted = true;
    if strlength(diagnostics.InitialTimingTerminationReason) == 0
        diagnostics.InitialTimingTerminationReason = ...
            direct.TerminationReason;
    end
    minimumOptions = options;
    minimumOptions.GoalTimeMode = "earliestArrival";
    minimumDirect = bmtpEngine.createDirectMotion( ...
        initialState, goalState, limits, minimumOptions);
    if minimumDirect.Success
        minimumDirectDuration_s = minimumDirect.MotionDuration_s;
        diagnostics.ExactMinimumDirectDuration_s = ...
            minimumDirectDuration_s;
        repairedWaitTime_s = max(0, duration_s - minimumDirectDuration_s);
        waitWasReduced = repairedWaitTime_s < waitTime_s - ...
            options.ArrivalTimeTolerance_s;
        if waitWasReduced
            waitTime_s = repairedWaitTime_s;
            delayedInitialState.time_s = initialState.time_s + waitTime_s;
            direct = bmtpEngine.createDirectMotion( ...
                delayedInitialState, delayedGoalState, limits, fixedOptions);
        end
    end
end
if ~direct.Success
    candidate = direct;
    candidate.SeedIndex = seed.Index;
    candidate.SeedSource = string(seed.Source);
    diagnostics.TerminationReason = direct.TerminationReason;
    diagnostics.ElapsedTime_s = toc(timer);
    candidate.SolverDiagnostics = diagnostics;
    return;
end

%% Section 3: Prepend The Constant Dwell

% Express wait and motion through one polynomial record so later validation
% sees the complete physical history instead of a hidden time offset.

directBreak_s = [direct.Polynomial.SegmentStartTime_s; ...
    direct.Polynomial.FinalTime_s] - delayedInitialState.time_s;
directJerk_deg_s3 = reshape(direct.Polynomial.jerkPower_deg_s3, ...
    direct.Polynomial.SegmentCount, numel(initialState.position_deg));
if waitTime_s > 0
    relativeBreak_s = [0; waitTime_s + directBreak_s];
    segmentJerk_deg_s3 = [zeros( ...
        1, numel(initialState.position_deg)); directJerk_deg_s3];
else
    relativeBreak_s = directBreak_s;
    segmentJerk_deg_s3 = directJerk_deg_s3;
end
candidate = bmtpEngine.createMotionRecord( ...
    direct, initialState, relativeBreak_s, segmentJerk_deg_s3, ...
    options.SampleTime_s, seed.Source);
candidate.SeedIndex = seed.Index;
candidate.Message = ...
    "An exact direct motion was realized after the timed dwell.";
[candidate.Success, candidate.OptimizerFeasible] = deal(true);
candidate.TerminationReason = "goalReached";
diagnostics.Accepted = true;
diagnostics.TerminationReason = candidate.TerminationReason;
diagnostics.WaitTime_s = waitTime_s;
diagnostics.InitialWaitTime_s = waitTime_s;
diagnostics.FinalWaitTime_s = waitTime_s;
diagnostics.ElapsedTime_s = toc(timer);
candidate.SolverDiagnostics = diagnostics;
end

%% Section 4: Local Functions

function hasWait = hasRepeatedWaypoint(position_deg)
% Treat a repeated consecutive guide point as an explicit spatial dwell.
if size(position_deg, 1) < 2
    hasWait = false;
    return;
end
coordinateScale_deg = bmtpEngine.createCoordinateTolerances(position_deg);
duplicateTolerance_deg = 256 * eps(coordinateScale_deg);
hasWait = any(vecnorm(diff(position_deg), 2, 2) <= ...
    duplicateTolerance_deg);
end
