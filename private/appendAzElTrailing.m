function command = appendAzElTrailing(command, goal, options, limits, ...
        arrivalTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   command = appendAzElTrailing(command, goal, options, limits, ...
%       arrivalTime_s)
%**************************************************************************
% PURPOSE
%   - Append coherent moving-goal tracking after first capture without
%     changing the arrival metric.
%**************************************************************************
% INPUTS
%   - command (scalar piecewise-quintic command ending at capture)
%   - goal (normalized moving-goal struct)
%   - options (scalar resolved mission options)
%   - limits (scalar physical limits)
%   - arrivalTime_s (finite capture time)
%**************************************************************************
% OUTPUTS
%   - command (scalar piecewise-quintic command with optional trailing)
%**************************************************************************
% UNITS
%   - Angles are degrees and time is seconds.

if options.trailingDuration_s <= options.arrivalTolerance_s
    return;
end
if goal.type ~= "moving"
    error("appendAzElTrailing:FixedGoalTrailing", ...
        "Trailing duration is supported only for a sampled moving goal.");
end
trailingEndTime_s = arrivalTime_s + options.trailingDuration_s;
internalGoalTimes_s = goal.time_s(goal.time_s > arrivalTime_s & ...
    goal.time_s < trailingEndTime_s);
trailingTime_s = [internalGoalTimes_s; trailingEndTime_s];

trailingPosition_deg = zeros(numel(trailingTime_s), 2);
trailingVelocity_deg_s = zeros(numel(trailingTime_s), 2);
trailingAcceleration_deg_s2 = zeros(numel(trailingTime_s), 2);
for sampleIndex = 1:numel(trailingTime_s)
    goalState = evaluateAzElGoal(goal, trailingTime_s(sampleIndex));
    trailingPosition_deg(sampleIndex, :) = goalState.position_deg;
    trailingVelocity_deg_s(sampleIndex, :) = goalState.velocity_deg_s;
    trailingAcceleration_deg_s2(sampleIndex, :) = ...
        goalState.acceleration_deg_s2;
end

command.time_s = [command.time_s; trailingTime_s];
command.unwrappedPosition_deg = [command.unwrappedPosition_deg; ...
    trailingPosition_deg];
command.velocity_deg_s = [command.velocity_deg_s; ...
    trailingVelocity_deg_s];
command.acceleration_deg_s2 = [command.acceleration_deg_s2; ...
    trailingAcceleration_deg_s2];
command.position_deg = command.unwrappedPosition_deg;
if options.azimuthWrap
    span_deg = diff(limits.azimuth_deg);
    command.position_deg(:, 1) = limits.azimuth_deg(1) + mod( ...
        command.unwrappedPosition_deg(:, 1) - limits.azimuth_deg(1), ...
        span_deg);
end
end
