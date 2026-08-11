function goalState = evaluateAzElGoal(goal, queryTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   goalState = evaluateAzElGoal(goal, queryTime_s)
%**************************************************************************
% PURPOSE
%   - Evaluate a fixed or linearly sampled moving complete-state goal.
%**************************************************************************
% INPUTS
%   - goal (normalized scalar struct)
%   - queryTime_s (finite scalar)
%**************************************************************************
% OUTPUTS
%   - goalState (scalar state struct)
%**************************************************************************
% UNITS
%   - Angles are degrees and time is seconds.

if goal.type == "fixed"
    goalState = struct( ...
        "time_s", queryTime_s, ...
        "position_deg", goal.unwrappedPosition_deg, ...
        "velocity_deg_s", goal.velocity_deg_s, ...
        "acceleration_deg_s2", goal.acceleration_deg_s2);
    return;
end
if queryTime_s < goal.time_s(1) || queryTime_s > goal.time_s(end)
    error("evaluateAzElGoal:QueryOutsideGoal", ...
        "Moving-goal query is outside its sampled interval.");
end
lowerIndex = find(goal.time_s <= queryTime_s, 1, "last");
if lowerIndex == numel(goal.time_s)
    lowerIndex = lowerIndex - 1;
end
upperIndex = lowerIndex + 1;
segmentStart_s = goal.time_s(lowerIndex);
segmentDuration_s = goal.time_s(upperIndex) - segmentStart_s;
localTime_s = queryTime_s - segmentStart_s;
startState = struct( ...
    "position_deg", goal.unwrappedPosition_deg(lowerIndex, :), ...
    "velocity_deg_s", goal.velocity_deg_s(lowerIndex, :), ...
    "acceleration_deg_s2", goal.acceleration_deg_s2(lowerIndex, :));
endState = struct( ...
    "position_deg", goal.unwrappedPosition_deg(upperIndex, :), ...
    "velocity_deg_s", goal.velocity_deg_s(upperIndex, :), ...
    "acceleration_deg_s2", goal.acceleration_deg_s2(upperIndex, :));
coefficients = quinticHermiteCoefficients( ...
    startState, endState, segmentDuration_s);
goalState = struct( ...
    "time_s", queryTime_s, ...
    "position_deg", evaluateGoalPolynomial(coefficients, localTime_s, 0), ...
    "velocity_deg_s", evaluateGoalPolynomial(coefficients, localTime_s, 1), ...
    "acceleration_deg_s2", evaluateGoalPolynomial( ...
        coefficients, localTime_s, 2));
end

function value = evaluateGoalPolynomial(coefficients, localTime_s, order)
%% Section 0: Header & Readme
% SYNTAX
%   value = evaluateGoalPolynomial(coefficients, localTime_s, order)
%**************************************************************************
% PURPOSE
%   - Evaluate a moving-goal quintic state coherently at one local time.
%**************************************************************************
% INPUTS
%   - coefficients (6-by-2 ascending-power numeric)
%   - localTime_s (nonnegative scalar)
%   - order (0, 1, or 2)
%**************************************************************************
% OUTPUTS
%   - value (1-by-2 numeric)
%**************************************************************************
% UNITS
%   - Units follow derivative order from degrees and seconds.

workingCoefficients = coefficients;
for derivativeIndex = 1:order
    powers = (1:(size(workingCoefficients, 1) - 1)).';
    workingCoefficients = workingCoefficients(2:end, :) .* powers;
end
powers = localTime_s .^ (0:(size(workingCoefficients, 1) - 1));
value = powers * workingCoefficients;
end
