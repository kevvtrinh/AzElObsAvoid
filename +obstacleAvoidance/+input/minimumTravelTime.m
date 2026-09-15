function duration_s = minimumTravelTime(initialState, goalState, limits)
%% Section 0: Header & Readme
% SYNTAX
%   duration_s = ...
%       obstacleAvoidance.input.minimumTravelTime(initialState, goalState, limits)
%**************************************************************************
% PURPOSE
%   - Bound obstacle-free travel time to a fixed terminal position.
%**************************************************************************
% INPUTS
%   - initialState (scalar struct)
%       Validated initial endpoint state.
%   - goalState (scalar struct)
%       Validated goal endpoint state.
%   - limits (scalar struct)
%       Normalized per-axis velocity, acceleration, and jerk limits.
%**************************************************************************
% OUTPUTS
%   - duration_s (nonnegative scalar)
%       Necessary duration bound, not an attainable C3 motion duration;
%       invalid input throws an error.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Bound The Displacement By The Velocity Limit

displacement_units = abs(goalState.position_units - initialState.position_units);
duration_s          = max(displacement_units ./ limits.maxVelocity_units_s);

endpointDerivatives = [initialState.velocity_units_s, initialState.acceleration_units_s2, ...
    goalState.velocity_units_s, goalState.acceleration_units_s2];
if any(endpointDerivatives ~= 0)
    return
end

%% Section 2: Tighten The Bound With Exact Rest-To-Rest Axis Motion

% Allowing jerk jumps enlarges the C3 feasible set. The exact C2 axis time
% is therefore a necessary bound shared by endpoint checks and arrival search.
for axisIndex = 1:2
    if initialState.position_units(axisIndex) == goalState.position_units(axisIndex)
        continue;
    end
    axisLimits = struct( ...
        'maxVelocity_units_s',      repmat(limits.maxVelocity_units_s(axisIndex), 1, 2), ...
        'maxAcceleration_units_s2', repmat(limits.maxAcceleration_units_s2(axisIndex), 1, 2), ...
        'maxJerk_units_s3',         repmat(limits.maxJerk_units_s3(axisIndex), 1, 2));
    initialAxisState = [initialState.position_units(axisIndex), 0];
    goalAxisState    = [goalState.position_units(axisIndex), 0];
    [~, times_s]     = bmtpEngine.createJerkLimitedChord(initialAxisState, goalAxisState, axisLimits, 3);
    duration_s       = max(duration_s, sum(times_s));
end
end
