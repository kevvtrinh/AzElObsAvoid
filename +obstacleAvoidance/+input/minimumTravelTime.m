function duration_s = minimumTravelTime(initialState, goalState, limits)
%% Section 0: Header & Readme
% SYNTAX
%   duration_s = ...
%       obstacleAvoidance.input.minimumTravelTime(initialState, goalState, limits)
%**************************************************************************
% PURPOSE
%   - Calculate a minimum time below which the vehicle cannot reach a fixed
%     goal position. Obstacles and the full motion requirements may add time.
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
%       Travel cannot take less than this many seconds. This calculation
%       does not guarantee that a valid motion can achieve the returned time.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Estimate Time From Distance And Maximum Speed

% time = distance / maximum speed. Both axes must finish, so use the larger
% of the x and y times. This first estimate ignores acceleration and jerk.
displacement_units = abs(goalState.position_units - initialState.position_units);
duration_s         = max(displacement_units ./ limits.maxVelocity_units_s);

% The stronger calculation below requires zero velocity and acceleration at
% both endpoints. Otherwise, keep the distance / speed estimate.
endpointVelocityAndAcceleration = [initialState.velocity_units_s, initialState.acceleration_units_s2, ...
    goalState.velocity_units_s, goalState.acceleration_units_s2];
if any(endpointVelocityAndAcceleration ~= 0)
    return
end

%% Section 2: Include Acceleration And Jerk For Stationary Endpoints

% Find the fastest motion on each axis when jerk may change instantly.
% The planner requires continuous jerk, which can take more time but cannot
% beat this less restrictive calculation.
for axisIndex = 1:2
    if initialState.position_units(axisIndex) == goalState.position_units(axisIndex)
        continue;
    end

    % The motion helper accepts [x y] positions. Place this axis's movement
    % in x and hold y at zero to calculate its travel time on its own.
    axisLimits = struct( ...
        'maxVelocity_units_s',      repmat(limits.maxVelocity_units_s(axisIndex), 1, 2), ...
        'maxAcceleration_units_s2', repmat(limits.maxAcceleration_units_s2(axisIndex), 1, 2), ...
        'maxJerk_units_s3',         repmat(limits.maxJerk_units_s3(axisIndex), 1, 2));
    initialAxisPosition_units = [initialState.position_units(axisIndex), 0];
    goalAxisPosition_units    = [goalState.position_units(axisIndex), 0];
    [~, axisSegmentDurations_s] = bmtpEngine.motion.createJerkLimitedChord( ...
        initialAxisPosition_units, goalAxisPosition_units, axisLimits, 3);
    duration_s = max(duration_s, sum(axisSegmentDurations_s));
end
end
