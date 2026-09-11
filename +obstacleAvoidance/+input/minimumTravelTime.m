function duration_s = minimumTravelTime(initialState, goalState, limits)
%% Section 0: Header & Readme
% SYNTAX: duration_s = obstacleAvoidance.input.minimumTravelTime(initial,goal,limits)
% PURPOSE: Bound travel time without obstacles for a fixed terminal position.
% INPUTS: Validated states and normalized per-axis velocity/acceleration/jerk limits.
% OUTPUTS: Necessary duration bound, not an attainable C3 motion duration.
% UNITS: Coordinate units, seconds, and physical derivatives.

%% Section 1: Bound Displacement And Rest-To-Rest Axis Motion
duration_s = max(abs(goalState.position_units-initialState.position_units)./limits.maxVelocity_units_s);
if any([initialState.velocity_units_s,initialState.acceleration_units_s2, ...
        goalState.velocity_units_s,goalState.acceleration_units_s2]~=0), return; end
% Allowing jerk jumps enlarges the C3 feasible set. The exact C2 axis time
% is therefore a necessary bound shared by endpoint checks and arrival search.
for axis=1:2
    if initialState.position_units(axis)==goalState.position_units(axis), continue; end
    axisLimits = struct('maxVelocity_units_s',repmat(limits.maxVelocity_units_s(axis),1,2), ...
        'maxAcceleration_units_s2',repmat(limits.maxAcceleration_units_s2(axis),1,2), ...
        'maxJerk_units_s3',repmat(limits.maxJerk_units_s3(axis),1,2));
    [~,times_s] = bmtpEngine.createJerkLimitedChord([initialState.position_units(axis),0], ...
        [goalState.position_units(axis),0],axisLimits,3);
    duration_s = max(duration_s,sum(times_s));
end
end
