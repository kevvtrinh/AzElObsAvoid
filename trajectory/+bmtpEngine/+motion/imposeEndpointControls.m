function controlPoint_units = imposeEndpointControls( ...
    controlPoint_units, segmentTime_s, initialState, goalState)
%% Section 0: Header & Readme
% SYNTAX
%   controlPoint_units = bmtpEngine.motion.imposeEndpointControls( ...
%       controlPoint_units, segmentTime_s, initialState, goalState)
%**************************************************************************
% PURPOSE
%   - Replace the controls at the start and end of a proposed Bezier motion
%     so its endpoint position, velocity, and acceleration match the supplied
%     states. The changed curve still needs later motion checks.
%**************************************************************************
% INPUTS
%   - controlPoint_units (S-by-(D+1)-by-2 numeric array)
%       S is the number of segments; each degree-D curve has D+1 controls
%       on two position axes. The planner uses D at least 5 so one segment
%       can hold separate sets of three controls at its two ends.
%   - segmentTime_s (S-element positive numeric vector)
%       Physical duration of each segment, in input order.
%   - initialState (scalar struct)
%       Starting position_units, velocity_units_s, and
%       acceleration_units_s2 on the two axes.
%   - goalState (scalar struct)
%       Requested final values for the same three fields.
%**************************************************************************
% OUTPUTS
%   - controlPoint_units (S-by-(D+1)-by-2 numeric array)
%       Controls with the first and last three points set from the endpoint
%       states. Other control points retain their supplied values.
%**************************************************************************
% UNITS
%   - Position and controls use coordinate units; velocity is units/s,
%     acceleration is units/s^2, and duration is seconds.
%**************************************************************************

%% Section 1: Set The Initial Position, Velocity, And Acceleration

degree = size(controlPoint_units, 2) - 1;

% The first three controls set the starting state. For degree D and time T,
% v0 = D x (P1 - P0) / T and a0 = D x (D - 1) x (P2 - 2 x P1 + P0) / T^2.
% Solve these for P0, P1, and P2. On one axis, D = 5, T = 10 s, and
% v0 = 2 units/s put P1 four units past P0. MATLAB stores P0 at index 1.
endpointSegmentTime_s = segmentTime_s(1);
position_units        = initialState.position_units;
velocity_units_s      = initialState.velocity_units_s;
acceleration_units_s2 = initialState.acceleration_units_s2;
initialControlPoints_units = [position_units; ...
    position_units + endpointSegmentTime_s * velocity_units_s / degree; ...
    position_units + 2 * endpointSegmentTime_s * velocity_units_s / degree + ...
    endpointSegmentTime_s ^ 2 * acceleration_units_s2 / (degree * (degree - 1))];
controlPoint_units(1, 1:3, :) = reshape(initialControlPoints_units, 1, 3, 2);

%% Section 2: Set The Goal Position, Velocity, And Acceleration

% Work backward from the goal state using the last segment's duration.
% On one axis, D = 5, T = 10 s, and goal velocity 2 units/s put the
% next-to-last control four units behind the goal position. The first
% segment may have a different duration, so it uses its own time above.
endpointSegmentTime_s = segmentTime_s(end);
position_units        = goalState.position_units;
velocity_units_s      = goalState.velocity_units_s;
acceleration_units_s2 = goalState.acceleration_units_s2;
goalControlPoints_units = [position_units - 2 * endpointSegmentTime_s * velocity_units_s / degree + ...
    endpointSegmentTime_s ^ 2 * acceleration_units_s2 / (degree * (degree - 1)); ...
    position_units - endpointSegmentTime_s * velocity_units_s / degree; ...
    position_units];
controlPoint_units(end, end - 2:end, :) = reshape(goalControlPoints_units, 1, 3, 2);
end
