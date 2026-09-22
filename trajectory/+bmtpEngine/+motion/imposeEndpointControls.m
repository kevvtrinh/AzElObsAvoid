function controlPoint_units = imposeEndpointControls( ...
    controlPoint_units, segmentTime_s, initialState, goalState)
%% Section 0: Header & Readme
% SYNTAX
%   controlPoint_units = bmtpEngine.motion.imposeEndpointControls( ...
%       controlPoint_units, segmentTime_s, initialState, goalState)
%**************************************************************************
% PURPOSE
%   - Set the first and last three Bezier controls to match the requested
%     endpoint positions, velocities, and accelerations.
%**************************************************************************
% INPUTS
%   - controlPoint_units (S-by-(D+1)-by-2 numeric array)
%       Bezier control points for S motion segments.
%   - segmentTime_s (positive numeric vector)
%       Duration of each motion segment.
%   - initialState (scalar struct)
%       Initial position, velocity, and acceleration state.
%   - goalState (scalar struct)
%       Goal position, velocity, and acceleration state.
%**************************************************************************
% OUTPUTS
%   - controlPoint_units (S-by-(D+1)-by-2 numeric array)
%       Controls updated to match the two endpoint states.
%**************************************************************************
% UNITS
%   - Position is coordinate units and duration is seconds.
%**************************************************************************

%% Section 1: Set The Initial Position, Velocity, And Acceleration

degree = size(controlPoint_units, 2) - 1;

% The first three controls determine the starting state. With degree D
% and duration T, v0 = D x (P1 - P0) / T and
% a0 = D x (D - 1) x (P2 - 2 x P1 + P0) / T^2. Solve for P0, P1, P2.
% P0 means the first control point, stored at MATLAB index 1.
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

% Work backward from the goal to find the final three controls. Use the
% last segment's duration; it may differ from the first segment's duration.
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
