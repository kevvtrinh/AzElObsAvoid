function controls_units = imposeEndpointControls(controls_units, durations_s, initialState, goalState)
%% Section 0: Header & Readme
% SYNTAX
%   controls_units = bmtpEngine.motion.imposeEndpointControls(controls_units, durations_s, initialState, goalState)
%**************************************************************************
% PURPOSE
%   - Express requested physical boundary states in the Bernstein basis.
%**************************************************************************
% INPUTS
%   - controls_units (S-by-(D+1)-by-2 numeric array)
%       Composite Bezier control points for S motion spans.
%   - durations_s (positive numeric vector)
%       Physical duration of each motion span.
%   - initialState (scalar struct)
%       Initial position, velocity, and acceleration state.
%   - goalState (scalar struct)
%       Goal position, velocity, and acceleration state.
%**************************************************************************
% OUTPUTS
%   - controls_units (S-by-(D+1)-by-2 numeric array)
%       Controls with endpoint position, velocity, and acceleration imposed.
%**************************************************************************
% UNITS
%   - Position is coordinate units and duration is seconds.
%**************************************************************************

%% Section 1: Use Each Endpoint Span's Physical Duration

degree = size(controls_units, 2) - 1;

duration_s            = durations_s(1);
position_units        = initialState.position_units;
velocity_units_s      = initialState.velocity_units_s;
acceleration_units_s2 = initialState.acceleration_units_s2;
initialTriple_units   = [position_units; ...
    position_units + duration_s * velocity_units_s / degree; ...
    position_units + 2 * duration_s * velocity_units_s / degree + ...
    duration_s ^ 2 * acceleration_units_s2 / (degree * (degree - 1))];
controls_units(1, 1:3, :) = reshape(initialTriple_units, 1, 3, 2);

duration_s            = durations_s(end);
position_units        = goalState.position_units;
velocity_units_s      = goalState.velocity_units_s;
acceleration_units_s2 = goalState.acceleration_units_s2;
goalTriple_units      = [position_units - 2 * duration_s * velocity_units_s / degree + ...
    duration_s ^ 2 * acceleration_units_s2 / (degree * (degree - 1)); ...
    position_units - duration_s * velocity_units_s / degree; ...
    position_units];
controls_units(end, end - 2:end, :) = reshape(goalTriple_units, 1, 3, 2);
end
