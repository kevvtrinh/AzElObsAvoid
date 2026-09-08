function controls_units = imposeEndpointControls(controls_units, durations_s, initialState, goalState)
%% Section 0: Header & Readme
% SYNTAX: controls = bmtpEngine.imposeEndpointControls(controls,times,initial,goal)
% PURPOSE: Express requested physical boundary states in the Bernstein basis.
% INPUTS: Composite controls, physical span durations, and full endpoint states.
% OUTPUTS: Controls with endpoint position, velocity, and acceleration imposed.
% UNITS: Coordinate units and seconds.

%% Section 1: Use Each Endpoint Span's Physical Duration
degree = size(controls_units,2)-1;
h = durations_s(1);
p = initialState.position_units; v = initialState.velocity_units_s; a = initialState.acceleration_units_s2;
controls_units(1,1:3,:) = reshape([p;p+h*v/degree;p+2*h*v/degree+h^2*a/(degree*(degree-1))],1,3,2);
h = durations_s(end);
p = goalState.position_units; v = goalState.velocity_units_s; a = goalState.acceleration_units_s2;
controls_units(end,end-2:end,:) = reshape([p-2*h*v/degree+h^2*a/(degree*(degree-1));p-h*v/degree;p],1,3,2);
end
