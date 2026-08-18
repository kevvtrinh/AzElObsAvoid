function solution = unpackAzElHs3Decision( ...
        decision, layout, meshTau, initialState, goalState)
%% Section 0: Header & Readme
% SYNTAX
%   solution = azElInternal.unpackAzElHs3Decision( ...
%       decision, layout, meshTau, initialState, goalState)
%**************************************************************************
% PURPOSE
%   - Integrate all HS-3 states from one reduced decision vector.
%**************************************************************************
% INPUTS
%   - decision (N-by-1 numeric vector)
%       Optional final time and packed jerk ordinates.
%   - layout (scalar struct)
%       Decision indices from azElHs3DecisionLayout.
%   - meshTau (M-by-1 numeric vector)
%       Strictly increasing normalized knot times.
%   - initialState, goalState (scalar structs)
%       Endpoint time, position, velocity, and acceleration states.
%**************************************************************************
% OUTPUTS
%   - solution (scalar struct)
%       Propagated knot states, midpoint states, and jerk controls.
%**************************************************************************
% UNITS
%   - Position is degrees. Time is seconds. Derivative units follow order.
%**************************************************************************

%% Section 1: Unpack And Propagate The Decision

segmentCount = numel(meshTau) - 1;
if layout.HasFinalTimeVariable
    finalTime_s = decision(layout.FinalTimeIndex);
else
    finalTime_s = goalState.time_s;
end
if finalTime_s < initialState.time_s
    finalTime_s = initialState.time_s;
end
knotControl = reshape( ...
    decision(layout.KnotControlIndex), segmentCount + 1, 2);
midpointControl = reshape( ...
    decision(layout.MidpointControlIndex), segmentCount, 2);
initialEndpoint = [initialState.position_deg, ...
    initialState.velocity_deg_s, initialState.acceleration_deg_s2];
solution = azElInternal.propagateAzElHs3Control( ...
    initialState.time_s, finalTime_s, meshTau, initialEndpoint, ...
    knotControl, midpointControl);
end
