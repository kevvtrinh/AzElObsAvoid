function decision = packAzElHs3Decision(solution, layout)
%% Section 0: Header & Readme
% SYNTAX
%   decision = azElInternal.packAzElHs3Decision(solution, layout)
%**************************************************************************
% PURPOSE
%   - Pack an HS-3 solution into the reduced decision layout.
%**************************************************************************
% INPUTS
%   - solution (scalar struct)
%       Final time and knot and midpoint jerk controls.
%   - layout (scalar struct)
%       Decision indices from azElHs3DecisionLayout.
%**************************************************************************
% OUTPUTS
%   - decision (N-by-1 numeric vector)
%       Optional final time followed by packed jerk ordinates.
%**************************************************************************
% UNITS
%   - Final time is seconds. Jerk is degrees per second cubed.
%**************************************************************************

%% Section 1: Pack The Decision

decision = zeros(layout.DecisionCount, 1);
if layout.HasFinalTimeVariable
    decision(layout.FinalTimeIndex) = solution.FinalTime_s;
end
decision(layout.KnotControlIndex) = solution.KnotControl(:);
decision(layout.MidpointControlIndex) = solution.MidpointControl(:);
end
