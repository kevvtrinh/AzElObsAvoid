function refined = resampleAzElHs3Solution( ...
        solution, oldMeshTau, newMeshTau)
%% Section 0: Header & Readme
% SYNTAX
%   refined = azElInternal.resampleAzElHs3Solution( ...
%       solution, oldMeshTau, newMeshTau)
%**************************************************************************
% PURPOSE
%   - Warm-start an h-refined mesh from corrected HS-3 polynomials.
%**************************************************************************
% INPUTS
%   - solution (scalar struct)
%       Current HS-3 knot and midpoint solution.
%   - oldMeshTau (N-by-1 numeric vector)
%       Current normalized collocation mesh.
%   - newMeshTau (M-by-1 numeric vector)
%       Refined normalized collocation mesh.
%**************************************************************************
% OUTPUTS
%   - refined (scalar struct)
%       HS-3 state and control values sampled on the refined mesh.
%**************************************************************************
% UNITS
%   - Time is seconds. State and control units follow the solution fields.
%**************************************************************************

%% Section 1: Sample The Refined Mesh

newMidpointTau = 0.5 * (newMeshTau(1:end - 1) + newMeshTau(2:end));
[newKnotState, newKnotControl] = azElInternal.sampleAzElHs3Solution( ...
    solution, oldMeshTau, newMeshTau);
[newMidpointState, newMidpointControl] = ...
    azElInternal.sampleAzElHs3Solution( ...
    solution, oldMeshTau, newMidpointTau);
refined = struct( ...
    "InitialTime_s", solution.InitialTime_s, ...
    "FinalTime_s", solution.FinalTime_s, ...
    "KnotState", newKnotState, ...
    "MidpointState", newMidpointState, ...
    "KnotControl", newKnotControl, ...
    "MidpointControl", newMidpointControl);
end
