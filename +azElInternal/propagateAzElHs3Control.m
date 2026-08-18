function solution = propagateAzElHs3Control( ...
        initialTime_s, finalTime_s, meshTau, firstState, knotControl, ...
        midpointControl)
%% Section 0: Header & Readme
% SYNTAX
%   solution = azElInternal.propagateAzElHs3Control( ...
%       initialTime_s, finalTime_s, meshTau, firstState, knotControl, ...
%       midpointControl)
%**************************************************************************
% PURPOSE
%   - Derive every HS-3 knot and midpoint state from one quadratic-jerk
%     control chain and one initial state.
%**************************************************************************
% INPUTS
%   - initialTime_s, finalTime_s (finite numeric scalars)
%       Motion interval with finalTime_s not less than initialTime_s.
%   - meshTau (increasing finite numeric vector)
%       Normalized knot locations from zero to one.
%   - firstState (1-by-6 finite numeric row)
%       Initial position, velocity, and acceleration by axis.
%   - knotControl ((N+1)-by-2 finite numeric matrix)
%       Jerk at each mesh knot.
%   - midpointControl (N-by-2 finite numeric matrix)
%       Jerk at each mesh midpoint.
%**************************************************************************
% OUTPUTS
%   - solution (scalar struct)
%       InitialTime_s, FinalTime_s, KnotState, MidpointState, KnotControl,
%       and MidpointControl for the dynamically consistent chain.
%**************************************************************************
% UNITS
%   - Time is seconds. State columns use degrees, deg/s, and deg/s^2.
%     Control columns use deg/s^3. Axis order is [azimuth elevation].
%**************************************************************************

%% Section 1: Propagate The Control Chain

segmentCount = numel(meshTau) - 1;
solution = struct( ...
    "InitialTime_s", initialTime_s, ...
    "FinalTime_s", finalTime_s, ...
    "KnotState", zeros(segmentCount + 1, 6), ...
    "MidpointState", zeros(segmentCount, 6), ...
    "KnotControl", knotControl, ...
    "MidpointControl", midpointControl);
solution.KnotState(1, :) = firstState;
duration_s = finalTime_s - initialTime_s;
for segmentIndex = 1:segmentCount
    segmentDuration_s = duration_s * ...
        (meshTau(segmentIndex + 1) - meshTau(segmentIndex));
    statePower = azElInternal.buildAzElHs3SegmentPolynomials( ...
        solution.KnotState(segmentIndex, :), ...
        knotControl(segmentIndex, :), ...
        midpointControl(segmentIndex, :), ...
        knotControl(segmentIndex + 1, :), segmentDuration_s);
    localTau = [0.5; 1];
    power = localTau .^ (0:size(statePower, 1) - 1);
    state = power * statePower;
    solution.MidpointState(segmentIndex, :) = state(1, :);
    solution.KnotState(segmentIndex + 1, :) = state(2, :);
end
end
