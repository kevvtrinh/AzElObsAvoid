function [inequality, equality] = evaluateAzElHs3NlpConstraints( ...
        decision, layout, meshTau, corridor, obstacleField, ...
        initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   [inequality, equality] = ...
%       azElInternal.evaluateAzElHs3NlpConstraints( ...
%       decision, layout, meshTau, corridor, obstacleField, ...
%       initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Evaluate terminal, continuous kinematic, and obstacle constraints.
%**************************************************************************
% INPUTS
%   - decision (N-by-1 numeric vector)
%       Reduced HS-3 final-time and jerk decision.
%   - layout, corridor, obstacleField (scalar structs)
%       Decision indices, frozen separators, and canonical obstacles.
%   - meshTau (M-by-1 numeric vector)
%       Strictly increasing normalized knot times.
%   - initialState, goalState, limits, options (scalar structs)
%       Resolved endpoint data, physical limits, and planner controls.
%**************************************************************************
% OUTPUTS
%   - inequality (P-by-1 numeric vector)
%       Feasible values are less than or equal to zero.
%   - equality (6-by-1 numeric vector)
%       Terminal state defect.
%**************************************************************************
% UNITS
%   - Constraint values retain their state or derivative units.
%**************************************************************************

%% Section 1: Propagate The Reduced Decision

solution = azElInternal.unpackAzElHs3Decision( ...
    decision, layout, meshTau, initialState, goalState);
goalEndpoint = [goalState.position_deg, goalState.velocity_deg_s, ...
    goalState.acceleration_deg_s2];
terminalStateDefect = solution.KnotState(end, :) - goalEndpoint;
equality = terminalStateDefect(:);

%% Section 2: Evaluate Obstacle And Kinematic Bounds

obstacleInequality = azElInternal.evaluateAzElHs3CorridorConstraints( ...
    solution, meshTau, corridor, obstacleField, initialState.time_s, ...
    options.ObstacleConstraintTolerance_deg);
kinematicInequality = continuousKinematicConstraints( ...
    solution, meshTau, limits, options);
inequality = [obstacleInequality; kinematicInequality];
end

%% Section 3: Local Functions

function inequality = continuousKinematicConstraints( ...
        solution, meshTau, limits, options)
% PURPOSE
%   - Bound state and quadratic jerk through Bernstein control hulls.
segmentCount = numel(meshTau) - 1;
inequality = zeros(segmentCount * 76, 1);
writeIndex = 0;
duration_s = solution.FinalTime_s - solution.InitialTime_s;
azimuthBounds_deg = [-Inf Inf];
if ~options.AllowAzimuthWrapping
    azimuthBounds_deg = options.AzimuthInterval_deg;
end
stateLower = [azimuthBounds_deg(1), ...
    options.ElevationInterval_deg(1), ...
    -limits.maxVelocity_deg_s, -limits.maxAcceleration_deg_s2];
stateUpper = [azimuthBounds_deg(2), ...
    options.ElevationInterval_deg(2), ...
    limits.maxVelocity_deg_s, limits.maxAcceleration_deg_s2];
for segmentIndex = 1:segmentCount
    segmentDuration_s = duration_s * ...
        (meshTau(segmentIndex + 1) - meshTau(segmentIndex));
    firstControl = solution.KnotControl(segmentIndex, :);
    midpointControl = solution.MidpointControl(segmentIndex, :);
    lastControl = solution.KnotControl(segmentIndex + 1, :);
    statePower = azElInternal.buildAzElHs3SegmentPolynomials( ...
        solution.KnotState(segmentIndex, :), firstControl, ...
        midpointControl, lastControl, segmentDuration_s);
    stateBernstein = azElInternal.powerToBernstein(statePower);
    finiteStateBound = isfinite(stateLower) & isfinite(stateUpper);
    lowerViolation = stateLower(finiteStateBound) - ...
        stateBernstein(:, finiteStateBound);
    upperViolation = stateBernstein(:, finiteStateBound) - ...
        stateUpper(finiteStateBound);
    stateRows = [lowerViolation(:); upperViolation(:)];
    stateRowCount = numel(stateRows);
    inequality(writeIndex + (1:stateRowCount)) = stateRows;
    writeIndex = writeIndex + stateRowCount;
    linearControl = -3 * firstControl + ...
        4 * midpointControl - lastControl;
    interiorControlBernstein = firstControl + linearControl / 2;
    controlRows = [ ...
        -limits.maxJerk_deg_s3 - interiorControlBernstein; ...
        interiorControlBernstein - limits.maxJerk_deg_s3];
    inequality(writeIndex + (1:numel(controlRows))) = controlRows(:);
    writeIndex = writeIndex + numel(controlRows);
end
inequality = inequality(1:writeIndex);
end
