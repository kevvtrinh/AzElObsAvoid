function [inequality, equality, inequalityGradient, equalityGradient] = ...
        evaluateHs3TrajectoryConstraints( ...
        decision, isEarliestArrival, fixedFinalTime_s, ...
        minimumFinalTime_s, maximumFinalTime_s, segmentCount, ...
        initialState, goalState, limits, options, obstacles, corridor, ...
        corridorTau, seedCorridor, reconstructFunction)
%% Section 0: Header & Readme
% SYNTAX
%   [inequality, equality] = ...
%       azElPlannerMethods.hs3.internal.motion.evaluateHs3TrajectoryConstraints( ...
%       decision, isEarliestArrival, fixedFinalTime_s, ...
%       minimumFinalTime_s, maximumFinalTime_s, segmentCount, ...
%       initialState, goalState, limits, options, obstacles, corridor, ...
%       corridorTau, seedCorridor, reconstructFunction)
%   [inequality, equality, inequalityGradient, equalityGradient] = ...
%       azElPlannerMethods.hs3.internal.motion.evaluateHs3TrajectoryConstraints( ...
%       decision, isEarliestArrival, fixedFinalTime_s, ...
%       minimumFinalTime_s, maximumFinalTime_s, segmentCount, ...
%       initialState, goalState, limits, options, obstacles, corridor, ...
%       corridorTau, seedCorridor, reconstructFunction)
%**************************************************************************
% PURPOSE
%   - Evaluate HS3 constraints with exact jerk Jacobian columns and a bounded
%     numerical final-time column for variable-duration optimization.
%**************************************************************************
% INPUTS
%   - decision (numeric column), axis-major jerk and optional final time.
%   - isEarliestArrival (logical scalar), selects the time decision.
%   - fixedFinalTime_s, minimumFinalTime_s, maximumFinalTime_s (seconds).
%   - segmentCount (positive integer scalar), HS3 mesh size.
%   - initialState, goalState, limits, options (resolved scalar structs).
%   - obstacles, corridor, seedCorridor (prepared geometry records).
%   - corridorTau (numeric column), normalized association times.
%   - reconstructFunction (function handle), exact HS3 polynomial builder.
%**************************************************************************
% OUTPUTS
%   - inequality, equality (numeric columns), feasible at c<=0 and ceq=0.
%   - inequalityGradient, equalityGradient (numeric matrices)
%       fmincon orientation: decision count by constraint count.
%**************************************************************************
% UNITS
%   - Constraint rows retain their physical units; time is seconds.
%**************************************************************************

%% Section 1: Evaluate Values And Exact Jerk Columns

[inequality, equality, corridorNormal, finalTime_s] = constraintValues( ...
    decision, isEarliestArrival, fixedFinalTime_s, segmentCount, ...
    initialState, goalState, limits, options, obstacles, corridor, ...
    corridorTau, seedCorridor, reconstructFunction);
if nargout < 3
    return;
end
duration_s = finalTime_s - initialState.time_s;
[inequalityMatrix, equalityMatrix] = ...
    azElPlannerMethods.hs3.internal.motion.buildFixedHs3ConstraintMatrices( ...
    segmentCount, duration_s, options.AllowAzimuthWrapping, ...
    seedCorridor, corridor, corridorTau, corridorNormal);
inequalityGradient = inequalityMatrix.';
equalityGradient = equalityMatrix.';
if ~isEarliestArrival
    return;
end

%% Section 2: Append One Safeguarded Final-Time Column

[differenceDirection, differenceStep_s] = timeDifferenceStep( ...
    finalTime_s, minimumFinalTime_s, maximumFinalTime_s, ...
    initialState.time_s, goalState, obstacles, corridor);
trialDecision = decision;
trialDecision(end) = finalTime_s + differenceDirection * differenceStep_s;
[trialInequality, trialEquality] = constraintValues( ...
    trialDecision, true, fixedFinalTime_s, segmentCount, ...
    initialState, goalState, limits, options, obstacles, corridor, ...
    corridorTau, seedCorridor, reconstructFunction);
if numel(trialInequality) ~= numel(inequality) || ...
        numel(trialEquality) ~= numel(equality)
    error("evaluateHs3TrajectoryConstraints:ConstraintLengthChanged", ...
        "A final-time perturbation changed the frozen constraint layout.");
end
if differenceDirection > 0
    timeInequalityGradient = (trialInequality - inequality) / differenceStep_s;
    timeEqualityGradient = (trialEquality - equality) / differenceStep_s;
else
    timeInequalityGradient = (inequality - trialInequality) / differenceStep_s;
    timeEqualityGradient = (equality - trialEquality) / differenceStep_s;
end
inequalityGradient(end + 1, :) = timeInequalityGradient.';
equalityGradient(end + 1, :) = timeEqualityGradient.';
end

function [inequality, equality, corridorNormal, finalTime_s] = ...
        constraintValues(decision, isEarliestArrival, fixedFinalTime_s, ...
        segmentCount, initialState, goalState, limits, options, obstacles, ...
        corridor, corridorTau, seedCorridor, reconstructFunction)
% Evaluate raw values without recursively requesting derivatives.
controlCount = 2 * segmentCount + 1;
jerkValueCount = 2 * controlCount;
jerk_deg_s3 = reshape(decision(1:jerkValueCount), controlCount, 2);
finalTime_s = fixedFinalTime_s;
if isEarliestArrival
    finalTime_s = decision(end);
end
polynomial = reconstructFunction(jerk_deg_s3, finalTime_s);
controlTime_s = initialState.time_s + ...
    corridorTau * (finalTime_s - initialState.time_s);
[~, controlPosition_deg] = azElInternal.evaluatePolynomial( ...
    polynomial, controlTime_s);
[corridorInequality, corridorNormal] = corridorConstraints( ...
    corridor, obstacles, controlTime_s, controlPosition_deg);
inequality = [continuousBoundConstraints(polynomial, limits, options); ...
    azElInternal.seedCorridorInequality(polynomial, seedCorridor); ...
    corridorInequality];
goalPosition_deg = azElInternal.goalPositionAtTime(goalState, finalTime_s);
terminalState = polynomial.TerminalState;
equality = [ ...
    terminalState.position_deg - goalPosition_deg, ...
    terminalState.velocity_deg_s - goalState.velocity_deg_s, ...
    terminalState.acceleration_deg_s2 - goalState.acceleration_deg_s2].';
end

function [direction, step_s] = timeDifferenceStep( ...
        finalTime_s, minimumFinalTime_s, maximumFinalTime_s, ...
        startTime_s, goalState, obstacles, corridor)
% Stay within decision bounds and one side of every known geometry event.
eventTime_s = zeros(0, 1);
if isfield(goalState, "targetTime_s") && ~isempty(goalState.targetTime_s)
    eventTime_s = double(goalState.targetTime_s(:));
end
for associationIndex = 1:numel(corridor)
    tau = corridor(associationIndex).Tau;
    if tau <= 0
        continue;
    end
    obstacleTime_s = double( ...
        obstacles(corridor(associationIndex).ObstacleIndex).time_s(:));
    eventTime_s = [eventTime_s; ...
        startTime_s + (obstacleTime_s - startTime_s) / tau]; %#ok<AGROW>
end
eventTime_s = unique(eventTime_s(isfinite(eventTime_s) & ...
    eventTime_s >= minimumFinalTime_s & eventTime_s <= maximumFinalTime_s));
scale_s = max(1, abs(finalTime_s));
eventTolerance_s = 64 * eps(scale_s);
previousEvent_s = max([minimumFinalTime_s; ...
    eventTime_s(eventTime_s < finalTime_s - eventTolerance_s)]);
nextEvent_s = min([maximumFinalTime_s; ...
    eventTime_s(eventTime_s > finalTime_s + eventTolerance_s)]);
backwardRoom_s = max(0, finalTime_s - previousEvent_s);
forwardRoom_s = max(0, nextEvent_s - finalTime_s);
baseStep_s = eps^(1 / 3) * scale_s;
atEvent = any(abs(eventTime_s - finalTime_s) <= eventTolerance_s);
if ~atEvent && forwardRoom_s >= baseStep_s
    direction = 1;
    step_s = baseStep_s;
    return;
end
if ~atEvent && backwardRoom_s >= baseStep_s
    direction = -1;
    step_s = baseStep_s;
    return;
end
if forwardRoom_s >= backwardRoom_s && forwardRoom_s > eventTolerance_s
    direction = 1;
    step_s = min(baseStep_s, 0.5 * forwardRoom_s);
elseif backwardRoom_s > eventTolerance_s
    direction = -1;
    step_s = min(baseStep_s, 0.5 * backwardRoom_s);
else
    error("evaluateHs3TrajectoryConstraints:NoTimeDifferenceRoom", ...
        "No nonzero final-time perturbation fits inside the current event interval.");
end
end

function inequality = continuousBoundConstraints(polynomial, limits, options)
% Use Bernstein convex-hull bounds over every complete HS3 segment.
positionViolation = bernsteinBoundViolations( ...
    polynomial.positionPower_deg, ...
    [limits.azimuthInterval_deg(1), limits.elevationInterval_deg(1)], ...
    [limits.azimuthInterval_deg(2), limits.elevationInterval_deg(2)]);
velocityViolation = bernsteinBoundViolations( ...
    polynomial.velocityPower_deg_s, -limits.maxVelocity_deg_s, ...
    limits.maxVelocity_deg_s);
accelerationViolation = bernsteinBoundViolations( ...
    polynomial.accelerationPower_deg_s2, ...
    -limits.maxAcceleration_deg_s2, limits.maxAcceleration_deg_s2);
jerkViolation = bernsteinBoundViolations( ...
    polynomial.jerkPower_deg_s3, -limits.maxJerk_deg_s3, ...
    limits.maxJerk_deg_s3);
if options.AllowAzimuthWrapping
    firstAxisViolation = [velocityViolation(:, 1, :); ...
        accelerationViolation(:, 1, :); jerkViolation(:, 1, :)];
else
    firstAxisViolation = [positionViolation(:, 1, :); ...
        velocityViolation(:, 1, :); accelerationViolation(:, 1, :); ...
        jerkViolation(:, 1, :)];
end
secondAxisViolation = [positionViolation(:, 2, :); ...
    velocityViolation(:, 2, :); accelerationViolation(:, 2, :); ...
    jerkViolation(:, 2, :)];
segmentCount = polynomial.SegmentCount;
inequalityBySegment = [ ...
    reshape(firstAxisViolation, [], segmentCount); ...
    reshape(secondAxisViolation, [], segmentCount)];
inequality = inequalityBySegment(:);
end

function violations = bernsteinBoundViolations( ...
        powerCoefficient, lowerBound, upperBound)
% Convert every segment and axis while retaining the established order.
segmentCount = size(powerCoefficient, 1);
coefficientCount = size(powerCoefficient, 3);
coefficientMatrix = reshape( ...
    permute(powerCoefficient, [3 1 2]), coefficientCount, []);
bernstein = azElInternal.powerToBernstein(coefficientMatrix);
bernstein = permute(reshape( ...
    bernstein, coefficientCount, segmentCount, 2), [1 3 2]);
violations = [bernstein - reshape(upperBound, 1, 2, 1); ...
    reshape(lowerBound, 1, 2, 1) - bernstein];
end

function [inequality, normalByAssociation] = corridorConstraints( ...
        corridor, obstacles, controlTime_s, controlPosition_deg)
% Evaluate each frozen association and retain its current jerk-space normal.
inequality = zeros(numel(corridor), 1);
normalByAssociation = zeros(numel(corridor), 2);
for associationIndex = 1:numel(corridor)
    association = corridor(associationIndex);
    controlIndex = association.ControlIndex;
    if association.GeometryIsFixed
        outwardNormal = association.FixedNormal;
        boundaryOffset_deg = association.FixedBoundaryOffset_deg;
    else
        [~, geometry] = azElInternal.obstacles.shapeAtTime( ...
            obstacles(association.ObstacleIndex), ...
            controlTime_s(controlIndex), true);
        if ~geometry.Active
            inequality(associationIndex) = -1;
            continue;
        end
        finiteVertices = isfinite(geometry.azimuth_deg) & ...
            isfinite(geometry.elevation_deg);
        vertices_deg = [geometry.azimuth_deg(finiteVertices), ...
            geometry.elevation_deg(finiteVertices)];
        if association.UseSupport
            outwardNormal = association.SupportNormal;
            boundaryOffset_deg = max(vertices_deg * outwardNormal.');
        else
            [edgeStart_deg, edgeEnd_deg] = geometryEdges(geometry);
            if association.EdgeIndex > size(edgeStart_deg, 1)
                inequality(associationIndex) = 1e3;
                continue;
            end
            edgeDelta_deg = edgeEnd_deg(association.EdgeIndex, :) - ...
                edgeStart_deg(association.EdgeIndex, :);
            if norm(edgeDelta_deg) <= eps
                inequality(associationIndex) = 1e3;
                continue;
            end
            leftNormal = [-edgeDelta_deg(2), edgeDelta_deg(1)] / ...
                norm(edgeDelta_deg);
            outwardNormal = association.OutwardSign * leftNormal;
            boundaryOffset_deg = edgeStart_deg(association.EdgeIndex, :) * ...
                outwardNormal.';
        end
    end
    normalByAssociation(associationIndex, :) = outwardNormal;
    pointProjection_deg = controlPosition_deg(controlIndex, :) * outwardNormal.';
    inequality(associationIndex) = boundaryOffset_deg + ...
        association.Clearance_deg - pointProjection_deg;
end
end

function [edgeStart_deg, edgeEnd_deg] = geometryEdges(geometry)
% Convert a NaN-separated canonical boundary to deterministic edges.
position_deg = [geometry.azimuth_deg, geometry.elevation_deg];
finiteRows = all(isfinite(position_deg), 2);
regionChanges = diff([false; finiteRows; false]);
regionStarts = find(regionChanges == 1);
regionStops = find(regionChanges == -1) - 1;
edgeStart_deg = zeros(0, 2);
edgeEnd_deg = zeros(0, 2);
for regionIndex = 1:numel(regionStarts)
    vertices_deg = position_deg( ...
        regionStarts(regionIndex):regionStops(regionIndex), :);
    if size(vertices_deg, 1) < 2
        continue;
    end
    if all(vertices_deg(1, :) == vertices_deg(end, :))
        vertices_deg(end, :) = [];
    end
    edgeStart_deg = [edgeStart_deg; vertices_deg]; %#ok<AGROW>
    edgeEnd_deg = [edgeEnd_deg; ...
        vertices_deg([2:end 1], :)]; %#ok<AGROW>
end
end
