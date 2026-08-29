function [inequality, equality, inequalityGradient, equalityGradient] = ...
        evaluateTrajectoryConstraints( ...
        decision, isEarliestArrival, fixedFinalTime_s, ...
        minimumFinalTime_s, maximumFinalTime_s, segmentCount, ...
        initialState, goalState, limits, options, obstacles, corridor, ...
        corridorTau, seedCorridor, reconstructFunction, ...
        collinearityDirection, constraintLayout)
%% Section 0: Header & Readme
% SYNTAX
%   [inequality, equality] = ...
%       obstacleAvoidance.planner.evaluateTrajectoryConstraints( ...
%       decision, isEarliestArrival, fixedFinalTime_s, ...
%       minimumFinalTime_s, maximumFinalTime_s, segmentCount, ...
%       initialState, goalState, limits, options, obstacles, corridor, ...
%       corridorTau, seedCorridor, reconstructFunction)
%   [inequality, equality] = ...
%       obstacleAvoidance.planner.evaluateTrajectoryConstraints( ...
%       decision, isEarliestArrival, fixedFinalTime_s, ...
%       minimumFinalTime_s, maximumFinalTime_s, segmentCount, ...
%       initialState, goalState, limits, options, obstacles, corridor, ...
%       corridorTau, seedCorridor, reconstructFunction, ...
%       collinearityDirection)
%   [inequality, equality] = ...
%       obstacleAvoidance.planner.evaluateTrajectoryConstraints( ...
%       decision, isEarliestArrival, fixedFinalTime_s, ...
%       minimumFinalTime_s, maximumFinalTime_s, segmentCount, ...
%       initialState, goalState, limits, options, obstacles, corridor, ...
%       corridorTau, seedCorridor, reconstructFunction, ...
%       collinearityDirection, constraintLayout)
%   [inequality, equality, inequalityGradient, equalityGradient] = ...
%       obstacleAvoidance.planner.evaluateTrajectoryConstraints( ...
%       decision, isEarliestArrival, fixedFinalTime_s, ...
%       minimumFinalTime_s, maximumFinalTime_s, segmentCount, ...
%       initialState, goalState, limits, options, obstacles, corridor, ...
%       corridorTau, seedCorridor, reconstructFunction)
%   [inequality, equality, inequalityGradient, equalityGradient] = ...
%       obstacleAvoidance.planner.evaluateTrajectoryConstraints( ...
%       decision, isEarliestArrival, fixedFinalTime_s, ...
%       minimumFinalTime_s, maximumFinalTime_s, segmentCount, ...
%       initialState, goalState, limits, options, obstacles, corridor, ...
%       corridorTau, seedCorridor, reconstructFunction, ...
%       collinearityDirection)
%   [inequality, equality, inequalityGradient, equalityGradient] = ...
%       obstacleAvoidance.planner.evaluateTrajectoryConstraints( ...
%       decision, isEarliestArrival, fixedFinalTime_s, ...
%       minimumFinalTime_s, maximumFinalTime_s, segmentCount, ...
%       initialState, goalState, limits, options, obstacles, corridor, ...
%       corridorTau, seedCorridor, reconstructFunction, ...
%       collinearityDirection, constraintLayout)
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
%   - collinearityDirection (1-by-2 numeric row, optional; default empty)
%       Optional direct-path direction. When supplied, the normal endpoint
%       equations are replaced by exact normal-jerk equations.
%   - constraintLayout (scalar struct, optional; default empty)
%       Precomputed corridor maps, row offsets, and final-time events.
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

% The decision vector is axis-major jerk followed, for earliest-arrival
% planning, by final time. Constraint values are evaluated first. Their jerk
% derivatives are exact because HS3 state histories are affine functions of
% jerk when duration is fixed. Only the final-time derivative needs a finite
% difference, since changing duration changes both polynomial scaling and the
% physical time at which moving obstacles are queried.

if nargin < 16
    collinearityDirection = zeros(0, 2);
end
if nargin < 17
    constraintLayout = struct();
end
% Interior-point restoration can probe a nonfinite duration after reaching a
% finite feasible iterate. Map only that observed invalid probe to its bound.
if isEarliestArrival && ~isfinite(decision(end))
    % Return a finite value to the optimizer's restoration step. Clamping
    % only this invalid probe avoids contaminating ordinary feasible iterates.
    if isnan(decision(end))
        decision(end) = mean([minimumFinalTime_s, maximumFinalTime_s]);
    else
        timeBounds_s = [minimumFinalTime_s, maximumFinalTime_s];
        decision(end) = timeBounds_s(1 + (decision(end) > 0));
    end
end
[inequality, equality, corridorNormal, finalTime_s] = constraintValues( ...
    decision, isEarliestArrival, fixedFinalTime_s, segmentCount, ...
    initialState, goalState, limits, options, obstacles, corridor, ...
    seedCorridor, reconstructFunction, collinearityDirection, ...
    constraintLayout);
if nargout < 3
    return;
end
duration_s = finalTime_s - initialState.time_s;
[inequalityMatrix, equalityMatrix] = ...
    obstacleAvoidance.planner.createConstraintMatrices( ...
    segmentCount, duration_s, options.AllowAzimuthWrapping, ...
    seedCorridor, corridor, corridorTau, corridorNormal, ...
    collinearityDirection, constraintLayout);
inequalityGradient = inequalityMatrix.';
equalityGradient = equalityMatrix.';
if ~isEarliestArrival
    return;
end

%% Section 2: Append One Safeguarded Final-Time Column

% Geometry changes at obstacle or moving-goal event times. A difference that
% crosses one of those events could compare two different constraint layouts
% and would not approximate a local derivative. Select a one-sided step that
% remains inside the current event interval, then confirm row counts agree.

[differenceDirection, differenceStep_s] = timeDifferenceStep( ...
    finalTime_s, minimumFinalTime_s, maximumFinalTime_s, ...
    initialState.time_s, goalState, obstacles, corridor, constraintLayout);
trialDecision = decision;
trialDecision(end) = finalTime_s + differenceDirection * differenceStep_s;
[trialInequality, trialEquality] = constraintValues( ...
    trialDecision, true, fixedFinalTime_s, segmentCount, ...
    initialState, goalState, limits, options, obstacles, corridor, ...
    seedCorridor, reconstructFunction, collinearityDirection, ...
    constraintLayout);
if numel(trialInequality) ~= numel(inequality) || ...
        numel(trialEquality) ~= numel(equality)
    error("evaluateTrajectoryConstraints:ConstraintLengthChanged", ...
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
        corridor, seedCorridor, reconstructFunction, ...
        collinearityDirection, constraintLayout)
% Evaluate raw values without recursively requesting derivatives.
% Jerk values are reshaped into one column per physical axis. Reconstruction
% integrates them from the supplied initial position, velocity, and
% acceleration to obtain the continuous piecewise-polynomial motion.
controlCount = 2 * segmentCount + 1;
jerkValueCount = 2 * controlCount;
jerk_deg_s3 = reshape(decision(1:jerkValueCount), controlCount, 2);
finalTime_s = fixedFinalTime_s;
if isEarliestArrival
    finalTime_s = decision(end);
end
polynomial = reconstructFunction(jerk_deg_s3, finalTime_s);
[corridorInequality, corridorNormal] = corridorConstraints( ...
    corridor, obstacles, polynomial, initialState.time_s, finalTime_s, ...
    constraintLayout);
inequality = [continuousBoundConstraints(polynomial, limits, options); ...
    obstacleAvoidance.search.seedCorridorInequality(polynomial, seedCorridor); ...
    corridorInequality];
goalPosition_deg = obstacleAvoidance.input.goalPositionAtTime(goalState, finalTime_s);
terminalState = polynomial.TerminalState;
terminalResidual = [ ...
    terminalState.position_deg - goalPosition_deg; ...
    terminalState.velocity_deg_s - goalState.velocity_deg_s; ...
    terminalState.acceleration_deg_s2 - goalState.acceleration_deg_s2];
if isempty(collinearityDirection)
    % General routes require both axes of terminal position, velocity, and
    % acceleration to match the requested terminal state.
    equality = reshape(terminalResidual.', [], 1);
else
    % A certified direct route uses tangent endpoint residuals and zero
    % normal jerk. This removes redundant endpoint equations while keeping
    % the entire motion on the straight line.
    tangent = collinearityDirection(:);
    normal = [-tangent(2); tangent(1)];
    equality = [terminalResidual * tangent; jerk_deg_s3 * normal];
end
end
function [direction, step_s] = timeDifferenceStep( ...
        finalTime_s, minimumFinalTime_s, maximumFinalTime_s, ...
        startTime_s, goalState, obstacles, corridor, constraintLayout)
% Stay within decision bounds and one side of every known geometry event.
% Solving t_event = start + tau*(final-start) gives the final times at which
% a normalized association time tau lands exactly on obstacle history data.
if isfield(constraintLayout, "FinalTimeEvent_s")
    eventTime_s = constraintLayout.FinalTimeEvent_s;
else
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
        eventTime_s >= minimumFinalTime_s & ...
        eventTime_s <= maximumFinalTime_s));
end
scale_s = max(1, abs(finalTime_s));
eventTolerance_s = 64 * eps(scale_s);
previousEvent_s = max([minimumFinalTime_s; ...
    eventTime_s(eventTime_s < finalTime_s - eventTolerance_s)]);
nextEvent_s = min([maximumFinalTime_s; ...
    eventTime_s(eventTime_s > finalTime_s + eventTolerance_s)]);
backwardRoom_s = max(0, finalTime_s - previousEvent_s);
forwardRoom_s = max(0, nextEvent_s - finalTime_s);
baseStep_s = eps^(1 / 3) * scale_s;
% eps^(1/3) balances truncation and roundoff for a first derivative when the
% evaluated constraints contain several layers of floating-point geometry.
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
    error("evaluateTrajectoryConstraints:NoTimeDifferenceRoom", ...
        "No nonzero final-time perturbation fits inside the current event interval.");
end
end

function inequality = continuousBoundConstraints(polynomial, limits, options)
% Use Bernstein convex-hull bounds over every complete HS3 segment.
% For each quantity, coefficients above the upper limit and below the lower
% limit become positive violations. Feasibility therefore always means that
% every returned element is less than or equal to zero.
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
bernstein = hs3Engine.polynomial.convertPowerToBernstein(coefficientMatrix);
bernstein = permute(reshape( ...
    bernstein, coefficientCount, segmentCount, 2), [1 3 2]);
violations = [bernstein - reshape(upperBound, 1, 2, 1); ...
    reshape(lowerBound, 1, 2, 1) - bernstein];
end

function [inequality, normalByAssociation] = corridorConstraints( ...
        corridor, obstacles, polynomial, startTime_s, finalTime_s, ...
        constraintLayout)
% Bound each frozen association across the whole sub-interval it owns and
% retain its current jerk-space normal.
% Each association identifies an obstacle boundary feature and the side on
% which the seed route lies. The resulting inequality keeps the optimized
% motion at least Clearance_deg beyond that same feature.
coefficientCount = size(polynomial.positionPower_deg, 3);
recordCount = numel(corridor);
normalByAssociation = zeros(recordCount, 2);
if recordCount == 0
    inequality = zeros(0, 1);
    return;
end
if isfield(constraintLayout, "CorridorRowOffset")
    if constraintLayout.PositionCoefficientCount ~= coefficientCount || ...
            numel(constraintLayout.CorridorRowOffset) ~= recordCount + 1
        error("evaluateTrajectoryConstraints:InvalidConstraintLayout", ...
            "The prepared corridor layout does not match the polynomial.");
    end
    segmentIndex = constraintLayout.CorridorSegmentIndex;
    hullMap = constraintLayout.CorridorHullMap;
    rowOffset = constraintLayout.CorridorRowOffset;
else
    useIntervalHull = [corridor.UseIntervalHull].';
    effectiveTauEnd = [corridor.TauEnd];
    effectiveTauEnd(~useIntervalHull.') = [corridor(~useIntervalHull).Tau];
    [segmentIndex, hullMap] = ...
        hs3Engine.polynomial.createSubintervalBernsteinMap( ...
        [corridor.Tau], effectiveTauEnd, polynomial.SegmentCount, ...
        coefficientCount);
    rowOffset = [0; cumsum(1 + ...
        (coefficientCount - 1) * useIntervalHull)];
end
inequality = zeros(rowOffset(end), 1);

% Stationary geometry can constrain a complete Bernstein hull. Changing
% geometry remains tied to its ordered association time and is certified by
% the independent adaptive collision validator after each solve.
duration_s = finalTime_s - startTime_s;
for associationIndex = 1:recordCount
    association = corridor(associationIndex);
    hullRows = rowOffset(associationIndex) + 1: ...
        rowOffset(associationIndex + 1);
    associationRowCount = numel(hullRows);
    if association.GeometryIsFixed
        % Reuse the stored support line so the nonlinear solve sees a fixed,
        % differentiable half-plane for stationary geometry.
        outwardNormal = association.FixedNormal;
        boundaryOffset_deg = association.FixedBoundaryOffset_deg;
    else
        queryTime_s = startTime_s + association.Tau * duration_s;
        if association.UseSupport
            % A support normal describes an extreme side of the full shape.
            [~, geometry] = obstacleAvoidance.obstacles.shapeAtTime( ...
                obstacles(association.ObstacleIndex), queryTime_s, true);
            if ~geometry.Active
                % An inactive obstacle imposes no restriction. A negative
                % value preserves row count while marking strict feasibility.
                inequality(hullRows) = -1;
                continue;
            end
            finiteVertices = isfinite(geometry.azimuth_deg) & ...
                isfinite(geometry.elevation_deg);
            vertices_deg = [geometry.azimuth_deg(finiteVertices), ...
                geometry.elevation_deg(finiteVertices)];
            outwardNormal = association.SupportNormal;
            boundaryOffset_deg = max(vertices_deg * outwardNormal.');
        else
            % Edge associations need only their selected canonical row. Ring
            % bounds preserve ordering without rebuilding the full boundary.
            [active, hasEdge, edgeStart_deg, edgeEnd_deg] = ...
                obstacleAvoidance.obstacles.queryBoundaryEdgeAtTime( ...
                obstacles(association.ObstacleIndex), queryTime_s, ...
                association.EdgeIndex);
            if ~active
                inequality(hullRows) = -1;
                continue;
            end
            if ~hasEdge
                inequality(hullRows) = 1e3;
                continue;
            end
            edgeDelta_deg = edgeEnd_deg - edgeStart_deg;
            if norm(edgeDelta_deg) <= eps
                inequality(hullRows) = 1e3;
                continue;
            end
            leftNormal = [-edgeDelta_deg(2), edgeDelta_deg(1)] / ...
                norm(edgeDelta_deg);
            outwardNormal = association.OutwardSign * leftNormal;
            boundaryOffset_deg = edgeStart_deg * outwardNormal.';
        end
    end
    normalByAssociation(associationIndex, :) = outwardNormal;
    segmentPower_deg = reshape( ...
        polynomial.positionPower_deg(segmentIndex(associationIndex), :, :), ...
        2, coefficientCount);
    projectionHull_deg = hullMap(:, :, associationIndex) * ...
        (outwardNormal * segmentPower_deg).';
    % Positive means the motion entered the clearance side of the line;
    % nonpositive means its full projected hull remains safely outside.
    inequality(hullRows) = boundaryOffset_deg + ...
        association.Clearance_deg - ...
        projectionHull_deg(1:associationRowCount);
end
end
