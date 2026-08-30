function [candidate, diagnostics] = createRuckigWaypointMotion( ...
        seed, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   [candidate, diagnostics] = ...
%       obstacleAvoidance.planner.createRuckigWaypointMotion( ...
%       seed, initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Compose exact state-to-state Ruckig motions along an input route.
%**************************************************************************
% INPUTS
%   - seed (scalar route-seed struct)
%       position_deg is an N-by-2 obstacle-derived route with N at least two.
%   - initialState, goalState (normalized scalar state structs)
%       Endpoint position and derivatives define the first and final states.
%   - limits (normalized scalar planner-limit struct)
%       Workspace, velocity, acceleration, and jerk limits use named units.
%   - options (normalized scalar planner-option struct)
%       GoalTimeMode controls earliest or exact fixed arrival behavior.
%**************************************************************************
% OUTPUTS
%   - candidate (scalar motion-candidate struct)
%       Exact composite motion, or a stable identified segment failure.
%   - diagnostics (scalar struct)
%       Ruckig provenance, segment counts, failures, and elapsed time.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds. Derivatives use deg/s,
%     deg/s^2, and deg/s^3. Route and histories are N-by-2.
%**************************************************************************

%% Section 1: Normalize The Route And Engine Request

solveTimer = tic;
candidate = bmtpEngine.createMotionRecord( ...
    struct(), initialState, [], [], options.SampleTime_s, seed.Source);
candidate.SeedIndex = seed.Index;
route_deg = double(seed.position_deg);
validateattributes(route_deg, {'numeric'}, ...
    {'real', 'finite', '2d', 'ncols', 2});
if size(route_deg, 1) < 2
    error("createRuckigWaypointMotion:InsufficientWaypoints", ...
        "seed.position_deg must contain at least two route vertices.");
end
coordinateScale_deg = max([1; abs(route_deg(:))]);
duplicateTolerance_deg = 256 * eps(coordinateScale_deg);
keepVertex = [true; vecnorm(diff(route_deg), 2, 2) > ...
    duplicateTolerance_deg];
route_deg = route_deg(keepVertex, :);
if size(route_deg, 1) < 2
    error("createRuckigWaypointMotion:ZeroLengthRoute", ...
        "The route must contain two distinct consecutive vertices.");
end

engineLimits = struct( ...
    "maximumVelocity", limits.maxVelocity_deg_s, ...
    "maximumAcceleration", limits.maxAcceleration_deg_s2, ...
    "maximumJerk", limits.maxJerk_deg_s3, ...
    "positionLower", [limits.azimuthInterval_deg(1), ...
    limits.elevationInterval_deg(1)], ...
    "positionUpper", [limits.azimuthInterval_deg(2), ...
    limits.elevationInterval_deg(2)]);
if options.AllowAzimuthWrapping
    engineLimits.positionLower(1) = -Inf;
    engineLimits.positionUpper(1) = Inf;
end
engineOptions = struct( ...
    "TimeMode", "earliestArrival", "FinalTime", [], ...
    "SampleTime", options.SampleTime_s, ...
    "ConstraintTolerance", options.ConstraintTolerance, ...
    "ArrivalTimeTolerance", options.ArrivalTimeTolerance_s, ...
    "Verbose", false);
partCount = size(route_deg, 1) - 1;
diagnostics = struct( ...
    "Identifier", "ruckigWaypointComposition", ...
    "ConstraintRepresentation", "exactStateToStateSegments", ...
    "Accepted", false, "RequestedPartCount", partCount, ...
    "CompletedPartCount", 0, "FailedPartIndex", 0, ...
    "EngineTerminationReason", "notRun", "ElapsedTime_s", 0);

%% Section 2: Solve Every Route Edge

polynomialParts = cell(partCount, 1);
maximumConstraintViolation = 0;
currentTime_s = initialState.time_s;
for partIndex = 1:partCount
    engineInitialState = struct( ...
        "time", currentTime_s, ...
        "position", route_deg(partIndex, :), ...
        "velocity", [0 0], "acceleration", [0 0]);
    if partIndex == 1
        engineInitialState.velocity = initialState.velocity_deg_s;
        engineInitialState.acceleration = initialState.acceleration_deg_s2;
    end
    terminalVelocity_deg_s = [0 0];
    terminalAcceleration_deg_s2 = [0 0];
    if partIndex == partCount
        terminalVelocity_deg_s = goalState.velocity_deg_s;
        terminalAcceleration_deg_s2 = goalState.acceleration_deg_s2;
        if options.GoalTimeMode == "fixedArrival"
            engineOptions.TimeMode = "fixed";
            engineOptions.FinalTime = goalState.time_s;
        end
    end
    engineTerminalState = struct( ...
        "position", route_deg(partIndex + 1, :), ...
        "velocity", terminalVelocity_deg_s, ...
        "acceleration", terminalAcceleration_deg_s2, ...
        "maximumTime", goalState.time_s);
    part = ruckigEngine.solve( ...
        engineInitialState, engineTerminalState, engineLimits, engineOptions);
    maximumConstraintViolation = max( ...
        maximumConstraintViolation, part.MaximumConstraintViolation);
    if ~part.Success
        diagnostics.FailedPartIndex = partIndex;
        diagnostics.EngineTerminationReason = part.TerminationReason;
        diagnostics.ElapsedTime_s = toc(solveTimer);
        candidate.Message = "Ruckig route segment " + partIndex + ...
            " failed. " + part.Message;
        candidate.TerminationReason = part.TerminationReason;
        candidate.MaximumConstraintViolation = maximumConstraintViolation;
        candidate.SolverDiagnostics = diagnostics;
        return;
    end
    polynomialParts{partIndex} = part.Polynomial;
    diagnostics.CompletedPartCount = partIndex;
    currentTime_s = part.FinalTime;
end

%% Section 3: Assemble The Exact Composite Motion

enginePolynomial = combinePolynomials(polynomialParts);
polynomial = convertPolynomial(enginePolynomial, initialState.time_s);
candidate = bmtpEngine.createMotionRecord( ...
    candidate, initialState, polynomial, [], options.SampleTime_s, seed.Source);
candidate.OptimizerFeasible = true;
candidate.Message = "Exact Ruckig state-to-state route motion was constructed.";
candidate.TerminationReason = "goalReached";
candidate.MaximumConstraintViolation = maximumConstraintViolation;
diagnostics.Accepted = true;
diagnostics.EngineTerminationReason = "goalReached";
diagnostics.ElapsedTime_s = toc(solveTimer);
candidate.SolverDiagnostics = diagnostics;
end

%% Section 4: Local Functions

function combined = combinePolynomials(parts)
% Concatenate exact switching segments without altering local coefficients.
combined = parts{1};
for partIndex = 2:numel(parts)
    part = parts{partIndex};
    combined.SegmentStartTime = [ ...
        combined.SegmentStartTime; part.SegmentStartTime];
    combined.SegmentDuration = [ ...
        combined.SegmentDuration; part.SegmentDuration];
    combined.positionPower = cat( ...
        1, combined.positionPower, part.positionPower);
    combined.velocityPower = cat( ...
        1, combined.velocityPower, part.velocityPower);
    combined.accelerationPower = cat( ...
        1, combined.accelerationPower, part.accelerationPower);
    combined.jerkPower = cat(1, combined.jerkPower, part.jerkPower);
    combined.SegmentCount = combined.SegmentCount + part.SegmentCount;
    combined.FinalTime = part.FinalTime;
    combined.TerminalState = part.TerminalState;
end
end

function polynomial = convertPolynomial(enginePolynomial, initialTime_s)
% Translate engine names into the planner's shared polynomial contract.
terminalState = enginePolynomial.TerminalState;
duration_s = enginePolynomial.FinalTime - initialTime_s;
segmentBreakTau = [enginePolynomial.SegmentStartTime; ...
    enginePolynomial.FinalTime] - initialTime_s;
segmentBreakTau = segmentBreakTau / duration_s;
polynomial = struct( ...
    "Degree", size(enginePolynomial.positionPower, 3) - 1, ...
    "SegmentCount", enginePolynomial.SegmentCount, ...
    "SegmentStartTime_s", enginePolynomial.SegmentStartTime, ...
    "SegmentDuration_s", enginePolynomial.SegmentDuration, ...
    "SegmentBreakTau", segmentBreakTau, ...
    "FinalTime_s", enginePolynomial.FinalTime, ...
    "positionPower_deg", enginePolynomial.positionPower, ...
    "velocityPower_deg_s", enginePolynomial.velocityPower, ...
    "accelerationPower_deg_s2", enginePolynomial.accelerationPower, ...
    "jerkPower_deg_s3", enginePolynomial.jerkPower, ...
    "TerminalState", struct( ...
    "position_deg", terminalState.position, ...
    "velocity_deg_s", terminalState.velocity, ...
    "acceleration_deg_s2", terminalState.acceleration));
end
