function [candidate, diagnostics] = createRuckigWaypointMotion(seed, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   [candidate, diagnostics] = ...
%       obstacleAvoidance.planner.createRuckigWaypointMotion( ...
%       seed, initialState, goalState, limits, options)
%
% PURPOSE
%   - Compose at most two exact state-to-state Ruckig route segments.
%
% INPUTS
%   - seed (scalar route-seed struct)
%       position_units is an N-by-2 obstacle-derived route. After consecutive
%       duplicate removal, N must be two or three; longer routes return an
%       expected unsupported result rather than running Ruckig.
%   - initialState, goalState (normalized scalar state structs)
%       Endpoint position and derivatives define the first and final states.
%   - limits (normalized scalar planner-limit struct)
%       Workspace, velocity, acceleration, and jerk limits use named units.
%   - options (normalized scalar planner-option struct)
%       GoalTimeMode controls earliest or exact fixed arrival behavior.
%
% OUTPUTS
%   - candidate (scalar motion-candidate struct)
%       Exact composite motion, or a stable identified segment failure.
%   - diagnostics (scalar struct)
%       Ruckig provenance, segment counts, failures, and elapsed time.
%
% UNITS
%   - Position is coordinate units and time is seconds. Derivatives use units/s,
%     units/s^2, and units/s^3. Route and histories are N-by-2.
%

%% Section 1: Normalize The Route And Engine Request

solveTimer = tic;
candidate  = bmtpEngine.createMotionRecord(struct(), initialState, [], [], options.SampleTime_s, seed.Source);
candidate.SeedIndex = seed.Index;
route_units = double(seed.position_units);
validateattributes(route_units, {'numeric'}, {'real', 'finite', '2d', 'ncols', 2});
if size(route_units, 1) < 2
    error("createRuckigWaypointMotion:InsufficientWaypoints", "seed.position_units must contain at least two route vertices.");
end
coordinateScale_units    = bmtpEngine.createCoordinateTolerances(route_units);
duplicateTolerance_units = 256 * eps(coordinateScale_units);
keepVertex             = [true; vecnorm(diff(route_units), 2, 2) > ...
    duplicateTolerance_units];
route_units = route_units(keepVertex, :);
if size(route_units, 1) < 2
    error("createRuckigWaypointMotion:ZeroLengthRoute", "The route must contain two distinct consecutive vertices.");
end

engineLimits = struct("maximumVelocity", limits.maxVelocity_units_s, ...
    "maximumAcceleration", limits.maxAcceleration_units_s2, ...
    "maximumJerk", limits.maxJerk_units_s3, ...
    "positionLower", [limits.xInterval_units(1), ...
    limits.yInterval_units(1)], "positionUpper", [limits.xInterval_units(2), limits.yInterval_units(2)]);
% Continuous coordinates may cross a workspace seam only on enabled axes.
wrapAxes = [options.WrapX, options.WrapY];
engineLimits.positionLower(wrapAxes) = -Inf;
engineLimits.positionUpper(wrapAxes) = Inf;
engineOptions = struct("TimeMode", "earliestArrival", "FinalTime", [], ...
    "SampleTime", options.SampleTime_s, ...
    "ConstraintTolerance", options.ConstraintTolerance, ...
    "ArrivalTimeTolerance", options.ArrivalTimeTolerance_s, ...
    "Verbose", false);
partCount                 = size(route_units, 1) - 1;
maximumSupportedPartCount = 2;
diagnostics               = struct("Identifier", "ruckigWaypointComposition", ...
    "ConstraintRepresentation", "exactStateToStateSegments", ...
    "Accepted", false, "RequestedPartCount", partCount, ...
    "MaximumSupportedPartCount", maximumSupportedPartCount, ...
    "CompletedPartCount", 0, "FailedPartIndex", 0, ...
    "EngineTerminationReason", "notRun", ...
    "WaypointPosition_units", route_units, ...
    "InteriorWaypointTime_s", NaN(max(0, partCount - 1), 1), ...
    "InteriorWaypointPosition_units", route_units(2:end - 1, :), ...
    "InteriorWaypointVelocity_units_s", ...
    zeros(max(0, partCount - 1), 2), "InteriorWaypointAcceleration_units_s2", zeros(max(0, partCount - 1), 2), "AllInteriorWaypointsConstrainedToRest", true, "ElapsedTime_s", 0);
% Return the documented unsupported result instead of approximating a route with more motion parts than the engine can preserve.
if partCount > maximumSupportedPartCount
    diagnostics.EngineTerminationReason = "ruckigWaypointSegmentLimitExceeded";
    diagnostics.ElapsedTime_s           = toc(solveTimer);
    candidate.Message           = "Ruckig waypoint composition supports at most " + maximumSupportedPartCount + " route segments; the normalized " + "route contains " + partCount + ".";
    candidate.TerminationReason = diagnostics.EngineTerminationReason;
    candidate.SolverDiagnostics = diagnostics;
    return;
end

%% Section 2: Solve Every Route Edge

polynomialParts            = cell(partCount, 1);
maximumConstraintViolation = 0;
currentTime_s              = initialState.time_s;
% Process each part while assembling the complete motion or interval result.
for partIndex = 1:partCount
    engineInitialState = struct("time", currentTime_s, ...
        "position", route_units(partIndex, :), ...
        "velocity", [0 0], "acceleration", [0 0]);
    if partIndex == 1
        engineInitialState.velocity     = initialState.velocity_units_s;
        engineInitialState.acceleration = initialState.acceleration_units_s2;
    end
    terminalVelocity_units_s      = [0 0];
    terminalAcceleration_units_s2 = [0 0];
    if partIndex == partCount
        terminalVelocity_units_s      = goalState.velocity_units_s;
        terminalAcceleration_units_s2 = goalState.acceleration_units_s2;
        % Use fixed-arrival construction and ranking when the arrival time is prescribed; otherwise optimize earliest arrival.
        if options.GoalTimeMode == "fixedArrival"
            engineOptions.TimeMode  = "fixed";
            engineOptions.FinalTime = goalState.time_s;
        end
    end
    engineTerminalState = struct("position", route_units(partIndex + 1, :), ...
        "velocity", terminalVelocity_units_s, ...
        "acceleration", terminalAcceleration_units_s2, ...
        "maximumTime", goalState.time_s);
    part                       = ruckigEngine.solve(engineInitialState, engineTerminalState, engineLimits, engineOptions);
    maximumConstraintViolation = max(maximumConstraintViolation, part.MaximumConstraintViolation);
    % Reject the complete waypoint motion if any constituent part fails, preserving that part's reason for diagnostics.
    if ~part.Success
        diagnostics.FailedPartIndex         = partIndex;
        diagnostics.EngineTerminationReason = part.TerminationReason;
        diagnostics.ElapsedTime_s           = toc(solveTimer);
        candidate.Message                    = "Ruckig route segment " + partIndex + " failed. " + part.Message;
        candidate.TerminationReason          = part.TerminationReason;
        candidate.MaximumConstraintViolation = maximumConstraintViolation;
        candidate.SolverDiagnostics          = diagnostics;
        return;
    end
    polynomialParts{partIndex} = part.Polynomial;
    diagnostics.CompletedPartCount = partIndex;
    currentTime_s = part.FinalTime;
    if partIndex < partCount
        diagnostics.InteriorWaypointTime_s(partIndex) = currentTime_s;
    end
end

%% Section 3: Assemble The Exact Composite Motion

enginePolynomial = combinePolynomials(polynomialParts);
polynomial       = convertPolynomial(enginePolynomial, initialState.time_s);
candidate        = bmtpEngine.createMotionRecord(candidate, initialState, polynomial, [], options.SampleTime_s, seed.Source);
[candidate.Success, candidate.OptimizerFeasible] = deal(true);
candidate.Message                    = "Exact Ruckig state-to-state route motion was constructed.";
candidate.TerminationReason          = "goalReached";
candidate.MaximumConstraintViolation = maximumConstraintViolation;
diagnostics.Accepted                = true;
diagnostics.EngineTerminationReason = "goalReached";
diagnostics.ElapsedTime_s           = toc(solveTimer);
candidate.SolverDiagnostics = diagnostics;
end

%% Section 4: Local Functions

function combined = combinePolynomials(parts)
    % Concatenate exact switching segments without altering local coefficients.
    combined = parts{1};
    % Process each part while assembling the complete motion or interval result.
    for partIndex = 2:numel(parts)
        part = parts{partIndex};
        combined.SegmentStartTime = [ ...
            combined.SegmentStartTime; part.SegmentStartTime];
        combined.SegmentDuration = [ ...
            combined.SegmentDuration; part.SegmentDuration];
        combined.positionPower     = cat(1, combined.positionPower, part.positionPower);
        combined.velocityPower     = cat(1, combined.velocityPower, part.velocityPower);
        combined.accelerationPower = cat(1, combined.accelerationPower, part.accelerationPower);
        combined.jerkPower         = cat(1, combined.jerkPower, part.jerkPower);
        combined.SegmentCount      = combined.SegmentCount + part.SegmentCount;
        combined.FinalTime         = part.FinalTime;
        combined.TerminalState     = part.TerminalState;
    end
end

function polynomial = convertPolynomial(enginePolynomial, initialTime_s)
    % Convert engine field names to the planner's polynomial format.
    terminalState   = enginePolynomial.TerminalState;
    duration_s      = enginePolynomial.FinalTime - initialTime_s;
    segmentBreakTau = [enginePolynomial.SegmentStartTime; ...
        enginePolynomial.FinalTime] - initialTime_s;
    segmentBreakTau = segmentBreakTau / duration_s;
    polynomial      = struct("Degree", size(enginePolynomial.positionPower, 3) - 1, ...
        "SegmentCount", enginePolynomial.SegmentCount, ...
        "SegmentStartTime_s", enginePolynomial.SegmentStartTime, ...
        "SegmentDuration_s", enginePolynomial.SegmentDuration, ...
        "SegmentBreakTau", segmentBreakTau, ...
        "FinalTime_s", enginePolynomial.FinalTime, ...
        "positionPower_units", enginePolynomial.positionPower, ...
        "velocityPower_units_s", enginePolynomial.velocityPower, ...
        "accelerationPower_units_s2", enginePolynomial.accelerationPower, ...
        "jerkPower_units_s3", enginePolynomial.jerkPower, ...
        "TerminalState", struct("position_units", terminalState.position, ...
        "velocity_units_s", terminalState.velocity, ...
        "acceleration_units_s2", terminalState.acceleration));
end
