function result = finalizeCandidate(result, candidate, route_units, diagnostics)
%% Section 0: Header & Readme
% SYNTAX
%   result = ...
%       obstacleAvoidance.input.finalizeCandidate(result, candidate, route_units, diagnostics)
%**************************************************************************
% PURPOSE
%   - Assemble one BMTP candidate and apply the independent validator.
%**************************************************************************
% INPUTS
%   - result (scalar struct)
%       Planner record awaiting candidate assembly.
%   - candidate (scalar struct)
%       BMTP motion candidate.
%   - route_units (N-by-2 numeric array)
%       Selected route positions.
%   - diagnostics (scalar struct)
%       Solver diagnostic record.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Planner result after the uniform candidate-acceptance decision.
%       Motion that fails validation returns Success = false with a
%       diagnostic Message and TerminationReason; invalid input throws.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Assemble The Complete Candidate

for fieldName = reshape(string(fieldnames(candidate)), 1, [])
    result.(fieldName) = candidate.(fieldName);
end
result.Route_units       = route_units;
result.SolverDiagnostics = diagnostics;

goalState = result.Inputs.goalState;
if candidate.Success && ~isempty(goalState.targetMotion)
    targetMotion  = goalState.targetMotion;
    arrivalTime_s = candidate.ArrivalTime_s;
    [tgtPosition_units, tgtVel_units_s, tgtAcc_units_s2] = ...
        obstacleAvoidance.input.targetPositionAtTime(targetMotion, arrivalTime_s);
    result.Intercept = struct( ...
        'Time_s',                     arrivalTime_s, ...
        'TargetPosition_units',       tgtPosition_units, ...
        'TerminalVelocityPolicy',     "explicit", ...
        'TerminalAccelerationPolicy', "explicit");
    if all(candidate.velocity_units_s(end, :) == 0)
        result.Intercept.TerminalVelocityPolicy = "zero";
    end
    if all(candidate.acceleration_units_s2(end, :) == 0)
        result.Intercept.TerminalAccelerationPolicy = "zero";
    end
    if result.Options.MatchTargetVelocity
        result.Intercept.TerminalVelocityPolicy = "matched";
        result.Intercept.TargetVelocity_units_s = tgtVel_units_s;
    end
    if result.Options.MatchTargetAcceleration
        result.Intercept.TerminalAccelerationPolicy  = "matched";
        result.Intercept.TargetAcceleration_units_s2 = tgtAcc_units_s2;
    end
end

%% Section 2: Carry The Outer Request Into An Accepted Record

% A periodic request is planned as plain requests in the unwrapped frame,
% and the chronological search plans trials on their own fixed clocks. A
% successful candidate's record declares the outer request before the one
% validation, so nothing is validated twice. A failed candidate keeps its
% own request so later stages can still read the outer one.
if isfield(result, 'OuterRequest') && candidate.Success
    result = obstacleAvoidance.input.applyOuterRequest(result, result.OuterRequest);
end

%% Section 3: Apply The One Public Acceptance Gate

result.Validation = obstacleAvoidance.validateTrajectory(result);
result.Success    = candidate.Success && result.Validation.Passed;

if candidate.Success && ~result.Validation.Passed
    result.Message = "BMTP returned motion that failed independent validation: " + ...
        result.Validation.Message;
    result.TerminationReason = "invalidMotion";
end
end
