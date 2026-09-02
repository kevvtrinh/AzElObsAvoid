function result = evaluateSynchronizedMotion( ...
        result, profile, initialState, terminalState, limits, options, ...
        pathConstraints)
%% Section 0: Header & Readme
% SYNTAX
%   result = ruckigEngine.evaluateSynchronizedMotion( ...
%       result, profile, initialState, terminalState, limits, options, ...
%       pathConstraints)
%**************************************************************************
% PURPOSE
%   - Evaluate the synchronized polynomial and check it against the request.
%   - Assemble the stable engine result without changing the solved profile.
%**************************************************************************
% INPUTS
%   - result (scalar empty Ruckig-engine result)
%       Stable request and diagnostic record to complete.
%   - profile (scalar exact switching-profile struct)
%       Successful synchronized polynomial and jerk controls.
%   - initialState, terminalState, limits, options (scalar structs)
%       Normalized request, limits, and resolved sampling controls.
%   - pathConstraints (scalar normalized struct)
%       Optional affine constraints checked continuously after construction.
%**************************************************************************
% OUTPUTS
%   - result (scalar Ruckig-engine result)
%       Sampled motion, continuous constraint residuals, validation, and status.
%**************************************************************************
% UNITS
%   - Units are caller-defined and consistent across motion derivatives.
%**************************************************************************

%% Section 1: Evaluate The Synchronized Motion

% Synchronization changes axis timing, so reconstruct the complete polynomial
% at uniform and switching times rather than combining per-axis samples.

result.FinalTime = profile.FinalTime;
result.Duration = profile.FinalTime - initialState.time;
result.ControlJerk = profile.ControlJerk;
result.Polynomial = profile.Polynomial;
result.IntegratedSquaredJerk = profile.IntegratedSquaredJerk;
uniformTime = (initialState.time:options.SampleTime:profile.FinalTime).';
sampleTime = unique([uniformTime; ...
    profile.Polynomial.SegmentStartTime; profile.FinalTime]);
[sampleTime, position, velocity, acceleration, jerk] = ...
    ruckigEngine.internal.evaluatePolynomial( ...
    profile.Polynomial, sampleTime);
result.time = sampleTime;
result.position = position;
result.velocity = velocity;
result.acceleration = acceleration;
result.jerk = jerk;

%% Section 2: Check The Returned Motion

% A synchronized profile is not accepted merely because each constructor
% succeeded. Re-evaluate continuous request and affine path constraints, then
% run the engine's independent result check over the assembled motion.

[inequality, equality] = ...
    ruckigEngine.internal.evaluatePolynomialConstraints( ...
    profile.Polynomial, terminalState, limits, pathConstraints);
result.MaximumConstraintViolation = max([ ...
    0; inequality(:); abs(equality(:))]);
result.Validation = ruckigEngine.internal.validateResult(result);
result.Success = result.Validation.Passed;
if result.Success
    result.Message = ...
        "A kinematically constrained trajectory was found and independently validated.";
    result.TerminationReason = "goalReached";
elseif ~isempty(pathConstraints.Tau)
    result.Message = "The exact switching profile violates an affine " + ...
        "path constraint. " + result.Validation.Message;
    result.TerminationReason = "pathConstraintViolation";
else
    result.Message = result.Validation.Message;
    result.TerminationReason = "exactProfileValidationFailed";
end
end
