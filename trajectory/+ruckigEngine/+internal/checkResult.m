function result = checkResult( ...
        result, profile, terminalState, limits, pathConstraints)
%% Section 0: Header & Readme
% SYNTAX
%   result = ruckigEngine.internal.checkResult( ...
%       result, profile, terminalState, limits, pathConstraints)
%**************************************************************************
% PURPOSE
%   - Check the assembled synchronized motion continuously against its
%     dimension-neutral request and classify the engine result.
%**************************************************************************
% INPUTS
%   - result (scalar Ruckig-engine result)
%       Evaluated sampled and polynomial motion fields.
%   - profile (scalar exact switching-profile struct)
%       Final synchronized polynomial.
%   - terminalState, limits (normalized scalar structs)
%       Requested terminal state and physical derivative bounds.
%   - pathConstraints (scalar normalized struct)
%       Optional affine rows to check continuously.
%**************************************************************************
% OUTPUTS
%   - result (scalar Ruckig-engine result)
%       Constraint residual, independent validation, success, and reason.
%**************************************************************************
% UNITS
%   - Units are caller-defined and consistent across motion derivatives.
%**************************************************************************

%% Section 1: Evaluate Continuous Request Constraints

[inequality, equality] = ...
    ruckigEngine.internal.evaluatePolynomialConstraints( ...
    profile.Polynomial, terminalState, limits, pathConstraints);
result.MaximumConstraintViolation = max([ ...
    0; inequality(:); abs(equality(:))]);

%% Section 2: Validate And Classify The Engine Result

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
