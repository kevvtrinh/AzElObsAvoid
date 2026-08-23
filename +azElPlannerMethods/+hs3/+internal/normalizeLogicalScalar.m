function value = normalizeLogicalScalar(value, fieldName, errorIdentifier)
%% Section 0: Header & Readme
% SYNTAX
%   value = azElPlannerMethods.hs3.internal.normalizeLogicalScalar( ...
%       value, fieldName, errorIdentifier)
%**************************************************************************
% PURPOSE
%   - Normalize the repository-wide logical-or-binary-numeric contract.
%   - Keep the owning public function's error identifier and field name in
%     the resulting diagnostic.
%**************************************************************************
% INPUTS
%   - value (logical scalar or numeric 0/1 scalar)
%       Candidate control value.
%   - fieldName (scalar text)
%       User-facing option name included in an invalid-value message.
%   - errorIdentifier (scalar text)
%       Owning function's identified-error name.
%**************************************************************************
% OUTPUTS
%   - value (logical scalar)
%       Normalized control value.
%**************************************************************************
% UNITS
%   - Logical controls are dimensionless.
%**************************************************************************
%% Section 1: Validate And Normalize The Logical Value
% Accept MATLAB logicals and numeric 0/1 values without accepting other numerics.
isLogicalScalar = islogical(value) && isscalar(value);
isBinaryNumericScalar = isnumeric(value) && isscalar(value) && ...
    isreal(value) && isfinite(value) && any(value == [0 1]);

if ~(isLogicalScalar || isBinaryNumericScalar)
    error(errorIdentifier, ...
        "%s must be scalar logical or binary numeric.", fieldName);
end

% Convert the accepted representation to the single type used internally.
value = logical(value);
end
