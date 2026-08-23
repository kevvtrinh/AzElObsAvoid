function value = normalizeLogicalScalar(value, fieldName, errorIdentifier)
%% Section 0: Header & Readme
% SYNTAX
%   value = azElPlannerMethods.corridor.internal.normalizeLogicalScalar( ...
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

%% Section 1: Accept Only Unambiguous Scalar Logical Values

% MATLAB commonly receives 0/1 configuration values from files and tables.
% Supporting those two numerics is convenient; accepting other nonzero values
% would hide configuration mistakes during logical conversion.
isLogicalScalar = islogical(value) && isscalar(value);
isBinaryNumericScalar = isnumeric(value) && isscalar(value) && isreal(value) && isfinite(value) && any(value == [0 1]);
if ~(isLogicalScalar || isBinaryNumericScalar)
    error(errorIdentifier, "%s must be scalar logical or binary numeric.", fieldName);
end
value = logical(value);
end
