function value = normalizeLogicalScalar(value, fieldName, errorIdentifier)
%% Section 0: Header & Readme
% SYNTAX
%   value = obstacleAvoidance.input.normalizeLogicalScalar( ...
%       value, fieldName, errorIdentifier)
%**************************************************************************
% PURPOSE
%   - Normalize the repository-wide logical-or-binary-numeric input rule.
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

% Accept true, false, 1, or 0 as one value. Reject arrays and other numbers.
% This prevents an option such as [true false] from reaching an if statement
% with unclear meaning.

% MATLAB commonly receives 0/1 configuration values from files and tables.
% The function also accepts these two numeric values. Other numbers are not
% safe. MATLAB converts 2, -1, and NaN to true. The scalar check also rejects a
% value such as [true false].
isLogicalScalar = islogical(value) && isscalar(value);
isBinaryNumericScalar = isnumeric(value) && isscalar(value) && isreal(value) && isfinite(value) && any(value == [0 1]);
if ~(isLogicalScalar || isBinaryNumericScalar)
    error(errorIdentifier, "%s must be scalar logical or binary numeric.", fieldName);
end
value = logical(value);
% Callers now receive one MATLAB logical value. The input can be true, false,
% zero, or one.
end
