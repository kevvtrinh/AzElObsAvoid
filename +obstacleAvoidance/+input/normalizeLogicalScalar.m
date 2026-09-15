function value = normalizeLogicalScalar(value, fieldName, errorIdentifier)
%% Section 0: Header & Readme
% SYNTAX
%   value = ...
%       obstacleAvoidance.input.normalizeLogicalScalar(value, fieldName, errorIdentifier)
%**************************************************************************
% PURPOSE
%   - Normalize the shared logical-or-binary-numeric input rule.
%**************************************************************************
% INPUTS
%   - value (logical scalar or numeric 0/1 scalar)
%       Candidate control value.
%   - fieldName (scalar text)
%       Diagnostic field name.
%   - errorIdentifier (scalar text)
%       Error identifier owned by the caller.
%**************************************************************************
% OUTPUTS
%   - value (logical scalar)
%       Normalized control value. Any other input throws errorIdentifier.
%**************************************************************************
% UNITS
%   - Logical controls are dimensionless.
%**************************************************************************

%% Section 1: Accept Only Unambiguous Scalar Logical Values

valueIsLogicalScalar       = islogical(value) && isscalar(value);
valueIsRealNumericScalar   = isnumeric(value) && isscalar(value) && isreal(value) && isfinite(value);
valueIsBinaryNumericScalar = valueIsRealNumericScalar && any(value == [0, 1]);
if ~(valueIsLogicalScalar || valueIsBinaryNumericScalar)
    error(errorIdentifier, "%s must be scalar logical or binary numeric.", fieldName);
end
value = logical(value);
end
