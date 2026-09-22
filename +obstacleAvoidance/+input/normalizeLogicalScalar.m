function controlValue = normalizeLogicalScalar(controlValue, fieldName, errorIdentifier)
%% Section 0: Header & Readme
% SYNTAX
%   controlValue = obstacleAvoidance.input.normalizeLogicalScalar( ...
%       controlValue, fieldName, errorIdentifier)
%**************************************************************************
% PURPOSE
%   - Accept one true, false, 0, or 1 value and return logical true or false.
%**************************************************************************
% INPUTS
%   - controlValue (logical scalar or numeric 0/1 scalar)
%       Input for an on/off setting.
%   - fieldName (scalar text)
%       Input field name to include in an error message.
%   - errorIdentifier (scalar text)
%       Error identifier to use if the value is invalid.
%**************************************************************************
% OUTPUTS
%   - controlValue (logical scalar)
%       Logical true or false. Any other input throws errorIdentifier.
%**************************************************************************
% UNITS
%   - Logical controls are dimensionless.
%**************************************************************************

%% Section 1: Check And Convert The Control Value

valueIsLogicalScalar       = islogical(controlValue) && isscalar(controlValue);
valueIsRealNumericScalar   = isnumeric(controlValue) && isscalar(controlValue) && ...
    isreal(controlValue) && isfinite(controlValue);
valueIsBinaryNumericScalar = valueIsRealNumericScalar && any(controlValue == [0, 1]);
if ~(valueIsLogicalScalar || valueIsBinaryNumericScalar)
    error(errorIdentifier, "%s must be scalar logical or binary numeric.", fieldName);
end
controlValue = logical(controlValue);
end
