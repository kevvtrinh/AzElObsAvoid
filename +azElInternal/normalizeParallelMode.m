function mode = normalizeParallelMode(mode, errorIdentifier)
%% Section 0: Header & Readme
% SYNTAX
%   mode = azElInternal.normalizeParallelMode(mode, errorIdentifier)
%**************************************************************************
% PURPOSE
%   - Normalize the shared serial/parallel control to auto, on, or off.
%   - Accept logical and binary numeric aliases without duplicating parser
%     behavior between the public planner and visibility search.
%**************************************************************************
% INPUTS
%   - mode (scalar text, logical scalar, or binary numeric scalar)
%       Requested automatic, enabled, or disabled parallel execution.
%   - errorIdentifier (scalar text)
%       Owning function's identified error for invalid requests.
%**************************************************************************
% OUTPUTS
%   - mode (scalar string)
%       One of "auto", "on", or "off".
%**************************************************************************
% UNITS
%   - Execution modes are dimensionless.
%**************************************************************************

if islogical(mode) || isnumeric(mode)
    mode = azElInternal.normalizeLogicalScalar( ...
        mode, "UseParallel", errorIdentifier);
    if mode
        mode = "on";
    else
        mode = "off";
    end
else
    mode = lower(string(mode));
end
if ~isscalar(mode) || ~any(mode == ["auto" "on" "off"])
    error(errorIdentifier, ...
        "UseParallel must be auto, on, off, or a logical scalar.");
end
end
