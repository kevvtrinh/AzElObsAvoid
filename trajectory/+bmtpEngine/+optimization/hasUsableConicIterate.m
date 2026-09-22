function canCheckCandidate = hasUsableConicIterate(solverValues, exitFlag)
%% Section 0: Header & Readme
% SYNTAX
%   canCheckCandidate = bmtpEngine.optimization.hasUsableConicIterate(solverValues, exitFlag)
%**************************************************************************
% PURPOSE
%   - Decide whether the returned solver values can be checked as a candidate
%     motion or separating line. This check does not establish feasibility.
%**************************************************************************
% INPUTS
%   - solverValues (numeric array)
%       Latest variable values returned by coneprog.
%   - exitFlag (numeric scalar)
%       Status code returned by coneprog.
%**************************************************************************
% OUTPUTS
%   - canCheckCandidate (logical scalar)
%       True when the caller may run the required checks on these values.
%**************************************************************************
% UNITS
%   - Inherited from solverValues.
%**************************************************************************

%% Section 1: Decide Whether The Returned Values Can Be Checked

% A positive flag reports solver success. Flag -7 means the solver stopped
% taking useful steps before meeting its constraint or optimality tolerance.
% Either result must still pass the caller's checks before it can be used.
% Reject empty values, NaN/Inf, and all other failure codes.

valuesAreFinite   = ~isempty(solverValues) && all(isfinite(solverValues), 'all');
canCheckCandidate = valuesAreFinite && (exitFlag > 0 || exitFlag == -7);
end
