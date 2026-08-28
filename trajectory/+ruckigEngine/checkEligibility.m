function eligibility = checkEligibility( ...
        initialState, terminalState, limits, options, pathConstraints)
%% Section 0: Header & Readme
% SYNTAX
%   eligibility = ruckigEngine.checkEligibility( ...
%       initialState, terminalState, limits, options, pathConstraints)
%**************************************************************************
% PURPOSE
%   - Report whether a normalized request belongs to the exact switching
%     engine's supported request family.
%**************************************************************************
% INPUTS
%   - initialState, terminalState, limits (normalized scalar structs)
%       Dimension-neutral boundary states and resolved coordinate bounds.
%   - options (resolved scalar struct)
%       Direct time and numerical options for this engine.
%   - pathConstraints (normalized scalar struct)
%       Nonempty affine rows are unsupported by this engine.
%**************************************************************************
% OUTPUTS
%   - eligibility (scalar struct)
%       Supported flag, identified reason, and actionable message.
%**************************************************************************
% UNITS
%   - Eligibility fields are dimensionless; inputs retain caller units.
%**************************************************************************

%% Section 1: Check Unsupported Request Features

eligibility = struct( ...
    "Supported", true, ...
    "TerminationReason", "eligible", ...
    "Message", "The request is eligible for exact switching profiles.");
if ~isempty(pathConstraints.Tau)
    eligibility.Supported = false;
    eligibility.TerminationReason = "unsupportedPathConstraints";
    eligibility.Message = ...
        "The Ruckig-derived engine does not support affine path constraints.";
    return;
end
if ~hasSymmetricDerivativeBounds(limits)
    eligibility.Supported = false;
    eligibility.TerminationReason = "unsupportedAsymmetricBounds";
    eligibility.Message = ...
        "The Ruckig-derived engine requires symmetric derivative bounds.";
    return;
end
if options.TimeMode == "fixed"
    finalTime = options.FinalTime;
    if isempty(finalTime)
        finalTime = terminalState.maximumTime;
    end
    if ~isscalar(finalTime) || ~isfinite(finalTime) || ...
            finalTime <= initialState.time
        error("ruckigEngine:InvalidFinalTime", ...
            "A fixed FinalTime must be finite and later than initialState.time.");
    end
end
end

%% Section 2: Local Functions

function value = hasSymmetricDerivativeBounds(limits)
% Compare every resolved derivative bound with its positive maximum.
scale = max([1, limits.maximumVelocity, ...
    limits.maximumAcceleration, limits.maximumJerk], [], 2);
tolerance = 128 * eps(scale);
value = all(abs(limits.velocityLower + ...
    limits.maximumVelocity) <= tolerance) && ...
    all(abs(limits.velocityUpper - ...
    limits.maximumVelocity) <= tolerance) && ...
    all(abs(limits.accelerationLower + ...
    limits.maximumAcceleration) <= tolerance) && ...
    all(abs(limits.accelerationUpper - ...
    limits.maximumAcceleration) <= tolerance) && ...
    all(abs(limits.jerkLower + limits.maximumJerk) <= tolerance) && ...
    all(abs(limits.jerkUpper - limits.maximumJerk) <= tolerance);
end
