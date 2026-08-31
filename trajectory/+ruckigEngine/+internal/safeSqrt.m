function value = safeSqrt(radicand)
%% Section 0: Header & Readme
% SYNTAX
%   value = ruckigEngine.internal.safeSqrt(radicand)
%**************************************************************************
% PURPOSE
%   - Evaluate a square root while accepting only roundoff-scale negatives.
%**************************************************************************
% INPUTS
%   - radicand (finite numeric scalar)
%       Value from a switching-profile equation.
%**************************************************************************
% OUTPUTS
%   - value (numeric scalar)
%       The nonnegative root, or NaN for a genuinely negative radicand.
%**************************************************************************
% UNITS
%   - Units are the square root of the caller's radicand units.
%**************************************************************************

%% Section 1: Distinguish Roundoff From A Negative Root

if radicand < -64 * eps(max(1, abs(radicand)))
    value = NaN;
else
    value = sqrt(max(0, radicand));
end
end
