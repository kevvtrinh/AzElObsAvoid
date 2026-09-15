function [resolvedOptions, unknownNames] = resolveOptions(defaultOptions, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   [resolvedOptions, unknownNames] = ...
%       obstacleAvoidance.input.resolveOptions(defaultOptions, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Apply the shared partial-option merge rule without reordering defaults.
%**************************************************************************
% INPUTS
%   - defaultOptions (scalar struct)
%       Complete option defaults.
%   - optionOverrides (scalar struct)
%       Partial overrides; empty values retain their defaults.
%**************************************************************************
% OUTPUTS
%   - resolvedOptions (scalar struct)
%       Defaults with known nonempty overrides applied.
%   - unknownNames (N-by-1 string vector)
%       Ignored override fields in caller-supplied order, so the caller can
%       issue one warning. A nonstruct override throws.
%**************************************************************************
% UNITS
%   - Values retain the units documented by the owning function.
%**************************************************************************

%% Section 1: Classify Override Names Without Reordering Defaults

if ~isstruct(optionOverrides) || ~isscalar(optionOverrides)
    error("resolveOptions:InvalidStructures", "optionOverrides must be a scalar struct.");
end

% Stable set operations keep both lists in the caller's input order.
defaultNames  = string(fieldnames(defaultOptions));
overrideNames = string(fieldnames(optionOverrides));
unknownNames  = setdiff(overrideNames, defaultNames, "stable");
knownNames    = intersect(overrideNames, defaultNames, "stable");

%% Section 2: Apply Only Known Nonempty Overrides

% An empty override means "keep the default". The owning function checks
% the type and range of every value transferred here.
resolvedOptions = defaultOptions;
for fieldName = reshape(knownNames, 1, [])
    if ~isempty(optionOverrides.(fieldName))
        resolvedOptions.(fieldName) = optionOverrides.(fieldName);
    end
end
end
