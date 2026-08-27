function [resolvedOptions, unknownNames] = resolveOptions(defaultOptions, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   [resolvedOptions, unknownNames] = azElInput.resolveOptions( ...
%       defaultOptions, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Apply the repository-wide partial-option merge invariant once.
%   - Preserve default field order and report, but never apply, unknown
%     fields so each public caller can emit its own warning identifier.
%**************************************************************************
% INPUTS
%   - defaultOptions (scalar struct)
%       Complete defaults. Its field order is preserved in the result.
%   - optionOverrides (scalar struct)
%       Partial overrides. Empty values retain their corresponding default.
%**************************************************************************
% OUTPUTS
%   - resolvedOptions (scalar struct)
%       Defaults with known, nonempty overrides applied.
%   - unknownNames (N-by-1 string vector)
%       Ignored override fields in caller-supplied order.
%**************************************************************************
% UNITS
%   - Values retain the units documented by their owning public function.
%**************************************************************************

%% Section 1: Classify Override Names Without Reordering Defaults

% Unknown names are returned to the public owner so it can emit one warning
% with the correct identifier. They are never copied into runtime behavior.
if ~isstruct(defaultOptions) || ~isscalar(defaultOptions) || ~isstruct(optionOverrides) || ~isscalar(optionOverrides)
    error("azElInput:resolveOptions:InvalidStructures", ...
        "defaultOptions and optionOverrides must be scalar structs.");
end

defaultNames = string(fieldnames(defaultOptions));
overrideNames = string(fieldnames(optionOverrides));
unknownNames = setdiff(overrideNames, defaultNames, "stable");
knownNames = intersect(overrideNames, defaultNames, "stable");

%% Section 2: Apply Only Known Nonempty Overrides

% Empty means "use the default" throughout the repository. Centralizing that
% rule prevents individual planner stages from interpreting partial options
% differently.
resolvedOptions = defaultOptions;

% Apply overrides in caller order while preserving the default struct's field order.
for fieldIndex = 1:numel(knownNames)
    fieldName = knownNames(fieldIndex);
    if ~isempty(optionOverrides.(fieldName))
        resolvedOptions.(fieldName) = optionOverrides.(fieldName);
    end
end
end
