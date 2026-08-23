function [resolvedOptions, unknownNames] = resolveOptions( ...
        defaultOptions, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   [resolvedOptions, unknownNames] = azElPlannerMethods.hs3.internal.resolveOptions( ...
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

if ~isstruct(defaultOptions) || ~isscalar(defaultOptions) || ...
        ~isstruct(optionOverrides) || ~isscalar(optionOverrides)
    error("azElInternal:resolveOptions:InvalidStructures", ...
        "defaultOptions and optionOverrides must be scalar structs.");
end

defaultNames = string(fieldnames(defaultOptions));
overrideNames = string(fieldnames(optionOverrides));
unknownNames = setdiff(overrideNames, defaultNames, "stable");
knownNames = intersect(overrideNames, defaultNames, "stable");

resolvedOptions = defaultOptions;

% Copy each recognized override in caller order; empty values intentionally
% leave the corresponding method default unchanged.
for fieldIndex = 1:numel(knownNames)
    fieldName = knownNames(fieldIndex);
    if ~isempty(optionOverrides.(fieldName))
        resolvedOptions.(fieldName) = optionOverrides.(fieldName);
    end
end
end
