function [resolvedOptions, unknownNames] = resolveOptions(defaultOptions, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   [resolvedOptions, unknownNames] = obstacleAvoidance.input.resolveOptions( ...
%       defaultOptions, optionOverrides)
%
% PURPOSE
%   - Apply the repository-wide partial-option merge rule once.
%   - Preserve default field order and report, but never apply, unknown
%     fields so each public caller can emit its own warning identifier.
%
% INPUTS
%   - defaultOptions (scalar struct)
%       Complete defaults. Its field order is preserved in the result.
%   - optionOverrides (scalar struct)
%       Partial overrides. Empty values retain their corresponding default.
%
% OUTPUTS
%   - resolvedOptions (scalar struct)
%       Defaults with known, nonempty overrides applied.
%   - unknownNames (N-by-1 string vector)
%       Ignored override fields in caller-supplied order.
%
% UNITS
%   - Values retain the units documented by their owning public function.
%

%% Section 1: Classify Override Names Without Reordering Defaults

% Find unknown options without changing defaults.

% Return unknown names so the caller can issue one warning.
if ~isstruct(optionOverrides) || ~isscalar(optionOverrides)
    error("resolveOptions:InvalidStructures", "optionOverrides must be a scalar struct.");
end

defaultNames  = string(fieldnames(defaultOptions));
overrideNames = string(fieldnames(optionOverrides));
% Keep unknown names in the user's input order.
unknownNames = setdiff(overrideNames, defaultNames, "stable");
knownNames   = intersect(overrideNames, defaultNames, "stable");

%% Section 2: Apply Only Known Nonempty Overrides

% Apply nonempty overrides; the caller checks their types and ranges.

resolvedOptions = defaultOptions;
% Apply the required validation or transfer to each field name.
for fieldName = reshape(knownNames, 1, [])
    if ~isempty(optionOverrides.(fieldName))
        resolvedOptions.(fieldName) = optionOverrides.(fieldName);
    end
end
end
