function [resolvedOptions, unknownOptionNames] = resolveOptions(defaultOptions, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   [resolvedOptions, unknownOptionNames] = ...
%       obstacleAvoidance.input.resolveOptions(defaultOptions, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Start with default options, then replace supported fields with the
%     nonempty values supplied by the caller. Keep the original field order.
%**************************************************************************
% INPUTS
%   - defaultOptions (scalar struct)
%       Complete option defaults.
%   - optionOverrides (scalar struct)
%       Options supplied by the caller; missing or empty fields keep defaults.
%**************************************************************************
% OUTPUTS
%   - resolvedOptions (scalar struct)
%       Default options with the supported, nonempty replacements applied.
%   - unknownOptionNames (N-by-1 string vector)
%       Unsupported field names, in their supplied order. These fields are
%       ignored here so the caller can report them. Invalid override structure
%       throws an error.
%**************************************************************************
% UNITS
%   - Values retain the units documented by the owning function.
%**************************************************************************

%% Section 1: Identify Supported And Unknown Options

if ~isstruct(optionOverrides) || ~isscalar(optionOverrides)
    error("resolveOptions:InvalidStructures", "optionOverrides must be a scalar struct.");
end

% Keep the supplied field order so warnings or errors list names as entered.
defaultOptionNames   = string(fieldnames(defaultOptions));
overrideOptionNames  = string(fieldnames(optionOverrides));
unknownOptionNames   = setdiff(overrideOptionNames, defaultOptionNames, "stable");
supportedOptionNames = intersect(overrideOptionNames, defaultOptionNames, "stable");

%% Section 2: Replace Defaults With Supported Values

% An empty override means "keep the default". The calling function is
% responsible for checking each value's type and allowed range.
resolvedOptions = defaultOptions;
for fieldName = reshape(supportedOptionNames, 1, [])
    if ~isempty(optionOverrides.(fieldName))
        resolvedOptions.(fieldName) = optionOverrides.(fieldName);
    end
end
end
