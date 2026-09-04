function value = stripPlannerRefactorRuntime(value)
%% Section 0: Header & Readme
% SYNTAX
%   value = stripPlannerRefactorRuntime(value)
%**************************************************************************
% PURPOSE
%   - Remove measured elapsed-time details before physical result comparison.
%   - Preserve arrival, trajectory, segment, waypoint, and obstacle times.
%**************************************************************************
% INPUTS
%   - value (any MATLAB value)
%       Structures and cells are traversed recursively. Other values pass
%       through unchanged.
%**************************************************************************
% OUTPUTS
%   - value (same general type as the input)
%       Runtime measurements are removed from structure fields.
%**************************************************************************
% UNITS
%   - Removed fields measure wall-clock seconds, not physical motion time.
%**************************************************************************

%% Section 1: Remove Runtime Fields Recursively

% Elapsed measurements and the retired StageOutputs inspection cache do not
% describe physical behavior. Ignore both when comparing historical captures.

if isstruct(value)
    fieldNames = string(fieldnames(value));
    isRuntimeField = contains(fieldNames, "ElapsedTime") | ...
        fieldNames == "FirstValidatedMotionTime_s" | ...
        fieldNames == "StageTiming" | fieldNames == "StageOutputs";
    if any(isRuntimeField)
        value = rmfield(value, cellstr(fieldNames(isRuntimeField)));
    end
    remainingNames = string(fieldnames(value));
    for elementIndex = 1:numel(value)
        for fieldIndex = 1:numel(remainingNames)
            fieldName = remainingNames(fieldIndex);
            value(elementIndex).(fieldName) = ...
                stripPlannerRefactorRuntime( ...
                value(elementIndex).(fieldName));
        end
    end
elseif iscell(value)
    for elementIndex = 1:numel(value)
        value{elementIndex} = stripPlannerRefactorRuntime( ...
            value{elementIndex});
    end
end
end
