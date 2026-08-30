function state = normalizePlannerState(state, stateName)
%% Section 0: Header & Readme
% SYNTAX
%   state = obstacleAvoidance.input.normalizePlannerState(state, stateName)
%**************************************************************************
% PURPOSE
%   - Validate and normalize one public planner endpoint state.
%**************************************************************************
% INPUTS
%   - state (scalar struct)
%       Requires time_s and two-element position_deg. Missing or empty
%       velocity_deg_s and acceleration_deg_s2 fields default to zero.
%   - stateName (scalar text)
%       Input name included in an identified structural validation error.
%**************************************************************************
% OUTPUTS
%   - state (scalar struct)
%       Numeric state values are double, with coordinate histories as rows.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s and deg/s^2.
%**************************************************************************

%% Section 1: Validate And Normalize The State

if ~isstruct(state) || ~isscalar(state) || ...
        ~all(isfield(state, {'time_s', 'position_deg'}))
    error("planTrajectory:InvalidState", ...
        "%s must be a scalar struct with time_s and position_deg.", stateName);
end
validateattributes(state.time_s, {'numeric'}, ...
    {'real', 'finite', 'scalar'});
validateattributes(state.position_deg, {'numeric'}, ...
    {'real', 'finite', 'vector', 'numel', 2});
state.time_s = double(state.time_s);
state.position_deg = double(state.position_deg(:).');
for fieldName = ["velocity_deg_s", "acceleration_deg_s2"]
    if ~isfield(state, fieldName) || isempty(state.(fieldName))
        state.(fieldName) = [0 0];
    else
        validateattributes(state.(fieldName), {'numeric'}, ...
            {'real', 'finite', 'vector', 'numel', 2});
        state.(fieldName) = double(state.(fieldName)(:).');
    end
end
end
