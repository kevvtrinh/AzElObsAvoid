function state = normalizePlannerState(state, stateName)
%% Section 0: Header & Readme
% SYNTAX
%   state = obstacleAvoidance.input.normalizePlannerState(state, stateName)
%
% PURPOSE
%   - Validate and normalize one public planner endpoint state.
%
% INPUTS
%   - state (scalar struct)
%       Requires time_s and two-element position_units. Missing or empty
%       velocity_units_s and acceleration_units_s2 fields default to zero.
%   - stateName (scalar text)
%       Input name included in an identified structural validation error.
%
% OUTPUTS
%   - state (scalar struct)
%       Numeric state values are double, with coordinate histories as rows.
%
% UNITS
%   - Position is coordinate units; time is seconds; derivatives use units/s and units/s^2.
%

%% Section 1: Validate And Normalize The State

if ~isstruct(state) || ~isscalar(state) || ~all(isfield(state, {'time_s', 'position_units'}))
    error("planTrajectory:InvalidState", "%s must be a scalar struct with time_s and position_units.", stateName);
end
validateattributes(state.time_s, {'numeric'}, {'real', 'finite', 'scalar'});
validateattributes(state.position_units, {'numeric'}, {'real', 'finite', 'vector', 'numel', 2});
state.time_s       = double(state.time_s);
state.position_units = double(state.position_units(:).');
% Apply the required validation or transfer to each field name.
for fieldName = ["velocity_units_s", "acceleration_units_s2"]
    if ~isfield(state, fieldName) || isempty(state.(fieldName))
        state.(fieldName) = [0 0];
    else
        validateattributes(state.(fieldName), {'numeric'}, {'real', 'finite', 'vector', 'numel', 2});
        state.(fieldName) = double(state.(fieldName)(:).');
    end
end
end
