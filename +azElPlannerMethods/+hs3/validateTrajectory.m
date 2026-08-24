function validation = validateTrajectory(varargin)
%% Section 0: Header & Readme
% SYNTAX
%   validation = azElPlannerMethods.hs3.validateTrajectory()
%   validation = azElPlannerMethods.hs3.validateTrajectory(result)
%   validation = azElPlannerMethods.hs3.validateTrajectory( ...
%       trajectory, obstacles, initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Preserve the method-qualified validation entry point for compatibility.
%   - Forward all validation to the single method-neutral public owner.
%**************************************************************************
% INPUTS
%   - varargin (zero arguments, one planner result, or six explicit inputs)
%**************************************************************************
% OUTPUTS
%   - validation (canonical stable validation record)
%**************************************************************************

%% Section 1: Forward The Preserved Contract

validation = validateAzElTrajectory(varargin{:});
end
