function [certified, minimumClearance_deg] = ...
        checkSeedCorridor(trajectory, obstacles, clearanceTolerance_deg)
%% Section 0: Header & Readme
% SYNTAX
%   [certified, minimumClearance_deg] = ...
%       obstacleAvoidance.validation.checkSeedCorridor( ...
%       trajectory, obstacles, clearanceTolerance_deg)
%**************************************************************************
% PURPOSE
%   - Check whether a motion stays on the obstacle side allowed by its seed.
%   - Preserve the established independent corridor-certificate mathematics.
%**************************************************************************
% INPUTS
%   - trajectory (scalar motion struct)
%       Complete polynomial motion and optional seed-corridor evidence.
%   - obstacles (prepared canonical obstacle struct array)
%       Protected geometry used to verify corridor coverage.
%   - clearanceTolerance_deg (nonnegative finite scalar)
%       Required separation allowance in degrees.
%**************************************************************************
% OUTPUTS
%   - certified (scalar logical)
%       True only when the complete corridor certificate is independently valid.
%   - minimumClearance_deg (scalar numeric)
%       Certified clearance in degrees or NaN when no certificate passes.
%**************************************************************************
% UNITS
%   - Geometry, clearance, and tolerance are degrees.
%**************************************************************************

%% Section 1: Check The Seed-Side Certificate

% A seed corridor is optional acceleration evidence, not planner approval.
% The full validator decides final acceptance after all other checks agree.

[certified, minimumClearance_deg] = ...
    obstacleAvoidance.validation.certifySeedCorridor( ...
    trajectory, obstacles, clearanceTolerance_deg);
end
