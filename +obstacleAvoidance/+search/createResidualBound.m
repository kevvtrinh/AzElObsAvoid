function residualBound_units2 = createResidualBound(coordinateScale_units)
%% Section 0: Header & Readme
% SYNTAX
%   residualBound_units2 = obstacleAvoidance.search.createResidualBound(coordinateScale_units)
%**************************************************************************
% PURPOSE
%   - The one algebraic slack shared by the exact affine predicate, its
%     broad phase, and its layer certificate.
%**************************************************************************
% INPUTS
%   - coordinateScale_units (numeric scalar or column)
%       Largest coordinate magnitude entering the residuals, one per residual.
%**************************************************************************
% OUTPUTS
%   - residualBound_units2 (same shape as the input)
%       Slack on a squared-coordinate residual.
%**************************************************************************
% UNITS
%   - Squared coordinate units.
%**************************************************************************

%% Section 1: Scale The Slack

% Shared algebraic slack for the exact predicate and its broad phase.
residualBound_units2 = 4096 * eps(coordinateScale_units .^ 2);
end
