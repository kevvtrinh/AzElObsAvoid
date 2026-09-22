function timePowerCones = createTimePowerCones(decisionVariableCount, timePowerIndices)
%% Section 0: Header & Readme
% SYNTAX
%   timePowerCones = bmtpEngine.optimization.createTimePowerCones( ...
%       decisionVariableCount, timePowerIndices)
%**************************************************************************
% PURPOSE
%   - Link the solver variables representing 1, T, T^2, and T^3 with two
%     inequalities that coneprog can handle. These constraints alone do
%     not force the variables to equal the exact powers of one duration.
%**************************************************************************
% INPUTS
%   - decisionVariableCount (integer scalar)
%       Number of unknown values in the solver vector.
%   - timePowerIndices (1-by-4 numeric row)
%       Locations of p0, p1, p2, p3, representing [1 T T^2 T^3].
%**************************************************************************
% OUTPUTS
%   - timePowerCones (2-by-1 secondordercone array)
%       Constraints enforcing p0 x p2 >= p1^2 and p1 x p3 >= p2^2.
%**************************************************************************
% UNITS
%   - Power variables represent 1, seconds, seconds^2, and seconds^3.
%**************************************************************************

%% Section 1: Express Each Time-Power Inequality As A Vector-Length Bound

% For three successive variables a, b, c, require
% norm([2 x b; a - c]) <= a + c. Squaring gives b^2 <= a x c.
% This vector-length form is a second-order cone constraint for coneprog.
emptyCone = secondordercone( ...
    zeros(2, decisionVariableCount), zeros(2, 1), zeros(decisionVariableCount, 1), 0);
timePowerCones = repmat(emptyCone, 2, 1);
for coneIndex = 1:2
    leftSideMap = zeros(2, decisionVariableCount);
    leftSideMap(1, timePowerIndices(coneIndex + 1)) = 2;
    leftSideMap(2, timePowerIndices(coneIndex))     = 1;
    leftSideMap(2, timePowerIndices(coneIndex + 2)) = -1;
    rightSideWeights = zeros(decisionVariableCount, 1);
    rightSideWeights(timePowerIndices([coneIndex, coneIndex + 2])) = 1;
    timePowerCones(coneIndex) = secondordercone(leftSideMap, zeros(2, 1), rightSideWeights, 0);
end
end
