function soc = createTimePowerCones(variableCount, powerIndex)
%% Section 0: Header & Readme
% SYNTAX
%   soc = bmtpEngine.optimization.createTimePowerCones(variableCount, powerIndex)
%**************************************************************************
% PURPOSE
%   - Build the two Bernstein clock-power cones.
%**************************************************************************
% INPUTS
%   - variableCount (integer scalar)
%       Decision-vector size.
%   - powerIndex (1-by-4 numeric row)
%       Decision-vector indices of the four clock-power variables.
%**************************************************************************
% OUTPUTS
%   - soc (2-by-1 secondordercone array)
%       Cones enforcing p0*p2 >= p1^2 and p1*p3 >= p2^2.
%**************************************************************************
% UNITS
%   - Power variables represent 1, seconds, seconds^2, and seconds^3.
%**************************************************************************

%% Section 1: Build The Two Consecutive Clock-Power Cones
% Each rotated quadratic p(i)*p(i+2) >= p(i+1)^2 becomes a standard cone on
% [2*p(i+1); p(i) - p(i+2)] bounded by the sum p(i) + p(i+2).
emptyCone = secondordercone(zeros(2, variableCount), zeros(2, 1), zeros(variableCount, 1), 0);
soc       = repmat(emptyCone, 2, 1);
for coneIndex = 1:2
    coneA = zeros(2, variableCount);
    coneA(1, powerIndex(coneIndex + 1)) = 2;
    coneA(2, powerIndex(coneIndex))     = 1;
    coneA(2, powerIndex(coneIndex + 2)) = -1;
    coneD = zeros(variableCount, 1);
    coneD(powerIndex([coneIndex, coneIndex + 2])) = 1;
    soc(coneIndex) = secondordercone(coneA, zeros(2, 1), coneD, 0);
end
end
