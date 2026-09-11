function soc = createTimePowerCones(variableCount, powerIndex)
%% Section 0: Header & Readme
% SYNTAX: soc = bmtpEngine.createTimePowerCones(variableCount, powerIndex)
% PURPOSE: Build the two Bernstein clock power cones.
% INPUTS: Decision-vector size and its four time-power indices.
% OUTPUTS: Two second-order cones enforcing p0*p2 >= p1^2 and p1*p3 >= p2^2.
% UNITS: The four power variables represent 1, seconds, seconds^2 and seconds^3.

%% Section 1: Assemble The Exact Constraints
% Create p0*p2>=p1^2 and p1*p3>=p2^2 as standard cones.
emptyCone = secondordercone(zeros(2, variableCount), zeros(2, 1), zeros(variableCount, 1), 0);
soc       = repmat(emptyCone, 2, 1);
for coneIndex = 1:2
    coneA = zeros(2, variableCount);
    coneA(1, powerIndex(coneIndex + 1)) = 2;
    coneA(2, powerIndex(coneIndex)) = 1;
    coneA(2, powerIndex(coneIndex + 2)) = -1;
    coneD = zeros(variableCount, 1);
    coneD(powerIndex([coneIndex coneIndex + 2])) = 1;
    soc(coneIndex) = secondordercone(coneA, zeros(2, 1), coneD, 0);
end
end
