function [equality, segmentDefect] = azElHs3EqualityDefects( ...
        solution, meshTau)
%% Section 0: Header & Readme
% SYNTAX
%   [equality, segmentDefect] = ...
%       azElInternal.azElHs3EqualityDefects(solution, meshTau)
%**************************************************************************
% PURPOSE
%   - Verify stored midpoint and endpoint states against the HS-3 chain.
%**************************************************************************
% INPUTS
%   - solution (scalar struct)
%       HS-3 states and controls.
%   - meshTau (N-by-1 numeric vector)
%       Strictly increasing normalized knot times.
%**************************************************************************
% OUTPUTS
%   - equality (M-by-1 numeric vector)
%       Flattened midpoint and endpoint state defects.
%   - segmentDefect ((N-1)-by-1 numeric vector)
%       Maximum absolute state defect for each segment.
%**************************************************************************
% UNITS
%   - Defect entries retain the units of their state component.
%**************************************************************************

%% Section 1: Evaluate Stored Collocation States

segmentDuration_s = ...
    (solution.FinalTime_s - solution.InitialTime_s) * diff(meshTau);
predictedMidpointState = zeros(size(solution.MidpointState));
predictedLastState = zeros(size(solution.MidpointState));
for segmentIndex = 1:numel(segmentDuration_s)
    [predictedState, ~] = azElInternal.evaluateAzElHs3Segment( ...
        solution, segmentIndex, ...
        segmentDuration_s(segmentIndex), [0.5; 1]);
    predictedMidpointState(segmentIndex, :) = predictedState(1, :);
    predictedLastState(segmentIndex, :) = predictedState(2, :);
end
midpointDefect = solution.MidpointState - predictedMidpointState;
endpointDefect = solution.KnotState(2:end, :) - predictedLastState;
equality = [midpointDefect(:); endpointDefect(:)];
segmentDefect = max(abs([midpointDefect, endpointDefect]), [], 2);
end
