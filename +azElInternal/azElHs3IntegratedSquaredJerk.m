function integratedSquaredJerk = azElHs3IntegratedSquaredJerk( ...
        solution, meshTau)
%% Section 0: Header & Readme
% SYNTAX
%   integratedSquaredJerk = ...
%       azElInternal.azElHs3IntegratedSquaredJerk(solution, meshTau)
%**************************************************************************
% PURPOSE
%   - Integrate squared quadratic HS-3 jerk exactly on every segment.
%**************************************************************************
% INPUTS
%   - solution (scalar struct)
%       HS-3 state and jerk controls.
%   - meshTau (N-by-1 numeric vector)
%       Strictly increasing normalized knot times.
%**************************************************************************
% OUTPUTS
%   - integratedSquaredJerk (nonnegative numeric scalar)
%       Sum of both axes over the full motion interval.
%**************************************************************************
% UNITS
%   - The result uses square degrees per second to the fifth power.
%**************************************************************************

%% Section 1: Integrate Each Segment

segmentDuration_s = (solution.FinalTime_s - solution.InitialTime_s) * ...
    diff(meshTau);
powerIndex = (0:2).';
integrationGram = 1 ./ (powerIndex + powerIndex.' + 1);
integratedSquaredJerk = 0;
for segmentIndex = 1:numel(segmentDuration_s)
    [~, controlPower] = ...
        azElInternal.buildAzElHs3SegmentPolynomials( ...
        solution.KnotState(segmentIndex, :), ...
        solution.KnotControl(segmentIndex, :), ...
        solution.MidpointControl(segmentIndex, :), ...
        solution.KnotControl(segmentIndex + 1, :), ...
        segmentDuration_s(segmentIndex));
    integratedSquaredJerk = integratedSquaredJerk + ...
        segmentDuration_s(segmentIndex) * sum( ...
        controlPower .* (integrationGram * controlPower), "all");
end
end
