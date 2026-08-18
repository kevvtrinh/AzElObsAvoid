function [state, control] = sampleAzElHs3Solution( ...
        solution, meshTau, queryTau)
%% Section 0: Header & Readme
% SYNTAX
%   [state, control] = azElInternal.sampleAzElHs3Solution( ...
%       solution, meshTau, queryTau)
%**************************************************************************
% PURPOSE
%   - Evaluate one HS-3 collocation solution at normalized query times.
%**************************************************************************
% INPUTS
%   - solution (scalar struct)
%       HS-3 knot states, knot controls, midpoint controls, and times.
%   - meshTau (N-by-1 numeric vector)
%       Strictly increasing normalized knot times.
%   - queryTau (M-by-1 numeric vector)
%       Normalized query times. Values outside the mesh use the nearest
%       boundary segment and are limited to that segment.
%**************************************************************************
% OUTPUTS
%   - state (M-by-6 numeric matrix)
%       Rows contain [azimuth, elevation, azimuth velocity, elevation
%       velocity, azimuth acceleration, elevation acceleration].
%   - control (M-by-2 numeric matrix)
%       Rows contain azimuth and elevation jerk.
%**************************************************************************
% UNITS
%   - Position is degrees. Time is seconds. Velocity, acceleration, and
%     jerk use degrees per second, second squared, and second cubed.
%**************************************************************************

%% Section 1: Evaluate The Requested Times

queryTau = queryTau(:);
state = zeros(numel(queryTau), 6);
control = zeros(numel(queryTau), 2);
duration_s = solution.FinalTime_s - solution.InitialTime_s;
segmentCount = numel(meshTau) - 1;
for segmentIndex = 1:segmentCount
    if segmentIndex < segmentCount
        queryIsInSegment = queryTau >= meshTau(segmentIndex) & ...
            queryTau < meshTau(segmentIndex + 1);
    else
        queryIsInSegment = queryTau >= meshTau(segmentIndex) & ...
            queryTau <= meshTau(segmentIndex + 1);
    end
    if segmentIndex == 1
        queryIsInSegment = queryIsInSegment | queryTau < meshTau(1);
    elseif segmentIndex == segmentCount
        queryIsInSegment = queryIsInSegment | queryTau > meshTau(end);
    end
    queryIndex = find(queryIsInSegment);
    if isempty(queryIndex)
        continue;
    end
    segmentWidth = meshTau(segmentIndex + 1) - meshTau(segmentIndex);
    localTau = (queryTau(queryIndex) - meshTau(segmentIndex)) / ...
        segmentWidth;
    localTau = min(1, max(0, localTau));
    segmentDuration_s = duration_s * segmentWidth;
    [state(queryIndex, :), control(queryIndex, :)] = ...
        azElInternal.evaluateAzElHs3Segment( ...
        solution, segmentIndex, segmentDuration_s, localTau);
end
end
