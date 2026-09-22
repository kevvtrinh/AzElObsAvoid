function decisionVariableIndices = controlIndexOf(segmentIndex, controlPointIndex, axisIndex, degree)
%% Section 0: Header & Readme
% SYNTAX
%   decisionVariableIndices = bmtpEngine.optimization.controlIndexOf( ...
%       segmentIndex, controlPointIndex, axisIndex, degree)
%**************************************************************************
% PURPOSE
%   - Find where a control point's x/y coordinates are stored in the
%     solver's single vector of unknown values.
%**************************************************************************
% INPUTS
%   - segmentIndex (positive integer scalar)
%       One-based trajectory segment index.
%   - controlPointIndex (nonnegative integer scalar)
%       Zero-based Bezier control index within the segment.
%   - axisIndex (positive integer scalar or vector)
%       1 selects x, 2 selects y, and [1 2] selects both coordinates.
%   - degree (positive integer scalar)
%       Bezier polynomial degree.
%**************************************************************************
% OUTPUTS
%   - decisionVariableIndices (positive integer scalar or vector)
%       Locations of the requested coordinates in the solver vector.
%**************************************************************************
% UNITS
%   - All inputs and outputs are dimensionless indices or counts.
%**************************************************************************

%% Section 1: Locate The Segment, Control Point, And Coordinate

% Store [P0x P0y P1x P1y ...] for each segment, then the next segment.
% At degree 5, each segment uses 6 x 2 = 12 values. Segment 2, control 0
% therefore has x/y at indices [13 14].
decisionVariableIndices = ((segmentIndex - 1) * (degree + 1) + controlPointIndex) * 2 + axisIndex;
end
