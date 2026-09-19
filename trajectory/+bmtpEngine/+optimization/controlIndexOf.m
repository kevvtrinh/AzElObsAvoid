function index = controlIndexOf(segmentIndex, controlIndex, axisIndex, degree)
%% Section 0: Header & Readme
% SYNTAX
%   index = bmtpEngine.optimization.controlIndexOf( ...
%       segmentIndex, controlIndex, axisIndex, degree)
%**************************************************************************
% PURPOSE
%   - Map trajectory controls into the conic decision vector.
%**************************************************************************
% INPUTS
%   - segmentIndex (positive integer scalar)
%       One-based trajectory segment index.
%   - controlIndex (nonnegative integer scalar)
%       Zero-based Bezier control index within the segment.
%   - axisIndex (positive integer scalar or vector)
%       One-based coordinate-axis indices.
%   - degree (positive integer scalar)
%       Bezier polynomial degree.
%**************************************************************************
% OUTPUTS
%   - index (positive integer scalar or vector)
%       Matching locations in the conic decision vector.
%**************************************************************************
% UNITS
%   - All inputs and outputs are dimensionless indices or counts.
%**************************************************************************

%% Section 1: Map The Control Index

% Map trajectory controls into the conic decision vector.
index = ((segmentIndex - 1) * (degree + 1) + controlIndex) * 2 + axisIndex;
end
