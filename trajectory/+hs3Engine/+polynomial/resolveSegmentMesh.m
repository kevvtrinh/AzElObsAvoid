function [segmentBreakTau, segmentDuration, isUniformMesh] = ...
        resolveSegmentMesh(segmentCount, duration, segmentBreakTau)
%% Section 0: Header & Readme
% SYNTAX
%   [segmentBreakTau, segmentDuration, isUniformMesh] = ...
%       hs3Engine.polynomial.resolveSegmentMesh(segmentCount, duration)
%   [segmentBreakTau, segmentDuration, isUniformMesh] = ...
%       hs3Engine.polynomial.resolveSegmentMesh( ...
%       segmentCount, duration, segmentBreakTau)
%**************************************************************************
% PURPOSE
%   - Resolve one normalized HS3 segment partition while preserving exact
%     legacy scalar arithmetic for an exactly uniform mesh.
%**************************************************************************
% INPUTS
%   - segmentCount (positive integer scalar)
%       Number of polynomial segments.
%   - duration (positive finite scalar)
%       Complete trajectory duration in caller-defined time units.
%   - segmentBreakTau ((N+1)-element vector, optional; default uniform)
%       Strictly increasing normalized segment boundaries from zero to one.
%**************************************************************************
% OUTPUTS
%   - segmentBreakTau ((N+1)-by-1 double)
%       Validated normalized segment boundaries.
%   - segmentDuration (positive scalar or N-by-1 double)
%       Scalar legacy duration for a uniform mesh, otherwise one duration
%       per segment.
%   - isUniformMesh (logical scalar)
%       True only when the normalized boundaries exactly equal the legacy
%       uniform construction.
%**************************************************************************
% UNITS
%   - segmentBreakTau is dimensionless. duration and segmentDuration use
%     the caller's time unit.
%**************************************************************************

%% Section 1: Validate Inputs & Resolve The Mesh

validateattributes(segmentCount, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'positive'});
validateattributes(duration, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'positive'});
uniformBreakTau = (0:segmentCount).' / segmentCount;
if nargin < 3 || isempty(segmentBreakTau)
    segmentBreakTau = uniformBreakTau;
end
validateattributes(segmentBreakTau, {'numeric'}, ...
    {'real', 'finite', 'vector', 'numel', segmentCount + 1});
segmentBreakTau = double(segmentBreakTau(:));
endpointTolerance = 32 * eps;
hasInvalidEndpoint = abs(segmentBreakTau(1)) > endpointTolerance || ...
    abs(segmentBreakTau(end) - 1) > endpointTolerance;
if hasInvalidEndpoint || any(diff(segmentBreakTau) <= 0)
    error("resolveSegmentMesh:InvalidSegmentBreakTau", ...
        "segmentBreakTau must strictly increase from zero to one.");
end
segmentBreakTau([1 end]) = [0; 1];
isUniformMesh = isequal(segmentBreakTau, uniformBreakTau);
if isUniformMesh
    % Multiplying duration by diff of a nominally uniform vector is not
    % bitwise equivalent and can change an ill-conditioned solver basin.
    segmentDuration = double(duration) / segmentCount;
else
    segmentDuration = double(duration) * diff(segmentBreakTau);
end
end
