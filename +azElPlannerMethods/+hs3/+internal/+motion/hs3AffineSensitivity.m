function model = hs3AffineSensitivity(segmentCount, duration_s, evaluationTau)
%% Section 0: Header & Readme
% SYNTAX
%   model = ...
%       azElPlannerMethods.hs3.internal.motion.hs3AffineSensitivity( ...
%       segmentCount, duration_s, evaluationTau)
%**************************************************************************
% PURPOSE
%   - Provide a deprecated compatibility alias to hs3.affineSensitivity.
%**************************************************************************
% INPUTS
%   - segmentCount (positive integer scalar), equal-duration segment count.
%   - duration_s (positive scalar), complete duration.
%   - evaluationTau (numeric vector), normalized evaluation coordinates.
%**************************************************************************
% OUTPUTS
%   - model (scalar struct), exact affine sensitivity maps.
%**************************************************************************
% UNITS
%   - duration_s uses seconds; evaluationTau is dimensionless.
%**************************************************************************
model = hs3.affineSensitivity(segmentCount, duration_s, evaluationTau);
model.SegmentDuration_s = model.SegmentDuration;
model = rmfield(model, "SegmentDuration");
end
