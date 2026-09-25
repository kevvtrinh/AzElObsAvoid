function result = createOptimizationResult(solverMessage, failureStage, failureKind, ...
    alternativeGuideEligible, controlPoint_units, segmentTime_s, separatingPlanes, taggedPairs)
%% Section 0: Header & Readme
% SYNTAX
%   result = bmtpEngine.optimization.createOptimizationResult(solverMessage, ...
%       failureStage, failureKind, alternativeGuideEligible, controlPoint_units, ...
%       segmentTime_s, separatingPlanes, taggedPairs)
%**************************************************************************
% PURPOSE
%   - Build the result record every trajectory optimizer returns to
%     bmtpEngine.solve, so the three optimizers agree on its fields and order.
%     Success means a candidate curve exists; the caller still checks it.
%**************************************************************************
% INPUTS
%   - solverMessage (string scalar)
%       Why the optimizer stopped, for the planner message.
%   - failureStage (string scalar)
%       "" on success; otherwise the stage that failed, such as "proposal".
%   - failureKind (string scalar)
%       "" on success; otherwise a stable failure name for the planner.
%   - alternativeGuideEligible (logical scalar)
%       True when the planner may try another route after this failure.
%   - controlPoint_units (S-by-(D+1)-by-2 numeric array)
%       Retained control points, or an empty array when nothing was retained.
%   - segmentTime_s (S-by-1 numeric vector or NaN)
%       Retained segment durations, or NaN when nothing was retained.
%   - separatingPlanes (S-by-R struct array)
%       Separating-line records that go with the retained controls.
%   - taggedPairs (S-by-R logical matrix)
%       True for each curve-segment/obstacle pair that has a separating line.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Success, SolverMessage, FailureStage, FailureKind,
%       AlternativeGuideEligible, ControlPoint_units, SegmentTime_s, Planes,
%       and TaggedPairs, in that order.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Assemble The Record

result = struct( ...
    'Success',                  ~isempty(controlPoint_units), ...
    'SolverMessage',            solverMessage, ...
    'FailureStage',             failureStage, ...
    'FailureKind',              failureKind, ...
    'AlternativeGuideEligible', alternativeGuideEligible, ...
    'ControlPoint_units',       controlPoint_units, ...
    'SegmentTime_s',            segmentTime_s, ...
    'Planes',                   separatingPlanes, ...
    'TaggedPairs',              taggedPairs);
end
