function [selectedPairs, maximumUnloadedLineViolation_units] = findViolatedPlanePairs( ...
    solverValues, separatingPlanes, regionActiveBySegment, loadedPlanePairs, degree, ...
    slackColumnByPair, trajectoryRoundoffReserve_units, constraintTolerance_units)
%% Section 0: Header & Readme
% SYNTAX
%   [selectedPairs, maximumUnloadedLineViolation_units] = ...
%       bmtpEngine.separation.findViolatedPlanePairs(solverValues, separatingPlanes, ...
%       regionActiveBySegment, loadedPlanePairs, degree, slackColumnByPair, ...
%       trajectoryRoundoffReserve_units, constraintTolerance_units)
%**************************************************************************
% PURPOSE
%   - After a solve using some separating lines, check the lines left out.
%     Add the worst violated line for each motion segment in the next solve.
%     Bezier coefficient bounds check the whole assigned curve portion,
%     including positions between samples.
%**************************************************************************
% INPUTS
%   - solverValues (numeric column)
%       Current solver vector: interleaved x/y controls for every segment,
%       followed by any other unknowns such as slack.
%   - separatingPlanes (S-by-R struct array)
%       Line records with normals, offsets, and selected curve fractions.
%   - regionActiveBySegment (S-by-R logical array)
%       True for segment/region pairs that need a separating line.
%   - loadedPlanePairs (S-by-R logical array)
%       True for pairs whose line rows were already in the solver matrix.
%   - degree (positive integer scalar)
%       Bezier degree of every curve segment.
%   - slackColumnByPair (S-by-R numeric array)
%       Solver column of the optional slack for each pair; zero means none.
%   - trajectoryRoundoffReserve_units (nonnegative numeric scalar)
%       Numerical gap required on the curve side of each line.
%   - constraintTolerance_units (nonnegative numeric scalar)
%       A pair is added only when its violation is greater than this value.
%**************************************************************************
% OUTPUTS
%   - selectedPairs (S-by-R logical array)
%       At most one unloaded pair per segment: the greatest violation above
%       tolerance. Equal violations keep the first region encountered.
%   - maximumUnloadedLineViolation_units (numeric scalar)
%       Greatest raw violation across all unloaded active pairs, even if
%       below tolerance. A nonpositive value meets every checked bound;
%       -Inf means there was no unloaded active pair to check.
%**************************************************************************
% UNITS
%   - Line violations, offsets, slack, and reserves are coordinate units.
%**************************************************************************

%% Section 1: Check All Applicable Pairs Not Yet Loaded

% For each segment, keep only its worst unloaded line. If two lines exceed
% the bound by 0.02 and 0.05 units, add the 0.05-unit line next, provided
% that violation exceeds the tolerance. This limits rows added per solve.

segmentCount      = size(regionActiveBySegment, 1);
planeSegmentCount = size(separatingPlanes, 1);
selectedPairs     = false(size(regionActiveBySegment));

maximumUnloadedLineViolation_units = -Inf;
maximumSegmentViolation_units     = -Inf(segmentCount, 1);
selectedRegionIndexBySegment       = zeros(segmentCount, 1);

startNormalWeights = 1 - (0:degree + 1).' / (degree + 1);
endNormalWeights   = 1 - startNormalWeights;
controlPoint_units = permute(reshape(solverValues(1:planeSegmentCount * (degree + 1) * 2), ...
    2, degree + 1, planeSegmentCount), [3, 2, 1]);
savedOverlapFractions       = NaN(planeSegmentCount, 2);
savedSubcurveControls_units = cell(planeSegmentCount, 1);
for pairIndex = reshape(find(regionActiveBySegment & ~loadedPlanePairs), 1, [])
    [segmentIndex, regionIndex] = ind2sub(size(regionActiveBySegment), pairIndex);
    plane                  = separatingPlanes(segmentIndex, regionIndex);
    pairControlPoint_units = squeeze(controlPoint_units(segmentIndex, :, :));

    % Several obstacles may use the same portion of this curve segment.
    % Reuse that portion's controls when its start/end fractions match.
    if ~isequal(plane.TimeFraction, [0, 1])
        if isequal(savedOverlapFractions(segmentIndex, :), plane.TimeFraction)
            pairControlPoint_units = savedSubcurveControls_units{segmentIndex};
        else
            pairControlPoint_units = bmtpEngine.motion.restrictBezier(pairControlPoint_units, plane.TimeFraction);
            savedOverlapFractions(segmentIndex, :)    = plane.TimeFraction;
            savedSubcurveControls_units{segmentIndex} = pairControlPoint_units;
        end
    end

    % A degree-D curve times a linear normal gives D+2 Bezier coefficients.
    % Match createPlaneRows so an unloaded line is tested by the same
    % coefficient bounds that would be inserted into the solver.
    normalPositionCoefficients_units = startNormalWeights .* [pairControlPoint_units * plane.Normal(1, :).'; 0] + ...
        endNormalWeights .* [0; pairControlPoint_units * plane.Normal(2, :).'];
    lineOffsetCoefficients_units = startNormalWeights * plane.Offset_units(1) + ...
        endNormalWeights * plane.Offset_units(2);
    clearanceSlack_units         = 0;
    slackColumnIndex             = slackColumnByPair(segmentIndex, regionIndex);
    if slackColumnIndex > 0
        clearanceSlack_units = solverValues(slackColumnIndex);
    end
    % Each coefficient must satisfy line-side value - slack + reserve <= 0.
    % The largest positive result is this pair's violation. Using > below
    % keeps the earlier region selected when two violations tie.
    pairLineViolation_units = max(normalPositionCoefficients_units + lineOffsetCoefficients_units - ...
        clearanceSlack_units + trajectoryRoundoffReserve_units);
    maximumUnloadedLineViolation_units = max(maximumUnloadedLineViolation_units, pairLineViolation_units);
    pairIsWorstViolation               = pairLineViolation_units > constraintTolerance_units && ...
        pairLineViolation_units > maximumSegmentViolation_units(segmentIndex);
    if pairIsWorstViolation
        maximumSegmentViolation_units(segmentIndex) = pairLineViolation_units;
        selectedRegionIndexBySegment(segmentIndex)  = regionIndex;
    end
end

%% Section 2: Select The Largest Violation In Each Curve Segment

for segmentIndex = reshape(find(selectedRegionIndexBySegment > 0), 1, [])
    selectedPairs(segmentIndex, selectedRegionIndexBySegment(segmentIndex)) = true;
end
end
