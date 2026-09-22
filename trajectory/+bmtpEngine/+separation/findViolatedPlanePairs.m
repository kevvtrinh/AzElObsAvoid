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
%   - Check every applicable line constraint not yet included in the solve.
%     Select the largest violation per curve segment for the next round.
%     Polynomial bounds cover the whole curve portion without sampling.
%**************************************************************************
% INPUTS
%   - solverValues (numeric column)
%       Current solver variables, beginning with interleaved x/y curve controls.
%   - separatingPlanes (S-by-R struct array)
%       Complete separating-plane set with normals, offsets, and time scope.
%   - regionActiveBySegment (S-by-R logical array)
%       Applicable curve-region pairs.
%   - loadedPlanePairs (S-by-R logical array)
%       Applicable pairs already included in the solve.
%   - degree (positive integer scalar)
%       Bezier degree of every curve segment.
%   - slackColumnByPair (S-by-R numeric array)
%       Optional slack column per pair; zero where the pair has no slack.
%   - trajectoryRoundoffReserve_units (nonnegative numeric scalar)
%       Numerical gap required on the curve side of each line.
%   - constraintTolerance_units (nonnegative numeric scalar)
%       How far a line bound must be exceeded before adding the pair.
%**************************************************************************
% OUTPUTS
%   - selectedPairs (S-by-R logical array)
%       At most one violated pair per segment, with the largest violation.
%   - maximumUnloadedLineViolation_units (numeric scalar)
%       Largest line-bound violation across every checked pair. A nonpositive
%       value satisfies the bound; -Inf means no unloaded pair was checked.
%**************************************************************************
% UNITS
%   - Line violations, offsets, slack, and reserves are coordinate units.
%**************************************************************************

%% Section 1: Check All Applicable Pairs Not Yet Loaded

% Check every unloaded pair, while remembering only the largest violation
% per segment to limit how many rows the next solve adds at once.

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

    % Multiplying a degree-D curve by a linear normal produces degree + 2
    % Bernstein coefficients. Each coefficient is the same line-side value
    % used by createPlaneRows, before accounting for offsets and slack.
    normalPositionCoefficients_units = startNormalWeights .* [pairControlPoint_units * plane.Normal(1, :).'; 0] + ...
        endNormalWeights .* [0; pairControlPoint_units * plane.Normal(2, :).'];
    lineOffsetCoefficients_units = startNormalWeights * plane.Offset_units(1) + ...
        endNormalWeights * plane.Offset_units(2);
    clearanceSlack_units         = 0;
    slackColumnIndex             = slackColumnByPair(segmentIndex, regionIndex);
    if slackColumnIndex > 0
        clearanceSlack_units = solverValues(slackColumnIndex);
    end
    % line-side value - slack + reserve <= 0. A positive result exceeds
    % the line bound. Equal worst violations keep the first encountered region.
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
