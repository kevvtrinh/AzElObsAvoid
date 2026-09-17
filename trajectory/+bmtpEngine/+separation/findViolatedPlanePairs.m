function [selectedPairs, maximumResidual] = findViolatedPlanePairs(decisionVector, planes, ...
        activePairs, retainedPairs, degree, slackColumnByPair, ...
        trajectoryReserve_units, tolerance)
%% Section 0: Header & Readme
% SYNTAX
%   [selectedPairs, maximumResidual] = bmtpEngine.separation.findViolatedPlanePairs( ...
%       decisionVector, planes, activePairs, retainedPairs, degree, ...
%       slackColumnByPair, trajectoryReserve_units, tolerance)
%**************************************************************************
% PURPOSE
%   - Separate omitted fixed-plane inequalities exactly and select the
%     greatest violation per motion span for the next constraint round.
%**************************************************************************
% INPUTS
%   - decisionVector (numeric column)
%       Current conic decision vector holding the composite controls first.
%   - planes (S-by-R struct array)
%       Complete separating-plane set with normals, offsets, and time scope.
%   - activePairs (S-by-R logical array)
%       Applicable curve-region pairs.
%   - retainedPairs (S-by-R logical array)
%       Applicable pairs already included in the solve.
%   - degree (positive integer scalar)
%       Bezier degree of every motion span.
%   - slackColumnByPair (S-by-R numeric array)
%       Optional slack column per pair; zero where the pair has no slack.
%   - trajectoryReserve_units (nonnegative numeric scalar)
%       Trajectory-side numerical reserve added to every residual.
%   - tolerance (nonnegative numeric scalar)
%       Conic tolerance a residual must exceed to count as violated.
%**************************************************************************
% OUTPUTS
%   - selectedPairs (S-by-R logical array)
%       At most one violated pair per motion span, the greatest violation.
%   - maximumResidual (numeric scalar)
%       Maximum residual over every omission; -Inf when none was evaluated.
%**************************************************************************
% UNITS
%   - Residuals, offsets, and the reserve are coordinate units.
%**************************************************************************

%% Section 1: Evaluate Every Omitted Pair In Physical Control Coordinates

segmentCount               = size(activePairs, 1);
planeSegmentCount          = size(planes, 1);
selectedPairs              = false(size(activePairs));
maximumResidual            = -Inf;
greatestViolationBySegment = -Inf(segmentCount, 1);
greatestRegionBySegment    = zeros(segmentCount, 1);
alpha    = 1 - (0:degree + 1).' / (degree + 1);
beta     = 1 - alpha;
controls = permute(reshape(decisionVector(1:planeSegmentCount * (degree + 1) * 2), ...
    2, degree + 1, planeSegmentCount), [3, 2, 1]);
lastTimeFraction      = NaN(planeSegmentCount, 2);
lastRestrictedControl = cell(planeSegmentCount, 1);
for pairIndex = reshape(find(activePairs & ~retainedPairs), 1, [])
    [segmentIndex, regionIndex] = ind2sub(size(activePairs), pairIndex);
    plane        = planes(segmentIndex, regionIndex);
    pairControls = squeeze(controls(segmentIndex, :, :));

    % Reuse the restricted controls whenever this span repeats the same time scope.
    if ~isequal(plane.TimeFraction, [0, 1])
        if isequal(lastTimeFraction(segmentIndex, :), plane.TimeFraction)
            pairControls = lastRestrictedControl{segmentIndex};
        else
            pairControls = bmtpEngine.motion.restrictBezier(pairControls, plane.TimeFraction);
            lastTimeFraction(segmentIndex, :)      = plane.TimeFraction;
            lastRestrictedControl{segmentIndex}    = pairControls;
        end
    end

    % Bound the pair with the exact degree-D by degree-one Bernstein product.
    product = alpha .* [pairControls * plane.Normal(1, :).'; 0] + ...
        beta .* [0; pairControls * plane.Normal(2, :).'];
    offsets = alpha * plane.Offset_units(1) + beta * plane.Offset_units(2);
    slack = 0;
    slackColumn = slackColumnByPair(segmentIndex, regionIndex);
    if slackColumn > 0
        slack = decisionVector(slackColumn);
    end
    pairResidual         = max(product + offsets - slack + trajectoryReserve_units);
    maximumResidual      = max(maximumResidual, pairResidual);
    pairIsWorstViolation = pairResidual > tolerance && ...
        pairResidual > greatestViolationBySegment(segmentIndex);
    if pairIsWorstViolation
        greatestViolationBySegment(segmentIndex) = pairResidual;
        greatestRegionBySegment(segmentIndex)    = regionIndex;
    end
end

%% Section 2: Select One Violated Pair Per Motion Span

for segmentIndex = reshape(find(greatestRegionBySegment > 0), 1, [])
    selectedPairs(segmentIndex, greatestRegionBySegment(segmentIndex)) = true;
end
end
