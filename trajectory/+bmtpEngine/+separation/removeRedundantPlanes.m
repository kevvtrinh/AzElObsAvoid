function separatingPlanes = removeRedundantPlanes( ...
    separatingPlanes, limits, roundoffReserve_units, useSingleConstraintProof)
%% Section 0: Header & Readme
% SYNTAX
%   separatingPlanes = bmtpEngine.separation.removeRedundantPlanes( ...
%       separatingPlanes, limits, roundoffReserve_units)
%   separatingPlanes = bmtpEngine.separation.removeRedundantPlanes( ...
%       separatingPlanes, limits, roundoffReserve_units, useSingleConstraintProof)
%**************************************************************************
% PURPOSE
%   - Disable separating-line constraints that other retained constraints
%     already enforce when solver slack is zero.
%**************************************************************************
% INPUTS
%   - separatingPlanes (S-by-R struct array)
%       Separating-line constraints for each trajectory segment and region.
%   - limits (scalar struct)
%       Validated workspace limits.
%   - roundoffReserve_units (nonnegative numeric scalar)
%       Trajectory-side numerical reserve.
%   - useSingleConstraintProof (logical scalar, optional; default false)
%       True: prove one retained constraint makes another unnecessary, even
%       when its normal changes. False: combine two constant-normal constraints.
%**************************************************************************
% OUTPUTS
%   - separatingPlanes (S-by-R struct array)
%       Unnecessary constraints have Active = false. Final motion must still
%       be checked against every original obstacle region.
%**************************************************************************
% UNITS
%   - Position, offsets, and reserve are coordinate units; normals and
%     combination weights are dimensionless.
%**************************************************************************

%% Section 1: Include Workspace Limits As Constraints

if nargin < 4
    useSingleConstraintProof = false;
end

% Each line requires normal x position + offset + reserve <= 0.
% The workspace limits also constrain position, so they can help prove that
% a line adds no restriction. Include x/y upper and lower bounds below.
workspaceIntervals_units         = [limits.xInterval_units; limits.yInterval_units];
maximumCoordinateMagnitude_units = max(abs(workspaceIntervals_units), [], 2).';
workspaceNormals                 = [1, 0; -1, 0; 0, 1; 0, -1];
workspaceOffsets_units           = repmat([-workspaceIntervals_units(1, 2); workspaceIntervals_units(1, 1); ...
    -workspaceIntervals_units(2, 2); workspaceIntervals_units(2, 1)], 1, 2);

%% Section 2: Disable A Line Only When Retained Constraints Enforce It

for segmentIndex = 1:size(separatingPlanes, 1)
    activePlaneIndices = find([separatingPlanes(segmentIndex, :).Active]);

    % Skip this optional reduction for large groups to limit its cost.
    % Every line remains active when the group is skipped.
    if numel(activePlaneIndices) < 2 || numel(activePlaneIndices) > 64
        continue
    end

    % First mode: one nonnegative weight must match both endpoint normals.
    % Store [start normal, end normal] together to require that same weight.
    if useSingleConstraintProof
        activePlaneCount        = numel(activePlaneIndices);
        constraintNormals       = zeros(activePlaneCount + 8, 4);
        constraintOffsets_units = zeros(activePlaneCount + 8, 2);
        for planeIndex = 1:activePlaneCount
            constraintNormals(planeIndex, :) = reshape( ...
                separatingPlanes(segmentIndex, activePlaneIndices(planeIndex)).Normal.', 1, []);
            constraintOffsets_units(planeIndex, :) = ...
                separatingPlanes(segmentIndex, activePlaneIndices(planeIndex)).Offset_units + roundoffReserve_units;
        end

        % A moving-line constraint combines two curve control points. Apply
        % workspace bounds to each point separately: four bounds per endpoint.
        for boundIndex = 1:4
            constraintNormals(activePlaneCount + boundIndex, 1:2) = workspaceNormals(boundIndex, :);
            constraintOffsets_units(activePlaneCount + boundIndex, 1) = workspaceOffsets_units(boundIndex, 1);
            constraintNormals(activePlaneCount + 4 + boundIndex, 3:4) = workspaceNormals(boundIndex, :);
            constraintOffsets_units(activePlaneCount + 4 + boundIndex, 2) = workspaceOffsets_units(boundIndex, 1);
        end

        constraintIsRetained = true(activePlaneCount + 8, 1);
        geometryScale_units  = max([1; abs(constraintOffsets_units(:)); maximumCoordinateMagnitude_units(:)]);
        % Use only constraints still retained. Never justify a removal using
        % a line already disabled earlier in this pass.
        for planeIndex = activePlaneCount:-1:1
            sourceConstraintIndices = find(constraintIsRetained);
            sourceConstraintIndices(sourceConstraintIndices == planeIndex) = [];
            sourceNormalSquaredLength = sum(constraintNormals(sourceConstraintIndices, :) .^ 2, 2);
            combinationWeights = (constraintNormals(sourceConstraintIndices, :) * ...
                constraintNormals(planeIndex, :).') ./ sourceNormalSquaredLength;
            combinationIsValid = isfinite(combinationWeights) & combinationWeights >= 0 & ...
                sourceNormalSquaredLength > realmin;
            sourceConstraintIndices = sourceConstraintIndices(combinationIsValid);
            combinationWeights      = combinationWeights(combinationIsValid);

            % A small normal mismatch can change the line value anywhere
            % in the workspace. Bound that change and include rounding error
            % before deciding whether the source constraint is strong enough.
            normalDifference = constraintNormals(planeIndex, :) - ...
                combinationWeights .* constraintNormals(sourceConstraintIndices, :);
            normalDifferenceAllowance_units = [abs(normalDifference(:, 1:2)) * maximumCoordinateMagnitude_units.', ...
                abs(normalDifference(:, 3:4)) * maximumCoordinateMagnitude_units.'];
            normalDifferenceAllowance_units = normalDifferenceAllowance_units + ...
                128 * eps(geometryScale_units) * (1 + combinationWeights);
            combinedOffset_units = combinationWeights .* constraintOffsets_units(sourceConstraintIndices, :);

            % Both endpoint offsets must pass for at least one source.
            if any(all(constraintOffsets_units(planeIndex, :) + ...
                normalDifferenceAllowance_units <= combinedOffset_units, 2))
                constraintIsRetained(planeIndex) = false;
                separatingPlanes(segmentIndex, activePlaneIndices(planeIndex)).Active = false;
            end
        end
        continue
    end

    % Second mode: combine two constraints with nonnegative weights.
    % Only lines whose normal stays constant use this calculation.
    if numel(activePlaneIndices) < 3
        continue
    end
    normalIsConstant = arrayfun(@(plane) isequal(plane.Normal(1, :), plane.Normal(2, :)), ...
        separatingPlanes(segmentIndex, activePlaneIndices));
    activePlaneIndices      = activePlaneIndices(normalIsConstant);
    activePlaneCount        = numel(activePlaneIndices);
    constraintNormals       = zeros(activePlaneCount, 2);
    constraintOffsets_units = zeros(activePlaneCount, 2);
    for planeIndex = 1:activePlaneCount
        constraintNormals(planeIndex, :) = separatingPlanes(segmentIndex, activePlaneIndices(planeIndex)).Normal(1, :);
        constraintOffsets_units(planeIndex, :) = ...
            separatingPlanes(segmentIndex, activePlaneIndices(planeIndex)).Offset_units + roundoffReserve_units;
    end
    constraintNormals       = [constraintNormals; workspaceNormals];
    constraintOffsets_units = [constraintOffsets_units; workspaceOffsets_units];
    constraintIsRetained     = true(activePlaneCount + 4, 1);
    geometryScale_units      = max([1; abs(constraintOffsets_units(:)); maximumCoordinateMagnitude_units(:)]);

    for planeIndex = activePlaneCount:-1:1
        otherPlaneIndices = find(constraintIsRetained);
        otherPlaneIndices(otherPlaneIndices == planeIndex) = [];
        [firstSourceIndices, secondSourceIndices] = find(triu(true(numel(otherPlaneIndices)), 1));
        firstSourceIndices  = otherPlaneIndices(firstSourceIndices);
        secondSourceIndices = otherPlaneIndices(secondSourceIndices);
        firstNormal         = constraintNormals(firstSourceIndices, :);
        secondNormal        = constraintNormals(secondSourceIndices, :);
        targetNormal        = constraintNormals(planeIndex, :);

        % Solve target normal = weight 1 x normal 1 + weight 2 x normal 2.
        % Nearly parallel source normals cannot provide a reliable solution.
        determinant = firstNormal(:, 1) .* secondNormal(:, 2) - ...
            firstNormal(:, 2) .* secondNormal(:, 1);
        combinationWeights = [(targetNormal(1) * secondNormal(:, 2) - ...
            targetNormal(2) * secondNormal(:, 1)) ./ determinant, ...
            (firstNormal(:, 1) * targetNormal(2) - ...
            firstNormal(:, 2) * targetNormal(1)) ./ determinant];
        combinationIsValid = abs(determinant) > 64 * eps & ...
            all(isfinite(combinationWeights) & combinationWeights >= 0, 2);
        combinationWeights  = combinationWeights(combinationIsValid, :);
        firstSourceIndices  = firstSourceIndices(combinationIsValid);
        secondSourceIndices = secondSourceIndices(combinationIsValid);

        % Account for normal mismatch and rounding error at both endpoints,
        % just as in the single-constraint calculation above.
        normalDifference = targetNormal - ...
            combinationWeights(:, 1) .* constraintNormals(firstSourceIndices, :) - ...
            combinationWeights(:, 2) .* constraintNormals(secondSourceIndices, :);
        normalDifferenceAllowance_units = abs(normalDifference) * maximumCoordinateMagnitude_units.' + ...
            128 * eps(geometryScale_units) * (1 + sum(combinationWeights, 2));
        combinedOffset_units = combinationWeights(:, 1) .* constraintOffsets_units(firstSourceIndices, :) + ...
            combinationWeights(:, 2) .* constraintOffsets_units(secondSourceIndices, :);
        if any(all(constraintOffsets_units(planeIndex, :) + ...
                normalDifferenceAllowance_units <= combinedOffset_units, 2))
            constraintIsRetained(planeIndex) = false;
            separatingPlanes(segmentIndex, activePlaneIndices(planeIndex)).Active = false;
        end
    end
end
end
