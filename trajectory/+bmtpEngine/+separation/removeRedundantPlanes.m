function planes = removeRedundantPlanes(planes, limits, roundoffReserve_units, useAffineOneSourceProof)
%% Section 0: Header & Readme
% SYNTAX
%   planes = bmtpEngine.separation.removeRedundantPlanes(planes, limits, roundoffReserve_units)
%   planes = bmtpEngine.separation.removeRedundantPlanes(planes, limits, roundoffReserve_units, ...
%       useAffineOneSourceProof)
%**************************************************************************
% PURPOSE
%   - Reduce solver rows without changing the unrelaxed feasible corridor.
%**************************************************************************
% INPUTS
%   - planes (S-by-R struct array)
%       Active solver separating planes.
%   - limits (scalar struct)
%       Validated workspace limits.
%   - roundoffReserve_units (nonnegative numeric scalar)
%       Trajectory-side numerical reserve.
%   - useAffineOneSourceProof (logical scalar, optional; default false)
%       Whether one-generator lifted proofs may remove affine-normal planes.
%**************************************************************************
% OUTPUTS
%   - planes (S-by-R struct array)
%       Plane array with provably implied halfspaces marked inactive. The
%       caller must still validate motion against every original region.
%**************************************************************************
% UNITS
%   - Position, offsets, and reserve are coordinate units; normals and
%     combination weights are dimensionless.
%**************************************************************************

%% Section 1: Include Workspace Bounds In The Implication Proof

if nargin < 4
    useAffineOneSourceProof = false;
end

% Remove only halfspaces proved implied by other retained halfspaces.
% Fixed-clock refinement uses the historical two-generator proof for constant
% normals. Collision discovery uses one scale in the lifted endpoint space.
domain_units           = [limits.xInterval_units; limits.yInterval_units];
coordinateBound_units  = max(abs(domain_units), [], 2).';
workspaceNormals       = [1, 0; -1, 0; 0, 1; 0, -1];
workspaceOffsets_units = repmat([-domain_units(1, 2); domain_units(1, 1); ...
    -domain_units(2, 2); domain_units(2, 1)], 1, 2);

%% Section 2: Remove Only Sequentially Redundant Affine Halfspaces

for spanIndex = 1:size(planes, 1)
    activePlaneIndices = find([planes(spanIndex, :).Active]);

    % Bound this preprocessing for large local sets; they retain every row.
    if numel(activePlaneIndices) < 2 || numel(activePlaneIndices) > 64
        continue
    end

    if useAffineOneSourceProof
        activePlaneCount = numel(activePlaneIndices);
        normals          = zeros(activePlaneCount + 8, 4);
        offsets_units    = zeros(activePlaneCount + 8, 2);
        for planeIndex = 1:activePlaneCount
            normals(planeIndex, :) = reshape( ...
                planes(spanIndex, activePlaneIndices(planeIndex)).Normal.', 1, []);
            offsets_units(planeIndex, :) = ...
                planes(spanIndex, activePlaneIndices(planeIndex)).Offset_units + roundoffReserve_units;
        end

        % Workspace bounds apply independently to the two controls in each
        % Bernstein product row, so lift one copy into each plane endpoint.
        for boundIndex = 1:4
            normals(activePlaneCount + boundIndex, 1:2) = workspaceNormals(boundIndex, :);
            offsets_units(activePlaneCount + boundIndex, 1) = workspaceOffsets_units(boundIndex, 1);
            normals(activePlaneCount + 4 + boundIndex, 3:4) = workspaceNormals(boundIndex, :);
            offsets_units(activePlaneCount + 4 + boundIndex, 2) = workspaceOffsets_units(boundIndex, 1);
        end

        kept        = true(activePlaneCount + 8, 1);
        scale_units = max([1; abs(offsets_units(:)); coordinateBound_units(:)]);
        for planeIndex = activePlaneCount:-1:1
            sourceIndices = find(kept);
            sourceIndices(sourceIndices == planeIndex) = [];
            denominator   = sum(normals(sourceIndices, :) .^ 2, 2);
            lambda        = (normals(sourceIndices, :) * normals(planeIndex, :).') ./ denominator;
            valid         = isfinite(lambda) & lambda >= 0 & denominator > realmin;
            sourceIndices = sourceIndices(valid);
            lambda        = lambda(valid);
            residual      = normals(planeIndex, :) - lambda .* normals(sourceIndices, :);
            guard_units   = [abs(residual(:, 1:2)) * coordinateBound_units.', ...
                abs(residual(:, 3:4)) * coordinateBound_units.'];
            guard_units   = guard_units + 128 * eps(scale_units) * (1 + lambda);
            implied_units = lambda .* offsets_units(sourceIndices, :);
            if any(all(offsets_units(planeIndex, :) + guard_units <= implied_units, 2))
                kept(planeIndex) = false;
                planes(spanIndex, activePlaneIndices(planeIndex)).Active = false;
            end
        end
        continue
    end

    if numel(activePlaneIndices) < 3
        continue
    end
    normalIsConstant = arrayfun(@(plane) isequal(plane.Normal(1, :), plane.Normal(2, :)), ...
        planes(spanIndex, activePlaneIndices));
    activePlaneIndices = activePlaneIndices(normalIsConstant);
    activePlaneCount   = numel(activePlaneIndices);
    normals            = zeros(activePlaneCount, 2);
    offsets_units      = zeros(activePlaneCount, 2);
    for planeIndex = 1:activePlaneCount
        normals(planeIndex, :) = planes(spanIndex, activePlaneIndices(planeIndex)).Normal(1, :);
        offsets_units(planeIndex, :) = ...
            planes(spanIndex, activePlaneIndices(planeIndex)).Offset_units + roundoffReserve_units;
    end
    normals       = [normals; workspaceNormals];
    offsets_units = [offsets_units; workspaceOffsets_units];
    kept          = true(activePlaneCount + 4, 1);
    scale_units   = max([1; abs(offsets_units(:)); coordinateBound_units(:)]);

    for planeIndex = activePlaneCount:-1:1
        otherPlaneIndices = find(kept);
        otherPlaneIndices(otherPlaneIndices == planeIndex) = [];
        [firstSourceIndices, secondSourceIndices] = find(triu(true(numel(otherPlaneIndices)), 1));
        firstSourceIndices  = otherPlaneIndices(firstSourceIndices);
        secondSourceIndices = otherPlaneIndices(secondSourceIndices);
        firstNormal         = normals(firstSourceIndices, :);
        secondNormal        = normals(secondSourceIndices, :);
        targetNormal        = normals(planeIndex, :);
        determinant = firstNormal(:, 1) .* secondNormal(:, 2) - ...
            firstNormal(:, 2) .* secondNormal(:, 1);
        lambda = [(targetNormal(1) * secondNormal(:, 2) - ...
            targetNormal(2) * secondNormal(:, 1)) ./ determinant, ...
            (firstNormal(:, 1) * targetNormal(2) - ...
            firstNormal(:, 2) * targetNormal(1)) ./ determinant];
        valid = abs(determinant) > 64 * eps & all(isfinite(lambda) & lambda >= 0, 2);
        lambda               = lambda(valid, :);
        firstSourceIndices  = firstSourceIndices(valid);
        secondSourceIndices = secondSourceIndices(valid);
        defect = targetNormal - lambda(:, 1) .* normals(firstSourceIndices, :) - ...
            lambda(:, 2) .* normals(secondSourceIndices, :);
        guard_units = abs(defect) * coordinateBound_units.' + ...
            128 * eps(scale_units) * (1 + sum(lambda, 2));
        implied_units = lambda(:, 1) .* offsets_units(firstSourceIndices, :) + ...
            lambda(:, 2) .* offsets_units(secondSourceIndices, :);
        if any(all(offsets_units(planeIndex, :) + guard_units <= implied_units, 2))
            kept(planeIndex) = false;
            planes(spanIndex, activePlaneIndices(planeIndex)).Active = false;
        end
    end
end
end
