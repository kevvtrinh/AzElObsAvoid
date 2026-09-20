function alignedUpper_units = alignCorrespondingRing(lower_units, upper_units)
%% Section 0: Header & Readme
% SYNTAX
%   alignedUpper_units = obstacleAvoidance.obstacles.alignCorrespondingRing( ...
%       lower_units, upper_units)
%**************************************************************************
% PURPOSE
%   - Select the centered circular-correlation vertex correspondence.
%**************************************************************************
% INPUTS
%   - lower_units (N-by-2 numeric array)
%       Finite reference ring with at least three vertices.
%   - upper_units (N-by-2 numeric array)
%       Finite ring with the same size as lower_units.
%**************************************************************************
% OUTPUTS
%   - alignedUpper_units (N-by-2 numeric array)
%       Upper vertices reordered only by cyclic shift and orientation.
%       Invalid input throws an error.
%**************************************************************************
% UNITS
%   - Ring coordinates use caller-consistent coordinate units.
%**************************************************************************

%% Section 1: Validate Corresponding Rings

validateattributes(lower_units, {'numeric'}, {'real', 'finite', 'ncols', 2});
validateattributes(upper_units, {'numeric'}, {'real', 'finite', 'size', size(lower_units)});
assert(size(lower_units, 1) >= 3, 'alignCorrespondingRing:InvalidRing', 'A ring needs three vertices.');
alignedUpper_units = upper_units;
if isequal(lower_units, upper_units)
    return
end

%% Section 2: Resolve Alignment At A Physical Anchor

% Center and scale before FFT correlation: translation does not affect
% the least-squares correspondence, and normalization avoids cancellation
% when a small polygon is far from the coordinate origin.
centeredLower_units = lower_units - mean(lower_units, 1);
centeredUpper_units = upper_units - mean(upper_units, 1);
scale_units         = max(abs([centeredLower_units; centeredUpper_units]), [], 'all');
if scale_units == 0
    return
end
centeredLower_units = centeredLower_units / scale_units;
centeredUpper_units = centeredUpper_units / scale_units;
lowerSpectrum       = fft(centeredLower_units);
anchorChoices       = find(lower_units(:, 1) == min(lower_units(:, 1)));
[~, anchorChoice]   = min(lower_units(anchorChoices, 2));
anchorIndex         = anchorChoices(anchorChoice);
bestSquaredCost     = Inf;

% Circular correlation evaluates every cyclic alignment in O(N log N).
% Only the two selected shifts are precomputed.
for orientationIndex = 1:2
    orientedUpper_units         = upper_units;
    orientedCenteredUpper_units = centeredUpper_units;
    if orientationIndex == 2
        orientedUpper_units = flipud(orientedUpper_units);
        orientedCenteredUpper_units = flipud(orientedCenteredUpper_units);
    end

    correlation = sum(real(ifft(lowerSpectrum .* conj(fft(orientedCenteredUpper_units)))), 2);
    % Resolve numerically tied alignments at a physical anchor vertex,
    % independent of either incoming ring's starting index.
    tieTolerance        = 64 * ceil(log2(size(lower_units, 1))) * eps(max(abs(correlation)));
    shifts              = find(correlation >= max(correlation) - tieTolerance);
    anchorVertices_units = orientedUpper_units(mod(anchorIndex - shifts, size(lower_units, 1)) + 1, :);
    choices             = find(anchorVertices_units(:, 1) == min(anchorVertices_units(:, 1)));
    [~, choice]         = min(anchorVertices_units(choices, 2));
    shiftIndex          = shifts(choices(choice));
    shiftCount          = shiftIndex - 1;
    squaredCost = sum((circshift(orientedCenteredUpper_units, shiftCount, 1) - centeredLower_units) .^ 2, 'all');
    if squaredCost < bestSquaredCost
        bestSquaredCost     = squaredCost;
        alignedUpper_units = circshift(orientedUpper_units, shiftCount, 1);
    end
end
end
