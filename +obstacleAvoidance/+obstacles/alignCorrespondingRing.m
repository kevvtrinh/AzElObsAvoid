function alignedNextVertices_units = alignCorrespondingRing(referenceVertices_units, nextVertices_units)
%% Section 0: Header & Readme
% SYNTAX
%   alignedNextVertices_units = obstacleAvoidance.obstacles.alignCorrespondingRing( ...
%       referenceVertices_units, nextVertices_units)
%**************************************************************************
% PURPOSE
%   - Reorder the next boundary sample so its vertices best match the
%     reference sample after removing translation. Try every starting vertex
%     in both boundary directions; keep every coordinate unchanged.
%**************************************************************************
% INPUTS
%   - referenceVertices_units (N-by-2 numeric array)
%       Reference boundary loop with at least three finite vertices.
%   - nextVertices_units (N-by-2 numeric array)
%       Next boundary sample with the same number of finite vertices.
%**************************************************************************
% OUTPUTS
%   - alignedNextVertices_units (N-by-2 numeric array)
%       Next sample with only its starting vertex and boundary direction changed.
%       Invalid input throws an error.
%**************************************************************************
% UNITS
%   - Both boundary samples use the same coordinate units.
%**************************************************************************

%% Section 1: Check The Two Boundary Samples

validateattributes(referenceVertices_units, {'numeric'}, {'real', 'finite', 'ncols', 2});
validateattributes(nextVertices_units, {'numeric'}, {'real', 'finite', 'size', size(referenceVertices_units)});
assert(size(referenceVertices_units, 1) >= 3, 'alignCorrespondingRing:InvalidRing', 'A ring needs three vertices.');
alignedNextVertices_units = nextVertices_units;
if isequal(referenceVertices_units, nextVertices_units)
    return
end

%% Section 2: Compare Vertex Orders And Choose The Closest Match

% Subtract each sample's average vertex position to compare shape without
% translation. Divide both by the same size scale to reduce numerical error
% when a small polygon has very large coordinate values.
centeredReference_units = referenceVertices_units - mean(referenceVertices_units, 1);
centeredNext_units      = nextVertices_units - mean(nextVertices_units, 1);
shapeScale_units        = max(abs([centeredReference_units; centeredNext_units]), [], 'all');
if shapeScale_units == 0
    return
end
normalizedReferenceVertices = centeredReference_units / shapeScale_units;
normalizedNextVertices      = centeredNext_units / shapeScale_units;
referenceSpectrum           = fft(normalizedReferenceVertices);

% Use the leftmost reference vertex as an anchor. If several have that x
% coordinate, use the lowest y coordinate. This ignores the supplied start index.
referenceAnchorCandidates = find(referenceVertices_units(:, 1) == min(referenceVertices_units(:, 1)));
[~, referenceAnchorChoice] = min(referenceVertices_units(referenceAnchorCandidates, 2));
referenceAnchorIndex      = referenceAnchorCandidates(referenceAnchorChoice);
bestAlignmentErrorSquared = Inf;

% The Fourier transform compares every possible starting vertex efficiently.
% Try the supplied boundary direction first, then the reversed direction.
% A larger correlation score means a smaller sum of squared vertex distances.
for orientationIndex = 1:2
    orientedNextVertices_units = nextVertices_units;
    orientedNormalizedVertices = normalizedNextVertices;
    if orientationIndex == 2
        orientedNextVertices_units = flipud(orientedNextVertices_units);
        orientedNormalizedVertices = flipud(orientedNormalizedVertices);
    end

    alignmentScores = sum(real(ifft(referenceSpectrum .* conj(fft(orientedNormalizedVertices)))), 2);

    % Scores within roundoff may describe equally good alignments. Choose
    % the one matching the reference anchor to the smallest x coordinate in
    % the next sample, using the smallest y coordinate to break an x tie.
    scoreTieTolerance     = 64 * ceil(log2(size(referenceVertices_units, 1))) * eps(max(abs(alignmentScores)));
    candidateShiftIndices = find(alignmentScores >= max(alignmentScores) - scoreTieTolerance);

    matchedAnchorVertices_units = orientedNextVertices_units( ...
        mod(referenceAnchorIndex - candidateShiftIndices, size(referenceVertices_units, 1)) + 1, :);
    matchedAnchorChoices        = find(matchedAnchorVertices_units(:, 1) == min(matchedAnchorVertices_units(:, 1)));
    [~, matchedAnchorChoice]    = min(matchedAnchorVertices_units(matchedAnchorChoices, 2));

    selectedShiftIndex = candidateShiftIndices(matchedAnchorChoices(matchedAnchorChoice));
    shiftCount         = selectedShiftIndex - 1;

    % Compare the chosen forward and reversed alignments using the same
    % squared-distance calculation. Equal errors keep the forward ordering.
    alignmentErrorSquared = sum((circshift(orientedNormalizedVertices, shiftCount, 1) - ...
        normalizedReferenceVertices) .^ 2, 'all');
    if alignmentErrorSquared < bestAlignmentErrorSquared
        bestAlignmentErrorSquared = alignmentErrorSquared;
        alignedNextVertices_units = circshift(orientedNextVertices_units, shiftCount, 1);
    end
end
end
