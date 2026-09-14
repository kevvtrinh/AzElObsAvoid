function alignedUpper_units = alignCorrespondingRing(lower_units, upper_units)
%% Section 0: Header & Readme
% SYNTAX: alignedUpper_units = obstacleAvoidance.obstacles.alignCorrespondingRing(lower_units,upper_units)
% PURPOSE: Select the existing centered circular-correlation correspondence.
% INPUTS: Equal-sized finite single rings, with at least three vertices.
% OUTPUTS: Upper vertices reordered by cyclic shift and orientation only.
% UNITS: Coordinate units.

%% Section 1: Validate Corresponding Rings
validateattributes(lower_units,{'numeric'},{'real','finite','ncols',2});
validateattributes(upper_units,{'numeric'},{'real','finite','size',size(lower_units)});
assert(size(lower_units,1)>=3,'alignCorrespondingRing:InvalidRing','A ring needs three vertices.');
alignedUpper_units = upper_units;
if isequal(lower_units,upper_units), return; end

%% Section 2: Resolve Alignment At A Physical Anchor
% Center and scale before FFT correlation: translation does not affect
% the least-squares correspondence, and normalization avoids cancellation
% when a small polygon is far from the coordinate origin.
centeredLower = lower_units-mean(lower_units,1);
centeredUpper = upper_units-mean(upper_units,1);
scale_units = max(abs([centeredLower;centeredUpper]),[],'all');
if scale_units==0, return; end
centeredLower = centeredLower/scale_units;
centeredUpper = centeredUpper/scale_units;
lowerSpectrum = fft(centeredLower);
anchorChoices = find(lower_units(:,1)==min(lower_units(:,1)));
[~,anchorChoice] = min(lower_units(anchorChoices,2));
anchorIndex = anchorChoices(anchorChoice);
bestSquaredCost = Inf;
% Circular correlation evaluates every cyclic alignment in O(N log N).
% Only the two selected shifts are materialized; there is no shift-loop fallback.
for orientationIndex = 1:2
    orientedUpper_units = upper_units;
    orientedUpper = centeredUpper;
    if orientationIndex == 2
        orientedUpper_units = flipud(orientedUpper_units);
        orientedUpper = flipud(orientedUpper);
    end
    correlation = sum(real(ifft(lowerSpectrum.*conj(fft(orientedUpper)))),2);
    % Resolve numerically tied alignments at a physical anchor vertex,
    % independent of either incoming ring's starting index.
    tieTolerance = 64*ceil(log2(size(lower_units,1)))*eps(max(abs(correlation)));
    shifts = find(correlation>=max(correlation)-tieTolerance);
    anchorVertices_units = orientedUpper_units(mod(anchorIndex-shifts,size(lower_units,1))+1,:);
    choices = find(anchorVertices_units(:,1)==min(anchorVertices_units(:,1)));
    [~,choice] = min(anchorVertices_units(choices,2));
    shiftIndex = shifts(choices(choice));
    shiftCount = shiftIndex-1;
    squaredCost = sum((circshift(orientedUpper,shiftCount,1)-centeredLower).^2,'all');
    if squaredCost < bestSquaredCost
        bestSquaredCost = squaredCost;
        alignedUpper_units = circshift(orientedUpper_units,shiftCount,1);
    end
end
end
