function [mergedTau, mergedBuffer_deg] = ...
        mergeAzElHs3CollisionRepairs( ...
        existingTau, existingBuffer_deg, newTau, newBuffer_deg)
%% Section 0: Header & Readme
% SYNTAX
%   [mergedTau, mergedBuffer_deg] = ...
%       azElInternal.mergeAzElHs3CollisionRepairs( ...
%       existingTau, existingBuffer_deg, newTau, newBuffer_deg)
%**************************************************************************
% PURPOSE
%   - Merge collision-repair times and retain the largest duplicate buffer.
%**************************************************************************
% INPUTS
%   - existingTau (N-by-1 numeric vector)
%       Existing normalized repair times.
%   - existingBuffer_deg (N-by-1 numeric vector)
%       Existing trajectory-tube buffers.
%   - newTau (M-by-1 numeric vector)
%       New normalized repair times.
%   - newBuffer_deg (M-by-1 numeric vector)
%       New trajectory-tube buffers.
%**************************************************************************
% OUTPUTS
%   - mergedTau (K-by-1 numeric vector)
%       Sorted unique repair times.
%   - mergedBuffer_deg (K-by-1 numeric vector)
%       Largest buffer for each retained repair time.
%**************************************************************************
% UNITS
%   - Tau is dimensionless. Buffers are degrees.
%**************************************************************************

%% Section 1: Merge Duplicate Repair Times

allTau = [existingTau(:); newTau(:)];
allBuffer_deg = [existingBuffer_deg(:); newBuffer_deg(:)];
mergedTau = zeros(0, 1);
mergedBuffer_deg = zeros(0, 1);
for repairIndex = 1:numel(allTau)
    duplicateIndex = find(abs(mergedTau - allTau(repairIndex)) <= ...
        64 * eps(max(1, abs(allTau(repairIndex)))), 1);
    if isempty(duplicateIndex)
        mergedTau(end + 1, 1) = allTau(repairIndex); %#ok<AGROW>
        mergedBuffer_deg(end + 1, 1) = ...
            allBuffer_deg(repairIndex); %#ok<AGROW>
    else
        mergedBuffer_deg(duplicateIndex) = max( ...
            mergedBuffer_deg(duplicateIndex), ...
            allBuffer_deg(repairIndex));
    end
end
[mergedTau, order] = sort(mergedTau);
mergedBuffer_deg = mergedBuffer_deg(order);
end
