function [rows, bounds] = createSelectedPlaneRows(planes, pairMask, degree, ...
        variableCount, slackColumnByPair, trajectoryRoundoffReserve_units)
%% Section 0: Header & Readme
% SYNTAX
%   [rows, bounds] = bmtpEngine.separation.createSelectedPlaneRows(planes, pairMask, ...
%       degree, variableCount, slackColumnByPair, trajectoryRoundoffReserve_units)
%**************************************************************************
% PURPOSE
%   - Materialize exact Bernstein separating rows for selected pairs.
%**************************************************************************
% INPUTS
%   - planes (struct array)
%       Plane records indexed by segment and region.
%   - pairMask (logical matrix)
%       Segment-region pairs that contribute constraint rows.
%   - degree (integer scalar)
%       Polynomial degree.
%   - variableCount (integer scalar)
%       Decision-vector size.
%   - slackColumnByPair (numeric matrix)
%       Slack column per pair; zero leaves the pair without slack.
%   - trajectoryRoundoffReserve_units (numeric scalar)
%       Trajectory-side reserve subtracted from every bound.
%**************************************************************************
% OUTPUTS
%   - rows (sparse matrix)
%       Inequality rows in segment-major order.
%   - bounds (numeric column)
%       Matching upper bounds in the same order.
%**************************************************************************
% UNITS
%   - Bounds and trajectory roundoff reserve are coordinate units.
%**************************************************************************

%% Section 1: Allocate The Segment-Major Triplet Buffers
rowCount          = nnz(pairMask) * (degree + 2);
maximumEntryCount = rowCount * (2 * (degree + 1) + 1);
bounds        = zeros(rowCount, 1);
rowIndices    = zeros(maximumEntryCount, 1);
columnIndices = zeros(maximumEntryCount, 1);
entryValues   = zeros(maximumEntryCount, 1);
nextRow       = 0;
nextEntry     = 0;

%% Section 2: Append Every Selected Pair In Segment-Major Order
for segmentIndex = 1:size(pairMask, 1)
    for regionIndex = reshape(find(pairMask(segmentIndex, :)), 1, [])
        [pairRows, offset_units] = bmtpEngine.separation.createPlaneRows( ...
            planes(segmentIndex, regionIndex), degree, variableCount, segmentIndex);
        targetRows = nextRow + (1:degree + 2);

        [pairRowIndex, pairColumnIndex, pairValues] = find(pairRows);
        entryIndices               = nextEntry + (1:numel(pairValues));
        rowIndices(entryIndices)    = nextRow + pairRowIndex;
        columnIndices(entryIndices) = pairColumnIndex;
        entryValues(entryIndices)   = pairValues;
        nextEntry                   = nextEntry + numel(pairValues);

        slackColumnIndex = slackColumnByPair(segmentIndex, regionIndex);
        if slackColumnIndex > 0
            entryIndices               = nextEntry + (1:degree + 2);
            rowIndices(entryIndices)    = targetRows;
            columnIndices(entryIndices) = slackColumnIndex;
            entryValues(entryIndices)   = -1;
            nextEntry                   = nextEntry + degree + 2;
        end
        bounds(targetRows) = -trajectoryRoundoffReserve_units - offset_units;
        nextRow            = targetRows(end);
    end
end

%% Section 3: Assemble The Sparse Constraint Block
rows = sparse(rowIndices(1:nextEntry), columnIndices(1:nextEntry), ...
    entryValues(1:nextEntry), rowCount, variableCount);
end
