function [lineConstraintRows, lineConstraintBounds] = createSelectedPlaneRows( ...
    separatingPlanes, selectedPairs, degree, decisionVariableCount, ...
    slackColumnByPair, trajectoryRoundoffReserve_units)
%% Section 0: Header & Readme
% SYNTAX
%   [lineConstraintRows, lineConstraintBounds] = ...
%       bmtpEngine.separation.createSelectedPlaneRows(separatingPlanes, selectedPairs, ...
%       degree, decisionVariableCount, slackColumnByPair, trajectoryRoundoffReserve_units)
%**************************************************************************
% PURPOSE
%   - Build solver inequalities for selected curve-segment/obstacle pairs.
%     Each line constrains the complete curve portion assigned to that pair.
%**************************************************************************
% INPUTS
%   - separatingPlanes (struct array)
%       Plane records indexed by segment and region.
%   - selectedPairs (logical matrix)
%       Segment-region pairs that contribute constraint rows.
%   - degree (integer scalar)
%       Polynomial degree.
%   - decisionVariableCount (integer scalar)
%       Number of unknown values in the solver vector.
%   - slackColumnByPair (numeric matrix)
%       Column of the variable allowed to relax each pair's line bound.
%       Zero means that pair has no relaxation variable.
%   - trajectoryRoundoffReserve_units (numeric scalar)
%       Trajectory-side reserve subtracted from every bound.
%**************************************************************************
% OUTPUTS
%   - lineConstraintRows (sparse matrix)
%       Rows ordered by segment, then obstacle region. Empty when none selected.
%   - lineConstraintBounds (numeric column)
%       Upper bounds: lineConstraintRows x solverValues <= lineConstraintBounds.
%**************************************************************************
% UNITS
%   - Bounds and trajectory roundoff reserve are coordinate units.
%**************************************************************************

%% Section 1: Allocate Storage For The Selected Constraints

% Each pair contributes degree + 2 rows. Store each nonzero entry by its
% row, column, and value, then build one sparse matrix at the end. A row
% can use every x/y control coordinate in its segment plus one slack value.
rowCount             = nnz(selectedPairs) * (degree + 2);
maximumEntryCount    = rowCount * (2 * (degree + 1) + 1);
lineConstraintBounds = zeros(rowCount, 1);
rowIndices           = zeros(maximumEntryCount, 1);
columnIndices        = zeros(maximumEntryCount, 1);
entryValues          = zeros(maximumEntryCount, 1);
filledRowCount       = 0;
filledEntryCount     = 0;

%% Section 2: Add Selected Lines In Segment Order

for segmentIndex = 1:size(selectedPairs, 1)
    for regionIndex = reshape(find(selectedPairs(segmentIndex, :)), 1, [])
        [pairConstraintRows, lineOffsetCoefficients_units] = bmtpEngine.separation.createPlaneRows( ...
            separatingPlanes(segmentIndex, regionIndex), degree, decisionVariableCount, segmentIndex);
        pairRowIndices = filledRowCount + (1:degree + 2);

        [localRowIndices, pairColumnIndices, pairEntryValues] = find(pairConstraintRows);
        newEntryIndices = filledEntryCount + (1:numel(pairEntryValues));
        rowIndices(newEntryIndices)    = filledRowCount + localRowIndices;
        columnIndices(newEntryIndices) = pairColumnIndices;
        entryValues(newEntryIndices)   = pairEntryValues;
        filledEntryCount = filledEntryCount + numel(pairEntryValues);

        % line-side value - slack <= -reserve. A positive slack value
        % relaxes the line bound; a zero column index adds no slack term.
        slackColumnIndex = slackColumnByPair(segmentIndex, regionIndex);
        if slackColumnIndex > 0
            newEntryIndices = filledEntryCount + (1:degree + 2);
            rowIndices(newEntryIndices)    = pairRowIndices;
            columnIndices(newEntryIndices) = slackColumnIndex;
            entryValues(newEntryIndices)   = -1;
            filledEntryCount = filledEntryCount + degree + 2;
        end
        lineConstraintBounds(pairRowIndices) = -trajectoryRoundoffReserve_units - lineOffsetCoefficients_units;
        filledRowCount = pairRowIndices(end);
    end
end

%% Section 3: Build The Sparse Constraint Matrix

% Ignore unused buffer entries; they were only reserved capacity.
lineConstraintRows = sparse(rowIndices(1:filledEntryCount), columnIndices(1:filledEntryCount), ...
    entryValues(1:filledEntryCount), rowCount, decisionVariableCount);
end
