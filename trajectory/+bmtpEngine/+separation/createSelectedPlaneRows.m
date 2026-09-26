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
%   - Collect the separating-line inequalities chosen for the solver.
%     Each segment/region pair constrains its assigned portion of the
%     motion curve, and the rows are stacked into one sparse matrix.
%**************************************************************************
% INPUTS
%   - separatingPlanes (S-by-R struct array)
%       One line record for each motion segment and obstacle region.
%   - selectedPairs (S-by-R logical matrix)
%       True for each segment/region pair to include in the solver.
%   - degree (integer scalar)
%       Degree D of each segment's position curve.
%   - decisionVariableCount (integer scalar)
%       Total solver unknowns, including curve controls and any slack.
%   - slackColumnByPair (S-by-R numeric matrix)
%       Solver column of the slack value for each pair. A positive slack
%       relaxes that pair's bound; zero means no slack column is added.
%   - trajectoryRoundoffReserve_units (numeric scalar)
%       Reserve subtracted from each line-side bound to cover numerical
%       rounding.
%**************************************************************************
% OUTPUTS
%   - lineConstraintRows (K-by-decisionVariableCount sparse matrix)
%       K = number of selected pairs x (D+2). Rows are ordered by segment,
%       then region; K is zero when no pair is selected.
%   - lineConstraintBounds (K-by-1 numeric column)
%       Upper bounds in lineConstraintRows * solverValues <= bounds.
%       Each row bounds one Bezier coefficient of dot(normal, position)
%       + offset - slack at or below -roundoff reserve.
%**************************************************************************
% UNITS
%   - Bounds, slack, and roundoff reserve are coordinate units.
%**************************************************************************

%% Section 1: Allocate Storage For The Selected Constraints

% A degree-D curve times a changing line normal gives D+2 Bezier
% coefficients, so each selected pair needs D+2 rows. For D = 1, two
% pairs contribute 2 x 3 = 6 rows. Reserve room for all x/y controls in
% a segment plus one slack value per row; store only entries actually used.
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

        % Move this pair's nonzero entries into the next block of global
        % rows. Column numbers already refer to the full solver vector.
        [localRowIndices, pairColumnIndices, pairEntryValues] = find(pairConstraintRows);
        newEntryIndices = filledEntryCount + (1:numel(pairEntryValues));
        rowIndices(newEntryIndices)    = filledRowCount + localRowIndices;
        columnIndices(newEntryIndices) = pairColumnIndices;
        entryValues(newEntryIndices)   = pairEntryValues;
        filledEntryCount = filledEntryCount + numel(pairEntryValues);

        % With slack, the line-side coefficient - slack must be <= -reserve.
        % A zero mapping means this pair has no slack variable.
        slackColumnIndex = slackColumnByPair(segmentIndex, regionIndex);
        if slackColumnIndex > 0
            newEntryIndices = filledEntryCount + (1:degree + 2);
            rowIndices(newEntryIndices)    = pairRowIndices;
            columnIndices(newEntryIndices) = slackColumnIndex;
            entryValues(newEntryIndices)   = -1;
            filledEntryCount = filledEntryCount + degree + 2;
        end
        % Move the line offset to the right side of the inequality.
        lineConstraintBounds(pairRowIndices) = -trajectoryRoundoffReserve_units - lineOffsetCoefficients_units;
        filledRowCount = pairRowIndices(end);
    end
end

%% Section 3: Build The Sparse Constraint Matrix

% Build the sparse matrix once, using only filled entries from the buffers.
lineConstraintRows = sparse(rowIndices(1:filledEntryCount), columnIndices(1:filledEntryCount), ...
    entryValues(1:filledEntryCount), rowCount, decisionVariableCount);
end
