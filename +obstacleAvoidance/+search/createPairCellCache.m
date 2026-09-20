function cache = createPairCellCache( ...
    nodePosition_units, cellLower_units, cellUpper_units, activeIntervals_s)
%% Section 0: Header & Readme
% SYNTAX
%   cache = obstacleAvoidance.search.createPairCellCache( ...
%       nodePosition_units, cellLower_units, cellUpper_units, activeIntervals_s)
%**************************************************************************
% PURPOSE
%   - Record, for every directed node pair, which cell boxes its segment
%     can meet and over which segment parameter interval, materialized
%     eagerly for small products and calculated on demand otherwise.
%**************************************************************************
% INPUTS
%   - nodePosition_units (N-by-2 numeric)
%       Node positions.
%   - cellLower_units, cellUpper_units (C-by-2 numeric)
%       Cell boxes from createCellBoxes.
%   - activeIntervals_s (C-by-2 numeric)
%       Active clock of every cell.
%**************************************************************************
% OUTPUTS
%   - cache (scalar struct)
%       Pair entries and the inputs needed to compute one on demand;
%       IsMaterialized says which.
%**************************************************************************
% UNITS
%   - Positions are coordinate units and clocks are seconds.
%**************************************************************************

%% Section 1: Materialize The Pair Entries When The Product Is Small

% Closed segment/box parameter intervals for directed node pairs. Bound
% eager storage; larger products use the identical calculation on demand.
nodeCount   = size(nodePosition_units, 1);
cellCount   = size(cellLower_units, 1);
pairCount   = nodeCount ^ 2;
maximumMaterializedPairCount     = 65536;
maximumMaterializedPairCellTests = 4e6;
materialize = pairCount <= maximumMaterializedPairCount && ...
    pairCount * cellCount <= maximumMaterializedPairCellTests;
cache = struct( ...
    'NodeCount',         nodeCount, ...
    'IsMaterialized',    materialize, ...
    'NodePosition_units', nodePosition_units, ...
    'CellLower_units',   cellLower_units, ...
    'CellUpper_units',   cellUpper_units, ...
    'ActiveIntervals_s', activeIntervals_s, ...
    'CellIndices',       {cell(0, 1)}, ...
    'QEnter',            {cell(0, 1)}, ...
    'QExit',             {cell(0, 1)}, ...
    'ActiveStart_s',     {cell(0, 1)}, ...
    'ActiveEnd_s',       {cell(0, 1)});
if ~materialize
    return
end

cellIndices = cell(pairCount, 1);
qEnterCache = cell(pairCount, 1);
qExitCache  = cell(pairCount, 1);
activeStartCache_s = cell(pairCount, 1);
activeEndCache_s   = cell(pairCount, 1);
for secondNodeIndex = 1:nodeCount
    for firstNodeIndex = 1:nodeCount
        pairIndex = firstNodeIndex + nodeCount * (secondNodeIndex - 1);
        first_units = nodePosition_units(firstNodeIndex, :);
        second_units = nodePosition_units(secondNodeIndex, :);
        [cellIndices{pairIndex}, qEnterCache{pairIndex}, ...
            qExitCache{pairIndex}, activeStartCache_s{pairIndex}, ...
            activeEndCache_s{pairIndex}] = obstacleAvoidance.search.computePairCellEntry( ...
            first_units, second_units, cellLower_units, cellUpper_units, ...
            activeIntervals_s);
    end
end
cache.CellIndices   = cellIndices;
cache.QEnter        = qEnterCache;
cache.QExit         = qExitCache;
cache.ActiveStart_s = activeStartCache_s;
cache.ActiveEnd_s   = activeEndCache_s;
end
