function nodePairCache = createPairCellCache( ...
    nodePosition_units, boxMinimum_units, boxMaximum_units, activeIntervals_s)
%% Section 0: Header & Readme
% SYNTAX
%   nodePairCache = obstacleAvoidance.search.createPairCellCache( ...
%       nodePosition_units, boxMinimum_units, boxMaximum_units, activeIntervals_s)
%**************************************************************************
% PURPOSE
%   - Save which moving-region boxes a straight segment between two nodes
%     can meet, and where it enters and leaves each box. Large searches
%     keep the inputs and calculate the same results when needed.
%**************************************************************************
% INPUTS
%   - nodePosition_units (N-by-2 numeric)
%       Node positions.
%   - boxMinimum_units, boxMaximum_units (C-by-2 numeric)
%       Lower and upper corners of region boxes from createCellBoxes.
%   - activeIntervals_s (C-by-2 numeric)
%       [start end] times when each region is active.
%**************************************************************************
% OUTPUTS
%   - nodePairCache (scalar struct)
%       IsPrecomputed is true when the node-pair results are already stored.
%       QEnter and QExit hold fractions along a segment: 0 is its start and
%       1 is its end. Otherwise, the saved inputs support later calculation.
%**************************************************************************
% UNITS
%   - Positions are coordinate units; active time intervals are seconds.
%     Segment fractions are unitless.
%**************************************************************************

%% Section 1: Choose Whether To Store Every Node-Pair Result

% N nodes give N x N directed pairs, including a node paired with itself.
% Limit the stored table size; this changes memory use, not which connections
% the search can check or how their collision results are calculated.
nodeCount   = size(nodePosition_units, 1);
regionCount = size(boxMinimum_units, 1);
pairCount   = nodeCount ^ 2;

maximumPrecomputedPairCount     = 65536;
maximumPrecomputedPairCellTests = 4e6;
storeEveryPair = pairCount <= maximumPrecomputedPairCount && ...
    pairCount * regionCount <= maximumPrecomputedPairCellTests;
nodePairCache = struct( ...
    'NodeCount',          nodeCount, ...
    'IsPrecomputed',      storeEveryPair, ...
    'NodePosition_units', nodePosition_units, ...
    'CellLower_units',    boxMinimum_units, ...
    'CellUpper_units',    boxMaximum_units, ...
    'ActiveIntervals_s',  activeIntervals_s, ...
    'CellIndices',        {cell(0, 1)}, ...
    'QEnter',             {cell(0, 1)}, ...
    'QExit',              {cell(0, 1)}, ...
    'ActiveStart_s',      {cell(0, 1)}, ...
    'ActiveEnd_s',        {cell(0, 1)});
if ~storeEveryPair
    return
end

%% Section 2: Calculate And Save Each Directed Pair

% Keep start-to-end and end-to-start entries separate because the fractions
% run in opposite directions. Region geometry and active times stay the same.
regionIndicesByPair      = cell(pairCount, 1);
boxEntryFractionsByPair  = cell(pairCount, 1);
boxExitFractionsByPair   = cell(pairCount, 1);
activeStartTimesByPair_s = cell(pairCount, 1);
activeEndTimesByPair_s   = cell(pairCount, 1);
for endNodeIndex = 1:nodeCount
    for startNodeIndex = 1:nodeCount
        pairIndex          = startNodeIndex + nodeCount * (endNodeIndex - 1);
        segmentStart_units = nodePosition_units(startNodeIndex, :);
        segmentEnd_units   = nodePosition_units(endNodeIndex, :);
        [regionIndicesByPair{pairIndex}, boxEntryFractionsByPair{pairIndex}, ...
            boxExitFractionsByPair{pairIndex}, activeStartTimesByPair_s{pairIndex}, ...
            activeEndTimesByPair_s{pairIndex}] = obstacleAvoidance.search.computePairCellEntry( ...
            segmentStart_units, segmentEnd_units, boxMinimum_units, boxMaximum_units, ...
            activeIntervals_s);
    end
end

nodePairCache.CellIndices   = regionIndicesByPair;
nodePairCache.QEnter        = boxEntryFractionsByPair;
nodePairCache.QExit         = boxExitFractionsByPair;
nodePairCache.ActiveStart_s = activeStartTimesByPair_s;
nodePairCache.ActiveEnd_s   = activeEndTimesByPair_s;
end
