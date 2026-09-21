function index = createMovingCellIndex(cells, nodePosition_units)
%% Section 0: Header & Readme
% SYNTAX
%   index = obstacleAvoidance.search.createMovingCellIndex(cells, nodePosition_units)
%**************************************************************************
% PURPOSE
%   - Build, once per search, everything the exact clearance predicate
%     needs to test a node-to-node segment against the moving convex cells:
%     the cells themselves, their outer boxes and orientation over their
%     clocks, and the segment/box parameter clocks of every directed node
%     pair (precomputed when the product is small, computed on demand
%     otherwise).
%**************************************************************************
% INPUTS
%   - cells (scalar struct)
%       Affine time cells from createTimeCells.
%   - nodePosition_units (N-by-2 numeric)
%       Node positions of the search.
%**************************************************************************
% OUTPUTS
%   - index (scalar struct)
%       The pair cache of createPairCellCache (NodeCount, IsPrecomputed,
%       NodePosition_units, CellLower_units, CellUpper_units,
%       ActiveIntervals_s, and the pair entries) with Cells and
%       CellIsCounterclockwise added.
%**************************************************************************
% UNITS
%   - Positions are coordinate units and clocks are seconds.
%**************************************************************************

%% Section 1: Box The Cells, Then Index Every Node Pair Against Them

% The boxes contain every point accepted by the exact residual predicate,
% including its tolerance near sharp corners. A cache miss therefore removes
% work only; uncertain and degenerating cells remain on the exact path.
nodeScale_units = max([1; abs(nodePosition_units(:))]);
[cellLower_units, cellUpper_units, cellIsCounterclockwise] = ...
    obstacleAvoidance.search.createCellBoxes(cells, nodeScale_units);
index = obstacleAvoidance.search.createPairCellCache( ...
    nodePosition_units, cellLower_units, cellUpper_units, cells.ActiveTimeInterval_s);
index.Cells                  = cells;
index.CellIsCounterclockwise = cellIsCounterclockwise;
end
