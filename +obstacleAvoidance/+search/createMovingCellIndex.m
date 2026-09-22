function movingCellLookup = createMovingCellIndex(timedRegions, nodePosition_units)
%% Section 0: Header & Readme
% SYNTAX
%   movingCellLookup = obstacleAvoidance.search.createMovingCellIndex( ...
%       timedRegions, nodePosition_units)
%**************************************************************************
% PURPOSE
%   - Prepare the geometry lookup used to check motion between route points.
%     Each convex region is a piece of protected obstacle geometry whose
%     vertices move linearly during its stored time interval.
%   - Save boxes that quickly exclude distant regions, then save which boxes
%     each node-to-node segment can meet when that table is small enough.
%**************************************************************************
% INPUTS
%   - timedRegions (scalar struct)
%       Moving protected regions from createTimeCells.
%   - nodePosition_units (N-by-2 numeric)
%       Fixed [x y] route points used by the search.
%**************************************************************************
% OUTPUTS
%   - movingCellLookup (scalar struct)
%       Node-pair lookup from createPairCellCache, plus the full moving
%       regions in Cells and their boundary directions in CellIsCounterclockwise.
%       Large node-pair tables are calculated as needed instead of stored.
%**************************************************************************
% UNITS
%   - Positions are coordinate units; active time intervals are seconds.
%**************************************************************************

%% Section 1: Prepare Boxes And Node-Pair Lookups

% Each box includes the collision tolerance, even near sharp corners.
% A segment that misses the box cannot touch that region; a segment that
% meets it still needs the full moving-region collision check.
nodeCoordinateScale_units = max([1; abs(nodePosition_units(:))]);
[boxMinimum_units, boxMaximum_units, regionIsCounterclockwise] = ...
    obstacleAvoidance.search.createCellBoxes(timedRegions, nodeCoordinateScale_units);

% Store the node-pair calculations separately from the full region geometry.
% The collision checker uses both, whether pair results are saved or calculated.
movingCellLookup = obstacleAvoidance.search.createPairCellCache( ...
    nodePosition_units, boxMinimum_units, boxMaximum_units, timedRegions.ActiveTimeInterval_s);
movingCellLookup.Cells                  = timedRegions;
movingCellLookup.CellIsCounterclockwise = regionIsCounterclockwise;
end
