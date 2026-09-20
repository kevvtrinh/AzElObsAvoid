function [selected, selectedQEnter, selectedQExit, activeStart_s, activeEnd_s] = ...
    computePairCellEntry(first_units, second_units, cellLower_units, ...
    cellUpper_units, activeIntervals_s)
%% Section 0: Header & Readme
% SYNTAX
%   [selected, selectedQEnter, selectedQExit, activeStart_s, activeEnd_s] = ...
%       obstacleAvoidance.search.computePairCellEntry(first_units, second_units, ...
%       cellLower_units, cellUpper_units, activeIntervals_s)
%**************************************************************************
% PURPOSE
%   - Conservative segment/box parameter clocks for one directed node
%     pair: the cells the segment can meet, ordered by active start, and
%     the guarded parameter interval over which it can meet each.
%**************************************************************************
% INPUTS
%   - first_units, second_units (1-by-2 numeric)
%       Segment endpoints.
%   - cellLower_units, cellUpper_units (C-by-2 numeric)
%       Cell boxes from createCellBoxes.
%   - activeIntervals_s (C-by-2 numeric)
%       Active clock of every cell.
%**************************************************************************
% OUTPUTS
%   - selected (K-by-1 numeric)
%       Indices of the cells the segment can meet.
%   - selectedQEnter, selectedQExit (K-by-1 numeric)
%       Guarded segment parameter interval per selected cell.
%   - activeStart_s, activeEnd_s (K-by-1 numeric)
%       Active clock per selected cell.
%**************************************************************************
% UNITS
%   - Positions are coordinate units and clocks are seconds.
%**************************************************************************

%% Section 1: Intersect The Segment With Every Cell Box

% Conservative segment/box parameter clocks for one directed node pair.
cellCount   = size(cellLower_units, 1);
delta_units = second_units - first_units;
qEnter      = zeros(cellCount, 1);
qExit       = ones(cellCount, 1);
canMeet     = true(cellCount, 1);
uncertain   = any(isnan(cellLower_units) | isnan(cellUpper_units), 2);
for dimensionIndex = 1:2
    if delta_units(dimensionIndex) == 0
        canMeet = canMeet & ( ...
            first_units(dimensionIndex) >= cellLower_units(:, dimensionIndex) & ...
            first_units(dimensionIndex) <= cellUpper_units(:, dimensionIndex));
    else
        firstIntersection = (cellLower_units(:, dimensionIndex) - ...
            first_units(dimensionIndex)) / delta_units(dimensionIndex);
        secondIntersection = (cellUpper_units(:, dimensionIndex) - ...
            first_units(dimensionIndex)) / delta_units(dimensionIndex);
        qEnter = max(qEnter, min(firstIntersection, secondIntersection));
        qExit  = min(qExit, max(firstIntersection, secondIntersection));
    end
end
qGuard = 256 * eps(max(1, max(abs([qEnter, qExit]), [], 2)));
qGuard(~isfinite(qGuard)) = 0;
canMeet = canMeet & qEnter <= qExit + qGuard & ...
    qExit >= -qGuard & qEnter <= 1 + qGuard;
canMeet(uncertain) = true;
qEnter(uncertain)  = 0;
qExit(uncertain)   = 1;
selected = find(canMeet);
if ~isempty(selected)
    [~, activeOrder] = sortrows( ...
        [activeIntervals_s(selected, 1), selected], [1, 2]);
    selected = selected(activeOrder);
end
selectedQEnter = max(0, qEnter(selected) - qGuard(selected));
selectedQExit  = min(1, qExit(selected) + qGuard(selected));
activeStart_s  = activeIntervals_s(selected, 1);
activeEnd_s    = activeIntervals_s(selected, 2);
end
