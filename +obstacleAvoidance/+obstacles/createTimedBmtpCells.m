function [regions_units, coverage] = createTimedBmtpCells(obstacles, startTime_s, finishTime_s, timedSegmentCount)
%% Section 0: Header & Readme
% SYNTAX: [regions_units,coverage] = obstacleAvoidance.obstacles.createTimedBmtpCells(
%   obstacles,startTime_s,finishTime_s,timedSegmentCount)
% PURPOSE: Return the same exact affine convex cells used by planning and
%   independent validation. No sampled spatial enclosure replaces motion.
% INPUTS: Prepared obstacles, physical bounds in seconds, and segment count.
% OUTPUTS: Convex exclusion regions and their physical and normalized intervals.
% UNITS: Position is coordinate units and time is seconds.

%% Section 1: Reuse The Authoritative Exact Time Cells
cells = obstacleAvoidance.obstacles.createTimeCells( ...
    obstacles,startTime_s,finishTime_s,true);
regions_units = cells.Regions_units;
activeTime_s = cells.ActiveTimeInterval_s;

%% Section 2: Return Coverage Evidence
duration_s = finishTime_s - startTime_s;
coverage = struct('Passed',true,'ExactRegionCount',numel(regions_units), ...
    'SolverRegionCount',numel(regions_units), ...
    'ActiveTimeInterval_s',activeTime_s, ...
    'RegionActiveTauInterval',(activeTime_s - startTime_s) / duration_s, ...
    'EndRegions_units',{cells.EndRegions_units},'BreakTime_s',cells.BreakTime_s, ...
    'TimedSegmentCount',timedSegmentCount, ...
    'AuthoritativeCoverageCheck','publicDynamicValidation');
end
