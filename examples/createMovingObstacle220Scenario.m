function [obstacles, initialState, goalState, limits] = createMovingObstacle220Scenario()
%% Section 0: Header & Readme
% SYNTAX: [obstacles, initialState, goalState, limits] = createMovingObstacle220Scenario()
% PURPOSE: Share the deterministic moving history between example and benchmark.
% INPUTS: None; the source has 220 vertices at each of 4,829 timestamps.
% OUTPUTS: Original obstacle history and public planner state/limit inputs.
% UNITS: Planar coordinate units, seconds, and physical derivatives.

%% Section 1: Generate The Complete Moving History
time_s = (2236.5:0.25:3443.5).';
angle_rad = 2*pi*(0:219).'/220;
outline_units = (20+6*cos(3*angle_rad)).*[cos(angle_rad),sin(angle_rad)];
elapsed_s = time_s-2840;
center_units = [40+0.005*elapsed_s,40+0.5*sin(elapsed_s/20)];
x_units = arrayfun(@(k)outline_units(:,1)+center_units(k,1), ...
    (1:numel(time_s)).','UniformOutput',false);
y_units = arrayfun(@(k)outline_units(:,2)+center_units(k,2), ...
    (1:numel(time_s)).','UniformOutput',false);
obstacles = obstacleAvoidance.obstacles.createObstacle( ...
    'synthetic 220-vertex history',time_s,x_units,y_units);
assert(all(cellfun(@(x)nnz(isfinite(x))==220,obstacles.x_units)));

%% Section 2: Define The Fixed-Arrival Request
% The 230-second motion window includes 921 source snapshots. These times
% describe the simulated motion, not the wall time spent planning it.
initialState = struct('time_s',2770,'position_units',[80,0]);
goalState = struct('time_s',3000,'position_units',[0,80]);
limits = struct('xInterval_units',[-180,180],'yInterval_units',[-90,90], ...
    'maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[0.75,0.75], ...
    'maxJerk_units_s3',[2,2]);
end
