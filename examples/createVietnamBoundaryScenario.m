function [obstacles, initialState, goalState, limits] = createVietnamBoundaryScenario()
%% Section 0: Header & Readme
% SYNTAX: [obstacles, initialState, goalState, limits] = createVietnamBoundaryScenario()
% PURPOSE: Interpolate the three supplied Vietnam azimuth/elevation boundaries.
% INPUTS: None; examples/data/vietnamBoundaryPoints.csv retains source decimals.
% OUTPUTS: Canonical obstacle with 921 slices of 280 vertices and planner inputs.
% UNITS: Degrees, seconds, and angular derivatives in degrees/s^order.

%% Section 1: Read And Normalize Only Explicit Closing Copies
source = readtable(fullfile(fileparts(mfilename('fullpath')), ...
    'data','vietnamBoundaryPoints.csv'));
anchorTime_s = [2770;2910;3000];
anchorVertices_deg = zeros(280,2,3);
assert(isequal(source.Properties.VariableNames, ...
    {'time_s','vertex_index','az_deg','el_deg'}));
assert(height(source)==842 && isequal(unique(source.time_s),anchorTime_s));
for anchorIndex = 1:3
    rows = source(source.time_s==anchorTime_s(anchorIndex),:);
    assert(isequal(rows.vertex_index,(1:height(rows)).'));
    vertices_deg = [rows.az_deg,rows.el_deg];
    if isequal(vertices_deg(1,:),vertices_deg(end,:))
        vertices_deg(end,:) = [];
    end
    assert(isequal(size(vertices_deg),[280,2]) && all(isfinite(vertices_deg),'all'));
    anchorVertices_deg(:,:,anchorIndex) = vertices_deg;
end

%% Section 2: Declare Piecewise Linear Interpolation By Supplied Vertex Index
% These interpolated samples are a reproducible model of the supplied data,
% not recovered measurements of the unsupplied physical motion. Keep the
% source ordering; do not resample, simplify, or fit a rigid translation.
time_s = (2770:0.25:3000).';
interpolated_deg = interp1(anchorTime_s, ...
    reshape(permute(anchorVertices_deg,[3,1,2]),3,[]),time_s,'linear');
x_units = mat2cell(interpolated_deg(:,1:280).',280,ones(921,1)).';
y_units = mat2cell(interpolated_deg(:,281:end).',280,ones(921,1)).';
obstacles = obstacleAvoidance.obstacles.createObstacle( ...
    'Vietnam supplied boundary',time_s,x_units,y_units);
assert(all(cellfun(@numel,obstacles.x_units)==280));
assert(all(cellfun(@numel,obstacles.y_units)==280));

%% Section 3: Retain The Supplied Physical Request
initialState = struct('time_s',2770,'position_units',[80,0]);
goalState = struct('time_s',3000,'position_units',[0,80]);
limits = struct('xInterval_units',[-180,180],'yInterval_units',[-90,90], ...
    'maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[0.75,0.75], ...
    'maxJerk_units_s3',[2,2]);
end
