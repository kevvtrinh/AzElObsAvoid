function warmStart = createWarmStart(request)
%% Section 0: Header & Readme
% SYNTAX
%   warmStart = bmtpEngine.createWarmStart(request)
%
% PURPOSE
%   - Convert the exact visibility route into degree-eight BMTP controls.
%
% INPUTS
%   - request: validated static BMTP solve request.
%
% OUTPUTS
%   - warmStart: route, control points, time, and all-pair region mask.
%
% UNITS
%   - Position is coordinate units and time is seconds.

%% Section 1: Use The Exact Visibility Route

route_units = double(request.Seed.position_units);
route_units([1 end], :) = [request.InitialState.position_units; request.GoalState.position_units];
segmentCount = size(route_units, 1) - 1;
regionActiveBySegment = true(segmentCount, numel(request.Regions_units));

%% Section 2: Create Linear Rest-To-Rest Controls

degree = request.Degree;
fraction = reshape(min(1, max(0, ((0:degree) - 2) / (degree - 4))), 1, [], 1);
start_units = reshape(route_units(1:end - 1, :), segmentCount, 1, 2);
finish_units = reshape(route_units(2:end, :), segmentCount, 1, 2);
controlPoint_units = (1 - fraction) .* start_units + fraction .* finish_units;
segmentTime_s = bmtpEngine.findRequiredSegmentTime(controlPoint_units, request.Limits);

%% Section 3: Return The Solver Initialization

warmStart = struct();
warmStart.Route_units = route_units;
warmStart.ControlPoint_units = controlPoint_units;
warmStart.SegmentTime_s = segmentTime_s;
warmStart.Duration_s = segmentCount * segmentTime_s;
warmStart.SegmentCount = segmentCount;
warmStart.RegionActiveBySegment = regionActiveBySegment;
warmStart.OriginalSeedSegmentCount = segmentCount;
warmStart.WarmRouteResampled = false;
end
