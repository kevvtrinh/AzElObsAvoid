function diagnostics = scheduleChordDeparture(request,direct)
%% Section 0: Header & Readme
% SYNTAX: diagnostics = bmtpEngine.scheduleChordDeparture(request,direct)
% PURPOSE: Project source time cells onto forbidden delays of an existing exact chord.
% INPUTS: Normalized request and an exact straight-progress rest-to-rest motion.
% OUTPUTS: Departure delay and covered forbidden intervals; independently validate motion.
% UNITS: Coordinate units and seconds.

%% Section 1: Reuse The Existing Exact Progress Polynomial
polynomial = direct.Polynomial;
durations_s = polynomial.SegmentDuration_s;
duration_s = direct.TrajectoryDuration_s;
maximumWait_s = request.MotionHorizon_s-duration_s;
initial_units = request.InitialState.position_units;
direction_units = request.GoalState.position_units-initial_units;
directionNorm2_units2 = sum(direction_units.^2);
normal = [-direction_units(2),direction_units(1)]/sqrt(directionNorm2_units2);
power = polynomial.positionPower_units;
power(:,:,1) = power(:,:,1)-initial_units;
progressPower = zeros(polynomial.SegmentCount,4);
for order = 1:4
    progressPower(:,order) = power(:,:,order)*direction_units.'/directionNorm2_units2;
end
phases = struct('StartTime_s',polynomial.SegmentStartTime_s-request.InitialState.time_s,'SegmentTime_s',durations_s);
endRegions_units = {};
if isfield(request.Coverage,'EndRegions_units'), endRegions_units = request.Coverage.EndRegions_units; end
[~,~,reserve_units] = bmtpEngine.createCoordinateTolerances(initial_units,request.GoalState.position_units, ...
    request.Limits.xInterval_units,request.Limits.yInterval_units,request.Regions_units,endRegions_units);
clearance_units = (1+2^20*eps)*request.Options.CollisionClearanceTolerance_units+3*reserve_units;
forbidden_s = zeros(0,2);

%% Section 2: Project Convex Space-Time Cells Onto Path Progress And Time

for region = 1:numel(request.Regions_units)
    interval_s = request.InitialState.time_s+[0,request.MotionHorizon_s];
    if isfield(request.Coverage,'ActiveTimeInterval_s'), interval_s = request.Coverage.ActiveTimeInterval_s(region,:); end
    first_units = clearanceEnvelope(request.Regions_units{region},clearance_units);
    last_units = clearanceEnvelope(request.Coverage.EndRegions_units{region},clearance_units);
    points = [first_units,repmat(interval_s(1),size(first_units,1),1); ...
        last_units,repmat(interval_s(2),size(last_units,1),1)];
    residual = (points(:,1:2)-initial_units)*normal.';
    [positive,negative] = ndgrid(find(residual>0),find(residual<0));
    positive = positive(:); negative = negative(:);
    fraction = residual(positive)./(residual(positive)-residual(negative));
    section = [points(residual==0,:);points(positive,:)+fraction.*(points(negative,:)-points(positive,:))];
    if isempty(section), continue; end
    section = unique([(section(:,1:2)-initial_units)*direction_units.'/directionNorm2_units2,section(:,3)],'rows');
    if size(section,1)>2 && rank(section-section(1,:))==2
        hull = convhull(section(:,1),section(:,2));
        section = section(hull(1:end-1),:);
    elseif size(section,1)>2
        delta = section(end,:)-section(1,:);
        [~,order] = sort(section*delta.');
        section = section(order([1,end]),:);
    end
    low_s = Inf; high_s = -Inf; firstStartOccupancy_s = Inf;
    for edge = 1:size(section,1)
        a = section(edge,:); b = section(mod(edge,size(section,1))+1,:);
        range = [max(0,min(a(1),b(1))),min(1,max(a(1),b(1)))];
        if range(1)>range(2), continue; end
        if a(1)==b(1)
            relative_s = inverseProgress(progressPower,phases,range(1));
            low_s = min(low_s,min(a(2),b(2))-request.InitialState.time_s-relative_s);
            high_s = max(high_s,max(a(2),b(2))-request.InitialState.time_s-relative_s);
            if a(1)==0, firstStartOccupancy_s = min(firstStartOccupancy_s,min(a(2),b(2))); end
            continue;
        end
        slope_s = (b(2)-a(2))/(b(1)-a(1));
        intercept_s = a(2)-slope_s*a(1);
        if range(1)==0, firstStartOccupancy_s = min(firstStartOccupancy_s,intercept_s); end
        for phase = 1:numel(durations_s)
            overlap = [max(range(1),progressPower(phase,1)),min(range(2),sum(progressPower(phase,:)))];
            if overlap(1)>overlap(2), continue; end
            u = [invertCubic(progressPower(phase,:),overlap(1)),invertCubic(progressPower(phase,:),overlap(2))];
            delayPower_s = slope_s*progressPower(phase,:);
            delayPower_s(1) = delayPower_s(1)+intercept_s-request.InitialState.time_s-phases.StartTime_s(phase);
            delayPower_s(2) = delayPower_s(2)-durations_s(phase);
            derivative = delayPower_s(2:end).*(1:3);
            last = find(derivative~=0,1,'last');
            stationary = [];
            if ~isempty(last), stationary = roots(fliplr(derivative(1:last))); end
            stationary = real(stationary(abs(imag(stationary))<=64*eps(max(1,abs(stationary)))));
            u = [u,reshape(stationary(stationary>=u(1) & stationary<=u(2)),1,[])];
            values_s = delayPower_s*[ones(size(u));u;u.^2;u.^3];
            low_s = min(low_s,min(values_s)); high_s = max(high_s,max(values_s));
        end
    end
    if low_s<=high_s, forbidden_s(end+1,:) = [low_s,high_s]; end %#ok<AGROW>
    % Waiting occupies the initial point for the entire delay, not just at departure.
    if isfinite(firstStartOccupancy_s)
        forbidden_s(end+1,:) = [firstStartOccupancy_s-request.InitialState.time_s,maximumWait_s]; %#ok<AGROW>
    end
end

%% Section 3: Select The First Gap And Export The Complete Motion

% Use the request's arrival resolution to keep proposals away from a grazing
% encounter; one collision subdivision step alone can remain unresolved at a
% sharp moving corner. Acceptance still uses the unchanged public validator.
departureReserve_s = max(request.Options.ArrivalTimeTolerance_s, request.Options.CollisionMinimumTimeStep_s);
forbidden_s = sortrows(forbidden_s + [-departureReserve_s departureReserve_s],1);
wait_s = 0;
for k = 1:size(forbidden_s,1)
    if forbidden_s(k,1)>wait_s, break; end
    if forbidden_s(k,2)>=wait_s
        wait_s = forbidden_s(k,2)+64*eps(max(1,abs(forbidden_s(k,2))));
    end
end
diagnostics = struct('DepartureDelay_s',wait_s,'ForbiddenDepartureInterval_s',forbidden_s, ...
    'Available',wait_s<=maximumWait_s,'MotionDuration_s',duration_s,'DepartureReserve_s',departureReserve_s);
end
function vertices_units = clearanceEnvelope(vertices_units,gap_units)
    % A square Minkowski envelope contains the required Euclidean clearance.
    offsets_units = gap_units*[-1,-1;1,-1;1,1;-1,1];
    vertices_units = reshape(permute(vertices_units+reshape(offsets_units.',1,2,4),[1,3,2]),[],2);
    hull = convhull(vertices_units(:,1),vertices_units(:,2));
    vertices_units = vertices_units(hull(1:end-1),:);
end

function relative_s = inverseProgress(progressPower,phases,position)
    phase = find(sum(progressPower,2)>=position,1);
    if isempty(phase), phase = size(progressPower,1); end
    relative_s = phases.StartTime_s(phase)+phases.SegmentTime_s(phase)*invertCubic(progressPower(phase,:),position);
end

function u = invertCubic(coefficients,position)
    low = 0; high = 1;
    for iteration = 1:48
        middle = (low+high)/2;
        value = coefficients*[1;middle;middle^2;middle^3];
        if value<position, low=middle; else, high=middle; end
    end
    u = (low+high)/2;
end

