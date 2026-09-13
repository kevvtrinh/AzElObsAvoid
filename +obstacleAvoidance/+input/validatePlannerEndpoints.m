function [feasible,message,reason] = validatePlannerEndpoints(obstacles,initialState,goalState,limits,options)
%% Section 0: Header & Readme
% SYNTAX: [ok,message,reason] = obstacleAvoidance.input.validatePlannerEndpoints(obstacles,initial,goal,limits,options)
% PURPOSE: Reject physical endpoint violations at their actual physical times.
% INPUTS: Prepared geometry, normalized full states, limits, and arrival policy.
% OUTPUTS: Feasibility and stable expected-failure description.
% UNITS: Coordinate units, seconds, and physical derivatives.

%% Section 1: Check Occupancy At The Requested Endpoint Times
feasible = false; message = ""; reason = "";
positions = initialState.position_units;
times = initialState.time_s;
if options.GoalTimeMode=="fixedArrival"
    positions(2,:) = goalState.position_units; times(2,1) = goalState.time_s;
end
if any(obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(obstacles,positions(:,1),positions(:,2),times))
    message = "A protected obstacle occupies the initial or fixed terminal state.";
    reason = "endpointBlocked"; return;
end
if options.GoalTimeMode=="fixedArrival" && ~options.WrapX && ~options.WrapY && ...
        terminalReachabilityIsBlocked(obstacles,initialState,goalState,limits)
    message = "Every jerk-limited approach to the fixed terminal state intersects a protected obstacle.";
    reason = "terminalReachabilityBlocked"; return;
end

%% Section 2: Check Physical States And Necessary Travel Time
checkGoal = options.GoalTimeMode=="fixedArrival" || isempty(goalState.targetMotion);
states = initialState;
if checkGoal
    states(2).position_units = goalState.position_units;
    states(2).velocity_units_s = goalState.velocity_units_s;
    states(2).acceleration_units_s2 = goalState.acceleration_units_s2;
end
intervals = [limits.xInterval_units;limits.yInterval_units];
for k = 1:numel(states)
    if any(abs(states(k).velocity_units_s)>limits.maxVelocity_units_s) || ...
            any(abs(states(k).acceleration_units_s2)>limits.maxAcceleration_units_s2)
        message = "An endpoint derivative exceeds its physical limit.";
        reason = "dynamicEndpointInfeasible"; return;
    end
    p = states(k).position_units;
    if any(p<intervals(:,1).' | p>intervals(:,2).')
        message = "An endpoint lies outside the workspace.";
        reason = "endpointOutsideWorkspace"; return;
    end
end
if checkGoal && obstacleAvoidance.input.minimumTravelTime(initialState,goalState,limits)> ...
        goalState.time_s-initialState.time_s+options.ArrivalTimeTolerance_s
    message = "The horizon is below a necessary travel-time bound.";
    reason = "timeWindowInfeasible"; return;
end
feasible = true;
end

%% Section 3: Local Functions
function blocked = terminalReachabilityIsBlocked(obstacles,initialState,goalState,limits)
    % A convex obstacle containing the complete backward-reachable box proves infeasibility.
    blocked = false;
    finalTime_s = goalState.time_s;
    previousEventTime_s = initialState.time_s;
    for obstacleIndex = 1:numel(obstacles)
        priorTimes_s = obstacles(obstacleIndex).time_s( ...
            obstacles(obstacleIndex).time_s<finalTime_s);
        if ~isempty(priorTimes_s)
            previousEventTime_s = max(previousEventTime_s,max(priorTimes_s));
        end
    end
    localDuration_s = finalTime_s-previousEventTime_s;
    if localDuration_s<=0
        return
    end
    candidateDuration_s = unique([localDuration_s*2.^-(0:16), ...
        linspace(localDuration_s/256,localDuration_s,256)]);
    for duration_s = reshape(candidateDuration_s,1,[])
        center_units = goalState.position_units-goalState.velocity_units_s*duration_s+ ...
            0.5*goalState.acceleration_units_s2*duration_s^2;
        radius_units = backwardPositionRadius(duration_s,goalState.acceleration_units_s2, ...
            limits.maxAcceleration_units_s2,limits.maxJerk_units_s3);
        corners_units = center_units+[-radius_units(1),-radius_units(2); ...
            -radius_units(1),radius_units(2);radius_units(1),-radius_units(2); ...
            radius_units(1),radius_units(2)];
        scene = obstacleAvoidance.obstacles.snapshot(obstacles,finalTime_s-duration_s);
        for sceneIndex = 1:numel(scene)
            for regionIndex = 1:numel(scene(sceneIndex).Regions_units)
                region_units = scene(sceneIndex).Regions_units{regionIndex};
                [inside,onBoundary] = inpolygon(corners_units(:,1),corners_units(:,2), ...
                    region_units(:,1),region_units(:,2));
                if all(inside | onBoundary)
                    blocked = true;
                    return
                end
            end
        end
    end
end

function radius_units = backwardPositionRadius(duration_s,finalAcceleration_units_s2,accelerationLimit_units_s2,jerkLimit_units_s3)
    % Ignore velocity limits to retain a sound outer bound; acceleration tightens long intervals.
    rampDuration_s = accelerationLimit_units_s2./jerkLimit_units_s3;
    radius_units = jerkLimit_units_s3*duration_s^3/6;
    accelerationLimited = finalAcceleration_units_s2==0 & duration_s>rampDuration_s;
    acceleration = accelerationLimit_units_s2(accelerationLimited);
    jerk = jerkLimit_units_s3(accelerationLimited);
    radius_units(accelerationLimited) = 0.5*acceleration*duration_s^2- ...
        acceleration.^2*duration_s./(2*jerk)+acceleration.^3./(6*jerk.^2);
end
