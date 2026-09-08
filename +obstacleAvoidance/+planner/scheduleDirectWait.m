function [candidate, diagnostics] = scheduleDirectWait(obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX: [candidate, diagnostics] = scheduleDirectWait(obstacles, initialState, goalState, limits, options)
% PURPOSE: Schedule an exact rest-to-rest chord against complete protected histories.
% INPUTS: Prepared source obstacles and normalized endpoint, limit, and option records.
% OUTPUTS: A motion proposal and explicit eligibility/departure evidence. The caller
%   must independently validate the complete motion and compare it with detours.
% UNITS: Coordinate units and seconds, with physical velocity, acceleration, and jerk.

%% Section 1: Require An Exact Straight-Progress Clock
minimumOptions = options; minimumOptions.GoalTimeMode = "earliestArrival";
candidate = bmtpEngine.createDirectMotion(initialState,goalState,limits,minimumOptions);
diagnostics = struct('Available',false,'Reason',"inapplicableDirectClock");
if isfield(goalState,'targetTime_s') && ~isempty(goalState.targetTime_s)
    diagnostics.Reason = "movingTargetClock";
    return;
end
if ~candidate.Success || ~candidate.UsedStraightProgress || ...
        any([initialState.velocity_units_s,initialState.acceleration_units_s2,goalState.velocity_units_s,goalState.acceleration_units_s2] ~= 0)
    return;
end

%% Section 2: Compile Source Geometry Into Convex Space-Time Cells
regions_units = {}; endRegions_units = {}; activeTimeInterval_s = zeros(0,2);
for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacles(obstacleIndex);
    historyTime_s = obstacle.time_s;
    if isscalar(historyTime_s)
        historyTime_s = [initialState.time_s;goalState.time_s];
    else
        lowerTime_s = max(initialState.time_s,historyTime_s(1)); upperTime_s = min(goalState.time_s,historyTime_s(end));
        if upperTime_s < lowerTime_s, continue; end
        historyTime_s = unique([lowerTime_s;historyTime_s(historyTime_s>lowerTime_s & historyTime_s<upperTime_s);upperTime_s]);
        if isscalar(historyTime_s), historyTime_s = [historyTime_s;historyTime_s]; end
    end
    for intervalIndex = 1:numel(historyTime_s)-1
        endpoints_s = historyTime_s(intervalIndex:intervalIndex+1).';
        [middleShape,geometry] = obstacleAvoidance.obstacles.preparedShapeAtTime(obstacle,mean(endpoints_s));
        if isempty(middleShape.Vertices), continue; end
        if geometry.VertexSpeedBound_units_s == 0
            pieces = obstacleAvoidance.geometry.convexPolygonRegions(middleShape);
            for pieceIndex = 1:numel(pieces)
                vertices_units = pieces(pieceIndex).Vertices;
                vertices_units = vertices_units(all(isfinite(vertices_units),2),:);
                if size(vertices_units,1)>128
                    diagnostics.Reason = "spaceTimeSectionBudget"; return;
                end
                regions_units{end+1,1} = vertices_units; endRegions_units{end+1,1} = vertices_units;
                activeTimeInterval_s(end+1,:) = endpoints_s;
            end
        elseif geometry.HasOrderedSingleRegion && geometry.IsConvex
            % Corresponding vertices move linearly over this source interval.
            % The convex hull in space-time contains every intermediate shape;
            % it may conservatively cover extra space for a deforming polygon.
            firstShape = obstacleAvoidance.obstacles.preparedShapeAtTime(obstacle,endpoints_s(1));
            lastShape = obstacleAvoidance.obstacles.preparedShapeAtTime(obstacle,endpoints_s(2));
            if size(firstShape.Vertices,1)>128 || size(lastShape.Vertices,1)>128
                diagnostics.Reason = "spaceTimeSectionBudget"; return;
            end
            regions_units{end+1,1} = firstShape.Vertices; endRegions_units{end+1,1} = lastShape.Vertices;
            activeTimeInterval_s(end+1,:) = endpoints_s;
        else
            diagnostics.Reason = "unsupportedDeformingConcavity"; return;
        end
    end
end
%% Section 3: Schedule And Export The Complete Wait And Motion
request = struct('InitialState',initialState,'GoalState',goalState,'Limits',limits,'Options',options, ...
    'MotionHorizon_s',goalState.time_s-initialState.time_s,'Regions_units',{regions_units}, ...
    'Coverage',struct('EndRegions_units',{endRegions_units},'ActiveTimeInterval_s',activeTimeInterval_s));
diagnostics = bmtpEngine.scheduleChordDeparture(request,candidate);
diagnostics.TimeCellCount = numel(regions_units);
if ~diagnostics.Available, return; end
waitTime_s = diagnostics.DepartureDelay_s;
relativeBreak_s = [candidate.Polynomial.SegmentStartTime_s;candidate.Polynomial.FinalTime_s]-initialState.time_s;
jerk_units_s3 = reshape(candidate.Polynomial.jerkPower_units_s3,candidate.Polynomial.SegmentCount,2);
if waitTime_s>0
    relativeBreak_s = [0;waitTime_s+relativeBreak_s]; jerk_units_s3 = [0 0;jerk_units_s3];
end
candidate = bmtpEngine.createMotionRecord(candidate,initialState,relativeBreak_s,jerk_units_s3,options.SampleTime_s,"scheduledChord");

candidate.Success = true;
candidate.Message = "An analytic departure proposal requires independent validation.";
candidate.SolverDiagnostics = diagnostics;
end


