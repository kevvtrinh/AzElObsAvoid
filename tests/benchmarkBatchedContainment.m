function measurements = benchmarkBatchedContainment
%% Section 0: Header & Readme
% SYNTAX: measurements = benchmarkBatchedContainment
% PURPOSE: Measure original and batched containment within identical clearance queries.
% INPUTS: None.
% OUTPUTS: Three alternating-order timings after common warmups and exact checks.
% UNITS: Seconds, coordinate units, query and vertex counts.

%% Section 1: Exercise Independently Varied Geometry And Query Counts
measurements = zeros(0,5);
for vertexCount = [12 120 1200]
    angle = (0:vertexCount-1).' * 2*pi/vertexCount;
    radius = 3+0.7*cos(5*angle);
    obstacle = obstacleAvoidance.obstacles.createObstacle("parity",[2;6],radius.*cos(angle),radius.*sin(angle),0);
    obstacle = obstacleAvoidance.obstacles.prepareObstacles(obstacle);
    [shape,geometry] = obstacleAvoidance.obstacles.preparedShapeAtTime(obstacle,4);
    for queryCount = [100 10000]
        points = 4*[sin((1:queryCount).'),cos((1:queryCount).'*sqrt(2))];
        scalar = @() pointPolygonClearanceReference(shape,points,geometry);
        batch = @() obstacleAvoidance.geometry.pointPolygonClearance(shape,points,geometry);
        assert(isequaln(scalar(),batch()));
        scalar(); batch();
        for repetition = 1:3
            if mod(repetition,2)
                timer=tic; scalar(); original_s=toc(timer);
                timer=tic; batch(); batched_s=toc(timer);
            else
                timer=tic; batch(); batched_s=toc(timer);
                timer=tic; scalar(); original_s=toc(timer);
            end
            measurements(end+1,:) = [vertexCount,queryCount,repetition,original_s,batched_s];
        end
    end
end
measurements = array2table(measurements,'VariableNames',{'VertexCount','QueryCount','Repetition','Original_s','Batched_s'});
disp(measurements);
end
