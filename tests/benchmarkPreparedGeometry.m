function measurements = benchmarkPreparedGeometry
%% Section 0: Header & Readme
% SYNTAX: measurements = benchmarkPreparedGeometry
% PURPOSE: Compare cached and original geometry queries on identical histories.
% INPUTS: None.
% OUTPUTS: Individual timings and exact geometry equivalence assertions.
% UNITS: Seconds and coordinate units.

%% Section 1: Prepare Representative Source Histories
counts = [12 120 1200];
measurements = zeros(numel(counts)*3, 4);
row = 0;
for vertexCount = counts
    angles = (0:vertexCount-1).' * 2*pi/vertexCount;
    radius = 3 + 0.5*cos(7*angles);
    vertices = radius .* [cos(angles),sin(angles)];
    obstacle = obstacleAvoidance.obstacles.createObstacle("query benchmark", [4;12], vertices(:,1),vertices(:,2),0);
    obstacle = obstacleAvoidance.obstacles.prepareObstacles(obstacle);
    for time_s = [4 5 8 12]
        [shape, geometry] = obstacleAvoidance.obstacles.preparedShapeAtTime(obstacle,time_s);
        [referenceShape, referenceGeometry] = preparedShapeAtTimeReference(obstacle,time_s);
        assert(isequaln(shape,referenceShape) && isequaln(geometry,referenceGeometry));
    end
    queryTimes_s = linspace(4,12,500);
    cached = @() queryAll(obstacle,queryTimes_s,true);
    scalar = @() queryAll(obstacle,queryTimes_s,false);
    cached(); scalar();
    for repeatIndex = 1:3
        if mod(repeatIndex,2)
            timer=tic; scalar(); scalar_s=toc(timer);
            timer=tic; cached(); cached_s=toc(timer);
        else
            timer=tic; cached(); cached_s=toc(timer);
            timer=tic; scalar(); scalar_s=toc(timer);
        end
        row=row+1;
        measurements(row,:)=[vertexCount,repeatIndex,scalar_s,cached_s];
    end
end
measurements = array2table(measurements,'VariableNames',{'VertexCount','Repetition','Original_s','Cached_s'});
disp(measurements);
end

function queryAll(obstacle,times_s,useCache)
    for time_s=times_s
        if useCache
            [~,~] = obstacleAvoidance.obstacles.preparedShapeAtTime(obstacle,time_s);
        else
            [~,~] = preparedShapeAtTimeReference(obstacle,time_s);
        end
    end
end
