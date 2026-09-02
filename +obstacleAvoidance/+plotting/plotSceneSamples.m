function sampleHandles = plotSceneSamples(axesHandle, scene)
%% Section 0: Header & Readme
% SYNTAX
%   sampleHandles = obstacleAvoidance.plotting.plotSceneSamples( ...
%       axesHandle, scene)
%**************************************************************************
% PURPOSE
%   - Draw prepared obstacle samples on caller-owned axes without querying or
%     rebuilding obstacle geometry.
%**************************************************************************
% INPUTS
%   - axesHandle (scalar axes handle)
%       Destination axes configured by the owning stage plotter.
%   - scene (scalar prepared-scene struct)
%       Contains preparedObstacles with retained SampleShapes.
%**************************************************************************
% OUTPUTS
%   - sampleHandles (column graphics array)
%       Polygon handles in obstacle and sample order.
%**************************************************************************
% UNITS
%   - Polygon coordinates are degrees.
%**************************************************************************

%% Section 1: Draw Retained Sample Shapes

if ~isgraphics(axesHandle, "axes") || ~isstruct(scene) || ...
        ~isscalar(scene) || ~isfield(scene, "preparedObstacles")
    error("plotSceneSamples:InvalidInput", ...
        "axesHandle must be axes and scene must contain preparedObstacles.");
end
sampleHandles = gobjects(0);
obstacles = scene.preparedObstacles;
colors = lines(max(1, numel(obstacles)));
for obstacleIndex = 1:numel(obstacles)
    preparation = obstacles(obstacleIndex).InternalPreparation;
    for sampleIndex = 1:numel(preparation.SampleShapes)
        shape = preparation.SampleShapes{sampleIndex};
        if isempty(shape.Vertices)
            continue;
        end
        sampleHandles(end + 1, 1) = plot(axesHandle, shape, ...
            "FaceColor", colors(obstacleIndex, :), "FaceAlpha", 0.06, ...
            "LineWidth", 0.8, "DisplayName", ...
            string(obstacles(obstacleIndex).targetName)); %#ok<AGROW>
    end
end
end
