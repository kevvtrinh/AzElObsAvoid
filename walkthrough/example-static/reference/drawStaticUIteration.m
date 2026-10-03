function drawStaticUIteration(slider, viewerData)
%% Section 0: Header & Readme
% SYNTAX
%   drawStaticUIteration(slider, viewerData)
%**************************************************************************
% PURPOSE
%   Repaint the Step 12 figure using values already calculated in the lesson.
%**************************************************************************
% INPUTS
%   slider: graphics control whose value selects an actual arrival iteration.
%   viewerData: axes, curve samples, pairs and precomputed objective readouts.
%**************************************************************************
% OUTPUTS
%   Updated graphics. No planning or acceptance calculations are performed.
%**************************************************************************
% UNITS
%   Coordinates and line/length costs use units; time uses s and its powers.
%**************************************************************************

%% Section 1: Select One Actual Iteration
iterationIndex = min(numel(viewerData.Samples_units), max(1, round(slider.Value)));
slider.Value = iterationIndex;
positions_units = viewerData.Samples_units{iterationIndex};
overlaps = viewerData.Pairs{iterationIndex};

%% Section 2: Repaint Geometry And The Sampled Pair Map
geometryAxes = viewerData.GeometryAxes;
cla(geometryAxes);
hold(geometryAxes, 'on');
for regionIndex = 1:numel(viewerData.Regions_units)
    vertices_units = viewerData.Regions_units{regionIndex};
    patch(geometryAxes, vertices_units(:, 1), vertices_units(:, 2), [0.75 0.75 0.75], 'FaceAlpha', 0.6);
end
for segmentIndex = 1:size(positions_units, 3)
    color = [0 0.35 0.8];
    if any(overlaps(segmentIndex, :))
        color = [0.85 0.1 0.05];
    end
    plot(geometryAxes, positions_units(:, 1, segmentIndex), positions_units(:, 2, segmentIndex), ...
        'Color', color, 'LineWidth', 2);
end
axis(geometryAxes, 'equal');
% Include the complete proposal, which can extend beyond the wall vertices.
axis(geometryAxes, 'padded');
grid(geometryAxes, 'on');
xlabel(geometryAxes, 'x [units]');
ylabel(geometryAxes, 'y [units]');
title(geometryAxes, sprintf('Iteration %d of %d | proposal %.6f s | retained %d', ...
    iterationIndex, numel(viewerData.Samples_units), viewerData.Duration_s(iterationIndex), ...
    viewerData.Retained(iterationIndex)));
hold(geometryAxes, 'off');
imagesc(viewerData.PairAxes, double(overlaps));
colormap(viewerData.PairAxes, [0.94 0.94 0.94; 0.85 0.1 0.05]);
clim(viewerData.PairAxes, [0 1]);
xticks(viewerData.PairAxes, 1:size(overlaps, 2));
xlabel(viewerData.PairAxes, 'Protected convex region');
ylabel(viewerData.PairAxes, 'Curve segment');
title(viewerData.PairAxes, 'Red cells: sampled overlap (discovery only; full proof follows)');

%% Section 3: Display The Already Calculated Objective Values
clockPowers = viewerData.ClockPowers(iterationIndex, :);
readoutLines = {
    sprintf('MINIMIZED arrival objective p3 = %.9g s^3 | solver exit = %d', ...
        viewerData.ArrivalObjective_s3(iterationIndex), viewerData.ExitFlags(iterationIndex))
    sprintf('Clock variables: p0 = %.6g, p1 = %.6g s, p2 = %.6g s^2, p3 = %.6g s^3', clockPowers)
    sprintf('Travel = %.9g s | polygon length (monitored) = %.9g units | retained = %d', ...
        viewerData.Duration_s(iterationIndex), viewerData.PolygonLength_units(iterationIndex), ...
        viewerData.Retained(iterationIndex))
    'MINIMIZED line q [units], updates for next proposal (S = segment, R = region):'
    };
descriptions = viewerData.LineDescriptions{iterationIndex};
if isempty(descriptions)
    readoutLines{end + 1} = 'No line updates: arrival loop stopped.';
else
    % Four labeled values per row keep all twelve updates readable.
    for firstIndex = 1:4:numel(descriptions)
        readoutLines{end + 1} = strjoin(descriptions(firstIndex:min(firstIndex + 3, numel(descriptions))), '   '); %#ok<AGROW>
    end
end
viewerData.ObjectiveReadout.String = readoutLines;
drawnow;
end
