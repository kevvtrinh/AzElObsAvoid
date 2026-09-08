function regions_units = convexRegions(shape)
%% Section 0: Header & Readme
% SYNTAX: regions_units = obstacleAvoidance.geometry.convexRegions(shape)
% PURPOSE: Exactly cover a polygon (including holes) with convex regions.
% INPUTS: A valid polyshape.
% OUTPUTS: Column cell array of convex vertex arrays.
% UNITS: Coordinate units.

%% Section 1: Keep Convex Components Or Triangulate Exact Geometry
regions_units = cell(0, 1);
components = regions(shape);
for k = 1:numel(components)
    vertices_units = components(k).Vertices;
    if components(k).NumHoles == 0 && all(isfinite(vertices_units), 'all')
        edges_units = circshift(vertices_units, -1) - vertices_units;
        next_units = circshift(edges_units, -1);
        turns_units2 = edges_units(:,1).*next_units(:,2) - edges_units(:,2).*next_units(:,1);
        if all(turns_units2 >= 0) || all(turns_units2 <= 0)
            regions_units{end+1,1} = vertices_units; %#ok<AGROW>
            continue;
        end
    end
    mesh = triangulation(components(k));
    for j = 1:size(mesh.ConnectivityList, 1)
        regions_units{end+1,1} = mesh.Points(mesh.ConnectivityList(j,:),:); %#ok<AGROW>
    end
end
end
