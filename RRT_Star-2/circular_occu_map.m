function map = circular_occu_map(mapSize,radius,center,rectSize,rectCenter)
% Parameters
%mapSize = 100; % Define the size of the occupancy map (e.g., 100x100 grid)
%radius = 20;   % Radius of the circular unoccupied area
%center = [50, 50]; % Center of the circle (x, y) in the map
resolution = 1; % Define the resolution (cells per meter)

% Rectangle dimensions and position
%rectSize = [10, 5]; % Size of the rectangle [a, b] in cells
%rectCenter = center; % Center the rectangle inside the circular area

% Create an empty occupancy map and set all cells to occupied (1)
map = occupancyMap(mapSize, mapSize, resolution);
%setOccupancy(map, ones(mapSize, 2), 1); % Initialize as fully occupied

% Create a 2D grid
[x, y] = meshgrid(1:mapSize, 1:mapSize);

% Calculate the distance from each point to the circle center
distanceFromCenter = sqrt((x - center(1)).^2 + (y - center(2)).^2);

% Set cells within the circular area as unoccupied (0)
unoccupiedCircle = distanceFromCenter <= radius;
circleCoords = [x(unoccupiedCircle), y(unoccupiedCircle)];
setOccupancy(map, circleCoords, 0);

% set cells outside of the circle as occupied (1)
outsideCircle = distanceFromCenter > radius;
outsideCoords = [x(outsideCircle), y(outsideCircle)];
setOccupancy(map, outsideCoords, 1);

% Define the rectangular region within the circle to be occupied (1)
xRange = (rectCenter(1) - rectSize(1)/2):(rectCenter(1) + rectSize(1)/2);
yRange = (rectCenter(2) - rectSize(2)/2):(rectCenter(2) + rectSize(2)/2);

% Ensure the indices are within map bounds
xRange = max(1, min(mapSize, xRange));
yRange = max(1, min(mapSize, yRange));

% Set the rectangle area to occupied (1)
[rectX, rectY] = meshgrid(xRange, yRange);
rectangleCoords = [rectX(:), rectY(:)];
setOccupancy(map, rectangleCoords, 1);

% Display the occupancy map
show(map);
title('2D Occupancy Map with Unoccupied Circle and Occupied Rectangle');
xlabel('X');
ylabel('Y');
