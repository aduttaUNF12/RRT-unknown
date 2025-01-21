function combinedMap = circular_occu_map_corrected(simpleMap,mapSize,radius,rectCenter,s,g)

resolution =1;
newmap = occupancyMap(mapSize, mapSize, resolution);
setOccupancy(newmap, ones(mapSize, 2), 1); % Initialize as fully occupied
[x, y] = meshgrid(1:mapSize, 1:mapSize);
distanceFromCenter = sqrt((x - rectCenter(1)).^2 + (y - rectCenter(2)).^2);

unoccupiedCircle = distanceFromCenter <= radius;
circleCoords = [x(unoccupiedCircle), y(unoccupiedCircle)];
setOccupancy(newmap, circleCoords, 0);
grid1 = getOccupancy(newmap);
grid2 = double(simpleMap);
combinedGrid = max(grid1, grid2);
combinedMap = occupancyMap(combinedGrid, 10);



% Display the occupancy map
show(combinedMap);
hold on;
plot(s(1),s(2),'.','MarkerSize',30)
hold on;
plot(g(1),g(2),'.','MarkerSize',30)
title('2D Occupancy Map with Unoccupied Circle and Occupied Rectangle');
xlabel('X');
ylabel('Y');