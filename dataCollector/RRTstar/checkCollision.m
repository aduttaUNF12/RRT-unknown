function isColliding = checkCollision(map, point)
% Check if a point collides with obstacles on the map
isColliding = getOccupancy(map, point(1:2)) > 0.01; % Check only (x, y) for collision
end
