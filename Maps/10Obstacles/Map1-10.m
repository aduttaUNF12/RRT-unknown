% Define grid size
gridSize = 100;

% Create figure
figure;
hold on;
axis([0 gridSize 0 gridSize]); % Set axis limits
xlabel('X Axis');
ylabel('Y Axis');
title('Occupancy Grid Map with Obstacle');

% Define obstacle vertices (bottom-left to top-right)
xObstacle = [0 30 30 0 0]; % x-coordinates (closing shape)
yObstacle = [20 20 30 30 20]; % y-coordinates (closing shape)

% Plot obstacle as a black-filled polygon
fill(xObstacle, yObstacle, 'k');

% Set grid and appearance
grid on;
set(gca, 'XTick', 0:10:gridSize, 'YTick', 0:10:gridSize);
hold off;
