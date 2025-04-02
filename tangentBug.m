clc; clear; close all;

%% Create an Occupancy Grid Map
mapSize = [20, 20]; % Map dimensions (meters)
resolution = 1; % Cells per meter
map = binaryOccupancyMap(mapSize(1), mapSize(2), resolution);

% Initialize all grid cells to free space (0)
setOccupancy(map, [1:mapSize(1); 1:mapSize(2)]', 0);

% Define obstacles [x y width height]
obstacles = [5 5 3 3;
    10 10 4 4;
    15 2 3 5];

% Add obstacles to the occupancy grid
for i = 1:size(obstacles,1)
    xRange = obstacles(i,1):(obstacles(i,1)+obstacles(i,3));
    yRange = obstacles(i,2):(obstacles(i,2)+obstacles(i,4));
    for x = xRange
        for y = yRange
            setOccupancy(map, [x, y], 1); % Mark as occupied
        end
    end
end

%% Define Start and Goal Positions
start = [2, 2];  % Robot start position
goal = [18, 18]; % Goal position

%% Display the Map
figure;
show(map);
hold on;
plot(start(1), start(2), 'go', 'MarkerSize', 12, 'LineWidth', 2);
plot(goal(1), goal(2), 'ro', 'MarkerSize', 12, 'LineWidth', 2);
title('Tangent Bug Algorithm - Real-Time Animation');

%% Initialize Robot Position
robotPos = start;
path = robotPos; % Store the path
stepSize = 0.25; % Movement step size
resumptionPoints = []; % Store points where the robot resumes straight-line motion

% Create real-time plot objects
robotPlot = plot(robotPos(1), robotPos(2), 'bo', 'MarkerSize', 8, 'LineWidth', 2);
pathPlot = plot(path(:,1), path(:,2), 'b-', 'LineWidth', 2);
resumePlot = plot([], [], 'ms', 'MarkerSize', 8, 'LineWidth', 2); % Resumption markers

isFollowingBoundary = false; % Flag to track if the robot is following an obstacle


%% Tangent Bug Algorithm with Real-Time Animation
while norm(robotPos - goal) > stepSize
    % Compute direct line distance to goal
    dirToGoal = goal - robotPos;
    dirToGoal = dirToGoal / norm(dirToGoal); % Normalize direction
    
    % Try moving directly toward the goal
    newPos = robotPos + stepSize * dirToGoal;
    
    % Round position for occupancy check
    %newPosRounded = (newPos);
    
    if getOccupancy(map, newPos) <0.2 % If free space, move
        if isFollowingBoundary
            resumptionPoints = [resumptionPoints; robotPos]; %#ok<AGROW>
            isFollowingBoundary = false;
        end
        robotPos = newPos;
    else
        % Obstacle detected, follow boundary
        isFollowingBoundary = true; % Robot is now in boundary-following mode
        angles = linspace(0, 2*pi, 16); % Discretized directions
        minDist = inf;
        bestPos = robotPos;
        
        for i = 1:length(angles)
            testPos = robotPos + stepSize * [cos(angles(i)), sin(angles(i))];
            %testPosRounded = (testPos); % Round for occupancy check
            
            if getOccupancy(map, testPos) <0.65 % Check free space
                distToGoal = norm(testPos - goal);
                if distToGoal < minDist
                    minDist = distToGoal;
                    bestPos = testPos;
                end
            end
        end
        robotPos = bestPos;
    end
    
    % Store path and update plot
    path = [path; robotPos];
    set(robotPlot, 'XData', robotPos(1), 'YData', robotPos(2)); % Move robot
    set(pathPlot, 'XData', path(:,1), 'YData', path(:,2)); % Update path
    % Update resumption points plot
%     if ~isempty(resumptionPoints)
%         set(resumePlot, 'XData', resumptionPoints(:,1), 'YData', resumptionPoints(:,2));
%     end
    drawnow; % Force immediate plot update
    pause(0.1); % Small delay for smooth animation
end

%% Display Final Path
plot(path(:,1), path(:,2), 'b-', 'LineWidth', 2);
plot(resumptionPoints(:,1), resumptionPoints(:,2), 'ms', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Resumption Points');
legend('Start', 'Goal', 'Robot', 'Path', 'Location','best');
hold off;
