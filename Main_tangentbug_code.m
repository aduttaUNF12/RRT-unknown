%% Find the initial path
% Parameters
EPS = 0.5;
numNodes = 100*10^2;
validationDist = 0.01;

% Create initial map
simpleMap = zeros(100, 100);
start = [0.5 0.5 0];
goal = [99 99 0];
map = occupancyMap(simpleMap, 1);

% Initial path planning
[initialPath, solutionInfo, sv] = planPath(map, start, goal, EPS, numNodes, validationDist);
currentPathMatrix = initialPath.States; % Store as matrix for evolving path

% Plot original path
figure; map.show; hold on;
plot(solutionInfo.TreeData(:,1), solutionInfo.TreeData(:,2), 'g.-');
plot(currentPathMatrix(:,1), currentPathMatrix(:,2), 'r-', 'LineWidth', 2);
plot(start(1), start(2), 'go', 'MarkerSize',8,'MarkerFaceColor','g');
plot(goal(1), goal(2), 'ro', 'MarkerSize',8,'MarkerFaceColor','r');
title("Original Path Without Obstacles");
%legend('Tree','Path','Start','Goal','Location','best');

%% replanning with unknown obstacles
% Define multiple obstacles [x, y, height, width]
obstaclesList = [
    70 1 10 60;
    50 50 10 50;
    30 1 10 70
    ];
resumptionPoint=start;
newPathStartingPoint = 1; %size(currentPathMatrix,1);


% Sequentially add obstacles and replan
for i = 1:size(obstaclesList,1)
    % Extract obstacle parameters
    x = obstaclesList(i,1);
    y = obstaclesList(i,2);
    h = obstaclesList(i,3);
    w = obstaclesList(i,4);

    % Update map with this obstacle
    simpleMap(x:x+h-1, y:y+w-1) = 1;
    map = occupancyMap(simpleMap, 1);

    % Run TangentBug replanning on updated currentPathMatrix and map
    [newPathMatrix, solutionInfo, collisionPoint, resumptionPoint,nPSP] = ...
        TangentBugBasedReplanning(map, currentPathMatrix, solutionInfo, goal, EPS, numNodes, validationDist, resumptionPoint, newPathStartingPoint);
    %disp(resumptionPoint); disp(' --- '); disp(collisionPoint);
    newPathStartingPoint = nPSP;
    %disp(newPathStartingPoint)
    % Update currentPathMatrix to the new path for next obstacle
    currentPathMatrix = newPathMatrix;

    % Plot updated path
    figure; map.show; hold on;
    %plot(solutionInfo.TreeData(:,1), solutionInfo.TreeData(:,2), 'g.-');
    plot(currentPathMatrix(:,1), currentPathMatrix(:,2), 'b-', 'LineWidth', 2);
    plot(start(1), start(2), 'go', 'MarkerSize',8,'MarkerFaceColor','g');
    plot(goal(1), goal(2), 'ro', 'MarkerSize',8,'MarkerFaceColor','r');
    if ~isempty(collisionPoint)
        plot(collisionPoint(1), collisionPoint(2), 'mx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName','Collision Point');
    end
    if ~isempty(resumptionPoint)
        plot(resumptionPoint(1), resumptionPoint(2), 'ms', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName','Resumption Point');
    end
    title(sprintf("Updated Path After Obstacle %d", i));
    %legend('Tree','Path','Start','Goal','Collision Point','Resumption Point');
end
hold off;
disp('Path planning complete for all obstacles.');


