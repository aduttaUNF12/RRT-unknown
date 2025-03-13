clearvars
close all
%% Variables

time =zeros(1,10);
pathLength=zeros(1,10);
successrate =zeros(1,10);
for run =1:10
    %% Timer start
clearvars -except time pathLength successrate run;
  %  clear(setdiff(who, {'time', 'pathLength', 'successrate','run'}));
EPS = 1;
numNodes = 10000;
validationDist = 0.001;
    tic;
    % Create a blank map (white = free space)
    simpleMap = zeros(1000, 1000); % White map, all free space
    map = occupancyMap(simpleMap, 10);
    Density = numNodes./(100*100);
    %% Define New Obstacles
    newObstacles = {
        %[700 1 600 200];
        %[200 400 600 200];
        %%%%%%%%%%%%%%%%%%%%%%%%%%
         [900, 40, 100, 100];
         [720, 210, 100, 100];
         [620, 330, 100, 100];
         [510, 500, 100, 100];
         [390, 650, 100, 100];
         [280, 790, 100, 100];
         [110, 890, 100, 100];
        };

    %% Define Start & Goal
    start = [0.5 0.5 0];
    goal = [99 99 0];

    % Initial Path Planning
    [path, solutionInfo] = planPath(map, start, goal, EPS, numNodes, validationDist);

    %% Visualize Initial Path
    % map.show
    % hold on
    % plot(solutionInfo.TreeData(:,1), solutionInfo.TreeData(:,2), 'g.-')
    % plot(path.States(:,1), path.States(:,2), 'r-', 'LineWidth', 2)
    % title('Initial Path on Blank Map');

    %% Sequentially Add Obstacles & Replan

    for k = 1:length(newObstacles)
        obstacle = newObstacles{k};
        x = obstacle(1);
        y = obstacle(2);
        w = obstacle(3);
        h = obstacle(4);

        % Add the new obstacle
        newObstacle = ones(h, w); % Black = occupied space
        simpleMap(x:x+h-1, y:y+w-1) = newObstacle;

        % Recalculate affected segment
        mapsize = [size(simpleMap,1), size(simpleMap,2)];
        ycentroid = size(newObstacle,1) / 2;
        xcentroid = size(newObstacle,2) / 2;
        center = mapsize / 2;
        radius = max(xcentroid, ycentroid) + 200;
        rectCenter = [1000 - x + ycentroid, y + xcentroid];
        numNodes = ceil(pi*radius^2* Density);
        % Find the segment affected and replan only that part
        [replannedSegment, solutionInfo, si, sj, s, g,collisionDetected] = ...
            replanPathWithNewObstacle(simpleMap, path, EPS, numNodes, validationDist, radius, rectCenter);

        % Merge the modified segment with the existing path
        if(collisionDetected)
            newPath.States = [path.States(1:si, :); replannedSegment.States; path.States(sj:end, :)];
        else
            newPath.States = path.States(:,:);
        end

        % Update path for next iteration
        path = newPath;



        %% Plot the updated path
        % figure;
        % map = occupancyMap(simpleMap, 10);
        % map.show
        % hold on
        % plot(solutionInfo.TreeData(:,1), solutionInfo.TreeData(:,2), 'g-')
        % plot(path.States(:,1), path.States(:,2), 'b-', 'LineWidth', 2)
        % plot(s(1), s(2), '--bs', 'LineWidth', 2, 'MarkerSize', 5, 'MarkerEdgeColor', 'c', 'MarkerFaceColor', [0.5, 0.5, 0.5])
        % plot(g(1), g(2), '--bs', 'LineWidth', 2, 'MarkerSize', 5, 'MarkerEdgeColor', 'c', 'MarkerFaceColor', [0.5, 0.5, 0.5])
        % title(['Path Replanned with Obstacle ', num2str(k)]);
        % Save the updated plot
        % saveas(gcf, ['/MATLAB Drive/Newobstacle/Replannedimages/path_replanned_obstacle_res1_' num2str(k) '.png']);
    end
    execution_time = toc;
    if(path.States(end,:) == goal)
        success =1;
    else
        success =0;
    end

    % path_length = 0;
    % p_length = path_length + newPath.States(prev_node, curr_node, 'euclidean');
    path_length = 0;
    numPoints = size(newPath.States, 1);
    for i = 1:numPoints-1
        distance = sqrt(sum(( newPath.States(i+1, :) -  newPath.States(i, :)).^2));
        path_length = path_length + distance;
    end
    time(run) =execution_time;
    pathLength(run) = path_length;
    successrate(run) = success;
end
fprintf('Average Path Length: %.4f units\n', mean(pathLength));
fprintf('Average Execution Time: %.6f seconds\n', mean(time));
fprintf('Average Success Rate: %.6f seconds\n', mean(successrate));





