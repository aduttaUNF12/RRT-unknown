clc;
clearvars;
close all;

%%  RUN METRICS 
time = zeros(1,10);
pathLength = zeros(1,10);
successrate = zeros(1,10);

for run = 1:1
    %% PARAMETERS 
    clearvars -except time pathLength successrate run;
    EPS = 1;                % RRT* max connection distance
    numNodes = 100*10^4;      % RRT* iterations on global plan
    validationDist = 0.1;    % state validator resolution (meters)
    obstaclenum = 6;
    tic;

    %% MAP 
    simpleMap = zeros(1000, 1000, 'uint8');  
    map = occupancyMap(simpleMap, 10);
    Density = numNodes./(100*100);            % node density per m^2

    %%  START & GOAL 
    start = [0.5 0.5 0];   
    goal  = [99 99 0];

    %%  GLOBAL PLAN 
    [path, solutionInfo, sv] = BiRRTplanPath(map, start, goal, EPS, numNodes, validationDist);

    figure('Name','Initial Empty-Map Plan');
    map.show; hold on;
    plot(solutionInfo.GoalTreeData(:,1), solutionInfo.GoalTreeData(:,2), 'g.-')
    plot(path.States(:,1), path.States(:,2), 'r-', 'LineWidth', 2)
    ax = gca; ax.XTick = 0:20:100; ax.YTick = 0:20:100; ax.FontSize = 14;
    title('Initial Path (No Obstacles)');

    %% ADD ALL OBSTACLES
switch obstaclenum
    case 6
    newObstacles = {
      [500 50 50 500];
         [400 380 50 500];
         [200 650 50 500];
         [100 780 50 500];
         [50 920 50 500];};
    case 8
    newObstacles = {
        [900 150 400 50];
        [700 300 400 50];
        [500 150 500 50];
        [300 350 450 50];
        [50  100 450 50];
    };
    case 9
        newObstacles = {
          [700 1  600 100];
          [500 30 700 100];
         [300 1 800  100];
         };
   
 
end
    for k = 1:length(newObstacles)
        obs = newObstacles{k};
        y0 = obs(1); x0 = obs(2); w = obs(3); h = obs(4);
        simpleMap(y0:y0+h-1, x0:x0+w-1) = 1;   % write as [rows, cols]
    end
    map = occupancyMap(simpleMap, 10);

    figure('Name','Map With Obstacles');
    map.show; hold on;
    plot(path.States(:,1), path.States(:,2), 'r-', 'LineWidth', 2)
    ax = gca; ax.XTick = 0:20:100; ax.YTick = 0:20:100; ax.FontSize = 14;
    title('Initial Path Overlaid On Obstacles');

    %%  REAL-TIME LOCAL REPLANNING 
    newPath = path;
    currentIndex = 1;
    res = 10;                        
    mapRows = size(simpleMap,1);       

    while currentIndex < size(newPath.States, 1)
        currentPos = newPath.States(currentIndex,:);
        nextPos    = newPath.States(currentIndex+1,:);  

        % ---- Collision check at NEXT waypoint
        if checkCollision(map, nextPos)
            fprintf('  Collision at step %d — local replanning...\n', currentIndex);

            %% --- s: waypoint just BEFORE collision
            if currentIndex > 1
                s = newPath.States(currentIndex - 1, :);
                sIndex = currentIndex - 1;
            else
                s = newPath.States(currentIndex, :);
                sIndex = currentIndex;
            end

            %% --- g: first SAFE waypoint AFTER obstacle
            g = []; goalIndex = [];
            for j = currentIndex+1:size(newPath.States, 1)
                candidate = newPath.States(j,:);
                if ~checkCollision(map, candidate)
                    g = candidate;
                    goalIndex = j;
                    break;
                end
            end
            if isempty(g)
                error('No collision-free waypoint found after the obstacle.');
            end

            %% Identify which obstacle caused the collision
           
            r = mapRows - round(nextPos(2) * res);
            c = round(nextPos(1) * res);

            hitObstacleIdx = -1;
            matchFound = false;
            for k = 1:length(newObstacles)
                obs = newObstacles{k};
                oy = obs(1); ox = obs(2); ow = obs(3); oh = obs(4);
                if (r >= oy) && (r <= oy + oh - 1) && (c >= ox) && (c <= ox + ow - 1)
                    hitObstacleIdx = k;
                    matchFound = true;
                    break;
                end
            end

            if ~matchFound
                minDist = inf;
                for k = 1:length(newObstacles)
                    obs = newObstacles{k};
                    oy = obs(1); ox = obs(2); ow = obs(3); oh = obs(4);
                    rcCenter = [oy + oh/2, ox + ow/2];   
                    d = hypot(double(r) - rcCenter(1), double(c) - rcCenter(2));
                    if d < minDist
                        minDist = d;
                        hitObstacleIdx = k;
                    end
                end
            end

            %% Use only the matched obstacle
            obs = newObstacles{hitObstacleIdx};
            oy = obs(1); ox = obs(2); ow = obs(3); oh = obs(4);

            % Obstacle center in MATRIX cell coordinates
            rowC = oy + oh/2;
            colC = ox + ow/2;

            % Robust radius in CELLS: half diagonal + margin
            margin_cells = 20;                           
            radiusCells  = 0.5 * hypot(double(ow), double(oh)) + margin_cells;

            % Center for LOCAL MAP: **matrix** rc (NO flip)
            rectCenter = [rowC, colC];

            %% Local map inside circle
            numNodesLocal  = ceil(pi * radiusCells^2 * Density);
            localSimpleMap = circular_occu_map_corrected(simpleMap, mapRows, radiusCells, rectCenter, s, g, res);
            localMap       = occupancyMap(localSimpleMap, res);

            %% Replan inside circle
            [replannedSegment, ~, ~] = BiRRTplanPath(localMap, s, g, EPS, numNodesLocal, validationDist);

            %% Merge path
            mergedStates = [newPath.States(1:sIndex, :); ...
                            replannedSegment.States; ...
                            newPath.States(goalIndex:end, :)];

            newPath = navPath(replannedSegment.StateSpace);
            append(newPath, mergedStates);

            %%  Plot everything
            figure('Name',sprintf('Local Replan step %d',currentIndex));
            map.show; hold on;
            plot(newPath.States(:,1), newPath.States(:,2), 'b-', 'LineWidth', 2)
            plot(s(1), s(2), 'ro', 'MarkerFaceColor', 'r')
            plot(g(1), g(2), 'go', 'MarkerFaceColor', 'g')

            %  Circle plotting in WORLD coords (origin bottom-left)
            x_center = (colC - 0.5) / res;                 
            y_center = (mapRows - (rowC - 0.5)) / res;    
            radius_m = radiusCells / res;

            try
                viscircles([x_center, y_center], radius_m, ...
                    'Color','k','LineStyle','--','LineWidth',1.5);
            catch
                th = linspace(0, 2*pi, 200);
                xc = x_center + radius_m*cos(th);
                yc = y_center + radius_m*sin(th);
                plot(xc, yc, 'k--', 'LineWidth', 1.5);
            end

            title(sprintf('Local Replan at step %d', currentIndex));
            ax = gca; ax.XTick = 0:20:100; ax.YTick = 0:20:100; ax.FontSize = 14;
            drawnow;
        end

        currentIndex = currentIndex + 1;
    end

    %%  METRICS 
    execution_time = toc;
    if all(abs(newPath.States(end,1:2) - goal(1:2)) < 1e-6)
        success = 1;
    else
        success = 0;
    end

    % total path length (meters)
    path_length = 0;
    numPoints = size(newPath.States, 1);
    for i = 1:numPoints-1
        path_length = path_length + norm(newPath.States(i+1,1:2) - newPath.States(i,1:2));
    end

    time(run) = execution_time;
    pathLength(run) = path_length;
    successrate(run) = success;
end

fprintf('Average Path Length: %.4f m\n', mean(pathLength));
fprintf('Average Execution Time: %.6f s\n', mean(time));
fprintf('Average Success Rate: %.6f\n', mean(successrate));

%%  HELPERS 
function [path, solutionInfo, sv] = BiRRTplanPath(map, start, goal, EPS, numNodes, validationDist)
    % Define state space and validator
    ss = stateSpaceSE2;
    sv = validatorOccupancyMap(ss);
    sv.Map = map;
    ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
    sv.ValidationDistance = validationDist;

    % Create the Bi-RRT planner
    planner = plannerBiRRT(ss, sv);
    planner.MaxIterations = numNodes;
    planner.MaxConnectionDistance = EPS;  % Step size
    planner.EnableConnectHeuristic = false;  % Helps speed up connection
    [path, solutionInfo] = plan(planner, start, goal);
   
end

function localSimpleMap = circular_occu_map_corrected(simpleMap, Nrows, radiusCells, rectCenterRC, s, g, res)


    % Convert s,g (world meters) -> MATRIX indices [row, col]
    s_rc = [Nrows - round(s(2)*res), round(s(1)*res)];
    g_rc = [Nrows - round(g(2)*res), round(g(1)*res)];

    rc = rectCenterRC(:)'; % [rowC, colC] in MATRIX indices (no flip)

    % Ensure s and g are inside the circle. If not, inflate radius.
    maxDelta = max([norm(double(s_rc - rc)), norm(double(g_rc - rc))]);
    if maxDelta > radiusCells - 2
        radiusCells = ceil(maxDelta) + 2;
    end

    % Start with all occupied
    localSimpleMap = ones(size(simpleMap), 'uint8');

    % Create circular mask in MATRIX space
    [RR, CC] = ndgrid(1:size(simpleMap,1), 1:size(simpleMap,2));
    maskCircle = ((RR - rc(1)).^2 + (CC - rc(2)).^2) <= (radiusCells^2);

    % Copy global occupancy ONLY inside the circle
    localSimpleMap(maskCircle) = simpleMap(maskCircle);

    % Make sure s and g are explicitly free in the local map
    s_r = min(max(s_rc(1),1), size(localSimpleMap,1));
    s_c = min(max(s_rc(2),1), size(localSimpleMap,2));
    g_r = min(max(g_rc(1),1), size(localSimpleMap,1));
    g_c = min(max(g_rc(2),1), size(localSimpleMap,2));
    localSimpleMap(s_r, s_c) = 0;
    localSimpleMap(g_r, g_c) = 0;
end
