function [replannedSegment, solutionInfo,si,sj,s,g,collisionDetected,sv] = replanPathWithNewObstacle(simpleMap,originalPath,EPS,numNodes,validationDist,radius,rectCenter)
map = occupancyMap(simpleMap,10);
newPath = [];
Err =1;
% Initialize newPath with the first portion of the original path
collisionDetected = false;
% Check each point in the original path to find collisions with the new obstacle
for i = 1:size(originalPath.States, 1)
    point = originalPath.States(i, :);
    %%fprintf('Occupancy at (%f, %f): %f\n', point(1), point(2), getOccupancy(map, point(1:2)));
    if ~collisionDetected
        solutionInfo = struct(); % Optional: can include any relevant info
        s = originalPath.States(i, :);
    end
    % If this point collides with the new obstacle
    if checkCollision(map, point)
        % Set s' as the last safe point before collision
        if i > 1
            s = originalPath.States(i - Err, :);
            %s=safepoint;
        else
            s = originalPath.States(1, :);
        end
        collisionDetected = true;

        % Find the next point g' in path P that is not colliding with the new obstacle
        g = [];
        for j = i+1:size(originalPath.States, 1)
            nextPoint = originalPath.States(j, :);
            if ~checkCollision(map, nextPoint)
                %g = nextPoint;
                g= originalPath.States(j, :);
                break;
            end
          
        end

        % If no safe point after the collision is found, exit the function
        if isempty(g)
            error('No collision-free path found after the new obstacle.');
        end
        %newmap1 = circular_occu_map(26,radius,center,rectSize,rectCenter);
        si = i-Err;
        sj = j;
        % Replan path from s' to g'
        mapsize =size(simpleMap);
        map = circular_occu_map_corrected(simpleMap,mapsize(1),radius,rectCenter,s, g);
        [replannedSegment, solutionInfo,sv] = planPath(map, s, g, EPS, numNodes, validationDist);
        break;
    end
end

% If no collision was detected, keep the original path as the new path
if ~collisionDetected
    solutionInfo = struct(); % Optional: can include any relevant info
    safepoint = originalPath.States(i, :);
     si = 1;
     sj = size(originalPath.States, 1);
     replannedSegment = originalPath;
     s=originalPath.States(1,:);
     g=originalPath.States(end,:);
      sv=0;
end 
end