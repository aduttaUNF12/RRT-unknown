function [finalPathMatrix, finalSolutionInfo, collisionPoint, resumptionPoint,newPathStartingPoint] = ...
    TangentBugBasedReplanning(map, pathMatrix, solutionInfo, goal, EPS, numNodes, validationDist,resumptionPoint, newPathStartingPoint)
    % Remove nodes inside obstacles
    occupied = checkOccupancy(map, solutionInfo.TreeData(:,1:2));
    validNodes = solutionInfo.TreeData(~occupied, :);

    % Find collision point on current pathMatrix in updated map
    %hitIdxAll = find(checkOccupancy(map, pathMatrix(newPathStartingPoint:end,1:2)))+ newPathStartingPoint - 1;
    hitIdx = find(checkOccupancy(map, pathMatrix(newPathStartingPoint:end,1:2)), 1, 'first') + newPathStartingPoint - 1;
    if isempty(hitIdx)
        disp('No collision. Using current path as final path.');
        finalPathMatrix = pathMatrix;
        finalSolutionInfo = solutionInfo;
        collisionPoint = []; 
        return;
    end
    collisionPoint = pathMatrix(hitIdx, :);
    %disp('are we even coming here?'); disp(collisionPoint);
    % Run TangentBug from collisionPoint to find boundary and resumption point
    [tangentBugPath, resumptionPoint] = runTangentBug2(map, collisionPoint, goal);

    % Replan from resumption point to goal
    [newPath, newSolutionInfo, ~] = planPath(map, resumptionPoint, goal, EPS, numNodes, validationDist);

    % Combine previous path (up to collision), boundary path, and new segment
    finalPathMatrix = [pathMatrix(1:hitIdx,:); tangentBugPath; newPath.States];
    newPathStartingPoint = hitIdx + size(tangentBugPath,1) + 1;
    %finalPathMatrix = [tangentBugPath; newPath.States];
    finalSolutionInfo = newSolutionInfo;
    finalSolutionInfo.TreeData = [validNodes; newSolutionInfo.TreeData];
end

function [tangentBugPath, resumptionPoint] = runTangentBug(map, startPos, goal)
    stepSize = 0.25;
    robotPos = startPos;
    path = robotPos;
    resumptionPoint = [];
    isFollowingBoundary = false;

    while norm(robotPos(1:2) - goal(1:2)) > stepSize
        dirToGoal = goal - robotPos;
        dirToGoal = dirToGoal / norm(dirToGoal);
        newPos = robotPos + stepSize * dirToGoal;
        if getOccupancy(map, newPos(1,1:2)) < 0.2
            if isFollowingBoundary
                resumptionPoint = robotPos;
                break; % Found resumption point
            end
            robotPos = newPos;
        else
            isFollowingBoundary = true;
            angles = linspace(0, 2*pi, 16);
            minDist = inf; bestPos = robotPos;
            for i = 1:length(angles)
                testPos = robotPos(1,1:2) + stepSize * [cos(angles(i)), sin(angles(i))];
                testPos = [testPos, 0];
                if getOccupancy(map, testPos(1,1:2)) < 0.65
                    distToGoal = norm(testPos - goal);
                    if distToGoal < minDist
                        minDist = distToGoal; bestPos = testPos;
                    end
                end
            end
            robotPos = bestPos;
        end
        path = [path; robotPos];
    end

    tangentBugPath = path;
    if isempty(resumptionPoint)
        resumptionPoint = robotPos;
    end
end


function [tangentBugPath, resumptionPoint] = runTangentBug2(map, startPos, goal)
    stepSize = 0.25;
    robotPos = startPos;
    path = robotPos;
    resumptionPoint = [];
    isFollowingBoundary = false;
    
    % Get map boundaries
    mapLimits = [map.XWorldLimits; map.YWorldLimits];
    boundaryTolerance = stepSize * 2; % Distance threshold for boundary detection
    
    % Variables for boundary handling
    hitPoint = startPos; % startPos is the first hitPoint
    exploredSides = []; % Track which sides of obstacle we've explored
    maxBoundarySteps = 200; % Maximum steps to follow boundary before giving up
    boundaryStepCount = 0;
    
    while norm(robotPos(1:2) - goal(1:2)) > stepSize
        dirToGoal = goal - robotPos;
        dirToGoal = dirToGoal / norm(dirToGoal);
        newPos = robotPos + stepSize * dirToGoal;
        
        if getOccupancy(map, newPos(1,1:2)) < 0.2
            if isFollowingBoundary
                resumptionPoint = robotPos;
                break; % Found resumption point
            end
            robotPos = newPos;
            boundaryStepCount = 0; % Reset boundary step counter
        else
            % Hit an obstacle
            if ~isFollowingBoundary
                % First time hitting obstacle - startPos is already the hit point
                isFollowingBoundary = true;
                boundaryStepCount = 0;
            end
            
            % Check if we're at environment boundary
            atBoundary = isAtBoundary(robotPos, mapLimits, boundaryTolerance);
            
            if atBoundary || boundaryStepCount > maxBoundarySteps
                % We've hit boundary or been following too long
                % Try the other side of the obstacle
                newSide = getOppositeSide(robotPos, hitPoint);
                
                % Check if we've already explored this side
                if ~any(exploredSides == newSide)
                    exploredSides = [exploredSides, newSide];
                    robotPos = exploreOtherSide(hitPoint, newSide, stepSize, map, goal);
                    %boundaryStepCount = 0;
                else
                    % Both sides explored, declare failure or try different strategy
                    fprintf('Warning: Both sides of obstacle explored. Attempting emergency navigation.\n');
                    robotPos = emergencyNavigation(robotPos, goal, stepSize, map);
                    if isempty(robotPos)
                        break; % No path found
                    end
                end
            else
                % Normal boundary following
                angles = linspace(0, 2*pi, 16);
                minDist = inf; 
                bestPos = robotPos;
                
                for i = 1:length(angles)
                    testPos = robotPos(1,1:2) + stepSize * [cos(angles(i)), sin(angles(i))];
                    testPos = [testPos, 0];
                    
                    % Check if test position is within map bounds
                    if isWithinBounds(testPos, mapLimits) && getOccupancy(map, testPos(1,1:2)) < 0.65
                        distToGoal = norm(testPos - goal);
                        if distToGoal < minDist
                            minDist = distToGoal; 
                            bestPos = testPos;
                        end
                    end
                end
                
                robotPos = bestPos;
                boundaryStepCount = boundaryStepCount + 1;
            end
        end
        
        path = [path; robotPos];
    end
    
    tangentBugPath = path;
    if isempty(resumptionPoint)
        resumptionPoint = robotPos;
    end
end

function atBoundary = isAtBoundary(pos, mapLimits, tolerance)
    % Check if position is near environment boundary
    atBoundary = (pos(1) <= mapLimits(1,1) + tolerance) || ...
                 (pos(1) >= mapLimits(1,2) - tolerance) || ...
                 (pos(2) <= mapLimits(2,1) + tolerance) || ...
                 (pos(2) >= mapLimits(2,2) - tolerance);
end

function withinBounds = isWithinBounds(pos, mapLimits)
    % Check if position is within map boundaries
    withinBounds = (pos(1) >= mapLimits(1,1)) && (pos(1) <= mapLimits(1,2)) && ...
                   (pos(2) >= mapLimits(2,1)) && (pos(2) <= mapLimits(2,2));
end

function side = getOppositeSide(currentPos, hitPoint)
    % Determine which side of obstacle we're on and return opposite
    % Simple heuristic: if we're to the right of hit point, go left (and vice versa)
    if currentPos(1) > hitPoint(1)
        side = -1; % Go to left side
    else
        side = 1;  % Go to right side
    end
end

function newPos = exploreOtherSide(hitPoint, side, stepSize, map, goal)
    % Move to explore the other side of the obstacle
    % Try to move perpendicular to the obstacle direction
    searchAngles = linspace(0, 2*pi, 32);
    
    % Prefer angles that go in the direction of the specified side
    if side == 1
        % Prefer right side angles
        searchAngles = [searchAngles(1:8), searchAngles(25:32), searchAngles(9:24)];
    else
        % Prefer left side angles
        searchAngles = [searchAngles(9:24), searchAngles(1:8), searchAngles(25:32)];
    end
    
    for angle = searchAngles
        testPos = hitPoint(1:2) + stepSize * 2 * [cos(angle), sin(angle)];
        testPos = [testPos, 0];
        
        if getOccupancy(map, testPos(1,1:2)) < 0.2
            newPos = testPos;
            return;
        end
    end
    
    % If no free space found, return hit point
    newPos = hitPoint;
end

function newPos = emergencyNavigation(currentPos, goal, stepSize, map)
    % Emergency navigation when normal boundary following fails
    % Try to find any free space that gets us closer to goal
    
    % Expand search radius progressively
    for radius = stepSize:stepSize:stepSize*4
        angles = linspace(0, 2*pi, 32);
        bestPos = [];
        minDist = inf;
        
        for angle = angles
            testPos = currentPos(1:2) + radius * [cos(angle), sin(angle)];
            testPos = [testPos, 0];
            
            if getOccupancy(map, testPos(1,1:2)) < 0.2
                distToGoal = norm(testPos - goal);
                if distToGoal < minDist
                    minDist = distToGoal;
                    bestPos = testPos;
                end
            end
        end
        
        if ~isempty(bestPos)
            newPos = bestPos;
            return;
        end
    end
    
    % If no position found, return empty (indicating failure)
    newPos = [];
end
