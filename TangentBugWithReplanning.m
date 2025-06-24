function [finalPathMatrix, finalSolutionInfo, collisionPoint, resumptionPoint] = ...
    TangentBugWithReplanning(map, pathMatrix, solutionInfo, goal, EPS, numNodes, validationDist,resumptionPoint)
    % Remove nodes inside obstacles
    occupied = checkOccupancy(map, solutionInfo.TreeData(:,1:2));
    validNodes = solutionInfo.TreeData(~occupied, :);

    % Find collision point on current pathMatrix in updated map
    hitIdx = find(checkOccupancy(map, pathMatrix(:,1:2)), 1, 'first');
    if isempty(hitIdx)
        disp('No collision. Using current path as final path.');
        finalPathMatrix = pathMatrix;
        finalSolutionInfo = solutionInfo;
        collisionPoint = []; 
        return;
    end
    collisionPoint = pathMatrix(hitIdx, :);

    % Run TangentBug from collisionPoint to find boundary and resumption point
    [tangentBugPath, resumptionPoint] = runTangentBug(map, collisionPoint, goal);

    % Replan from resumption point to goal
    [newPath, newSolutionInfo, sv] = planPath(map, resumptionPoint, goal, EPS, numNodes, validationDist);

    % Combine previous path (up to collision), boundary path, and new segment
    finalPathMatrix = [pathMatrix(1:hitIdx,:); tangentBugPath; newPath.States];
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

