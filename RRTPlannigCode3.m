%% Path planning with KD-tree bridge (addYtoT) and TangentBug replanning

% Parameters
EPS = 0.5;                 % planner connection distance / δ for extension
numNodes = 100*10^2;
validationDist = 0.01;

% Create initial empty map
simpleMap = zeros(100, 100);
start = [0.5 0.5 0];
goal  = [99 99 0];
map   = occupancyMap(simpleMap, 1);

% Initial path planning
[initialPath, solutionInfo, sv] = planPath(map, start, goal, EPS, numNodes, validationDist);
currentPathMatrix = initialPath.States; % evolving path

% Plot original path
figure; map.show; hold on;
plot(solutionInfo.TreeData(:,1), solutionInfo.TreeData(:,2), 'g.-');
plot(currentPathMatrix(:,1), currentPathMatrix(:,2), 'r-', 'LineWidth', 2);
plot(start(1), start(2), 'go', 'MarkerSize',8,'MarkerFaceColor','g');
plot(goal(1), goal(2), 'ro', 'MarkerSize',8,'MarkerFaceColor','r');
title("Original Path Without Obstacles");

%% Replanning with unknown rectangular obstacles [x, y, h, w]
obstaclesList = [
    70 1  10 60
    50 30 10 70
    30 1  10 80
];
resumptionPoint   = start;
newPathStartingPt = 1;

for i = 1:size(obstaclesList,1)
    % Add the i-th obstacle
    x0 = obstaclesList(i,1);
    y0 = obstaclesList(i,2);
    h  = obstaclesList(i,3);
    w  = obstaclesList(i,4);

    simpleMap(x0:x0+h-1, y0:y0+w-1) = 1;
    map = occupancyMap(simpleMap, 1);

    % Replan around new obstacle(s)
    [newPathMatrix, solutionInfo, collisionPoint, resumptionPoint, newPathStartingPt] = ...
        TangentBugBasedReplanning(map, currentPathMatrix, solutionInfo, goal, ...
                                  EPS, numNodes, validationDist, resumptionPoint, newPathStartingPt);

    % Update the evolving path
    currentPathMatrix = newPathMatrix;

    % Plot updated path
    figure; map.show; hold on;
    plot(currentPathMatrix(:,1), currentPathMatrix(:,2), 'b-', 'LineWidth', 2);
    plot(start(1), start(2), 'go', 'MarkerSize',8,'MarkerFaceColor','g');
    plot(goal(1), goal(2), 'ro', 'MarkerSize',8,'MarkerFaceColor','r');
    if ~isempty(collisionPoint)
        plot(collisionPoint(1), collisionPoint(2), 'mx', 'MarkerSize', 10, 'LineWidth', 2);
    end
    if ~isempty(resumptionPoint)
        plot(resumptionPoint(1), resumptionPoint(2), 'ms', 'MarkerSize', 10, 'LineWidth', 2);
    end
    title(sprintf("Updated Path After Obstacle %d", i));
end
hold off;
disp('Path planning complete for all obstacles.');

%% --------------------------- Functions ---------------------------

function [path, solutionInfo, sv] = planPath(map, start, goal, EPS, numNodes, validationDist)
    ss = stateSpaceSE2;
    sv = validatorOccupancyMap(ss); sv.Map = map;
    ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
    sv.ValidationDistance = validationDist;
    planner = plannerRRTStar(ss, sv, ...
        ContinueAfterGoalReached=true, MaxIterations=numNodes, ...
        MaxConnectionDistance=EPS, ...
        GoalReachedFcn=@(~,s,g)(norm(s(1:2)-g(1:2))<0.001), GoalBias=0.01);
    rng(100,'twister');
    [path, solutionInfo] = plan(planner, start, goal);
end

function [finalPathMatrix, finalSolutionInfo, collisionPoint, resumptionPoint, newPathStartingPoint] = ...
    TangentBugBasedReplanning(map, pathMatrix, solutionInfo, goal, EPS, numNodes, validationDist, resumptionPoint, newPathStartingPoint)
    % Remove nodes inside obstacles for visualization/robustness
    occupied   = checkOccupancy(map, solutionInfo.TreeData(:,1:2));
    validNodes = solutionInfo.TreeData(~occupied, :);

    % First collision index along the current (remaining) path
    hitIdx = find(checkOccupancy(map, pathMatrix(newPathStartingPoint:end,1:2)), 1, 'first');
    if ~isempty(hitIdx)
        hitIdx = hitIdx + newPathStartingPoint - 1;
    end

    if isempty(hitIdx)
        % Nothing blocks the current path – keep it
        finalPathMatrix   = pathMatrix;
        finalSolutionInfo = solutionInfo;
        collisionPoint    = [];
        return;
    end

    collisionPoint = pathMatrix(hitIdx, :);

    % TangentBug-like boundary following from the collision point → resumption point y
    [tangentBugPath, resumptionPoint] = runTangentBug2(map, collisionPoint, goal);

    % === KD-tree "addYtoT": connect resumption point y to the existing tree T ===
    T = solutionInfo.TreeData(:, 1:min(3, size(solutionInfo.TreeData,2)));   % x,y,(theta)
    maxEdge = EPS;     % δ
    K       = 15;      % number of neighbor candidates
    R       = 4*EPS;   % search radius
    [bridgePath, nearestIdx] = addYtoT(map, resumptionPoint, T, maxEdge, K, R); %#ok<ASGLU>

    % Append bridge nodes (except the last node which is an existing tree node)
    if ~isempty(bridgePath)
        toAppend = bridgePath(1:end-1, :);  % y ... (up to but not including x)
        padCols  = max(0, size(solutionInfo.TreeData,2) - size(toAppend,2));
        if padCols > 0, toAppend = [toAppend, zeros(size(toAppend,1), padCols)]; end
        solutionInfo.TreeData = [solutionInfo.TreeData; toAppend];
    end

    % Replan from resumption point y → goal
    [newPath, newSolutionInfo] = planPath(map, resumptionPoint, goal, EPS, numNodes, validationDist);

    % Compose the new executed path: start→hit, obstacle-following, then new plan
    finalPathMatrix   = [pathMatrix(1:hitIdx,:); tangentBugPath; newPath.States];
    newPathStartingPoint = hitIdx + size(tangentBugPath,1) + 1;

    % Merge trees for plotting/reference (old valid nodes + new tree)
    finalSolutionInfo           = newSolutionInfo;
    finalSolutionInfo.TreeData  = [validNodes; newSolutionInfo.TreeData];
end

function [tangentBugPath, resumptionPoint] = runTangentBug2(map, startPos, goal)
    % TangentBug with "right end first, else left end, else emergency".
    stepSize = 0.25;
    robotPos = startPos;
    path = robotPos;
    resumptionPoint = [];
    isFollowingBoundary = false;

    mapLimits = [map.XWorldLimits; map.YWorldLimits];
    boundaryTolerance = stepSize * 2;
    occThresh = 0.2;               % free-space threshold

    hitPoint = startPos;
    maxBoundarySteps = 200;
    boundaryStepCount = 0;

    while norm(robotPos(1:2) - goal(1:2)) > stepSize
        % try greedy step toward goal
        dirToGoal = goal - robotPos;
        dirToGoal = dirToGoal / norm(dirToGoal);
        newPos = robotPos + stepSize * dirToGoal;

        if getOccupancy(map, newPos(1,1:2)) < occThresh
            % free progress
            if isFollowingBoundary
                % we've just found a place to resume toward goal
                resumptionPoint = robotPos;
                break;
            end
            robotPos = newPos;
            boundaryStepCount = 0;

        else
            % blocked: start/continue boundary following
            if ~isFollowingBoundary
                isFollowingBoundary = true;
                boundaryStepCount = 0;
                hitPoint = robotPos;           % remember where we hit
            end

            atBoundary = isAtBoundary(robotPos, mapLimits, boundaryTolerance);

            if atBoundary || boundaryStepCount > maxBoundarySteps
                % --- Bailout: go to obstacle RIGHT end; if bad, go LEFT end; else emergency ---
                [endRight, endLeft] = findObstacleEnds(map, hitPoint, stepSize, boundaryTolerance, occThresh);

                useRight = ~isempty(endRight) && ~isNearBoundaryPoint(endRight(1:2), mapLimits, boundaryTolerance);
                useLeft  = ~isempty(endLeft)  && ~isNearBoundaryPoint(endLeft(1:2),  mapLimits, boundaryTolerance);

                if useRight
                    robotPos = endRight;                 % jump to RIGHT end
                    path = [path; robotPos];
                    resumptionPoint = robotPos;
                    break;
                elseif useLeft
                    robotPos = endLeft;                  % jump to LEFT end
                    path = [path; robotPos];
                    resumptionPoint = robotPos;
                    break;
                else
                    % both ends near boundary → emergency local reseed
                    robotPos = emergencyNavigation(robotPos, goal, stepSize, map);
                    if isempty(robotPos), break; end
                    path = [path; robotPos];
                    resumptionPoint = robotPos;
                    break;
                end

            else
                % Regular boundary-following step: sample 16 directions, pick the one that
                % reduces distance to goal the most among "free enough" and in bounds.
                angles = linspace(0, 2*pi, 16);
                bestPos = robotPos;
                minDist = inf;
                for i = 1:numel(angles)
                    test2d = robotPos(1,1:2) + stepSize * [cos(angles(i)), sin(angles(i))];
                    testPos = [test2d, 0];
                    if isWithinBounds(testPos, mapLimits) && getOccupancy(map, test2d) < 0.65
                        d = norm(testPos - goal);
                        if d < minDist
                            minDist = d; bestPos = testPos;
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
    if isempty(resumptionPoint), resumptionPoint = robotPos; end
end

function atBoundary = isAtBoundary(pos, mapLimits, tol)
    atBoundary = (pos(1) <= mapLimits(1,1) + tol) || ...
                 (pos(1) >= mapLimits(1,2) - tol) || ...
                 (pos(2) <= mapLimits(2,1) + tol) || ...
                 (pos(2) >= mapLimits(2,2) - tol);
end

function withinBounds = isWithinBounds(pos, mapLimits)
    withinBounds = (pos(1) >= mapLimits(1,1)) && (pos(1) <= mapLimits(1,2)) && ...
                   (pos(2) >= mapLimits(2,1)) && (pos(2) <= mapLimits(2,2));
end

function tf = isNearBoundaryPoint(p, mapLimits, tol)
    tf = (p(1) <= mapLimits(1,1) + tol) || ...
         (p(1) >= mapLimits(1,2) - tol) || ...
         (p(2) <= mapLimits(2,1) + tol) || ...
         (p(2) >= mapLimits(2,2) - tol);
end







function side = getOppositeSide(currentPos, hitPoint)
    if currentPos(1) > hitPoint(1), side = -1; else, side = 1; end
end

function newPos = exploreOtherSide(hitPoint, side, stepSize, map, goal)
    searchAngles = linspace(0, 2*pi, 32);
    if side == 1
        searchAngles = [searchAngles(1:8), searchAngles(25:32), searchAngles(9:24)];
    else
        searchAngles = [searchAngles(9:24), searchAngles(1:8), searchAngles(25:32)];
    end
    for angle = searchAngles
        testPos = hitPoint(1:2) + stepSize * 2 * [cos(angle), sin(angle)];
        testPos = [testPos, 0];
        if getOccupancy(map, testPos(1,1:2)) < 0.2
            newPos = testPos; return;
        end
    end
    newPos = hitPoint;
end

function newPos = emergencyNavigation(currentPos, goal, stepSize, map)
    for radius = stepSize:stepSize:stepSize*4
        angles = linspace(0, 2*pi, 32);
        bestPos = []; minDist = inf;
        for angle = angles
            testPos = currentPos(1:2) + radius * [cos(angle), sin(angle)];
            testPos = [testPos, 0];
            if getOccupancy(map, testPos(1,1:2)) < 0.2
                d = norm(testPos - goal);
                if d < minDist, minDist = d; bestPos = testPos; end
            end
        end
        if ~isempty(bestPos), newPos = bestPos; return; end
    end
    newPos = [];
end

function [bridgePath, nearestIdx] = addYtoT(map, y, T, maxEdge, K, radius)
% ADDYTOT: Find nearest free node(s) in T via KD-tree and connect by
% "extending y towards x by δ until connected". Returns the Y→X mini-path.

    if nargin < 5 || isempty(K),      K = 15;   end
    if nargin < 6 || isempty(radius), radius = 4*maxEdge; end

    D = size(T,2);
    y  = y(:).'; y = [y, zeros(1, max(0, D-numel(y)))];

    % keep only free nodes
    occ = checkOccupancy(map, T(:,1:2));
    freeMask = occ < 0.65;
    P = T(freeMask, 1:2);
    idxMap = find(freeMask);

    bridgePath = [];
    nearestIdx = [];

    if isempty(P), return; end

    % KD-tree neighbors
    useKD = exist('createns','file') == 2;
    if useKD
        ns = createns(P, "NSMethod","kdtree", "Distance","euclidean");
        if ~isempty(radius) && radius > 0
            candCell = rangesearch(ns, y(1,1:2), radius);
            cand = candCell{1};
            if isempty(cand)
                cand = knnsearch(ns, y(1,1:2), "K", min(K, size(P,1)));
            end
        else
            cand = knnsearch(ns, y(1,1:2), "K", min(K, size(P,1)));
        end
        cand = cand(:).';
    else
        dists = vecnorm(P - y(1,1:2), 2, 2);
        [~, ord] = sort(dists, 'ascend');
        cand = ord(1:min(K,numel(ord)));
        if ~isempty(radius)
            cand = cand(dists(cand) <= radius);
            if isempty(cand), cand = ord(1:min(K,numel(ord))); end
        end
    end

    % Try candidates: extend y → x by δ until connected
    for c = cand
        x = T(idxMap(c), :);                   % neighbor (2D or 3D)
        % If straight line Y→X is already free, just subdivide
        if isEdgeFree(map, y(1,1:2), x(1,1:2), 0.25, 0.2)
            seg = connectPoints(y, x, maxEdge);      % y → x
            bridgePath = seg; nearestIdx = idxMap(c); return;
        else
            [seg, y_new, ok] = extendYtowardsX(map, x, y, maxEdge, 0.25, 0.2);
            if ok
                % seg is Y→…→X, success
                bridgePath = seg; nearestIdx = idxMap(c); return;
            end
        end
    end
end

function ok = allEdgesFree(map, path, step, freeThresh)
    if nargin < 3, step = 0.25; end
    if nargin < 4, freeThresh = 0.2; end
    ok = true;
    for i = 1:size(path,1)-1
        if ~isEdgeFree(map, path(i,1:2), path(i+1,1:2), step, freeThresh)
            ok = false; return;
        end
    end
end

function tf = isEdgeFree(map, a, b, step, thresh)
    if nargin < 4, step = 0.25; end
    if nargin < 5, thresh = 0.2; end
    L = norm(b - a);
    n = max(1, ceil(L / step));
    xs = linspace(a(1), b(1), n+1);
    ys = linspace(a(2), b(2), n+1);
    occ = getOccupancy(map, [xs(:), ys(:)]);
    tf = all(occ < thresh);
end

function waypoints = connectPoints(start_pt, end_pt, d)
% Straight-line chain from start_pt to end_pt with edges <= d
    start_pt = start_pt(:).'; end_pt = end_pt(:).';
    if numel(start_pt) ~= numel(end_pt)
        error('start_pt and end_pt must have the same dimension');
    end
    if d <= 0, error('Maximum edge length d must be positive'); end

    L = norm(end_pt(1:2) - start_pt(1:2));
    if L < eps
        waypoints = [start_pt; end_pt];
        return;
    end

    n = max(1, ceil(L / d));           % number of edges
    t = linspace(0, 1, n+1).';
    waypoints = (1 - t).*start_pt + t.*end_pt;
end

function [seg, y_new, success] = extendYtowardsX(map, x, y, delta, step, freeThresh)
% Incrementally extend y → x in hops of length delta until y→x line is clear.
    if nargin < 5, step = 0.25; end
    if nargin < 6, freeThresh = 0.2; end

    D = numel(y);
    x  = x(:).'; y = y(:).';
    if D < 3, x(3) = 0; y(3) = 0; end

    seg = y;
    y_new = y;
    success = false;

    if isEdgeFree(map, y(1:2), x(1:2), step, freeThresh)
        seg = connectPoints(y, x, delta); y_new = x; success = true; return;
    end

    maxIters = max(1, ceil(norm(x(1:2)-y(1:2))/delta));
    for k = 1:maxIters
        dir2d = x(1:2) - y_new(1:2);
        L = norm(dir2d);
        if L < delta
            cand2d = x(1:2);
        else
            cand2d = y_new(1:2) + delta * (dir2d / L);
        end
        cand = [cand2d, y_new(3)];

        % hop must be free
        if ~isEdgeFree(map, y_new(1:2), cand(1:2), step, freeThresh)
            return; % blocked → cannot extend further on this ray
        end

        seg   = [seg; cand];
        y_new = cand;

        % after hop, check direct visibility to x
        if isEdgeFree(map, y_new(1:2), x(1:2), step, freeThresh)
            tail = connectPoints(y_new, x, delta);
            seg  = [seg; tail(2:end,:)];    % append, avoid duplicate y_new
            y_new = x; success = true; return;
        end
    end
end
function side = chooseSideRightFirst(hitPoint, goal, mapLimits, boundaryTol)
% Return +1 for "right" or -1 for "left".
% Right-first, but if right would hug the boundary near the hitPoint,
% choose left.

    % Direction toward goal (as a proxy for local obstacle orientation)
    dir = (goal(1:2) - hitPoint(1:2));
    if norm(dir) < eps
        dir = [1 0]; % fallback
    else
        dir = dir / norm(dir);
    end
    
    % Right and left perpendiculars
    perpRight = [ -dir(2),  dir(1) ];  % +1
    perpLeft  = [  dir(2), -dir(1) ];  % -1
    
    % Probe a small offset from the hit point to see boundary proximity
    probeDist = 2 * 0.25;  % small, ~2*stepSize used in TB
    pRight = hitPoint(1:2) + probeDist * perpRight;
    pLeft  = hitPoint(1:2) + probeDist * perpLeft;
    
    nearRight = isNearBoundaryPoint(pRight, mapLimits, boundaryTol);
    nearLeft  = isNearBoundaryPoint(pLeft,  mapLimits, boundaryTol);
    
    if ~nearRight
        side = +1;   % prefer right
    elseif ~nearLeft
        side = -1;   % right is tight → use left
    else
        % both near boundary: still pick right (will fail fast and flip later)
        side = +1;
    end
end

function newPos = exploreOtherSideRF(hitPoint, side, stepSize, map, goal, mapLimits, boundaryTol)
% Move from the collision hitPoint along the chosen side (right:+1, left:-1).
% Keep stepping perpendicular to goal direction until we're no longer
% "hugging" the obstacle (no occupied cells within a small ring).
% Returns [] if no acceptable offset found within a budget.

    dir = (goal(1:2) - hitPoint(1:2));
    if norm(dir) < eps, dir = [1 0]; else, dir = dir / norm(dir); end
    
    perp = side * [ -dir(2), dir(1) ];   % +1 right, -1 left
    
    stride      = 2 * stepSize;          % step outward perpendicularly
    maxTravel   = 20 * stepSize;         % how far we'll search on this side
    ringRadius  = 1.5 * stepSize;        % "hugging" detection radius
    occThresh   = 0.2;                   % free threshold for occupancy
    nRingSamples = 16;                   % ring sampling for hugging test
    
    newPos = []; % default: fail
    
    for travel = stride:stride:maxTravel
        cand2d = hitPoint(1:2) + travel * perp;
        
        % Bounds & boundary safety
        if isNearBoundaryPoint(cand2d, mapLimits, boundaryTol), continue; end
        if ~isWithinBounds([cand2d, 0], mapLimits), continue; end
        
        % The candidate itself must be in free space
        if getOccupancy(map, cand2d) >= occThresh
            continue;
        end
        
        % Not "hugging" the obstacle? (ring of free samples around the point)
        if ~isHuggingObstacle(map, cand2d, ringRadius, nRingSamples, occThresh)
            newPos = [cand2d, 0];   % keep theta=0 for SE2 placeholder
            return;
        end
    end
    
    % If we get here, we couldn't find a good offset on this side.
    % Return [] to let caller try opposite side or emergency.
end
function tf = isHuggingObstacle(map, center, radius, nSamples, occThresh)
% True if ANY sample on a ring around 'center' is occupied; i.e., we're
% still too close to the obstacle boundary.

    ang = linspace(0, 2*pi, nSamples+1); ang(end) = [];
    ring = center + radius * [cos(ang(:)), sin(ang(:))];
    occ  = getOccupancy(map, ring);
    tf   = any(occ >= occThresh);
end
function [endRight, endLeft] = findObstacleEnds(map, hitPoint, stepSize, boundaryTol, occThresh)
% Identify the right and left ends of the obstacle intersecting the row of hitPoint.
% Returns world points just OUTSIDE the obstacle with a small clearance.
    endRight = [];
    endLeft  = [];

    M = occupancyMatrix(map);                 % probabilities in grid
    [nRows, nCols] = size(M);

    % grid index of the hit point (NOTE: world2grid returns a 1x2 vector)
    ij = world2grid(map, hitPoint(1,1:2));
    r = max(1, min(nRows, ij(1)));
    c = max(1, min(nCols, ij(2)));

    if all(M(r,:) < occThresh)
        return; % nothing occupied on this row
    end

    % If not currently inside the obstacle, slide to nearest occupied cell
    if M(r,c) < occThresh
        left  = find(M(r,1:c)   >= occThresh, 1, 'last');
        right = find(M(r,c:end) >= occThresh, 1, 'first');
        if isempty(left) && isempty(right), return; end
        if isempty(left)
            c = c + right - 1;
        elseif isempty(right)
            c = left;
        else
            if (c - left) <= (right - 1)
                c = left;
            else
                c = c + right - 1;
            end
        end
    end

    % Scan left and right along this row to find obstacle span
    cL = c;
    while cL >= 1     && M(r,cL) >= occThresh, cL = cL - 1; end
    cR = c;
    while cR <= nCols && M(r,cR) >= occThresh, cR = cR + 1; end
    % cL: first FREE col to the left of blob; cR: first FREE col to the right

    % How many free cells beyond the obstacle edge we desire
    clearanceCells = max(2, round(2*stepSize*map.Resolution));

    % LEFT end candidate column (further left from obstacle)
    colLeftTarget  = cL - clearanceCells;
    if colLeftTarget >= 1
        endLeft = pickFreeInColumn(map, M, colLeftTarget, r, occThresh);
    end

    % RIGHT end candidate column (further right from obstacle)
    colRightTarget = cR + clearanceCells;
    if colRightTarget <= nCols
        endRight = pickFreeInColumn(map, M, colRightTarget, r, occThresh);
    end
end
function pWorld = pickFreeInColumn(map, M, col, rowCenter, occThresh)
% Search up/down a few rows from rowCenter at fixed 'col' to find a free cell.
    [nRows, ~] = size(M);
    wiggle = 3;
    pWorld = [];

    for dr = 0:wiggle
        r1 = rowCenter - dr;
        r2 = rowCenter + dr;
        if r1 >= 1 && M(r1, col) < occThresh
            p = grid2world(map, [r1, col]);   % returns 1x2 [x y]
            pWorld = [p, 0]; 
            return;
        end
        if r2 <= nRows && M(r2, col) < occThresh
            p = grid2world(map, [r2, col]);
            pWorld = [p, 0];
            return;
        end
    end
end
