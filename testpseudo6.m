%% BuRRT: KD-tree Bridge + TangentBug — 10-trial batch runner
% Requires Robotics System Toolbox (occupancyMap, plannerRRTStar, etc.)

clear; clc;

% Number of trials and plotting toggle
trials    = 1;
showPlots = true;

% Run
results = run_burrt_trials(trials, showPlots);

%% ----------------------------- Functions -----------------------------

function results = run_burrt_trials(trials, showPlots)
% RUN_BURRT_TRIALS  Run the full pipeline multiple times and collect metrics.
%   results = run_burrt_trials(10, false)

    if nargin < 1, trials = 10; end
    if nargin < 2, showPlots = false; end

    % --- Parameters (match the original script) ---
    EPS = 1;
    numNodes = 100*10^6;
    validationDist = 0.01;
    start = [0.5 0.5 0];
    goal  = [99 99 0];

    % Unknown rectangular obstacles [x, y, h, w]
    % obstaclesList = [
    %     70 1  10 60
    %     50 30 10 70
    %     30 1  10 80
    % ];

   % obstaclesList =  [70 1 10 60
   %      20 40 10 60];

%%piano
    obstaclesList =   [49 5 50 5
          29 25 50 5
         19 50 50 5
         15 75 50 5
         10 90 50 5];
%% map7

   % obstaclesList =    [70 1 10 40 
   %    50 1 10 50 
   %     40 55 50 10
   %     30 65  50 10
   %    10 45  10 50
   %    10 86 40 10 ]; 
%% new map

% obstaclesList = [950 150  50 450
%         850 300  50 400
%         750 350  50 500
%         650 450  50 450
%         550 200  50 800
%          450 50  50 600
%          350 200 50 700
%         250 650  50 300
%          150 450 50 450];


   %%

    nObs = size(obstaclesList,1);

    % Preallocate results
    results(trials) = struct('trial',[],'seed',[],'time_s',[],'path_len',[], ...
                             'per_obstacle_success',[],'total_success',[]);

    for t = 1:trials
        seed = 1000 + t;   % distinct seed each trial

        % Fresh map each trial
        simpleMap = zeros(100,100);
        map = occupancyMap(simpleMap, 1);

        tStart = tic;

        % Initial plan (seeded)
        [initialPath, solutionInfo] = planPath(map, start, goal, EPS, numNodes, validationDist, seed);
        currentPathMatrix = initialPath.States;

        resumptionPoint   = start;
        newPathStartingPt = 1;

        % Optional plotting
        if showPlots
            figure; map.show; hold on;
            plot(solutionInfo.TreeData(:,1), solutionInfo.TreeData(:,2), 'g.-');
            plot(currentPathMatrix(:,1), currentPathMatrix(:,2), 'r-', 'LineWidth', 2);
            plot(start(1), start(2), 'go', 'MarkerSize',8,'MarkerFaceColor','g');
            plot(goal(1),  goal(2),  'ro', 'MarkerSize',8,'MarkerFaceColor','r');
            title("Original Path Without Obstacles");
        end

        % Per-obstacle success tracking (exact equality check)
        perObsSuccess = zeros(1, nObs);

        % Replanning loop through obstacles
        for i = 1:nObs
            x0 = obstaclesList(i,1);
            y0 = obstaclesList(i,2);
            h  = obstaclesList(i,3);
            w  = obstaclesList(i,4);

            simpleMap(x0:x0+h-1, y0:y0+w-1) = 1;
            map = occupancyMap(simpleMap, 1);

            % pass a slightly different seed to each replan
            seed_i = seed + i;

            [newPathMatrix, solutionInfo, collisionPoint, resumptionPoint, newPathStartingPt] = ...
                TangentBugBasedReplanning(map, currentPathMatrix, solutionInfo, goal, ...
                                          EPS, numNodes, validationDist, resumptionPoint, newPathStartingPt, seed_i);

            currentPathMatrix = newPathMatrix;

            % --- Success after THIS obstacle (EXACT equality on x,y) ---
            perObsSuccess(i) = double( isequal(currentPathMatrix(end,1:2), goal(1:2)) );

            if showPlots
                figure; map.show; hold on;
                plot(currentPathMatrix(:,1), currentPathMatrix(:,2), 'b-', 'LineWidth', 2);
                plot(start(1), start(2), 'go', 'MarkerSize',8,'MarkerFaceColor','g');
                plot(goal(1),  goal(2),  'ro', 'MarkerSize',8,'MarkerFaceColor','r');
                if ~isempty(collisionPoint)
                    plot(collisionPoint(1), collisionPoint(2), 'mx', 'MarkerSize', 10, 'LineWidth', 2);
                end
                if ~isempty(resumptionPoint)
                    plot(resumptionPoint(1), resumptionPoint(2), 'ms', 'MarkerSize', 10, 'LineWidth', 2);
                end
                title(sprintf("Updated Path After Obstacle %d (trial %d) — success=%d", i, t, perObsSuccess(i)));
            end
        end

        time_s = toc(tStart);

        % --- Metrics ---
        path_len = pathLength2D(currentPathMatrix);

        % Total success = 1 ONLY if success==1 after EVERY obstacle
        total_success = double(all(perObsSuccess == 1));

        results(t) = struct('trial',t,'seed',seed,'time_s',time_s, ...
                            'path_len',path_len,'per_obstacle_success',perObsSuccess, ...
                            'total_success',total_success);
    end

    % --- Summary ---
    T = struct2table(results);
    disp(T);

    % Overall total-success stats
    nTotalSucc = sum([results.total_success]);
    fprintf('\nTOTAL success (all obstacles succeeded): %d/%d (%.1f%%)\n', ...
            nTotalSucc, trials, 100*nTotalSucc/trials);

    % Per-obstacle success rates across trials
    perObsMat = vertcat(results.per_obstacle_success);
    perObsRates = mean(perObsMat, 1) * 100;
    for i = 1:size(perObsMat,2)
        fprintf('Obstacle %d success: %.1f%%\n', i, perObsRates(i));
    end

    fprintf('Path length  : mean %.3f, std %.3f\n', mean([results.path_len]), std([results.path_len]));
    fprintf('Time (s)     : mean %.3f, std %.3f\n', mean([results.time_s]),  std([results.time_s]));
end

function [path, solutionInfo, sv] = planPath(map, start, goal, EPS, numNodes, validationDist, seed)
% planPath  RRT* plan with optional RNG seed (for varied trials)
    if nargin >= 7 && ~isempty(seed)
        rng(seed,'twister');
    else
        rng('shuffle');
    end

    ss = stateSpaceSE2;
    sv = validatorOccupancyMap(ss); sv.Map = map;
    ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
    sv.ValidationDistance = validationDist;
    planner = plannerRRTStar(ss, sv, ...
        ContinueAfterGoalReached=true, MaxIterations=numNodes, ...
        MaxConnectionDistance=EPS, ...
        GoalReachedFcn=@(~,s,g)(norm(s(1:2)-g(1:2))<0.001), GoalBias=0.01);

    [path, solutionInfo] = plan(planner, start, goal);
end

function [finalPathMatrix, finalSolutionInfo, collisionPoint, resumptionPoint, newPathStartingPoint] = ...
    TangentBugBasedReplanning(map, pathMatrix, solutionInfo, goal, EPS, numNodes, validationDist, resumptionPoint, newPathStartingPoint, seed)
% Robust TangentBug + KD-bridge replanning using existing RRT* tree only.
% Uses shortestpathtree for efficient goal connection.

    % --- Remove invalid nodes for robustness ---
    if ~isempty(solutionInfo.TreeData)
        occupied   = checkOccupancy(map, solutionInfo.TreeData(:,1:2));
        validNodes = solutionInfo.TreeData(~occupied, :);
    else
        validNodes = [];
    end

    % --- Detect first collision along current path ---
    hitIdx = [];
    if ~isempty(pathMatrix) && newPathStartingPoint <= size(pathMatrix,1)
        XY = pathMatrix(newPathStartingPoint:end,1:2);
        if ~isempty(XY)
            occAlongPath = checkOccupancy(map, XY);
            hitIdx = find(occAlongPath, 1, 'first');
            if ~isempty(hitIdx)
                hitIdx = hitIdx + newPathStartingPoint - 1;
            end
        end
    end

    if isempty(hitIdx)
        % Nothing blocks the current path – keep it
        finalPathMatrix   = pathMatrix;
        finalSolutionInfo = solutionInfo;
        collisionPoint    = [];
        return;
    end

    collisionPoint = pathMatrix(hitIdx, :);

    % --- TangentBug to find resumption point ---
    [tangentBugPath, resumptionPoint] = runTangentBug2(map, collisionPoint, goal);

    % --- KD-bridge: connect resumption point y to tree T ---
    if ~isempty(solutionInfo.TreeData)
        T = solutionInfo.TreeData(:, 1:min(3, size(solutionInfo.TreeData,2)));
    else
        T = [];
    end
    maxEdge = EPS;
    [bridgePath, nearestIdx] = addYtoT(map, resumptionPoint, T, maxEdge, 15, 4*EPS);

    if ~isempty(bridgePath)
        toAppend = bridgePath(1:end-1, :);
        padCols  = max(0, size(solutionInfo.TreeData,2) - size(toAppend,2));
        if padCols > 0
            toAppend = [toAppend, zeros(size(toAppend,1), padCols)];
        end
        solutionInfo.TreeData = [solutionInfo.TreeData; toAppend];
    end

    % --- Build graph from existing tree (fast KNN fallback) ---
    Tfull = [];
    if ~isempty(solutionInfo.TreeData)
        Tfull = solutionInfo.TreeData(:, 1:min(3, size(solutionInfo.TreeData,2)));
    end

    % try parent edges first
    E = [];
    candParentFields = {'Parent','Parents','TreeParent','TreeParents','ParentIndices','ParentIndex'};
    for k = 1:numel(candParentFields)
        f = candParentFields{k};
        if isfield(solutionInfo, f)
            p = solutionInfo.(f)(:);
            idx = (1:numel(p))';
            E = [idx, p];
            break;
        end
    end
    if ~isempty(E)
        E = E(all(E > 0, 2), :);
    end

    % fallback: KNN graph
    if isempty(E) && ~isempty(Tfull)
        P2 = Tfull(:,1:2);
        occT  = checkOccupancy(map, P2);
        freeMaskT = occT < 0.65;
        idxFree   = find(freeMaskT);
        Pfree     = P2(freeMaskT,:);

        E = [];
        if ~isempty(Pfree)
            K = 15;
            useKD = exist('createns','file') == 2;

            if useKD
                ns = createns(Pfree, "NSMethod","kdtree", "Distance","euclidean");
                [Idx,~] = knnsearch(ns, Pfree, "K", K+1);
            else
                D = pdist2(Pfree, Pfree);
                [~, Idx] = sort(D,2,"ascend");
                Idx = Idx(:,1:K+1);
            end

            for i = 1:size(Idx,1)
                ai = Pfree(i,:);
                for j = 2:size(Idx,2) % skip self
                    nb = Idx(i,j);
                    if nb==0, continue; end
                    bj = Pfree(nb,:);
                    if isEdgeFree(map, ai, bj, 0.5, 0.2)
                        E(end+1,:) = [idxFree(i), idxFree(nb)]; %#ok<AGROW>
                    end
                end
            end
        end
    end

    % --- Restrict to free nodes ---
    spNodes = [];
    if ~isempty(Tfull) && ~isempty(E)
        occT  = checkOccupancy(map, Tfull(:,1:2));
        freeMaskT = occT < 0.65;
        idxFree   = find(freeMaskT);
        Tfree     = Tfull(freeMaskT, :);

        mapToFree = zeros(size(Tfull,1),1);
        mapToFree(idxFree) = 1:numel(idxFree);

        Ef = E(all(freeMaskT(E),2), :);
        Ef = [mapToFree(Ef(:,1)), mapToFree(Ef(:,2))];

        if ~isempty(Tfree) && ~isempty(Ef)
            [~, goalNearFree] = min(vecnorm(Tfree(:,1:2) - goal(1:2), 2, 2));
            attachFree = mapToFree(nearestIdx);
            if attachFree == 0
                [~, attachFree] = min(vecnorm(Tfree(:,1:2) - resumptionPoint(1,1:2), 2, 2));
            end

            % Build graph
            w = vecnorm(Tfree(Ef(:,1),1:2) - Tfree(Ef(:,2),1:2), 2, 2);
            G = graph(Ef(:,1), Ef(:,2), w);

            % Use shortestpathtree for efficiency
            Tsp = shortestpathtree(G, attachFree);
            try
                spIdx = shortestpath(Tsp, attachFree, goalNearFree);
            catch
                spIdx = [];
            end
            if ~isempty(spIdx)
                spNodes = Tfree(spIdx, :);
            end
        end
    end

    % --- Final path composition ---
    if ~isempty(bridgePath)
        bridgeCore = bridgePath(1:end-1, :);
    else
        bridgeCore = [];
    end

    finalPathMatrix = [ ...
        pathMatrix(1:hitIdx,:); ...
        tangentBugPath; ...
        bridgeCore; ...
        spNodes ...
    ];

    newPathStartingPoint = hitIdx + size(tangentBugPath,1) + size(bridgeCore,1) + 1;

    finalSolutionInfo          = solutionInfo;
    finalSolutionInfo.TreeData = [validNodes; solutionInfo.TreeData];
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
                % Regular boundary-following step: sample directions, pick best
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

    if nargin < 5 || isempty(K),      K = 30;   end
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
            [seg, y_new, ok] = extendYtowardsX(map, x, y, maxEdge, 0.25, 0.2); %#ok<ASGLU>
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

    dir = (goal(1:2) - hitPoint(1:2));
    if norm(dir) < eps
        dir = [1 0]; % fallback
    else
        dir = dir / norm(dir);
    end
    
    % Right and left perpendiculars
    perpRight = [ -dir(2),  dir(1) ];  % +1
    perpLeft  = [  dir(2), -dir(1) ];  % -1
    
    % Probe a small offset
    probeDist = 2 * 0.25;
    pRight = hitPoint(1:2) + probeDist * perpRight;
    pLeft  = hitPoint(1:2) + probeDist * perpLeft;
    
    nearRight = isNearBoundaryPoint(pRight, mapLimits, boundaryTol);
    nearLeft  = isNearBoundaryPoint(pLeft,  mapLimits, boundaryTol);
    
    if ~nearRight
        side = +1;   % prefer right
    elseif ~nearLeft
        side = -1;   % right is tight → use left
    else
        side = +1;   % both tight: still try right
    end
end

function newPos = exploreOtherSideRF(hitPoint, side, stepSize, map, goal, mapLimits, boundaryTol)
% Move from the collision hitPoint along the chosen side (right:+1, left:-1).
% Keep stepping perpendicular to goal direction until we're no longer
% "hugging" the obstacle.
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
        
        if isNearBoundaryPoint(cand2d, mapLimits, boundaryTol), continue; end
        if ~isWithinBounds([cand2d, 0], mapLimits), continue; end
        if getOccupancy(map, cand2d) >= occThresh, continue; end
        
        if ~isHuggingObstacle(map, cand2d, ringRadius, nRingSamples, occThresh)
            newPos = [cand2d, 0];
            return;
        end
    end
end

function tf = isHuggingObstacle(map, center, radius, nSamples, occThresh)
% True if ANY sample on a ring around 'center' is occupied.
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

    % grid index of the hit point
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

function L = pathLength2D(P)
    if isempty(P) || size(P,1) < 2
        L = NaN; return;
    end
    diffs = diff(P(:,1:2),1,1);
    L = sum(vecnorm(diffs,2,2));
end
