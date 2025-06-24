function [path, solutionInfo, sv] = planPath(map, start, goal, EPS, numNodes, validationDist)
    ss = stateSpaceSE2;
    sv = validatorOccupancyMap(ss); sv.Map = map;
    ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
    sv.ValidationDistance = validationDist;
    planner = plannerRRTStar(ss, sv, ...
        ContinueAfterGoalReached=true, MaxIterations=numNodes, ...
        MaxConnectionDistance=EPS, GoalReachedFcn=@(~,s,g)(norm(s(1:2)-g(1:2))<0.001), GoalBias=0.01);
    rng(100,'twister');
    [path, solutionInfo] = plan(planner, start, goal);
end
