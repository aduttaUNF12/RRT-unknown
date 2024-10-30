function [path, solutionInfo] = planPath(map,start,goal, EPS, numNodes, validationDist)
% Define state space and validator for your RRT planner
ss = stateSpaceSE2;
sv = validatorOccupancyMap(ss);
sv.Map = map;
ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
sv.ValidationDistance = validationDist;

% Create the RRT planner
planner = plannerRRTStar(ss,sv, ...
          ContinueAfterGoalReached=true, ...
          MaxIterations=numNodes, ...
          MaxConnectionDistance=EPS);
rng(100,'twister') % repeatable result
[path, solutionInfo] = plan(planner, start, goal);
end
