% nodes:    Contains list of all explored nodes. Each node contains its
%           coordinates, cost to reach and its parent.
% 
% Brief description of first use of RRT* algorithm: 
% 1. Pick a random node q_rand.
% 2. Find the closest node q_near from explored nodes to branch out from, towards
%    q_rand.
% 3. Steer from q_near towards q_rand: interpolate if node is too far away, reach
%    q_new. Check that obstacle is not hit.
% 4. Update cost of reaching q_new from q_near, treat it as Cmin. For now,
%    q_near acts as the parent node of q_new.
% 5. From the list of 'visited' nodes, check for nearest neighbors with a 
%    given radius, insert in a list q_nearest.
% 6. In all members of q_nearest, check if q_new can be reached from a
%    different parent node with cost lower than Cmin, and without colliding
%    with the obstacle. Select the node that results in the least cost and 
%    update the parent of q_new.
% 7. Add q_new to node list.
% 8. Continue until maximum number of nodes is reached or goal is hit.

clearvars
close all

%Creating graph and obstacle 
x_max = 100;
y_max = 100;
obstacle = [50,20,20,40];
EPS = 1;
numNodes = 1000;        

%Arbitrarly choosing start and goal nodes 
q_start.coord = [0 0];
q_start.cost = 0;
q_start.parent = 0;
q_goal.coord = [99 99];
q_goal.cost = 0;

%Creating figure for visualization 
% figure(1)
% axis([0 x_max 0 y_max])
% rectangle('Position',obstacle,'FaceColor',[0 .5 .5])
% hold on

%Call to RRT Function 
% Define state space and validator for your RRT planner
%bounds = [0, x_max; 0, y_max; -pi, pi]; %% orientation 360 degrees 
ss = stateSpaceSE2;
sv = validatorOccupancyMap(ss);

load exampleMaps.mat
map = occupancyMap(simpleMap,10);
sv.Map = map;

%stateSpace = stateSpaceSE2(bounds);   %% within bounds and does not collide, need 
ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
sv.ValidationDistance = 0.01;

% Create the RRT planner
planner = plannerRRTStar(ss,sv, ...
          ContinueAfterGoalReached=true, ...
          MaxIterations=2500, ...
          MaxConnectionDistance=0.3);


start = [0.5 0.5 0];
goal = [2.5 0.2 0]; 

rng(100,'twister') % repeatable result
[path, solutionInfo] = plan(planner, start, goal);

%% Visualize the results.

map.show
hold on
% Tree expansion
plot(solutionInfo.TreeData(:,1),solutionInfo.TreeData(:,2),'g.-')
% Draw path
plot(path.States(:,1),path.States(:,2),'r-','LineWidth',2)        
%disp(path);
