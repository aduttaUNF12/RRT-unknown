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
EPS = 2;
numNodes = 1000;        

%Arbitrarly choosing start and goal nodes 
q_start.coord = [0 0];
q_start.cost = 0;
q_start.parent = 0;
q_goal.coord = [99 99];
q_goal.cost = 0;

%Creating figure for visualization 
figure(1)
axis([0 x_max 0 y_max])
rectangle('Position',obstacle,'FaceColor',[0 .5 .5])
hold on

%Call to RRT Function 
% Define state space and validator for your RRT planner
bounds = [0, x_max; 0, y_max; -pi, pi];
stateSpace = stateSpaceSE2(bounds);  
stateVal = validatorOccupancyMap(stateSpace);

% Create the RRT planner
planner = plannerRRT(stateSpace, stateVal,'MaxConnectionDistance', EPS, 'MaxNumTreeNodes', numNodes);

start = [q_start.coord(1), q_start.coord(2)]; 
goal = [q_goal.coord(1), q_goal.coord(2)]; 

[path, solutionInfo] = plan(planner, start, goal);

 
%%  
%Creating new obstacle 
bounding_box = [10,20,10,10];
rectangle('Position',bounding_box,'FaceColor',[0 .5 .5] )

%creating final_path to add all final points to this array
pathT = {}; 
final_path = {};
pathT{end+1} = q_start.coord;
pathT = flip(pathT);

%Creating new graph and determining total number of nodes  
x_max1 = 90;
y_max1 = 90;
rand_max_x = 50;
rand_max_y = 50;
EPS1 = 1;
Area = x_max1*y_max1;
radii = 50; %radius of new area around bounding box
radius = 3.14*(radii^2); %area around bounding box
bb_area = bounding_box(3)*bounding_box(4); %area of bounding_box
A = radius-bb_area; %total area around bounding_box, subtracting area of bounding_box itself
original_density = EPS/(x_max*y_max); %density of nodes in original graph with just obstacle
numNodes1 = A*original_density; %number of Nodes for area around bounding_box

%Trying to do vectorization below
%prevPoints = [{pathT{end}}, pathT(1:end-1)];
%currentPoints = pathT;

%collide = cell(1, length(pathT));
%collide(2:n) = arrayfun(@(i) noCollision(currentPoints{i}, prevPoints{i}, bounding_box), 2:length(pathT), 'UniformOutput',false);

% collide = arrayfun(@noCollision,pathT);

collide  = {};
for i = 1:1:length(pathT)-1
  collide{end+1} = noCollision(pathT{i+1},pathT{i},bounding_box);
end

for i = 1:1:(length(collide)-1)
    if collide{i} == 0
       q_start1.coord = pathT{i-1};
       q_start1.cost = 0;
       q_start1.parent = 0;
       for j = i:1:length(collide)
           if collide{j} == 1
               q_goal1.coord = pathT{j};
               q_goal1.cost = 0;
               break
           end
       end
    else
        final_path{end+1} = pathT{i};
    end

end


for i = 1:1:length(collide) -1 
    if collide{i} == 0
        nodes(2) = q_start1;
        figure(1)
        axis([0 x_max 0 y_max])
        rectangle('Position',bounding_box,'FaceColor',[0 .5 .5] )
        rectangle('Position',obstacle,'FaceColor', [0 .5 .5])
        hold on 
        
        for i = 1:1:numNodes1
          q_rand1 = [floor(rand(1)*rand_max_x) floor(rand(1)*rand_max_y)];
          plot(q_rand1(1), q_rand1(2), 'x', 'Color',  [0 0.4470 0.7410])
            
          % Break if goal node is already reached
          for j = 1:1:length(nodes)
             if nodes(j).coord == q_goal1.coord
                  break
             end 
          end
                 
                
          ndist = [];
          for j = 1:1:length(nodes)
             n = nodes(j);
             tmp = dist(n.coord, q_rand1);
             ndist = [ndist tmp];
          end
          [val, idx] = min(ndist);
          q_near1 = nodes(idx);
        
        %% 
        
           q_new1.coord = steer(q_rand1, q_near1.coord, val, EPS1);        
           if noCollision(q_rand1, q_near1.coord, bounding_box)
              if noCollision(q_rand1,q_near1.coord,obstacle)
                line([q_near1.coord(1), q_new1.coord(1)], [q_near1.coord(2), q_new1.coord(2)], 'Color', 'k', 'LineWidth', 2);
                drawnow
                hold on
                q_new1.cost = dist(q_new1.coord, q_near1.coord) + q_near1.cost;
                % Within a radius of r, find all existing nodes
                q_nearest1 = [];
                r = 2;
                neighbor_count1 = 1;
                for j = 1:1:length(nodes)
                    if (noCollision(nodes(j).coord, q_new1.coord, bounding_box) == 1) && (noCollision(nodes(j).coord,q_new1.coord, obstacle)== 1) && (dist(nodes(j).coord, q_new1.coord) <= r)
                        q_nearest1(neighbor_count1).coord = nodes(j).coord;
                        q_nearest1(neighbor_count1).cost = nodes(j).cost;
                        neighbor_count1 = neighbor_count1+1;
                     end
                 end
        
                 q_min1 = q_near1;
                 C_min1 = q_new1.cost;
        
                 for k = 1:1:length(q_nearest1)
                     if noCollision(q_nearest1(k).coord, q_new1.coord, bounding_box) && noCollision(q_nearest1(k).coord, q_new1.coord, obstacle) && (q_nearest1(k).cost + dist(q_nearest1(k).coord, q_new1.coord) < C_min1)
                          q_min1 = q_nearest1(k);
                          C_min1 = q_nearest1(k).cost + dist(q_nearest1(k).coord, q_new1.coord);
                          line([q_min1.coord(1), q_new1.coord(1)], [q_min1.coord(2), q_new1.coord(2)], 'Color', 'g');                
                          hold on
                     end
                 end
        
                 for j = 1:1:length(nodes)
                     if nodes(j).coord == q_min1.coord
                        q_new1.parent = j;
                     end
                  end
        
                  nodes = [nodes q_new1];
              end
           end
        end 
                
        
            D = [];
            for j = 1:1:length(nodes)
                tmpdist = dist(nodes(j).coord, q_goal1.coord);
                D = [D tmpdist];
            end
            
            [val, idx] = min(D);
            q_final = nodes(idx);
            q_goal1.parent = idx;
            q_end1 = q_goal1;
            nodes = [nodes q_goal1];
            while q_end1.parent ~= 0
               start = q_end1.parent;
               line([q_end1.coord(1), nodes(start).coord(1)], [q_end1.coord(2), nodes(start).coord(2)], 'Color', 'b', 'LineWidth', 2)
               pathpoint1 = [q_end1.coord(1), q_end1.coord(2)];
               final_path{end+1} = pathpoint1;
               hold on
               q_end1 = nodes(start);
            end

    end
end
         
disp(final_path);
