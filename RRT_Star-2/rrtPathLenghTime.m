close all;

% Initialize execution time and path length storage for 5 runs
executionTimes = zeros(1, 5);
pathLengths = zeros(1, 5);  % Store path lengths for each run

for run = 1:5  % Run 5 times
    % Start timing for this run
    

    % Load and process the map
    img = imread('map5.png');

    % Convert to grayscale if the image is RGB
    if size(img, 3) == 3
        img = rgb2gray(img);
    end

    % Convert to binary and invert (black objects as occupied)
    bwImg = imbinarize(img);
    bwImg = ~bwImg;

    % Resize to high resolution
    highResMap = imresize(bwImg, [1000, 1000], 'nearest'); % High resolution 

    % Create the binary occupancy map with a physical size of 10x10
    map = binaryOccupancyMap(highResMap, 100); % 100 cells per unit size
    save("map.mat", "map");

    % Display the final map
    figure;
    show(map);
    hold on;
    tic;
    % Define parameters
    EPS = 0.3;
    numNodes = 2500;  
    validationDist = 0.01;

    % Define start and goal positions & call RRT* function
    start = [0.5 0.5 0];
    goal = [9.5 9.2 0];

    % Call the path planning function
    [path, solutionInfo] = planPath(map, start, goal, EPS, numNodes, validationDist);

    % Visualize the results
    plot(solutionInfo.TreeData(:,1), solutionInfo.TreeData(:,2), 'g.-'); % Tree expansion
    plot(path.States(:,1), path.States(:,2), 'r-', 'LineWidth', 2);  % Path

    % Store the execution time for this run
    executionTimes(run) = toc;  % End timing for this run

    % Calculate the path length for this run
    pathLengths(run) = calculatePathLength(path);  % Store path length
end

% Calculate average and standard deviation of execution times
avgTime = mean(executionTimes);
stdTime = std(executionTimes);

% Calculate average and standard deviation of path lengths
avgPathLength = mean(pathLengths);
stdPathLength = std(pathLengths);

% Display the results
disp(['Average Execution Time (seconds): ', num2str(avgTime)]);
disp(['Standard Deviation of Execution Time (seconds): ', num2str(stdTime)]);
disp(['Average Path Length (meters): ', num2str(avgPathLength)]);
disp(['Standard Deviation of Path Length (meters): ', num2str(stdPathLength)]);

% Function to calculate the path length
function length = calculatePathLength(path)
    length = 0;
    for k = 1:size(path.States, 1)-1
        % Euclidean distance between consecutive points
        length = length + norm(path.States(k+1, 1:2) - path.States(k, 1:2));
    end
end

% RRT* Path Planning Function
function [path, solutionInfo] = planPath(map, start, goal, EPS, numNodes, validationDist)
    % Define state space and validator for your RRT planner
    ss = stateSpaceSE2;
    sv = validatorOccupancyMap(ss);
    sv.Map = map;
    ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
    sv.ValidationDistance = validationDist;

    % Create the RRT planner
    planner = plannerRRTStar(ss, sv, ...
          ContinueAfterGoalReached = true, ...
          MaxIterations = numNodes, ...
          MaxConnectionDistance = EPS);
    rng(100, 'twister'); % Repeatable result
    [path, solutionInfo] = plan(planner, start, goal);
end
