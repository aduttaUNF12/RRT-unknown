clearvars
close all

%% Variables
EPS = 1;
numNodes = 10000;
validationDist = 0.001;
imageFolder = '/MATLAB Drive/RRTx/RRTStar/image'; % Update with your folder path
imageFiles = dir(fullfile(imageFolder, '*.png'));
len = length(imageFiles);

% Define new obstacles (6 in total)
newObstacles = {
    [700, 200, 200, 200];
    [500, 500, 150, 150];
    [300, 700, 250, 250];
   % [600, 100, 180, 180];
   % [800, 600, 200, 200];
   % [400, 400, 220, 220];
};

for i = 1:length(imageFiles)
    %% Create the map
    clearvars -except imageFiles imageFolder EPS numNodes validationDist i len newObstacles;
    img = imread(fullfile(imageFolder, imageFiles(i).name));
    
    % Convert to grayscale if the image is RGB
    if size(img, 3) == 3
        img = rgb2gray(img);
    end

    % Convert to binary and invert (black objects as occupied)
    bwImg = imbinarize(img);
    
    % Resize to high resolution
    simpleMap = imresize(bwImg, [1000, 1000], 'nearest'); % High resolution 
    map = occupancyMap(simpleMap, 10);

    %% Define Start & Goal
    start = [0.5 0.5 0];
    goal = [99 99 0];

    % Initial Path Planning
    [path, solutionInfo] = planPath(map, start, goal, EPS, numNodes, validationDist);

    %% Visualize the initial results
   
    map.show
    hold on
    plot(solutionInfo.TreeData(:,1), solutionInfo.TreeData(:,2), 'g.-')
    plot(path.States(:,1), path.States(:,2), 'r-', 'LineWidth', 2)
    title(['Initial Path: ', imageFiles(i).name]);

    %% Sequentially Add Obstacles & Replan
    for k = 1:length(newObstacles)
        obstacle = newObstacles{k};
        x = obstacle(1);
        y = obstacle(2);
        w = obstacle(3);
        h = obstacle(4);

        % Add the new obstacle
        newObstacle = ones(h, w);
        simpleMap(x:x+h-1, y:y+w-1) = newObstacle;

        % Recalculate affected segment
        mapsize = [size(simpleMap,1), size(simpleMap,2)];
        xcentroid = size(newObstacle,1) / 2;
        ycentroid = size(newObstacle,2) / 2;
        center = mapsize / 2;
        radius = max(xcentroid, ycentroid) + 200;
        rectCenter = [1000 - x + ycentroid, y + xcentroid];

        % Find the segment affected and replan only that part
        [replannedSegment, solutionInfo, si, sj, s, g] = ...
            replanPathWithNewObstacle(simpleMap, path, EPS, numNodes, validationDist, radius, rectCenter);

        % Merge the modified segment with the existing path
        newPath.States = [path.States(1:si, :); replannedSegment.States; path.States(sj:end, :)];

        % Update path for next iteration
        path = newPath;

        %% Plot the updated path
        figure;
        map = occupancyMap(simpleMap, 10);
        map.show
        hold on
        plot(solutionInfo.TreeData(:,1), solutionInfo.TreeData(:,2), 'g-')
        plot(path.States(:,1), path.States(:,2), 'b-', 'LineWidth', 2)
        plot(s(1), s(2), '--bs', 'LineWidth', 2, 'MarkerSize', 5, 'MarkerEdgeColor', 'c', 'MarkerFaceColor', [0.5, 0.5, 0.5])
        plot(g(1), g(2), '--bs', 'LineWidth', 2, 'MarkerSize', 5, 'MarkerEdgeColor', 'c', 'MarkerFaceColor', [0.5, 0.5, 0.5])
        title(['Path Replanned with Obstacle ', num2str(k)]);
    %   replanimageFolder ='/MATLAB Drive/RRTx/RRTStar/Replannedimages';
    % savePath = fullfile(replanimageFolder, ['Final_Replanned_',num2str(k), '.png']);
     %figureHandle = figure('Name', ['Replanned Map: ', imageFiles(i).name]); 
    % saveas(figureHandle, savePath);
     end

    %% Save final updated path image
    
end