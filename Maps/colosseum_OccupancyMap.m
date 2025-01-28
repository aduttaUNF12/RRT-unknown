clearvars
close all

img = imread('colosseum.png');

% Convert to grayscale if the image is RGB
if size(img, 3) == 3
    img = rgb2gray(img);
end

% Convert to binary and invert (black objects as occupied)
bwImg = imbinarize(img);

% Resize to high resolution
highResMap = imresize(bwImg, [1000, 1000], 'nearest'); % High resolution 

% Create the binary occupancy map with a physical size of 10x10
colosseumMap = binaryOccupancyMap(highResMap, 100); % 100 cells per unit size
save("gridmaps.mat","colosseumMap");
% Display the final map
figure;
show(colosseumMap);
hold on;
