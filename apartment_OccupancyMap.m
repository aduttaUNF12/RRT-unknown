clearvars
close all

img = imread('Apartment.png');

% Convert to grayscale if the image is RGB
if size(img, 3) == 3
    img = rgb2gray(img);
end

% Convert to binary and invert (black objects as occupied)
bwImg = imbinarize(img);

% Resize to high resolution
highResMap = imresize(bwImg, [1000, 1000], 'nearest'); % High resolution (e.g., 100x100)

% Create the binary occupancy map with a physical size of 10x10
apartmentMap = binaryOccupancyMap(highResMap, 100); % 10 cells per unit size
save("gridmaps.mat","apartmentMap");
% Display the final map
figure;
show(apartmentMap);
hold on;
