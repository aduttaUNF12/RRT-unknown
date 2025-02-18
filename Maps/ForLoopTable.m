objects = [
    
];

% Number of objects
numObjects = length(objects);

% Preallocate arrays for table
outTime = zeros(numObjects, 1);
outPathLength = zeros(numObjects, 1);
xTime = zeros(numObjects, 1);
xPathLength = zeros(numObjects, 1);
mapTitles = strings(numObjects, 1);

% Load object data into arrays
for i = 1:numObjects
    outTime(i) = objects(i).outTime;
    outPathLength(i) = objects(i).outPathLength;
    xTime(i) = objects(i).xTime;
    xPathLength(i) = objects(i).xPathLength;
    mapTitles(i) = sprintf('Map %d', i); % Generate map title
end

% Create table
T = table(mapTitles, outTime, outPathLength, xTime, xPathLength, ...
    'VariableNames', {'Map', 'Our_Time', 'Our_PathLength', 'X_Time', 'X_PathLength'});

% Display table
disp(T);