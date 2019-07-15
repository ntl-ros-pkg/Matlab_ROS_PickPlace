%% Extract Training Images

%% Connect to ROS-Enabled System
rosshutdown;
rosinit '192.168.177.131';

%% Get spawned balls
gazebo = ExampleHelperGazeboCommunicator();
models = getSpawnedModels(gazebo);

%% Acquire images
imDirBase = 'trainingData';
dateTxt = datestr(now,'yyyymmdd_HHMM');
[~,~,~] = mkdir(imDirBase);

% Create subscribers
ptsub = rossubscriber('/camera/depth/points');
receive(ptsub);
 
spawnedBall1 = ExampleHelperGazeboSpawnedModel('tiny_cricket_ball_1',gazebo);
spawnedBall2 = ExampleHelperGazeboSpawnedModel('tiny_cricket_ball_2',gazebo);
spawnedBall3 = ExampleHelperGazeboSpawnedModel('tiny_cricket_ball_3',gazebo);

imDir = fullfile(imDirBase);

receive(ptsub); % dummy
orientation = [0,0,0];
numImages = 30;

for k = 1:numImages
    orientation(3) = 2*pi*k/numImages;
    setState(spawnedBall1,'orientation',orientation);
    setState(spawnedBall2,'orientation',orientation);
    setState(spawnedBall3,'orientation',orientation);
    pause(1);
    ptcloud = receive(ptsub);
    rgb = readRGB(ptcloud);
    I = permute(reshape(rgb,[640 480 3]),[2 1 3]);
    I = I(:,:,[3 2 1]); % workaround for Gazebo
    imwrite(I,fullfile(imDir,[dateTxt '_' num2str(k) '.png']));
end

%% Show acquired images
winopen(imDir);

%% End