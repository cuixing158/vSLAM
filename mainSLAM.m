%% 使用曾总的图像做vSLAM
parkingLotRoot = "E:\AllDataAndModels\underParkingLotImages20220505";%"E:\AllDataAndModels\parkingLotImages";%
validIndexStart = 4;
imds = imageDatastore(parkingLotRoot);
imds = subset(imds,validIndexStart:length(imds.Files));
gTruthData = readtable(fullfile(parkingLotRoot,"simUE.csv"));
gTruthData = gTruthData(validIndexStart:end,:);
gTruth = helperGetSensorGroundTruth(gTruthData);

% Camera intrinsics
focalLength    = [1046, 1046];  % specified in units of pixels
principalPoint = [ 1920/2,1080/2];  % in pixels [x, y]
imageSize      = [1080, 1920]; % in pixels [mrows, ncols]
% focalLength    = [700, 700];  % specified in units of pixels
% principalPoint = [ 600,180];  % in pixels [x, y]
% imageSize      = [370, 1230]; % in pixels [mrows, ncols]
intrinsics     = cameraIntrinsics(focalLength, principalPoint, imageSize);

[mapPlot, optimizedPoses, addedFramesIdx] = helperVisualSLAM(imds, intrinsics); 

% Plot the camera ground truth trajectory

scaledTrajectory = plotActualTrajectory(mapPlot, gTruth(addedFramesIdx), optimizedPoses);

% Show legend
showLegend(mapPlot);


function gTruth = helperGetSensorGroundTruth(gTruthData)
gTruth = repmat(rigid3d, height(gTruthData), 1);
for i = 1:height(gTruthData) 
    currLocations = gTruthData{i,3:5};
    gTruth(i).Translation = [currLocations(1:2),0];
    % Ignore the roll and the pitch rotations since the ground is flat
    yaw = currLocations(3);
    gTruth(i).Rotation = [cos(yaw), sin(yaw), 0; ...
        -sin(yaw), cos(yaw), 0; ...
        0, 0, 1];
end
end

