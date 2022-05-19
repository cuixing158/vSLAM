%% 使用曾总的图像做vSLAM
parkingLotRoot = "E:\AllDataAndModels\underParkingLotImages20220505";%"H:\dataSets\vSLAM\parkingLotImages";%"E:\AllDataAndModels\parkingLotImages";%
validIndexStart = 4;
imds = imageDatastore(parkingLotRoot);
imds = subset(imds,validIndexStart:length(imds.Files));
gTruthData = readtable(fullfile(parkingLotRoot,"simUE.csv"));
gTruthData = gTruthData(validIndexStart:end,:);
gTruth = helperGetSensorGroundTruth(gTruthData);

%% 我们摄像头停车场视频
% videoReader = VideoReader('E:\AllDataAndModels\videos20220519\WIN_20220519_11_16_10_Pro.mp4');
% videoPlayer = vision.VideoPlayer;
% while hasFrame(videoReader)
%    frame = readFrame(videoReader);
%    frame = imrotate(frame,180);
%    step(videoPlayer,frame);
% end
% videoPlayer.release()

%% Camera intrinsics
% focalLength    = [467.421, 467.3539];  % specified in units of pixels，自己标定的，暂时标定参数与实际采集视频不匹配，有问题再弄
% principalPoint = [ 320.7214,180.4589];  % in pixels [x, y]
% imageSize      = [1080, 1920]; % in pixels [mrows, ncols]

focalLength    = [1046, 1046];  % specified in units of pixels，曾总提供的
principalPoint = [ 1920/2,1080/2];  % in pixels [x, y]
imageSize      = [1080, 1920]; % in pixels [mrows, ncols]

% focalLength    = [700, 700];  % specified in units of pixels,demo 自带的
% principalPoint = [ 600,180];  % in pixels [x, y]
% imageSize      = [370, 1230]; % in pixels [mrows, ncols]
intrinsics     = cameraIntrinsics(focalLength, principalPoint, imageSize);

[mapPlot, optimizedPoses,addedFramesIdx] = helperVisualSLAM(imds, intrinsics); 

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
    gTruth(i).Rotation = [cos(yaw), sin(yaw), 0; ... % 变换使用newPt = oldPt*rotation;
        -sin(yaw), cos(yaw), 0; ...
        0, 0, 1];
end
end

