%% 用于曾总UE里面仿真场景数据的解析

%% 自定义数据
simoutFile = "E:\AllDataAndModels\mysimOut.mat";
% dstRoot = "\\yunpan02\豪恩汽电\豪恩汽电研发中心\临时文件夹\simout\parkingLotImages";
dstRoot = "E:\AllDataAndModels\parkingLotImages";
xLim = [-45,30]; % 看仿真数据的实际范围估计
yLim = [20,55]; % 看仿真数据的实际范围
zLim = [0,7];
if ~isfolder(dstRoot)
    mkdir(dstRoot)
end

%% decode data
% images
alldata = load(simoutFile);
imgsArrds = arrayDatastore(alldata.simOut.images.Data,ReadSize=1, IterationDimension=4);
numId = 1;
s = dir(dstRoot);
while imgsArrds.hasdata()
    imgDataCell = read(imgsArrds);
    imgName = sprintf("%04d.png",numId);
    imwrite(imgDataCell{1},fullfile(dstRoot,imgName));
    numId= numId+1;
end
% locations and orientations
locationCamera = ts2timetable(alldata.simOut.location);
orientationCam = ts2timetable(alldata.simOut.orientation);
% locationVehicle = ts2timetable(alldata.simout.locationVehicle);
% orientationVehicle = ts2timetable(alldata.simout.orientationVehicle);
simData = synchronize(locationCamera,orientationCam);

% simData = decodeSimUE(simoutFile);
arrds = arrayDatastore(simData,ReadSize=1, OutputType="same");

%% show first image
firstData = read(arrds);
currCamLocation = firstData.Data_locationCamera;
% currVehLocation = firstData.locationVehicle;

MapPointsPlot = pcplayer(xLim, yLim, zLim,'MarkerSize', 5);
hold(MapPointsPlot.Axes,'on');

% Plot camera trajectory
trajectory = reshape(currCamLocation,[],3);
gTrajectory = plot3(MapPointsPlot.Axes, trajectory(:,1), trajectory(:,2), ...
    1, 'r.-', 'LineWidth', 2 , 'DisplayName', 'ground Truth trajectory');
refPath = [];

%% main loop
arrds.reset();
tic;
while hasdata(arrds)
    currData = read(arrds);
    currCamLocation = currData.Data_locationCamera;
    currOriCam = currData.Data_orientationCam;
%     currVehLocation = currData.locationVehicle;
%     currOriVehicle = currData.orientationVehicle;

    % update plot
    trajectory = reshape(currCamLocation,[],3);
    eular = reshape(currOriCam,[],3);
    refPath = [refPath;trajectory];
    rotMatrix = eul2rotm(eular,'XYZ');
    cameraR = rotMatrix*roty(90)*rotz(-90); % 注意顺序和通用旋转矩阵乘法format
    cameraAbsolutePose = rigid3d(cameraR',trajectory);
    if mod(size(refPath,1),100)==1
        plotCamera('AbsolutePose',cameraAbsolutePose,...
            'Parent', MapPointsPlot.Axes, 'Size', 1,...
            'Color',[0.2,0.7,0.3],'Opacity',0.5,'AxesVisible',true);
    end
    % Update the camera trajectory
    set(gTrajectory, 'XData', refPath(:,1), 'YData', ...
        refPath(:,2), 'ZData', refPath(:,3));
     drawnow limitrate
end
toc
legend(MapPointsPlot.Axes,'TextColor','white','Location','northeast');

%% write to csv
filename = 'simUE_eular.csv';
imds = imageDatastore(dstRoot);
level = wildcardPattern + filesep;
pat = asManyOfPattern(level);
simData.image = extractAfter(imds.Files,pat);
writetimetable(simData,fullfile(dstRoot,filename))

%% 另一种四元数保存方式
% filename = 'simUE_quaternion.csv';
% oriCam = squeeze(simData.orientationCamera);
% oriVehicle = squeeze(simData.orientationVehicle);
% q_cam = eul2quat(oriCam,'XYZ');
% q_vehicle = eul2quat(oriVehicle,'XYZ');
% simData.orientationCamera = flip(q_cam,2);
% simData.orientationVehicle = flip(q_vehicle,2);
% writetimetable(simData,fullfile(dstRoot,filename))

%% support functions
function simData = decodeSimUE(simoutFile)
% Brief: 从simulink-UE仿真中的结果数据解析出来
% Details:
%    从simulink-UE仿真场景图中的vehicle,camera图像和位姿数据解析出来，便于C环境使用
%
% Syntax:
%     simData = decodeSimUE(simoutFile)
%
% Inputs:
%    simoutFile - mat数据文件
%
% Outputs:
%    simData - [m,n] size,[double] type,Description
%
% Example:
%    None
%
% See also: None

% Author:                          cuixingxing
% Email:                           xingxing.cui@long-horn.com
% Created:                         24-Apr-2022 13:42:08
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2022 long-horn.All Rights Reserved.
%

data = load(simoutFile);
simData = data.out.simout;
image = ts2timetable(simData.image);
locationCamera = ts2timetable(simData.locationCamera);
orientationCam = ts2timetable(simData.orientationCamera);
locationVehicle = ts2timetable(simData.locationVehicle);
orientationVehicle = ts2timetable(simData.orientationVehicle);
simData = synchronize(image,locationCamera,orientationCam,locationVehicle,...
    orientationVehicle);
end
