%% 用于曾总UE里面仿真场景数据的解析,实际上是外参欧拉角形式

%% 自定义数据
simoutFile = "\\yunpan02\豪恩汽电\豪恩汽电研发中心\临时文件夹\simout\20220606\simout100mNormalLight20220606.mat";
% dstRoot = "\\yunpan02\豪恩汽电\豪恩汽电研发中心\临时文件夹\simout\parkingLotImages";
dstRoot = "E:\AllDataAndModels\underParkingLotImages20220606";
xLim = [-25,30]; % 看仿真数据的实际范围估计
yLim = [-50,15]; % 看仿真数据的实际范围
zLim = [0,7];
if ~isfolder(dstRoot)
    mkdir(dstRoot)
end
% 相机系数
calibration_time = "06/13/22 14:12:21";
image_width = 1920;
image_height = 1080;
cameraIntrinsics = [1046,0,image_width/2-1;
    0,1046,image_height/2-1;
    0,0,1];
k1 = 0;k2 = 0;p1=0;p2 = 0;% opencv 格式

%% decode data
% images
alldata = load(simoutFile);
imgsArrds = arrayDatastore(alldata.out.simout.image.Data,ReadSize=1, IterationDimension=4);
numId = 1;
s = dir(dstRoot);
while imgsArrds.hasdata()&&length(s)<3
    imgDataCell = read(imgsArrds);
    imgName = sprintf("%04d.png",numId);
    imwrite(imgDataCell{1},fullfile(dstRoot,imgName));
    numId= numId+1;
end
% locations and orientations
locationCamera = ts2timetable(alldata.out.simout.locationCamera);
orientationCam = ts2timetable(alldata.out.simout.orientationCamera);
locationVehicle = ts2timetable(alldata.out.simout.locationVehicle);
orientationVehicle = ts2timetable(alldata.out.simout.orientationVehicle);
simData = synchronize(locationCamera,orientationCam,locationVehicle,orientationVehicle);

% adjust format
simData.locationCamera = squeeze(simData.locationCamera);
simData.locationVehicle = squeeze(simData.locationVehicle);
oriCam = squeeze(simData.orientationCamera);% n*3
oriVehicle = squeeze(simData.orientationVehicle);% n*3
simData.orientationCamera = oriCam;
simData.orientationVehicle = oriVehicle;
% simData.orientationCamera(:,2) = -oriCam(:,2);% 注意数据有"问题"，可能是UE软件左手坐标系导致
% simData.orientationVehicle(:,2) = -oriVehicle(:,2);% 注意数据有"问题"，可能是UE软件左手坐标系导致

% simData = decodeSimUE(simoutFile);
arrds = arrayDatastore(simData,ReadSize=1, OutputType="same");

%% show first image
firstData = read(arrds);
currCamLocation = firstData.locationCamera;
% currVehLocation = firstData.locationVehicle;

MapPointsPlot = pcplayer(xLim, yLim, zLim,'MarkerSize', 5);
hold(MapPointsPlot.Axes,'on');

% Plot camera trajectory
trajectory = reshape(currCamLocation,[],3);
gTrajectory = plot3(MapPointsPlot.Axes, trajectory(:,1), trajectory(:,2), ...
    1, 'r.-', 'LineWidth', 2 , 'DisplayName', 'ground Truth trajectory');


%% main loop
refPath = [];
cameraExtrinsics = [];% numImgs*16,齐次矩阵按行展开形式
image_count = 0;
arrds.reset();
tic;
while hasdata(arrds)
    currData = read(arrds);
    currCamLocation = currData.locationCamera;
    currOriCam = currData.orientationCamera;
%     currVehLocation = currData.locationVehicle;
%     currOriVehicle = currData.orientationVehicle;

    % update plot
    trajectory = reshape(currCamLocation,[],3);
    eular = reshape(currOriCam,[],3);
    eular = double(rad2deg(eular));
    refPath = [refPath;trajectory];
    rotMatrix = rotz(eular(3))*roty(eular(2))*rotx(eular(1));
    cameraR = rotMatrix*roty(90)*rotz(-90); % 注意顺序和通用旋转矩阵乘法format
    cameraRT = [cameraR,trajectory';0,0,0,1]; % 通用齐次形式
    cameraRT = cameraRT';
    cameraOneColum = cameraRT(:);% 16*1
    cameraExtrinsics = [cameraExtrinsics;cameraOneColum'];
    cameraAbsolutePose = rigid3d(cameraRT);
    if mod(size(refPath,1),100)==1
        plotCamera('AbsolutePose',cameraAbsolutePose,...
            'Parent', MapPointsPlot.Axes, 'Size', 1.2,...
            'Color',[0.2,0.7,0.3],'Opacity',0.3,'AxesVisible',true);
    end
    % Update the camera trajectory
    set(gTrajectory, 'XData', refPath(:,1), 'YData', ...
        refPath(:,2), 'ZData', refPath(:,3));
     drawnow limitrate
     image_count = image_count+1;
end
toc
allDistances = sum(vecnorm(diff(simData.locationCamera),2,2));
aveSpeed = allDistances/seconds(simData.Time(end));
titleStr = sprintf("Distance:%.2f m,aveSpeed:%.2f m/s",allDistances,aveSpeed);
title(MapPointsPlot.Axes,titleStr);
legend(MapPointsPlot.Axes,'TextColor','white','Location','northeast');

%% write to csv
filename = 'simUE_eular.csv';
imds = imageDatastore(dstRoot);
level = wildcardPattern + filesep;
pat = asManyOfPattern(level);
simData.image = extractAfter(imds.Files,pat);
simData = movevars(simData,"image","Before","locationCamera");
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

%% 齐次矩阵，以yml格式保存
% 先导出csv，再opencv c++读取保存为yml格式
writematrix(cameraExtrinsics,'cameraExtrinsics.csv');

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
