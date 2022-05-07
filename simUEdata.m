%% 用于曾总UE里面仿真场景数据的解析

%% 自定义数据
simoutFile = "\\yunpan02\豪恩汽电\豪恩汽电研发中心\临时文件夹\simout\20220426\simout20220505.mat";
% dstRoot = "\\yunpan02\豪恩汽电\豪恩汽电研发中心\临时文件夹\simout\parkingLotImages";
dstRoot = "E:\AllDataAndModels\underParkingLotImages20220505";
xLim = [-18,25];
yLim = [10,15];
zLim = [-0.5,2.5];
if ~isfolder(dstRoot)
    mkdir(dstRoot)
end

%% decode data
simData = decodeSimUE(simoutFile);
arrds = arrayDatastore(simData,ReadSize=1, OutputType="same");

%% show first image
firstData = read(arrds);
currImg = squeeze(firstData.image);
currCamLocation = firstData.locationCamera;
currVehLocation = firstData.locationVehicle;

% plot image
imgObj = imshow(currImg,Border="tight");
MapPointsPlot = pcplayer(xLim, yLim, zLim, ...
    'VerticalAxis', 'y', 'VerticalAxisDir', 'down', 'MarkerSize', 5);
hold(MapPointsPlot.Axes,'on');

% Plot camera trajectory
trajectory = reshape(currCamLocation,[],3);
gTrajectory = plot3(MapPointsPlot.Axes, trajectory(:,1), trajectory(:,2), ...
    1, 'r.-', 'LineWidth', 2 , 'DisplayName', 'ground Truth trajectory');
refPath = trajectory;
view(MapPointsPlot.Axes, [0 0 1]);

%% main loop
numId = 1;
arrds.reset();
tic;
while hasdata(arrds)
    currData = read(arrds);
    currImg = squeeze(currData.image);
    currCamLocation = currData.locationCamera;
    currVehLocation = currData.locationVehicle;
    imgName = sprintf("%04d.png",numId);
%     imwrite(currImg,fullfile(dstRoot,imgName));

    % update plot
    imgObj.CData = currImg;
    trajectory = reshape(currCamLocation,[],3);
    refPath = [refPath;trajectory];
    drawnow limitrate
    numId = numId+1;
end
toc
% Update the camera trajectory
set(gTrajectory, 'XData', refPath(:,1), 'YData', ...
    refPath(:,2), 'ZData', ones(size(refPath(:,1))));
legend(MapPointsPlot.Axes,'TextColor','white','Location','northeast');

%% write to csv
filename = 'simUE.csv';
imds = imageDatastore(dstRoot);
level = wildcardPattern + filesep;
pat = asManyOfPattern(level);
simData.image = extractAfter(imds.Files,pat);
writetimetable(simData,fullfile(dstRoot,filename))

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
locationVehicle = ts2timetable(simData.locationVehicle);
simData = synchronize(image,locationCamera,locationVehicle);
end
