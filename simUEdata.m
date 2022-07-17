%% 用于曾总UE里面仿真场景数据的解析,实际上是外参欧拉角形式

%% 自定义数据
simoutFile = "\\yunpan02\豪恩汽电\豪恩汽电研发中心\临时文件夹\simout\tranningParking.mat";
% dstRoot = "\\yunpan02\豪恩汽电\豪恩汽电研发中心\临时文件夹\simout\parkingLotImages";
dstRoot = "E:\AllDataAndModels\underParkingLotImages20220708";
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
parseSimMatImg(simoutFile,dstRoot,writeBinFile=true);
alldata = load(simoutFile);
imgsArrds = arrayDatastore(alldata.ans.image.Data,ReadSize=1, IterationDimension=4);
numId = 1;
s = dir(dstRoot);
while imgsArrds.hasdata()&&length(s)<3
    imgDataCell = read(imgsArrds);
    imgName = sprintf("%d.png",numId);
    imwrite(imgDataCell{1},fullfile(dstRoot,imgName));
    numId= numId+1;
end
% locations and orientations
locationCamera = ts2timetable(alldata.out.simout.locationCamera);
orientationCam = ts2timetable(alldata.out.simout.orientationCamera);
locationVehicle = ts2timetable(alldata.out.simout.locationVehicle);
orientationVehicle = ts2timetable(alldata.out.simout.orientationVehicle);
simData = synchronize(locationCamera,orientationCam,locationVehicle,orientationVehicle);

% adjust format,均为外参形式
simData.locationCamera = squeeze(simData.locationCamera);
simData.locationVehicle = squeeze(simData.locationVehicle);
oriCam = squeeze(simData.orientationCamera);% n*3
oriVehicle = squeeze(simData.orientationVehicle);% n*3
simData.orientationCamera = oriCam;
simData.orientationVehicle = oriVehicle;

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

function  parseSimMatImg(simoutFile,dstRoot,folderNames,writeBinFile)
% Brief: 根据simulink仿真输出的simoutFile解析为folderNames指定名字的图像文件
% Details:
%    simoutFile为simulink输出的mat文件，为后续鲁棒性可见，里面域名和层次应当每次
% 保持一致，域名数量与folderNames数量保持一一对应关系；解析的图像名字从1开始递增。
%
% Syntax:
%     flag = parseSimMatImg(simoutFile,dstRoot,folderNames)
%
% Inputs:
%    simoutFile - [1,1] size,[string] type, mat文件路径名字
%    dstRoot - [1,1] size,[string] type,要保持图像的根目录
%    folderNames - [1,n] size,[string] type,人为指定的存放各个视角的文件夹名字
%    writeBinFile - [1,1] size,[logical] type,是否存为bin文件
%
% Outputs:
%    None
%
% Example:
%    simoutFile = "yourSim.mat";
%    parseSimMatImg(simoutFile)
%
% See also: None

% Author:                          cuixingxing
% Email:                           xingxing.cui@long-horn.com
% Created:                         09-Jul-2022 09:51:32
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2022 long-horn.All Rights Reserved.
%
arguments
    simoutFile (1,:) char
    dstRoot (1,:) char = "./"; % 默认当前工作目录
    folderNames (1,:) string = ["imgFrontSurround","imgRearSurround",...
        "imgLeftSurround","imgRightSurround","imageimgFrontWindshield"];% 子文件夹名字,与simulink模块输出名字保持一致！
    writeBinFile (1,1) logical = false; % 默认不必要存bin文件，存储推荐用csv文件
end

% step1: make dstRoot and sub-folders
if ~isfolder(dstRoot)
    mkdir(dstRoot)
end

allPaths = fullfile(dstRoot,folderNames);
for idx = 1:length(folderNames)
    currFolder = allPaths(idx);
    if ~isfolder(currFolder)
        mkdir(currFolder);
    end
end

% step2: parsing temporary intermediate variables
alldata = load(simoutFile);
level1 = fieldnames(alldata);% 临时中间变量
assert(numel(level1)==1,"level1 is a temp node?(default is a temp)");

% step3: preprocess, convert all simulink data to timetable type
allData = structfun(@timeseries2timetable,alldata.(level1{1}),UniformOutput=false);
allDataTable = struct2array(allData);
mustBeMember(folderNames,allDataTable.Properties.VariableNames);% folderNames must be the same as the output names in simulink
imagesTable = allDataTable(:,folderNames);

% step4: write sensor data and images 
% write sensor data
numItems = height(allDataTable);
for idx = 1:length(folderNames)
    currFolderName = folderNames(idx);
    imgVarNames = currFolderName + "/"+ (1:numItems)' + ".png";
    allDataTable.(currFolderName) = imgVarNames; % replace image data with image names 
end
writetimetable(allDataTable,fullfile(dstRoot,"sensorData.csv")); % convenient to read data in c++

% write all images to each sub-folders
arrds = arrayDatastore(imagesTable,ReadSize=1, OutputType="same");
num = 1;
while arrds.hasdata()
    dataItem = read(arrds);% timetable type
    for idx = 1:length(folderNames)
        currFolderName = folderNames(idx);
        currSubImgName = allDataTable.(currFolderName)(num);
        imgFullName = fullfile(dstRoot,currSubImgName);
        if isfile(imgFullName) % avoid duplicate image writing
            continue;
        end
        imgData = squeeze(dataItem.(currFolderName));
        imwrite(imgData,imgFullName);
    end
    num = num+1;
end

% step5: write binary file, reference: 2022.3 encodeBinLogFile function
% Not recommended to save as bin files, compared to csv files is not 
% intuitive, and will not be much more efficient, in addition to read in
% C++ also need to pay attention to the variable storage order, type, size.
if writeBinFile 
    binFolder = fullfile(dstRoot,"sensorBinFiles");
    if ~isfolder(binFolder)
        mkdir(binFolder)
    end
    sensorDataTable = removevars(allDataTable,folderNames);% exclude all data other than images
    sensorVarNames = sensorDataTable.Properties.VariableNames;
    sensorDataDS = arrayDatastore(sensorDataTable,ReadSize=1,OutputType="same");
    num = 1;
    while sensorDataDS.hasdata()
        sensorItem = read(sensorDataDS);% timetable type
        binFullName = fullfile(binFolder,num2str(num)+".bin");% each bin file corresponds to an image file of the same name except that the suffix is different.
        fileID = fopen(binFullName,'w');
        for idx = 1:length(sensorVarNames)
            currVarName = sensorVarNames{idx};
            varData = squeeze(sensorItem.(currVarName));
            fwrite(fileID,varData,'double');% write actual size shape
        end
        fclose(fileID);
        num = num+1;
    end
end
end

