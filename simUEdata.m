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
flag = parseSimMatImg(simoutFile,dstRoot);
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

function flag = parseSimMatImg(simoutFile,dstRoot,folderNames)
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
%
% Outputs:
%    flag - [1,1] size,[bool] type,为true解析成功，false失败
%
% Example:
%    simoutFile = "yourSim.mat";
%    flag = parseSimMatImg(simoutFile)
%
% See also: None

% Author:                          cuixingxing
% Email:                           xingxing.cui@long-horn.com
% Created:                         08-Jul-2022 11:01:32
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
end

% make dstRoot
if ~isfolder(dstRoot)
    mkdir(dstRoot)
end

% parse fieldnames
alldata = load(simoutFile);
level1 = fieldnames(alldata);% 临时中间变量
assert(numel(level1)==1,"level1 is a temp node?(default is a temp)");

% preprocess
allData = structfun(@ts2timetable,alldata.(level1),UniformOutput=false);
allDataTable = struct2array(allData);
imagesTable = allDataTable(:,folderNames);
arrds = arrayDatastore(alldataTable,ReadSize=1, OutputType="same");
allPaths = fullfile(dstRoot,folderNames);
for idx = 1:length(folderNames)
    currFolder = allPaths(idx);
    if ~isfolder(currFolder)
        mkdir(currFolder);
    end
end

% write image in a loop
num = 1;
while arrds.hasdata()
dataItem = read(arrds);% timetable type
% 2022.7.8 待完成
end
end

function  flag = encodeBinLogFile(pathFound,binName,deltaX,deltaY,timeStamp,imgGrayIn,...
    xEgoInGloabalWorld,yEgoInGloabalWorld,thetaEgoInGloabalWorld,...
    direction,laneIn,UltrosonicData)
% 功能：用于调试使用log函数，记录输入的二进制bin信息。
% 输入：
%     pathFound: 取值{-1，0，1，2，3，...},-1为没找到规划路径
%     binName:字符向量，用于定义bin文件的名字
%     其他参数等同于曾总的pathPlanningWide入口总函数，略
% 输出：
%     flag：1*1 bool类型，成功记录就是1，否则为0
%
% Email:xingxing.cui@long-horn.com
% 2022.2.26 create this file
% 2022.2.27 加入RLE高效算法
% 2022.2.28 cuixingxing,1、修改pathFound不等于-1才记录bin文件；
%                       2、加入bin文件名字自定义，string或者character vector
%                       3、pathFound类型由double改为int8
%                       4、timeStamp类型由uint16改为uint32，以防止溢出
% 2022.3.1 文件名由pathLog改为encodeBinLogFile
% 2022.3.12 为防止bin文件写入格式错误，导致后期解析bin文件不对，输入做断言判断
%
assert(numel(deltaX)==1,'input image "deltaX" must 1*1 size');
assert(numel(deltaY)==1,'input image "deltaY" must 1*1 size');
assert(numel(timeStamp)==1,'input image "timeStamp" must 1*1 size');
assert(numel(xEgoInGloabalWorld)==1,'input image "xEgoInGloabalWorld" must 1*1 size');
assert(numel(yEgoInGloabalWorld)==1,'input image "yEgoInGloabalWorld" must 1*1 size');
assert(numel(thetaEgoInGloabalWorld)==1,'input image "thetaEgoInGloabalWorld" must 1*1 size');
assert(numel(direction)==1,'input image "direction" must 1*1 size');
assert(all(size(laneIn)==[1,10]),'input image "laneIn" must 1*10 size');
assert(all(size(UltrosonicData)==[1,12]),'input image "UltrosonicData" must 1*12 size');
assert(all(size(imgGrayIn)==[260,260]),'input image "imgGrayIn" must 260*260 size');

flag = 0;
persistent bufferData currentData
if isempty(currentData)
    currentData = struct('timeStamp',uint32(zeros(1)),...
        'egoDirection',int8(0),...
        'egoVelocity',0,...
        'deltaXY',[0 0],...
        'poseEgoInGloabalWorld',[0 0 0],...
        'laneIn',[0 0 0 0 0 0 0 0 0 0 ],...
        'UltrosonicData',[0 0 0 0 0 0 0 0 0 0 0 0],...
        'imgIn',uint32(zeros(1,0)));
    bufferData = [];
    coder.varsize('currentData.imgIn', [1,260*260+1], [0,1]);% https://www.mathworks.com/help/fixedpoint/ref/coder.varsize.html
end
currentData.timeStamp = uint32(timeStamp);
currentData.egoDirection = int8(direction);
currentData.deltaXY = [deltaX,deltaY];
currentData.poseEgoInGloabalWorld = ...
    [xEgoInGloabalWorld,yEgoInGloabalWorld,thetaEgoInGloabalWorld];
currentData.laneIn =laneIn;
currentData.UltrosonicData = UltrosonicData;
binaryImg = imbinarize(imgGrayIn);
currentData.imgIn = rleEncodingImage(binaryImg);

% 加入缓存
bufferData = [bufferData,currentData];
queueSize = 100;% 假设100个,理论存储大小约为260*260*100/8/1024=825Kb
if length(bufferData)>queueSize
    bufferData(1) =[];
end

% 生成bin文件
if pathFound~=int8(-1) % 没找到路径打log查看信息
    fileID = fopen(string(binName)+".bin",'w');
    numsData = uint16(length(bufferData));
    fwrite(fileID,numsData,'uint16');% 先写入缓存数据大小，便于加载使用
    for i = 1:numsData
        fwrite(fileID,bufferData(i).timeStamp,'uint32');% 1*1
        fwrite(fileID,bufferData(i).egoDirection,'int8');% 1*1
        fwrite(fileID,bufferData(i).egoVelocity,'double');% 1*1
        fwrite(fileID,bufferData(i).deltaXY,'double');% 1*2
        fwrite(fileID,bufferData(i).poseEgoInGloabalWorld,'double');% 1*3
        fwrite(fileID,bufferData(i).laneIn,'double');%  1*10
        fwrite(fileID,bufferData(i).UltrosonicData,'double');% 1*12
        fwrite(fileID,length(bufferData(i).imgIn),'double');% 1*1, 便于后续再读取使用
        fwrite(fileID,bufferData(i).imgIn,'uint32');% 1*n
        %         fwrite(fileID,bufferData(i).imgIn,'ubit1');% 260*260, https://ww2.mathworks.cn/matlabcentral/answers/285101-is-it-possible-to-create-a-binary-vector-then-write-it-to-a-file
    end
    fclose(fileID);
    flag =1;
end
end

