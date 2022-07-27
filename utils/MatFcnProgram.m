function  myfun(timeStamp,egoVelocity,poseEgoInGlobalWorld,positionRotationEgoInGloableWorld,...
    imgFrontSurround,imgRearSurround,imgLeftSurround,imgRightSurround,...
    imageImgFrontWindshield,ultrasonicData)
% matlab function block使用，用于曾总Simulation 3D Camera 模块实时仿真输出存储，2022.7.10
coder.extrinsic("logSensorSubFcn");
logSensorSubFcn(timeStamp,egoVelocity,poseEgoInGlobalWorld,positionRotationEgoInGloableWorld,...
    imgFrontSurround,imgRearSurround,imgLeftSurround,imgRightSurround,...
    imageImgFrontWindshield,ultrasonicData)
end

function logSensorSubFcn(timeStamp,egoVelocity,poseEgoInGlobalWorld,positionRotationEgoInGloableWorld,...
    imgFrontSurround,imgRearSurround,imgLeftSurround,imgRightSurround,...
    imageImgFrontWindshield,ultrasonicData,...
    dstRoot,folderNames,writeBinFile)
% Brief: 用于simulink matlab function模块的调用函数，记录传感器数据，便于C++使用其数据
% Details:
%    子函数，undefined function时候，可以独立为m文件使用.
%    为加快simulink仿真速度，设置定步长求解器，采样时间设置为场景模块的采样时间的整数倍，这样可以保证生成的图像有连续性.
% 
% Syntax:  
%     logSensorSubFcn(timeStamp,egoVelocity,poseEgoInGlobalWorld,positionRotationEgoInGloableWorld,imgFrontSurround,imgRearSurround,imgLeftSurround,imgRightSurround,imageImgFrontWindshield,ultrasonicData)
% 
% Inputs:
%    timeStamp - [1,1] size,[double] type,Description
%    egoVelocity - [1,1] size,[double] type,Description
%    poseEgoInGlobalWorld - [1,3] size,[double] type,Description
%    positionRotationEgoInGloableWorld - [1,6] size,[double] type,Description
%    imgFrontSurround - [m,n] size,[None] type,Description
%    imgRearSurround - [m,n] size,[None] type,Description
%    imgLeftSurround - [m,n] size,[None] type,Description
%    imgRightSurround - [m,n] size,[None] type,Description
%    imageImgFrontWindshield - [M,N] size,[None] type,Description
%    ultrasonicData - [1,12] size,[double] type,Description
% 
% Outputs:
%    None
% 
% Example: 
%    None
% 
% See also: None

% Author:                          cuixingxing
% Email:                           xingxing.cui@long-horn.com
% Created:                         26-Jul-2022 13:43:21
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2022 long-horn.All Rights Reserved.
%
arguments
    % required
    timeStamp (1,1) double
    egoVelocity (1,1) double
    poseEgoInGlobalWorld (1,3) double
    positionRotationEgoInGloableWorld (1,6) double
    imgFrontSurround 
    imgRearSurround 
    imgLeftSurround 
    imgRightSurround
    imageImgFrontWindshield 
    ultrasonicData (1,12) double
    % optional
    dstRoot (1,:) char = "./"; % 默认输出到当前工作目录
    folderNames (1,:) string = ["imgFrontSurround","imgRearSurround",...
        "imgLeftSurround","imgRightSurround","imageimgFrontWindshield"];% 子文件夹名字,与simulink模块输出名字保持一致！
    writeBinFile (1,1) logical = false; % 默认不必要存bin文件，存储推荐用csv文件
end

% step1: initialize variables 
timeStamp = duration(seconds(timeStamp));
currTT = timetable(timeStamp,egoVelocity,poseEgoInGlobalWorld,...
positionRotationEgoInGloableWorld,ultrasonicData);% corresponding to C++ mat2BinStruct type
    
persistent numStep allTT allPaths
if isempty(numStep)
    numStep = 1;
    allPaths = fullfile(dstRoot,folderNames);
    for idx = 1:length(folderNames)
        currFolder = allPaths(idx);
        if ~isfolder(currFolder)
            mkdir(currFolder);
        end
    end
    allTT = currTT;
else
    allTT = [allTT;currTT];
end

% step2: write current images
imwrite(imgFrontSurround       ,allPaths(1)+"/"+num2str(numStep)+".png");
imwrite(imgRearSurround        ,allPaths(2)+"/"+num2str(numStep)+".png");
imwrite(imgLeftSurround        ,allPaths(3)+"/"+num2str(numStep)+".png");
imwrite(imgRightSurround       ,allPaths(4)+"/"+num2str(numStep)+".png");
imwrite(imageImgFrontWindshield,allPaths(5)+"/"+num2str(numStep)+".png");

% step3: write csv sensor data
writetimetable(allTT,dstRoot+"sensorData.csv");

% step4: write bin sensor data
% write binary file, reference: 2022.3 encodeBinLogFile function
% Not recommended to save as bin files, compared to csv files is not 
% intuitive, and will not be much more efficient, in addition to read in
% C++ also need to pay attention to the variable storage order, type, size.
if writeBinFile
    binFolder = dstRoot+"sensorBinFiles";
    if ~isfolder(binFolder)
        mkdir(binFolder)
    end
    binFullName = fullfile(binFolder,num2str(numStep)+".bin");% each bin file corresponds to an image file of the same name except that the suffix is different.
    
    fileID = fopen(binFullName,'w');% write actual size shape
    fwrite(fileID,seconds(currTT.timeStamp),'double');
    fwrite(fileID,currTT.egoVelocity,'double');
    fwrite(fileID,currTT.poseEgoInGlobalWorld,'double');
    fwrite(fileID,currTT.positionRotationEgoInGloableWorld,'double');
    fwrite(fileID,currTT.ultrasonicData,'double');
    fclose(fileID);
end

% step5: numStep plus one
numStep  = numStep+1;
end