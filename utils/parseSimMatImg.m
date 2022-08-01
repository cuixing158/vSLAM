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

% step1: make folders
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

