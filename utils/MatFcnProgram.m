function  myfun(tout,img,loc,ori)
% matlab function block使用，用于Simulation 3D Camera 模块实时仿真输出存储，2022.7.10
coder.extrinsic("logSensorSubFcn");
logSensorSubFcn(tout,img,loc,ori)
end

function logSensorSubFcn(tout,img,loc,ori)
% 子函数，undefined function时候，可以独立为m文件使用
% step1: parsing arguments
currRowTime = duration(seconds(tout));
currImg = img;
currLoc = loc;
currOri = ori;

% step2: write current image
dstRoot = "./"; % default is current work path
subFolder = "Images";
writeFolder = fullfile(dstRoot,subFolder);
if ~isfolder(writeFolder)
    mkdir(writeFolder);
end
currImgName = "time_"+num2str(tout)+".png";
imgFullName = fullfile(writeFolder,currImgName);
imwrite(currImg,imgFullName);

% step3: write current sensor data
currImg = subFolder+"/"+currImgName;% replace image data with image name
timeTab = timetable(currRowTime,currImg,currLoc,currOri);
sensorFullName = replace(imgFullName,".png",".csv");
writetimetable(timeTab,sensorFullName);
end