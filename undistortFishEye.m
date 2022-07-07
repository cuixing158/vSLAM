%% 鱼眼相机图像去畸变,测试看能否成功3维重建，由于缺少参照物实际大小，尺度不易确定
% 去畸变后等价的相机内参会怎样变？ 待实验

%% 预处理数据
videoPath = "E:\long_horn_newTech\from_wjc\地下车库\NOR_20190101_000021.mp4";
cameraParaPath = "E:\long_horn_newTech\from_wjc\HJ6029-MIC-DIST.xlsx";
vi = VideoReader(videoPath);% fisheye image
cameraData = readmatrix(cameraParaPath,"Range","A40:E840");
cameraData(:,4) = [];% remove duplicate colum
C = getCommaList(cameraData);
cameraData = table(C{:},VariableNames=["angle","real_height","ref_height","distort"]);
focallength = mean(cameraData.ref_height./tand(cameraData.angle),"omitnan");% unit: milimeter
sensorratio = 0.003;% 0.003mm/pixel
angle_d = atand(cameraData.real_height./focallength);
cameraData = addvars(cameraData,angle_d,After='angle');
scalars = cameraData.ref_height./cameraData.real_height;
scalars(isnan(scalars))=1;
cameraData.scalars = scalars;
writetable(cameraData,"fisheye.csv");

%% 以像素为单位的内参
img = readFrame(vi);
fx = focallength/sensorratio;% unit: pixel
fy = fx;
cx = size(img,2)/2;
cy = size(img,1)/2;
K = [fx,0,cx;
    0,fy,cy;
    0,0,1];

%% 去畸变主函数
[outImg,mapX,mapY] = undistortImgFast(img,K,cameraData,"valid");
isSave = true;
if isSave
    K = [fx,0,size(outImg,2)/2;
        0,fy,size(outImg,1)/2;
        0,0,1];
    save('fisheyeCamParas.mat',"K","mapY","mapX");
end

currAxes = axes;
hImg = imshow(outImg,Parent=currAxes);
while hasFrame(vi)
    img = im2single(im2gray(readFrame(vi)));
    undistortImg = interp2(img,mapX,mapY,"linear",0);% 以2维插值代替像素索引映射操作
    hImg.CData = undistortImg;
    drawnow limitrate;
end



function [undistortImg,mapX,mapY] = undistortImgFast(oriImg,K,cameraData,outputView)
% Brief: 超广角（鱼眼）图像去畸变
% Details:
%    数据表仅需入射角，出射角(或real_height)，焦距即可，此函数支持C代码生成，快速高效
%
% Syntax:
%     [undistortImg,mapX,mapY] = undistortImgFast(oriImg,K,cameraData)
%
% Inputs:
%    oriImg - [m,n] size,[any] type,Description
%    K - [m,n] size,[double] type,Description
%    cameraData - [m,n] size,[table] type,Description
%
% Outputs:
%    undistortImg - [m,n] size,[double] type,Description
%    mapX - [m,n] size,[double] type,Description
%    mapY - [m,n] size,[double] type,Description
%
% Example:
%    None
%
% See also: None

% Author:                          cuixingxing
% Email:                           xingxing.cui@long-horn.com
% Created:                         24-Jun-2022 14:02:23
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2022 long-horn.All Rights Reserved.
%
arguments
    oriImg
    K (3,3) double
    cameraData table
    outputView (1,:) char {mustBeMember(outputView,{'same','valid','full'})} = 'same'
end
cx = K(1,3);
cy = K(2,3);
flength = K(1);
refHeight = flength*tand(cameraData.angle);
realHeight = flength*tand(cameraData.angle_d);
oriImg = im2double(im2gray(oriImg));
[h,w] = size(oriImg);

% 获取矫正图像宽和高
if strcmp(outputView,"same")
    H = h;
    W = w;
elseif strcmp(outputView,"valid")
    tmRelPt = [w/2,0]-[cx,cy];
    lmRelPt = [0,h/2]-[cx,cy];
    edgeRelPts = [tmRelPt;lmRelPt];
    distortL = vecnorm(edgeRelPts,2,2);
    undistortL = interp1(realHeight,refHeight,distortL,"linear");
    undistortL(isnan(undistortL)) = refHeight(end);
    conersX = edgeRelPts(:,1).*undistortL./distortL;
    conersY = edgeRelPts(:,2).*undistortL./distortL;
    W = 2*abs(diff(conersX));
    H = 2*abs(diff(conersY));
else % "full"
    conerRelPt = [w,h]-[cx,cy];
    distortL = vecnorm(conerRelPt,2,2);
    undistortL = interp1(realHeight,refHeight,distortL,"linear");
    undistortL(isnan(undistortL)) = refHeight(end);
    conerX = conerRelPt(1).*undistortL./distortL;
    conerY = conerRelPt(2).*undistortL./distortL;
    W = 2*conerX;
    H = 2*conerY;
end

% 鱼眼图像去畸变
[X,Y] = meshgrid(1:W,1:H);
undistortD = sqrt((X-W/2).^2+(Y-H/2).^2);
distortD = interp1(refHeight,realHeight,undistortD,"linear");% 以1维插值代替查表操作
distortD(isnan(distortD)) = realHeight(end);
mapX = (X-W/2).*distortD./undistortD+w/2;
mapY = (Y-H/2).*distortD./undistortD+h/2;
undistortImg = interp2(oriImg,mapX,mapY,"linear",0);% 以2维插值代替像素索引映射操作
end

function C = getCommaList(numercalMatrix)
% 把数值矩阵的每一列转换为逗号表达式
numcols = size(numercalMatrix,2);
C = cell(1,numcols);
for i = 1:numcols
    C{i} = numercalMatrix(:,i);
end
end
