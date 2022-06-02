%%  计算外参矩阵
cameraPositions = readmatrix('E:\AllDataAndModels\underParkingLotImages20220524\simUE_eular.csv',...
    'Range','C2:E407');
vehiclePositions = readmatrix('E:\AllDataAndModels\underParkingLotImages20220524\simUE_eular.csv',...
    'Range','I2:K407');
dis = vecnorm((cameraPositions-vehiclePositions),2,2);
[tform,inlierIndex,status]  = estimateGeometricTransform3D(cameraPositions,vehiclePositions,"rigid");



%% 郭工左手坐标系的转换矩阵
data = readmatrix('E:\AllDataAndModels\underParkingLotImages20220524\simUE_eular.csv',...
    'Range','C2:E407');
T1 = [1,0,0;0,1,0;0,0,-1];
Tz = @(x)[cos(x),-sin(x),0;sin(x),cos(x),0;0,0,1];
Tx = @(x)[1,0,0;0,cos(x),sin(x);0,-sin(x),cos(x)];
Ty = @(x)[cos(x),0,-sin(x);0,1,0;sin(x),0,cos(x)];
new_data = T1*Tz(pi/2)*Tx(pi/2)*data';

p1 = [1;1;1];
P1 = T1*Tz(pi/2)*Tx(pi/2)*p1

<<<<<<< HEAD




=======
>>>>>>> 570ee2e123138efa8375dad8cdc8153cdfc1881b
