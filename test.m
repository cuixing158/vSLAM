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

%%
thetaRadians = rand(1,3);
thetaDegrees = rad2deg(thetaRadians);
myR1 = eul2rotm(thetaRadians,'XYZ');
myR2 = rotx(thetaDegrees(1))*roty(thetaDegrees(2))*rotz(thetaDegrees(3));
err = myR1-myR2

%%
cameraPose = rigid3d((myR*roty(90)*rotz(-90))',[0,0,0]);
figure;plotCamera('AbsolutePose',cameraPose)
axis on;axis equal
grid on


