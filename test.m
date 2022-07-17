%%  计算外参矩阵
cameraPositions = readmatrix('E:\AllDataAndModels\underParkingLotImages20220524\simUE_eular.csv',...
    'Range','C2:E407');
vehiclePositions = readmatrix('E:\AllDataAndModels\underParkingLotImages20220524\simUE_eular.csv',...
    'Range','I2:K407');
dis = vecnorm((cameraPositions-vehiclePositions),2,2);
[tform,inlierIndex,status]  = estimateGeometricTransform3D(cameraPositions,vehiclePositions,"rigid");


%% 验证eul2rotm函数使用原则
thetaRadians = rand(1,3);
thetaDegrees = rad2deg(thetaRadians);
myR1 = eul2rotm(thetaRadians,'XYZ');
myR2 = rotx(thetaDegrees(1))*roty(thetaDegrees(2))*rotz(thetaDegrees(3));
err = myR1-myR2

%% 模拟车载摄像头朝向
myR = eul2rotm([0,-pi/6,-pi],'XYZ');
cameraPose = rigid3d((myR*roty(90)*rotz(-90))',[0,0,0]);
figure;plotCamera('AbsolutePose',cameraPose,'AxesVisible',true)
axis on;axis equal
grid on
xlabel('x');ylabel('y');zlabel('z')




