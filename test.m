% format long
% eulars = [-0.001180313	-0.1101197	-3.122032;
% -0.001413606	-0.1086281	-3.119092;
% -0.001928661	-0.1062565	-3.116415];
% a = eul2quat(eulars,'XYZ')
% b = quaternion(eulars,'euler','XYZ','point')

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

